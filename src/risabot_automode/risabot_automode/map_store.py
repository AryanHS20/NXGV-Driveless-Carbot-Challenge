#!/usr/bin/env python3
"""Persistent run mapping: record every run, reuse the map on later runs.

Phase 1 (this module + map_recorder node): RECORD + PERSIST + RELOAD.
Nothing in autonomy consumes the map yet — consumers arrive after real
track data exists. Recording is append-only and can never break driving.

On-disk layout under <map_dir> (default ``~/risabot_maps``):
  run_<UTC>.jsonl   one JSON object per line: {"meta": {...}} first, then samples
  map_best.json     derived map: odom trail, averaged landmarks, solved anchors

Sample schema (all perception fields optional/None-tolerant):
  {"t_wall": float, "t_mono": float,
   "ox": float|None, "oy": float|None, "oyaw": float|None, "speed": float|None,
   "lane_error": float|None, "curvature": float|None, "lane_lost": bool|None,
   "tl": str|None, "hill": bool|None, "tunnel": bool|None,
   "uwb": {"x": float|None, "y": float|None, "valid": bool,
           "anchors": [{"id": str, "range_m": float, "age_ms": float}],
           "age": float}|None}

UWB interface contract (for the teammate positioning tool): publish the above
``uwb`` object, JSON-encoded, as std_msgs/String on /uwb_fix at >= 5 Hz.
Position (x, y) is in track metres; anchors carry string ids matching the
zone map (e.g. "A1", "A2", "A3"). A missing/invalid fix must still be
published regularly with "valid": false — silence is not a signal.
"""

import json
import math
import os
import time
from typing import Dict, List, Optional, Tuple

import numpy as np

MAP_BEST_FILENAME = 'map_best.json'
MAP_VERSION = 1


def parse_uwb_fix(raw: str):
    """Validate a /uwb_fix payload. Returns a normalized dict or None."""
    try:
        d = json.loads(raw)
    except (json.JSONDecodeError, TypeError, ValueError):
        return None
    if not isinstance(d, dict):
        return None
    anchors = d.get('anchors', [])
    if not isinstance(anchors, list):
        return None
    clean = []
    for a in anchors:
        if not isinstance(a, dict):
            continue
        try:
            rid = str(a['id'])
            rng = float(a['range_m'])
            age = float(a.get('age_ms', 0.0))
        except (KeyError, TypeError, ValueError):
            continue
        if not math.isfinite(rng) or rng < 0 or not math.isfinite(age) or age < 0:
            continue
        clean.append({'id': rid, 'range_m': rng, 'age_ms': age})

    def _opt_float(v):
        try:
            f = float(v)
        except (TypeError, ValueError):
            return None
        return f if math.isfinite(f) else None

    out = {
        'x': _opt_float(d.get('x')),
        'y': _opt_float(d.get('y')),
        'valid': bool(d.get('valid', False)) and _opt_float(d.get('x')) is not None
        and _opt_float(d.get('y')) is not None,
        'anchors': clean,
    }
    try:
        out['t'] = float(d.get('t', time.time()))
    except (TypeError, ValueError):
        out['t'] = time.time()
    return out


def _solve_from(pts, ax: float, ay: float, iters: int, tol: float):
    """Gauss-Newton from one start. Returns (x, y, total squared residual)."""
    for _ in range(max(1, iters)):
        jtj = np.zeros((2, 2))
        jtr = np.zeros(2)
        for x, y, r in pts:
            dx, dy = ax - x, ay - y
            dist = math.hypot(dx, dy) + 1e-9
            res = dist - r
            gx, gy = dx / dist, dy / dist
            jtj[0, 0] += gx * gx
            jtj[0, 1] += gx * gy
            jtj[1, 1] += gy * gy
            jtr[0] += gx * res
            jtr[1] += gy * res
        jtj[1, 0] = jtj[0, 1]
        try:
            step = np.linalg.solve(jtj + np.eye(2) * 1e-9, jtr)
        except np.linalg.LinAlgError:
            break
        ax -= float(step[0])
        ay -= float(step[1])
        if abs(step[0]) + abs(step[1]) < tol:
            break
    cost = sum((math.hypot(ax - x, ay - y) - r) ** 2 for x, y, r in pts)
    return ax, ay, cost


def solve_anchors(samples, iters: int = 50, tol: float = 1e-4) -> Dict[str, Tuple[float, float]]:
    """Solve UWB anchor positions from robot poses + ranges (Gauss-Newton).

    ``samples``: iterable of (rx, ry, [(anchor_id, range_m), ...]).
    Anchors observed from fewer than 3 distinct robot poses are skipped.
    Multi-start (centroid plus four offsets) keeps the lowest-residual fix,
    but a near-straight robot path leaves a mirror ambiguity no range-only
    solver can resolve — real tracks loop, so prefer runs with turns.
    Returns {anchor_id: (x, y)}. Pure function — no ROS.
    """
    from collections import defaultdict
    obs: Dict[str, List[Tuple[float, float, float]]] = defaultdict(list)
    for rx, ry, ranges in samples:
        try:
            px, py = float(rx), float(ry)
        except (TypeError, ValueError):
            continue
        if not (math.isfinite(px) and math.isfinite(py)):
            continue
        for aid, rng in ranges:
            try:
                r = float(rng)
            except (TypeError, ValueError):
                continue
            if math.isfinite(r) and r > 0:
                obs[str(aid)].append((px, py, r))

    solved: Dict[str, Tuple[float, float]] = {}
    for aid, pts in obs.items():
        # Need geometric diversity: 3+ distinct robot poses.
        uniq = {(round(x, 2), round(y, 2)) for x, y, _ in pts}
        if len(uniq) < 3:
            continue
        # Multi-start: centroid plus four offsets at mean-range distance.
        cx = sum(x for x, _, _ in pts) / len(pts)
        cy = sum(y for _, y, _ in pts) / len(pts)
        spread = sum(r for _, _, r in pts) / len(pts)
        best = None
        for sx, sy in ((cx, cy), (cx + spread, cy), (cx - spread, cy),
                       (cx, cy + spread), (cx, cy - spread)):
            cand = _solve_from(pts, sx, sy, iters, tol)
            if best is None or cand[2] < best[2]:
                best = cand
        solved[aid] = (float(best[0]), float(best[1]))
    return solved


class MapStore:
    """Append-only per-run recording + best-map persistence."""

    def __init__(self, map_dir: str):
        self.map_dir = os.path.abspath(os.path.expanduser(map_dir))
        os.makedirs(self.map_dir, exist_ok=True)
        self._fh = None
        self._run_path: Optional[str] = None
        self._since_flush = 0

    def begin_run(self, meta: Optional[dict] = None) -> str:
        """Open a new run file. Returns its path."""
        self.close()
        stamp = time.strftime('%Y%m%d_%H%M%S', time.gmtime())
        self._run_path = os.path.join(self.map_dir, f'run_{stamp}.jsonl')
        self._fh = open(self._run_path, 'a', encoding='utf-8')
        self._fh.write(json.dumps({'meta': meta or {}, 't_wall': time.time()}) + '\n')
        self._since_flush = 0
        return self._run_path

    def record(self, sample: dict) -> None:
        """Append one sample (see module docstring for schema)."""
        if self._fh is None:
            self.begin_run()
        self._fh.write(json.dumps(sample, default=float) + '\n')
        self._since_flush += 1
        if self._since_flush >= 50:
            self._fh.flush()
            self._since_flush = 0

    def close(self) -> None:
        """Flush and close the current run file, if any."""
        try:
            if self._fh is not None:
                self._fh.flush()
                self._fh.close()
        except (OSError, ValueError):
            pass
        finally:
            self._fh = None

    @staticmethod
    def load_run(path: str) -> List[dict]:
        """Read a run file back. Never raises on corrupt lines (skips them)."""
        out = []
        try:
            with open(path, encoding='utf-8') as fh:
                for line in fh:
                    line = line.strip()
                    if not line:
                        continue
                    try:
                        out.append(json.loads(line))
                    except (json.JSONDecodeError, ValueError):
                        continue
        except OSError:
            pass
        return out

    def best_path(self) -> str:
        """Absolute path of the derived best map."""
        return os.path.join(self.map_dir, MAP_BEST_FILENAME)

    def save_best(self, map_dict: dict) -> str:
        """Write map_best.json (keeps one .prev backup). Returns its path."""
        path = self.best_path()
        try:
            if os.path.exists(path):
                os.replace(path, path + '.prev')
        except OSError:
            pass
        with open(path, 'w', encoding='utf-8') as fh:
            json.dump(map_dict, fh)
            fh.write('\n')
        return path

    @staticmethod
    def load_best(map_dir: str) -> dict:
        """Load map_best.json. Returns {} when missing/corrupt — never raises."""
        path = os.path.join(os.path.abspath(os.path.expanduser(map_dir)),
                            MAP_BEST_FILENAME)
        try:
            with open(path, encoding='utf-8') as fh:
                d = json.load(fh)
        except (OSError, ValueError):
            return {}
        return d if isinstance(d, dict) else {}

    @staticmethod
    def build_map_skeleton(samples: List[dict]) -> dict:
        """Derive a map skeleton from recorded samples.

        - track: odom trail decimated to >= 5 cm spacing [(x, y, yaw)]
        - landmarks: observations grouped by (type, 0.3 m cell), averaged
        - anchors: solved from (odom pose, UWB ranges) via solve_anchors
        Pure function — no ROS, no disk.
        """
        track = []
        last = None
        for s in samples:
            if not isinstance(s, dict):
                continue
            x, y, w = s.get('ox'), s.get('oy'), s.get('oyaw')
            try:
                x, y = float(x), float(y)
            except (TypeError, ValueError):
                continue
            if not (math.isfinite(x) and math.isfinite(y)):
                continue
            try:
                yaw = float(w) if w is not None else 0.0
            except (TypeError, ValueError):
                yaw = 0.0
            if last is None or math.hypot(x - last[0], y - last[1]) >= 0.05:
                track.append([x, y, yaw if math.isfinite(yaw) else 0.0])
                last = track[-1]

        land_cells: Dict[tuple, List[list]] = {}
        for s in samples:
            if not isinstance(s, dict):
                continue
            ox, oy = s.get('ox'), s.get('oy')
            for lm in s.get('lm') or []:
                try:
                    lx = float(ox) + float(lm['dx'])
                    ly = float(oy) + float(lm['dy'])
                    key = (str(lm['type']), round(lx / 0.3), round(ly / 0.3))
                except (KeyError, TypeError, ValueError):
                    continue
                if math.isfinite(lx) and math.isfinite(ly):
                    land_cells.setdefault(key, []).append([lx, ly])
        landmarks = {}
        for (typ, _cx, _cy), pts in land_cells.items():
            n = len(pts)
            landmarks.setdefault(typ, []).append({
                'x': sum(p[0] for p in pts) / n,
                'y': sum(p[1] for p in pts) / n,
                'n': n,
            })

        anchor_obs = []
        for s in samples:
            if not isinstance(s, dict):
                continue
            uwb = s.get('uwb') or {}
            ranges = [(a['id'], a['range_m']) for a in uwb.get('anchors', [])
                      if isinstance(a, dict) and 'id' in a and 'range_m' in a]
            if ranges:
                anchor_obs.append((s.get('ox'), s.get('oy'), ranges))
        anchors = {k: list(v) for k, v in solve_anchors(anchor_obs).items()}

        return {'version': MAP_VERSION, 'track': track,
                'landmarks': landmarks, 'anchors': anchors}
