#!/usr/bin/env python3
"""Export a recorded run (JSONL) to a simulator replay file (v1 JSON).

Usage:
    python3 tools/export_replay.py <run.jsonl> [--status-log status.jsonl]
            [--out replay.json] [--max-frames 20000]

The optional status log holds V4 road/status snapshots as one JSON object
per line: {"t_wall": float, "status": {...}}. Snapshots merge into frames
by nearest timestamp within --merge-tol seconds. Without it, corridor
stays empty and the replay still carries odometry, lane and mission flags.
See tools/REPLAY_FORMAT.md for the schema contract.
"""

import argparse
import json
import os
import sys


def read_run_samples(path):
    """Yield sample dicts from a run file, skipping meta and bad lines."""
    samples = []
    with open(path, encoding='utf-8') as handle:
        for line in handle:
            line = line.strip()
            if not line:
                continue
            try:
                obj = json.loads(line)
            except ValueError:
                continue
            if isinstance(obj, dict) and 'meta' in obj:
                continue
            if isinstance(obj, dict):
                samples.append(obj)
    return samples


def read_status_log(path):
    """Yield (t_wall, corridor) from a V4 status sidecar log."""
    entries = []
    with open(path, encoding='utf-8') as handle:
        for line in handle:
            line = line.strip()
            if not line:
                continue
            try:
                obj = json.loads(line)
            except ValueError:
                continue
            if not isinstance(obj, dict):
                continue
            try:
                stamp = float(obj.get('t_wall'))
            except (TypeError, ValueError):
                continue
            status = obj.get('status', {})
            corridor = []
            if isinstance(status, dict):
                raw = status.get('corridor', {}).get('primary', [])
                for point in raw if isinstance(raw, list) else []:
                    try:
                        corridor.append({
                            'forward_m': float(point['forward_m']),
                            'left_m': float(point['left_m']),
                            'width_m': float(point['width_m']),
                        })
                    except (KeyError, TypeError, ValueError):
                        continue
            entries.append((stamp, corridor))
    entries.sort(key=lambda item: item[0])
    return entries


def _odom_of(sample):
    try:
        x, y, yaw = float(sample['ox']), float(sample['oy']), float(sample['oyaw'])
    except (KeyError, TypeError, ValueError):
        return None
    import math
    if not all(math.isfinite(v) for v in (x, y, yaw)):
        return None
    return {'x': x, 'y': y, 'yaw': yaw}


def export_run(run_path, status_path=None, out_path=None, max_frames=20000,
               merge_tol=0.5):
    """Build a v1 replay doc; write it unless out_path is None.

    Returns (doc, out_path or None).
    """
    samples = read_run_samples(run_path)
    status = read_status_log(status_path) if status_path else []
    stride = max(1, (len(samples) + max(1, max_frames) - 1) // max(1, max_frames))
    frames = []
    cursor = 0
    for sample in samples[::stride]:
        try:
            stamp = float(sample.get('t_wall', 0.0))
        except (TypeError, ValueError):
            stamp = 0.0
        corridor = []
        if status:
            # Monotone nearest-seek: frames are chronological and status is
            # sorted, so the nearest index never moves backwards. Seek purely
            # by closeness first; only then apply the tolerance gate. (Gating
            # the seek itself strands the cursor behind large gaps.)
            while (cursor + 1 < len(status)
                    and abs(status[cursor + 1][0] - stamp)
                    < abs(status[cursor][0] - stamp)):
                cursor += 1
            if abs(status[cursor][0] - stamp) <= merge_tol:
                corridor = status[cursor][1]
        frames.append({
            't': stamp,
            'odom': _odom_of(sample),
            'speed': sample.get('speed'),
            'lane_error': sample.get('lane_error'),
            'curvature': sample.get('curvature'),
            'lane_lost': sample.get('lane_lost'),
            'tl': sample.get('tl'),
            'hill': sample.get('hill'),
            'tunnel': sample.get('tunnel'),
            'corridor': corridor,
        })
    doc = {
        'version': 1,
        'source_run': os.path.basename(run_path),
        'frames': frames,
    }
    if out_path is None:
        stem = os.path.basename(run_path)
        if stem.endswith('.jsonl'):
            stem = stem[:-len('.jsonl')]
        out_path = os.path.join(os.path.dirname(os.path.abspath(run_path)),
                                stem + '.replay.json')
    with open(out_path, 'w', encoding='utf-8') as handle:
        json.dump(doc, handle)
        handle.write('\n')
    return doc, out_path


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', help='recorded run_*.jsonl file')
    parser.add_argument('--status-log', default=None,
                        help='V4 status sidecar (JSON lines)')
    parser.add_argument('--out', default=None, help='output replay path')
    parser.add_argument('--max-frames', type=int, default=20000)
    parser.add_argument('--merge-tol', type=float, default=0.5)
    args = parser.parse_args(argv)
    doc, out_path = export_run(args.run, args.status_log, args.out,
                               args.max_frames, args.merge_tol)
    print(f'wrote {out_path} ({len(doc["frames"])} frames)')
    return 0


if __name__ == '__main__':
    sys.exit(main())
