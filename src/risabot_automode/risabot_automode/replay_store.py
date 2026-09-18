#!/usr/bin/env python3
"""Replay files and static sim views served by the dashboard.

No ROS, no numpy — pure stdlib so the dashboard HTTP layer and the
offline exporter share these helpers. Replay files live next to the
recorded runs in ~/risabot_maps as <run-stem>.replay.json (see
tools/export_replay.py and tools/REPLAY_FORMAT.md).
"""

import json
import os

REPLAY_VERSION = 1
REPLAY_SUFFIX = '.replay.json'
RUN_SUFFIX = '.jsonl'
MAX_LISTED = 50

_MIME = {
    '.html': 'text/html; charset=utf-8',
    '.css': 'text/css',
    '.js': 'application/javascript',
    '.json': 'application/json',
    '.png': 'image/png',
    '.svg': 'image/svg+xml',
    '.ico': 'image/x-icon',
}


def mime_for(filename: str):
    """MIME type for a static file, or None when not servable."""
    return _MIME.get(os.path.splitext(filename)[1].lower())


def safe_static_path(base_dir: str, rel_path: str):
    """Resolve a /sim/<path> request. Returns an abs path or None.

    Rejects traversal, missing files, directories and unknown suffixes.
    """
    if not rel_path or not base_dir:
        return None
    if rel_path != rel_path.strip():
        return None
    normalized = os.path.normpath(rel_path).replace('\\', '/').lstrip('/')
    if normalized.startswith('..') or os.path.isabs(normalized):
        return None
    if os.path.basename(normalized).rstrip(' .') != os.path.basename(normalized):
        return None
    candidate = os.path.abspath(os.path.join(base_dir, normalized))
    if not candidate.startswith(os.path.abspath(base_dir) + os.sep):
        return None
    if mime_for(candidate) is None:
        return None
    if not os.path.isfile(candidate):
        return None
    return candidate


def replay_dir(home: str = None) -> str:
    """Directory holding runs and replay files."""
    return os.path.join(os.path.expanduser('~' if home is None else home),
                         'risabot_maps')


def safe_replay_path(maps_dir: str, name: str):
    """Resolve a replay ?name= request. Returns an abs path or None."""
    if not name or not maps_dir:
        return None
    if os.path.basename(name) != name or not name.endswith('.json'):
        return None
    candidate = os.path.abspath(os.path.join(maps_dir, name))
    if not candidate.startswith(os.path.abspath(maps_dir) + os.sep):
        return None
    if not os.path.isfile(candidate):
        return None
    return candidate


def list_replays(maps_dir: str) -> dict:
    """List recorded runs and exported replays, newest first."""
    runs, replays = [], []
    try:
        entries = sorted(os.listdir(maps_dir), reverse=True)
    except OSError:
        entries = []
    for entry in entries:
        full = os.path.join(maps_dir, entry)
        if not os.path.isfile(full):
            continue
        if entry.endswith(REPLAY_SUFFIX):
            replays.append(entry)
        elif entry.endswith(RUN_SUFFIX):
            runs.append(entry)
    return {'runs': runs[:MAX_LISTED], 'replays': replays[:MAX_LISTED]}


def summarize_replay(doc: dict) -> dict:
    """Small metadata line for the dashboard card. Never raises."""
    try:
        frames = doc.get('frames', [])
        n = len(frames)
        odom = sum(1 for f in frames if isinstance(f, dict) and f.get('odom'))
        corr = sum(1 for f in frames
                   if isinstance(f, dict) and f.get('corridor'))
        duration = 0.0
        if n > 1:
            try:
                duration = float(frames[-1].get('t', 0.0)) - float(frames[0].get('t', 0.0))
            except (TypeError, ValueError):
                duration = 0.0
        return {
            'version': doc.get('version'),
            'frames': n,
            'duration_s': round(max(0.0, duration), 1),
            'has_odom': odom > 0,
            'corridor_frames': corr,
            'source_run': doc.get('source_run', ''),
        }
    except (AttributeError, TypeError, ValueError):
        return {'version': None, 'frames': 0, 'duration_s': 0.0,
                'has_odom': False, 'corridor_frames': 0, 'source_run': ''}


def load_replay_json(path: str):
    """Read a replay file. Returns (doc, error)."""
    try:
        with open(path, encoding='utf-8') as handle:
            doc = json.load(handle)
    except (OSError, ValueError) as exc:
        return None, str(exc)
    if not isinstance(doc, dict) or doc.get('version') != REPLAY_VERSION:
        return None, 'unsupported replay version'
    return doc, ''
