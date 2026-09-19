"""Simulator views and replay routes (moved verbatim from the dashboard handler)."""
import json
import os

try:
    from ament_index_python.packages import get_package_share_directory
except ImportError:
    get_package_share_directory = None

from ..replay_store import (
    list_replays,
    load_replay_json,
    mime_for,
    replay_dir,
    safe_replay_path,
    safe_static_path,
    summarize_replay,
)

_SIM_DIR = None


def get_sim_dir():
    """Installed sim_views dir (ament share first, source tree fallback)."""
    global _SIM_DIR
    if _SIM_DIR is None:
        candidates = []
        if get_package_share_directory is not None:
            try:
                candidates.append(os.path.join(
                    get_package_share_directory('risabot_automode'), 'sim_views'))
            except Exception:
                pass
        candidates.append(os.path.join(
            os.path.dirname(os.path.abspath(__file__)), '..', '..', 'sim_views'))
        for candidate in candidates:
            if os.path.isdir(candidate):
                _SIM_DIR = candidate
                break
    return _SIM_DIR


def serve_static(ctx, rel):
    """Serve one file from sim_views (path-traversal safe)."""
    handler = ctx.h
    base = get_sim_dir()
    path = safe_static_path(base, rel) if base else None
    if not path:
        handler.send_response(404)
        handler.end_headers()
        return True
    try:
        with open(path, 'rb') as handle:
            blob = handle.read()
    except OSError:
        handler.send_response(404)
        handler.end_headers()
        return True
    handler.send_response(200)
    handler.send_header('Content-Type', mime_for(path))
    if path.endswith('.js') or path.endswith('.css'):
        handler.send_header('Cache-Control', 'max-age=3600')
    else:
        handler.send_header('Cache-Control', 'no-cache')
    handler.send_header('Content-Length', str(len(blob)))
    handler.end_headers()
    handler.wfile.write(blob)
    return True


def serve_index(ctx, path):
    """Serve the simulator landing page."""
    return serve_static(ctx, 'index.html')


def serve_file(ctx, path):
    """Serve one simulator asset below /sim/."""
    from urllib.parse import urlparse
    return serve_static(ctx, urlparse(path).path[len('/sim/'):])


def replay_list(ctx, path):
    """List recorded runs and exported replays, newest first."""
    payload = {'ok': True}
    payload.update(list_replays(replay_dir()))
    handler = ctx.h
    handler.send_response(200)
    handler.send_header('Content-Type', 'application/json')
    handler.send_header('Access-Control-Allow-Origin', '*')
    handler.end_headers()
    handler.wfile.write(json.dumps(payload).encode())
    return True


def replay_get(ctx, path):
    """Serve one replay file (or just its metadata with ?meta=1)."""
    from urllib.parse import urlparse, parse_qs
    qs = parse_qs(urlparse(path).query)
    name = qs.get('name', [''])[0]
    meta_only = qs.get('meta', [''])[0] == '1'
    handler = ctx.h
    route_path = safe_replay_path(replay_dir(), name)
    if not route_path:
        result = {'ok': False, 'error': 'unknown replay'}
    elif meta_only:
        doc, err = load_replay_json(route_path)
        if doc is None:
            result = {'ok': False, 'error': err}
        else:
            result = {'ok': True, 'meta': summarize_replay(doc)}
    else:
        try:
            with open(route_path, 'rb') as handle:
                blob = handle.read()
        except OSError:
            blob = None
        if blob is None:
            result = {'ok': False, 'error': 'unreadable replay'}
        else:
            handler.send_response(200)
            handler.send_header('Content-Type', 'application/json')
            handler.send_header('Cache-Control', 'no-cache')
            handler.send_header('Content-Length', str(len(blob)))
            handler.end_headers()
            handler.wfile.write(blob)
            return True
    handler.send_response(200)
    handler.send_header('Content-Type', 'application/json')
    handler.send_header('Access-Control-Allow-Origin', '*')
    handler.end_headers()
    handler.wfile.write(json.dumps(result).encode())
    return True
