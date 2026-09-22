"""Opt-in console preview and draft-planning API; no robot command endpoints."""
import atexit
import json
import logging
import os
from pathlib import Path
import secrets
import threading
import time
from urllib.parse import urlsplit

from .console_contract import catalog

_LOCK = threading.Lock()
_SESSION = None
_TOKEN = secrets.token_urlsafe(32)
_LAST_ERROR = {}
_LOGGER = logging.getLogger(__name__)
MAX_BODY = 2_000_000


def session():
    global _SESSION
    with _LOCK:
        if _SESSION is None:
            from .console_planning import PlanningSession
            # An explicitly separate draft root. Never overwrite central config.
            root = Path(os.environ.get('XDG_STATE_HOME', str(Path.home() / '.local' / 'state')))
            _SESSION = PlanningSession(root / 'risabot' / 'dashboard-drafts')
            atexit.register(_SESSION.close)
        return _SESSION


def response(handler, status, document):
    body = json.dumps(document, allow_nan=False).encode('utf-8')
    handler.send_response(status)
    handler.send_header('Content-Type', 'application/json; charset=utf-8')
    handler.send_header('Cache-Control', 'no-store')
    handler.send_header('X-Content-Type-Options', 'nosniff')
    handler.send_header('Content-Length', str(len(body)))
    handler.end_headers()
    handler.wfile.write(body)
    return True


def serve(ctx, path):
    from .registry import _read
    document = _read('shell/console.html').replace('/* CONSOLE_SCRIPT */', _read('script/console.js'))
    encoded = document.encode('utf-8')
    ctx.h.send_response(200)
    ctx.h.send_header('Content-Type', 'text/html; charset=utf-8')
    ctx.h.send_header('Cache-Control', 'no-store')
    ctx.h.send_header('Content-Length', str(len(encoded)))
    ctx.h.end_headers()
    ctx.h.wfile.write(encoded)
    return True


def get(ctx, path):
    name = urlsplit(path).path.rsplit('/', 1)[-1]
    try:
        if name == 'catalog':
            return response(ctx.h, 200, dict(ok=True, token=_TOKEN, catalog=catalog()))
        if name == 'status':
            data = json.loads(ctx.node.get_json()) if ctx.node else None
            return response(ctx.h, 200, dict(ok=True, connected=data is not None, data=data,
                                           preview=True, motion_control=False))
        if name == 'planning':
            return response(ctx.h, 200, dict(ok=True, planning=session().snapshot()))
        return response(ctx.h, 404, dict(ok=False, error='Unknown console endpoint'))
    except Exception as exc:
        return failure(ctx.h, exc)


def failure(handler, exc):
    from .console_jobs import JobConflict
    code = 409 if isinstance(exc, JobConflict) else 400 if isinstance(exc, ValueError) else 503
    if code == 503:
        key = type(exc).__name__
        now = time.monotonic()
        with _LOCK:
            last = _LAST_ERROR.get(key, -float('inf'))
            _LAST_ERROR[key] = now if now - last >= 10 else last
        if now - last >= 10:
            _LOGGER.exception('Console request failed; dashboard remains available')
    return response(handler, code, dict(ok=False, error=str(exc),
                                       recovery='Stop a running computation before retrying. Check dependencies and logs if the error persists.'))


def action(ctx, path):
    h = ctx.h
    try:
        if urlsplit(path).path != '/api/console/action':
            return response(h, 404, dict(ok=False, error='Unknown console endpoint'))
        if h.headers.get('X-Console-Token') != _TOKEN:
            return response(h, 403, dict(ok=False, error='Reload the console before sending actions'))
        if h.headers.get_content_type() != 'application/json' or h.headers.get('Transfer-Encoding'):
            raise ValueError('Send a bounded JSON body')
        size = int(h.headers.get('Content-Length', '0'))
        if not 0 < size <= MAX_BODY:
            raise ValueError('Invalid or excessive request size')
        h.connection.settimeout(5)
        body = h.rfile.read(size)
        if len(body) != size:
            raise ValueError('Incomplete action body')
        req = json.loads(body, parse_constant=lambda value: (_ for _ in ()).throw(ValueError('Non-finite number')))
        if not isinstance(req, dict):
            raise ValueError('Action body must be an object')
        operation = req.get('action')
        state = session()
        from .console_jobs import JobConflict
        with state.lock:
            state._collect()
            if operation == 'cancel':
                job = state.jobs.snapshot()
                if not job or req.get('job_id') != job['id']:
                    raise JobConflict('The running job changed; refresh before stopping it')
                state.jobs.cancel()
            else:
                if type(req.get('revision')) is not int or req['revision'] != state.revision:
                    raise JobConflict('The draft changed in another request. Refresh and review before retrying.')
                result = None
                if operation == 'import_lap':
                    state.set_lap(req.get('points'), req.get('frame'))
                elif operation == 'discard':
                    state.discard_lap()
                elif operation in ('fit', 'refit'):
                    state.fit(operation == 'refit')
                elif operation in ('move_handle', 'reset_handle'):
                    state.move_handle(req.get('handle'), req.get('point'), operation == 'reset_handle')
                elif operation == 'save_map':
                    result = state.save_map()
                elif operation == 'pose':
                    state.edit_pose(req.get('index'), req.get('edit'), req.get('point'),
                                    req.get('degrees'), req.get('snap', False))
                elif operation == 'plan':
                    state.plan()
                elif operation == 'save_mission':
                    result = state.save_mission()
                else:
                    raise ValueError('This action is not connected to a runtime adapter')
                return response(h, 200, dict(ok=True, result=result, planning=state.snapshot()))
            return response(h, 200, dict(ok=True, planning=state.snapshot()))
    except Exception as exc:
        return failure(h, exc)
