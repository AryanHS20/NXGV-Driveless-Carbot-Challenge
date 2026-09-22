"""Plugin registry for the dashboard page.

Each card (or script block) lives in its own file under dashboard_panels/
and is assembled in ORDER below. The concatenation is byte-identical to
the pre-split template; tests/test_dashboard_panels.py pins it against
checked-in goldens, so any visual change must update the goldens
deliberately. Shared CSS stays in shell/head.html on purpose: one visual
language, owned in one place. Route handlers move here in a later phase;
this phase covers structure only.
"""

import importlib
import os
from types import SimpleNamespace

_PANELS_DIR = os.path.dirname(os.path.abspath(__file__))

ORDER = [
    'shell/head.html',
    'left/state/card.html',
    'left/traffic/card.html',
    'left/imu/card.html',
    'left/manual/card.html',
    'left/record/card.html',
    'left/lane/card.html',
    'center/camera/card.html',
    'center/driveviz/card.html',
    'center/lidar/card.html',
    'center/trackmap/card.html',
    'center/v4views/card.html',
    'center/v4status/card.html',
    'right/sensors/card.html',
    'right/odom/card.html',
    'right/controller/card.html',
    'flow/card.html',
    'eventlog/card.html',
    'paramdrawer/card.html',
    'ctrldrawer/card.html',
    'script/core_head.js',
    'script/drawers.js',
    'script/log.js',
    'script/flow.js',
    'script/camera.js',
    'script/driveviz.js',
    'script/trackmap.js',
    'script/sim.js',
    'script/record.js',
    'script/imu.js',
    'script/comp.js',
    'script/navigation.js',
    'script/core_update.js',
    'script/params.js',
    'shell/tail.html',
]

TEACH_PAGE = 'teach/page.html'

_cache = {}


def _read(rel_path: str) -> str:
    """Read a fragment exactly (newline-preserving)."""
    if rel_path not in _cache:
        full = os.path.join(_PANELS_DIR, *rel_path.split('/'))
        with open(full, encoding='utf-8', newline='') as handle:
            _cache[rel_path] = handle.read()
    return _cache[rel_path]


def plugin_names():
    """Short names derived from fragment paths (stable API for later phases)."""
    names = []
    for rel_path in ORDER:
        parts = rel_path.split('/')
        if parts[0] in ('left', 'center', 'right'):
            names.append(parts[1])
        elif parts[0] == 'script':
            names.append('js_' + os.path.splitext(parts[1])[0])
        else:
            names.append(parts[0])
    return names


def build_dashboard_html() -> str:
    """Assemble the main page from fragments in registry order."""
    return ''.join(_read(rel_path) for rel_path in ORDER)


def build_teach_html() -> str:
    """Assemble the teach page (single fragment, kept whole)."""
    return _read(TEACH_PAGE)


# (method, match-kind, pattern, module, function), evaluated in order.
# Mirrors the original handler branch order exactly.
ROUTES = [
    ('GET', 'exact', '/console', 'routes_console', 'serve'),
    ('GET', 'exact', '/console/', 'routes_console', 'serve'),
    ('GET', 'prefix', '/api/console/', 'routes_console', 'get'),
    ('POST', 'prefix', '/api/console/', 'routes_console', 'action'),
    ('GET', 'exact', '/data', 'routes_core', 'serve_data'),
    ('GET', 'exact', '/lidar_data', 'routes_lidar', 'serve_data'),
    ('GET', 'prefix', '/camera_feed', 'routes_camera', 'serve_feed'),
    ('GET', 'prefix', '/api/set_cam_view', 'routes_camera', 'set_view'),
    ('GET', 'prefix', '/api/get_param', 'routes_params', 'get_param'),
    ('GET', 'prefix', '/api/recording_data', 'routes_record', 'recording_data'),
    ('GET', 'exact', '/sim', 'routes_replay', 'serve_index'),
    ('GET', 'exact', '/sim/', 'routes_replay', 'serve_index'),
    ('GET', 'prefix', '/sim/', 'routes_replay', 'serve_file'),
    ('GET', 'prefix', '/api/replay/list', 'routes_replay', 'replay_list'),
    ('GET', 'prefix', '/api/replay/get', 'routes_replay', 'replay_get'),
    ('GET', 'prefix', '/api/v4_telemetry', 'routes_trackmap', 'v4_telemetry'),
    ('GET', 'prefix', '/teach', 'routes_core', 'serve_teach'),
    ('GET', 'catch-all', '', 'routes_core', 'serve_index'),
    ('POST', 'exact', '/api/reset_odom', 'routes_state', 'reset_odom'),
    ('POST', 'exact', '/api/set_param', 'routes_params', 'set_param'),
    ('POST', 'exact', '/api/save_defaults', 'routes_params', 'save_defaults'),
    ('POST', 'exact', '/api/record_playback', 'routes_record', 'record_playback'),
    ('POST', 'exact', '/api/reset_competition', 'routes_state', 'reset_competition'),
    ('POST', 'exact', '/api/calibrate_imu', 'routes_imu', 'calibrate_imu'),
    ('POST', 'catch-all', '', 'routes_core', 'serve_404'),
]

_route_cache = {}


def _resolve(module_name: str, func_name: str):
    """Import a route handler lazily (keeps this module dependency-free)."""
    key = (module_name, func_name)
    if key not in _route_cache:
        module = importlib.import_module('.' + module_name, __package__)
        _route_cache[key] = getattr(module, func_name)
    return _route_cache[key]


def make_context(handler, node, helpers: dict):
    """Build the context passed to every route handler."""
    return SimpleNamespace(h=handler, node=node, **helpers)


def dispatch(ctx, method: str, path: str) -> bool:
    """Route one request to the first matching plugin handler."""
    for route_method, kind, pattern, module_name, func_name in ROUTES:
        if route_method != method:
            continue
        if kind == 'exact' and path != pattern:
            continue
        if kind == 'prefix' and not path.startswith(pattern):
            continue
        return bool(_resolve(module_name, func_name)(ctx, path))
    return False
