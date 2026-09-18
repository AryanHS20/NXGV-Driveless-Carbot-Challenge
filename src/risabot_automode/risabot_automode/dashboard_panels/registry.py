"""Plugin registry for the dashboard page.

Each card (or script block) lives in its own file under dashboard_panels/
and is assembled in ORDER below. The concatenation is byte-identical to
the pre-split template; tests/test_dashboard_panels.py pins it against
checked-in goldens, so any visual change must update the goldens
deliberately. Shared CSS stays in shell/head.html on purpose: one visual
language, owned in one place. Route handlers move here in a later phase;
this phase covers structure only.
"""

import os

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
    'center/lidar/card.html',
    'center/v4views/card.html',
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
    'script/sim.js',
    'script/record.js',
    'script/imu.js',
    'script/comp.js',
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
