"""HTTP behavior and mockup control coverage, all transports inert."""
from email.message import Message
from io import BytesIO
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/risabot_automode'))
from risabot_automode.dashboard_panels import routes_console as api
from risabot_automode.dashboard_panels.console_contract import catalog
from risabot_automode.dashboard_panels.console_inventory import INVENTORY
from risabot_automode.dashboard_panels.console_planning import PlanningSession
from risabot_automode.dashboard_panels import registry


class Handler:
    def __init__(self, body=b'{}', token=True):
        self.headers = Message()
        self.headers['Content-Type'] = 'application/json'
        self.headers['Content-Length'] = str(len(body))
        if token:
            self.headers['X-Console-Token'] = api._TOKEN
        self.rfile, self.wfile = BytesIO(body), BytesIO()
        self.connection = SimpleNamespace(settimeout=lambda value: None)
        self.status = None

    def send_response(self, status):
        self.status = status

    def send_header(self, *args):
        pass

    def end_headers(self):
        pass

    def document(self):
        return json.loads(self.wfile.getvalue())


@pytest.fixture
def state(tmp_path, monkeypatch):
    planning = PlanningSession(tmp_path / 'drafts')
    monkeypatch.setattr(api, '_SESSION', planning)
    yield planning
    planning.close()


def request(body, token=True):
    handler = Handler(json.dumps(body).encode(), token)
    api.action(SimpleNamespace(h=handler, node=None), '/api/console/action')
    return handler


def test_catalog_preserves_every_section_and_button_with_explicit_contract():
    original = json.loads((Path(api.__file__).parent / 'console_control_inventory.json').read_text(encoding='utf-8'))
    assert INVENTORY == original
    current = catalog()
    assert len(current['screens']) == 24
    assert sum(len(s['buttons']) for s in current['screens']) == 138
    assert len(current['shellButtons']) == 28
    for before, after in zip(original['screens'], current['screens']):
        assert before['id'] == after['id'] and len(before['fields']) == len(after['fields'])
        assert [b['label'] for b in before['buttons']] == [b['mock_label'] for b in after['buttons']]
        for b in after['buttons']:
            assert (b['disabled'] and b['reason']) or (b['action'] and not b['disabled'])


def test_preview_cannot_claim_live_robot_or_actuator_authority():
    handler = Handler()
    api.get(SimpleNamespace(h=handler, node=None), '/api/console/status')
    doc = handler.document()
    assert doc['data'] is None and not doc['connected'] and not doc['motion_control']


def test_new_route_does_not_replace_existing_dashboard():
    assert next(r for r in registry.ROUTES if r[0] == 'GET' and r[1] == 'catch-all')[4] == 'serve_index'
    assert any(r[2] == '/console' and r[3] == 'routes_console' for r in registry.ROUTES)


def test_action_requires_current_page_token_before_accessing_session(state):
    response = request(dict(action='discard', revision=0), token=False)
    assert response.status == 403 and state.revision == 0


def test_stale_browser_revision_cannot_modify_new_draft(state):
    assert request(dict(action='discard', revision=0)).status == 200
    stale = request(dict(action='discard', revision=0))
    assert stale.status == 409 and state.revision == 1
    assert 'Refresh' in stale.document()['error']


@pytest.mark.parametrize('body', [b'null', b'[]', b'{', b'{"x": NaN}', b'{"x": Infinity}'])
def test_malformed_payload_does_not_crash_http_handler(state, body):
    handler = Handler(body)
    api.action(SimpleNamespace(h=handler, node=None), '/api/console/action')
    assert handler.status == 400 and not handler.document()['ok']
    assert state.revision == 0


def test_payload_size_limit_is_checked_before_reading(state):
    handler = Handler()
    handler.headers.replace_header('Content-Length', str(api.MAX_BODY + 1))
    api.action(SimpleNamespace(h=handler, node=None), '/api/console/action')
    assert handler.status == 400 and handler.rfile.tell() == 0


@pytest.mark.parametrize('action', ['launch', 'start', 'estop', 'manual', 'shell', 'save_mission'])
def test_missing_runtime_and_unsatisfied_planning_actions_never_report_success(state, action):
    result = request(dict(action=action, revision=0))
    assert result.status == 400 and not result.document()['ok']
    assert not state.directory.exists()


def test_missing_dependency_returns_recoverable_503(monkeypatch):
    def unavailable():
        raise ImportError('numpy unavailable: install the planning dependencies')
    monkeypatch.setattr(api, 'session', unavailable)
    handler = Handler()
    api.get(SimpleNamespace(h=handler, node=None), '/api/console/planning')
    assert handler.status == 503 and 'numpy' in handler.document()['error']
    healthy = Handler()
    api.get(SimpleNamespace(h=healthy, node=None), '/api/console/status')
    assert healthy.status == 200


def test_unknown_console_route_is_json_404_not_dashboard_html():
    handler = Handler()
    api.get(SimpleNamespace(h=handler, node=None), '/api/console/unknown')
    assert handler.status == 404 and not handler.document()['ok']


def test_saved_map_requires_all_sections_not_just_a_subset(state):
    state.report = {'loop': {'ok': True}}
    response = request(dict(action='save_map', revision=0))
    assert response.status == 400 and not state.directory.exists()
