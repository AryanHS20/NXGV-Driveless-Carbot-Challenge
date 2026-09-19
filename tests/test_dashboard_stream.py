"""Exercise the dashboard route plugins without ROS or a robot connection."""
import io
import json
import os
import tempfile
import threading
import time
import types
import unittest

import ros_stub

ros_stub.install()

from risabot_automode.dashboard_panels import registry
from risabot_automode.dashboard_panels import routes_camera


class ShortCondition(threading.Condition):
    def __init__(self):
        super().__init__()
        self.waits = 0

    def wait_for(self, predicate, timeout=None):
        self.waits += 1
        return super().wait_for(predicate, timeout=0.01)


class FakeHandler:
    def __init__(self, path, body=b''):
        self.path = path
        self.wfile = io.BytesIO()
        self.headers = {'Content-Length': str(len(body))}
        self.rfile = io.BytesIO(body)
        self.status = None

    def send_response(self, code):
        self.status = code

    def send_header(self, *args):
        pass

    def end_headers(self):
        pass


def make_node(**overrides):
    node = types.SimpleNamespace(
        camera_clients_lock=threading.Lock(), num_camera_clients=0,
        jpeg_condition=ShortCondition(), frame_id=42, latest_jpeg=None,
        lidar_lock=threading.Lock(), lidar_points=[],
        data_lock=threading.Lock(), data={},
        tunnel_debug_lock=threading.Lock(), tunnel_debug='',
        active_camera_view='raw',
        active_camera_source='forward',
    )
    for key, value in overrides.items():
        setattr(node, key, value)
    return node


def make_ctx(handler, node=None, **overrides):
    helpers = {
        'get_param': lambda n, p: (overrides.get('param_value', 1.0), None),
        'set_param': lambda n, p, v: (True, 'ok'),
        'save_defaults': lambda: {'ok': True},
        'param_defaults': {},
        'dashboard_html': '<html>dash</html>',
        'teach_html': '<html>teach</html>',
    }
    helpers.update({k: v for k, v in overrides.items() if k in helpers})
    return registry.make_context(handler, node, helpers)


class CameraStreamTests(unittest.TestCase):
    def test_empty_view_waits_then_releases_client(self):
        handler = FakeHandler('/camera_feed')
        node = make_node()
        ctx = make_ctx(handler, node)
        routes_camera.serve_feed(ctx, handler.path)
        self.assertEqual(node.jpeg_condition.waits, 1)
        self.assertEqual(node.num_camera_clients, 0)
        self.assertEqual(handler.wfile.getvalue(), b'')

    def test_frame_is_sent_once_then_idle_connection_expires(self):
        handler = FakeHandler('/camera_feed')
        node = make_node(latest_jpeg=b'jpeg')
        ctx = make_ctx(handler, node)
        routes_camera.serve_feed(ctx, handler.path)
        self.assertEqual(handler.wfile.getvalue().count(b'--frame'), 1)
        self.assertIn(b'Content-Length: 4', handler.wfile.getvalue())
        self.assertEqual(node.jpeg_condition.waits, 2)
        self.assertEqual(node.num_camera_clients, 0)

    def test_set_view_switches_and_toggles_debug(self):
        handler = FakeHandler('/api/set_cam_view?view=signage')
        node = make_node()
        calls = []
        ctx = make_ctx(handler, node,
                       set_param=lambda n, p, v: calls.append((n, p, v)) or (True, 'ok'))
        routes_camera.set_view(ctx, handler.path)
        self.assertEqual(node.active_camera_view, 'signage')
        deadline = time.monotonic() + 2.0
        while len(calls) < 3 and time.monotonic() < deadline:
            time.sleep(0.02)
        by_node = {name: value for name, param, value in calls}
        self.assertEqual(by_node.get('signage_detector'), 'true')
        self.assertEqual(by_node.get('line_follower_camera'), 'false')
        self.assertEqual(handler.status, 200)


class DispatchTests(unittest.TestCase):
    def test_routes_table_resolves(self):
        seen = set()
        for method, kind, pattern, module_name, func_name in registry.ROUTES:
            self.assertNotIn((method, kind, pattern), seen)
            seen.add((method, kind, pattern))
            self.assertTrue(registry._resolve(module_name, func_name))

    def test_unknown_get_serves_dashboard(self):
        handler = FakeHandler('/nope')
        ctx = make_ctx(handler)
        self.assertTrue(registry.dispatch(ctx, 'GET', handler.path))
        self.assertEqual(handler.status, 200)
        self.assertEqual(handler.wfile.getvalue(), b'<html>dash</html>')

    def test_unknown_post_is_404(self):
        handler = FakeHandler('/nope')
        ctx = make_ctx(handler)
        self.assertTrue(registry.dispatch(ctx, 'POST', handler.path))
        self.assertEqual(handler.status, 404)

    def test_data_and_teach(self):
        node = types.SimpleNamespace(get_json=lambda: '{"a":1}')
        handler = FakeHandler('/data')
        registry.dispatch(make_ctx(handler, node), 'GET', handler.path)
        self.assertEqual(json.loads(handler.wfile.getvalue()), {'a': 1})
        handler = FakeHandler('/teach')
        registry.dispatch(make_ctx(handler), 'GET', handler.path)
        self.assertEqual(handler.wfile.getvalue(), b'<html>teach</html>')

    def test_lidar_with_tunnel_debug(self):
        handler = FakeHandler('/lidar_data')
        node = make_node(lidar_points=[(1.0, 2.0)],
                         data={'tunnel_detected': True},
                         tunnel_debug=json.dumps({'l': 1, 'r': 2, 'lat': 0.1,
                                                  'w': 0.2, 'cl': [3]}))
        registry.dispatch(make_ctx(handler, node), 'GET', handler.path)
        payload = json.loads(handler.wfile.getvalue())
        self.assertEqual(payload['points'], [[1.0, 2.0]])
        self.assertEqual(payload['left_dist'], 1)
        self.assertEqual(payload['centerline'], [3])

    def test_param_round_trip(self):
        handler = FakeHandler('/api/get_param?node=n&param=p')
        ctx = make_ctx(handler, None, param_defaults={'n': {'p': 5}})
        registry.dispatch(ctx, 'GET', handler.path)
        payload = json.loads(handler.wfile.getvalue())
        self.assertTrue(payload['ok'])
        self.assertEqual(payload['default'], 5)
        handler = FakeHandler('/api/set_param',
                              json.dumps({'node': 'n', 'param': 'p',
                                          'value': '2'}).encode())
        registry.dispatch(make_ctx(handler), 'POST', handler.path)
        self.assertTrue(json.loads(handler.wfile.getvalue())['ok'])

    def test_state_routes(self):
        node = types.SimpleNamespace(
            data_lock=threading.Lock(),
            data={'distance': 9.0, 'odom_x': 1.0, 'odom_y': 2.0,
                  'odom_yaw': 3.0, 'speed': 4.0})
        handler = FakeHandler('/api/reset_odom', b'{}')
        registry.dispatch(make_ctx(handler, node), 'POST', handler.path)
        self.assertEqual(node.data['distance'], 0.0)
        self.assertTrue(json.loads(handler.wfile.getvalue())['ok'])

    def test_recording_data_missing(self):
        handler = FakeHandler('/api/recording_data')
        registry.dispatch(make_ctx(handler), 'GET', handler.path)
        payload = json.loads(handler.wfile.getvalue())
        self.assertFalse(payload['ok'])


if __name__ == '__main__':
    unittest.main()
