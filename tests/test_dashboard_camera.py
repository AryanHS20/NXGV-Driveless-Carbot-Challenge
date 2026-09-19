"""Unit tests for dashboard camera source/view switching (no robot)."""
import threading
import time
import types
import unittest

import numpy as np

import ros_stub

ros_stub.install()

from risabot_automode.dashboard import DashboardNode
from risabot_automode.dashboard_panels import routes_camera


class FakeBridge:
    def imgmsg_to_cv2(self, msg, desired_encoding='bgr8'):
        return np.zeros((480, 640, 3), dtype=np.uint8)


def make_dashboard(view='raw', source='forward'):
    node = DashboardNode.__new__(DashboardNode)
    node.active_camera_view = view
    node.active_camera_source = source
    node.bridge = FakeBridge()
    node.camera_clients_lock = threading.Lock()
    node.num_camera_clients = 1
    node.jpeg_condition = threading.Condition()
    node.latest_jpeg = None
    node.frame_id = 0
    node._last_encode_mono = 0.0
    node._encode_min_interval = 0.0
    return node


class ImageRoutingTests(unittest.TestCase):
    def encodes(self, view, source, incoming):
        node = make_dashboard(view, source)
        node._image_cb(object(), incoming)
        return node.latest_jpeg is not None

    def test_forward_raw_accepts_raw_only(self):
        self.assertTrue(self.encodes('raw', 'forward', 'raw'))
        self.assertFalse(self.encodes('raw', 'forward', 'line_follower'))
        self.assertFalse(self.encodes('raw', 'forward', 'second'))

    def test_forward_debug_views_routed(self):
        self.assertTrue(self.encodes('line_follower', 'forward', 'line_follower'))
        self.assertTrue(self.encodes('traffic_light', 'forward', 'signage'))
        self.assertFalse(self.encodes('signage', 'forward', 'traffic_light'))

    def test_side_sources_are_raw_only(self):
        self.assertTrue(self.encodes('raw', 'second', 'second'))
        self.assertTrue(self.encodes('raw', 'third', 'third'))
        self.assertFalse(self.encodes('line_follower', 'forward', 'second'))
        self.assertFalse(self.encodes('raw', 'second', 'third'))
        self.assertFalse(self.encodes('raw', 'second', 'raw'))

    def test_no_clients_skips_encode(self):
        node = make_dashboard('raw', 'forward')
        node.num_camera_clients = 0
        node._image_cb(object(), 'raw')
        self.assertIsNone(node.latest_jpeg)


class FakeHandler:
    def __init__(self, path):
        self.path = path
        self.wfile = __import__('io').BytesIO()
        self.status = None

    def send_response(self, code):
        self.status = code

    def send_header(self, *args):
        pass

    def end_headers(self):
        pass


def make_ctx(handler, node, calls):
    return types.SimpleNamespace(
        h=handler, node=node,
        set_param=lambda n, p, v: calls.append((n, p, v)) or (True, 'ok'))


class SetViewRouteTests(unittest.TestCase):
    def run_route(self, path):
        handler = FakeHandler(path)
        node = make_dashboard()
        calls = []
        routes_camera.set_view(make_ctx(handler, node, calls), path)
        deadline = time.monotonic() + 2.0
        while len(calls) < 3 and time.monotonic() < deadline:
            time.sleep(0.02)
        return handler, node, calls

    def test_view_only_keeps_source(self):
        handler, node, _ = self.run_route('/api/set_cam_view?view=signage')
        self.assertEqual(node.active_camera_view, 'signage')
        self.assertEqual(node.active_camera_source, 'forward')
        self.assertEqual(handler.status, 200)

    def test_source_switch_disables_all_debug(self):
        _, node, calls = self.run_route('/api/set_cam_view?view=raw&source=second')
        self.assertEqual(node.active_camera_source, 'second')
        values = {name: value for name, _, value in calls}
        self.assertEqual(values, {'line_follower_camera': 'false',
                                  'obstacle_avoidance_camera': 'false',
                                  'signage_detector': 'false'})

    def test_forward_debug_view_enables_only_its_node(self):
        _, _, calls = self.run_route('/api/set_cam_view?view=line_follower&source=forward')
        values = {name: value for name, _, value in calls}
        self.assertEqual(values.get('line_follower_camera'), 'true')
        self.assertEqual(values.get('signage_detector'), 'false')

    def test_unknown_source_ignored(self):
        _, node, _ = self.run_route('/api/set_cam_view?view=raw&source=bogus')
        self.assertEqual(node.active_camera_source, 'forward')


if __name__ == '__main__':
    unittest.main()
