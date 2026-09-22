"""Unit tests for dashboard camera source/view switching (no robot)."""
import threading
import time
import types
import unittest
import json

import cv2
import numpy as np

import ros_stub

ros_stub.install()

from risabot_automode.dashboard import DashboardNode
from risabot_automode.dashboard_panels import routes_camera


class FakeBridge:
    def __init__(self):
        self.calls = 0

    def imgmsg_to_cv2(self, msg, desired_encoding='bgr8'):
        self.calls += 1
        if hasattr(msg, 'image'):
            return msg.image
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
    node._last_camera_source_mono = 0.0
    node._encode_min_interval = 0.0
    node._camera_selection_generation = 0
    node._init_v4_composite_state()
    return node


def image_msg(stamp_sec, color, nanosec=0, shape=(120, 240, 3)):
    image = np.empty(shape, dtype=np.uint8)
    image[:] = color
    return types.SimpleNamespace(
        header=types.SimpleNamespace(
            stamp=types.SimpleNamespace(sec=stamp_sec, nanosec=nanosec)),
        image=image,
    )


def decode_latest(node):
    return cv2.imdecode(np.frombuffer(node.latest_jpeg, np.uint8), cv2.IMREAD_COLOR)


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
        self.assertEqual(node.bridge.calls, 0)

    def test_idle_dashboard_does_not_override_other_camera_requesters(self):
        node = make_dashboard('raw', 'second')
        node.num_camera_clients = 0
        published = []
        node.side_camera_request_pub = types.SimpleNamespace(
            publish=lambda msg: published.append(msg.data))
        node._side_camera_lease_loop()
        self.assertEqual(published, [])

    def test_side_viewer_renews_camera_lease(self):
        node = make_dashboard('raw', 'second')
        published = []
        node.side_camera_request_pub = types.SimpleNamespace(
            publish=lambda msg: published.append(msg.data))
        node._side_camera_lease_loop()
        self.assertEqual(published, ['right'])


class V4CompositeTests(unittest.TestCase):
    TILE_CENTERS = {
        'bev': (160, 136),
        'coverage': (480, 136),
        'candidate': (800, 136),
        'connected': (1120, 136),
        'fused': (1440, 136),
    }

    def make_node(self):
        node = make_dashboard('road', 'forward')
        node._v4_sync_wait_sec = 0.0
        return node

    def send_set(self, node, stamp, colors):
        for name in ('bev', 'coverage', 'candidate', 'connected', 'fused'):
            node._v4_comp_cb(image_msg(stamp, colors[name]), name)

    def assert_color_near(self, actual, expected, tolerance=18):
        self.assertTrue(
            np.all(np.abs(actual.astype(int) - np.array(expected)) <= tolerance),
            (actual, expected),
        )

    def test_complete_composite_uses_one_source_stamp(self):
        node = self.make_node()
        colors = {
            'bev': (20, 40, 220),
            'coverage': (20, 210, 40),
            'candidate': (220, 40, 20),
            'connected': (180, 180, 30),
            'fused': (180, 30, 180),
        }
        self.send_set(node, 12, colors)

        frame = decode_latest(node)
        self.assertEqual(frame.shape[:2], (272, 1600))
        self.assertEqual(node._v4_render_token[0:2], (12_000_000_000, 'fresh'))
        for name, (x, y) in self.TILE_CENTERS.items():
            self.assert_color_near(frame[y, x], colors[name])

    def test_new_partial_stamp_never_reuses_old_tiles(self):
        node = self.make_node()
        old_colors = {name: (30, 30, 210) for name, _ in (
            ('bev', ''), ('coverage', ''), ('candidate', ''),
            ('connected', ''), ('fused', ''),
        )}
        self.send_set(node, 20, old_colors)
        previous_frame_id = node.frame_id

        node._v4_comp_cb(image_msg(21, (20, 210, 40)), 'bev')
        frame = decode_latest(node)
        self.assertGreater(node.frame_id, previous_frame_id)
        self.assertEqual(node._v4_render_token[0:2], (21_000_000_000, 'partial'))
        self.assert_color_near(frame[136, 160], (20, 210, 40))
        # Coverage is a placeholder for stamp 21, not the red stamp-20 tile.
        self.assertLess(int(frame[136, 480].max()), 55)

        partial_frame_id = node.frame_id
        node._v4_comp_cb(image_msg(20, (220, 220, 220)), 'coverage')
        self.assertEqual(node.frame_id, partial_frame_id)

    def test_missing_inputs_render_explicit_placeholders(self):
        node = self.make_node()
        node._v4_comp_cb(image_msg(30, (200, 100, 20)), 'candidate')

        frame = decode_latest(node)
        self.assertIsNotNone(frame)
        self.assertEqual(node._v4_render_token[1], 'partial')
        self.assertLess(int(frame[136, 160].max()), 55)
        self.assert_color_near(frame[136, 800], (200, 100, 20))

    def test_stale_set_replaces_every_old_image(self):
        node = self.make_node()
        colors = {name: (210, 120, 30) for name, _ in (
            ('bev', ''), ('coverage', ''), ('candidate', ''),
            ('connected', ''), ('fused', ''),
        )}
        self.send_set(node, 40, colors)
        with node._v4_lock:
            node._v4_sets[40_000_000_000]['first_received'] -= 2.0
        node._last_encode_mono = 0.0

        node._v4_comp_watchdog()
        frame = decode_latest(node)
        self.assertEqual(node._v4_render_token[1], 'stale')
        for x, y in self.TILE_CENTERS.values():
            self.assertLess(int(frame[y, x].max()), 55)

    def test_road_view_is_forward_only_and_demand_driven(self):
        node = make_dashboard('road', 'second')
        node._v4_comp_cb(image_msg(50, (1, 2, 3)), 'bev')
        self.assertEqual(node.bridge.calls, 0)
        self.assertIsNone(node.latest_jpeg)

        node.active_camera_source = 'forward'
        node.num_camera_clients = 0
        node._v4_comp_cb(image_msg(50, (1, 2, 3)), 'bev')
        self.assertEqual(node.bridge.calls, 0)

    def test_stamp_cache_stays_bounded(self):
        node = self.make_node()
        for stamp in range(60, 66):
            node._v4_comp_cb(image_msg(stamp, (20, 40, 60)), 'bev')
        with node._v4_lock:
            self.assertEqual(sorted(node._v4_sets), [
                63_000_000_000, 64_000_000_000, 65_000_000_000,
            ])

    def test_unstamped_input_is_not_converted_or_shown_as_current(self):
        node = self.make_node()
        msg = image_msg(0, (1, 2, 3))
        node._v4_comp_cb(msg, 'bev')
        self.assertEqual(node.bridge.calls, 0)
        self.assertEqual(node._v4_render_token, ('waiting',))
        self.assertIsNotNone(node.latest_jpeg)

    def test_letterboxing_preserves_source_aspect_ratio(self):
        image = np.full((100, 200, 3), 240, dtype=np.uint8)
        tile = DashboardNode._v4_tile(image, 'WIDE')
        # 2:1 image fits at 320x160 in the 320x208 content area.
        self.assertTrue(np.all(tile[40, 160] < 30))
        self.assertTrue(np.all(tile[136, 160] == 240))
        self.assertTrue(np.all(tile[230, 160] < 30))


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

    def test_processed_view_forces_forward_source(self):
        _, node, calls = self.run_route(
            '/api/set_cam_view?view=road&source=second')
        self.assertEqual(node.active_camera_view, 'road')
        self.assertEqual(node.active_camera_source, 'forward')
        values = {name: value for name, _, value in calls}
        self.assertTrue(all(value == 'false' for value in values.values()))

    def test_view_switch_clears_obsolete_v4_frames(self):
        handler = FakeHandler('/api/set_cam_view?view=raw&source=second')
        node = make_dashboard('road', 'forward')
        node.latest_jpeg = b'old-road-frame'
        with node._v4_lock:
            node._v4_sets[1] = {'first_received': time.monotonic(), 'tiles': {}}
        calls = []
        routes_camera.set_view(make_ctx(handler, node, calls), handler.path)
        with node._v4_lock:
            self.assertEqual(node._v4_sets, {})
        self.assertIsNone(node.latest_jpeg)
        self.assertEqual(node.active_camera_view, 'raw')
        self.assertEqual(node.active_camera_source, 'second')

    def test_forward_debug_view_enables_only_its_node(self):
        _, _, calls = self.run_route('/api/set_cam_view?view=line_follower&source=forward')
        values = {name: value for name, _, value in calls}
        self.assertEqual(values.get('line_follower_camera'), 'true')
        self.assertEqual(values.get('signage_detector'), 'false')

    def test_unknown_source_ignored(self):
        _, node, _ = self.run_route('/api/set_cam_view?view=raw&source=bogus')
        self.assertEqual(node.active_camera_source, 'forward')


class DashboardStatusTests(unittest.TestCase):
    def make_node(self):
        node = DashboardNode.__new__(DashboardNode)
        node.data_lock = threading.Lock()
        node.data = {
            'v4_status': {},
            'cmd_safety_estop': False,
            'cmd_safety_timeout_count': 0,
            'cmd_safety_autonomy_source': 'unknown',
        }
        node.topic_last_update = {'cmd_safety_status': 0.0}
        return node

    def test_v4_status_callback_retains_payload_and_receive_time(self):
        node = self.make_node()
        node._v4_status_cb('control', types.SimpleNamespace(data=json.dumps({
            'algorithm_stage': 8, 'enabled': True, 'blockers': []})))
        status = node.data['v4_status']['control']
        self.assertEqual(status['algorithm_stage'], 8)
        self.assertGreater(status['_received_mono'], 0.0)

    def test_safety_status_exposes_selected_autonomy_source(self):
        node = self.make_node()
        node._cmd_safety_cb(types.SimpleNamespace(data=json.dumps({
            'estop': True, 'timeout_count': 3, 'autonomy_source': 'v4'})))
        self.assertTrue(node.data['cmd_safety_estop'])
        self.assertEqual(node.data['cmd_safety_timeout_count'], 3)
        self.assertEqual(node.data['cmd_safety_autonomy_source'], 'v4')

    def test_mode_heartbeat_logs_only_real_transitions(self):
        node = DashboardNode.__new__(DashboardNode)
        node.data_lock = threading.Lock()
        node.data = {'auto_mode': False}
        node.topic_last_update = {'auto_mode': 0.0}
        logs = []
        node.get_logger = lambda: types.SimpleNamespace(info=logs.append)

        node._auto_mode_cb(types.SimpleNamespace(data=False))
        self.assertEqual(logs, [])
        self.assertGreater(node.topic_last_update['auto_mode'], 0.0)

        node._auto_mode_cb(types.SimpleNamespace(data=True))
        node._auto_mode_cb(types.SimpleNamespace(data=True))
        self.assertEqual(logs, ['Mode changed: AUTO'])


if __name__ == '__main__':
    unittest.main()
