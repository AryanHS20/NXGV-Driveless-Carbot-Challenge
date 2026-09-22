"""Unit tests for the V4 telemetry bridge (pure python + stub node)."""
import json
import math
import time
import types
import unittest

import ros_stub

ros_stub.install()

from risabot_automode import v4_telemetry_bridge as bridge


class PureHelperTests(unittest.TestCase):
    def test_odom_pose_and_sigma(self):
        pose = bridge.odom_pose(ros_stub.Odometry())
        self.assertEqual(pose, {'x': 0.0, 'y': 0.0, 'yaw': 0.0})

    def test_corridor_validation_and_cap(self):
        payload = {'corridor': {'primary': [
            {'forward_m': 0.5, 'left_m': 0.0, 'width_m': 0.3},
            {'forward_m': 1.0},
            {'forward_m': 1.0, 'left_m': 0.0, 'width_m': -1.0},
        ]}}
        got = bridge.corridor_points(payload)
        self.assertEqual(len(got), 1)
        self.assertEqual(bridge.corridor_points({}), [])
        self.assertEqual(bridge.corridor_points(None), [])

    def test_selected_trajectory(self):
        payload = {'selected_diagnostic_only': {
            'id': 3, 'valid': True,
            'endpoint_m': {'forward': 2.0, 'left': -0.1, 'yaw': 0.0}}}
        got = bridge.selected_trajectory(payload)
        self.assertEqual(got['id'], 3)
        self.assertAlmostEqual(got['forward_m'], 2.0)
        bad = {'selected_diagnostic_only': {'valid': False}}
        self.assertIsNone(bridge.selected_trajectory(bad))
        self.assertIsNone(bridge.selected_trajectory({}))

    def test_scan_downsample_and_gates(self):
        msg = types.SimpleNamespace(
            ranges=[0.01, 0.5, 1.0, float('inf'), 20.0, 2.0],
            angle_min=0.0, angle_increment=math.pi / 180.0)
        pts = bridge.scan_points(msg)
        dists = sorted(math.hypot(x, y) for x, y in pts)
        self.assertTrue(all(0.05 <= d <= 8.0 for d in dists))
        self.assertLessEqual(len(pts), bridge.MAX_OBSTACLE_POINTS)
        self.assertEqual(bridge.scan_points(types.SimpleNamespace()), [])

    def test_build_payload_freshness(self):
        now = time.monotonic()
        latest = {
            'local': ({'x': 1.0, 'y': 2.0, 'yaw': 0.1}, now),
            'coarse': ({'x': 1.1, 'y': 2.1}, now - 5.0),
        }
        doc = bridge.build_payload(latest, now, 1.0)
        self.assertEqual(doc['local']['x'], 1.0)
        self.assertIsNone(doc['coarse'])
        self.assertIn('coarse', doc['stale'])
        self.assertNotIn('local', doc['stale'])
        self.assertFalse(doc['motion_authority'])
        self.assertEqual(doc['corridor'], [])
        self.assertEqual(doc['obstacles'], [])


class NodeWiringTests(unittest.TestCase):
    def test_subscriptions_and_tick(self):
        node = bridge.V4TelemetryBridge()
        self.assertEqual(set(node.subs), {
            '/v4_experimental/pose/local', '/v4_experimental/pose/coarse',
            '/v4_experimental/road/status', '/v4_experimental/trajectory/status',
            '/scan'})
        node._road_cb(ros_stub.Message(json.dumps({'corridor': {'primary': [
            {'forward_m': 0.5, 'left_m': 0.0, 'width_m': 0.3}]}})))
        node._tick()
        self.assertEqual(len(node._pub.messages), 1)
        doc = json.loads(node._pub.messages[0].data)
        self.assertFalse(doc['motion_authority'])
        self.assertEqual(len(doc['corridor']), 1)
        self.assertIn('local', doc['stale'])  # never received
        node.close() if hasattr(node, 'close') else None

    def test_read_only_source_audit(self):
        import pathlib
        source = pathlib.Path(__file__).resolve().parents[1] / 'src' / 'risabot_automode' \
            / 'risabot_automode' / 'v4_telemetry_bridge.py'
        text = source.read_text(encoding='utf-8')
        for banned in ('Twist', 'cmd_vel', 'create_client', 'set_parameters',
                       'Servo', 'motor', 'call_async'):
            self.assertNotIn(banned, text)


if __name__ == '__main__':
    unittest.main()
