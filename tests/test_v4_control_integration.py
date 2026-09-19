"""Stage 8 behavioral tests with inert ROS interfaces."""

import unittest
from unittest.mock import patch
import sys
import types

import ros_stub

ros_stub.install()
executors = types.ModuleType('rclpy.executors')
executors.ExternalShutdownException = type('ExternalShutdownException', (Exception,), {})
sys.modules['rclpy.executors'] = executors
from ros_stub import Twist

from risabot_v4_control.motion_executor import MotionExecutor


class V4ControlIntegrationTests(unittest.TestCase):
    def setUp(self):
        self.clock = patch('time.monotonic', return_value=100.0)
        self.clock.start()
        self.addCleanup(self.clock.stop)

    def ready(self):
        node = MotionExecutor()
        node._enabled = True
        node._gates = {name: True for name in node._gate_names}
        node._state = 'LANE_FOLLOW'
        node._state_stamp = 100.0
        node._proposal = {
            'source': 'trajectory', 'action': 'follow_curvature',
            'reference': {'valid': True, 'command_steer_rad_diagnostic_only': 0.1},
        }
        node._proposal_stamp = 100.0
        return node

    def test_closed_gate_always_publishes_zero(self):
        node = self.ready()
        node._gates['operator_motion_authorized'] = False
        node._control_loop()
        cmd = node._cmd_pub.messages[-1]
        self.assertEqual((cmd.linear.x, cmd.angular.z), (0.0, 0.0))

    def test_valid_lane_proposal_produces_guarded_raw_request(self):
        node = self.ready()
        node._control_loop()
        cmd = node._cmd_pub.messages[-1]
        self.assertGreater(cmd.linear.x, 0.0)
        self.assertLess(cmd.angular.z, 0.0)  # left-positive path -> right-positive contract

    def test_stale_proposal_and_estop_each_force_zero(self):
        for mutate in ('stale', 'estop'):
            node = self.ready()
            if mutate == 'stale':
                node._proposal_stamp = 98.0
            else:
                node._estop = True
            node._control_loop()
            cmd = node._cmd_pub.messages[-1]
            self.assertEqual((cmd.linear.x, cmd.angular.z), (0.0, 0.0))

    def test_tunnel_uses_only_fresh_specialized_command(self):
        node = self.ready()
        node._state = 'TUNNEL'
        node._legacy_stamp = 100.0
        node._legacy = Twist()
        node._legacy.linear.x = .06
        node._legacy.angular.z = .2
        node._control_loop()
        cmd = node._cmd_pub.messages[-1]
        self.assertEqual((cmd.linear.x, cmd.angular.z), (.06, .2))
        node._legacy_stamp = 98.0
        node._control_loop()
        cmd = node._cmd_pub.messages[-1]
        self.assertEqual((cmd.linear.x, cmd.angular.z), (0.0, 0.0))


if __name__ == '__main__':
    unittest.main()
