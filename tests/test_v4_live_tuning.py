"""Atomic validation for dashboard-exposed V4 lane parameters."""

import sys
import types
import unittest

import ros_stub

ros_stub.install()
executors = types.ModuleType('rclpy.executors')
executors.ExternalShutdownException = type('ExternalShutdownException', (Exception,), {})
sys.modules['rclpy.executors'] = executors

from ros_stub import Parameter
from risabot_v4_experimental.bev_shadow import BevShadow
from risabot_v4_experimental.road_mask_core import RoadMaskConfig
from risabot_v4_experimental.road_mask_shadow import RoadMaskShadow
from risabot_v4_experimental.trajectory_core import TrajectoryConfig
from risabot_v4_experimental.trajectory_shadow import TrajectoryShadow


class V4LiveTuningTests(unittest.TestCase):
    def test_bev_rate_is_bounded_and_live(self):
        node = object.__new__(BevShadow)
        node._max_hz = 5.0
        accepted = node._on_parameters([Parameter('max_hz', value=8.0)])
        self.assertTrue(accepted.successful)
        self.assertEqual(node._max_hz, 8.0)
        rejected = node._on_parameters([Parameter('max_hz', value=100.0)])
        self.assertFalse(rejected.successful)
        self.assertEqual(node._max_hz, 8.0)

    def test_road_threshold_update_is_atomic(self):
        node = object.__new__(RoadMaskShadow)
        node._config = RoadMaskConfig()
        node._seed_radius_m = 0.10
        node._min_width_m = 0.12
        node._max_width_m = 0.80
        node._row_step = 8
        node._memory_age = 2.0
        node._memory_distance = 0.5
        node._reset_jump = 0.5
        node._reset_yaw = 1.0
        accepted = node._on_parameters([
            Parameter('value_max', value=145),
            Parameter('min_corridor_width_m', value=0.15),
        ])
        self.assertTrue(accepted.successful)
        self.assertEqual(node._config.value_max, 145)
        self.assertEqual(node._min_width_m, 0.15)
        rejected = node._on_parameters([
            Parameter('morph_open_px', value=4),
            Parameter('value_max', value=155),
        ])
        self.assertFalse(rejected.successful)
        self.assertEqual(node._config.value_max, 145)

    def test_trajectory_config_rejects_unsafe_support(self):
        node = object.__new__(TrajectoryShadow)
        node._config = TrajectoryConfig()
        accepted = node._on_parameters([
            Parameter('lookahead_m', value=0.11),
        ])
        self.assertTrue(accepted.successful)
        self.assertEqual(node._config.lookahead_m, 0.11)
        rejected = node._on_parameters([
            Parameter('minimum_road_support', value=1.1),
        ])
        self.assertFalse(rejected.successful)
        self.assertEqual(node._config.minimum_road_support, 0.98)

    def test_validation_gates_are_restart_only(self):
        node = object.__new__(RoadMaskShadow)
        node._config = RoadMaskConfig()
        node._seed_radius_m = 0.10
        node._min_width_m = 0.12
        node._max_width_m = 0.80
        node._row_step = 8
        node._memory_age = 2.0
        node._memory_distance = 0.5
        node._reset_jump = 0.5
        node._reset_yaw = 1.0
        result = node._on_parameters([
            Parameter('thresholds_validated', value=True),
        ])
        self.assertFalse(result.successful)


if __name__ == '__main__':
    unittest.main()
