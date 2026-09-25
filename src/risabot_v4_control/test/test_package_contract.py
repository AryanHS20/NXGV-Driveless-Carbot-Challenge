from pathlib import Path
import unittest

import yaml

from risabot_v4_control.track_test_config import track_test_overrides


class PackageContractTests(unittest.TestCase):
    def test_stage8_ships_disabled_with_every_gate_closed(self):
        root = Path(__file__).parents[1]
        config = yaml.safe_load((root / 'config' / 'v4_control.yaml').read_text())
        params = config['v4_motion_executor']['ros__parameters']
        self.assertFalse(params['enabled'])
        for gate in ('command_contract_validated', 'stop_preemption_validated',
                     'timeout_validated', 'operator_motion_authorized'):
            self.assertFalse(params[gate])

    def test_stage8_uses_separate_raw_topic_and_never_hardware_topic(self):
        source = (Path(__file__).parents[1] / 'risabot_v4_control' / 'motion_executor.py').read_text()
        self.assertIn("'/cmd_vel_v4_raw'", source)
        self.assertNotIn("'/cmd_vel'", source)
        self.assertNotIn("'/cmd_vel_auto'", source)
        self.assertNotIn('servo_controller', source)

    def test_risabot5_track_test_holds_the_32cm_lane_center(self):
        params = track_test_overrides('risabot5', 65.0, 2.0)
        trajectory = params['v4_trajectory_shadow']
        self.assertTrue(trajectory['enforce_road_support_in_track_test'])
        self.assertEqual(trajectory['road_support_cost_weight'], 100.0)
        self.assertEqual(trajectory['footprint_padding_m'], 0.010)
        self.assertEqual(trajectory['expected_lane_width_m'], 0.32)
        self.assertEqual(trajectory['plan_hold_sec'], 0.0)
        executor = params['v4_motion_executor']
        self.assertAlmostEqual(executor['minimum_speed_scale'], 48.0 / 65.0)
        self.assertEqual(executor['steering_slowdown_gain'], 0.85)

    def test_risabot1_uses_measured_road_and_tire_width(self):
        trajectory = track_test_overrides('risabot1')['v4_trajectory_shadow']
        self.assertTrue(trajectory['enforce_road_support_in_track_test'])
        self.assertEqual(trajectory['plan_hold_sec'], 0.0)
        self.assertEqual(trajectory['road_timeout_sec'], 0.35)
        self.assertEqual(trajectory['minimum_road_support'], 0.96)
        self.assertEqual(trajectory['expected_lane_width_m'], 0.295)
        self.assertEqual(trajectory['vehicle_length_m'], 0.275)
        self.assertEqual(trajectory['vehicle_width_m'], 0.185)
        self.assertEqual(trajectory['footprint_padding_m'], 0.010)
        self.assertEqual(trajectory['cross_track_gain'], 0.75)
        self.assertEqual(trajectory['near_center_guard_m'], 0.0)
        self.assertEqual(trajectory['steering_rate_rad_sec'], 2.0)
        self.assertEqual(trajectory['boundary_recovery_error_m'], 0.020)
        self.assertEqual(trajectory['boundary_recovery_steer_rad'], 0.30)
        self.assertEqual(trajectory['minimum_scan_range_m'], 0.12)

    def test_risabot1_track_launch_starts_advisory_signage(self):
        launch = (Path(__file__).parents[1] / 'launch' / 'track_test.launch.py').read_text()
        self.assertIn("vehicle == 'risabot1' and flag('start_signage')", launch)
        self.assertIn("executable='signage_detector'", launch)
        self.assertIn("'publish_boom_state': True", launch)


if __name__ == '__main__':
    unittest.main()
