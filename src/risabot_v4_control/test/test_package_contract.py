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
        self.assertFalse(trajectory['enforce_road_support_in_track_test'])
        self.assertEqual(trajectory['road_support_cost_weight'], 100.0)
        self.assertEqual(trajectory['footprint_padding_m'], 0.010)
        self.assertEqual(trajectory['expected_lane_width_m'], 0.32)
        self.assertEqual(trajectory['plan_hold_sec'], 0.60)
        executor = params['v4_motion_executor']
        self.assertAlmostEqual(executor['minimum_speed_scale'], 48.0 / 65.0)
        self.assertEqual(executor['steering_slowdown_gain'], 0.85)


if __name__ == '__main__':
    unittest.main()
