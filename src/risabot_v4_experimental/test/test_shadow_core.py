from pathlib import Path
import unittest

import yaml

from risabot_v4_experimental.shadow_core import evaluate_inputs


class ShadowCoreTests(unittest.TestCase):
    def test_missing_and_stale_inputs_block_shadow_readiness(self):
        report = evaluate_inputs(
            now=10.0,
            last_seen={'camera': 9.0, 'scan': None, 'odom': 9.9},
            timeouts={'camera': 0.45, 'scan': 0.5, 'odom': 0.25},
            required=['camera', 'scan', 'odom'],
        )
        self.assertFalse(report['ready_for_shadow_evaluation'])
        self.assertEqual(report['missing'], ['scan'])
        self.assertEqual(report['stale'], ['camera'])
        self.assertFalse(report['motion_authority'])

    def test_fresh_inputs_only_enable_evaluation(self):
        report = evaluate_inputs(
            now=10.0,
            last_seen={'camera': 9.8, 'scan': 9.7, 'odom': 9.9},
            timeouts={'camera': 0.45, 'scan': 0.5, 'odom': 0.25},
            required=['camera', 'scan', 'odom'],
        )
        self.assertTrue(report['ready_for_shadow_evaluation'])
        self.assertFalse(report['motion_authority'])

    def test_package_has_no_motion_message_or_command_topic(self):
        source_dir = Path(__file__).parents[1] / 'risabot_v4_experimental'
        node_source = '\n'.join(
            path.read_text(encoding='utf-8')
            for path in sorted(source_dir.glob('*.py'))
        )
        self.assertNotIn('geometry_msgs', node_source)
        self.assertNotIn("'/cmd_vel", node_source)
        self.assertNotIn('"/cmd_vel', node_source)
        self.assertNotIn('AckermannDrive', node_source)
        self.assertNotIn('/parking_command', node_source)
        self.assertNotIn('/parking_cmd_vel', node_source)

    def test_stage4_physical_validation_gates_ship_closed(self):
        config_path = Path(__file__).parents[1] / 'config' / 'v4_experimental.yaml'
        config = yaml.safe_load(config_path.read_text(encoding='utf-8'))
        params = config['v4_trajectory_shadow']['ros__parameters']
        self.assertFalse(params['enabled'])
        self.assertFalse(params['vehicle_geometry_validated'])
        self.assertFalse(params['minimum_turn_radius_validated'])
        self.assertFalse(params['lidar_extrinsics_validated'])

        pose_params = config['v4_pose_shadow']['ros__parameters']
        self.assertFalse(pose_params['enabled'])
        self.assertFalse(pose_params['uwb_frame_alignment_validated'])

        parking_params = config['v4_parking_shadow']['ros__parameters']
        self.assertFalse(parking_params['enabled'])
        for gate in (
            'parking_goal_source_validated', 'slot_geometry_validated',
            'rear_coverage_validated', 'vehicle_geometry_validated',
            'minimum_turn_radius_validated', 'lidar_extrinsics_validated',
        ):
            self.assertFalse(parking_params[gate])


if __name__ == '__main__':
    unittest.main()
