from pathlib import Path
import unittest

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

    def test_node_has_no_motion_message_or_command_topic(self):
        node_source = (
            Path(__file__).parents[1]
            / 'risabot_v4_experimental'
            / 'shadow_monitor.py'
        ).read_text(encoding='utf-8')
        self.assertNotIn('geometry_msgs', node_source)
        self.assertNotIn("'/cmd_vel", node_source)
        self.assertNotIn("'/cmd_vel_auto", node_source)


if __name__ == '__main__':
    unittest.main()
