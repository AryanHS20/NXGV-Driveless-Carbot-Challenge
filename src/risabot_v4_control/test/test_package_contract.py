from pathlib import Path
import unittest

import yaml


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


if __name__ == '__main__':
    unittest.main()
