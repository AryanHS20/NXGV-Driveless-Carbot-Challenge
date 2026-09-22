"""V4 dashboard parameter persistence keeps validation gates fail-closed."""

import tempfile
from pathlib import Path
import unittest
from unittest.mock import patch

import ros_stub

ros_stub.install()

from risabot_automode import dashboard


class DashboardV4ParameterTests(unittest.TestCase):
    def test_save_preserves_comments_and_does_not_persist_gates(self):
        original = (
            "v4_motion_executor:\n"
            "  ros__parameters:\n"
            "    # Must remain closed after dashboard tuning.\n"
            "    operator_motion_authorized: false\n"
            "    forward_speed_mps: 0.08  # crawl speed\n"
        )
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'v4_control.yaml'
            path.write_text(original, encoding='utf-8')
            old_paths = dashboard._PARAMS_SOURCE_PATHS
            old_defaults = dashboard._DEFAULT_PARAMS
            self.addCleanup(setattr, dashboard, '_PARAMS_SOURCE_PATHS', old_paths)
            self.addCleanup(setattr, dashboard, '_DEFAULT_PARAMS', old_defaults)
            dashboard._PARAMS_SOURCE_PATHS = [str(path)]
            dashboard._DEFAULT_PARAMS = {}

            def get_param(_node, name):
                values = {
                    'operator_motion_authorized': 'true',
                    'forward_speed_mps': '0.06',
                }
                return values[name], None

            with patch.object(dashboard, '_ros_get_param', side_effect=get_param):
                result = dashboard._save_params_to_yaml()

            saved = path.read_text(encoding='utf-8')
            self.assertTrue(result['ok'])
            self.assertIn('# Must remain closed after dashboard tuning.', saved)
            self.assertIn('operator_motion_authorized: false', saved)
            self.assertIn('forward_speed_mps: 0.06  # crawl speed', saved)


if __name__ == '__main__':
    unittest.main()
