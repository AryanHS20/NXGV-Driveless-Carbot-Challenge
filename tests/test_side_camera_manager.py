import os
import sys
import unittest

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, os.path.join(ROOT, 'src', 'risabot_automode'))

import ros_stub
ros_stub.install()

from risabot_automode.side_camera_manager import normalize_mode


class SideCameraModeTests(unittest.TestCase):
    def test_valid_modes_normalize(self):
        self.assertEqual(normalize_mode(' RIGHT '), 'right')
        self.assertEqual(normalize_mode('off'), 'off')
        self.assertEqual(normalize_mode('both'), 'both')

    def test_invalid_mode_rejected(self):
        with self.assertRaises(ValueError):
            normalize_mode('forward')


if __name__ == '__main__':
    unittest.main()
