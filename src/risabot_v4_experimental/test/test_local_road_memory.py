import unittest
import math

import numpy as np

from risabot_v4_experimental.bev_core import profile_from_mapping
from risabot_v4_experimental.local_road_memory import LocalRoadMemory, Pose2D


def memory_profile():
    return profile_from_mapping('memory', {
        'calibrated': True,
        'resolution': [101, 101],
        'camera_matrix': [[100.0, 0.0, 50.0], [0.0, 100.0, 50.0], [0.0, 0.0, 1.0]],
        'distortion_coefficients': [0.0, 0.0, 0.0, 0.0, 0.0],
        'source_points_px': [[0.0, 0.0], [100.0, 0.0], [100.0, 100.0], [0.0, 100.0]],
        'ground_points_m': [[1.0, 0.5], [1.0, -0.5], [0.0, -0.5], [0.0, 0.5]],
        'ground_bounds_m': {'forward': [0.0, 1.0], 'left': [-0.5, 0.5]},
        'pixels_per_meter': 100.0,
    })


class LocalRoadMemoryTests(unittest.TestCase):
    def test_same_pose_renders_recent_evidence(self):
        profile = memory_profile()
        memory = LocalRoadMemory(cell_size_m=0.02)
        mask = np.zeros((101, 101), np.uint8)
        mask[20:91, 40:61] = 255
        count = memory.integrate(mask, profile, Pose2D(0.0, 0.0, 0.0), 1.0, 0.0)
        rendered = memory.render(profile, Pose2D(0.0, 0.0, 0.0), 1.1, 0.0, 2.0, 0.5)
        self.assertGreater(count, 100)
        self.assertGreater(int((rendered > 0).sum()), 100)
        self.assertGreater(rendered[50, 50], 0)

    def test_odometry_translation_moves_memory_in_vehicle_frame(self):
        profile = memory_profile()
        memory = LocalRoadMemory(cell_size_m=0.01)
        mask = np.zeros((101, 101), np.uint8)
        mask[50, 50] = 255  # forward 0.5 m, centreline
        memory.integrate(mask, profile, Pose2D(0.0, 0.0, 0.0), 1.0, 0.0)
        rendered = memory.render(profile, Pose2D(0.2, 0.0, 0.0), 1.1, 0.2, 2.0, 0.5)
        # Point is now 0.3 m ahead, which maps from row 50 to row 70.
        self.assertGreater(rendered[70, 50], 0)
        self.assertEqual(rendered[50, 50], 0)

    def test_odometry_rotation_moves_memory_in_vehicle_frame(self):
        profile = memory_profile()
        memory = LocalRoadMemory(cell_size_m=0.01)
        mask = np.zeros((101, 101), np.uint8)
        mask[50, 50] = 255  # world point 0.5 m ahead at the original heading
        memory.integrate(mask, profile, Pose2D(0.0, 0.0, 0.0), 1.0, 0.0)
        rendered = memory.render(
            profile, Pose2D(0.0, 0.0, math.pi / 2), 1.1, 0.0, 2.0, 0.5
        )
        # After a 90-degree left turn, that point is 0.5 m to the right.
        self.assertGreater(rendered[100, 100], 0)
        self.assertEqual(rendered[50, 50], 0)

    def test_memory_expires_by_age_and_travel(self):
        profile = memory_profile()
        memory = LocalRoadMemory(cell_size_m=0.01)
        mask = np.zeros((101, 101), np.uint8)
        mask[50, 50] = 255
        memory.integrate(mask, profile, Pose2D(0.0, 0.0, 0.0), 1.0, 0.0)
        aged = memory.render(profile, Pose2D(0.0, 0.0, 0.0), 4.0, 0.0, 2.0, 0.5)
        travelled = memory.render(profile, Pose2D(0.0, 0.0, 0.0), 1.1, 0.8, 2.0, 0.5)
        self.assertFalse(np.any(aged))
        self.assertFalse(np.any(travelled))

    def test_clear_removes_all_cells(self):
        profile = memory_profile()
        memory = LocalRoadMemory()
        mask = np.full((101, 101), 255, np.uint8)
        memory.integrate(mask, profile, Pose2D(0.0, 0.0, 0.0), 1.0, 0.0)
        self.assertTrue(memory.cells)
        memory.clear()
        self.assertFalse(memory.cells)


if __name__ == '__main__':
    unittest.main()
