from pathlib import Path
import unittest

import cv2
import numpy as np

from risabot_v4_experimental.bev_core import (
    CalibrationError,
    build_homography,
    ground_to_bev_points,
    load_profiles,
    profile_from_mapping,
    warp_to_bev,
)
from risabot_v4_experimental.calibrate_intrinsics import (
    calibrate_from_observations,
    checkerboard_object_points,
)


def calibrated_mapping():
    return {
        'calibrated': True,
        'resolution': [101, 101],
        'camera_matrix': [[100.0, 0.0, 50.0], [0.0, 100.0, 50.0], [0.0, 0.0, 1.0]],
        'distortion_coefficients': [0.0, 0.0, 0.0, 0.0, 0.0],
        'source_points_px': [[0.0, 0.0], [100.0, 0.0], [100.0, 100.0], [0.0, 100.0]],
        'ground_points_m': [[1.0, 0.5], [1.0, -0.5], [0.0, -0.5], [0.0, 0.5]],
        'ground_bounds_m': {'forward': [0.0, 1.0], 'left': [-0.5, 0.5]},
        'pixels_per_meter': 100.0,
    }


class BevCoreTests(unittest.TestCase):
    def test_synthetic_intrinsic_calibration_recovers_low_reprojection_error(self):
        obj = checkerboard_object_points(9, 6, 0.024)
        matrix = np.array(
            [[720.0, 0.0, 480.0], [0.0, 715.0, 272.0], [0.0, 0.0, 1.0]],
            dtype=np.float64,
        )
        objects = []
        images = []
        for index in range(8):
            rvec = np.array([0.03 * index, -0.02 + 0.008 * index, 0.01 * index])
            tvec = np.array([-0.08 + 0.02 * index, -0.05 + 0.01 * index, 0.65 + 0.04 * index])
            projected, _ = cv2.projectPoints(obj, rvec, tvec, matrix, np.zeros(5))
            objects.append(obj.copy())
            images.append(projected.astype(np.float32))
        result = calibrate_from_observations(objects, images, (960, 544))
        self.assertEqual(result['frames_used'], 8)
        self.assertLess(result['intrinsic_rms_px'], 1e-3)

    def test_metric_correspondence_builds_identity_homography(self):
        profile = profile_from_mapping('test', calibrated_mapping())
        expected = np.array(
            [[0.0, 0.0], [100.0, 0.0], [100.0, 100.0], [0.0, 100.0]],
            dtype=np.float32,
        )
        np.testing.assert_allclose(ground_to_bev_points(profile), expected, atol=1e-5)
        np.testing.assert_allclose(build_homography(profile), np.eye(3), atol=1e-5)

    def test_identity_profile_preserves_synthetic_image_and_coverage(self):
        profile = profile_from_mapping('test', calibrated_mapping())
        x = np.arange(101, dtype=np.uint8)
        image = np.dstack((np.tile(x, (101, 1)), np.tile(x[:, None], (1, 101)), np.full((101, 101), 90, np.uint8)))
        bev, coverage = warp_to_bev(image, profile)
        self.assertEqual(bev.shape, image.shape)
        self.assertEqual(coverage.shape, image.shape[:2])
        np.testing.assert_allclose(bev, image, atol=1)
        self.assertGreater(float((coverage > 0).mean()), 0.99)

    def test_uncalibrated_repository_profiles_are_explicitly_rejected(self):
        config = Path(__file__).parents[1] / 'config' / 'camera_profiles.yaml'
        profiles = load_profiles(str(config))
        self.assertFalse(profiles['primary'].calibrated)
        self.assertFalse(profiles['secondary'].calibrated)
        with self.assertRaisesRegex(CalibrationError, 'not calibrated'):
            warp_to_bev(np.zeros((544, 960, 3), np.uint8), profiles['primary'])

    def test_resolution_mismatch_is_rejected(self):
        profile = profile_from_mapping('test', calibrated_mapping())
        with self.assertRaisesRegex(CalibrationError, 'expected 101x101'):
            warp_to_bev(np.zeros((100, 101, 3), np.uint8), profile)

    def test_out_of_image_correspondence_is_rejected(self):
        raw = calibrated_mapping()
        raw['source_points_px'][1][0] = 101.0
        with self.assertRaisesRegex(CalibrationError, 'outside the image'):
            profile_from_mapping('test', raw)


if __name__ == '__main__':
    unittest.main()
