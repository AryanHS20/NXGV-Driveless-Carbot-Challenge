import math
import unittest

from risabot_v4_experimental.pose_estimator_core import (
    EstimatorConfig,
    Pose2D,
    SeparatedPoseEstimator,
)


class PoseEstimatorCoreTests(unittest.TestCase):
    def test_uwb_changes_only_coarse_offset(self):
        estimator = SeparatedPoseEstimator()
        estimator.update_local(Pose2D(1.0, 2.0, 0.3), 1.0)
        before = estimator.local_pose
        result = estimator.update_uwb(1.4, 1.8, 1.1, sigma_m=0.1)
        self.assertEqual(result['accepted'], 1.0)
        self.assertEqual(estimator.local_pose, before)
        self.assertNotEqual(estimator.coarse_pose(), before)

    def test_high_noise_measurement_has_lower_gain(self):
        low = SeparatedPoseEstimator()
        high = SeparatedPoseEstimator()
        for estimator in (low, high):
            estimator.update_local(Pose2D(0.0, 0.0, 0.0), 1.0)
        low_gain = low.update_uwb(0.3, 0.0, 1.1, sigma_m=0.05)['gain']
        high_gain = high.update_uwb(0.3, 0.0, 1.1, sigma_m=1.0)['gain']
        self.assertGreater(low_gain, high_gain)

    def test_gross_outlier_is_rejected(self):
        estimator = SeparatedPoseEstimator()
        estimator.update_local(Pose2D(0.0, 0.0, 0.0), 1.0)
        result = estimator.update_uwb(100.0, -100.0, 1.1, sigma_m=0.1)
        self.assertEqual(result['accepted'], 0.0)
        self.assertEqual(estimator.coarse_pose(), estimator.local_pose)

    def test_local_motion_moves_local_and_coarse_equally(self):
        estimator = SeparatedPoseEstimator()
        estimator.update_local(Pose2D(0.0, 0.0, 0.0), 1.0)
        estimator.update_uwb(0.2, -0.1, 1.1, sigma_m=0.1)
        first_local = estimator.local_pose
        first_coarse = estimator.coarse_pose()
        estimator.update_local(Pose2D(0.1, 0.03, 0.02), 1.2)
        second_local = estimator.local_pose
        second_coarse = estimator.coarse_pose()
        self.assertAlmostEqual(second_local.x - first_local.x, second_coarse.x - first_coarse.x)
        self.assertAlmostEqual(second_local.y - first_local.y, second_coarse.y - first_coarse.y)

    def test_odom_discontinuity_clears_global_offset(self):
        config = EstimatorConfig(odom_reset_jump_m=0.3)
        estimator = SeparatedPoseEstimator(config)
        estimator.update_local(Pose2D(0.0, 0.0, 0.0), 1.0)
        estimator.update_uwb(0.2, 0.0, 1.1, sigma_m=0.1)
        estimator.update_local(Pose2D(1.0, 0.0, 0.0), 1.2)
        self.assertEqual(estimator.offset_x, 0.0)
        self.assertEqual(estimator.odom_resets, 1)

    def test_repeated_uwb_timestamp_is_rejected(self):
        estimator = SeparatedPoseEstimator()
        estimator.update_local(Pose2D(0.0, 0.0, 0.0), 1.0)
        estimator.update_uwb(0.1, 0.0, 1.1, sigma_m=0.1)
        result = estimator.update_uwb(0.2, 0.0, 1.1, sigma_m=0.1)
        self.assertEqual(result['reason_code'], 1.0)

    def test_frame_alignment_rotates_local_into_coarse_frame(self):
        estimator = SeparatedPoseEstimator(
            EstimatorConfig(frame_yaw_rad=math.pi / 2.0)
        )
        estimator.update_local(Pose2D(1.0, 0.0, 0.2), 1.0)
        coarse = estimator.coarse_pose()
        self.assertAlmostEqual(coarse.x, 0.0, places=7)
        self.assertAlmostEqual(coarse.y, 1.0, places=7)
        self.assertAlmostEqual(coarse.yaw, 0.2 + math.pi / 2.0, places=7)


if __name__ == '__main__':
    unittest.main()
