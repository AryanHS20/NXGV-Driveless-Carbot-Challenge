import math
import unittest

import cv2
import numpy as np

from risabot_v4_experimental.bev_core import CameraProfile, metric_to_bev
from risabot_v4_experimental.parking_core import (
    ParkingConfig,
    ParkingError,
    goal_fits_slot,
    parking_goal_from_mapping,
    plan_parking,
    reeds_shepp_candidates,
)
from risabot_v4_experimental.trajectory_core import PathPoint, VehicleGeometry


def profile():
    return CameraProfile(
        name='parking_test', calibrated=True, resolution=(100, 100),
        camera_matrix=np.eye(3), distortion=np.zeros(5),
        source_points_px=np.array(
            [[0, 0], [99, 0], [99, 99], [0, 99]], np.float64
        ),
        ground_points_m=np.array(
            [[-1.0, -1.0], [1.5, -1.0], [1.5, 1.0], [-1.0, 1.0]],
            np.float64,
        ),
        forward_bounds_m=(-1.0, 1.5), left_bounds_m=(-1.0, 1.0),
        pixels_per_meter=100.0,
    )


def road_mask(camera_profile, half_width=0.8):
    width, height = camera_profile.output_size
    mask = np.zeros((height, width), np.uint8)
    polygon = metric_to_bev(camera_profile, np.array([
        [-1.0, -half_width], [1.5, -half_width],
        [1.5, half_width], [-1.0, half_width],
    ], np.float64))
    cv2.fillConvexPoly(mask, np.rint(polygon).astype(np.int32), 255)
    return mask


class ParkingCoreTests(unittest.TestCase):
    def setUp(self):
        self.profile = profile()
        self.geometry = VehicleGeometry()
        self.start = PathPoint(0.0, 0.0, 0.0)

    def test_straight_forward_connection_reaches_exact_goal(self):
        goal = PathPoint(0.60, 0.0, 0.0)
        candidates = reeds_shepp_candidates(self.start, goal, 0.40)
        self.assertTrue(candidates)
        self.assertAlmostEqual(candidates[0].path_length_m, 0.60, places=7)
        self.assertLessEqual(candidates[0].endpoint_position_error_m, 1e-5)
        self.assertLessEqual(candidates[0].endpoint_yaw_error_rad, 1e-5)

    def test_reverse_connection_is_generated_for_goal_behind(self):
        goal = PathPoint(-0.30, 0.0, 0.0)
        candidates = reeds_shepp_candidates(self.start, goal, 0.40)
        self.assertTrue(any(candidate.reverse_distance_m >= 0.299 for candidate in candidates))
        self.assertTrue(any(point.direction == -1 for point in candidates[0].points))

    def test_turning_connections_respect_curvature_and_endpoint(self):
        goal = PathPoint(0.30, 0.30, math.pi / 2.0)
        candidates = reeds_shepp_candidates(self.start, goal, 0.40)
        self.assertTrue(candidates)
        for candidate in candidates:
            self.assertLessEqual(
                max(abs(point.curvature) for point in candidate.points),
                1.0 / 0.40 + 1e-12,
            )
            self.assertLessEqual(candidate.endpoint_position_error_m, 1e-5)
            self.assertLessEqual(candidate.endpoint_yaw_error_rad, 1e-5)

    def test_representative_pose_pairs_have_analytic_connections(self):
        goals = (
            PathPoint(-0.5, -0.4, -2.4),
            PathPoint(-0.2, 0.6, 2.8),
            PathPoint(0.0, -0.5, -math.pi / 2.0),
            PathPoint(0.2, 0.4, math.pi / 2.0),
            PathPoint(0.8, -0.2, 0.4),
        )
        for goal in goals:
            candidates = reeds_shepp_candidates(self.start, goal, 0.40)
            self.assertTrue(candidates, msg=f'no connection to {goal}')
            keys = {
                (candidate.family, tuple(round(x, 8) for x in candidate.segment_lengths_m))
                for candidate in candidates
            }
            self.assertEqual(len(keys), len(candidates))

    def test_parallel_plan_prefers_checked_reverse_docking(self):
        goal = PathPoint(0.55, 0.0, 0.0)
        candidates = plan_parking(
            self.start, goal, road_mask(self.profile), self.profile,
            self.geometry, require_reverse=True,
        )
        self.assertTrue(candidates[0].valid)
        self.assertGreater(candidates[0].reverse_distance_m, 0.0)
        self.assertIn('docking straight', candidates[0].stage)

    def test_narrow_observed_area_rejects_swept_body(self):
        goal = PathPoint(0.55, 0.0, 0.0)
        candidates = plan_parking(
            self.start, goal, road_mask(self.profile, half_width=0.07),
            self.profile, self.geometry,
        )
        self.assertFalse(any(candidate.valid for candidate in candidates))
        self.assertTrue(all(candidate.road_blocked_samples for candidate in candidates))

    def test_obstacle_intersection_rejects_candidates(self):
        goal = PathPoint(0.55, 0.0, 0.0)
        candidates = plan_parking(
            self.start, goal, road_mask(self.profile), self.profile,
            self.geometry, obstacles_m=[(0.30, 0.0)], require_reverse=True,
        )
        self.assertTrue(any(candidate.obstacle_blocked_samples for candidate in candidates))
        straight = [
            candidate for candidate in candidates
            if candidate.family.startswith('LSL') and candidate.road_blocked_samples == 0
        ]
        self.assertTrue(straight)
        self.assertTrue(all(not candidate.valid for candidate in straight))

    def test_slot_must_fit_complete_vehicle_with_clearance(self):
        self.assertTrue(goal_fits_slot(0.69, 0.47, self.geometry))
        self.assertFalse(goal_fits_slot(0.29, 0.47, self.geometry))
        self.assertFalse(goal_fits_slot(0.69, 0.19, self.geometry))

    def test_reverse_and_gear_limits_are_enforced(self):
        goal = PathPoint(-0.30, 0.0, 0.0)
        config = ParkingConfig(maximum_reverse_distance_m=0.20)
        candidates = plan_parking(
            self.start, goal, road_mask(self.profile), self.profile,
            self.geometry, config, require_reverse=True,
        )
        self.assertFalse(any(candidate.valid for candidate in candidates))
        self.assertTrue(any('reverse-distance' in candidate.reject_reason for candidate in candidates))

    def test_measured_goal_contract_rejects_wrong_frame_and_bad_kind(self):
        raw = {
            'kind': 'parallel', 'frame_id': 'base_link',
            'x_m': 0.4, 'y_m': -0.2, 'yaw_rad': 1.2,
            'slot_length_m': 0.69, 'slot_width_m': 0.47,
            'confidence': 0.9, 'image_stamp_sec': 12.5,
        }
        goal = parking_goal_from_mapping(raw)
        self.assertEqual(goal.kind, 'parallel')
        self.assertEqual(goal.frame_id, 'base_link')
        with self.assertRaises(ParkingError):
            parking_goal_from_mapping({**raw, 'frame_id': 'map'})
        with self.assertRaises(ParkingError):
            parking_goal_from_mapping({**raw, 'kind': 'diagonal'})


if __name__ == '__main__':
    unittest.main()
