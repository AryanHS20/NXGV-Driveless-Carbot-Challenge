import math
import unittest

import cv2
import numpy as np

from risabot_v4_experimental.bev_core import CameraProfile, metric_to_bev
from risabot_v4_experimental.trajectory_core import (
    PathPoint,
    TrajectoryConfig,
    VehicleGeometry,
    footprint_points,
    generate_candidates,
    reference_from_corridor,
    road_support,
)


def profile():
    return CameraProfile(
        name='test', calibrated=True, resolution=(100, 100),
        camera_matrix=np.eye(3), distortion=np.zeros(5),
        source_points_px=np.array([[0, 0], [99, 0], [99, 99], [0, 99]], np.float64),
        ground_points_m=np.array([[0, -0.5], [1, -0.5], [1, 0.5], [0, 0.5]], np.float64),
        forward_bounds_m=(-0.10, 1.00), left_bounds_m=(-0.50, 0.50),
        pixels_per_meter=100.0,
    )


def corridor_mask(camera_profile, half_width=0.22):
    width, height = camera_profile.output_size
    mask = np.zeros((height, width), np.uint8)
    polygon = metric_to_bev(camera_profile, np.array([
        [-0.10, -half_width], [1.00, -half_width],
        [1.00, half_width], [-0.10, half_width],
    ], np.float64))
    cv2.fillConvexPoly(mask, np.rint(polygon).astype(np.int32), 255)
    return mask


class TrajectoryCoreTests(unittest.TestCase):
    def setUp(self):
        self.profile = profile()
        self.reference = reference_from_corridor([(x, 0.0) for x in np.linspace(0.03, 0.90, 30)])

    def test_straight_corridor_selects_valid_center_candidate(self):
        candidates = generate_candidates(self.reference, corridor_mask(self.profile), self.profile)
        self.assertTrue(candidates[0].valid)
        self.assertEqual(candidates[0].offset_m, 0.0)
        self.assertEqual(len(candidates), 9)

    def test_curvature_never_exceeds_minimum_radius(self):
        candidates = generate_candidates(self.reference, corridor_mask(self.profile, 0.35), self.profile)
        limit = 1.0 / VehicleGeometry().minimum_turn_radius_m
        self.assertLessEqual(max(abs(k) for c in candidates for k in c.curvatures), limit + 1e-12)

    def test_narrow_corridor_rejects_full_body(self):
        candidates = generate_candidates(self.reference, corridor_mask(self.profile, 0.07), self.profile)
        self.assertFalse(any(candidate.valid for candidate in candidates))
        self.assertTrue(all(candidate.road_blocked > 0 for candidate in candidates))

    def test_obstacle_rejects_intersecting_rollouts(self):
        candidates = generate_candidates(
            self.reference, corridor_mask(self.profile, 0.35), self.profile,
            obstacles_m=[(0.30, 0.0)],
        )
        self.assertTrue(any(candidate.obstacle_blocked > 0 for candidate in candidates))
        center = next(candidate for candidate in candidates if candidate.offset_m == 0.0)
        self.assertFalse(center.valid)

    def test_corner_outside_mask_reduces_support(self):
        mask = corridor_mask(self.profile, 0.11)
        points = footprint_points(PathPoint(0.1, 0.03, math.radians(20.0)), VehicleGeometry(), 0.02)
        self.assertLess(road_support(points, mask, self.profile), 1.0)

    def test_selected_candidate_is_minimum_cost_among_valid(self):
        candidates = generate_candidates(self.reference, corridor_mask(self.profile, 0.35), self.profile)
        valid_costs = [candidate.cost for candidate in candidates if candidate.valid]
        self.assertEqual(candidates[0].cost, min(valid_costs))


if __name__ == '__main__':
    unittest.main()
