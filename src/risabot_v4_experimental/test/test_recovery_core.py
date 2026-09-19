import unittest

import cv2
import numpy as np

from risabot_v4_experimental.bev_core import CameraProfile, metric_to_bev
from risabot_v4_experimental.recovery_core import (
    RecoveryConfig,
    RecoveryError,
    forward_candidates_exhausted,
    plan_recovery,
    recovery_request_from_mapping,
    request_blockers,
    select_rejoin_goals,
)
from risabot_v4_experimental.trajectory_core import VehicleGeometry


def profile():
    return CameraProfile(
        name='recovery_test', calibrated=True, resolution=(100, 100),
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


def full_mask(camera_profile):
    width, height = camera_profile.output_size
    return np.full((height, width), 255, np.uint8)


def curved_corridor():
    return [
        (0.10, 0.10), (0.20, 0.08), (0.30, 0.05),
        (0.40, 0.02), (0.50, 0.00), (0.60, 0.00),
    ]


class RecoveryCoreTests(unittest.TestCase):
    def setUp(self):
        self.profile = profile()
        self.mask = full_mask(self.profile)
        self.geometry = VehicleGeometry()

    def test_selects_bounded_goals_along_corridor(self):
        goals = select_rejoin_goals(curved_corridor())
        self.assertGreaterEqual(len(goals), 2)
        self.assertLessEqual(len(goals), 5)
        self.assertGreaterEqual(goals[0].x, 0.20)

    def test_valid_recovery_is_reverse_then_forward_once(self):
        candidates = plan_recovery(
            curved_corridor(), self.mask, self.mask, self.profile, self.geometry
        )
        selected = next(candidate for candidate in candidates if candidate.valid)
        directions = []
        for point in selected.path.points:
            if not directions or directions[-1] != point.direction:
                directions.append(point.direction)
        self.assertEqual(directions, [-1, 1])
        self.assertEqual(selected.path.gear_changes, 1)
        self.assertGreaterEqual(selected.path.reverse_distance_m, 0.03)
        self.assertLessEqual(selected.path.reverse_distance_m, 0.20)
        self.assertLessEqual(selected.path.path_length_m, 1.60)

    def test_unknown_rear_area_rejects_every_reverse_path(self):
        unknown = np.zeros_like(self.mask)
        candidates = plan_recovery(
            curved_corridor(), self.mask, unknown, self.profile, self.geometry
        )
        self.assertFalse(any(candidate.valid for candidate in candidates))
        reverse = [candidate for candidate in candidates if candidate.path.reverse_distance_m]
        self.assertTrue(reverse)
        self.assertTrue(all(
            'rear coverage' in candidate.reject_reason for candidate in reverse
        ))

    def test_lidar_point_inside_swept_body_vetoes_candidate(self):
        baseline = plan_recovery(
            curved_corridor(), self.mask, self.mask, self.profile, self.geometry
        )
        chosen = next(candidate for candidate in baseline if candidate.valid)
        reverse_points = [point for point in chosen.path.points if point.direction == -1]
        obstacle = reverse_points[len(reverse_points) // 2]
        blocked = plan_recovery(
            curved_corridor(), self.mask, self.mask, self.profile, self.geometry,
            obstacles_m=[(obstacle.x, obstacle.y)],
        )
        self.assertTrue(any(
            candidate.path.obstacle_blocked_samples > 0 for candidate in blocked
        ))
        same_family = [
            candidate for candidate in blocked
            if candidate.path.family == chosen.path.family
            and candidate.goal_index == chosen.goal_index
        ]
        self.assertTrue(same_family)
        self.assertFalse(same_family[0].valid)

    def test_narrow_road_rejects_full_vehicle_footprint(self):
        mask = np.zeros_like(self.mask)
        polygon = metric_to_bev(self.profile, np.array([
            [-1.0, -0.06], [1.5, -0.06], [1.5, 0.06], [-1.0, 0.06],
        ], np.float64))
        cv2.fillConvexPoly(mask, np.rint(polygon).astype(np.int32), 255)
        candidates = plan_recovery(
            curved_corridor(), mask, self.mask, self.profile, self.geometry,
            RecoveryConfig(road_tolerance_m=0.0),
        )
        self.assertFalse(any(candidate.valid for candidate in candidates))
        self.assertTrue(any(candidate.path.road_blocked_samples for candidate in candidates))

    def test_hard_hold_and_attempt_limit_block_before_planning(self):
        request = recovery_request_from_mapping({
            'problem': 'no_forward_candidate', 'hard_hold': 'EMERGENCY_STOP',
            'permitted': True, 'stopped': True, 'attempts': 3,
            'image_stamp_sec': 2.0, 'frame_id': 'base_link',
        })
        blockers = request_blockers(request, RecoveryConfig())
        self.assertIn('hard hold active: EMERGENCY_STOP', blockers)
        self.assertIn('recovery attempt limit reached', blockers)

    def test_forward_status_must_prove_all_candidates_rejected(self):
        exhausted = {
            'algorithm_stage': 4, 'selected_diagnostic_only': None,
            'candidates': [{'valid': False}, {'valid': False}],
        }
        self.assertTrue(forward_candidates_exhausted(exhausted))
        self.assertFalse(forward_candidates_exhausted({
            **exhausted, 'candidates': [{'valid': False}, {'valid': True}],
        }))
        self.assertFalse(forward_candidates_exhausted({
            **exhausted, 'candidates': [],
        }))

    def test_request_contract_rejects_bad_frame_and_empty_problem(self):
        valid = {
            'problem': 'no_forward_candidate', 'hard_hold': '',
            'permitted': True, 'stopped': True, 'attempts': 0,
            'image_stamp_sec': 2.0, 'frame_id': 'base_link',
        }
        self.assertEqual(recovery_request_from_mapping(valid).attempts, 0)
        with self.assertRaises(RecoveryError):
            recovery_request_from_mapping({**valid, 'frame_id': 'map'})
        with self.assertRaises(RecoveryError):
            recovery_request_from_mapping({**valid, 'problem': ''})
        with self.assertRaises(RecoveryError):
            recovery_request_from_mapping({**valid, 'problem': 'traffic_light'})
        with self.assertRaises(RecoveryError):
            recovery_request_from_mapping({**valid, 'permitted': 'false'})
        with self.assertRaises(RecoveryError):
            recovery_request_from_mapping({**valid, 'attempts': 1.5})


if __name__ == '__main__':
    unittest.main()
