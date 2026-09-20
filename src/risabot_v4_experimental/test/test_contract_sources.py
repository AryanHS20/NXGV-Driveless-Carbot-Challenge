import unittest

import cv2
import numpy as np

from risabot_v4_experimental.arbitration_core import select_diagnostic_intent
from risabot_v4_experimental.bev_core import CameraProfile, metric_to_bev
from risabot_v4_experimental.parking_goal_core import detect_parking_goal
from risabot_v4_experimental.recovery_policy_core import (
    build_recovery_request,
    dashboard_state,
)


def profile():
    return CameraProfile(
        name='slot_test', calibrated=True, resolution=(100, 100),
        camera_matrix=np.eye(3), distortion=np.zeros(5),
        source_points_px=np.array([[0, 0], [99, 0], [99, 99], [0, 99]], np.float64),
        ground_points_m=np.array([[-1, -1], [1.5, -1], [1.5, 1], [-1, 1]], np.float64),
        forward_bounds_m=(-1, 1.5), left_bounds_m=(-1, 1), pixels_per_meter=100,
    )


class ContractSourceTests(unittest.TestCase):
    def test_bright_closed_marking_produces_measured_goal(self):
        camera = profile()
        width, height = camera.output_size
        image = np.zeros((height, width, 3), np.uint8)
        coverage = np.full((height, width), 255, np.uint8)
        corners_m = np.array([[0.15, -0.35], [0.85, -0.35],
                              [0.85, 0.12], [0.15, 0.12]], np.float64)
        corners_px = np.rint(metric_to_bev(camera, corners_m)).astype(np.int32)
        cv2.polylines(image, [corners_px], True, (255, 255, 255), 5)
        goal = detect_parking_goal(image, coverage, camera, 12.5)
        self.assertIsNotNone(goal)
        self.assertAlmostEqual(goal.slot_length_m, 0.70, delta=0.08)
        self.assertAlmostEqual(goal.slot_width_m, 0.47, delta=0.08)
        self.assertAlmostEqual(goal.target.x, 0.50, delta=0.05)

    def test_unobserved_marking_is_not_a_goal(self):
        camera = profile()
        width, height = camera.output_size
        image = np.full((height, width, 3), 255, np.uint8)
        coverage = np.zeros((height, width), np.uint8)
        self.assertIsNone(detect_parking_goal(image, coverage, camera, 1.0))

    def test_recovery_policy_requires_exhausted_paths_and_allowed_state(self):
        exhausted = {'algorithm_stage': 4, 'selected_diagnostic_only': None,
                     'candidates': [{'valid': False}]}
        request = build_recovery_request(exhausted, 'LANE_RECOVERY|1|2', 0.0, 3.0, 0)
        self.assertTrue(request.permitted)
        self.assertTrue(request.stopped)
        self.assertEqual(dashboard_state('TRAFFIC_LIGHT|1'), 'TRAFFIC_LIGHT')
        held = build_recovery_request(exhausted, 'TRAFFIC_LIGHT|1', 0.0, 3.0, 0)
        self.assertFalse(held.permitted)
        self.assertEqual(held.hard_hold, 'TRAFFIC_LIGHT')
        self.assertIsNone(build_recovery_request(
            {**exhausted, 'candidates': [{'valid': True}]}, 'LANE_RECOVERY', 0.0, 3.0, 0))

    def test_arbitration_holds_override_all_proposals(self):
        valid = {'selected_diagnostic_only': {'valid': True, 'id': 1}}
        decision = select_diagnostic_intent('EMERGENCY_STOP|1', True, valid, valid, valid)
        self.assertEqual((decision.source, decision.action), ('hold', 'stop'))
        decision = select_diagnostic_intent('LANE_FOLLOW|1', False, valid, valid, valid)
        self.assertEqual((decision.source, decision.action), ('hold', 'stop'))

    def test_arbitration_selects_only_state_appropriate_source(self):
        forward = {'selected_diagnostic_only': {'valid': True, 'id': 2}}
        parking = {'selected_diagnostic_only': {'valid': True, 'family': 'LSR'}}
        recovery = {'selected_diagnostic_only': {'valid': True, 'family': 'RLR'}}
        self.assertEqual(select_diagnostic_intent(
            'LANE_FOLLOW|1', True, forward, parking, recovery).source, 'trajectory')
        self.assertEqual(select_diagnostic_intent(
            'ROUNDABOUT|1', True, forward, parking, recovery).source, 'trajectory')
        self.assertEqual(select_diagnostic_intent(
            'HILL|1', True, forward, parking, recovery).source, 'trajectory')
        self.assertEqual(select_diagnostic_intent(
            'PARALLEL_PARK|2', True, forward, parking, recovery).source, 'parking')
        self.assertEqual(select_diagnostic_intent(
            'LANE_RECOVERY|1', True, {}, parking, recovery).source, 'recovery')

    def test_lane_recovery_never_falls_through_to_forward_trajectory(self):
        forward = {'selected_diagnostic_only': {'valid': True, 'id': 2}}
        decision = select_diagnostic_intent(
            'LANE_RECOVERY|1', True, forward, {}, {})
        self.assertEqual((decision.source, decision.action), ('hold', 'stop'))


if __name__ == '__main__':
    unittest.main()
