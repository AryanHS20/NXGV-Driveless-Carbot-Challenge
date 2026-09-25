import math
import unittest

import cv2
import numpy as np

from risabot_v4_experimental.bev_core import CameraProfile, metric_to_bev
from risabot_v4_experimental.trajectory_core import (
    PathPoint,
    TrajectoryConfig,
    TrajectoryError,
    VehicleGeometry,
    centerline_steering_command,
    enforce_inward_boundary_steer,
    evaluate_steering_command,
    footprint_points,
    generate_candidates,
    near_field_bootstrap_from_corridor,
    reference_from_corridor,
    road_support,
    smooth_centerline_reference,
    steering_reference_from_corridor,
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

    def test_sent_steering_rollout_tracks_the_command_and_checks_obstacles(self):
        mask = corridor_mask(self.profile, 0.35)
        left = evaluate_steering_command(0.30, mask, self.profile)
        right = evaluate_steering_command(-0.30, mask, self.profile)
        self.assertGreater(left.points[-1].y, 0.0)
        self.assertLess(right.points[-1].y, 0.0)
        self.assertAlmostEqual(left.points[-1].y, -right.points[-1].y, places=5)
        blocked = evaluate_steering_command(
            0.30, mask, self.profile, obstacles_m=[(0.30, 0.05)]
        )
        self.assertGreater(blocked.obstacle_blocked, 0)
        with self.assertRaises(TrajectoryError):
            evaluate_steering_command(math.nan, mask, self.profile)

    def test_corner_outside_mask_reduces_support(self):
        mask = corridor_mask(self.profile, 0.11)
        points = footprint_points(PathPoint(0.1, 0.03, math.radians(20.0)), VehicleGeometry(), 0.02)
        self.assertLess(road_support(points, mask, self.profile), 1.0)

    def test_selected_candidate_is_minimum_cost_among_valid(self):
        candidates = generate_candidates(self.reference, corridor_mask(self.profile, 0.35), self.profile)
        valid_costs = [candidate.cost for candidate in candidates if candidate.valid]
        self.assertEqual(candidates[0].cost, min(valid_costs))

    def test_bounded_near_field_bootstrap_covers_camera_blind_strip(self):
        mask = corridor_mask(self.profile, 0.22)
        # Reproduce a camera whose connected road begins well ahead of the
        # rear-axle origin: erase every observed road pixel below 0.45 m.
        blind = metric_to_bev(self.profile, np.array([[0.45, 0.0]], np.float64))[0]
        mask[int(round(blind[1])) + 1:, :] = 0
        samples = [(0.45 + 0.04 * i, 0.0, 0.44) for i in range(8)]
        bootstrap = near_field_bootstrap_from_corridor(
            samples, VehicleGeometry(), maximum_gap_m=0.55, settle_m=0.04)
        without = generate_candidates(self.reference, mask, self.profile)
        with_bootstrap = generate_candidates(
            self.reference, mask, self.profile, near_field=bootstrap)
        self.assertFalse(any(candidate.valid for candidate in without))
        self.assertTrue(any(candidate.valid for candidate in with_bootstrap))

    def test_near_field_bootstrap_rejects_unbounded_gap_or_narrow_road(self):
        with self.assertRaises(TrajectoryError):
            near_field_bootstrap_from_corridor(
                [(0.70, 0.0, 0.44), (0.75, 0.0, 0.44)],
                VehicleGeometry(), maximum_gap_m=0.55, settle_m=0.04)
        with self.assertRaises(TrajectoryError):
            near_field_bootstrap_from_corridor(
                [(0.45, 0.0, 0.15), (0.49, 0.0, 0.15)],
                VehicleGeometry(), maximum_gap_m=0.55, settle_m=0.04)

    def test_track_gain_strengthens_both_directions_before_rollout(self):
        for offset in (-0.05, 0.05):
            reference = reference_from_corridor([(x, offset) for x in np.linspace(0.45, 0.90, 16)])
            mask = corridor_mask(self.profile, 0.35)
            base = generate_candidates(reference, mask, self.profile)[0]
            stronger = generate_candidates(reference, mask, self.profile,
                                           config=TrajectoryConfig(steering_gain=2.0))[0]
            self.assertGreater(abs(stronger.command_steer_rad), abs(base.command_steer_rad) * 1.8)
            self.assertGreater(stronger.command_steer_rad * offset, 0)
            self.assertGreater(abs(stronger.curvatures[0]), abs(base.curvatures[0]))

    def test_single_observed_edge_reconstructs_32cm_lane_center(self):
        samples = [
            {
                'forward_m': forward, 'left_m': 0.055, 'width_m': 0.21,
                'boundaries_observed': False,
                'left_boundary_observed': True,
                'right_boundary_observed': False,
            }
            for forward in (0.35, 0.45, 0.55)
        ]
        reference = steering_reference_from_corridor(samples, 0.32)
        self.assertEqual(len(reference), 3)
        self.assertTrue(all(abs(point.y) < 1e-9 for point in reference))

        for sample in samples:
            sample['left_m'] = -0.055
            sample['left_boundary_observed'] = False
            sample['right_boundary_observed'] = True
        reference = steering_reference_from_corridor(samples, 0.32)
        self.assertTrue(all(abs(point.y) < 1e-9 for point in reference))

    def test_painted_boundary_position_controls_single_edge_center(self):
        samples = [
            {'forward_m': x, 'left_m': 0.0, 'width_m': 0.20,
             'left_boundary_observed': True,
             'right_boundary_observed': False,
             'left_boundary_m': 0.155}
            for x in (0.40, 0.50, 0.60)
        ]
        reference = steering_reference_from_corridor(samples, 0.31)
        self.assertTrue(all(abs(point.y) < 1e-9 for point in reference))

    def test_green_bend_does_not_join_opposite_one_sided_edges_across_gap(self):
        # Measured rows from the Risabot 1 slow manual lap at 120 s. The
        # right-only near lane was followed by floor-connected mask pixels,
        # then a different left-only edge. Joining them drew the guide onto
        # the white line and pointed the car away from the first bend.
        samples = [
            {'forward_m': .45, 'left_m': .085, 'width_m': .125,
             'left_boundary_observed': False, 'right_boundary_observed': True,
             'right_boundary_m': .020},
            {'forward_m': .61, 'left_m': .115, 'width_m': .225,
             'left_boundary_observed': False, 'right_boundary_observed': True,
             'right_boundary_m': -.005},
            {'forward_m': .77, 'left_m': .100, 'width_m': .405,
             'left_boundary_observed': False, 'right_boundary_observed': True,
             'right_boundary_m': -.105},
            {'forward_m': .93, 'left_m': .0025, 'width_m': .750,
             'left_boundary_observed': False, 'right_boundary_observed': False},
            {'forward_m': 1.09, 'left_m': -.240, 'width_m': .435,
             'left_boundary_observed': True, 'right_boundary_observed': False,
             'left_boundary_m': -.015},
            {'forward_m': 1.25, 'left_m': -.435, 'width_m': .205,
             'left_boundary_observed': True, 'right_boundary_observed': False,
             'left_boundary_m': -.325},
        ]
        reference = steering_reference_from_corridor(samples, .295)
        self.assertEqual(len(reference), 3)
        self.assertAlmostEqual(reference[0].y, .1675)
        self.assertLess(reference[-1].x, 1.0)

    def test_transverse_paint_cannot_define_a_narrow_two_sided_lane(self):
        samples = [
            {'forward_m': .4, 'left_m': 0., 'width_m': .295,
             'left_boundary_observed': True, 'right_boundary_observed': True,
             'left_boundary_m': .1475, 'right_boundary_m': -.1475},
            {'forward_m': .5, 'left_m': .2, 'width_m': .155,
             'left_boundary_observed': True, 'right_boundary_observed': True,
             'left_boundary_m': .2775, 'right_boundary_m': .1225},
            {'forward_m': .6, 'left_m': 0., 'width_m': .295,
             'left_boundary_observed': True, 'right_boundary_observed': True,
             'left_boundary_m': .1475, 'right_boundary_m': -.1475},
        ]
        reference = steering_reference_from_corridor(samples, .295)
        self.assertEqual(len(reference), 2)
        self.assertTrue(all(abs(point.y) < 1e-9 for point in reference))

    def test_smoothed_centerline_controller_steers_toward_curve(self):
        straight = reference_from_corridor([
            (x, 0.0) for x in np.linspace(0.25, 0.75, 12)
        ])
        smoothed, coefficients = smooth_centerline_reference(straight)
        command, diagnostics = centerline_steering_command(
            smoothed, coefficients, VehicleGeometry(), 0.22, 1.1, 0.85, 0.9
        )
        self.assertAlmostEqual(command, 0.0, places=6)
        self.assertAlmostEqual(diagnostics['lateral_error_m'], 0.0, places=6)

        for direction in (-1.0, 1.0):
            curved = reference_from_corridor([
                (x, direction * (0.03 + 0.45 * (x - 0.25) ** 2))
                for x in np.linspace(0.25, 0.75, 12)
            ])
            smoothed, coefficients = smooth_centerline_reference(curved)
            command, diagnostics = centerline_steering_command(
                smoothed, coefficients, VehicleGeometry(), 0.22,
                1.1, 0.85, 0.9,
            )
            self.assertGreater(command * direction, 0.0)
            self.assertGreater(diagnostics['curvature_per_m'] * direction, 0.0)

    def test_near_lane_heading_is_not_reversed_by_distant_widening(self):
        # The green-bend recording showed a straight near corridor drifting
        # left, followed by a wide region around a painted crossing. Fitting
        # one quadratic through both made its near tangent point right.
        for direction in (-1.0, 1.0):
            observed = reference_from_corridor([
                (x, direction * (0.01 + 0.05 * (x - 0.5)))
                for x in np.linspace(0.49, 1.29, 21)
            ] + [
                (x, direction * (0.04 + 0.55 * (x - 1.29)))
                for x in np.linspace(1.33, 1.73, 11)
            ])
            fitted, coefficients = smooth_centerline_reference(observed, alpha=1.0)
            command, diagnostic = centerline_steering_command(
                fitted, coefficients, VehicleGeometry(), 0.22,
                0.75, 0.85, 0.9, near_reference=observed,
            )
            self.assertGreater(command * direction, 0.05)
            self.assertGreater(diagnostic['heading_error_rad'] * direction, 0.03)
            self.assertTrue(diagnostic['near_fit_used'])

    def test_centerline_coefficients_are_temporally_filtered(self):
        left = reference_from_corridor([(0.3, 0.04), (0.5, 0.04), (0.7, 0.04)])
        _, previous = smooth_centerline_reference(left)
        right = reference_from_corridor([(0.3, -0.04), (0.5, -0.04), (0.7, -0.04)])
        _, filtered = smooth_centerline_reference(right, previous, alpha=0.25)
        self.assertGreater(filtered[0], 0.0)

    def test_risabot1_feedback_corrects_large_offset_and_ignores_tiny_noise(self):
        geometry = VehicleGeometry(footprint_padding_m=0.010)
        reference = reference_from_corridor([(0.3, 0.12), (0.5, 0.12), (0.7, 0.12)])
        command, diagnostic = centerline_steering_command(
            reference, (0.12, 0.0, 0.0), geometry, 0.22,
            1.4, 0.85, 0.9, 0.32, 0.14, 0.02, 0.05, 0.15,
        )
        self.assertGreater(command, 0.0)
        self.assertAlmostEqual(diagnostic['control_lateral_error_m'], 0.12)
        self.assertFalse(diagnostic['near_center_guard_active'])
        reference = reference_from_corridor([(0.3, -0.015), (0.5, -0.015)])
        command, diagnostic = centerline_steering_command(
            reference, (-0.015, 0.0, 0.0), geometry, 0.22,
            1.4, 0.85, 0.9, 0.32, 0.14, 0.02, 0.05, 0.15,
        )
        self.assertEqual(command, 0.0)
        self.assertTrue(diagnostic['near_center_guard_active'])

    def test_blind_strip_intercept_does_not_force_early_turn(self):
        # Reconstructed from failed trial 20260922T161355Z: the polynomial
        # intercept was 28.7 cm left, while the visible lane at 41 cm was only
        # about 7.7 cm left. The unseen intercept must not dominate steering.
        coefficients = (0.2867, -0.591, 0.356)
        reference = reference_from_corridor([
            (x, coefficients[0] + coefficients[1] * x + coefficients[2] * x * x)
            for x in np.linspace(0.41, 0.75, 10)
        ])
        command, diagnostics = centerline_steering_command(
            reference, coefficients, VehicleGeometry(footprint_padding_m=0.010), 0.22,
            1.1, 0.85, 0.9,
        )
        self.assertLess(abs(diagnostics['lateral_error_m']), 0.12)
        self.assertLess(
            abs(diagnostics['lateral_error_m']),
            abs(diagnostics['intercept_lateral_error_m']) * 0.5,
        )
        self.assertLessEqual(abs(diagnostics['control_lateral_error_m']), 0.0541)
        self.assertLess(abs(command), 0.25)

    def test_boundary_recovery_keeps_steering_inward_when_heading_opposes_it(self):
        # A car right of centre has a positive leftward lane error. Heading
        # feedback can otherwise request a right turn near the white line.
        for side in (-1.0, 1.0):
            coefficients = (side * 0.275, -side * 0.45, 0.0)
            reference = reference_from_corridor([
                (x, coefficients[0] + coefficients[1] * x)
                for x in (0.50, 0.60, 0.70)
            ])
            command, diagnostics = centerline_steering_command(
                reference, coefficients, VehicleGeometry(), 0.22,
                1.4, 0.85, 0.9, 0.31, 0.14, 0.02, 0.05, 0.15,
                0.020, 0.10,
            )
            self.assertTrue(diagnostics['boundary_recovery_active'])
            self.assertGreaterEqual(command * side, 0.10 - 1e-9)

        centered = reference_from_corridor([(0.5, 0.0), (0.7, 0.0)])
        command, diagnostics = centerline_steering_command(
            centered, (0.0, 0.0, 0.0), VehicleGeometry(), 0.22,
            1.4, 0.85, 0.9, 0.31, 0.14, 0.02, 0.05, 0.15,
            0.020, 0.10,
        )
        self.assertEqual(command, 0.0)
        self.assertFalse(diagnostics['boundary_recovery_active'])

    def test_final_boundary_guard_overrides_outward_rate_limit_or_hold(self):
        # The green-bend AUTO bag switched from a prior hard right command
        # to a leftward boundary correction. The rate limiter still sent a
        # right command for several perception frames.
        self.assertEqual(
            enforce_inward_boundary_steer(-0.47, 0.14, 0.30, True), 0.30,
        )
        self.assertEqual(
            enforce_inward_boundary_steer(0.60, -0.14, 0.30, True), -0.30,
        )
        self.assertEqual(
            enforce_inward_boundary_steer(-0.47, 0.14, 0.30, False), -0.47,
        )

    def test_risabot1_right_stripe_trial_gets_substantial_left_correction(self):
        # The 20260922T225222Z AUTO recording had a 14.5 cm leftward lane
        # error while heading feedback opposed the needed correction.
        coefficients = (0.449, -0.907, 0.515)
        reference = reference_from_corridor([
            (x, coefficients[0] + coefficients[1] * x + coefficients[2] * x * x)
            for x in (0.45, 0.53, 0.61, 0.69)
        ])
        command, diagnostics = centerline_steering_command(
            reference, coefficients, VehicleGeometry(width_m=0.205), 0.22,
            0.75, 0.85, 0.90, 0.31, 0.14, 0.02, 0.05, 0.15,
            0.020, 0.30,
        )
        self.assertGreater(diagnostics['lateral_error_m'], 0.10)
        self.assertGreaterEqual(command, 0.30)

    def test_track_mode_relaxes_mask_support_without_hiding_observed_obstacles(self):
        mask = corridor_mask(self.profile, 0.07)
        candidates = generate_candidates(self.reference, mask, self.profile, enforce_road_support=False)
        self.assertTrue(any(c.valid for c in candidates))
        self.assertTrue(all(c.road_blocked > 0 for c in candidates))
        blocked = generate_candidates(self.reference, mask, self.profile,
                                      obstacles_m=[(0.0, 0.0)], enforce_road_support=False)
        self.assertFalse(any(c.valid for c in blocked))


if __name__ == '__main__':
    unittest.main()
