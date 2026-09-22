import math
import unittest

from risabot_v4_control.control_core import (
    ControlContractError,
    dashboard_state,
    parse_path,
    path_command,
    proposal_contract,
    steering_from_curvature,
    trajectory_command,
    transform_path,
)


def path_payload(direction=1):
    points = [{'forward_m': 0.0, 'left_m': 0.0, 'yaw_rad': 0.0,
               'direction': direction, 'curvature_per_m': 0.0}]
    points.extend({
        'forward_m': 0.1 * i, 'left_m': 0.01 * i, 'yaw_rad': 0.01 * i,
        'direction': direction, 'curvature_per_m': 0.2,
    } for i in range(1, 6))
    return {'points': points}


class ControlCoreTests(unittest.TestCase):
    def test_dashboard_state_strips_mission_payload(self):
        self.assertEqual(dashboard_state('LANE_FOLLOW|2|3.2|'), 'LANE_FOLLOW')

    def test_left_positive_path_becomes_right_negative_control(self):
        self.assertLess(steering_from_curvature(1.0, 0.216, math.radians(50)), 0.0)
        self.assertGreater(steering_from_curvature(-1.0, 0.216, math.radians(50)), 0.0)

    def test_trajectory_command_is_bounded_and_slows_for_turn(self):
        straight = trajectory_command(
            {'valid': True, 'command_steer_rad_diagnostic_only': 0.0},
            .216, math.radians(50), .08, .4)
        turn = trajectory_command(
            {'valid': True, 'command_steer_rad_diagnostic_only': .5},
            .216, math.radians(50), .08, .4)
        self.assertEqual(straight.speed, .08)
        self.assertLess(turn.speed, straight.speed)
        self.assertGreaterEqual(turn.steering, -1.0)

    def test_turn_slowdown_is_adjustable_and_never_reaches_zero(self):
        reference = {'valid': True, 'command_steer_rad_diagnostic_only': .5}
        gentle = trajectory_command(
            reference, .216, math.radians(50), 65 / 255, 40 / 65, .4)
        strong = trajectory_command(
            reference, .216, math.radians(50), 65 / 255, 40 / 65, .85)
        self.assertLess(strong.speed, gentle.speed)
        self.assertGreaterEqual(strong.speed * 255, 40.0)

    def test_boundary_proximity_reduces_speed_without_stopping(self):
        near = trajectory_command({
            'valid': True, 'command_steer_rad_diagnostic_only': .1,
            'boundary_clearance_m': 0.0,
        }, .216, math.radians(50), 65 / 255, 40 / 65, .85, .03)
        clear = trajectory_command({
            'valid': True, 'command_steer_rad_diagnostic_only': .1,
            'boundary_clearance_m': .04,
        }, .216, math.radians(50), 65 / 255, 40 / 65, .85, .03)
        self.assertAlmostEqual(near.speed * 255, 40.0)
        self.assertGreater(clear.speed, near.speed)

    def test_invalid_or_nonfinite_contract_is_rejected(self):
        with self.assertRaises(ControlContractError):
            proposal_contract({'source': 'trajectory', 'action': 'drive', 'reference': {}})
        bad = path_payload()
        bad['points'][1]['curvature_per_m'] = float('nan')
        with self.assertRaises(ControlContractError):
            parse_path(bad)

    def test_transform_and_progress_stop_at_endpoint(self):
        points = parse_path(path_payload())
        world = transform_path(points, (1.0, 2.0, math.pi / 2))
        self.assertAlmostEqual(world[1].x, .99, places=6)
        self.assertAlmostEqual(world[1].y, 2.1, places=6)
        decision, index, complete = path_command(
            world, (world[-1].x, world[-1].y, world[-1].yaw), 0,
            .216, math.radians(50), .06, .05, .04)
        self.assertTrue(complete)
        self.assertEqual(decision.speed, 0.0)
        self.assertGreaterEqual(index, len(world) - 2)

    def test_reverse_path_requests_negative_speed(self):
        points = transform_path(parse_path(path_payload(-1)), (0.0, 0.0, 0.0))
        decision, _, complete = path_command(
            points, (0.0, 0.0, 0.0), 0, .216, math.radians(50), .06, .05, .02)
        self.assertFalse(complete)
        self.assertLess(decision.speed, 0.0)


if __name__ == '__main__':
    unittest.main()
