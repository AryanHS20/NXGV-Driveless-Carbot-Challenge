"""Live lane control: rank the same constant-steering arcs that are checked.

No recorded lap, global pose, learned driver commands, polynomial extrapolation,
or previous direction is an input. Positive metric steering means LEFT.
"""

import math

import numpy as np

from .trajectory_core import TrajectoryError, evaluate_steering_command


def select_live_lane_arc(reference, mask, profile, geometry, config,
                         obstacles=(), near_field=None, initial_steer=0.0):
    """Return (selected status or None, ranked diagnostics) for one live frame.

    Reuse the existing footprint/lag model and strict road gate. The unseen
    strip under the car still uses the existing bounded bootstrap; this is
    a model assumption, not measured free space. Replan on every fresh frame.
    """
    geometry.validate()
    config.validate()
    if len(reference) < 3:
        raise TrajectoryError('live lane needs at least three observed rows')
    xy = np.asarray([(p.x, p.y) for p in reference], dtype=float)
    if not np.isfinite(xy).all() or np.any(np.diff(xy[:, 0]) <= 0):
        raise TrajectoryError('live lane rows must be finite and forward ordered')
    # Never continue on a centreline wholly beyond the prediction horizon.
    if xy[0, 0] <= 0 or xy[0, 0] > config.horizon_m - 2 * config.step_m:
        raise TrajectoryError('live lane is too far away for this horizon')
    target_x = min(xy[-1, 0], max(config.lookahead_m, xy[0, 0]))
    target_y = float(np.interp(target_x, xy[:, 0], xy[:, 1]))
    maximum = math.atan(geometry.wheelbase_m / geometry.minimum_turn_radius_m)
    preferred = float(np.clip(math.atan2(
        2 * geometry.wheelbase_m * target_y, target_x**2 + target_y**2
    ), -maximum, maximum))
    # Include fine corrections around pure pursuit and broader bend options.
    alternatives = np.unique(np.clip(np.concatenate((
        np.linspace(-maximum, maximum, 17),
        preferred + np.asarray([-0.09, -0.045, -0.02, 0, 0.02, 0.045, 0.09]),
        [0.0],
    )), -maximum, maximum))
    # Pure pursuit is the normal controller. Search alternatives only when
    # its exact rollout fails, keeping ordinary camera updates inexpensive.
    angles = [preferred] + [float(a) for a in alternatives if abs(a - preferred) > 1e-9]
    near = xy[xy[:, 0] <= xy[0, 0] + 0.12]
    heading = math.atan2(near[-1, 1] - near[0, 1],
                         near[-1, 0] - near[0, 0]) if len(near) > 1 else 0.0
    lateral = float(xy[0, 1])
    front = geometry.length_m - geometry.rear_overhang_m + geometry.footprint_padding_m
    clearance = (config.expected_lane_width_m / 2 - geometry.width_m / 2
                 - geometry.footprint_padding_m - abs(lateral)
                 - front * abs(math.sin(heading)))
    candidates = []
    for index, angle in enumerate(angles):
        result = evaluate_steering_command(
            float(angle), mask, profile, geometry, config, obstacles,
            near_field, initial_steer_rad=initial_steer,
        )
        observed = [p for p in result.points if xy[0, 0] <= p.x <= xy[-1, 0]]
        reaches_observation = len(observed) >= 2
        errors = [p.y - float(np.interp(p.x, xy[:, 0], xy[:, 1])) for p in observed]
        # Rank by tracking the CURRENT observed lane; a feasible straight arc
        # is not automatically preferred when the visible lane bends.
        cost = (float(np.mean(np.square(errors))) if errors else 1e3)
        cost += 0.002 * (float(angle) - preferred)**2
        valid = (reaches_observation and result.road_blocked == 0
                 and result.obstacle_blocked == 0)
        end = result.points[-1]
        candidates.append({
            'id': index, 'offset_m': 0.0, 'valid': valid, 'cost': cost,
            'command_steer_rad_diagnostic_only': float(angle),
            'steering_source': 'live_lane_arc',
            'minimum_support': result.minimum_support,
            'road_blocked_steps': result.road_blocked,
            'obstacle_blocked_steps': result.obstacle_blocked,
            'sent_command_road_blocked_steps': result.road_blocked,
            'sent_command_obstacle_blocked_steps': result.obstacle_blocked,
            'sent_command_minimum_support': result.minimum_support,
            'endpoint_m': {'forward': end.x, 'left': end.y, 'yaw': end.yaw},
            'sent_command_endpoint_m': {'forward': end.x, 'left': end.y},
            'reject_reason': ('' if valid else 'arc lacks road support, hits obstacle, or cannot reach observed lane'),
            'target_forward_m': target_x, 'target_left_m': target_y,
            'preferred_steer_rad': preferred,
            'lateral_error_m': lateral, 'heading_error_rad': heading,
            'curvature_per_m': math.tan(float(angle)) / geometry.wheelbase_m,
            'boundary_clearance_m': clearance,
            'boundary_recovery_active': False,
            'preview_lateral_shift_m': float(np.interp(
                min(xy[-1, 0], xy[0, 0] + 1), xy[:, 0], xy[:, 1]) - lateral),
            'observed_centerline_fraction': 1.0,
            'path_shape': 'lane',
        })
        if index == 0 and valid:
            break
    candidates.sort(key=lambda c: (not c['valid'], c['cost']))
    return (dict(candidates[0]) if candidates[0]['valid'] else None), candidates
