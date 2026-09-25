"""Shared, ROS-free settings for an operator-selected lane test."""

import math


def track_test_overrides(vehicle='risabot5', motor_duty=65.0,
                         steering_gain=2.8, minimum_turn_duty=48.0,
                         steering_slowdown_gain=0.85,
                         enable_reverse_recovery=False):
    """Keep physical motor duty distinct from the legacy speed-request units.

    Wheel angle/ranges are starting estimates, not new calibration claims.
    The centres below are the owner's initial settings. R1 at 110 drifts
    right in a forward-only MANUAL test; straight-running neutral is not
    validated yet. Calibrate it at the shared servo mapping before AUTO.
    """
    centers = {'risabot1': 110, 'risabot5': 80}
    if vehicle not in centers:
        raise ValueError('vehicle must be risabot1 or risabot5')
    if not math.isfinite(motor_duty) or not 0 < motor_duty <= 100:
        raise ValueError('motor_duty must be a percentage in (0, 100]')
    if not math.isfinite(steering_gain) or not 0 < steering_gain <= 4:
        raise ValueError('steering_gain must be in (0, 4]')
    if (not math.isfinite(minimum_turn_duty)
            or not 0 < minimum_turn_duty <= motor_duty):
        raise ValueError('minimum_turn_duty must be in (0, motor_duty]')
    if (not math.isfinite(steering_slowdown_gain)
            or steering_slowdown_gain < 0):
        raise ValueError('steering_slowdown_gain must be finite and nonnegative')
    duty_map = 255.0
    risabot1 = vehicle == 'risabot1'
    hill_duty = max(90.0, motor_duty) if risabot1 else motor_duty
    wheelbase = 0.21
    max_steer_deg = 50.0
    return {
        'auto_driver': {
            'track_test_mode': True, 'lane_readiness_source': 'v4',
        },
        'cmd_safety_controller': {
            'track_test_mode': True, 'autonomy_source': 'v4',
            'require_signage': False, 'max_linear_speed': hill_duty / duty_map,
            'max_angular_speed': 1.0, 'max_linear_accel': 0.8,
            'max_angular_accel': 8.0, 'deadband_angular': 0.0,
        },
        'servo_controller': {
            'allow_recorded_playback': not risabot1,
            'servo_center': centers[vehicle], 'motor_duty_per_mps': duty_map,
            'auto_motor_duty_limit': hill_duty, 'wheel_base': wheelbase,
            'steering_max_deg': max_steer_deg,
            # Manual reference laps use the full proportional stick range:
            # half stick is half duty, full stick is full duty.
            'default_speed_index': 4,
            # The old 1.3 boost was car-1 tuning. Keep the trajectory's
            # left/right angle model consistent on car 2 until measured.
            'auto_right_steer_boost': 1.0,
        },
        'v4_bev_shadow': {'enabled': True, 'process_secondary': False},
        'v4_road_mask_shadow': {
            'enabled': True, 'process_secondary': False,
            # Wheel odometry has not been calibrated on this chassis. Keep
            # its sign/scale errors out of camera-only lane centering.
            'use_road_memory': False,
        },
        'v4_pose_shadow': {'enabled': True},
        'v4_trajectory_shadow': {
            'lane_controller': 'live_lane_arc' if risabot1 else 'filtered_centerline',
            'enabled': True, 'track_test_mode': True, 'require_lidar': False,
            'road_timeout_sec': 0.35,
            # The R1 LiDAR bag contains persistent 5-8 cm returns from the
            # chassis/mount. Those points overlap every predicted footprint
            # and halted the car despite a visible lane. The independent
            # tunnel wall follower keeps its own unmodified scan.
            'minimum_scan_range_m': 0.12 if risabot1 else 0.03,
            'wheelbase_m': wheelbase,
            # Risabot 1 measurements rechecked on 2026-09-23: 27.5 cm long,
            # 18.5 cm across the outside of the rear tires, 29.5 cm dark strip.
            'vehicle_length_m': 0.275 if risabot1 else 0.300,
            'vehicle_width_m': 0.185 if risabot1 else 0.192,
            # Use the configured actuator range, rather than the unmeasured
            # 0.4 m radius that restricted steering to 55% of its range.
            'minimum_turn_radius_m': wheelbase / math.tan(math.radians(max_steer_deg)),
            'steering_gain': steering_gain,
            # Allow for measurement and localization error beyond tire width.
            'footprint_padding_m': 0.010,
            # Prefer clearance strongly but keep steering through a brief
            # border overlap instead of stopping outside a hard mask margin.
            'road_support_cost_weight': 100.0,
            # The centered R1 straight on 2026-09-23 had 0.9635 minimum
            # support for the final shallow steer because the near camera
            # edge clips a few footprint samples. Keep the road gate active
            # while allowing this measured mask tolerance.
            'minimum_road_support': 0.96 if risabot1 else 0.98,
            'expected_lane_width_m': 0.295 if risabot1 else 0.32,
            'centerline_filter_alpha': 0.60,
            # The 19.7 s corner capture showed a 6 cm error commanding about
            # 0.31 rad, then overshooting to the opposite white line. Dampen
            # routine centering while the boundary guard below keeps the
            # command pointed inward near a line.
            'cross_track_gain': 0.75 if risabot1 else 1.10,
            'max_cross_track_feedback_m': 0.14 if risabot1 else 0.0,
            # The 2026-09-23 green-bend bag shows this 2 cm dead zone holding
            # steering at exactly zero for 1.6 s while the car yawed right.
            'near_center_guard_m': 0.0,
            'near_heading_guard_rad': 0.05 if risabot1 else 0.0,
            'near_curvature_guard_per_m': 0.15 if risabot1 else 0.0,
            'boundary_recovery_error_m': 0.020 if risabot1 else 0.0,
            # The latest R1 trial reached the right stripe while the old
            # 0.10 rad inward floor yielded only about six degrees of turn.
            'boundary_recovery_steer_rad': 0.30 if risabot1 else 0.0,
            'heading_gain': 0.85,
            'curvature_feedforward_gain': 0.90,
            'reliable_support_threshold': 0.75,
            'minimum_observed_centerline_fraction': 0.50,
            'low_support_direction_hold_sec': 2.50,
            'low_support_steer_decay_sec': 1.50,
            # The previous 0.8 rad/s cap took roughly 0.6 s to reverse a
            # correction; the camera showed the car crossing the lane then.
            'steering_rate_rad_sec': 2.0 if risabot1 else 0.80,
            # R1's saved bend trial selected paths with no observed road
            # support. Until the mask is repaired, reject those commands and
            # stop on a bad frame rather than replaying a held steering plan.
            'plan_hold_sec': 0.0,
            'enforce_road_support_in_track_test': True,
            # Keep remembered near-field pixels from suddenly shortening the
            # target to 9.5 cm once the car begins moving.
            'lookahead_m': 0.22,
        },
        'v4_arbitration_shadow': {
            'enabled': True, 'track_test_mode': True, 'lane_only': True,
        },
        'v4_motion_executor': {
            'enabled': True, 'track_test_mode': True,
            'allow_legacy_challenge_passthrough': False,
            'forward_motor_duty_percent': motor_duty,
            'motor_duty_per_mps': duty_map,
            'wheelbase_m': wheelbase, 'maximum_steer_deg': max_steer_deg,
            'minimum_speed_scale': minimum_turn_duty / motor_duty,
            'steering_slowdown_gain': steering_slowdown_gain,
            'boundary_slowdown_margin_m': 0.03,
            'enable_boundary_reverse_recovery': bool(enable_reverse_recovery),
            'enable_tunnel_follow': risabot1,
            'tunnel_motor_duty_percent': min(motor_duty, 55.0),
            'hill_boost_motor_duty_percent': hill_duty,
        },
    }
