"""Shared, ROS-free settings for an operator-selected lane test."""

import math


def track_test_overrides(vehicle='risabot5', motor_duty=65.0,
                         steering_gain=2.8, minimum_turn_duty=48.0,
                         steering_slowdown_gain=0.85,
                         enable_reverse_recovery=False, target_speed_mps=0.0):
    """Keep physical motor duty distinct from the legacy speed-request units.

    Wheel angle/ranges are starting estimates, not new calibration claims.
    Only the servo centres below have been confirmed by the owner.
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
    if not math.isfinite(target_speed_mps) or not 0 <= target_speed_mps <= 0.4:
        raise ValueError('target_speed_mps must be in [0, 0.4]; zero selects legacy duty mode')
    if target_speed_mps and enable_reverse_recovery:
        raise ValueError('encoder speed mode is forward-only')
    request = motor_duty / duty_map
    wheelbase = 0.21
    max_steer_deg = 50.0
    return {
        'auto_driver': {
            'track_test_mode': True, 'lane_readiness_source': 'v4',
        },
        'cmd_safety_controller': {
            # Operator explicitly requested continuous supervised AUTO;
            # manual takeover replaces the diagnostic time cutoff.
            'auto_trial_limit_sec': 0.0,
            'track_test_mode': True, 'autonomy_source': 'v4',
            'require_signage': False, 'max_linear_speed': request,
            # Applies only to an explicitly selected named parking recording;
            # ordinary AUTO lane duty remains motor_duty/255.
            'recorded_parking_max_linear_speed': 100.0 / duty_map,
            'max_angular_speed': 1.0, 'max_linear_accel': 0.8,
            'max_angular_accel': 8.0, 'deadband_angular': 0.0,
        },
        'servo_controller': {
            'servo_center': centers[vehicle], 'motor_duty_per_mps': duty_map,
            'auto_motor_duty_limit': motor_duty, 'wheel_base': wheelbase,
            'recorded_parking_duty_limit': 100.0,
            'parallel_recording': ('parking_movement_20260925_1602'
                                   if vehicle == 'risabot5' else ''),
            'perpendicular_recording': ('perpendicular_practice_20260925_0212'
                                        if vehicle == 'risabot5' else ''),
            'steering_max_deg': max_steer_deg,
            # R5's forward wheel rotation publishes negative raw /odom with
            # the uncorrected encoder polarity (saved 2026-09-24 forward run).
            # Correct it at the source so every odometry consumer agrees.
            'odom_reverse_polarity': vehicle == 'risabot5',
            'publish_encoder_diagnostics': vehicle == 'risabot5',
            # R5 MANUAL trace: ~97k raw ticks over an observed 0.30-0.40 m.
            # Use the midpoint until a measured straight run refines it.
            'ticks_per_meter': 280000.0 if vehicle == 'risabot5' else 1050.0,
            # Legitimate R5 deltas exceeded 2.9k/50 ms; the generic 800
            # threshold discarded 35 selected-channel samples in one move.
            'encoder_jump_threshold': 15000.0 if vehicle == 'risabot5' else 800.0,
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
            # +/-2 units were commanded in the attended run but the front
            # wheels did not visibly turn. +/-5 moved in the lifted test.
            'minimum_servo_step_units': 5 if vehicle == 'risabot5' else 2,
            'servo_range_left_units': 50,
            'servo_range_right_units': 70,
            'lane_controller': 'filtered_centerline',
            'enabled': True, 'track_test_mode': True, 'require_lidar': False,
            'use_lidar_obstacles': vehicle != 'risabot5',
            'wheelbase_m': wheelbase,
            # Use the configured actuator range, rather than the unmeasured
            # 0.4 m radius that restricted steering to 55% of its range.
            'minimum_turn_radius_m': wheelbase / math.tan(math.radians(max_steer_deg)),
            'steering_gain': steering_gain,
            # Keep the measured 19.2 cm body inside the dark lane surface.
            # The steering reference remains the midpoint between two observed
            # white boundaries.
            'footprint_padding_m': 0.010,
            # Prefer clearance strongly but keep steering through a brief
            # border overlap instead of stopping outside a hard mask margin.
            'road_support_cost_weight': 100.0,
            'expected_lane_width_m': 0.32,
            'centerline_filter_alpha': 0.60,
            # In the saved roundabout approach, far rows pulled the fitted
            # heading left while the near lane required a right correction.
            'centerline_fit_horizon_m': 0.85 if vehicle == 'risabot5' else 1.8,
            'left_branch_preview_enabled': vehicle == 'risabot5',
            'cross_track_gain': 1.10,
            'heading_gain': 0.85,
            'curvature_feedforward_gain': 0.90,
            'reliable_support_threshold': 0.75,
            'minimum_observed_centerline_fraction': 0.50,
            'low_support_direction_hold_sec': 2.50,
            'low_support_steer_decay_sec': 1.50,
            'steering_rate_rad_sec': 0.80,
            'plan_hold_sec': 0.60, 'road_timeout_sec': 1.10,
            'enforce_road_support_in_track_test': False,
            # Keep remembered near-field pixels from suddenly shortening the
            # target to 9.5 cm once the car begins moving.
            'lookahead_m': 0.22,
            **({'rollout_speed_mps': target_speed_mps} if target_speed_mps else {}),
        },
        'v4_arbitration_shadow': {
            'enabled': True, 'track_test_mode': True, 'lane_only': True,
        },
        'v4_motion_executor': {
            'enabled': True, 'track_test_mode': True,
            'encoder_speed_control': target_speed_mps > 0,
            # /odom is now forward-positive on R5; do not flip it a second time.
            'encoder_speed_sign': 1.0,
            'forward_speed_mps': target_speed_mps if target_speed_mps else 0.08,
            'allow_legacy_challenge_passthrough': False,
            'forward_motor_duty_percent': motor_duty,
            'motor_duty_per_mps': duty_map,
            'wheelbase_m': wheelbase, 'maximum_steer_deg': max_steer_deg,
            'minimum_speed_scale': minimum_turn_duty / motor_duty,
            'steering_slowdown_gain': steering_slowdown_gain,
            'boundary_slowdown_margin_m': 0.03,
            'enable_boundary_reverse_recovery': bool(enable_reverse_recovery),
        },
    }
