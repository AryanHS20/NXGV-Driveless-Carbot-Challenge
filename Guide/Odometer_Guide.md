# RISA-Bot Odometry Guide

This guide explains how odometry (distance and position tracking) is implemented on RISA-Bot in ROS 2.

1. Hardware odometry (primary): reads encoder ticks from Rosmaster.
2. Software odometry (fallback/display): integrates commanded velocity.

## 1. Hardware Odometry (Primary)

Primary odometry is produced by `servo_controller.py` and published on `/odom` (`nav_msgs/msg/Odometry`).

Current implementation includes:

- Tick delta integration at 20 Hz.
- Encoder jump rejection.
- Ackermann yaw from steering command.
- Velocity low-pass filtering.
- Runtime calibration parameters.

Key runtime parameters on `/servo_controller`:

- `ticks_per_meter`
- `odom_distance_scale`
- `odom_yaw_scale`
- `encoder_jump_threshold`
- `max_linear_velocity`
- `max_angular_velocity`
- `wheel_base`
- `steering_max_deg`
- `odom_vel_alpha`
- `odom_velocity_deadband`

## 2. Software Odometry (Fallback)

Software odometry is open-loop and assumes the robot moves exactly as commanded (`/cmd_vel`).

It is useful when:

- Encoders are unavailable.
- You want quick testing without hardware calibration.

It is less accurate under slip, battery sag, and varying friction.

## 3. Calibration Procedure

### Distance calibration

1. Reset odometry.
2. Drive exactly `1.00 m` straight.
3. Record reported distance `D_reported`.
4. Update one of the following:

```text
new_odom_distance_scale = old_odom_distance_scale * (1.00 / D_reported)
```

or

```text
new_ticks_per_meter = old_ticks_per_meter * (D_reported / 1.00)
```

### Yaw calibration

1. Command a known turn (for example 90 degrees).
2. Measure reported yaw.
3. Update:

```text
new_odom_yaw_scale = old_odom_yaw_scale * (target_yaw / reported_yaw)
```

## 4. Recommended Tuning Order

1. Set `encoder_jump_threshold` to remove rare spikes.
2. Tune `ticks_per_meter` or `odom_distance_scale` for straight distance.
3. Tune `odom_yaw_scale` for turn accuracy.
4. Tune `odom_vel_alpha` for smoothness versus responsiveness.
5. Tune `odom_velocity_deadband` to suppress standstill drift.

## 5. Runtime Commands

```bash
ros2 param set /servo_controller odom_distance_scale 0.62
ros2 param set /servo_controller ticks_per_meter 1700.0
ros2 param set /servo_controller odom_yaw_scale 0.95
ros2 param set /servo_controller encoder_jump_threshold 600.0
ros2 param set /servo_controller max_linear_velocity 0.80
ros2 param set /servo_controller odom_velocity_deadband 0.03
```

## 6. Dashboard Note

`dashboard` also exposes visualization-only scales:

- `sim_odom_scale`
- `hw_odom_scale`
- `hw_odom_yaw_scale`

Use `servo_controller` parameters for true robot behavior. Use dashboard scales only for display fallback.
