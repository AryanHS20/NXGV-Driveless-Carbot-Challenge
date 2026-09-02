# Competition Validation Guide

This guide is for field testing the current `refactor-test` stack on hardware.

## 1. Which Launch to Use

- `competition.launch.py`: full run stack (camera, lidar, perception, auto driver, cmd safety, joystick, servo controller, dashboard, health monitor). Use this for real runs.
- `bringup.launch.py`: minimal debug stack (lidar, TF, auto driver, cmd safety, health monitor). Use this when you want to test nodes one-by-one or replay data.

Quick rule:
- Course run: `competition.launch.py`
- Debug/integration isolation: `bringup.launch.py`

## 2. Pre-Run Checklist

1. Build and source:
```bash
colcon build --symlink-install
source install/setup.bash
```
2. Launch full stack:
```bash
ros2 launch risabot_automode competition.launch.py
```
3. Confirm critical topics exist:
```bash
ros2 topic list | grep -E "cmd_vel_auto_raw|cmd_vel_auto|loop_stats|health_status|odom|dashboard_state"
```
4. Confirm health is stable:
```bash
ros2 topic echo /health_status
```
- Expected in auto-ready state: `"ok": true` (or only transient stale keys during startup).

## 3. Ghost Controller Input Test

Goal: verify startup ghost axes do not move the robot.

1. Keep sticks/triggers untouched at startup.
2. Press `Y` once.
- Expected: controller unlock only, no mode toggle motion command.
3. Center sticks (if needed), then press `Y` again.
- Expected: mode toggles, still no twitch from stale axis frame.
4. Watch:
```bash
ros2 topic echo /cmd_vel
```
- Expected: no non-zero spike on the unlock frame.

If needed, tune:
- `/servo_controller unlock_requires_neutral` (keep `true`)
- `/servo_controller unlock_neutral_threshold` (start `0.15`)
- `joy_node deadzone` (currently set in `competition.launch.py`)

## 4. Command Safety and Failsafe Tests

### A. E-stop path

```bash
ros2 topic pub -1 /e_stop std_msgs/msg/Bool "{data: true}"
```
- Expected:
  - `/cmd_vel_auto` goes to zero
  - `auto_driver` state can show `EMERGENCY_STOP`
  - dashboard shows safety stop reason

Clear:
```bash
ros2 topic pub -1 /e_stop std_msgs/msg/Bool "{data: false}"
```

### B. Auto command dropout

1. Put robot in auto mode.
2. Stop `cmd_safety_controller` process (or block its input stream).
3. Expected within `auto_cmd_timeout` (~0.4s):
  - `servo_controller` forces manual stop
  - motor command goes zero

## 5. Loop Frequency and Jitter Checks

Watch loop stats:
```bash
ros2 topic echo /loop_stats
```

Pass targets:
- `auto_driver:auto_driver_cmd` avg >= 45 Hz
- `cmd_safety_controller:cmd_safety` avg >= 45 Hz
- `servo_controller:servo_encoder` avg >= 18 Hz
- `servo_controller:servo_hw` avg >= 9 Hz
- overruns should stay low and not continuously increase each second

## 6. Hardware Odometry Calibration (No IMU)

### A. Straight distance

1. Reset odometry display.
2. Drive exactly `1.00 m` straight (5 trials, same battery state).
3. Use median reported distance `D_med`.

Update either:
```text
new_ticks_per_meter = old_ticks_per_meter * (D_med / 1.00)
```
or:
```text
new_odom_distance_scale = old_odom_distance_scale * (1.00 / D_med)
```

Recommended:
- Use `ticks_per_meter` for main correction.
- Use `odom_distance_scale` for final fine tuning.

### B. Standstill drift

With robot stationary for 10 seconds:
- `/odom` linear speed should stay near zero (target < 0.03 m/s).
- If drifting, increase `/servo_controller odom_velocity_deadband` slightly.

### C. Runtime tuning commands

```bash
ros2 param set /servo_controller ticks_per_meter 1700.0
ros2 param set /servo_controller odom_distance_scale 0.62
ros2 param set /servo_controller odom_yaw_scale 0.95
ros2 param set /servo_controller odom_velocity_deadband 0.03
```

## 7. Bag Replay Regression

Record:
```bash
ros2 bag record /loop_stats /health_status /cmd_safety_status /cmd_vel_auto /odom
```

Replay + validate:
```bash
ros2 run risabot_automode bag_regression_validator --ros-args \
  -p window_sec:=45.0 \
  -p output_file:=/tmp/risa_regression.json
```

Validator reports PASS/FAIL using:
- loop frequency ratios
- loop overruns
- max command/odom speed bounds
- health status failures
- cmd safety timeout/e-stop counters

## 8. Competition Acceptance Checklist

- No startup twitch from controller unlock
- E-stop halts within one control cycle
- Auto command dropout forces manual stop
- Loop stats stay within target bands for full lap runtime
- Straight-line odometry error within +/-0.15 m per 1 m
- Health monitor remains `ok=true` during stable run
