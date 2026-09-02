# Main Branch Guide

The `main` branch is for **individual node development and testing**.

## How to Run

```bash
cd ~/risabotcar_ws/src/RISA-bot
git checkout main && gp
cb && sos
run_risabot
```

## What `run_risabot` Launches

Opens 6 separate xfce4-terminal tabs (or tmux windows with `run_trisabot`):

| Order | Tab Name       | Node                                               |
| ----- | -------------- | -------------------------------------------------- |
| 1     | Astra Camera   | `ros2 launch astra_camera astra_mini.launch.py`    |
| 2     | YDLiDAR        | `ydlidar_ros2_driver_node` (2s delay after camera) |
| 3     | Servo Ctrl     | `servo_controller` (joystick driving)              |
| 4     | Obstacle Avoid | `obstacle_avoidance` (LiDAR)                       |
| 5     | Auto Driver    | `auto_driver` (state machine + motor control)      |
| 6     | Line Follower  | `line_follower_camera`                             |

> **Note:** For SSH sessions, use `run_trisabot` instead — same nodes but in tmux.

## Launching Additional Nodes

```bash
# Camera
ros2 launch astra_camera astra_mini.launch.py

# Line follower (needs camera running)
ros2 run risabot_automode line_follower_camera

# Obstacle avoidance (LiDAR)
ros2 run obstacle_avoidance obstacle_avoidance

# Obstacle avoidance (camera)
ros2 run obstacle_avoidance_camera obstacle_avoidance_camera

# Joystick controller (manual driving)
ros2 launch control_servo robot_rc.launch.py
```

## Controller Node (`servo_controller`)

The main branch uses the **original monolithic controller** at:
`control_servo/control_servo/servo_controller.py` (V9 — Competition Ready)

### Controls

| Input                  | Action                                        |
| ---------------------- | --------------------------------------------- |
| Left Stick Up/Down     | Drive forward/reverse                         |
| Right Stick Left/Right | Ackermann steering (servo 4)                  |
| D-Pad Up/Down          | Gear shift (5 levels: 15, 25, 40, 60, 100)    |
| Y Button               | Toggle AUTO / MANUAL mode                     |
| X Button               | Trigger parking (stop + center for N seconds) |

### Auto Mode Behavior

When Y is pressed to enter auto mode, the controller handles everything internally:

1. **Lane Following** — PD control with 3-tier gain:
   - Small error (<0.03): No steering
   - Medium error (<0.15): Gain=30 with derivative damping
   - Large error (>0.15): Gain=60 with turn persistence (holds direction for 0.8s minimum)

2. **Obstacle Dodge** — When `/obstacle_front` = True:
   - Checks LiDAR left vs right for clearer side
   - Steers 45° to the clear side for 1.7 seconds
   - Returns to lane following

3. **Roundabout** — If consistent left turn for >1.5 seconds:
   - Locks full left (45°) for 3.5 seconds
   - Then resumes normal lane following

4. **Parking** — X button:
   - Stops and centers steering
   - Resumes after fixed duration

### Known Issue: Crashes in Auto Mode

The controller opens `Rosmaster()` which uses the same serial port as `auto_driver`. Running both simultaneously causes serial port conflicts. **Solution:** Only run ONE at a time — either `run_risabot` (which has `auto_driver`) OR `robot_rc.launch.py` (which has the controller), not both.

## Key Topics (Main Branch)

| Topic                         | Type      | Published By              |
| ----------------------------- | --------- | ------------------------- |
| `/scan`                       | LaserScan | ydlidar_ros2_driver       |
| `/odom`                       | Odometry  | servo_controller          |
| `/cmd_vel`                    | Twist     | auto_driver               |
| `/auto_mode`                  | Bool      | servo_controller          |
| `/lane_error`                 | Float32   | line_follower_camera      |
| `/obstacle_front`             | Bool      | obstacle_avoidance        |
| `/obstacle_detected_fused`    | Bool      | auto_driver               |
| `/joy`                        | Joy       | joy_node                  |
| `/dashboard_state`            | String    | auto_driver               |
| `/dashboard_ctrl`             | String    | servo_controller          |
| `/camera/debug/line_follower` | Image     | line_follower_camera      |
| `/camera/debug/traffic_light` | Image     | traffic_light_detector    |
| `/camera/debug/obstacle`      | Image     | obstacle_avoidance_camera |
