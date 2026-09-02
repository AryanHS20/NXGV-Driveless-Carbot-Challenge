# Commands Reference

Quick-copy commands for operating the RISA-bot.

---

## Robot Bash Aliases

These are set up in `~/.bashrc` on the robot (user `sunrise`).

### Node Shortcuts

| Alias        | Runs                                             |
| ------------ | ------------------------------------------------ |
| `astra`      | Astra Mini camera launch                         |
| `ydlidar`    | YDLiDAR Tmini Plus driver                        |
| `servoc`     | Servo controller (joystick)                      |
| `obstav`     | LiDAR obstacle avoidance                         |
| `autod`      | Auto driver node                                 |
| `linefollow` | Line follower camera                             |
| `risabot`    | **Competition launch** (`competition.launch.py`) |
| `sos`        | Source the workspace                             |
| `s`          | Source `~/.bashrc`                               |

### Build Shortcuts

| Alias       | What it does                                                                                                                                                                              |
| ----------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `cb`        | `colcon build --symlink-install` (standard full rebuild). Use this 95% of the time after changing Python/C++ files.                                                                       |
| `cbp <pkg>` | Build one package only (fastest)                                                                                                                                                          |
| `cbc`       | `rm -rf build install` + `colcon build` (clean rebuild). Use this **ONLY** when `cb` throws weird `CMake` or `ament_prefix_path` errors. It deletes cache and forces a fresh compilation. |

### Git Shortcuts

| Alias      | What it does                       |
| ---------- | ---------------------------------- |
| `gs`       | `git status`                       |
| `gp`       | `git pull`                         |
| `gc "msg"` | `git add . && git commit -m "msg"` |
| `gpu`      | `git push`                         |

### Multi-Node Launchers

**`run_risabot`** — Opens each node in a separate **xfce4-terminal** tab:

1. Astra Camera
2. YDLiDAR (2s delay after camera)
3. Servo Controller
4. Obstacle Avoidance
5. Auto Driver
6. Line Follower

**`run_trisabot`** — Same as above but in **tmux** windows (for SSH sessions):

```bash
run_trisabot    # starts all nodes in tmux session "risabot"
tmux attach -t risabot   # reattach if disconnected
tmux kill-session -t risabot   # kill everything
```

### Utility

| Alias       | What it does                                     |
| ----------- | ------------------------------------------------ |
| `fix_astra` | Restores `openni2_redist` from backup if missing |

---

## Build & Deploy

```bash
# On your PC — push changes
git add . && git commit -m "message" && git push

# On the robot — pull and build
ssh risabot
cd ~/risabotcar_ws/src/RISA-bot
git checkout test && gp     # or: main
cb && sos
```

---

## Launch

```bash
# --- TEST BRANCH ---
ros2 launch risabot_automode competition.launch.py

# --- MAIN BRANCH ---
run_risabot                                    # LiDAR + auto_driver
ros2 launch control_servo robot_rc.launch.py   # joystick controller
ros2 launch astra_camera astra_mini.launch.py  # camera (separate)
```

### Run Individual Nodes

```bash
ros2 run risabot_automode auto_driver
ros2 run risabot_automode line_follower_camera
ros2 run risabot_automode traffic_light_detector
ros2 run risabot_automode boom_gate_detector
ros2 run risabot_automode tunnel_wall_follower
ros2 run risabot_automode obstruction_avoidance
ros2 run risabot_automode parking_controller
ros2 run obstacle_avoidance obstacle_avoidance
ros2 run obstacle_avoidance_camera obstacle_avoidance_camera
```

---

## Monitor Topics

```bash
# See all active topics
ros2 topic list

# Watch a topic (live values)
ros2 topic echo /lane_error
ros2 topic echo /traffic_light_state
ros2 topic echo /boom_gate_open
ros2 topic echo /tunnel_detected
ros2 topic echo /obstruction_active
ros2 topic echo /parking_complete
ros2 topic echo /obstacle_front
ros2 topic echo /auto_mode
ros2 topic echo /cmd_vel

# Topic frequency (check if node is publishing)
ros2 topic hz /scan
ros2 topic hz /camera/color/image_raw
ros2 topic hz /lane_error
```

---

## Control the Robot (CLI)

```bash
# Toggle auto mode
ros2 topic pub --once /auto_mode std_msgs/Bool "data: true"
ros2 topic pub --once /auto_mode std_msgs/Bool "data: false"

# Set challenge state (test branch)
ros2 topic pub --once /set_challenge std_msgs/String "data: LANE_FOLLOW"
ros2 topic pub --once /set_challenge std_msgs/String "data: TUNNEL"
ros2 topic pub --once /set_challenge std_msgs/String "data: TRAFFIC_LIGHT"
ros2 topic pub --once /set_challenge std_msgs/String "data: PARALLEL_PARK"
ros2 topic pub --once /set_challenge std_msgs/String "data: PERPENDICULAR_PARK"

# Trigger parking
ros2 topic pub --once /parking_command std_msgs/String "data: parallel"
ros2 topic pub --once /parking_command std_msgs/String "data: perpendicular"

# Manual drive from CLI (test only)
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.15}, angular: {z: 0.0}}" --rate 10
# Ctrl+C to stop (will send zero automatically)
```

---

## Set Parameters (Live Tuning)

```bash
# List all parameters for a node
ros2 param list /auto_driver
ros2 param list /line_follower_camera

# Get current value
ros2 param get /auto_driver steering_gain
ros2 param get /line_follower_camera smoothing_alpha

# Set new value (takes effect immediately)
ros2 param set /auto_driver steering_gain 0.5
ros2 param set /line_follower_camera smoothing_alpha 0.3
ros2 param set /line_follower_camera dead_zone 0.03
ros2 param set /line_follower_camera white_threshold 150

# Traffic light HSV
ros2 param set /traffic_light_detector sat_min 80
ros2 param set /traffic_light_detector val_min 80

# Tunnel PD
ros2 param set /tunnel_wall_follower kp 1.2
ros2 param set /tunnel_wall_follower kd 0.3

# Obstruction dodge
ros2 param set /obstruction_avoidance detect_dist 0.50
ros2 param set /obstruction_avoidance steer_angular 0.6

# Parking
ros2 param set /parking_controller drive_speed 0.15
ros2 param set /parking_controller parallel_steer_angle 0.6

# Controller speed limits
ros2 param set /servo_controller max_linear_speed 0.20
ros2 param set /servo_controller max_angular_speed 0.80
```

---

## Debugging

```bash
# Check which nodes are running
ros2 node list

# Check node details (subscriptions, publishers, params)
ros2 node info /auto_driver
ros2 node info /line_follower_camera

# Check if camera is Publishing
ros2 topic hz /camera/color/image_raw    # Should be ~30 Hz

# Check LiDAR
ros2 topic hz /scan                      # Should be ~8-12 Hz

# View camera feed (if X11 forwarding is set up)
ros2 run rqt_image_view rqt_image_view

# Kill a single node
ros2 lifecycle set /auto_driver shutdown
# Or just Ctrl+C the launch terminal

# Check serial ports
ls -la /dev/serial/by-id/
```

---

## Useful Shortcuts

```bash
# Emergency stop — send zero velocity
ros2 topic pub --once /cmd_vel geometry_msgs/Twist "{}"

# Quick check if robot is responsive — spin in place
ros2 topic pub --once /cmd_vel geometry_msgs/Twist "{angular: {z: 0.5}}"

# Verify joystick is connected
ls /dev/input/js*
jstest /dev/input/js0    # See live button/axis values
```
