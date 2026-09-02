# Tuning Guide — Physical Course

Step-by-step parameter adjustments for on when testing on the course.
**Rule: Tune in this order.** Lane following first — everything else depends on it.

---

## Prerequisites

```bash
# Terminal 1: Launch everything
ros2 launch risabot_automode competition.launch.py

# Terminal 2: Keep open for tuning commands
```

Use **Start button** on controller to toggle auto/manual. Switch to manual whenever the robot misbehaves.

---

## Step 1: Lane Following

Place robot on a **straight section**. Toggle to auto mode.

```bash
ros2 topic echo /lane_error      # watch live error values
```

> **Tip:** Debug visualization is ON by default (`show_debug: true`). Open the dashboard at
> `http://<robot_ip>:8080` and select the **Lane Lines** debug overlay to see what the robot detects:
> - **Blue dots** = left white border
> - **Pink dots** = right white border
> - **Green dots** = computed lane center
> - **Red vertical line** = image center reference

| Symptom | Fix |
|---|---|
| Oscillates on straight road | `ros2 param set /auto_driver pid_kp 0.5` (lower P gain) |
| Still oscillating after lowering kp | `ros2 param set /auto_driver pid_kd 0.3` (increase damping) |
| Too slow to respond to curves | `ros2 param set /auto_driver pid_kp 1.0` |
| Drifts to one side on a straight | `ros2 param set /auto_driver pid_ki 0.02` (small integral correction) |
| Too fast in turns | `ros2 param set /auto_driver min_turn_speed 0.3` |
| Not detecting white lane borders | `ros2 param set /line_follower_camera white_threshold 180` (lower = more sensitive) |
| Floor is detected as lane border | `ros2 param set /line_follower_camera white_threshold 220` (higher = stricter) |
| Small noise dots trigger false lines | `ros2 param set /line_follower_camera morph_open_size 5` (larger noise cleanup) |
| Gaps in lane lines (broken detection) | `ros2 param set /line_follower_camera morph_close_size 7` (bridge bigger gaps) |
| Loses lock on sharp turns | `ros2 param set /line_follower_camera search_radius_px 100` (wider search) |
| Reads too far ahead / cuts corners | `ros2 param set /line_follower_camera crop_ratio_base 0.3` |
| Error signal is too jumpy | Lower `kalman_measurement_noise` to 0.05 (trust camera more) |
| Error signal is too sluggish | Raise `kalman_process_noise` to 0.05 (faster reaction) |

**Tuning order:**
1. Set `pid_kp` = 0.5, `pid_kd` = 0.2, `forward_speed` = 0.10 → check no shaking on straight
2. Slowly increase `pid_kp` until curves work (0.6–1.0 is typical)
3. If still oscillating, increase `pid_kd` to 0.3
4. Adjust `white_threshold` until debug overlay shows clean border detection
5. Increase `forward_speed` gradually (0.12 → 0.15 → 0.20)
6. Test curves → if cuts corners, increase `crop_ratio_base` to 0.5

### Parameter Ranges

| Parameter | Node | Default | Range |
|---|---|---|---|
| `pid_kp` | auto_driver | 0.8 | 0.3 – 1.5 |
| `pid_ki` | auto_driver | 0.01 | 0.0 – 0.05 |
| `pid_kd` | auto_driver | 0.20 | 0.05 – 0.5 |
| `forward_speed` | auto_driver | 0.15 | 0.08 – 0.25 |
| `min_turn_speed` | auto_driver | 0.4 | 0.2 – 0.8 |
| `white_threshold` | line_follower_camera | 200 | 150 – 240 |
| `crop_ratio_base` | line_follower_camera | 0.4 | 0.2 – 0.6 |
| `search_radius_px` | line_follower_camera | 80 | 40 – 120 |
| `morph_open_size` | line_follower_camera | 3 | 0 – 7 |
| `morph_close_size` | line_follower_camera | 5 | 0 – 9 |
| `dead_zone` | line_follower_camera | 0.05 | 0.0 – 0.1 |

### Advanced: IPM (Bird's Eye View)

IPM is **disabled by default** because the camera is mounted horizontally at 8.5cm height. If you tilt the camera downward (recommended: 15–30°), you can enable IPM for better curve detection:

```bash
ros2 param set /line_follower_camera ipm_enabled true
ros2 param set /line_follower_camera ipm_top_width_ratio 0.3
```

> ⚠️ IPM requires calibration. Use the debug overlay to verify that lane lines appear **parallel** after the warp. If they curve inward or outward, adjust `ipm_top_width_ratio`.

---

## Step 2: Front Obstacle Detection

Place object **in front** at ~0.4m.

```bash
ros2 topic echo /obstacle_front
```

| Symptom | Fix |
|---|---|
| Stops too far away | `ros2 param set /obstacle_avoidance_node min_obstacle_distance 0.35` |
| Hits object before stopping | `ros2 param set /obstacle_avoidance_node min_obstacle_distance 0.55` |

---

## Step 3: Camera Obstacle (Edge Detection)

The camera obstacle node detects objects by measuring **edge density** in the center of the frame. Objects have sharp edges; a flat track does not.

Place an object (any color) ~30cm in front of the camera:

```bash
ros2 topic echo /obstacle_detected_camera
```

> **Tip:** Use the **Obstacle** debug tab on the Dashboard to see the live edge overlay and density percentage.

| Symptom | Fix |
|---|---|
| Not detecting the object | `ros2 param set /obstacle_avoidance_camera edge_threshold 0.08` |
| False positives on track lines | `ros2 param set /obstacle_avoidance_camera edge_threshold 0.18` |
| Too sensitive to texture/noise | `ros2 param set /obstacle_avoidance_camera canny_low 80` |
| Missing subtle edges | `ros2 param set /obstacle_avoidance_camera canny_low 30` |
| Flickering on/off rapidly | Increase `hysteresis_on` to 5 and `hysteresis_off` to 7 |

### Parameter Ranges

| Parameter | Default | Range | Effect |
|---|---|---|---|
| `edge_threshold` | 0.02 | 0.01 – 0.10 | Ratio of edge pixels to trigger (2% default) |
| `canny_low` | 50 | 20 – 100 | Lower = more edges (more sensitive) |
| `canny_high` | 150 | 100 – 250 | Upper Canny threshold |
| `blur_kernel` | 5 | 3 – 9 | Larger = smoother (reduces noise, fewer edges) |
| `hysteresis_on` | 3 | 1 – 10 | Frames before STOP triggers |
| `hysteresis_off` | 5 | 1 – 15 | Frames before CLEAR triggers |

## Step 4: Obstruction Avoidance (Lateral Dodge)

Place obstacle in the lane. Set state:
```bash
ros2 topic pub --once /set_challenge std_msgs/String "data: OBSTRUCTION"
```

| Symptom | Fix |
|---|---|
| Doesn't dodge early enough | `ros2 param set /obstruction_avoidance detect_dist 0.65` |
| Doesn't steer far enough | `ros2 param set /obstruction_avoidance steer_angular 0.8` |
| Clips while passing | `ros2 param set /obstruction_avoidance pass_duration 2.5` |
| Overshoots returning to lane | `ros2 param set /obstruction_avoidance steer_back_duration 1.0` |

---

## Step 5: Traffic Light

Hold colored cards in front of camera. Set state:
```bash
ros2 topic pub --once /set_challenge std_msgs/String "data: TRAFFIC_LIGHT"
ros2 topic echo /traffic_light_state
```

| Symptom | Fix |
|---|---|
| Not detecting any color | `ros2 param set /traffic_light_detector sat_min 50` then `val_min 50` |
| Confusing red/green | Narrow the H ranges for each color |
| False positives | `ros2 param set /traffic_light_detector min_pixel_count 100` |

> ⚠️ HSV thresholds are **very sensitive to lighting**. Always tune at the competition venue.

---

## Step 6: Boom Gate

Drive toward the boom gate. Set state:
```bash
ros2 topic pub --once /set_challenge std_msgs/String "data: BOOM_GATE_2"
ros2 topic echo /boom_gate_open
```

| Symptom | Fix |
|---|---|
| Gate closed but reads OPEN | `ros2 param set /boom_gate_detector min_gate_points 3` |
| Gate open but reads CLOSED | `ros2 param set /boom_gate_detector distance_variance_max 0.08` |
| Detection range wrong | Adjust `min_detect_dist` and `max_detect_dist` |

---

## Step 7: Tunnel Wall Following

Drive into the tunnel. Set state:
```bash
ros2 topic pub --once /set_challenge std_msgs/String "data: TUNNEL"
ros2 topic echo /tunnel_detected
```

| Symptom | Fix |
|---|---|
| Oscillates between walls | `ros2 param set /tunnel_wall_follower kp 0.8` |
| Still oscillating | `ros2 param set /tunnel_wall_follower kd 0.5` |
| Drifts to one side | Adjust `target_center_dist` ±0.05 |
| Not entering tunnel mode | `ros2 param set /tunnel_wall_follower min_wall_points 2` |
| Too fast in tunnel | `ros2 param set /tunnel_wall_follower forward_speed 0.10` |

---

## Step 8: Parking (tune last)

> ⚠️ Start with **very low speeds** (0.08) and short distances (0.15). Increase gradually.

```bash
# Parallel
ros2 topic pub --once /set_challenge std_msgs/String "data: PARALLEL_PARK"
ros2 topic pub --once /parking_command std_msgs/String "data: parallel"

# Perpendicular
ros2 topic pub --once /set_challenge std_msgs/String "data: PERPENDICULAR_PARK"
ros2 topic pub --once /parking_command std_msgs/String "data: perpendicular"
```

| Symptom | Fix |
|---|---|
| Doesn't pull far enough past slot | increase `parallel_forward_dist` |
| Doesn't enter slot fully | increase `parallel_reverse_dist` |
| Turns too sharply | decrease `parallel_steer_angle` |
| Doesn't turn enough | increase `parallel_steer_angle` |
| Too fast | decrease `drive_speed` and `reverse_speed` |

## Step 9: State Transition Distances

These control when the state machine auto-advances. Run a full lap and adjust:

```bash
ros2 param set /auto_driver dist_roundabout 2.0         # how far through roundabout
ros2 param set /auto_driver dist_boom_gate_1_pass 0.5    # after boom gate 1
ros2 param set /auto_driver dist_boom_gate_2_pass 0.5    # after boom gate 2
ros2 param set /auto_driver dist_hill 1.0                # over the hill
ros2 param set /auto_driver dist_bumper 0.8              # over bumpers
ros2 param set /auto_driver dist_traffic_light_pass 0.5  # after green light
ros2 param set /auto_driver dist_drive_to_perp 1.0       # parallel → perp parking
```

| Symptom | Fix |
|---|---|
| Transitions too early | Increase the relevant `dist_*` parameter |
| Stuck in a state too long | Decrease the relevant `dist_*` parameter |
| Doesn't move forward | `ros2 param set /auto_driver forward_speed 0.2` |

---

## Quick Cheat Sheet

```bash
# The 8 most common params you'll adjust:
ros2 param set /auto_driver forward_speed 0.15
ros2 param set /auto_driver pid_kp 0.8
ros2 param set /auto_driver pid_kd 0.20
ros2 param set /auto_driver min_turn_speed 0.4
ros2 param set /line_follower_camera white_threshold 200
ros2 param set /line_follower_camera crop_ratio_base 0.4
ros2 param set /tunnel_wall_follower kp 1.2
ros2 param set /parking_controller drive_speed 0.15
```

