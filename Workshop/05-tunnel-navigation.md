# Module 5: Tunnel Navigation with LiDAR

## Learning Objectives

By the end of this module, you will:
- Understand why LiDAR is used for tunnel navigation instead of a camera
- Know how the RISA-bot's tunnel wall follower processes LiDAR data to stay centred
- Understand the centerline path algorithm and PD controller used for steering
- Be able to launch the tunnel system, observe it on the dashboard, and tune its parameters
- Know how the `auto_driver` automatically switches between lane following and tunnel mode

---

## 1. Why LiDAR for Tunnels?

In Module 4, you learned how the camera detects lane markings to steer. But what happens when the robot enters a tunnel or a dark corridor?

**The camera fails** — it relies on visible lane lines, and those disappear in darkness. This is where the **LiDAR** takes over. Remember from Module 2, the LiDAR fires laser beams in all directions and measures the distance to whatever they hit. Lasers work perfectly in the dark — they don't need ambient light.

```text
Camera lane following:              LiDAR wall following:

   Can't see in the dark! ✗           Lasers work in the dark! ✓
   
   ┌─── tunnel walls ───┐            ┌─── tunnel walls ───┐
   │                     │            │ ←0.3m    0.3m→     │
   │  [no lane lines]    │            │   ·  ·  ·  ·  ·    │
   │      🚗             │            │      🚗             │
   │                     │            │   ·  ·  ·  ·  ·    │
   └─────────────────────┘            └─────────────────────┘
```

The idea: if the LiDAR detects walls on **both sides**, the robot must be in a tunnel. It can then compute the **centre** between the two walls and steer to stay on that path — just like the lane follower stays centred between lane lines.

---

## 2. System Architecture: Where the Tunnel Node Fits

The tunnel wall follower is one node in the RISA-bot's full system. The `auto_driver` (the robot's "brain") automatically decides which navigation mode to use:

```text
┌────────────────────────┐
│ line_follower_camera   │──▶ /lane_error ──────────────┐
│ (camera-based)         │                              │
└────────────────────────┘                              │
                                                        ▼
┌────────────────────────┐                    ┌──────────────────┐
│ tunnel_wall_follower   │──▶ /tunnel_cmd_vel ──▶│   auto_driver    │──▶ Motors
│ (LiDAR-based)          │                    │  (picks which    │
│                        │──▶ /tunnel_detected──▶│   mode to use)   │
└────────────────────────┘                    └──────────────────┘
```

**How does `auto_driver` decide?** It uses a priority system:

| Priority | Condition | Action |
|----------|-----------|--------|
| Higher | Obstacle detected | Avoid obstacle |
| ↕ | **Tunnel detected** (`/tunnel_detected = True`) | **Use `/tunnel_cmd_vel`** |
| Lower | Default | Use lane following (camera) |

When the `tunnel_wall_follower` node detects walls on both sides, it publishes `True` on `/tunnel_detected`. The `auto_driver` immediately switches from camera lane following to the tunnel's steering commands. When the walls disappear (robot exits the tunnel), it switches back automatically.

---

## 3. How the Tunnel Wall Follower Works

Open `src/risabot_automode/risabot_automode/tunnel_wall_follower.py` and follow along.

### Step 1 — Convert LiDAR data and classify walls

The LiDAR publishes hundreds of distance readings at different angles. The first step is to convert each reading from **polar coordinates** (angle, distance) to **Cartesian coordinates** (x, y) and classify which side of the robot it's on:

```python
for i, r in enumerate(msg.ranges):
    # Skip invalid readings (out of range, NaN, too far away)
    if not (msg.range_min <= r <= msg.range_max):
        continue
    if r > max_wall_dist:  # ignore anything beyond 80cm
        continue

    # Calculate the angle (with 180° offset for LiDAR mounting)
    angle = msg.angle_min + i * msg.angle_increment + offset
    angle = math.atan2(math.sin(angle), math.cos(angle))  # normalize to -π to +π

    # Convert to x, y coordinates
    x = r * math.cos(angle)   # forward distance
    y = r * math.sin(angle)   # side distance (+ = left, - = right)

    # Classify based on angle
    if left_angle_min <= angle <= left_angle_max:     # 15° to 120°
        left_xy.append((x, y))
    elif right_angle_min <= angle <= right_angle_max:  # -120° to -15°
        right_xy.append((x, y))
```

```text
            y (left)
            ↑
            │  · · · LEFT WALL points (angle 15° to 120°)
            │  ·
            │
  ──────────🚗──────────→ x (forward)
            │
            │  ·
            │  · · · RIGHT WALL points (angle -120° to -15°)
```

**Why exclude 0° to 15° and -15° to 0°?** Points directly in front of the robot could be an obstacle or the end of the tunnel — not a wall. By excluding a narrow cone ahead, we avoid confusing "wall" with "end of tunnel."

> [!NOTE]
> **LiDAR Angle Offset:** The RISA-bot's LiDAR is mounted with its 0° pointing **backward**. Adding `3.1416` (π = 180°) rotates all angles so that 0° means forward, matching the robot's driving direction.

### Step 2 — Hysteresis for tunnel detection

Detecting walls in a single scan could be unreliable — a random reflection or brief gap might cause flicker. The node uses **hysteresis**: it requires **3 consecutive scans** with both walls before declaring "in tunnel," and 3 consecutive scans without before declaring "exited."

```python
# Need both left AND right walls
walls_detected = len(left_xy) >= min_wall_points and len(right_xy) >= min_wall_points

if walls_detected:
    self._tunnel_on_count += 1    # count consecutive frames with walls
    self._tunnel_off_count = 0
else:
    self._tunnel_off_count += 1   # count consecutive frames without walls
    self._tunnel_on_count = 0

# Only change state after 3 consecutive frames (hysteresis)
if not self.last_in_tunnel and self._tunnel_on_count >= 3:
    self.last_in_tunnel = True    # TUNNEL ENTERED
elif self.last_in_tunnel and self._tunnel_off_count >= 3:
    self.last_in_tunnel = False   # TUNNEL EXITED
```

This prevents the robot from flickering between "tunnel" and "lane follow" modes at the entrance/exit.

### Step 3 — Compute the centerline path

This is the core algorithm. Instead of just averaging the left and right distances (which fails in curves), the node computes a **centerline path** through the tunnel:

```text
Step 1: Bin wall points by forward distance (x-coordinate)

  x=0.05m:  L=0.25m  R=-0.28m  → center_y = (0.25 + (-0.28))/2 = -0.015m
  x=0.10m:  L=0.23m  R=-0.30m  → center_y = (0.23 + (-0.30))/2 = -0.035m
  x=0.15m:  L=0.20m  R=-0.32m  → center_y = (0.20 + (-0.32))/2 = -0.060m
  x=0.20m:  L=0.18m  R=-0.34m  → center_y = (0.18 + (-0.34))/2 = -0.080m

Step 2: Connect the center_y values → that's the centerline!

     LEFT WALL               CENTERLINE              RIGHT WALL
        ·  ·                     ·                       ·  ·
          ·  ·                    ·                     ·  ·
            ·  ·                   ·                   ·  ·
              ·  ·                  ·                 ·  ·
                ·  ·                ·               ·  ·
                              🚗
```

In a **straight tunnel**, the centerline is straight. In a **curved tunnel**, the centerline curves too — the robot follows it naturally.

```python
# For each bin, compute the midpoint between left and right wall
for b, data in bins.items():
    if data['left'] and data['right']:
        avg_left_y = sum(data['left']) / len(data['left'])
        avg_right_y = sum(data['right']) / len(data['right'])
        center_y = (avg_left_y + avg_right_y) / 2.0
        centerline.append((center_x, center_y))
```

### Step 4 — PD control with two error terms

The node uses two error signals from the centerline, not just one:

```text
1. LATERAL ERROR: How far is the centerline from the robot's centre?
   → The nearest centerline point's y-coordinate
   → Positive = centerline is to the left → steer left
   → Negative = centerline is to the right → steer right

2. HEADING ERROR: What angle is the centerline pointing?
   → Fit a line through the nearest 5 centerline points
   → If the slope tilts left, the tunnel curves left → steer left early
```

```python
# Lateral error: y-offset of nearest centerline point
lateral_error = centerline[0][1]

# Heading error: slope of centerline (linear regression on nearest points)
slope = ...  # compute from centerline points
heading_error = math.atan(slope)  # convert to angle

# PD control: combine both errors with proportional (P) and derivative (D) gains
angular_z = (kp * lateral_error + kd * d_lateral
           + kp_heading * heading_error + kd_heading * d_heading)
```

**Why two errors?** Lateral error alone only reacts when the robot is already off-centre. Heading error lets the robot **anticipate curves** and start turning before it drifts.

| Gain | What It Controls | Too Low | Too High |
|------|-----------------|---------|----------|
| `kp` (5.0) | Lateral centering strength | Robot drifts to one side | Robot oscillates left-right |
| `kd` (0.5) | Lateral damping | Slow correction, overshoots | Jerky, reacts to noise |
| `kp_heading` (1.0) | Curve anticipation | Late turning in curves | Oversteers into straight walls |
| `kd_heading` (0.1) | Heading damping | Wobbly curve following | Too sensitive to noise |

### Step 5 — EMA smoothing and publish

The raw PD output could spike between frames. An **Exponential Moving Average (EMA)** smooths it:

```python
# alpha=0.4 → 40% new value, 60% previous value
self.smoothed_angular_z = alpha * angular_z + (1 - alpha) * self.smoothed_angular_z

cmd.linear.x = forward_speed    # constant forward speed (0.12 m/s)
cmd.angular.z = -self.smoothed_angular_z  # publish as Twist
```

The final `Twist` message is published on `/tunnel_cmd_vel`, where the `auto_driver` picks it up when in tunnel mode.

---

## 4. Launching the Tunnel System

The tunnel wall follower runs as part of the full system bringup.

### Build and launch

```bash
cd ~/risabotcar_ws
colcon build --packages-select risabot_automode
source install/setup.bash
ros2 launch risabot_automode bringup.launch.py
```

This starts **all** nodes including the tunnel wall follower. You will see in the logs:

```
[tunnel_wall_follower]: Tunnel Wall Follower started (Centerline Path)
```

### Verify tunnel topics are publishing

Open a **second SSH terminal** and check:

```bash
source ~/risabotcar_ws/install/setup.bash

# Tunnel detection flag (True when both walls are detected)
ros2 topic echo /tunnel_detected

# Tunnel steering commands (only active when in tunnel)
ros2 topic echo /tunnel_cmd_vel

# Debug info (distances, errors, centerline points)
ros2 topic echo /tunnel_debug
```

---

## 5. Testing the Tunnel Detection

You can test tunnel detection without driving the robot. Set up a simple "tunnel" by placing two parallel objects (boxes, books, boards) approximately 40–60 cm apart, one on each side of the robot.

### Check if the LiDAR sees the walls

1. Open the **dashboard** at `http://192.168.x.x:8080`.
2. Look at the **LiDAR canvas** (bottom-right panel). You should see red dots forming the shape of your makeshift walls.
3. When walls are detected on both sides, the dashboard will show:
   - **Sensors panel:** Tunnel → `IN TUNNEL` (green dot)
   - **State Machine:** Should switch to `TUNNEL` state when in auto mode
   - **LiDAR canvas:** Shows `● TUNNEL` indicator in green

### Monitor the topics

In a terminal, watch the tunnel debug output:

```bash
ros2 topic echo /tunnel_debug
```

You will see JSON output like:

```json
{"l": 0.312, "r": 0.298, "lat": -0.007, "w": 0.035, "cl": [{"x": 0.05, "y": -0.007}, ...]}
```

| Field | Meaning |
|-------|---------|
| `l` | Average distance to **left** wall (metres) |
| `r` | Average distance to **right** wall (metres) |
| `lat` | **Lateral error** (negative = right of centre) |
| `w` | **Angular velocity** command (steering strength) |
| `cl` | **Centerline** points (x, y pairs) |

**Test it!** Slide the robot toward one wall:
- Closer to the left wall → `l` decreases, `lat` goes negative, `w` steers right
- Closer to the right wall → `r` decreases, `lat` goes positive, `w` steers left
- Centred → `lat` near 0, `w` near 0

---

## 6. Running Autonomous Tunnel Navigation

1. Place the robot at the entrance of your tunnel or corridor.
2. Open the dashboard in your browser.
3. Press **Start** on the joystick to enable auto mode.
4. The state machine should show `LANE_FOLLOW` initially.
5. Push the robot into the tunnel — when the LiDAR detects walls on both sides, the state automatically switches to `TUNNEL`.
6. Watch the robot navigate through the tunnel!
7. When it exits (walls disappear), it switches back to `LANE_FOLLOW`.

> [!IMPORTANT]
> Always hold the joystick and be ready to press **Start** to return to manual mode. The tunnel is tight — have your hand ready to catch the robot if it gets too close to a wall.

### Manually forcing tunnel mode

You can force the robot into tunnel mode for testing (even without walls) using:

```bash
ros2 topic pub /set_challenge std_msgs/String "data: 'TUNNEL'" --once
```

This tells `auto_driver` to override its state machine and use the tunnel steering commands. Use this for testing when you want to bypass the automatic detection.

---

## 7. Key Parameters and Tuning

All tunnel parameters are in `config/params.yaml` under `tunnel_wall_follower:`. You can change them live using the dashboard's parameter drawer or via CLI:

### Wall Detection Parameters

```bash
# How far away walls can be detected (metres)
ros2 param set /tunnel_wall_follower max_wall_dist 0.80

# Minimum LiDAR points needed to count as a wall
ros2 param set /tunnel_wall_follower min_wall_points 5

# Consecutive scans required before toggling tunnel state
ros2 param set /tunnel_wall_follower tunnel_hysteresis_frames 3
```

### Steering Parameters

```bash
# Forward speed inside tunnel (m/s) — slower = safer in tight spaces
ros2 param set /tunnel_wall_follower forward_speed 0.12

# Lateral centering: how hard to steer toward centre
ros2 param set /tunnel_wall_follower kp 5.0

# Lateral damping: prevent oscillation
ros2 param set /tunnel_wall_follower kd 0.5

# Curve anticipation: steer early based on tunnel direction
ros2 param set /tunnel_wall_follower kp_heading 1.0

# Output smoothing (0 = frozen, 1 = no smoothing)
ros2 param set /tunnel_wall_follower output_alpha 0.4
```

> [!TIP]
> Changes take effect immediately — no rebuild needed. Once you find values that work well, click **💾 Save Current as Default** in the dashboard's Parameters drawer to write them to `params.yaml`.

---

## 8. Exercises

### Exercise 1: Observe Tunnel Entry and Exit

1. Launch the full system with `ros2 launch risabot_automode bringup.launch.py`.
2. Open the dashboard and watch the Sensors panel.
3. Place two boxes on either side of the robot about 40cm apart.
4. Watch the Tunnel indicator flip from `NO` to `IN TUNNEL`.
5. Remove one box — what happens? Does the indicator change immediately or with a delay?

**Question:** Why does the indicator take a moment to change? What parameter controls this delay?

### Exercise 2: Compare Lane Following vs Tunnel Following

1. With the system running, watch `/tunnel_cmd_vel` and `/lane_error` simultaneously:
   ```bash
   # Terminal A
   ros2 topic echo /tunnel_cmd_vel

   # Terminal B
   ros2 topic echo /lane_error
   ```
2. When the robot is on the track (no walls), which topic is active? Which is zeros?
3. When the robot enters the tunnel, which topic becomes active?

This demonstrates how the `auto_driver` seamlessly switches between two completely different navigation strategies.

### Exercise 3: Tune Tunnel Speed

1. Start with the default `forward_speed` of `0.12`.
2. Increase it step by step while the robot drives through the tunnel:
   ```bash
   ros2 param set /tunnel_wall_follower forward_speed 0.15
   ros2 param set /tunnel_wall_follower forward_speed 0.18
   ```
3. At what speed does the robot start hitting walls?
4. If it oscillates at higher speed, try increasing `kd`:
   ```bash
   ros2 param set /tunnel_wall_follower kd 0.8
   ```

### Exercise 4: Understand the LiDAR Angle Ranges

The default settings scan from 15° to 120° on each side. What happens if you widen or narrow this range?

```bash
# Narrow the scan range (only 30° to 90°)
ros2 param set /tunnel_wall_follower left_angle_min 0.52
ros2 param set /tunnel_wall_follower left_angle_max 1.57
ros2 param set /tunnel_wall_follower right_angle_min -1.57
ros2 param set /tunnel_wall_follower right_angle_max -0.52
```

Does the robot behave differently in curves? Reset to defaults afterward:
```bash
ros2 param set /tunnel_wall_follower left_angle_min 0.26
ros2 param set /tunnel_wall_follower left_angle_max 2.09
ros2 param set /tunnel_wall_follower right_angle_min -2.09
ros2 param set /tunnel_wall_follower right_angle_max -0.26
```

---

## 9. Troubleshooting

| Symptom | Likely Cause | Fix |
|---------|-------------|-----|
| `/tunnel_detected` is always `False` | Walls too far apart or not enough LiDAR points | Move walls closer (< 80cm), or lower `min_wall_points` to `3` |
| `/tunnel_detected` flickers rapidly | Hysteresis too low | Increase `tunnel_hysteresis_frames` to `5` |
| Robot oscillates in tunnel | `kp` too high or `kd` too low | Reduce `kp` to `3.0`, increase `kd` to `0.8` |
| Robot hits one wall consistently | Steering offset or asymmetric LiDAR | Check `target_center_dist` — try `0.02` or `-0.02` |
| Robot doesn't steer at all in tunnel | `tunnel_cmd_vel` not publishing | Check `ros2 topic hz /tunnel_cmd_vel` |
| State doesn't switch to TUNNEL | `auto_driver` not receiving detection | Check `ros2 topic echo /tunnel_detected` |

---

## 10. Comparing Camera vs LiDAR Navigation

Now you have seen both navigation approaches in the RISA-bot. Here is a summary:

| Aspect | Camera Lane Following (Module 4) | LiDAR Tunnel Following (Module 5) |
|--------|----------------------------------|-----------------------------------|
| **Sensor** | RGB Camera | Laser Range Finder |
| **Works in dark?** | No ✗ | Yes ✓ |
| **What it detects** | Painted lane lines | Physical walls |
| **Output** | Error number (`/lane_error`) | Full Twist command (`/tunnel_cmd_vel`) |
| **Control** | PID in `auto_driver` | PD in `tunnel_wall_follower` itself |
| **Error signals** | 1 (lane centre offset) | 2 (lateral offset + heading angle) |
| **Smoothing** | Kalman filter | EMA (Exponential Moving Average) |
| **Detection** | Always active | Hysteresis-based (3 frames to toggle) |

The key insight: **the same `auto_driver` brain switches between both modes automatically**, based on what the sensors detect. This is a pattern used in real autonomous vehicles — they combine multiple navigation strategies for different conditions.

---

## 11. What You've Learned

In this module you explored the complete LiDAR-based tunnel navigation system:

```text
LiDAR → Polar to Cartesian → Wall Classification → Centerline Path → PD Control → Twist → Motors
```

You learned that:
- **LiDAR** provides distance measurements that work in darkness, unlike cameras
- The tunnel node **classifies** LiDAR points as left or right wall based on their angle
- A **centerline path** is computed by binning wall points and finding midpoints — this works for both straight and curved tunnels
- **Two error signals** (lateral position + heading angle) give more accurate steering than position alone
- **Hysteresis** prevents false tunnel detection by requiring 3 consecutive frames
- The `auto_driver` **automatically switches** between camera and LiDAR navigation based on conditions
- All parameters can be tuned live via `ros2 param set` or the dashboard

---

**Previous:** [Module 4 — Lane Following](04-lane-follower.md)
**Next:** [Module 6 — Autonomous Parking](06-autonomous-parking.md)
