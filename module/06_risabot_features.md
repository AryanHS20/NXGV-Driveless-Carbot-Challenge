# Module 6: RISA-bot Custom Features & User Guide

This module covers the high-level custom features built into RISA-bot, including the web-based interactive dashboard, the game controller joystick mapping, challenge state sequencing, and the manual movement record-and-playback system.

---

## 1. The RISA-bot Web Dashboard

The RISA-bot has a built-in web dashboard designed for headless monitoring and debugging. 

### 1.1. How to Access It
1. Start the dashboard node on the robot:
   ```bash
   ros2 run risabot_automode dashboard
   ```
2. On your laptop browser, navigate to:
   ```text
   http://<robot_ip>:8080
   ```

### 1.2. Features
- **Live Streams**: Toggle between camera color feed, lane follower debugging, lidar obstacle, and signage detection output.
- **LiDAR 2D Canvas**: A real-time polar grid plotting incoming `/scan` points. It highlights active detection windows for wall follower and obstacles.
- **System Health Monitor**: Monitors active ROS 2 nodes and shows warning logs when standard streams become stale or crash.
- **Dynamic Parameter Tuning**: Get/Set node parameters (such as PID gains, thresholds, speeds) on-the-fly without restarting the ROS system.

![RISA-bot Web Dashboard Interface Screenshot](images/dashboard_interface_screenshot.png)
<!-- Image Description: Full screenshot of the RISA-bot web dashboard, showing the camera stream, telemetry stats (yaw, speed, error), the LiDAR radar plot, and the parameter group accordion list. -->

---

## 2. Joystick Controller Mappings

The robot can be driven manually or overridden at any time using a standard wireless game controller. The `servo_controller` node processes these commands.

![Game Controller Button Mappings Diagram](images/controller_mappings.png)
<!-- Image Description: A diagram of a game controller with labels pointing to sticks and buttons mapping to RISA-bot functions (Throttle, Steering, Gear changes, Recording, Playback, and Challenges). -->

### Axis & Button Controls:
- **Left Stick Y (Axis 1)**: Throttle (moves the robot forward/reverse).
- **Right Stick X (Axis 2)**: Steering (adjusts front servo angle).
- **D-Pad UP/DOWN (Axis 7)**: Speed Gear selection (`[25%, 40%, 60%, 100%]` power caps).
- **Start (Button 11) / Y (Button 4)**: Toggle between **Manual** and **Autonomous** modes.
- **LB (Button 6)**: Previous challenge state.
- **RB (Button 7)**: Next challenge state.
- **Button A (Button 0)**: Start / Stop Recording movement.
- **Button X (Button 3)**: Start / Stop Playback of current movement.
- **Button B (Button 1)**: Save the current recorded movement to a file.
- **D-Pad Left/Right (Axis 6)**: Cycle through saved recording files on disk.

---

## 3. Challenge Sequencing

RISA-bot's auto mode is designed around a state machine containing 12 predefined challenge states. You can cycle states manually with LB/RB, or let the `auto_driver` node transition automatically based on distance:

```text
  LANE_FOLLOW ──> OBSTRUCTION ──> ROUNDABOUT ──> BOOM_GATE_1 ──> TUNNEL ──> BOOM_GATE_2 
                                                                                │
  PARALLEL_PARK <── TRAFFIC_LIGHT <── BUMPER <── HILL <── PERPENDICULAR_PARK <──┘
```

The active state controls which sensors the robot listens to (e.g., ignoring LiDAR in roundabout, disabling lane camera inside the tunnel).

---

## 4. Movement Recording & Playback

For highly complex, non-algorithmic tasks like parallel parking, RISA-bot features a **Movement Record & Playback System**.

### 4.1. The Recording Mechanism
1. Place the robot in **manual mode**.
2. Press **Button A** on the controller. The dashboard indicator will flash `RECORDING`.
3. Drive the robot manually to perform the desired maneuver. The `servo_controller` captures steering and motor inputs at **20 Hz** (every 0.05 seconds).
4. Press **Button A** again to stop. The actions are saved in the internal buffer.
5. Press **Button B** to write the buffer to disk under `~/risabot_recordings/` as a JSON file.

### 4.2. Playback and Signage Triggers
- You can manually replay the buffer by pressing **Button X** (or through the Web Dashboard UI).
- **Automatic Playback Trigger**: In the autonomous state machine, when the robot enters the `PARALLEL_PARK` or `PERPENDICULAR_PARK` states and the signage detector recognizes the parking signboard, it will automatically load the designated recording and replay the actions to park the robot precisely without camera/lane guidelines.
---
**Previous:** [Module 5 — AI Detection & BPU](05_ai_detection.md)
**Next:** [Module 7 — Developer Troubleshooting](07_troubleshooting.md)
