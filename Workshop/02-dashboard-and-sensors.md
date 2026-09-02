# Module 2: Dashboard & Sensors

## Learning Objectives

By the end of this module, you will:
- Understand how to interact with a "headless" robot.
- Learn how to start the RISA-bot local dashboard.
- Learn how to connect to the dashboard via a web browser on your laptop.
- Understand how camera and LiDAR data flows in ROS 2
- View camera images using ROS tools
- Read and interpret LiDAR scan data
- Know which topics the RISA-bot sensors publish on

---

## 1. What is a Headless Robot?

In the previous module, you connected to the robot using **SSH**. This is called a **headless** setup because the robot itself does not have a physical monitor, keyboard, or mouse attached to it. 

While SSH is fantastic for running commands and seeing text output, what happens when we want to look at a live camera feed or see a 2D map of a room? We can't view pictures inside an SSH terminal!

To solve this, the RISA-bot has a built-in **Web Dashboard**. This dashboard acts as your "virtual monitor." It runs on the robot and serves a webpage over the Wi-Fi network directly to your laptop's browser.

---

## 2. Launching the Dashboard

Now that you have built your own `student_ws`, it is time to transition into the main `risabotcar_ws` where the advanced features live.

1. Open a new terminal on your Windows laptop and SSH into the robot:
   ```bash
   ssh sunrise@192.168.x.x
   ```
2. Navigate to the pre-installed workspace and source it:
   ```bash
   cd ~/risabotcar_ws
   source install/setup.bash
   ```
3. Run the dashboard node:
   ```bash
   ros2 run risabot_automode dashboard
   ```

You should see an output saying `Dashboard node starting on http://0.0.0.0:8080`.

---

## 3. Viewing the Dashboard

1. On your Windows laptop, open a web browser (like Chrome, Edge, or Firefox).
2. In the address bar, type `http://192.168.x.x:8080` (replacing `192.168.x.x` with your robot's actual IP address).
3. Hit Enter!

You should now see the RISA-bot web dashboard interface! 

> [!NOTE]
> **Why does it look empty?**
> Right now, the dashboard is running, but the camera and LiDAR nodes are *not* running yet! In the next section, we will learn how to turn on the sensors, and you will see the dashboard automatically populate with real-time video and data!

---

## 4. RISA-bot Sensors

| Sensor | Model | Topic | Message Type |
|---|---|---|---|
| Camera | Astra Mini | `/camera/color/image_raw` | `sensor_msgs/Image` |
| Depth | Astra Mini | `/camera/depth/image_raw` | `sensor_msgs/Image` |
| LiDAR | YDLiDAR Tmini Plus | `/scan` | `sensor_msgs/LaserScan` |

---

## 5. Hands-On: Camera

### 5.1. Start the camera

```bash
ros2 launch astra_camera astra_mini.launch.py
```

### 5.2. Check camera topics

```bash
ros2 topic list | grep camera
# /camera/color/camera_info
# /camera/color/image_raw
# /camera/depth/image_raw
```

### 5.3. Check frame rate

```bash
ros2 topic hz /camera/color/image_raw
# Should be ~30 Hz
```

### 5.4. View the Camera Stream

Because our robot is headless, we cannot use traditional graphical tools like `rqt_image_view`. Instead, we will use the dashboard we set up earlier!

1. Open a **new terminal** and SSH into the robot.
2. Ensure your workspace is sourced:
   ```bash
   cd ~/risabotcar_ws
   source install/setup.bash
   ```
3. Run the dashboard node:
   ```bash
   ros2 run risabot_automode dashboard
   ```
4. Open a web browser on your laptop and go to `http://192.168.x.x:8080`.

Since you already started the camera in Step 5.1, the dashboard will now automatically display the live video feed!

### 5.5. Understanding the Image message

```bash
ros2 topic echo /camera/color/image_raw --no-arr
# Shows header (timestamp, frame_id), height, width, encoding
# --no-arr hides the actual pixel data (too large to print)
```

Key fields:
- `height`: 480 pixels
- `width`: 640 pixels
- `encoding`: `rgb8` (3 bytes per pixel: R, G, B)
- `data`: Raw pixel array (480 × 640 × 3 = 921,600 bytes)

---

## 6. Hands-On: LiDAR

### 6.1. Start the LiDAR

```bash
ros2 run ydlidar_ros2_driver ydlidar_ros2_driver_node --ros-args \
  --params-file ~/risabotcar_ws/src/risabot_automode/config/ydlidar.yaml \
  -p port:=/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0
```

> **Why do we need both `--params-file` and `-p port`?**
> - The **params file** tells the driver which LiDAR model protocol to use (baudrate, intensity mode, etc.). Without it, the driver uses wrong defaults and produces `Check Sum` errors.
> - The **port override** uses a stable device path. Linux assigns `/dev/ttyUSB0`, `/dev/ttyUSB1` in random order at boot — using `/dev/serial/by-id/...` always finds the LiDAR.

### 6.2. Check the scan topic

```bash
ros2 topic hz /scan
# Should be ~8-12 Hz
```

### 6.3. Read a single scan

```bash
ros2 topic echo /scan --once
```

### 6.4. Understanding the LaserScan message

```text
header:
  stamp: {sec: ..., nanosec: ...}     ← Timestamp
angle_min: -3.14                       ← Start angle (radians, -180°)
angle_max: 3.14                        ← End angle (radians, +180°)
angle_increment: 0.0087                ← Angle between readings (~0.5°)
range_min: 0.05                        ← Minimum valid range (meters)
range_max: 12.0                        ← Maximum valid range (meters)
ranges: [0.45, 0.46, 0.48, ...]       ← Distance array (one per angle)
```

**Interpreting ranges:**
- `ranges[0]` = distance at `angle_min` (behind-left)
- `ranges[180]` = distance at roughly 0° (straight ahead)
- `ranges[360]` = distance at `angle_max` (behind-right)
- `inf` or values > `range_max` = nothing detected

### 6.5. Quick distance check

```python
# In Python, to get the distance directly ahead:
import math
front_index = len(ranges) // 2  # Middle of array = straight ahead
front_distance = ranges[front_index]
```

### 6.6. Visualize LiDAR on the Dashboard

Looking at an array of thousands of numbers in the terminal can be confusing! Luckily, you can use your dashboard to visually see the LiDAR scan.

1. Ensure the `ydlidar_ros2_driver_node` is running in your first terminal.
2. In a second terminal, start your dashboard:
   ```bash
   ros2 run risabot_automode dashboard
   ```
3. Open your laptop's web browser and go to `http://192.168.x.x:8080`.

You will now see the LiDAR canvas drawing red dots in real-time! The dashboard automatically subscribes to the `/scan` topic and converts the distance data into a 2D map of the room around the robot. Try walking around the robot and watch your legs appear on the dashboard!

### 6.7. Adjusting the LiDAR Orientation (Angle Offset)

When you look at the LiDAR canvas on the dashboard, you might notice that objects in front of the robot appear on the **wrong side** of the canvas. This happens because the LiDAR hardware's 0° direction doesn't always match the robot's forward direction — it depends on how the LiDAR is physically mounted on the chassis.

```text
LiDAR mounted with 0° facing backward:

    Physical Reality:              Dashboard (without offset):
    
        FRONT                         "FRONT"
          ↑                              ↑
     ───────────                    ───────────
    |   LiDAR   |                  |   LiDAR   |
    |   0° →    |  ← facing back   |   0° →    |  ← object in front
     ───────────                    ───────────     appears behind!
```

The **`lidar_angle_offset`** parameter corrects this by rotating all readings. It is measured in **radians**:

| LiDAR 0° faces | Offset needed | Value (radians) |
|-----------------|---------------|-----------------|
| Forward | No correction | `0.0` |
| Right (+90°) | Rotate -90° | `1.5708` (π/2) |
| Backward (+180°) | Rotate -180° | `3.1416` (π) |
| Left (-90°) | Rotate +90° | `-1.5708` (-π/2) |

**How to find the correct offset:**

1. Start the LiDAR and dashboard.
2. Stand directly **in front** of the robot.
3. Look at the dashboard LiDAR canvas — where do the red dots (your legs) appear?
4. If they appear at the **bottom** of the canvas instead of the **top**, the offset is wrong.

**How to adjust it:**

The RISA-bot's LiDAR is mounted with 0° pointing **backward**, so the offset is `3.1416` (π = 180°). You can change this at runtime:

```bash
# The dashboard uses this offset for its LiDAR canvas visualization
# Check the current value in params.yaml:
cat ~/risabotcar_ws/src/risabot_automode/config/params.yaml | grep lidar_angle_offset

# The tunnel wall follower and obstacle nodes also use it:
ros2 param set /tunnel_wall_follower lidar_angle_offset 3.1416
```

The offset is configured in `config/params.yaml` under each node that processes LiDAR data:

```yaml
tunnel_wall_follower:
  ros__parameters:
    lidar_angle_offset: 3.1416   # 180° — LiDAR 0° points backward

obstacle_avoidance:
  ros__parameters:
    lidar_angle_offset: 3.1416   # Same offset for obstacle detection
```

> [!TIP]
> If you physically remount the LiDAR (e.g. rotate it 90°), you only need to change this one parameter across the nodes — you don't need to rewire anything!

---

## 7. Testing Your Sensors

Now that everything is running, let's verify the sensors are working accurately! 

**Testing the LiDAR Range:**
1. While watching your dashboard LiDAR canvas, place your hand about **30cm directly in front** of the robot.
2. You will see a cluster of red dots appear very close to the center of the canvas!
3. Now, move your hand slowly to the right side of the robot. You will see the red dots move along the circular canvas in real-time.

**Understanding the Raw Data:**
1. Go back to your SSH terminal and run a single scan:
   ```bash
   ros2 topic echo /scan --once
   ```
2. Scroll through the giant `ranges:` array.
3. **How many readings are there?** The YDLiDAR Tmini Plus outputs roughly 360 to 400 readings per scan, meaning the length of the `ranges` array is around `400`. Each reading corresponds to a fraction of a degree around the robot!
4. If you place your hand in front of the LiDAR and run the command again, you will notice the numbers near the **middle** of the array (e.g., `ranges[200]`) drop from `inf` down to `0.30` (which is 30 centimeters).

---

**Previous:** [Module 1 — Introduction to ROS 2](01-introduction-to-ros.md)
**Next:** [Module 3 — Putting It Together](03-putting-it-together.md)

