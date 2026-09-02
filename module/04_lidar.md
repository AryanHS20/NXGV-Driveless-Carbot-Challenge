# Module 4: LiDAR Operations & Map Processing

RISA-bot uses a 2D LiDAR (Light Detection and Ranging) sensor, specifically the **YDLiDAR Tmini Plus**. This sensor spins 360 degrees and uses laser beams to measure distances, producing a flat 2D slice of the environment. It is the primary sensor for obstacle avoidance, wall following, and boom gate detection.

---

## 1. Triggering the LiDAR Driver

The driver node is named `ydlidar_ros2_driver_node`.

### 1.1. Run the LiDAR Driver Command
SSH into the robot, source the workspace, and run the driver:
```bash
ros2 run ydlidar_ros2_driver ydlidar_ros2_driver_node --ros-args \
  --params-file ~/risabotcar_ws/src/risabot_automode/config/ydlidar.yaml \
  -p port:=/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0
```

> [!IMPORTANT]
> **Why do we specify the port via `by-id`?**
> The operating system randomly assigns device ports like `/dev/ttyUSB0` or `/dev/ttyUSB1` when the robot boots up. If you configure a static path like `/dev/ttyUSB0`, the LiDAR driver might accidentally attempt to communicate with the motor controller serial port instead. The stable ID path (`/dev/serial/by-id/usb-Silicon_Labs...`) is linked directly to the physical LiDAR interface chip and never changes.

### 1.2. Verify Scan Topic
Ensure the `/scan` topic is active and publishing data at approximately 8 to 12 Hz:
```bash
ros2 topic hz /scan
```

---

## 2. Understanding the LaserScan Message

The LiDAR publishes messages of type `sensor_msgs/msg/LaserScan` on the `/scan` topic. You can check a single raw scan message structure using:
```bash
ros2 topic echo /scan --once
```

Key metadata variables include:
- `angle_min`: Starting angle of the scan in radians (usually `-3.14159` or -180°).
- `angle_max`: Ending angle of the scan in radians (usually `3.14159` or +180°).
- `angle_increment`: Angular distance between successive beams in radians (~0.0087 rad or 0.5°).
- `ranges`: A dynamic float array containing distance measurements in meters. 
  - An element with value `inf` or `nan` means the beam did not hit anything within the maximum range limit.

---

## 3. LiDAR Mounting & Angle Offset Calibration

The LiDAR is physically mounted on RISA-bot with its 0° reference pointing **backward**.

Because of this physical orientation, objects located directly in front of the robot will register near the starting or ending indices of the `ranges` array rather than in the middle. To align the scan coordinate system with the robot's physical heading, software nodes use a `lidar_angle_offset` parameter.

```text
    Physical Reality:                     LiDAR Coordinate Frame:
    
          FRONT (12 o'clock)                       BACK (0° / 360°)
               ↑                                         ↑
         ─────────────                             ─────────────
        |    Robot    |                           |    Robot    |
        |  LiDAR (0°) | ← facing back             |  LiDAR (0°) | ← 0° is here!
         ─────────────                             ─────────────
               ↓                                         ↓
      BACK (6 o'clock / 0°)                     FRONT (180° / 3.14 rad)
```

- **Offset Value**: `3.1416` (radians, equal to 180°). This value is passed into the `tunnel_wall_follower` and `obstacle_avoidance` nodes so they look forward instead of backward.

![Dashboard LiDAR Visualization Canvas](images/dashboard_lidar_canvas.png)
<!-- Image Description: Screenshot of the dashboard showing the LiDAR visualization window, displaying red dots forming walls/obstacles relative to the blue triangle robot icon at the center. -->

---

## 4. Subscribing to /scan in Python

Below is a complete python node that subscribes to `/scan` and prints the distance to the nearest object directly in front of the robot (with the 180-degree offset correction).

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

class LidarTestNode(Node):
    def __init__(self):
        super().__init__('lidar_test_node')
        
        # Subscribe to /scan topic
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value # Sensor data profile
        )
        
        # LiDAR is mounted backward, offset = 180° (pi radians)
        self.lidar_angle_offset = 3.14159

    def scan_callback(self, msg):
        # 1. Determine index corresponding to the front of the robot (180 degrees)
        # angle = angle_min + index * angle_increment
        # index = (angle - angle_min) / angle_increment
        
        target_angle_rad = self.lidar_angle_offset
        
        if target_angle_rad < msg.angle_min or target_angle_rad > msg.angle_max:
            self.get_logger().warn("Target angle is outside scan limits!")
            return
            
        # Calculate array index
        front_idx = int((target_angle_rad - msg.angle_min) / msg.angle_increment)
        
        # Guard index boundaries
        if 0 <= front_idx < len(msg.ranges):
            front_distance = msg.ranges[front_idx]
            
            # Filter out out-of-range/invalid laser data
            if math.isinf(front_distance) or math.isnan(front_distance):
                self.get_logger().info("Front Path: Clear (No obstacles detected)")
            else:
                self.get_logger().info(f"Object detected in front! Distance: {front_distance:.2f} meters")

def main(args=None):
    rclpy.init(args=args)
    node = LidarTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```
---
**Previous:** [Module 3 — Camera Operations](03_camera.md)
**Next:** [Module 5 — AI Detection & BPU](05_ai_detection.md)
