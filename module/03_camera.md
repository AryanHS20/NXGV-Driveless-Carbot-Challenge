# Module 3: Depth Camera Operations & Image Processing

RISA-bot is equipped with an Orbbec Astra Mini Depth Camera. This sensor provides both standard RGB color video and structured-light depth frames, which are crucial for detecting obstacles, signage, and lane boundaries.

---

## 1. Launching the Camera Node

The camera node is wrapped in the `astra_camera` package.

### 1.1. Run the Launch File
To start the camera feed on the robot, SSH into your RDK X5 terminal and execute:
```bash
ros2 launch astra_camera astra_mini.launch.py
```

### 1.2. Verify the Topics
Open a second terminal, source your workspace, and list the active camera topics:
```bash
ros2 topic list | grep camera
```
You should see output similar to this:
```text
/camera/color/camera_info
/camera/color/image_raw
/camera/depth/camera_info
/camera/depth/image_raw
```

### 1.3. Check the Update Rate (Hz)
Verify the camera is publishing at its target frame rate (normally ~30 Hz):
```bash
ros2 topic hz /camera/color/image_raw
```

---

## 2. Visualizing the Camera Feed

Since RISA-bot is operated headless, you can view the video feed in two ways:

### 2.1. Web Dashboard (Recommended)
Open a web browser on your laptop and go to `http://<robot_ip>:8080`. Under the camera stream window, select the **Raw Camera** or **Lane Lines** view. The dashboard server takes the ROS 2 image topic, compresses it into JPEG bytes, and streams it live to the web page.

### 2.2. Graphical Tools (rqt_image_view)
If you are connected to the robot using a remote desktop environment like **NoMachine**, or have X11 forwarding configured:
1. Open a terminal inside the remote desktop session.
2. Run the image viewer:
   ```bash
   ros2 run rqt_image_view rqt_image_view
   ```
3. In the dropdown menu on the top-left, select `/camera/color/image_raw` to see the live feed.

![rqt_image_view window displaying raw camera stream](images/rqt_image_view_raw_feed.png)
<!-- Image Description: Screenshot of rqt_image_view displaying a live color frame of the track, showing the dropdown menu with active image topics. -->

---

## 3. Subscribing and Processing Image Data in Python

To process camera images using OpenCV, you must convert the ROS `sensor_msgs/msg/Image` type into a standard NumPy array using a tool called **`cv_bridge`**.

Below is a complete, runnable python node that subscribes to `/camera/color/image_raw`, converts it to grayscale using OpenCV, and publishes the grayscale image on a new topic called `/camera/gray/image_raw`.

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class GrayImageProcessor(Node):
    def __init__(self):
        super().__init__('gray_image_processor')
        
        # Create subscriber to raw camera topic
        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10
        )
        
        # Create publisher for the processed gray image
        self.publisher_ = self.create_publisher(Image, '/camera/gray/image_raw', 10)
        
        # Initialize the CvBridge
        self.bridge = CvBridge()
        
        self.get_logger().info('Gray Image Processor Node Started.')

    def image_callback(self, msg):
        try:
            # 1. Convert ROS Image message to OpenCV BGR image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # 2. Process image using standard OpenCV (convert to grayscale)
            gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            
            # 3. Add debug info (e.g. print dimensions)
            h, w = gray_image.shape[:2]
            
            # 4. Convert OpenCV image back to ROS Image message
            out_msg = self.bridge.cv2_to_imgmsg(gray_image, encoding='mono8')
            out_msg.header = msg.header # Preserve timestamp and frame ID
            
            # 5. Publish the processed image
            self.publisher_.publish(out_msg)
            
        except Exception as e:
            self.get_logger().error(f'Failed to process frame: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = GrayImageProcessor()
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
**Previous:** [Module 2 — The RISA-bot Controller](02_controller.md)
**Next:** [Module 4 — LiDAR Operations](04_lidar.md)
