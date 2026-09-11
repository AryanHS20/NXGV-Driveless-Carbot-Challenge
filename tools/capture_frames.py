#!/usr/bin/env python3

import os
import time
from datetime import datetime

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

class FrameCaptureNode(Node):
    def __init__(self):
        super().__init__('frame_capture_node')
        
        # Subscribe to the camera topic
        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10
        )
        
        self.bridge = CvBridge()
        self.last_capture_time = time.time()
        
        # Create output directory
        self.output_dir = os.path.join(os.getcwd(), 'captured_frames')
        os.makedirs(self.output_dir, exist_ok=True)
        self.get_logger().info(f"Saving frames to: {self.output_dir}")
        self.get_logger().info("Will capture 1 frame every 2 seconds. Press Ctrl+C to stop.")

    def image_callback(self, msg):
        current_time = time.time()
        
        # Check if 2 seconds have passed since the last capture
        if (current_time - self.last_capture_time) >= 2.0:
            try:
                # Convert ROS Image message to OpenCV image
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                
                # Generate a filename with the current timestamp
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
                filename = os.path.join(self.output_dir, f"frame_{timestamp}.jpg")
                
                # Save the image
                cv2.imwrite(filename, cv_image)
                self.get_logger().info(f"Captured: {filename}")
                
                # Update the last capture time
                self.last_capture_time = current_time
                
            except Exception as e:
                self.get_logger().error(f"Error capturing frame: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = FrameCaptureNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Stopping frame capture...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
