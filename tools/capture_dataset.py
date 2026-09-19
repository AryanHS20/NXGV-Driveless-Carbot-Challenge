#!/usr/bin/env python3
"""Dataset capture for traffic-light retraining.

Runs ON the robot (subscribes to the live camera topic) and saves full-
resolution frames into datasets/<label>/, stopping automatically at --count.

Labels requested by the training side:
  green / red / yellow / empty   (~100 images each, varied angles/distances/lighting)

Usage (on the robot, bringup already running):
  python3 capture_dataset.py --label green --count 100
  python3 capture_dataset.py --label empty --count 100 --interval 1.0 --out ~/datasets

Press Ctrl+C to stop early; progress is printed every capture.
"""

import argparse
import os
import time
from datetime import datetime

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import Image


class DatasetCaptureNode(Node):
    def __init__(self, label: str, out_root: str, interval: float, count: int):
        super().__init__('dataset_capture_node')

        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            QoSPresetProfiles.SENSOR_DATA.value
        )

        self.bridge = CvBridge()
        self.interval = interval
        self.target = count
        self.saved = 0
        self.last_capture_time = 0.0  # capture first frame immediately

        self.output_dir = os.path.join(os.path.expanduser(out_root), label)
        os.makedirs(self.output_dir, exist_ok=True)
        self.get_logger().info(
            f"Saving [{label}] frames to: {self.output_dir} "
            f"(1 every {interval:.1f}s, target {count})")
        self.done = False

    def image_callback(self, msg):
        if self.done:
            return
        current_time = time.time()
        if (current_time - self.last_capture_time) < self.interval:
            return
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
            filename = os.path.join(self.output_dir, f"{timestamp}.jpg")
            if not cv2.imwrite(filename, cv_image):
                raise IOError('Image write failed: ' + filename)
            self.saved += 1
            self.last_capture_time = current_time
            self.get_logger().info(f"[{self.saved}/{self.target}] {filename}")
            if self.saved >= self.target:
                self.get_logger().info("Target reached, shutting down.")
                self.done = True
                raise KeyboardInterrupt
        except KeyboardInterrupt:
            raise
        except Exception as e:
            self.get_logger().error(f"Error capturing frame: {str(e)}")


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--label', required=True,
                        choices=['green', 'red', 'yellow', 'empty'],
                        help='Which dataset folder to capture into')
    parser.add_argument('--count', type=int, default=100,
                        help='Stop after this many images (default: 100)')
    parser.add_argument('--interval', type=float, default=2.0,
                        help='Seconds between captures (default: 2.0)')
    parser.add_argument('--out', default='~/datasets',
                        help='Dataset root dir (default: ~/datasets)')
    cli = parser.parse_args()

    rclpy.init(args=args)
    node = DatasetCaptureNode(cli.label, cli.out, cli.interval, cli.count)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.5)
    except KeyboardInterrupt:
        node.get_logger().info(f"Stopped early at {node.saved}/{node.target}.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
