#!/usr/bin/env python3
"""Use the production YOLO11 decoder on live images; never command motors.

Source the installed workspace first. Model path can be set with standard ROS
arguments: --ros-args -p model_path:=/path/to/model.bin
"""
import time
import rclpy
from risabot_automode.signage_detector import SignageDetector


def main():
    rclpy.init()
    node = SignageDetector()
    deadline = time.monotonic()+30
    good = False
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
            if node.last_observation > 0:
                print('Production decode accepted a live frame; this is not an accuracy evaluation')
                good = True
                break
    finally:
        node.destroy_node()
        rclpy.shutdown()
    if not good:
        raise SystemExit('No successful live decode within 30 seconds')


if __name__ == '__main__':
    main()
