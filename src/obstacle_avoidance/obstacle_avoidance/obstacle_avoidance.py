#!/usr/bin/env python3
"""
Obstacle Avoidance Node
=======================
Uses LiDAR scans to detect obstacles in the immediate front path of the robot.
Publishes boolean flags if an obstacle is closer than the configured minimum distance.
Applies temporal smoothing to avoid false positives.
"""

import math
from typing import Dict

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

class ObstacleAvoidanceNode(Node):
    """
    Subscribes to `/scan` and publishes `/obstacle_front` and `/obstacle_detected`.
    Filters out side/rear points and focuses on a narrow forward cone.
    """
    def __init__(self):
        super().__init__('obstacle_avoidance_node')

        # Parameters (tunable at runtime)
        self.declare_parameter('min_obstacle_distance', 0.48)
        self.declare_parameter('heartbeat_sec', 0.5)
        self._param_cache: Dict[str, object] = {}
        self._update_param_cache()
        self.add_on_set_parameters_callback(self._on_params)

        # Publishers
        self.obstacle_all_pub = self.create_publisher(Bool, '/obstacle_detected', 10)
        self.obstacle_front_pub = self.create_publisher(Bool, '/obstacle_front', 10)

        # Subscriber
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            QoSPresetProfiles.SENSOR_DATA.value
        )

        # State — temporal smoothing buffer
        self.obstacle_active_any = False
        self.distance_buffer = []
        self.buffer_size = 5
        self._heartbeat_timer = self.create_timer(
            float(self._param_cache['heartbeat_sec']),
            self._heartbeat_publish
        )

        self.get_logger().info('Obstacle Avoidance Node Started (Precise Directional Detection)')

    def _update_param_cache(self) -> None:
        """Cache frequently used parameters to avoid per-scan lookups."""
        self._param_cache = {
            'min_obstacle_distance': float(self.get_parameter('min_obstacle_distance').value),
            'heartbeat_sec': float(self.get_parameter('heartbeat_sec').value),
        }

    def _on_params(self, params) -> SetParametersResult:
        """Update cached parameters when set via CLI or services."""
        for p in params:
            if p.name in self._param_cache:
                self._param_cache[p.name] = p.value
        return SetParametersResult(successful=True)

    def _heartbeat_publish(self) -> None:
        """Publish last obstacle state on a fixed heartbeat."""
        self.obstacle_front_pub.publish(Bool(data=self.obstacle_active_any))
        self.obstacle_all_pub.publish(Bool(data=self.obstacle_active_any))

    def scan_callback(self, msg: LaserScan) -> None:
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        ranges = msg.ranges

        min_front = float('inf')

        # Read parameter once per callback
        min_dist = self._param_cache['min_obstacle_distance']

        for i, r in enumerate(ranges):
            if not (msg.range_min <= r <= msg.range_max) or math.isnan(r) or math.isinf(r):
                continue

            angle = angle_min + i * angle_increment
            # Compensate for 90° clockwise mount
            angle += math.pi / 2

            # Normalize
            while angle > math.pi:
                angle -= 2 * math.pi
            while angle < -math.pi:
                angle += 2 * math.pi

            # ✅ Robot front = angle near 0 (±30° = ±π/6)
            if -math.pi/6 <= angle <= math.pi/6:
                min_front = min(min_front, r)

        # Only add to buffer if we got a valid (non-corrupt) reading
        # Corrupt LiDAR packets often produce 0.0 or inf — ignore them
        if min_front < float('inf') and min_front > 0.05:
            self.distance_buffer.append(min_front)
        elif min_front == float('inf'):
            # No valid reading in front arc — treat as clear
            self.distance_buffer.append(float('inf'))

        if len(self.distance_buffer) > self.buffer_size:
            self.distance_buffer.pop(0)

        # Use MEDIAN instead of min — one bad packet won't poison the buffer
        valid = [d for d in self.distance_buffer if d < float('inf')]
        if valid:
            smoothed_min = sorted(valid)[len(valid) // 2]  # median
        else:
            smoothed_min = float('inf')  # no data = clear

        # Publish front-only detection
        front_obstacle = smoothed_min < min_dist

        # Debug: log only when obstacle detected
        if front_obstacle:
            self.get_logger().warn(f'FRONT obstacle. min_front = {smoothed_min:.2f} m')

        # Publish
        msg_front = Bool()
        msg_front.data = front_obstacle
        self.obstacle_front_pub.publish(msg_front)

        # Also publish any-obstacle
        self.obstacle_all_pub.publish(Bool(data=front_obstacle))
        self.obstacle_active_any = front_obstacle

def main(args=None) -> None:
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
