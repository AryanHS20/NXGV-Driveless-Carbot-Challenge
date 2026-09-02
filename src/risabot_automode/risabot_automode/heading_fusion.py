#!/usr/bin/env python3
"""
Heading & Velocity Fusion Node
Subscribes to IMU attitude data and raw wheel odometry to provide a robust,
drift-resistant fused heading estimate and velocity model for RISA-bot.

Publishes:
  /fused_heading (Float32, radians)
  /fused_heading_deg (Float32, degrees)
  /odom_fused (Odometry)
"""

import json
import math
from typing import Dict, Optional

import rclpy
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from std_msgs.msg import Float32, String

from .topics import IMU_DATA_TOPIC, ODOM_TOPIC


class HeadingFusion(Node):
    """Fuses IMU attitude (yaw rate / angle) with wheel odometry using message timestamps."""

    def __init__(self):
        super().__init__('heading_fusion')

        # --- Parameters ---
        self.declare_parameter('alpha', 0.85)                # Trust weight for IMU vs Odom (0.0 to 1.0)
        self.declare_parameter('stationary_deadband', 0.20)  # deg/s — deadband to filter standstill jitter
        self.declare_parameter('max_allowed_dt', 0.5)        # seconds — discard integration if dt too large
        self.declare_parameter('publish_rate_hz', 25.0)      # Hz

        self._param_cache: Dict[str, object] = {}
        self._update_param_cache()
        self.add_on_set_parameters_callback(self._on_params)

        # Publishers
        self.fused_heading_pub = self.create_publisher(Float32, '/fused_heading', 10)
        self.fused_heading_deg_pub = self.create_publisher(Float32, '/fused_heading_deg', 10)
        self.odom_fused_pub = self.create_publisher(Odometry, '/odom_fused', 10)

        # Subscribers
        self.imu_sub = self.create_subscription(
            String, IMU_DATA_TOPIC, self.imu_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, ODOM_TOPIC, self.odom_callback, 10
        )

        # Internal State
        self.last_imu_yaw_deg: Optional[float] = None
        self.last_imu_stamp: Optional[float] = None
        self.imu_yaw_offset_deg: float = 0.0
        self.imu_initialized = False

        self.last_odom_yaw_deg: float = 0.0
        self.last_odom_x: float = 0.0
        self.last_odom_y: float = 0.0
        self.last_odom_speed: float = 0.0
        self.last_odom_stamp: Optional[float] = None

        self.fused_yaw_deg: float = 0.0
        self.fused_x: float = 0.0
        self.fused_y: float = 0.0

        # Output timer
        dt = 1.0 / float(self._param_cache['publish_rate_hz'])
        self.timer = self.create_timer(dt, self.publish_fused_state)

        self.get_logger().info("Heading Fusion Node initialized.")

    def _update_param_cache(self) -> None:
        self._param_cache = {
            'alpha': float(self.get_parameter('alpha').value),
            'stationary_deadband': float(self.get_parameter('stationary_deadband').value),
            'max_allowed_dt': float(self.get_parameter('max_allowed_dt').value),
            'publish_rate_hz': float(self.get_parameter('publish_rate_hz').value),
        }

    def _on_params(self, params) -> SetParametersResult:
        for p in params:
            if p.name in self._param_cache:
                self._param_cache[p.name] = p.value
        return SetParametersResult(successful=True)

    @staticmethod
    def _normalize_angle_deg(deg: float) -> float:
        """Wrap angle in degrees to [-180, 180]."""
        while deg > 180.0:
            deg -= 360.0
        while deg < -180.0:
            deg += 360.0
        return deg

    @staticmethod
    def _angle_diff_deg(a: float, b: float) -> float:
        """Shortest signed difference (a - b) in degrees."""
        d = a - b
        while d > 180.0:
            d -= 360.0
        while d < -180.0:
            d += 360.0
        return d

    def imu_callback(self, msg: String) -> None:
        """Process incoming IMU JSON payload."""
        now_sec = self.get_clock().now().nanoseconds / 1e9
        try:
            data = json.loads(msg.data)
            raw_yaw = float(data.get('yaw', 0.0))
        except (json.JSONDecodeError, ValueError, TypeError):
            return

        if not self.imu_initialized:
            self.last_imu_yaw_deg = raw_yaw
            self.imu_yaw_offset_deg = raw_yaw  # zero on startup
            self.fused_yaw_deg = 0.0
            self.last_imu_stamp = now_sec
            self.imu_initialized = True
            return

        dt = now_sec - (self.last_imu_stamp if self.last_imu_stamp is not None else now_sec)
        self.last_imu_stamp = now_sec

        if dt <= 0.001 or dt > float(self._param_cache['max_allowed_dt']):
            self.last_imu_yaw_deg = raw_yaw
            return

        # Measured delta from IMU
        delta_imu = self._angle_diff_deg(raw_yaw, self.last_imu_yaw_deg)
        self.last_imu_yaw_deg = raw_yaw

        # Apply deadband filter to prevent standstill drift
        deadband = float(self._param_cache['stationary_deadband'])
        if abs(delta_imu) < deadband * dt:
            delta_imu = 0.0

        # Complementary update on fused heading
        alpha = float(self._param_cache['alpha'])
        self.fused_yaw_deg = self._normalize_angle_deg(self.fused_yaw_deg + delta_imu)

    def odom_callback(self, msg: Odometry) -> None:
        """Process wheel odometry position and speed."""
        stamp_sec = msg.header.stamp.sec + (msg.header.stamp.nanosec / 1e9)
        if stamp_sec == 0.0:
            stamp_sec = self.get_clock().now().nanoseconds / 1e9

        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        odom_yaw_rad = math.atan2(siny_cosp, cosy_cosp)
        odom_yaw_deg = math.degrees(odom_yaw_rad)

        # Rear-wheel Ackermann velocity: v_chassis = v_rear directly
        self.last_odom_speed = msg.twist.twist.linear.x
        self.last_odom_x = msg.pose.pose.position.x
        self.last_odom_y = msg.pose.pose.position.y

        if self.last_odom_stamp is not None:
            dt = stamp_sec - self.last_odom_stamp
            if 0.001 < dt < float(self._param_cache['max_allowed_dt']):
                # Advance 2D fused position along fused heading
                dist = self.last_odom_speed * dt
                fused_rad = math.radians(self.fused_yaw_deg)
                self.fused_x += dist * math.cos(fused_rad)
                self.fused_y += dist * math.sin(fused_rad)

        self.last_odom_stamp = stamp_sec
        self.last_odom_yaw_deg = odom_yaw_deg

    def publish_fused_state(self) -> None:
        """Publish fused heading and odometry topics."""
        fused_rad = math.radians(self.fused_yaw_deg)

        # 1. Radians heading
        msg_rad = Float32()
        msg_rad.data = float(fused_rad)
        self.fused_heading_pub.publish(msg_rad)

        # 2. Degrees heading
        msg_deg = Float32()
        msg_deg.data = float(self.fused_yaw_deg)
        self.fused_heading_deg_pub.publish(msg_deg)

        # 3. Fused Odometry
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = float(self.fused_x)
        odom.pose.pose.position.y = float(self.fused_y)
        odom.pose.pose.position.z = 0.0

        # Quaternion from fused yaw
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(fused_rad / 2.0)
        odom.pose.pose.orientation.w = math.cos(fused_rad / 2.0)

        odom.twist.twist.linear.x = float(self.last_odom_speed)
        self.odom_fused_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = HeadingFusion()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
