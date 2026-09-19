#!/usr/bin/env python3
"""
Obstruction Avoidance Node — Dynamic VFH+ & Bezier Spline Trajectory Planner
Replaces open-loop duration timers with real-time polar sector histogram analysis,
Gaussian smoothing with narrow-gap preservation, hysteresis band switching, and
smooth 3rd-order Bezier curve tracking with continuous LiDAR clearance verification.

Publishes:
  /obstruction_cmd_vel (Twist)
  /obstruction_active (Bool)
"""

import math
import time
from enum import Enum
from typing import Dict, List, Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from nav_msgs.msg import Odometry
from .control_contract import fresh, valid_scan

from .topics import OBSTRUCTION_ACTIVE_TOPIC, OBSTRUCTION_CMD_TOPIC


class AvoidState(Enum):
    CLEAR = 0           # No obstacle detected — lane follow active
    STOP_TOO_CLOSE = 1  # Obstacle too close (<0.20m) for safe curve generation
    SPLINE_DODGE = 2    # Tracking 3rd-order Bezier avoidance curve
    REJOIN_LANE = 3     # Obstacle cleared — returning to lane center


class ObstructionAvoidance(Node):
    """Dynamic VFH+ and Bezier spline obstacle avoidance planner."""

    def __init__(self):
        super().__init__('obstruction_avoidance')

        # --- Parameters ---
        self.declare_parameter('detect_dist', 0.50)           # m — trigger avoidance at this range
        self.declare_parameter('min_safe_dist', 0.20)         # m — degenerate emergency stop threshold
        self.declare_parameter('lateral_offset_m', 0.18)      # m — lateral dodge displacement (>15cm)
        self.declare_parameter('spline_length_m', 0.80)       # m — total longitudinal length of dodge spline
        self.declare_parameter('forward_speed', 0.12)         # m/s — progress speed during dodge
        self.declare_parameter('max_angular', 1.2)            # rad/s — steering clamp
        self.declare_parameter('hysteresis_ratio', 0.15)      # 15% clearance advantage required to flip direction
        self.declare_parameter('lidar_angle_offset', 3.1416)  # 180° backward mount correction
        self.declare_parameter('side_clear_dist', 0.50)       # m — lateral threshold to verify obstacle passed
        self.declare_parameter('max_timeout_sec', 6.0)        # s — bounded timeout safety fallback

        self._param_cache: Dict[str, object] = {}
        self.declare_parameter('wheelbase', 0.14)
        self.declare_parameter('steering_max_deg', 50.0)
        self.declare_parameter('lookahead_m', 0.15)
        self.declare_parameter('sensor_timeout', 0.5)
        self._update_param_cache()
        self.add_on_set_parameters_callback(self._on_params)

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, OBSTRUCTION_CMD_TOPIC, 10)
        self.active_pub = self.create_publisher(Bool, OBSTRUCTION_ACTIVE_TOPIC, 10)

        # Subscriber
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan',
            self.scan_callback,
            QoSPresetProfiles.SENSOR_DATA.value
        )

        # State
        self.state = AvoidState.CLEAR
        self.avoid_dir = 0               # +1 = LEFT, -1 = RIGHT
        self.dodge_start_time = 0.0
        self.dist_progress = 0.0
        self.last_loop_time = self.get_clock().now().nanoseconds / 1e9
        self.lateral_obstacle_present = False
        self.min_forward_dist = 999.0
        self.scan_stamp = self.odom_stamp = 0.0
        self.scan_valid = False
        self.odom_x = self.odom_y = self.odom_yaw = 0.0
        self.start_pose = (0.0, 0.0, 0.0)
        self.selected = False
        self.selected_stamp = 0.0
        self.fault = False
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        self.create_subscription(String, '/dashboard_state', self._state_cb, 10)

        # Timer
        self.timer = self.create_timer(0.05, self.control_loop)  # 20 Hz
        self.get_logger().info('Dynamic VFH+ & Bezier Obstruction Avoidance Node started.')

    def _update_param_cache(self) -> None:
        self._param_cache = {
            'detect_dist': float(self.get_parameter('detect_dist').value),
            'min_safe_dist': float(self.get_parameter('min_safe_dist').value),
            'lateral_offset_m': float(self.get_parameter('lateral_offset_m').value),
            'spline_length_m': float(self.get_parameter('spline_length_m').value),
            'forward_speed': float(self.get_parameter('forward_speed').value),
            'max_angular': float(self.get_parameter('max_angular').value),
            'hysteresis_ratio': float(self.get_parameter('hysteresis_ratio').value),
            'lidar_angle_offset': float(self.get_parameter('lidar_angle_offset').value),
            'side_clear_dist': float(self.get_parameter('side_clear_dist').value),
            'max_timeout_sec': float(self.get_parameter('max_timeout_sec').value),
        }

    def _on_params(self, params) -> SetParametersResult:
        for p in params:
            if p.name in self._param_cache:
                self._param_cache[p.name] = p.value
        return SetParametersResult(successful=True)

    def scan_callback(self, msg: LaserScan) -> None:
        """Evaluate forward polar sector histogram (VFH+) and monitor lateral clearance."""
        angle_offset = float(self._param_cache['lidar_angle_offset'])
        detect_dist = float(self._param_cache['detect_dist'])
        min_safe = float(self._param_cache['min_safe_dist'])
        side_clear = float(self._param_cache['side_clear_dist'])
        self.scan_stamp = time.monotonic()
        self.scan_valid = valid_scan(msg)
        if not self.scan_valid:
            return

        # Build 1D polar histogram: 20 bins across [-50°, +50°]
        n_bins = 20
        sector_density = [0.0] * n_bins
        min_fwd = 999.0
        side_pts_count = 0

        for i, r in enumerate(msg.ranges):
            if not (msg.range_min <= r <= msg.range_max) or math.isnan(r) or math.isinf(r):
                continue

            # Forward arc angle with 180° mounting offset
            angle = msg.angle_min + i * msg.angle_increment + angle_offset
            while angle > math.pi: angle -= 2 * math.pi
            while angle < -math.pi: angle += 2 * math.pi

            # Forward sector evaluation: -50° (-0.87 rad) to +50° (+0.87 rad)
            if -0.87 <= angle <= 0.87:
                if r < min_fwd and -0.35 <= angle <= 0.35:
                    min_fwd = r

                if r <= detect_dist:
                    bin_idx = int((angle + 0.87) / (1.74 / n_bins))
                    bin_idx = max(0, min(n_bins - 1, bin_idx))
                    sector_density[bin_idx] += (1.0 / max(0.1, r))

            # Lateral sector monitoring (checking if obstacle is alongside the car: 45° to 120°)
            if self.avoid_dir != 0:
                side_min = 0.78  # ~45 deg
                side_max = 2.09  # ~120 deg
                # If avoiding LEFT (+1), obstacle is on the RIGHT (negative angle)
                # If avoiding RIGHT (-1), obstacle is on the LEFT (positive angle)
                check_angle = -self.avoid_dir * angle
                if side_min <= check_angle <= side_max and r <= side_clear:
                    side_pts_count += 1

        self.min_forward_dist = min_fwd
        self.lateral_obstacle_present = (side_pts_count >= 3)

        # Apply [0.2, 0.6, 0.2] Gaussian smoothing to polar histogram (preserves 40cm gaps)
        smoothed = [0.0] * n_bins
        for k in range(n_bins):
            left_val = sector_density[k-1] if k > 0 else sector_density[0]
            right_val = sector_density[k+1] if k < n_bins - 1 else sector_density[-1]
            smoothed[k] = 0.2 * left_val + 0.6 * sector_density[k] + 0.2 * right_val

        # Compute Left clearance (bins n_bins//2 to end) vs Right clearance (bins 0 to n_bins//2)
        mid = n_bins // 2
        right_density = sum(smoothed[:mid])
        left_density = sum(smoothed[mid:])

        # State transition trigger from CLEAR
        if self.state == AvoidState.CLEAR:
            if not self.selected and not fresh(self.odom_stamp, time.monotonic(), 0.5):
                return
            if min_fwd < min_safe:
                # Degenerate: too close to curve safely
                self.get_logger().warn(f"Obstacle critically close ({min_fwd:.2f}m) -> Emergency Yield")
                self._start_state(AvoidState.STOP_TOO_CLOSE)
            elif min_fwd <= detect_dist and (left_density > 2.0 or right_density > 2.0):
                # Pick direction with Hysteresis
                hyst = float(self._param_cache['hysteresis_ratio'])
                if left_density < right_density * (1.0 - hyst):
                    self.avoid_dir = 1   # Steer LEFT (left has less obstacle density)
                elif right_density < left_density * (1.0 - hyst):
                    self.avoid_dir = -1  # Steer RIGHT
                else:
                    self.avoid_dir = 1 if left_density <= right_density else -1

                dir_str = "LEFT" if self.avoid_dir > 0 else "RIGHT"
                self.get_logger().info(f"Obstruction detected at {min_fwd:.2f}m -> Generating Bezier Spline ({dir_str})")
                self._start_state(AvoidState.SPLINE_DODGE)

    def _start_state(self, new_state: AvoidState) -> None:
        self.state = new_state
        self.dodge_start_time = self.get_clock().now().nanoseconds / 1e9
        self.dist_progress = 0.0
        self.start_pose = (self.odom_x, self.odom_y, self.odom_yaw)

    def _state_cb(self, msg):
        self.selected = msg.data.split('|')[0] == 'OBSTRUCTION'
        self.selected_stamp = time.monotonic()
        if msg.data.split('|')[0] == 'MANUAL':
            self.fault = False
            self.state = AvoidState.CLEAR

    def _odom_cb(self, msg):
        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        if not all(math.isfinite(v) for v in (x, y, q.x, q.y, q.z, q.w)):
            self.odom_stamp = 0.0
            return
        if self.selected and self.odom_stamp > 0:
            delta = math.hypot(x - self.odom_x, y - self.odom_y)
            if delta < 0.1:
                self.dist_progress += delta
        self.odom_x, self.odom_y = x, y
        self.odom_yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        self.odom_stamp = time.monotonic()

    def _eval_bezier(self, t: float, lat_m: float, len_m: float) -> Tuple[float, float, float]:
        """
        Evaluate 3rd-order Bezier curve B(t) = (x(t), y(t)) and tangent angle psi(t).
        P0 = (0, 0)
        P1 = (4/3 * lat_m, 0.33 * len_m)
        P2 = (4/3 * lat_m, 0.66 * len_m)
        P3 = (0, len_m)
        This ensures apex displacement at t=0.5 is exactly equal to lat_m (e.g. 0.18m >= 15cm).
        """
        t = max(0.0, min(1.0, t))
        omt = 1.0 - t
        scale_lat = (4.0 / 3.0) * lat_m

        p0 = (0.0, 0.0)
        p1 = (scale_lat, 0.33 * len_m)
        p2 = (scale_lat, 0.66 * len_m)
        p3 = (0.0, len_m)

        # Position (x lateral, y forward)
        x = (omt**3)*p0[0] + 3*(omt**2)*t*p1[0] + 3*omt*(t**2)*p2[0] + (t**3)*p3[0]
        y = (omt**3)*p0[1] + 3*(omt**2)*t*p1[1] + 3*omt*(t**2)*p2[1] + (t**3)*p3[1]

        # Derivatives dx/dt, dy/dt
        dx = 3*(omt**2)*(p1[0] - p0[0]) + 6*omt*t*(p2[0] - p1[0]) + 3*(t**2)*(p3[0] - p2[0])
        dy = 3*(omt**2)*(p1[1] - p0[1]) + 6*omt*t*(p2[1] - p1[1]) + 3*(t**2)*(p3[1] - p2[1])

        tangent_angle = math.atan2(dx, max(0.01, dy))
        return x, y, tangent_angle

    def control_loop(self) -> None:
        """Track a local curve using measured pose and continuously check clearance."""
        now = self.get_clock().now().nanoseconds / 1e9
        cmd = Twist()
        active = self.state != AvoidState.CLEAR
        ttl = float(self.get_parameter('sensor_timeout').value)
        observations_ok = (self.scan_valid and fresh(self.scan_stamp, time.monotonic(), ttl)
                           and fresh(self.odom_stamp, time.monotonic(), ttl))
        if active and not observations_ok:
            self.fault = True
        if active and now - self.dodge_start_time > self._param_cache['max_timeout_sec']:
            self.fault = True
        if active and self.min_forward_dist <= self._param_cache['min_safe_dist']:
            self.fault = True
        if self.state == AvoidState.STOP_TOO_CLOSE:
            # No automatic timed release from an obstacle stop.
            self.fault = True
        if active and not self.fault and observations_ok:
            # Do not command or advance a maneuver while another behavior owns motion.
            if self.selected and fresh(self.selected_stamp, time.monotonic(), 0.6):
                length = self._param_cache['spline_length_m']
                lookahead = float(self.get_parameter('lookahead_m').value)
                t = min(1.0, (self.dist_progress + lookahead) / max(0.1, length))
                lateral, forward, _ = self._eval_bezier(t, self.avoid_dir * self._param_cache['lateral_offset_m'], length)
                sx, sy, yaw = self.start_pose
                gx = sx + forward*math.cos(yaw) - lateral*math.sin(yaw)
                gy = sy + forward*math.sin(yaw) + lateral*math.cos(yaw)
                dx, dy = gx-self.odom_x, gy-self.odom_y
                local_x = dx*math.cos(self.odom_yaw) + dy*math.sin(self.odom_yaw)
                local_y = -dx*math.sin(self.odom_yaw) + dy*math.cos(self.odom_yaw)
                curvature = 2*local_y / max(0.01, local_x*local_x + local_y*local_y)
                angle = math.atan(float(self.get_parameter('wheelbase').value)*curvature)
                cmd.angular.z = max(-1., min(1., -angle/math.radians(float(self.get_parameter('steering_max_deg').value))))
                cmd.linear.x = self._param_cache['forward_speed']
                endpoint_distance = math.hypot(self.odom_x-(sx+length*math.cos(yaw)), self.odom_y-(sy+length*math.sin(yaw)))
                if self.dist_progress >= length*.9 and endpoint_distance < 0.08 and not self.lateral_obstacle_present:
                    self.state = AvoidState.CLEAR
                    active = False
                    cmd = Twist()
        self.cmd_vel_pub.publish(cmd)
        self.active_pub.publish(Bool(data=active))


def main(args=None):
    rclpy.init(args=args)
    node = ObstructionAvoidance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
