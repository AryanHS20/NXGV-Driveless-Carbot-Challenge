#!/usr/bin/env python3
"""Safety envelope node for autonomous velocity commands."""

import json
import math
import time
from typing import Dict

import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from std_msgs.msg import Bool, String
from sensor_msgs.msg import Image, LaserScan
from rclpy.qos import QoSPresetProfiles
from .control_contract import fresh, valid_scan, swept_path_clear, observation_age

from .loop_monitor import LoopMonitor
from .topics import (
    AUTO_CMD_VEL_RAW_TOPIC,
    AUTO_CMD_VEL_TOPIC,
    CMD_SAFETY_STATUS_TOPIC,
    E_STOP_TOPIC,
    LOOP_STATS_TOPIC,
    V4_CMD_VEL_RAW_TOPIC,
)


class CmdSafetyController(Node):
    """Applies limits, timeout handling, and e-stop gating to autonomous cmd_vel."""

    def __init__(self) -> None:
        super().__init__('cmd_safety_controller')

        self.declare_parameter('publish_hz', 50.0)
        self.declare_parameter('cmd_timeout', 0.35)
        self.declare_parameter('max_linear_speed', 0.30)
        self.declare_parameter('max_angular_speed', 1.2)
        self.declare_parameter('max_linear_accel', 0.8)
        self.declare_parameter('max_angular_accel', 4.0)
        self.declare_parameter('deadband_linear', 0.01)
        self.declare_parameter('deadband_angular', 0.01)
        self.declare_parameter('publish_loop_stats', True)
        self.declare_parameter('sensor_timeout', 0.5)
        self.declare_parameter('min_scan_points', 20)
        self.declare_parameter('require_signage', True)
        # Explicit command authority. It ships as legacy and may only be
        # changed to v4 after Stage 8 is running and its physical gates pass.
        self.declare_parameter('autonomy_source', 'legacy')
        self.declare_parameter('lidar_angle_offset', math.pi)
        self.declare_parameter('footprint_half_width', .12)
        self.declare_parameter('footprint_front', .22)
        self.declare_parameter('footprint_rear', .22)
        self.declare_parameter('sweep_distance', .15)
        self._param_cache: Dict[str, object] = {}
        self._update_param_cache()
        self.add_on_set_parameters_callback(self._on_params)

        self.cmd_pub = self.create_publisher(Twist, AUTO_CMD_VEL_TOPIC, 10)
        self.status_pub = self.create_publisher(String, CMD_SAFETY_STATUS_TOPIC, 10)
        self.loop_stats_pub = self.create_publisher(String, LOOP_STATS_TOPIC, 10)

        self.create_subscription(Twist, AUTO_CMD_VEL_RAW_TOPIC, self._raw_cmd_cb, 10)
        self.create_subscription(Twist, V4_CMD_VEL_RAW_TOPIC, self._v4_cmd_cb, 10)
        self.create_subscription(Twist, '/playback_cmd_vel_raw', self._playback_cmd_cb, 10)
        self.create_subscription(String, '/record_playback_state', self._playback_state_cb, 10)
        self.playback_active = False
        self.playback_cmd = Twist()
        self.playback_cmd_stamp = 0.0
        self.playback_state_stamp = 0.0
        self.playback_started = 0.0
        self.create_subscription(Bool, E_STOP_TOPIC, self._estop_cb, 10)
        self.permit_pub = self.create_publisher(Bool, '/motion_permitted', 10)
        qos = QoSPresetProfiles.SENSOR_DATA.value
        self.create_subscription(Image, '/camera/color/image_raw', self._image_cb, qos)
        self.create_subscription(LaserScan, '/scan', self._scan_cb, qos)
        self.last_image_t = self.last_scan_t = 0.0
        self.scan_valid = False
        self.last_scan = None
        self.servo_geometry = {'wheelbase': .14, 'steering_max_deg': 50., 'right_boost': 1.3}
        self.create_subscription(String, '/servo_geometry', self._geometry_cb, 10)
        self.signage_valid = False
        self.signage_stamp = 0.0
        self.create_subscription(Bool, '/signage_valid', self._signage_cb, 10)

        self.target_cmd = Twist()
        self.v4_target_cmd = Twist()
        self.v4_last_input_t = 0.0
        self.output_cmd = Twist()
        self.estop = False
        self.last_input_t = 0.0
        self.last_loop_t = time.monotonic()
        self.timeout_count = 0
        self.estop_count = 0
        self.limit_count = 0

        hz = float(self._param_cache['publish_hz'])
        self.loop_monitor = LoopMonitor('cmd_safety', hz)
        period = 1.0 / hz if hz > 0 else 0.02
        self.create_timer(period, self._control_loop)
        self.create_timer(1.0, self._publish_status)

        self.get_logger().info('Cmd safety controller started')

    def _update_param_cache(self) -> None:
        """Cache frequently used parameters to avoid per-loop lookups."""
        autonomy_source = str(self.get_parameter('autonomy_source').value)
        if autonomy_source not in ('legacy', 'v4'):
            raise ValueError('autonomy_source must be legacy or v4')
        self._param_cache = {
            'publish_hz': float(self.get_parameter('publish_hz').value),
            'cmd_timeout': float(self.get_parameter('cmd_timeout').value),
            'max_linear_speed': float(self.get_parameter('max_linear_speed').value),
            'max_angular_speed': float(self.get_parameter('max_angular_speed').value),
            'max_linear_accel': float(self.get_parameter('max_linear_accel').value),
            'max_angular_accel': float(self.get_parameter('max_angular_accel').value),
            'deadband_linear': float(self.get_parameter('deadband_linear').value),
            'deadband_angular': float(self.get_parameter('deadband_angular').value),
            'publish_loop_stats': bool(self.get_parameter('publish_loop_stats').value),
            'sensor_timeout': float(self.get_parameter('sensor_timeout').value),
            'min_scan_points': int(self.get_parameter('min_scan_points').value),
            'require_signage': bool(self.get_parameter('require_signage').value),
            'autonomy_source': autonomy_source,
        }

    def _on_params(self, params) -> SetParametersResult:
        """Update cache for dynamic params."""
        for p in params:
            if p.name == 'autonomy_source' and str(p.value) not in ('legacy', 'v4'):
                return SetParametersResult(
                    successful=False, reason='autonomy_source must be legacy or v4')
        for p in params:
            if isinstance(p.value, (int, float)) and not isinstance(p.value, bool):
                signed_allowed = p.name == 'lidar_angle_offset'
                zero_allowed = p.name in ('deadband_linear', 'deadband_angular')
                invalid_limit = not signed_allowed and (p.value < 0 or (p.value == 0 and not zero_allowed))
                if not math.isfinite(p.value) or invalid_limit:
                    return SetParametersResult(successful=False, reason='Safety limits must be finite and positive')
        for p in params:
            if p.name in self._param_cache:
                self._param_cache[p.name] = p.value
                if p.name == 'publish_hz':
                    self.get_logger().warn('publish_hz change requires node restart to retime timer')
        return SetParametersResult(successful=True)

    def _raw_cmd_cb(self, msg: Twist) -> None:
        """Store incoming autonomous command."""
        if not all(math.isfinite(x) for x in (msg.linear.x, msg.angular.z)):
            self.target_cmd = Twist()
            self.last_input_t = 0.0
            return
        self.target_cmd = msg
        self.last_input_t = time.monotonic()

    def _v4_cmd_cb(self, msg: Twist) -> None:
        """Store Stage 8 commands separately; never silently fall back."""
        if not all(math.isfinite(x) for x in (msg.linear.x, msg.angular.z)):
            self.v4_target_cmd = Twist()
            self.v4_last_input_t = 0.0
            return
        self.v4_target_cmd = msg
        self.v4_last_input_t = time.monotonic()

    def _image_cb(self, msg):
        age = observation_age(msg, self.get_clock().now().nanoseconds / 1e9)
        if age <= self._param_cache['sensor_timeout'] and msg.width > 0 and msg.height > 0 and len(msg.data) > 0:
            self.last_image_t = time.monotonic() - age

    def _scan_cb(self, msg):
        self.last_scan = msg
        age = observation_age(msg, self.get_clock().now().nanoseconds / 1e9)
        self.last_scan_t = time.monotonic() - age
        self.scan_valid = age <= self._param_cache['sensor_timeout'] and valid_scan(msg, int(self._param_cache['min_scan_points']))

    def _signage_cb(self, msg):
        self.signage_valid = bool(msg.data)
        self.signage_stamp = time.monotonic()

    def _geometry_cb(self, msg):
        try:
            values = json.loads(msg.data)
            if all(math.isfinite(values[k]) and values[k] > 0 for k in self.servo_geometry):
                self.servo_geometry = {k: float(values[k]) for k in self.servo_geometry}
        except (ValueError, TypeError, KeyError):
            pass

    def _estop_cb(self, msg: Bool) -> None:
        """Update e-stop latch."""
        self.estop = bool(msg.data)
        if self.estop:
            self.estop_count += 1

    def _playback_cmd_cb(self, msg):
        if all(math.isfinite(x) for x in (msg.linear.x, msg.angular.z)):
            self.playback_cmd = msg
            self.playback_cmd_stamp = time.monotonic()
        else:
            self.playback_cmd_stamp = 0.0

    def _playback_state_cb(self, msg):
        try:
            active = json.loads(msg.data).get('state') == 'PLAYBACK'
            if active and not self.playback_active:
                self.playback_started = time.monotonic()
                self.playback_cmd_stamp = 0.0
            self.playback_active = active
            self.playback_state_stamp = time.monotonic()
        except (ValueError, TypeError):
            self.playback_state_stamp = 0.0

    @staticmethod
    def _slew(current: float, target: float, max_rate: float, dt: float) -> float:
        """Rate-limit a value based on max units/second."""
        step = max_rate * dt
        if target > current + step:
            return current + step
        if target < current - step:
            return current - step
        return target

    def _control_loop(self) -> None:
        """Apply safety envelope and publish safe cmd_vel."""
        self.loop_monitor.tick()
        now = time.monotonic()
        dt = max(1e-3, min(.1, now - self.last_loop_t))
        self.last_loop_t = now

        cmd_timeout = float(self._param_cache['cmd_timeout'])
        source = str(self._param_cache['autonomy_source'])
        input_stamp = self.v4_last_input_t if source == 'v4' else self.last_input_t
        selected_cmd = self.v4_target_cmd if source == 'v4' else self.target_cmd
        stale = not fresh(input_stamp, now, cmd_timeout)
        if self.playback_active:
            selected_cmd = self.playback_cmd
            stale = stale or not fresh(self.playback_state_stamp, now, cmd_timeout)
            if not fresh(self.playback_cmd_stamp, now, cmd_timeout):
                # Brief startup allowance only outputs zero; never an old sample.
                selected_cmd = Twist()
                stale = stale or self.playback_cmd_stamp > 0 or now - self.playback_started > 0.1
        if stale:
            self.timeout_count += 1

        sensor_timeout = float(self._param_cache['sensor_timeout'])
        sensors_ok = (fresh(self.last_image_t, now, sensor_timeout)
                      and fresh(self.last_scan_t, now, sensor_timeout) and self.scan_valid)
        if self._param_cache['require_signage']:
            sensors_ok = sensors_ok and self.signage_valid and fresh(self.signage_stamp, now, sensor_timeout)
        if self.estop or stale or not sensors_ok:
            target_lin = 0.0
            target_ang = 0.0
        else:
            target_lin = float(selected_cmd.linear.x)
            target_ang = float(selected_cmd.angular.z)

        max_lin = float(self._param_cache['max_linear_speed'])
        max_ang = float(self._param_cache['max_angular_speed'])
        limited_lin = max(-max_lin, min(max_lin, target_lin))
        limited_ang = max(-max_ang, min(max_ang, target_ang))
        if limited_lin != target_lin or limited_ang != target_ang:
            self.limit_count += 1

        max_lin_acc = float(self._param_cache['max_linear_accel'])
        max_ang_acc = float(self._param_cache['max_angular_accel'])
        out_lin = self._slew(self.output_cmd.linear.x, limited_lin, max_lin_acc, dt)
        out_ang = self._slew(self.output_cmd.angular.z, limited_ang, max_ang_acc, dt)
        # Safety stops and explicit zero requests bypass the comfort ramp.
        if self.estop or stale or not sensors_ok or target_lin == 0.0:
            out_lin = 0.0
            out_ang = 0.0

        db_lin = float(self._param_cache['deadband_linear'])
        db_ang = float(self._param_cache['deadband_angular'])
        if abs(out_lin) < db_lin:
            out_lin = 0.0
        if abs(out_ang) < db_ang:
            out_ang = 0.0

        # Check the actual slew-limited command, not just the requested turn.
        clearance_ok = self.last_scan is not None and swept_path_clear(
            self.last_scan, out_lin, out_ang,
            offset=float(self.get_parameter('lidar_angle_offset').value),
            half_width=float(self.get_parameter('footprint_half_width').value),
            front=float(self.get_parameter('footprint_front').value),
            rear=float(self.get_parameter('footprint_rear').value),
            horizon=float(self.get_parameter('sweep_distance').value), **self.servo_geometry)
        permit = sensors_ok and not self.estop and not stale and clearance_ok
        self.permit_pub.publish(Bool(data=permit))
        if not permit:
            out_lin = out_ang = 0.0

        self.output_cmd.linear.x = out_lin
        self.output_cmd.angular.z = out_ang
        self.cmd_pub.publish(self.output_cmd)

    def _publish_status(self) -> None:
        """Publish safety and loop diagnostics as JSON strings."""
        payload = {
            'estop': self.estop,
            'timeout_count': self.timeout_count,
            'estop_count': self.estop_count,
            'limit_count': self.limit_count,
            'autonomy_source': self._param_cache['autonomy_source'],
            'stamp_sec': round(time.time(), 3),
        }
        self.status_pub.publish(String(data=json.dumps(payload, separators=(',', ':'))))

        if bool(self._param_cache['publish_loop_stats']):
            loop_payload = self.loop_monitor.snapshot()
            loop_payload['node'] = self.get_name()
            self.loop_stats_pub.publish(String(data=json.dumps(loop_payload, separators=(',', ':'))))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CmdSafetyController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
