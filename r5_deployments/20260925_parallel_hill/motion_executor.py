"""Stage 8: fail-closed execution of validated V4 motion proposals."""

import json
import math
import time
from typing import Dict, Optional, Tuple

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32, String
from .speed_core import SpeedRegulator
from .hill_boost_core import HillBoost

from .control_core import (
    ControlContractError,
    LEGACY_CHALLENGE_STATES,
    STOP_STATES,
    V4_PATH_STATES,
    V4_TRAJECTORY_STATES,
    dashboard_state,
    parse_path,
    path_command,
    proposal_contract,
    trajectory_command,
    transform_path,
)


V4_RAW_TOPIC = '/cmd_vel_v4_raw'
STATUS_TOPIC = '/v4_control/status'


def _yaw_from_odom(msg: Odometry) -> float:
    q = msg.pose.pose.orientation
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class MotionExecutor(Node):
    """Convert validated Stage 7 proposals into guarded raw commands.

    This node does not talk to hardware. Its output still passes through the
    production command safety controller before the hardware bridge can act.
    """

    def __init__(self) -> None:
        super().__init__('v4_motion_executor')
        self.declare_parameter('enabled', False)
        self.declare_parameter('track_test_mode', False)
        self.declare_parameter('boom_gate_wait_enabled', False)
        self.declare_parameter('encoder_speed_control', False)
        self.declare_parameter('encoder_speed_sign', 1.0)
        self.declare_parameter('forward_motor_duty_percent', 65.0)
        self.declare_parameter('motor_duty_per_mps', 255.0)
        self.declare_parameter('enable_boundary_reverse_recovery', False)
        self._gate_names = (
            'command_contract_validated', 'stop_preemption_validated',
            'timeout_validated', 'operator_motion_authorized',
        )
        for name in self._gate_names:
            self.declare_parameter(name, False)
        self.declare_parameter('allow_legacy_challenge_passthrough', True)
        for name, value in (
            ('publish_hz', 50.0), ('proposal_timeout_sec', 0.35),
            ('path_timeout_sec', 0.50), ('path_max_age_sec', 45.0),
            ('odom_timeout_sec', 0.25),
            ('state_timeout_sec', 0.50), ('legacy_timeout_sec', 0.35),
            ('wheelbase_m', 0.210), ('maximum_steer_deg', 50.0),
            ('forward_speed_mps', 0.08), ('path_forward_speed_mps', 0.06),
            ('path_reverse_speed_mps', 0.05), ('minimum_speed_scale', 0.40),
            ('steering_slowdown_gain', 0.65),
            ('boundary_slowdown_margin_m', 0.03),
            ('completion_distance_m', 0.04),
            ('hill_max_speed_mps', 0.18),
            ('boundary_recovery_trigger_m', 0.0),
            ('boundary_recovery_confirm_sec', 0.35),
            ('boundary_recovery_duration_sec', 0.45),
            ('boundary_recovery_reverse_duty_percent', 30.0),
            ('boundary_recovery_steering', 0.65),
            ('boundary_recovery_rear_clearance_m', 0.35),
            ('scan_timeout_sec', 0.35),
        ):
            self.declare_parameter(name, value)

        self._enabled = bool(self.get_parameter('enabled').value)
        self._track_test_mode = bool(self.get_parameter('track_test_mode').value)
        self._boom_gate_wait_enabled = bool(self.get_parameter('boom_gate_wait_enabled').value)
        self._encoder_speed_control = bool(self.get_parameter('encoder_speed_control').value)
        self._encoder_speed_sign = float(self.get_parameter('encoder_speed_sign').value)
        if self._encoder_speed_sign not in (-1.0, 1.0):
            raise ControlContractError('encoder_speed_sign must be +1 or -1')
        self._motor_duty = float(self.get_parameter('forward_motor_duty_percent').value)
        self._duty_map = float(self.get_parameter('motor_duty_per_mps').value)
        self._boundary_recovery_enabled = bool(
            self.get_parameter('enable_boundary_reverse_recovery').value
        )
        if (not math.isfinite(self._motor_duty) or not 0.0 < self._motor_duty <= 100.0
                or not math.isfinite(self._duty_map) or self._duty_map <= 0.0):
            raise ControlContractError('motor duty must be in (0, 100] and duty map positive')
        self._gates = {name: bool(self.get_parameter(name).value) for name in self._gate_names}
        self._allow_legacy = bool(self.get_parameter('allow_legacy_challenge_passthrough').value)
        self._p = {name: float(self.get_parameter(name).value) for name in (
            'publish_hz', 'proposal_timeout_sec', 'path_timeout_sec', 'path_max_age_sec',
            'odom_timeout_sec', 'state_timeout_sec', 'legacy_timeout_sec',
            'wheelbase_m', 'maximum_steer_deg', 'forward_speed_mps',
            'path_forward_speed_mps', 'path_reverse_speed_mps',
            'minimum_speed_scale', 'steering_slowdown_gain',
            'boundary_slowdown_margin_m',
            'completion_distance_m', 'hill_max_speed_mps',
            'boundary_slowdown_margin_m',
            'boundary_recovery_trigger_m', 'boundary_recovery_confirm_sec',
            'boundary_recovery_duration_sec',
            'boundary_recovery_reverse_duty_percent',
            'boundary_recovery_steering',
            'boundary_recovery_rear_clearance_m', 'scan_timeout_sec',
        )}
        self._validate_parameters()
        if self._encoder_speed_control and (not self._track_test_mode or self._boundary_recovery_enabled):
            raise ControlContractError('encoder speed mode requires forward-only track test')
        self._speed_regulator = SpeedRegulator(self._motor_duty)
        self._measured_speed = float('nan')
        self._raw_encoder_speed = float('nan')
        self._speed_stamp = 0.0
        self._speed_target = 0.0
        self.add_on_set_parameters_callback(self._on_parameters)

        self._state = ''
        self._state_stamp = 0.0
        self._estop = False
        self._proposal: Optional[Dict[str, object]] = None
        self._proposal_stamp = 0.0
        self._odom_pose: Optional[Tuple[float, float, float]] = None
        self._odom_stamp = 0.0
        self._legacy = Twist()
        self._legacy_stamp = 0.0
        self._paths: Dict[str, Dict[str, object]] = {}
        self._active_path_source = ''
        self._active_path = ()
        self._active_path_index = 0
        self._last_reason = 'not evaluated'
        self._last_source = 'hold'
        self._last_error = ''
        self._nonzero_count = 0
        self._rear_clearance_m = 0.0
        self._scan_stamp = 0.0
        self._traffic_state = 'unknown'
        self._traffic_stamp = 0.0
        self._boom_gate_open = None
        self._boom_gate_stamp = 0.0
        self._hill_sign = False
        self._hill_stamp = 0.0
        self._hill_model_visible = False
        self._hill_model_stamp = 0.0
        self._hill_boost = HillBoost()
        self._pitch_deg = 0.0
        self._pitch_stamp = 0.0
        self._bump_sign = False
        self._bump_until_m = 0.0
        self._travel_m = 0.0
        self._previous_odom_xy = None
        self._boundary_violation_since = 0.0
        self._boundary_recovery_until = 0.0
        self._boundary_recovery_cooldown_until = 0.0
        self._boundary_recovery_steer = 0.0

        self._cmd_pub = self.create_publisher(Twist, V4_RAW_TOPIC, 10)
        self._status_pub = self.create_publisher(String, STATUS_TOPIC, 10)
        self.create_subscription(String, '/v4_experimental/arbitration/proposed_request', self._proposal_cb, 10)
        self.create_subscription(String, '/v4_experimental/parking/proposed_path', lambda m: self._path_cb('parking', m), 10)
        self.create_subscription(String, '/v4_experimental/recovery/proposed_path', lambda m: self._path_cb('recovery', m), 10)
        self.create_subscription(String, '/dashboard_state', self._state_cb, 10)
        self.create_subscription(Bool, '/e_stop', self._estop_cb, 10)
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        self.create_subscription(String, '/traffic_light_state', self._traffic_cb, 10)
        self.create_subscription(Bool, '/boom_gate_open', self._boom_gate_cb, 10)
        self.create_subscription(Bool, '/hill_sign_detected', self._hill_cb, 10)
        self.create_subscription(Bool, '/hill_model_visible', self._hill_model_cb, 10)
        self.create_subscription(Bool, '/speed_bump_detected', self._bump_cb, 10)
        self.create_subscription(Float32, '/imu/pitch', self._pitch_cb, 10)
        self.create_subscription(Twist, '/cmd_vel_auto_raw', self._legacy_cb, 10)
        self.create_subscription(
            LaserScan, '/scan', self._scan_cb, qos_profile_sensor_data
        )

        period = 1.0 / self._p['publish_hz']
        self.create_timer(period, self._control_loop)
        self.create_timer(0.5, self._publish_status)
        self.get_logger().warning(
            'V4 Stage 8 starts fail-closed; output requires enabled=true and every validation gate'
        )

    def _validate_parameters(self) -> None:
        positive = (
            'publish_hz', 'proposal_timeout_sec', 'path_timeout_sec', 'path_max_age_sec',
            'odom_timeout_sec', 'state_timeout_sec', 'legacy_timeout_sec',
            'wheelbase_m', 'maximum_steer_deg', 'forward_speed_mps',
            'path_forward_speed_mps', 'path_reverse_speed_mps',
            'completion_distance_m', 'hill_max_speed_mps',
        )
        if any(not math.isfinite(self._p[name]) or self._p[name] <= 0.0 for name in positive):
            raise ControlContractError('Stage 8 positive parameters must be finite and positive')
        if not 0.0 < self._p['minimum_speed_scale'] <= 1.0:
            raise ControlContractError('minimum_speed_scale must be in (0, 1]')
        if (not math.isfinite(self._p['steering_slowdown_gain'])
                or self._p['steering_slowdown_gain'] < 0.0):
            raise ControlContractError('steering_slowdown_gain must be finite and non-negative')
        if not 0.0 <= self._p['boundary_recovery_steering'] <= 1.0:
            raise ControlContractError('boundary recovery steering must be in [0, 1]')
        if not 0.0 < self._p['boundary_recovery_reverse_duty_percent'] <= 100.0:
            raise ControlContractError('boundary recovery duty must be in (0, 100]')
        for name in (
            'boundary_recovery_confirm_sec', 'boundary_recovery_duration_sec',
            'boundary_recovery_rear_clearance_m', 'scan_timeout_sec',
        ):
            if not math.isfinite(self._p[name]) or self._p[name] <= 0.0:
                raise ControlContractError(f'{name} must be finite and positive')
        if not math.isfinite(self._p['boundary_recovery_trigger_m']):
            raise ControlContractError('boundary recovery trigger must be finite')

    def _on_parameters(self, parameters) -> SetParametersResult:
        safe = {
            'forward_speed_mps', 'path_forward_speed_mps',
            'path_reverse_speed_mps', 'minimum_speed_scale',
            'steering_slowdown_gain', 'boundary_slowdown_margin_m',
            'hill_max_speed_mps',
        }
        protected = set(self._gate_names) | {
            'enabled', 'track_test_mode', 'forward_motor_duty_percent', 'motor_duty_per_mps',
            'encoder_speed_control',
            'encoder_speed_sign',
            'enable_boundary_reverse_recovery',
            'allow_legacy_challenge_passthrough', 'publish_hz',
            'proposal_timeout_sec', 'path_timeout_sec', 'path_max_age_sec',
            'odom_timeout_sec', 'state_timeout_sec', 'legacy_timeout_sec',
            'wheelbase_m', 'maximum_steer_deg', 'completion_distance_m',
            'boundary_recovery_trigger_m', 'boundary_recovery_confirm_sec',
            'boundary_recovery_duration_sec',
            'boundary_recovery_reverse_duty_percent',
            'boundary_recovery_steering',
            'boundary_recovery_rear_clearance_m', 'scan_timeout_sec',
        }
        changes = {}
        for parameter in parameters:
            if parameter.name in protected:
                return SetParametersResult(
                    successful=False,
                    reason=f'{parameter.name} requires a node restart',
                )
            if parameter.name in safe:
                changes[parameter.name] = parameter.value
        if not changes:
            return SetParametersResult(successful=True)
        if not self._gate_blockers():
            return SetParametersResult(
                successful=False,
                reason='speed tuning is locked while V4 motion authority is open',
            )
        previous = dict(self._p)
        try:
            self._p.update({name: float(value) for name, value in changes.items()})
            self._validate_parameters()
        except (ControlContractError, TypeError, ValueError) as exc:
            self._p = previous
            return SetParametersResult(successful=False, reason=str(exc))
        return SetParametersResult(successful=True)

    @staticmethod
    def _fresh(stamp: float, now: float, timeout: float) -> bool:
        return stamp > 0.0 and 0.0 <= now - stamp <= timeout

    def _proposal_cb(self, msg: String) -> None:
        try:
            value = json.loads(msg.data)
            proposal_contract(value)
            self._proposal = value
            self._proposal_stamp = time.monotonic()
            self._last_error = ''
        except (json.JSONDecodeError, TypeError, ControlContractError) as exc:
            self._proposal = None
            self._proposal_stamp = 0.0
            self._last_error = f'invalid arbitration proposal: {exc}'

    def _path_cb(self, source: str, msg: String) -> None:
        try:
            value = json.loads(msg.data)
            if not isinstance(value, dict):
                raise ControlContractError('path payload must be an object')
            points = parse_path(value)
            now = time.monotonic()
            self._paths[source] = {'payload': value, 'points': points, 'stamp': now}
        except (json.JSONDecodeError, TypeError, ControlContractError) as exc:
            self._paths.pop(source, None)
            self._last_error = f'invalid {source} path: {exc}'

    def _state_cb(self, msg: String) -> None:
        self._state = dashboard_state(msg.data)
        self._state_stamp = time.monotonic()

    def _estop_cb(self, msg: Bool) -> None:
        self._estop = bool(msg.data)

    def _odom_cb(self, msg: Odometry) -> None:
        self._raw_encoder_speed = float(msg.twist.twist.linear.x)
        # R5's forward encoder odometry is negative in the attended trace.
        # Polarity is an explicit per-vehicle launch setting, leaving the
        # odometry message and MANUAL controls untouched.
        self._measured_speed = self._encoder_speed_sign * self._raw_encoder_speed
        self._speed_stamp = time.monotonic()
        pose = msg.pose.pose.position
        values = (float(pose.x), float(pose.y), _yaw_from_odom(msg))
        if all(math.isfinite(v) for v in values):
            if self._previous_odom_xy is not None:
                step = math.hypot(values[0] - self._previous_odom_xy[0],
                                  values[1] - self._previous_odom_xy[1])
                if 0.002 <= step < 0.25 and self._measured_speed > 0.02:
                    self._travel_m += step
            self._previous_odom_xy = values[:2]
            self._odom_pose = values
            self._odom_stamp = time.monotonic()

    def _traffic_cb(self, msg: String) -> None:
        self._traffic_state = str(msg.data).lower()
        self._traffic_stamp = time.monotonic()

    def _boom_gate_cb(self, msg: Bool) -> None:
        self._boom_gate_open = bool(msg.data)
        self._boom_gate_stamp = time.monotonic()

    def _hill_cb(self, msg: Bool) -> None:
        self._hill_sign = bool(msg.data)
        self._hill_stamp = time.monotonic()

    def _hill_model_cb(self, msg: Bool) -> None:
        self._hill_model_visible = bool(msg.data)
        self._hill_model_stamp = time.monotonic()

    def _pitch_cb(self, msg: Float32) -> None:
        if math.isfinite(float(msg.data)):
            self._pitch_deg = float(msg.data)
            self._pitch_stamp = time.monotonic()

    def _bump_cb(self, msg: Bool) -> None:
        detected = bool(msg.data)
        if detected and not self._bump_sign:
            self._bump_until_m = self._travel_m + 1.5
        self._bump_sign = detected

    def _legacy_cb(self, msg: Twist) -> None:
        if all(math.isfinite(v) for v in (msg.linear.x, msg.angular.z)):
            self._legacy = msg
            self._legacy_stamp = time.monotonic()

    def _scan_cb(self, msg: LaserScan) -> None:
        rear = []
        for index, range_m in enumerate(msg.ranges):
            value = float(range_m)
            if not math.isfinite(value) or value <= 0.0:
                continue
            angle = float(msg.angle_min) + index * float(msg.angle_increment)
            if math.cos(angle) <= -0.80:
                rear.append(value)
        if rear:
            self._rear_clearance_m = min(rear)
            self._scan_stamp = time.monotonic()

    def _boundary_recovery_command(
        self, reference: Dict[str, object], now: float
    ) -> Optional[Twist]:
        """Return a short reverse correction only after confirmed line overlap."""
        if not self._boundary_recovery_enabled:
            return None
        if now < self._boundary_recovery_until:
            cmd = Twist()
            cmd.linear.x = -self._p['boundary_recovery_reverse_duty_percent'] / self._duty_map
            cmd.angular.z = self._boundary_recovery_steer
            self._last_source = 'boundary_reverse_recovery'
            self._last_reason = 'bounded reverse correction with rear LiDAR clearance'
            return cmd
        if self._boundary_recovery_until > 0.0:
            self._boundary_recovery_until = 0.0
            self._boundary_recovery_cooldown_until = now + 2.0
        try:
            clearance = float(reference['boundary_clearance_m'])
            lateral_error = float(reference['lateral_error_m'])
        except (KeyError, TypeError, ValueError):
            self._boundary_violation_since = 0.0
            return None
        if (not math.isfinite(clearance) or not math.isfinite(lateral_error)
                or clearance > self._p['boundary_recovery_trigger_m']
                or now < self._boundary_recovery_cooldown_until):
            self._boundary_violation_since = 0.0
            return None
        if self._boundary_violation_since <= 0.0:
            self._boundary_violation_since = now
            return None
        if now - self._boundary_violation_since < self._p['boundary_recovery_confirm_sec']:
            return None
        scan_fresh = self._fresh(
            self._scan_stamp, now, self._p['scan_timeout_sec']
        )
        if (not scan_fresh
                or self._rear_clearance_m
                < self._p['boundary_recovery_rear_clearance_m']):
            return None
        self._boundary_recovery_steer = -math.copysign(
            self._p['boundary_recovery_steering'], lateral_error
        )
        self._boundary_recovery_until = (
            now + self._p['boundary_recovery_duration_sec']
        )
        self._boundary_violation_since = 0.0
        return self._boundary_recovery_command(reference, now)

    def _gate_blockers(self) -> list:
        blockers = []
        if not self._enabled:
            blockers.append('node disabled')
        if not self._track_test_mode:
            blockers.extend(name for name, value in self._gates.items() if not value)
        return blockers

    def _stop(self, reason: str):
        self._last_source, self._last_reason = 'hold', reason
        return Twist()

    def _path_twist(self, source: str, reference: Dict[str, object], now: float) -> Twist:
        if not self._fresh(self._odom_stamp, now, self._p['odom_timeout_sec']) or self._odom_pose is None:
            return self._stop('odometry is stale')
        item = self._paths.get(source)
        if item is None:
            return self._stop(f'{source} path is unavailable')
        signature = f'{source}:{item["stamp"]}'
        maximum_age = (self._p['path_timeout_sec'] if signature != self._active_path_source
                       else self._p['path_max_age_sec'])
        if not self._fresh(float(item['stamp']), now, maximum_age):
            return self._stop(f'{source} path is stale')
        payload = item['payload']
        family = str(reference.get('family', ''))
        if family and str(payload.get('family', '')) != family:
            return self._stop(f'{source} path does not match selected family')
        if signature != self._active_path_source:
            self._active_path = transform_path(item['points'], self._odom_pose)
            self._active_path_source = signature
            self._active_path_index = 0
        decision, index, _ = path_command(
            self._active_path, self._odom_pose, self._active_path_index,
            self._p['wheelbase_m'], math.radians(self._p['maximum_steer_deg']),
            self._p['path_forward_speed_mps'], self._p['path_reverse_speed_mps'],
            self._p['completion_distance_m'],
        )
        self._active_path_index = index
        self._last_source, self._last_reason = decision.source, decision.reason
        cmd = Twist()
        cmd.linear.x, cmd.angular.z = decision.speed, decision.steering
        return cmd

    def _select(self, now: float) -> Twist:
        blockers = self._gate_blockers()
        if blockers:
            return self._stop('; '.join(blockers))
        if self._estop:
            return self._stop('e-stop asserted')
        if not self._fresh(self._state_stamp, now, self._p['state_timeout_sec']):
            return self._stop('mission state is stale')
        state = self._state
        if state in STOP_STATES:
            return self._stop(f'mission hold: {state or "UNKNOWN"}')
        if self._track_test_mode and state != 'LANE_FOLLOW':
            return self._stop('track test accepts lane following only')
        if self._track_test_mode and self._boom_gate_wait_enabled:
            if not self._fresh(self._boom_gate_stamp, now, 1.0):
                return self._stop('boom gate camera state is stale')
            if self._boom_gate_open is False:
                return self._stop('WAITING FOR BOOM GATE')
        if (self._track_test_mode and self._fresh(self._traffic_stamp, now, 1.0)
                and self._traffic_state in ('red', 'yellow')):
            return self._stop('traffic model: ' + self._traffic_state)
        if state in LEGACY_CHALLENGE_STATES:
            if not self._allow_legacy:
                return self._stop(f'legacy challenge passthrough disabled: {state}')
            if not self._fresh(self._legacy_stamp, now, self._p['legacy_timeout_sec']):
                return self._stop(f'legacy {state.lower()} command is stale')
            self._last_source, self._last_reason = 'legacy_challenge', state
            cmd = Twist()
            cmd.linear.x = float(self._legacy.linear.x)
            cmd.angular.z = float(self._legacy.angular.z)
            return cmd
        if not self._fresh(self._proposal_stamp, now, self._p['proposal_timeout_sec']) or self._proposal is None:
            return self._stop('V4 arbitration proposal is stale')
        try:
            source, action, reference = proposal_contract(self._proposal)
            if action == 'stop':
                return self._stop(str(self._proposal.get('reason', 'arbitrated stop')))
            if state in V4_TRAJECTORY_STATES:
                if source != 'trajectory' or action != 'follow_curvature':
                    return self._stop('trajectory state has incompatible proposal')
                recovery = self._boundary_recovery_command(reference, now)
                if recovery is not None:
                    return recovery
                decision = trajectory_command(
                    reference, self._p['wheelbase_m'],
                    math.radians(self._p['maximum_steer_deg']),
                    (self._motor_duty / self._duty_map if self._track_test_mode and not self._encoder_speed_control
                    else self._p['forward_speed_mps']), self._p['minimum_speed_scale'],
                    self._p['steering_slowdown_gain'],
                    self._p['boundary_slowdown_margin_m'],
                )
                self._last_source, self._last_reason = decision.source, decision.reason
                cmd = Twist()
                speed = decision.speed
                hill_boost_active = self._hill_boost.update(
                    self._hill_model_visible
                    and self._fresh(self._hill_model_stamp, now, 0.4), now
                )
                if self._track_test_mode and not self._encoder_speed_control and speed > 0.0:
                    if self._travel_m < self._bump_until_m:
                        speed = min(speed, 40.0 / self._duty_map)
                    elif hill_boost_active:
                        speed = min(100.0 / self._duty_map, speed * 2.0)
                if (state == 'HILL'
                        and self._fresh(self._legacy_stamp, now, self._p['legacy_timeout_sec'])
                        and self._legacy.linear.x > 0.0):
                    speed = min(self._p['hill_max_speed_mps'],
                                max(speed, float(self._legacy.linear.x)))
                cmd.linear.x, cmd.angular.z = speed, decision.steering
                return cmd
            expected = V4_PATH_STATES.get(state)
            if expected is not None:
                if source != expected or action != 'follow_path':
                    return self._stop(f'{state} has incompatible proposal')
                return self._path_twist(source, reference, now)
            return self._stop(f'unsupported mission state: {state}')
        except (ControlContractError, KeyError, TypeError, ValueError) as exc:
            self._last_error = str(exc)
            return self._stop(f'control contract rejected proposal: {exc}')

    def _control_loop(self) -> None:
        now = time.monotonic()
        cmd = self._select(now)
        if self._encoder_speed_control:
            # Metric target belongs here; the downstream legacy bridge still
            # expects duty / duty_map. Never send 0.4 straight to that bridge.
            self._speed_target = float(cmd.linear.x)
            if self._state == 'MANUAL' and self._fresh(self._state_stamp, now, self._p['state_timeout_sec']):
                self._speed_regulator.reset(clear_fault=True)
            duty = self._speed_regulator.update(
                self._speed_target, self._measured_speed,
                now - self._speed_stamp if self._speed_stamp else float('inf'), now)
            cmd.linear.x = duty / self._duty_map
            if self._speed_regulator.fault:
                cmd = self._stop('speed feedback: ' + self._speed_regulator.fault)
        if cmd.linear.x != 0.0 or cmd.angular.z != 0.0:
            self._nonzero_count += 1
        self._cmd_pub.publish(cmd)

    def _publish_status(self) -> None:
        payload = {
            'algorithm_stage': 8,
            'enabled': self._enabled,
            'track_test_mode': self._track_test_mode,
            'encoder_speed_control': self._encoder_speed_control,
            'cruise_speed_target_mps': self._p['forward_speed_mps'] if self._encoder_speed_control else None,
            'active_speed_target_mps': self._speed_target if self._encoder_speed_control else None,
            'encoder_speed_mps': self._measured_speed if math.isfinite(self._measured_speed) else None,
            'encoder_raw_speed_mps': self._raw_encoder_speed if math.isfinite(self._raw_encoder_speed) else None,
            'encoder_speed_sign': self._encoder_speed_sign,
            'encoder_speed_age_sec': time.monotonic() - self._speed_stamp if self._speed_stamp else None,
            'speed_regulator_duty_percent': self._speed_regulator.duty if self._encoder_speed_control else None,
            'speed_regulator_fault': self._speed_regulator.fault,
            'speed_measurement_source': 'existing encoder odometry; ground scale unverified',
            'forward_motor_duty_percent': self._motor_duty if self._track_test_mode else None,
            'traffic_model_state': self._traffic_state,
            'traffic_model_age_sec': time.monotonic() - self._traffic_stamp if self._traffic_stamp else None,
            'boom_gate_open': self._boom_gate_open if self._boom_gate_wait_enabled else None,
            'boom_gate_age_sec': time.monotonic() - self._boom_gate_stamp if self._boom_gate_stamp else None,
            'hill_sign_active': self._hill_sign,
            'hill_model_visible': self._hill_model_visible,
            'hill_boost_active': time.monotonic() < self._hill_boost.boost_until,
            'hill_pitch_deg': self._pitch_deg if self._pitch_stamp else None,
            'bump_remaining_m': max(0.0, self._bump_until_m - self._travel_m),
            'minimum_turn_motor_duty_percent': (
                self._motor_duty * self._p['minimum_speed_scale']
                if self._track_test_mode and not self._encoder_speed_control else None
            ),
            'steering_slowdown_gain': self._p['steering_slowdown_gain'],
            'boundary_reverse_recovery_enabled': self._boundary_recovery_enabled,
            'rear_clearance_m': self._rear_clearance_m,
            'boundary_recovery_active': (
                time.monotonic() < self._boundary_recovery_until
            ),
            'motion_authority': not self._gate_blockers(),
            'can_publish_motion': not self._gate_blockers(),
            'validation_gates': self._gates,
            'blockers': self._gate_blockers(),
            'state': self._state,
            'selected_source': self._last_source,
            'reason': self._last_reason,
            'last_error': self._last_error,
            'nonzero_command_count': self._nonzero_count,
            'safety_controller_still_required': True,
        }
        self._status_pub.publish(String(data=json.dumps(payload, separators=(',', ':'), sort_keys=True)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MotionExecutor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
