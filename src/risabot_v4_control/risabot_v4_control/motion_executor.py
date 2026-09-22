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
from std_msgs.msg import Bool, String

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
        pose = msg.pose.pose.position
        values = (float(pose.x), float(pose.y), _yaw_from_odom(msg))
        if all(math.isfinite(v) for v in values):
            self._odom_pose = values
            self._odom_stamp = time.monotonic()

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
                    (self._motor_duty / self._duty_map if self._track_test_mode
                    else self._p['forward_speed_mps']), self._p['minimum_speed_scale'],
                    self._p['steering_slowdown_gain'],
                    self._p['boundary_slowdown_margin_m'],
                )
                self._last_source, self._last_reason = decision.source, decision.reason
                cmd = Twist()
                speed = decision.speed
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
        cmd = self._select(time.monotonic())
        if cmd.linear.x != 0.0 or cmd.angular.z != 0.0:
            self._nonzero_count += 1
        self._cmd_pub.publish(cmd)

    def _publish_status(self) -> None:
        payload = {
            'algorithm_stage': 8,
            'enabled': self._enabled,
            'track_test_mode': self._track_test_mode,
            'forward_motor_duty_percent': self._motor_duty if self._track_test_mode else None,
            'minimum_turn_motor_duty_percent': (
                self._motor_duty * self._p['minimum_speed_scale']
                if self._track_test_mode else None
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
