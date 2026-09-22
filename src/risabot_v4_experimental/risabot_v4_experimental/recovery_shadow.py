"""Stage 6 diagnostic recovery planner with no motion authority."""

import json
import math
import time
from typing import Dict, List, Optional, Tuple

from cv_bridge import CvBridge, CvBridgeError
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String

from .bev_core import CalibrationError, load_profiles, profile_report
from .recovery_core import (
    RecoveryConfig,
    RecoveryError,
    RecoveryRequest,
    plan_recovery,
    forward_candidates_exhausted,
    recovery_request_from_mapping,
    request_blockers,
)
from .trajectory_core import TrajectoryError, VehicleGeometry


STATUS_TOPIC = '/v4_experimental/recovery/status'
PATH_TOPIC = '/v4_experimental/recovery/proposed_path'


class RecoveryShadow(Node):
    def __init__(self) -> None:
        super().__init__('v4_recovery_shadow')
        self.declare_parameter('enabled', False)
        self.declare_parameter('profile_path', '')
        self.declare_parameter('request_topic', '/v4_experimental/recovery/request')
        self.declare_parameter('road_status_topic', '/v4_experimental/road/status')
        self.declare_parameter('trajectory_status_topic', '/v4_experimental/trajectory/status')
        self.declare_parameter('road_mask_topic', '/v4_experimental/road/secondary/fused')
        self.declare_parameter('rear_coverage_topic', '/v4_experimental/bev/secondary/coverage')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('request_timeout_sec', 0.35)
        self.declare_parameter('road_timeout_sec', 0.35)
        self.declare_parameter('trajectory_timeout_sec', 0.35)
        self.declare_parameter('scan_timeout_sec', 0.50)
        self.declare_parameter('sync_tolerance_sec', 0.10)
        self.declare_parameter('maximum_plan_hz', 0.5)

        gate_names = (
            'recovery_request_source_validated', 'hard_hold_source_validated',
            'forward_status_source_validated',
            'rear_coverage_validated', 'vehicle_geometry_validated',
            'minimum_turn_radius_validated', 'lidar_extrinsics_validated',
            'road_tolerance_validated',
        )
        for name in gate_names:
            self.declare_parameter(name, False)

        for name, value in (
            ('vehicle_length_m', 0.300), ('vehicle_width_m', 0.192),
            ('wheelbase_m', 0.210), ('rear_overhang_m', 0.042),
            ('minimum_turn_radius_m', 0.400), ('footprint_padding_m', 0.005),
            ('lidar_x_m', 0.0), ('lidar_y_m', 0.0), ('lidar_yaw_rad', 0.0),
            ('minimum_scan_range_m', 0.03), ('maximum_scan_range_m', 2.0),
            ('sample_step_m', 0.006), ('footprint_sample_spacing_m', 0.02),
            ('minimum_road_support', 0.98), ('obstacle_margin_m', 0.015),
            ('minimum_reverse_distance_m', 0.03),
            ('maximum_reverse_distance_m', 0.20),
            ('maximum_path_length_m', 1.60),
            ('minimum_rear_evidence_fraction', 0.55),
            ('road_tolerance_m', 0.05),
            ('first_goal_distance_m', 0.25), ('goal_spacing_m', 0.15),
            ('maximum_goal_count', 5), ('maximum_attempts', 3),
        ):
            self.declare_parameter(name, value)

        self._enabled = bool(self.get_parameter('enabled').value)
        self._request_timeout = float(self.get_parameter('request_timeout_sec').value)
        self._road_timeout = float(self.get_parameter('road_timeout_sec').value)
        self._trajectory_timeout = float(self.get_parameter('trajectory_timeout_sec').value)
        self._scan_timeout = float(self.get_parameter('scan_timeout_sec').value)
        self._sync_tolerance = float(self.get_parameter('sync_tolerance_sec').value)
        maximum_plan_hz = float(self.get_parameter('maximum_plan_hz').value)
        if not math.isfinite(maximum_plan_hz) or maximum_plan_hz <= 0.0:
            raise RecoveryError('maximum_plan_hz must be finite and positive')
        self._minimum_plan_interval = 1.0 / maximum_plan_hz
        self._gates = {name: bool(self.get_parameter(name).value) for name in gate_names}
        self._geometry = VehicleGeometry(
            length_m=float(self.get_parameter('vehicle_length_m').value),
            width_m=float(self.get_parameter('vehicle_width_m').value),
            wheelbase_m=float(self.get_parameter('wheelbase_m').value),
            rear_overhang_m=float(self.get_parameter('rear_overhang_m').value),
            minimum_turn_radius_m=float(self.get_parameter('minimum_turn_radius_m').value),
            footprint_padding_m=float(self.get_parameter('footprint_padding_m').value),
        )
        self._config = RecoveryConfig(
            sample_step_m=float(self.get_parameter('sample_step_m').value),
            footprint_sample_spacing_m=float(self.get_parameter('footprint_sample_spacing_m').value),
            minimum_road_support=float(self.get_parameter('minimum_road_support').value),
            obstacle_margin_m=float(self.get_parameter('obstacle_margin_m').value),
            minimum_reverse_distance_m=float(self.get_parameter('minimum_reverse_distance_m').value),
            maximum_reverse_distance_m=float(self.get_parameter('maximum_reverse_distance_m').value),
            maximum_path_length_m=float(self.get_parameter('maximum_path_length_m').value),
            minimum_rear_evidence_fraction=float(self.get_parameter('minimum_rear_evidence_fraction').value),
            road_tolerance_m=float(self.get_parameter('road_tolerance_m').value),
            first_goal_distance_m=float(self.get_parameter('first_goal_distance_m').value),
            goal_spacing_m=float(self.get_parameter('goal_spacing_m').value),
            maximum_goal_count=int(self.get_parameter('maximum_goal_count').value),
            maximum_attempts=int(self.get_parameter('maximum_attempts').value),
        )
        self._geometry.validate()
        self._config.validate()
        self._lidar_pose = tuple(float(self.get_parameter(name).value) for name in (
            'lidar_x_m', 'lidar_y_m', 'lidar_yaw_rad'
        ))
        self._scan_min = float(self.get_parameter('minimum_scan_range_m').value)
        self._scan_max = float(self.get_parameter('maximum_scan_range_m').value)
        if not all(math.isfinite(value) for value in self._lidar_pose):
            raise RecoveryError('LiDAR extrinsics must be finite')
        if self._scan_min < 0.0 or self._scan_max <= self._scan_min:
            raise RecoveryError('scan range limits are invalid')

        self._profiles = {}
        self._profile_error = ''
        try:
            self._profiles = load_profiles(str(self.get_parameter('profile_path').value))
        except (CalibrationError, OSError, ValueError) as exc:
            self._profile_error = str(exc)

        self._bridge = CvBridge()
        self._request: Optional[RecoveryRequest] = None
        self._request_mono: Optional[float] = None
        self._road_status: Optional[Dict[str, object]] = None
        self._road_mono: Optional[float] = None
        self._trajectory_status: Optional[Dict[str, object]] = None
        self._trajectory_mono: Optional[float] = None
        self._road_mask = None
        self._road_stamp: Optional[float] = None
        self._road_mask_mono: Optional[float] = None
        self._coverage = None
        self._coverage_stamp: Optional[float] = None
        self._coverage_mono: Optional[float] = None
        self._scan_points: List[Tuple[float, float]] = []
        self._scan_mono: Optional[float] = None
        self._last_signature = None
        self._last_plan_mono = 0.0
        self._processed_plans = 0
        self._last_plan_duration_ms = None
        self._last_error = ''
        self._selected = None
        self._candidate_reports = []

        self._status_pub = self.create_publisher(String, STATUS_TOPIC, 10)
        self._path_pub = self.create_publisher(String, PATH_TOPIC, 2)
        subscriptions = (
            (String, 'request_topic', self._request_callback),
            (String, 'road_status_topic', self._road_status_callback),
            (String, 'trajectory_status_topic', self._trajectory_status_callback),
            (Image, 'road_mask_topic', self._road_mask_callback),
            (Image, 'rear_coverage_topic', self._coverage_callback),
            (LaserScan, 'scan_topic', self._scan_callback),
        )
        for message_type, parameter, callback in subscriptions:
            self.create_subscription(
                message_type, str(self.get_parameter(parameter).value),
                callback, qos_profile_sensor_data,
            )
        self.create_timer(0.5, self._publish_status)
        state = 'enabled' if self._enabled else 'disabled'
        self.get_logger().warning(
            f'V4 recovery shadow is {state}; diagnostic proposals only; '
            'motion authority is permanently false'
        )

    @staticmethod
    def _stamp(msg: Image) -> float:
        return float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9

    def _request_callback(self, msg: String) -> None:
        if not self._enabled:
            return
        try:
            self._request = recovery_request_from_mapping(json.loads(msg.data))
            self._request_mono = time.monotonic()
            self._last_error = ''
            self._try_plan()
        except (json.JSONDecodeError, RecoveryError, TypeError, ValueError) as exc:
            self._request = None
            self._request_mono = None
            self._last_error = f'invalid recovery request: {exc}'

    def _road_status_callback(self, msg: String) -> None:
        if not self._enabled:
            return
        try:
            payload = json.loads(msg.data)
            if not isinstance(payload, dict):
                raise ValueError('road status must be an object')
            self._road_status = payload
            self._road_mono = time.monotonic()
            self._try_plan()
        except (json.JSONDecodeError, TypeError, ValueError) as exc:
            self._last_error = f'invalid road status: {exc}'

    def _trajectory_status_callback(self, msg: String) -> None:
        if not self._enabled:
            return
        try:
            payload = json.loads(msg.data)
            if not isinstance(payload, dict):
                raise ValueError('trajectory status must be an object')
            self._trajectory_status = payload
            self._trajectory_mono = time.monotonic()
            self._try_plan()
        except (json.JSONDecodeError, TypeError, ValueError) as exc:
            self._last_error = f'invalid trajectory status: {exc}'

    def _image(self, msg: Image, coverage: bool) -> None:
        if not self._enabled:
            return
        try:
            image = self._bridge.imgmsg_to_cv2(msg, desired_encoding='mono8').copy()
            stamp = self._stamp(msg)
            if coverage:
                self._coverage, self._coverage_stamp = image, stamp
                self._coverage_mono = time.monotonic()
            else:
                self._road_mask, self._road_stamp = image, stamp
                self._road_mask_mono = time.monotonic()
            self._try_plan()
        except CvBridgeError as exc:
            self._last_error = str(exc)

    def _road_mask_callback(self, msg: Image) -> None:
        self._image(msg, False)

    def _coverage_callback(self, msg: Image) -> None:
        self._image(msg, True)

    def _scan_callback(self, msg: LaserScan) -> None:
        if not self._enabled:
            return
        if not all(math.isfinite(float(value)) for value in (msg.angle_min, msg.angle_increment)):
            self._last_error = 'LiDAR angle metadata is non-finite'
            return
        lx, ly, yaw = self._lidar_pose
        cosine, sine = math.cos(yaw), math.sin(yaw)
        points = []
        for index, raw_range in enumerate(msg.ranges):
            distance = float(raw_range)
            if not math.isfinite(distance) or not self._scan_min <= distance <= self._scan_max:
                continue
            angle = float(msg.angle_min) + index * float(msg.angle_increment)
            sensor_x, sensor_y = distance * math.cos(angle), distance * math.sin(angle)
            points.append((lx + sensor_x * cosine - sensor_y * sine,
                           ly + sensor_x * sine + sensor_y * cosine))
        self._scan_points = points
        self._scan_mono = time.monotonic()
        self._try_plan()

    def _corridor(self):
        if self._road_status is None:
            return []
        corridor = self._road_status.get('corridor', {})
        if not isinstance(corridor, dict):
            return []
        raw = corridor.get('primary', [])
        try:
            return [(float(item['forward_m']), float(item['left_m'])) for item in raw]
        except (KeyError, TypeError, ValueError):
            return []

    def _blockers(self, now: float) -> List[str]:
        blockers = []
        profile = self._profiles.get('secondary')
        if profile is None or not profile.calibrated:
            blockers.append('secondary camera profile is uncalibrated')
        labels = {
            'recovery_request_source_validated': 'recovery request source is not validated',
            'hard_hold_source_validated': 'hard-hold source is not validated',
            'forward_status_source_validated': 'forward-status source is not validated',
            'rear_coverage_validated': 'rear camera coverage is not validated',
            'vehicle_geometry_validated': 'vehicle geometry is not validated',
            'minimum_turn_radius_validated': 'minimum turn radius is not validated',
            'lidar_extrinsics_validated': 'LiDAR extrinsics are not validated',
            'road_tolerance_validated': 'road-edge tolerance is not validated',
        }
        blockers.extend(label for gate, label in labels.items() if not self._gates[gate])
        if self._request is None or self._request_mono is None:
            blockers.append('no recovery request')
        else:
            if now - self._request_mono > self._request_timeout:
                blockers.append('recovery request is stale')
            blockers.extend(request_blockers(self._request, self._config))
        if self._road_status is None or self._road_mono is None:
            blockers.append('no road status')
        else:
            if now - self._road_mono > self._road_timeout:
                blockers.append('road status is stale')
            if not bool(self._road_status.get('thresholds_validated', False)):
                blockers.append('road thresholds are not validated')
            if len(self._corridor()) < 2:
                blockers.append('no usable forward corridor')
        if self._trajectory_status is None or self._trajectory_mono is None:
            blockers.append('no forward trajectory status')
        elif now - self._trajectory_mono > self._trajectory_timeout:
            blockers.append('forward trajectory status is stale')
        elif not forward_candidates_exhausted(self._trajectory_status):
            blockers.append('forward candidates are not proven exhausted')
        for value, stamp, label in (
            (self._road_mask, self._road_mask_mono, 'recovery road mask'),
            (self._coverage, self._coverage_mono, 'rear coverage mask'),
        ):
            if value is None or stamp is None:
                blockers.append(f'no {label}')
            elif now - stamp > self._road_timeout:
                blockers.append(f'{label} is stale')
        if self._road_stamp is not None and self._coverage_stamp is not None:
            if abs(self._road_stamp - self._coverage_stamp) > self._sync_tolerance:
                blockers.append('road and rear-coverage timestamps do not match')
        if self._request is not None and self._road_stamp is not None:
            if abs(self._request.image_stamp_sec - self._road_stamp) > self._sync_tolerance:
                blockers.append('request and road-mask timestamps do not match')
        if self._scan_mono is None:
            blockers.append('no LiDAR scan')
        elif now - self._scan_mono > self._scan_timeout:
            blockers.append('LiDAR scan is stale')
        return list(dict.fromkeys(blockers))

    def _try_plan(self) -> None:
        now = time.monotonic()
        blockers = self._blockers(now)
        if blockers:
            self._candidate_reports, self._selected = [], None
            self._last_error = '; '.join(blockers)
            return
        signature = (
            self._request.image_stamp_sec, self._road_stamp,
            round(self._scan_mono, 3), round(self._trajectory_mono, 3),
        )
        if signature == self._last_signature or now - self._last_plan_mono < self._minimum_plan_interval:
            return
        started = time.perf_counter()
        try:
            candidates = plan_recovery(
                self._corridor(), self._road_mask, self._coverage,
                self._profiles['secondary'], self._geometry, self._config,
                self._scan_points,
            )
        except (CalibrationError, RecoveryError, TrajectoryError, ValueError) as exc:
            self._candidate_reports, self._selected = [], None
            self._last_error = str(exc)
            return
        self._last_plan_duration_ms = (time.perf_counter() - started) * 1000.0
        self._last_plan_mono, self._last_signature = now, signature
        self._processed_plans += 1
        self._candidate_reports = [self._report(item) for item in candidates[:24]]
        winner = next((item for item in candidates if item.valid), None)
        self._selected = None if winner is None else self._report(winner)
        self._last_error = '' if winner is not None else 'no safe recovery proposal'
        if winner is not None:
            self._publish_path(winner)

    @staticmethod
    def _report(candidate) -> Dict[str, object]:
        path = candidate.path
        return {
            'goal_index': candidate.goal_index, 'family': path.family,
            'path_length_m': round(path.path_length_m, 4),
            'reverse_distance_m': round(path.reverse_distance_m, 4),
            'gear_changes': path.gear_changes, 'cost': round(path.cost, 5),
            'rear_evidence_fraction': round(candidate.rear_evidence_fraction, 4),
            'minimum_road_support': round(path.minimum_road_support, 4),
            'road_blocked_samples': path.road_blocked_samples,
            'obstacle_blocked_samples': path.obstacle_blocked_samples,
            'valid': candidate.valid, 'reject_reason': candidate.reject_reason,
        }

    def _publish_path(self, candidate) -> None:
        path = candidate.path
        stride = max(1, int(math.ceil(len(path.points) / 120.0)))
        points = path.points[::stride]
        if points[-1] is not path.points[-1]:
            points.append(path.points[-1])
        payload = {
            'algorithm_stage': 6, 'diagnostic_only': True, 'can_execute': False,
            'image_stamp_sec': self._request.image_stamp_sec,
            'family': path.family,
            'points': [{
                'forward_m': round(point.x, 4), 'left_m': round(point.y, 4),
                'yaw_rad': round(point.yaw, 5), 'direction': point.direction,
                'curvature_per_m': round(point.curvature, 5),
            } for point in points],
        }
        msg = String(data=json.dumps(payload, separators=(',', ':'), sort_keys=True))
        self._path_pub.publish(msg)

    def _publish_status(self) -> None:
        blockers = self._blockers(time.monotonic()) if self._enabled else ['node disabled']
        payload = {
            'algorithm_stage': 6, 'mode': 'shadow', 'enabled': self._enabled,
            'motion_authority': False, 'can_publish_motion': False,
            'can_execute_proposed_path': False, 'validation_gates': self._gates,
            'profile_error': self._profile_error,
            'profile': None if 'secondary' not in self._profiles else profile_report(self._profiles['secondary']),
            'blockers': blockers, 'processed_plans': self._processed_plans,
            'last_plan_duration_ms': None if self._last_plan_duration_ms is None else round(self._last_plan_duration_ms, 2),
            'obstacle_points': len(self._scan_points), 'last_error': self._last_error,
            'selected_diagnostic_only': self._selected, 'candidates': self._candidate_reports,
        }
        self._status_pub.publish(String(data=json.dumps(payload, separators=(',', ':'), sort_keys=True)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RecoveryShadow()
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
