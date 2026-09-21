"""Stage 5 diagnostic Reeds-Shepp parking planner.

The node publishes JSON proposals only. It never subscribes to the live parking
command and has no vehicle-command publisher.
"""

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
from .parking_core import (
    ParkingConfig,
    ParkingError,
    ParkingGoal,
    goal_fits_slot,
    parking_goal_from_mapping,
    plan_parking,
)
from .trajectory_core import PathPoint, TrajectoryError, VehicleGeometry


STATUS_TOPIC = '/v4_experimental/parking/status'
PATH_TOPIC = '/v4_experimental/parking/proposed_path'


class ParkingShadow(Node):
    def __init__(self) -> None:
        super().__init__('v4_parking_shadow')
        self.declare_parameter('enabled', False)
        self.declare_parameter('profile_path', '')
        self.declare_parameter('goal_topic', '/v4_experimental/parking/goal')
        self.declare_parameter('road_status_topic', '/v4_experimental/road/status')
        self.declare_parameter('parking_mask_topic', '/v4_experimental/road/secondary/fused')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('required_goal_frame', 'base_link')
        self.declare_parameter('goal_timeout_sec', 0.50)
        self.declare_parameter('road_timeout_sec', 0.35)
        self.declare_parameter('scan_timeout_sec', 0.50)
        self.declare_parameter('sync_tolerance_sec', 0.10)
        self.declare_parameter('maximum_plan_hz', 0.5)
        self.declare_parameter('minimum_goal_confidence', 0.75)
        self.declare_parameter('slot_clearance_m', 0.005)

        self.declare_parameter('parking_goal_source_validated', False)
        self.declare_parameter('slot_geometry_validated', False)
        self.declare_parameter('rear_coverage_validated', False)
        self.declare_parameter('vehicle_geometry_validated', False)
        self.declare_parameter('minimum_turn_radius_validated', False)
        self.declare_parameter('lidar_extrinsics_validated', False)

        self.declare_parameter('vehicle_length_m', 0.300)
        self.declare_parameter('vehicle_width_m', 0.192)
        self.declare_parameter('wheelbase_m', 0.210)
        self.declare_parameter('rear_overhang_m', 0.042)
        self.declare_parameter('minimum_turn_radius_m', 0.400)
        self.declare_parameter('footprint_padding_m', 0.005)
        self.declare_parameter('lidar_x_m', 0.0)
        self.declare_parameter('lidar_y_m', 0.0)
        self.declare_parameter('lidar_yaw_rad', 0.0)
        self.declare_parameter('minimum_scan_range_m', 0.03)
        self.declare_parameter('maximum_scan_range_m', 2.0)

        self.declare_parameter('sample_step_m', 0.006)
        self.declare_parameter('footprint_sample_spacing_m', 0.02)
        self.declare_parameter('minimum_road_support', 0.98)
        self.declare_parameter('obstacle_margin_m', 0.015)
        self.declare_parameter('maximum_gear_changes', 4)
        self.declare_parameter('maximum_reverse_distance_m', 1.20)

        self._enabled = bool(self.get_parameter('enabled').value)
        self._required_frame = str(self.get_parameter('required_goal_frame').value)
        self._goal_timeout = float(self.get_parameter('goal_timeout_sec').value)
        self._road_timeout = float(self.get_parameter('road_timeout_sec').value)
        self._scan_timeout = float(self.get_parameter('scan_timeout_sec').value)
        self._sync_tolerance = float(self.get_parameter('sync_tolerance_sec').value)
        maximum_plan_hz = float(self.get_parameter('maximum_plan_hz').value)
        if not math.isfinite(maximum_plan_hz) or maximum_plan_hz <= 0.0:
            raise ParkingError('maximum_plan_hz must be finite and positive')
        self._minimum_plan_interval = 1.0 / maximum_plan_hz
        self._minimum_confidence = float(
            self.get_parameter('minimum_goal_confidence').value
        )
        self._slot_clearance = float(self.get_parameter('slot_clearance_m').value)
        if not 0.0 <= self._minimum_confidence <= 1.0:
            raise ParkingError('minimum goal confidence must be in [0, 1]')

        self._gates = {
            name: bool(self.get_parameter(name).value)
            for name in (
                'parking_goal_source_validated', 'slot_geometry_validated',
                'rear_coverage_validated', 'vehicle_geometry_validated',
                'minimum_turn_radius_validated', 'lidar_extrinsics_validated',
            )
        }
        self._geometry = VehicleGeometry(
            length_m=float(self.get_parameter('vehicle_length_m').value),
            width_m=float(self.get_parameter('vehicle_width_m').value),
            wheelbase_m=float(self.get_parameter('wheelbase_m').value),
            rear_overhang_m=float(self.get_parameter('rear_overhang_m').value),
            minimum_turn_radius_m=float(
                self.get_parameter('minimum_turn_radius_m').value
            ),
            footprint_padding_m=float(
                self.get_parameter('footprint_padding_m').value
            ),
        )
        self._config = ParkingConfig(
            sample_step_m=float(self.get_parameter('sample_step_m').value),
            footprint_sample_spacing_m=float(
                self.get_parameter('footprint_sample_spacing_m').value
            ),
            minimum_road_support=float(
                self.get_parameter('minimum_road_support').value
            ),
            obstacle_margin_m=float(
                self.get_parameter('obstacle_margin_m').value
            ),
            maximum_gear_changes=int(
                self.get_parameter('maximum_gear_changes').value
            ),
            maximum_reverse_distance_m=float(
                self.get_parameter('maximum_reverse_distance_m').value
            ),
        )
        self._geometry.validate()
        self._config.validate()

        self._scan_min = float(self.get_parameter('minimum_scan_range_m').value)
        self._scan_max = float(self.get_parameter('maximum_scan_range_m').value)
        self._lidar_pose = (
            float(self.get_parameter('lidar_x_m').value),
            float(self.get_parameter('lidar_y_m').value),
            float(self.get_parameter('lidar_yaw_rad').value),
        )
        if not all(math.isfinite(value) for value in self._lidar_pose):
            raise ParkingError('LiDAR extrinsics must be finite')
        if (
            not math.isfinite(self._scan_min)
            or not math.isfinite(self._scan_max)
            or self._scan_min < 0.0
            or self._scan_max <= self._scan_min
        ):
            raise ParkingError('scan range limits are invalid')

        self._profiles = {}
        self._profile_error = ''
        try:
            self._profiles = load_profiles(
                str(self.get_parameter('profile_path').value)
            )
        except (CalibrationError, OSError, ValueError) as exc:
            self._profile_error = str(exc)

        self._bridge = CvBridge()
        self._goal: Optional[ParkingGoal] = None
        self._goal_mono: Optional[float] = None
        self._road_status: Optional[Dict[str, object]] = None
        self._road_status_mono: Optional[float] = None
        self._mask = None
        self._mask_stamp: Optional[float] = None
        self._mask_mono: Optional[float] = None
        self._scan_points: List[Tuple[float, float]] = []
        self._scan_mono: Optional[float] = None
        self._last_plan_mono = 0.0
        self._last_signature = None
        self._last_plan_duration_ms: Optional[float] = None
        self._last_error = ''
        self._processed_plans = 0
        self._candidate_reports: List[Dict[str, object]] = []
        self._selected: Optional[Dict[str, object]] = None

        self._status_pub = self.create_publisher(String, STATUS_TOPIC, 10)
        self._path_pub = self.create_publisher(String, PATH_TOPIC, 2)
        self.create_subscription(
            String, str(self.get_parameter('goal_topic').value),
            self._goal_callback, qos_profile_sensor_data,
        )
        self.create_subscription(
            String, str(self.get_parameter('road_status_topic').value),
            self._road_status_callback, qos_profile_sensor_data,
        )
        self.create_subscription(
            Image, str(self.get_parameter('parking_mask_topic').value),
            self._mask_callback, qos_profile_sensor_data,
        )
        self.create_subscription(
            LaserScan, str(self.get_parameter('scan_topic').value),
            self._scan_callback, qos_profile_sensor_data,
        )
        self.create_timer(0.5, self._publish_status)
        state = 'enabled' if self._enabled else 'disabled'
        self.get_logger().warning(
            f'V4 parking shadow is {state}; proposed paths only; '
            'motion authority is permanently false'
        )

    def _goal_callback(self, msg: String) -> None:
        if not self._enabled:
            return
        try:
            raw = json.loads(msg.data)
            self._goal = parking_goal_from_mapping(raw, self._required_frame)
            self._goal_mono = time.monotonic()
            self._last_error = ''
            self._try_plan()
        except (json.JSONDecodeError, ParkingError, TypeError, ValueError) as exc:
            self._goal = None
            self._goal_mono = None
            self._last_error = f'invalid parking goal: {exc}'

    def _road_status_callback(self, msg: String) -> None:
        if not self._enabled:
            return
        try:
            payload = json.loads(msg.data)
            if not isinstance(payload, dict):
                raise ValueError('road status must be an object')
            self._road_status = payload
            self._road_status_mono = time.monotonic()
            self._try_plan()
        except (json.JSONDecodeError, TypeError, ValueError) as exc:
            self._last_error = f'invalid road status: {exc}'

    def _mask_callback(self, msg: Image) -> None:
        if not self._enabled:
            return
        try:
            self._mask = self._bridge.imgmsg_to_cv2(
                msg, desired_encoding='mono8'
            ).copy()
            self._mask_stamp = (
                float(msg.header.stamp.sec)
                + float(msg.header.stamp.nanosec) * 1e-9
            )
            self._mask_mono = time.monotonic()
            self._try_plan()
        except CvBridgeError as exc:
            self._last_error = str(exc)

    def _scan_callback(self, msg: LaserScan) -> None:
        if not self._enabled:
            return
        if not all(
            math.isfinite(float(value))
            for value in (msg.angle_min, msg.angle_increment)
        ):
            self._last_error = 'LiDAR angle metadata is non-finite'
            return
        lx, ly, yaw = self._lidar_pose
        cosine = math.cos(yaw)
        sine = math.sin(yaw)
        points = []
        for index, raw_range in enumerate(msg.ranges):
            distance = float(raw_range)
            if (
                not math.isfinite(distance)
                or distance < self._scan_min
                or distance > self._scan_max
            ):
                continue
            angle = float(msg.angle_min) + index * float(msg.angle_increment)
            sensor_x = distance * math.cos(angle)
            sensor_y = distance * math.sin(angle)
            points.append((
                lx + sensor_x * cosine - sensor_y * sine,
                ly + sensor_x * sine + sensor_y * cosine,
            ))
        self._scan_points = points
        self._scan_mono = time.monotonic()
        self._try_plan()

    def _expected_mask_stamp(self) -> Optional[float]:
        if self._road_status is None:
            return None
        stamps = self._road_status.get('last_image_stamp_sec')
        if not isinstance(stamps, dict):
            return None
        try:
            value = float(stamps.get('secondary'))
        except (TypeError, ValueError):
            return None
        return value if math.isfinite(value) and value >= 0.0 else None

    def _blockers(self, now: float) -> List[str]:
        blockers = []
        profile = self._profiles.get('secondary')
        if profile is None or not profile.calibrated:
            blockers.append('secondary camera profile is uncalibrated')
        gate_labels = {
            'parking_goal_source_validated': 'parking goal source is not validated',
            'slot_geometry_validated': 'parking slot geometry is not validated',
            'rear_coverage_validated': 'rear camera coverage is not validated',
            'vehicle_geometry_validated': 'vehicle geometry is not validated',
            'minimum_turn_radius_validated': 'minimum turn radius is not validated',
            'lidar_extrinsics_validated': 'LiDAR extrinsics are not validated',
        }
        blockers.extend(
            label for gate, label in gate_labels.items() if not self._gates[gate]
        )
        if self._goal is None or self._goal_mono is None:
            blockers.append('no measured parking goal')
        else:
            if now - self._goal_mono > self._goal_timeout:
                blockers.append('parking goal is stale')
            if self._goal.confidence < self._minimum_confidence:
                blockers.append('parking goal confidence is below threshold')
            mask_stamp = -1.0 if self._mask_stamp is None else self._mask_stamp
            if abs(self._goal.image_stamp_sec - mask_stamp) > self._sync_tolerance:
                blockers.append('parking goal and mask timestamps do not match')
        if self._road_status is None or self._road_status_mono is None:
            blockers.append('no road status')
        elif now - self._road_status_mono > self._road_timeout:
            blockers.append('road status is stale')
        elif not bool(self._road_status.get('thresholds_validated', False)):
            blockers.append('road thresholds are not validated')
        if self._mask is None or self._mask_mono is None:
            blockers.append('no parking-area mask')
        elif now - self._mask_mono > self._road_timeout:
            blockers.append('parking-area mask is stale')
        expected_stamp = self._expected_mask_stamp()
        if expected_stamp is None or self._mask_stamp is None:
            blockers.append('parking-area mask timestamp unavailable')
        elif abs(expected_stamp - self._mask_stamp) > self._sync_tolerance:
            blockers.append('parking-area mask and road status timestamps do not match')
        if self._scan_mono is None:
            blockers.append('no LiDAR scan')
        elif now - self._scan_mono > self._scan_timeout:
            blockers.append('LiDAR scan is stale')
        return blockers

    def _try_plan(self) -> None:
        now = time.monotonic()
        blockers = self._blockers(now)
        if blockers:
            self._candidate_reports = []
            self._selected = None
            self._last_error = '; '.join(blockers)
            return
        signature = (
            self._goal.image_stamp_sec, self._mask_stamp,
            round(self._scan_mono, 3), self._goal.kind,
        )
        if self._last_signature == signature:
            return
        if now - self._last_plan_mono < self._minimum_plan_interval:
            return
        if not goal_fits_slot(
            self._goal.slot_length_m, self._goal.slot_width_m,
            self._geometry, self._slot_clearance,
        ):
            self._candidate_reports = []
            self._selected = None
            self._last_error = 'measured parking slot cannot contain full vehicle footprint'
            return

        started = time.perf_counter()
        try:
            candidates = plan_parking(
                PathPoint(0.0, 0.0, 0.0), self._goal.target,
                self._mask, self._profiles['secondary'], self._geometry,
                self._config, self._scan_points,
                require_reverse=self._goal.kind == 'parallel',
            )
        except (CalibrationError, ParkingError, TrajectoryError, ValueError) as exc:
            self._candidate_reports = []
            self._selected = None
            self._last_error = str(exc)
            return
        self._last_plan_duration_ms = (time.perf_counter() - started) * 1000.0
        self._last_plan_mono = now
        self._last_signature = signature
        self._processed_plans += 1
        self._candidate_reports = [self._candidate_report(item) for item in candidates[:24]]
        winner = next((item for item in candidates if item.valid), None)
        self._selected = None if winner is None else self._candidate_report(winner)
        self._last_error = '' if winner is not None else 'no collision-free parking proposal'
        if winner is not None:
            self._publish_path(winner)

    @staticmethod
    def _candidate_report(candidate) -> Dict[str, object]:
        endpoint = candidate.points[-1]
        return {
            'id': candidate.candidate_id,
            'family': candidate.family,
            'stage': candidate.stage,
            'segment_lengths_m': [round(value, 4) for value in candidate.segment_lengths_m],
            'path_length_m': round(candidate.path_length_m, 4),
            'reverse_distance_m': round(candidate.reverse_distance_m, 4),
            'gear_changes': candidate.gear_changes,
            'cost': round(candidate.cost, 5),
            'valid': candidate.valid,
            'minimum_road_support': round(candidate.minimum_road_support, 4),
            'road_blocked_samples': candidate.road_blocked_samples,
            'obstacle_blocked_samples': candidate.obstacle_blocked_samples,
            'reject_reason': candidate.reject_reason,
            'endpoint_m': {
                'forward': round(endpoint.x, 4),
                'left': round(endpoint.y, 4),
                'yaw': round(endpoint.yaw, 4),
            },
        }

    def _publish_path(self, candidate) -> None:
        stride = max(1, int(math.ceil(len(candidate.points) / 120.0)))
        points = candidate.points[::stride]
        if points[-1] is not candidate.points[-1]:
            points.append(candidate.points[-1])
        payload = {
            'algorithm_stage': 5,
            'diagnostic_only': True,
            'can_execute': False,
            'image_stamp_sec': self._goal.image_stamp_sec,
            'kind': self._goal.kind,
            'family': candidate.family,
            'stage': candidate.stage,
            'points': [{
                'forward_m': round(point.x, 4),
                'left_m': round(point.y, 4),
                'yaw_rad': round(point.yaw, 5),
                'direction': point.direction,
                'curvature_per_m': round(point.curvature, 5),
            } for point in points],
        }
        msg = String()
        msg.data = json.dumps(payload, separators=(',', ':'), sort_keys=True)
        self._path_pub.publish(msg)

    def _publish_status(self) -> None:
        blockers = self._blockers(time.monotonic()) if self._enabled else ['node disabled']
        payload = {
            'algorithm_stage': 5,
            'mode': 'shadow',
            'enabled': self._enabled,
            'motion_authority': False,
            'can_publish_motion': False,
            'can_execute_proposed_path': False,
            'validation_gates': self._gates,
            'profile_error': self._profile_error,
            'profile': None if 'secondary' not in self._profiles else profile_report(
                self._profiles['secondary']
            ),
            'goal': None if self._goal is None else {
                'kind': self._goal.kind,
                'frame_id': self._goal.frame_id,
                'confidence': self._goal.confidence,
                'image_stamp_sec': self._goal.image_stamp_sec,
                'target_rear_axle_m': {
                    'forward': self._goal.target.x,
                    'left': self._goal.target.y,
                    'yaw': self._goal.target.yaw,
                },
                'slot_length_m': self._goal.slot_length_m,
                'slot_width_m': self._goal.slot_width_m,
            },
            'blockers': blockers,
            'processed_plans': self._processed_plans,
            'last_plan_duration_ms': None if self._last_plan_duration_ms is None else round(
                self._last_plan_duration_ms, 2
            ),
            'obstacle_points': len(self._scan_points),
            'last_error': self._last_error,
            'selected_diagnostic_only': self._selected,
            'candidates': self._candidate_reports,
        }
        msg = String()
        msg.data = json.dumps(payload, separators=(',', ':'), sort_keys=True)
        self._status_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ParkingShadow()
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
