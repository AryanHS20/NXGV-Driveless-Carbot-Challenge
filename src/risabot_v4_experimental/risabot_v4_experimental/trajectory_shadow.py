"""Stage 4 debug-only trajectory candidate generator.

Candidate generation is blocked until camera, road thresholds, vehicle geometry,
and LiDAR extrinsics have each been explicitly validated.
"""

import json
import math
import time
from typing import Dict, List, Optional, Tuple

from cv_bridge import CvBridge, CvBridgeError
import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String

from .bev_core import CalibrationError, load_profiles, profile_report
from .trajectory_core import (
    TrajectoryConfig,
    TrajectoryError,
    VehicleGeometry,
    generate_candidates,
    reference_from_corridor,
)


STATUS_TOPIC = '/v4_experimental/trajectory/status'


class TrajectoryShadow(Node):
    def __init__(self) -> None:
        super().__init__('v4_trajectory_shadow')
        self.declare_parameter('enabled', False)
        self.declare_parameter('profile_path', '')
        self.declare_parameter('road_status_topic', '/v4_experimental/road/status')
        self.declare_parameter('road_mask_topic', '/v4_experimental/road/primary/fused')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('road_timeout_sec', 0.35)
        self.declare_parameter('scan_timeout_sec', 0.50)
        self.declare_parameter('road_mask_sync_tolerance_sec', 0.02)
        self.declare_parameter('vehicle_geometry_validated', False)
        self.declare_parameter('minimum_turn_radius_validated', False)
        self.declare_parameter('lidar_extrinsics_validated', False)
        self.declare_parameter('vehicle_length_m', 0.300)
        self.declare_parameter('vehicle_width_m', 0.192)
        self.declare_parameter('wheelbase_m', 0.216)
        self.declare_parameter('rear_overhang_m', 0.042)
        self.declare_parameter('minimum_turn_radius_m', 0.400)
        self.declare_parameter('footprint_padding_m', 0.005)
        self.declare_parameter('lidar_x_m', 0.0)
        self.declare_parameter('lidar_y_m', 0.0)
        self.declare_parameter('lidar_yaw_rad', 0.0)
        self.declare_parameter('minimum_scan_range_m', 0.03)
        self.declare_parameter('maximum_scan_range_m', 2.0)
        for name, value in (
            ('horizon_m', 0.55), ('step_m', 0.01),
            ('lookahead_m', 0.095), ('rollout_speed_mps', 0.08),
            ('steering_lag_sec', 0.12),
            ('steering_rate_rad_sec', math.pi),
            ('footprint_sample_spacing_m', 0.02),
            ('minimum_road_support', 0.98), ('obstacle_margin_m', 0.015),
        ):
            self.declare_parameter(name, value)

        self._enabled = bool(self.get_parameter('enabled').value)
        self._geometry_validated = bool(
            self.get_parameter('vehicle_geometry_validated').value
        )
        self._turn_radius_validated = bool(
            self.get_parameter('minimum_turn_radius_validated').value
        )
        self._lidar_validated = bool(
            self.get_parameter('lidar_extrinsics_validated').value
        )
        self._road_timeout = float(self.get_parameter('road_timeout_sec').value)
        self._scan_timeout = float(self.get_parameter('scan_timeout_sec').value)
        self._sync_tolerance = float(
            self.get_parameter('road_mask_sync_tolerance_sec').value
        )
        self._scan_min = float(self.get_parameter('minimum_scan_range_m').value)
        self._scan_max = float(self.get_parameter('maximum_scan_range_m').value)
        self._lidar_pose = (
            float(self.get_parameter('lidar_x_m').value),
            float(self.get_parameter('lidar_y_m').value),
            float(self.get_parameter('lidar_yaw_rad').value),
        )
        if not all(math.isfinite(value) for value in self._lidar_pose):
            raise TrajectoryError('LiDAR extrinsics must be finite')
        if (
            not math.isfinite(self._scan_min)
            or not math.isfinite(self._scan_max)
            or self._scan_min < 0.0
            or self._scan_max <= self._scan_min
        ):
            raise TrajectoryError('scan range limits are invalid')
        self._geometry = VehicleGeometry(
            length_m=float(self.get_parameter('vehicle_length_m').value),
            width_m=float(self.get_parameter('vehicle_width_m').value),
            wheelbase_m=float(self.get_parameter('wheelbase_m').value),
            rear_overhang_m=float(self.get_parameter('rear_overhang_m').value),
            minimum_turn_radius_m=float(self.get_parameter('minimum_turn_radius_m').value),
            footprint_padding_m=float(self.get_parameter('footprint_padding_m').value),
        )
        self._config = self._trajectory_config_from_parameters()
        self._geometry.validate()
        self._config.validate()
        self.add_on_set_parameters_callback(self._on_parameters)

        self._profiles = {}
        self._profile_error = ''
        try:
            self._profiles = load_profiles(str(self.get_parameter('profile_path').value))
        except (CalibrationError, OSError, ValueError) as exc:
            self._profile_error = str(exc)

        self._bridge = CvBridge()
        self._road_status: Optional[Dict[str, object]] = None
        self._road_status_mono: Optional[float] = None
        self._scan_points: List[Tuple[float, float]] = []
        self._scan_mono: Optional[float] = None
        self._mask = None
        self._mask_stamp: Optional[float] = None
        self._mask_mono: Optional[float] = None
        self._last_processed_stamp: Optional[float] = None
        self._last_candidates: List[Dict[str, object]] = []
        self._selected: Optional[Dict[str, object]] = None
        self._last_error = ''
        self._processed_frames = 0

        self._status_pub = self.create_publisher(String, STATUS_TOPIC, 10)
        self.create_subscription(
            String,
            str(self.get_parameter('road_status_topic').value),
            self._road_status_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            str(self.get_parameter('road_mask_topic').value),
            self._mask_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            LaserScan,
            str(self.get_parameter('scan_topic').value),
            self._scan_callback,
            qos_profile_sensor_data,
        )
        self.create_timer(0.2, self._publish_status)
        state = 'enabled' if self._enabled else 'disabled'
        self.get_logger().warning(
            f'V4 trajectory shadow is {state}; candidate reports only; '
            'motion authority is permanently false'
        )

    def _trajectory_config_from_parameters(self, overrides=None) -> TrajectoryConfig:
        overrides = overrides or {}
        names = (
            'horizon_m', 'step_m', 'lookahead_m', 'rollout_speed_mps',
            'steering_lag_sec', 'steering_rate_rad_sec',
            'footprint_sample_spacing_m', 'minimum_road_support',
            'obstacle_margin_m',
        )
        values = {
            name: float(overrides.get(name, self.get_parameter(name).value))
            for name in names
        }
        return TrajectoryConfig(**values)

    def _on_parameters(self, parameters) -> SetParametersResult:
        safe = {
            'horizon_m', 'step_m', 'lookahead_m', 'rollout_speed_mps',
            'steering_lag_sec', 'steering_rate_rad_sec',
            'footprint_sample_spacing_m', 'minimum_road_support',
            'obstacle_margin_m',
        }
        protected = {
            'enabled', 'profile_path', 'vehicle_geometry_validated',
            'minimum_turn_radius_validated', 'lidar_extrinsics_validated',
            'vehicle_length_m', 'vehicle_width_m', 'wheelbase_m',
            'rear_overhang_m', 'minimum_turn_radius_m',
            'footprint_padding_m', 'lidar_x_m', 'lidar_y_m',
            'lidar_yaw_rad', 'minimum_scan_range_m', 'maximum_scan_range_m',
            'road_status_topic', 'road_mask_topic', 'scan_topic',
            'road_timeout_sec', 'scan_timeout_sec',
            'road_mask_sync_tolerance_sec',
        }
        overrides = {}
        for parameter in parameters:
            if parameter.name in protected:
                return SetParametersResult(
                    successful=False,
                    reason=f'{parameter.name} requires a node restart',
                )
            if parameter.name in safe:
                overrides[parameter.name] = parameter.value
        if not overrides:
            return SetParametersResult(successful=True)
        try:
            current = {name: getattr(self._config, name) for name in safe}
            current.update({name: float(value) for name, value in overrides.items()})
            config = TrajectoryConfig(**current)
            config.validate()
        except (TrajectoryError, TypeError, ValueError) as exc:
            return SetParametersResult(successful=False, reason=str(exc))
        self._config = config
        return SetParametersResult(successful=True)

    def _road_status_callback(self, msg: String) -> None:
        if not self._enabled:
            return
        try:
            payload = json.loads(msg.data)
            if not isinstance(payload, dict):
                raise ValueError('road status must be an object')
            self._road_status = payload
            self._road_status_mono = time.monotonic()
            self._try_process()
        except (json.JSONDecodeError, TypeError, ValueError) as exc:
            self._last_error = f'invalid road status: {exc}'

    def _scan_callback(self, msg: LaserScan) -> None:
        if not self._enabled:
            return
        if not all(math.isfinite(float(value)) for value in (msg.angle_min, msg.angle_increment)):
            self._last_error = 'LiDAR angle metadata is non-finite'
            return
        lx, ly, lyaw = self._lidar_pose
        cosine = math.cos(lyaw)
        sine = math.sin(lyaw)
        points = []
        for index, range_m in enumerate(msg.ranges):
            value = float(range_m)
            if not math.isfinite(value) or value < self._scan_min or value > self._scan_max:
                continue
            angle = float(msg.angle_min) + index * float(msg.angle_increment)
            sensor_x = value * math.cos(angle)
            sensor_y = value * math.sin(angle)
            points.append((
                lx + sensor_x * cosine - sensor_y * sine,
                ly + sensor_x * sine + sensor_y * cosine,
            ))
        self._scan_points = points
        self._scan_mono = time.monotonic()

    def _expected_road_stamp(self) -> Optional[float]:
        if self._road_status is None:
            return None
        stamps = self._road_status.get('last_image_stamp_sec')
        if not isinstance(stamps, dict):
            return None
        try:
            value = float(stamps.get('primary'))
        except (TypeError, ValueError):
            return None
        return value if math.isfinite(value) and value >= 0.0 else None

    def _blockers(self, now: float) -> List[str]:
        blockers = []
        profile = self._profiles.get('primary')
        if profile is None or not profile.calibrated:
            blockers.append('primary camera profile is uncalibrated')
        if not self._geometry_validated:
            blockers.append('vehicle geometry is not measured and validated')
        if not self._turn_radius_validated:
            blockers.append('minimum turn radius is not measured and validated')
        if not self._lidar_validated:
            blockers.append('LiDAR extrinsics are not measured and validated')
        if self._road_status is None or self._road_status_mono is None:
            blockers.append('no road status')
        elif now - self._road_status_mono > self._road_timeout:
            blockers.append('road status is stale')
        elif not bool(self._road_status.get('thresholds_validated', False)):
            blockers.append('road thresholds are not validated')
        if self._scan_mono is None:
            blockers.append('no LiDAR scan')
        elif now - self._scan_mono > self._scan_timeout:
            blockers.append('LiDAR scan is stale')
        if self._mask is None or self._mask_mono is None:
            blockers.append('no road mask')
        elif now - self._mask_mono > self._road_timeout:
            blockers.append('road mask is stale')
        expected_stamp = self._expected_road_stamp()
        if expected_stamp is None or self._mask_stamp is None:
            blockers.append('road mask timestamp unavailable')
        elif abs(float(expected_stamp) - self._mask_stamp) > self._sync_tolerance:
            blockers.append('road mask and corridor timestamps do not match')
        return blockers

    def _mask_callback(self, msg: Image) -> None:
        if not self._enabled:
            return
        try:
            self._mask = self._bridge.imgmsg_to_cv2(msg, desired_encoding='mono8').copy()
            self._mask_stamp = (
                float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
            )
            self._mask_mono = time.monotonic()
            self._try_process()
        except CvBridgeError as exc:
            self._last_error = str(exc)

    def _try_process(self) -> None:
        now = time.monotonic()
        blockers = self._blockers(now)
        if blockers:
            self._last_error = '; '.join(blockers)
            self._last_candidates = []
            self._selected = None
            return
        expected_stamp = self._expected_road_stamp()
        if self._last_processed_stamp == expected_stamp:
            return
        try:
            raw_corridor = self._road_status.get('corridor', {}).get('primary', [])
            reference = reference_from_corridor([
                (float(sample['forward_m']), float(sample['left_m']))
                for sample in raw_corridor
            ])
            candidates = generate_candidates(
                reference,
                self._mask,
                self._profiles['primary'],
                self._geometry,
                self._config,
                self._scan_points,
            )
        except (CalibrationError, KeyError, TrajectoryError, TypeError, ValueError) as exc:
            self._last_error = str(exc)
            self._last_candidates = []
            self._selected = None
            return

        self._last_candidates = [{
            'id': candidate.candidate_id,
            'offset_m': round(candidate.offset_m, 4),
            'valid': candidate.valid,
            'cost': round(candidate.cost, 5),
            'minimum_support': round(candidate.minimum_support, 4),
            'road_blocked_steps': candidate.road_blocked,
            'obstacle_blocked_steps': candidate.obstacle_blocked,
            'command_steer_rad_diagnostic_only': round(candidate.command_steer_rad, 5),
            'reject_reason': candidate.reject_reason,
            'endpoint_m': {
                'forward': round(candidate.points[-1].x, 4),
                'left': round(candidate.points[-1].y, 4),
                'yaw': round(candidate.points[-1].yaw, 4),
            },
        } for candidate in candidates]
        selected = next((item for item in self._last_candidates if item['valid']), None)
        self._selected = selected
        self._last_error = '' if selected else 'all trajectory candidates rejected'
        self._processed_frames += 1
        self._last_processed_stamp = expected_stamp

    def _publish_status(self) -> None:
        blockers = self._blockers(time.monotonic()) if self._enabled else ['node disabled']
        payload = {
            'algorithm_stage': 4,
            'mode': 'shadow',
            'enabled': self._enabled,
            'motion_authority': False,
            'can_publish_motion': False,
            'can_select_for_control': False,
            'vehicle_geometry_validated': self._geometry_validated,
            'minimum_turn_radius_validated': self._turn_radius_validated,
            'lidar_extrinsics_validated': self._lidar_validated,
            'profile_error': self._profile_error,
            'profile': None if 'primary' not in self._profiles else profile_report(self._profiles['primary']),
            'blockers': blockers,
            'processed_frames': self._processed_frames,
            'obstacle_points': len(self._scan_points),
            'last_error': self._last_error,
            'selected_diagnostic_only': self._selected,
            'candidates': self._last_candidates,
        }
        msg = String()
        msg.data = json.dumps(payload, separators=(',', ':'), sort_keys=True)
        self._status_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TrajectoryShadow()
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
