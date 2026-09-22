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
    centerline_steering_command,
    generate_candidates,
    near_field_bootstrap_from_corridor,
    smooth_centerline_reference,
    steering_reference_from_corridor,
)


STATUS_TOPIC = '/v4_experimental/trajectory/status'


class TrajectoryShadow(Node):
    def __init__(self) -> None:
        super().__init__('v4_trajectory_shadow')
        self.declare_parameter('enabled', False)
        self.declare_parameter('track_test_mode', False)
        self.declare_parameter('enforce_road_support_in_track_test', False)
        self.declare_parameter('require_lidar', True)
        self.declare_parameter('profile_path', '')
        self.declare_parameter('road_status_topic', '/v4_experimental/road/status')
        self.declare_parameter('road_mask_topic', '/v4_experimental/road/primary/fused')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('road_timeout_sec', 1.10)
        self.declare_parameter('plan_hold_sec', 0.0)
        self.declare_parameter('scan_timeout_sec', 0.50)
        self.declare_parameter('road_mask_sync_tolerance_sec', 0.02)
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
        for name, value in (
            ('horizon_m', 0.55), ('step_m', 0.01),
            ('lookahead_m', 0.095), ('rollout_speed_mps', 0.08),
            ('steering_lag_sec', 0.12),
            ('steering_rate_rad_sec', math.pi),
            ('steering_gain', 1.0),
            ('footprint_sample_spacing_m', 0.02),
            ('minimum_road_support', 0.98),
            ('road_support_cost_weight', 40.0),
            ('expected_lane_width_m', 0.32),
            ('centerline_filter_alpha', 0.60),
            ('cross_track_gain', 1.10),
            ('heading_gain', 0.85),
            ('curvature_feedforward_gain', 0.90),
            ('reliable_support_threshold', 0.75),
            ('minimum_observed_centerline_fraction', 0.50),
            ('low_support_direction_hold_sec', 2.50),
            ('low_support_steer_decay_sec', 1.50),
            ('obstacle_margin_m', 0.015),
            ('near_field_max_gap_m', 0.55), ('near_field_settle_m', 0.04),
        ):
            self.declare_parameter(name, value)

        self._enabled = bool(self.get_parameter('enabled').value)
        self._track_test_mode = bool(self.get_parameter('track_test_mode').value)
        self._enforce_track_test_road_support = bool(
            self.get_parameter('enforce_road_support_in_track_test').value
        )
        self._require_lidar = bool(self.get_parameter('require_lidar').value)
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
        self._plan_hold = float(self.get_parameter('plan_hold_sec').value)
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
        self._last_success_mono: Optional[float] = None
        self._last_candidates: List[Dict[str, object]] = []
        self._selected: Optional[Dict[str, object]] = None
        self._last_error = ''
        self._last_warning = ''
        self._centerline_coefficients: Tuple[float, ...] = ()
        self._last_control_diagnostics: Dict[str, object] = {}
        self._last_control_steer_rad = 0.0
        self._last_control_steer_mono = 0.0
        self._reliable_steer_rad = 0.0
        self._reliable_steer_mono = 0.0
        self._imu_yaw_rad: Optional[float] = None
        self._imu_mono = 0.0
        self._hold_target_yaw_rad: Optional[float] = None
        self._hold_feedforward_rad = 0.0
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
        self.create_subscription(String, '/imu/rpy', self._imu_callback, 10)
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
            'steering_lag_sec', 'steering_rate_rad_sec', 'steering_gain',
            'footprint_sample_spacing_m', 'minimum_road_support',
            'road_support_cost_weight',
            'expected_lane_width_m',
            'centerline_filter_alpha', 'cross_track_gain', 'heading_gain',
            'curvature_feedforward_gain', 'reliable_support_threshold',
            'minimum_observed_centerline_fraction',
            'low_support_direction_hold_sec', 'low_support_steer_decay_sec',
            'obstacle_margin_m',
            'near_field_max_gap_m', 'near_field_settle_m',
        )
        values = {
            name: float(overrides.get(name, self.get_parameter(name).value))
            for name in names
        }
        return TrajectoryConfig(**values)

    def _on_parameters(self, parameters) -> SetParametersResult:
        safe = {
            'horizon_m', 'step_m', 'lookahead_m', 'rollout_speed_mps',
            'steering_lag_sec', 'steering_rate_rad_sec', 'steering_gain',
            'footprint_sample_spacing_m', 'minimum_road_support',
            'road_support_cost_weight',
            'expected_lane_width_m',
            'centerline_filter_alpha', 'cross_track_gain', 'heading_gain',
            'curvature_feedforward_gain', 'reliable_support_threshold',
            'minimum_observed_centerline_fraction',
            'low_support_direction_hold_sec', 'low_support_steer_decay_sec',
            'obstacle_margin_m',
            'near_field_max_gap_m', 'near_field_settle_m',
        }
        protected = {
            'enabled', 'track_test_mode', 'enforce_road_support_in_track_test',
            'require_lidar', 'profile_path', 'vehicle_geometry_validated',
            'minimum_turn_radius_validated', 'lidar_extrinsics_validated',
            'vehicle_length_m', 'vehicle_width_m', 'wheelbase_m',
            'rear_overhang_m', 'minimum_turn_radius_m',
            'footprint_padding_m', 'lidar_x_m', 'lidar_y_m',
            'lidar_yaw_rad', 'minimum_scan_range_m', 'maximum_scan_range_m',
            'road_status_topic', 'road_mask_topic', 'scan_topic',
            'road_timeout_sec', 'scan_timeout_sec',
            'plan_hold_sec',
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

    def _imu_callback(self, msg: String) -> None:
        try:
            yaw = math.radians(float(json.loads(msg.data)['yaw']))
        except (KeyError, TypeError, ValueError, json.JSONDecodeError):
            return
        if math.isfinite(yaw):
            self._imu_yaw_rad = yaw
            self._imu_mono = time.monotonic()

    @staticmethod
    def _angle_difference(target: float, current: float) -> float:
        return math.atan2(math.sin(target - current), math.cos(target - current))

    def _imu_plan_hold_steer(self, now: float):
        if (self._hold_target_yaw_rad is None or self._imu_yaw_rad is None
                or now - self._imu_mono > 0.35):
            return None
        heading_error = self._angle_difference(
            self._hold_target_yaw_rad, self._imu_yaw_rad
        )
        maximum_steer = math.atan(
            self._geometry.wheelbase_m / self._geometry.minimum_turn_radius_m
        )
        held = (self._hold_feedforward_rad
                + self._config.heading_gain * heading_error)
        return (
            max(-maximum_steer, min(maximum_steer, held)),
            heading_error,
        )

    def _apply_imu_plan_hold(self, now: float) -> None:
        held = self._imu_plan_hold_steer(now)
        if self._selected is None or held is None:
            return
        steer, heading_error = held
        self._selected = dict(self._selected)
        self._selected['command_steer_rad_diagnostic_only'] = round(
            steer, 5
        )
        self._selected['steering_source'] = 'imu_plan_hold'
        self._selected['imu_heading_error_rad'] = round(heading_error, 5)

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
        if not self._track_test_mode and not self._geometry_validated:
            blockers.append('vehicle geometry is not measured and validated')
        if not self._track_test_mode and not self._turn_radius_validated:
            blockers.append('minimum turn radius is not measured and validated')
        if not self._track_test_mode and not self._lidar_validated:
            blockers.append('LiDAR extrinsics are not measured and validated')
        if self._road_status is None or self._road_status_mono is None:
            blockers.append('no road status')
        elif now - self._road_status_mono > self._road_timeout:
            blockers.append('road status is stale')
        elif not self._track_test_mode and not bool(self._road_status.get('thresholds_validated', False)):
            blockers.append('road thresholds are not validated')
        if self._require_lidar and self._scan_mono is None:
            blockers.append('no LiDAR scan')
        elif self._require_lidar and now - self._scan_mono > self._scan_timeout:
            blockers.append('LiDAR scan is stale')
        if self._mask is None or self._mask_mono is None:
            blockers.append('no road mask')
        elif now - self._mask_mono > self._road_timeout:
            blockers.append('road mask is stale')
        expected_stamp = self._expected_road_stamp()
        if expected_stamp is None or self._mask_stamp is None:
            blockers.append('road mask timestamp unavailable')
        elif (
            abs(float(expected_stamp) - self._mask_stamp) > self._sync_tolerance
            and (
                self._selected is None
                or self._last_success_mono is None
                or now - self._last_success_mono > self._road_timeout
            )
        ):
            blockers.append('road mask and corridor timestamps do not match')
        return blockers

    def _inputs_synchronized(self) -> bool:
        """Return whether the current mask belongs to the current corridor."""
        expected_stamp = self._expected_road_stamp()
        return bool(
            expected_stamp is not None
            and self._mask_stamp is not None
            and abs(float(expected_stamp) - self._mask_stamp)
            <= self._sync_tolerance
        )

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
        # Road status and its mask are separate ROS messages.  One normally
        # arrives just before the other.  Keep the last recent matched plan
        # during that bounded interval instead of publishing a one-cycle stop
        # or combining data from two different camera frames.
        if not self._inputs_synchronized():
            return
        expected_stamp = self._expected_road_stamp()
        if self._last_processed_stamp == expected_stamp:
            return
        try:
            raw_corridor = self._road_status.get('corridor', {}).get('primary', [])
            try:
                near_field = near_field_bootstrap_from_corridor([
                    (float(sample['forward_m']), float(sample['left_m']), float(sample['width_m']))
                    for sample in raw_corridor
                ], self._geometry, self._config.near_field_max_gap_m,
                    self._config.near_field_settle_m)
            except TrajectoryError:
                if not self._track_test_mode:
                    raise
                near_field = None
            # The near-field bootstrap describes footprint support, not a
            # measured lane center. A fully observed row uses its midpoint;
            # a turn with one edge outside the FOV uses the remaining observed
            # edge plus the measured lane width.
            steering_rows = [sample for sample in raw_corridor
                             if (near_field is None or
                              float(sample['forward_m']) >= near_field.forward_m)]
            observed_centerline_fraction = (
                sum(
                    bool(sample.get('left_boundary_observed'))
                    or bool(sample.get('right_boundary_observed'))
                    for sample in steering_rows
                ) / len(steering_rows)
                if steering_rows else 0.0
            )
            reference = steering_reference_from_corridor(
                steering_rows, self._config.expected_lane_width_m
            )
            reference, coefficients = smooth_centerline_reference(
                reference, self._centerline_coefficients,
                self._config.centerline_filter_alpha,
            )
            control_steer, control_diagnostics = centerline_steering_command(
                reference, coefficients, self._geometry,
                self._config.lookahead_m, self._config.cross_track_gain,
                self._config.heading_gain,
                self._config.curvature_feedforward_gain,
                self._config.expected_lane_width_m,
            )
            if self._last_control_steer_mono > 0.0:
                control_dt = max(
                    0.02, min(0.50, now - self._last_control_steer_mono)
                )
                maximum_change = self._config.steering_rate_rad_sec * control_dt
                control_steer = max(
                    self._last_control_steer_rad - maximum_change,
                    min(self._last_control_steer_rad + maximum_change,
                        control_steer),
                )
            scan_fresh = (self._scan_mono is not None and
                          now - self._scan_mono <= self._scan_timeout)
            candidates = generate_candidates(
                reference,
                self._mask,
                self._profiles['primary'],
                self._geometry,
                self._config,
                self._scan_points if scan_fresh else (),
                near_field,
                enforce_road_support=(
                    not self._track_test_mode
                    or self._enforce_track_test_road_support
                ),
            )
        except (CalibrationError, KeyError, TrajectoryError, TypeError, ValueError) as exc:
            if (self._selected is not None and self._last_success_mono is not None
                    and now - self._last_success_mono <= self._plan_hold):
                self._apply_imu_plan_hold(now)
                self._last_error = ''
                self._last_warning = f'{exc}; holding recent valid plan'
                self._last_processed_stamp = expected_stamp
                return
            self._last_error = str(exc)
            self._last_warning = ''
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
        if selected is not None:
            selected = dict(selected)
            support = float(selected.get('minimum_support', 0.0))
            command = float(control_steer)
            source = 'filtered_centerline'
            reliable_age = (
                now - self._reliable_steer_mono
                if self._reliable_steer_mono > 0.0 else math.inf
            )
            imu_heading_error = None
            if (support < self._config.reliable_support_threshold
                    and math.isfinite(reliable_age)):
                if (observed_centerline_fraction >=
                        self._config.minimum_observed_centerline_fraction):
                    source = 'low_support_observed_centerline'
                    self._last_warning = (
                        'footprint support is low; following observed lane boundaries'
                    )
                elif reliable_age <= self._config.low_support_direction_hold_sec:
                    imu_hold = self._imu_plan_hold_steer(now)
                    if imu_hold is None:
                        command = self._reliable_steer_rad
                        source = 'low_support_direction_hold'
                    else:
                        command, imu_heading_error = imu_hold
                        source = 'low_support_imu_hold'
                    self._last_warning = (
                        'low road support; following the recent reliable plan'
                    )
                else:
                    decay_age = (
                        reliable_age
                        - self._config.low_support_direction_hold_sec
                    )
                    decay = max(
                        0.0,
                        1.0 - decay_age
                        / self._config.low_support_steer_decay_sec,
                    )
                    command = self._reliable_steer_rad * decay
                    source = 'low_support_steer_decay'
                    self._last_warning = (
                        'road support remains low; decaying steering toward straight'
                    )
            else:
                self._last_warning = ''
            selected['command_steer_rad_diagnostic_only'] = round(command, 5)
            selected['steering_source'] = source
            selected['observed_centerline_fraction'] = round(
                observed_centerline_fraction, 4
            )
            if imu_heading_error is not None:
                selected['imu_heading_error_rad'] = round(
                    imu_heading_error, 5
                )
            selected.update({
                name: round(float(value), 5)
                for name, value in control_diagnostics.items()
            })
            body_half_width = (
                0.5 * self._geometry.width_m
                + self._geometry.footprint_padding_m
            )
            clearance = (0.5 * self._config.expected_lane_width_m
                         - body_half_width
                         - abs(float(control_diagnostics['lateral_error_m'])))
            selected['boundary_clearance_m'] = round(clearance, 5)
            selected['path_shape'] = (
                'roundabout_like'
                if abs(float(control_diagnostics['curvature_per_m'])) >= 1.0
                else 'lane'
            )
            if support >= self._config.reliable_support_threshold:
                self._reliable_steer_rad = command
                self._reliable_steer_mono = now
                if self._imu_yaw_rad is not None and now - self._imu_mono <= 0.35:
                    self._hold_target_yaw_rad = (
                        self._imu_yaw_rad
                        + float(control_diagnostics['heading_error_rad'])
                    )
                    self._hold_feedforward_rad = (
                        self._config.curvature_feedforward_gain
                        * float(control_diagnostics['feedforward_steer_rad'])
                    )
            self._centerline_coefficients = coefficients
            self._last_control_diagnostics = dict(control_diagnostics)
            self._last_control_steer_rad = command
            self._last_control_steer_mono = now
        self._selected = selected
        self._last_error = '' if selected else 'all trajectory candidates rejected'
        if selected is None:
            self._last_warning = ''
        self._processed_frames += 1
        self._last_processed_stamp = expected_stamp
        self._last_success_mono = now

    def _publish_status(self) -> None:
        blockers = self._blockers(time.monotonic()) if self._enabled else ['node disabled']
        payload = {
            'algorithm_stage': 4,
            'mode': 'shadow',
            'enabled': self._enabled,
            'track_test_mode': self._track_test_mode,
            'road_support_enforced': (
                not self._track_test_mode
                or self._enforce_track_test_road_support
            ),
            'require_lidar': self._require_lidar,
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
            'last_warning': self._last_warning,
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
