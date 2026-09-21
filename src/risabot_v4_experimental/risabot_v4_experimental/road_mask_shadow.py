"""Stage 2 diagnostic road-mask and corridor node.

The node consumes Stage 1 BEV debug images and publishes only experimental
masks and JSON observations. It has no vehicle-command publisher.
"""

import json
import math
import time
from typing import Dict, Optional, Tuple

from cv_bridge import CvBridge, CvBridgeError
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

from .bev_core import (
    CalibrationError,
    bev_to_metric,
    load_profiles,
    metric_to_bev,
    profile_report,
)
from .local_road_memory import LocalRoadMemory, Pose2D
from .road_mask_core import (
    RoadMaskConfig,
    RoadMaskError,
    extract_corridor,
    prior_center_from_mask,
    process_bev,
    timestamps_synchronized,
)


STATUS_TOPIC = '/v4_experimental/road/status'


def _stamp_seconds(msg: Image) -> float:
    return float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9


class RoadMaskShadow(Node):
    def __init__(self) -> None:
        super().__init__('v4_road_mask_shadow')
        self.declare_parameter('enabled', False)
        self.declare_parameter('profile_path', '')
        self.declare_parameter('thresholds_validated', False)
        self.declare_parameter('process_secondary', True)
        self.declare_parameter('primary_bev_topic', '/v4_experimental/bev/primary/image')
        self.declare_parameter('primary_coverage_topic', '/v4_experimental/bev/primary/coverage')
        self.declare_parameter('secondary_bev_topic', '/v4_experimental/bev/secondary/image')
        self.declare_parameter('secondary_coverage_topic', '/v4_experimental/bev/secondary/coverage')
        self.declare_parameter('value_min', 0)
        self.declare_parameter('value_max', 135)
        self.declare_parameter('saturation_max', 255)
        self.declare_parameter('morph_open_px', 3)
        self.declare_parameter('morph_close_px', 7)
        self.declare_parameter('min_component_px', 100)
        self.declare_parameter('seed_radius_m', 0.10)
        self.declare_parameter('min_corridor_width_m', 0.12)
        self.declare_parameter('max_corridor_width_m', 0.80)
        self.declare_parameter('corridor_row_step_px', 8)
        self.declare_parameter('sync_tolerance_sec', 0.10)
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('odom_timeout_sec', 0.25)
        self.declare_parameter('memory_cell_size_m', 0.025)
        self.declare_parameter('memory_retention_sec', 25.0)
        self.declare_parameter('memory_planning_age_sec', 2.0)
        self.declare_parameter('memory_planning_distance_m', 0.50)
        self.declare_parameter('memory_reset_jump_m', 0.50)
        self.declare_parameter('memory_reset_yaw_rad', 1.0)

        self._enabled = bool(self.get_parameter('enabled').value)
        self._active_names = (
            ('primary', 'secondary')
            if bool(self.get_parameter('process_secondary').value)
            else ('primary',)
        )
        self._thresholds_validated = bool(
            self.get_parameter('thresholds_validated').value
        )
        self._sync_tolerance = float(self.get_parameter('sync_tolerance_sec').value)
        self._seed_radius_m = float(self.get_parameter('seed_radius_m').value)
        self._min_width_m = float(
            self.get_parameter('min_corridor_width_m').value
        )
        self._max_width_m = float(
            self.get_parameter('max_corridor_width_m').value
        )
        self._row_step = int(self.get_parameter('corridor_row_step_px').value)
        self._odom_timeout = float(self.get_parameter('odom_timeout_sec').value)
        self._memory_age = float(
            self.get_parameter('memory_planning_age_sec').value
        )
        self._memory_distance = float(
            self.get_parameter('memory_planning_distance_m').value
        )
        self._reset_jump = float(self.get_parameter('memory_reset_jump_m').value)
        self._reset_yaw = float(self.get_parameter('memory_reset_yaw_rad').value)
        self._config = RoadMaskConfig(
            value_min=int(self.get_parameter('value_min').value),
            value_max=int(self.get_parameter('value_max').value),
            saturation_max=int(self.get_parameter('saturation_max').value),
            morph_open_px=int(self.get_parameter('morph_open_px').value),
            morph_close_px=int(self.get_parameter('morph_close_px').value),
            min_component_px=int(self.get_parameter('min_component_px').value),
        )
        self._config.validate()
        self.add_on_set_parameters_callback(self._on_parameters)

        self._profiles = {}
        self._profile_error = ''
        try:
            self._profiles = load_profiles(
                str(self.get_parameter('profile_path').value)
            )
        except (CalibrationError, OSError, ValueError) as exc:
            self._profile_error = str(exc)
            self.get_logger().error(f'V4 road profiles rejected: {exc}')

        self._bridge = CvBridge()
        self._coverage: Dict[str, Optional[Tuple[np.ndarray, float]]] = {
            name: None for name in self._active_names
        }
        # Stage 1 publishes the BEV image before its same-frame coverage mask.
        # Cache the image so the coverage callback can pair them by timestamp;
        # otherwise every image is compared with the previous frame's mask.
        self._pending_images: Dict[str, Optional[Image]] = {
            name: None for name in self._active_names
        }
        self._frames = {name: 0 for name in self._active_names}
        self._last_error = {name: '' for name in self._active_names}
        self._last_corridor = {name: [] for name in self._active_names}
        self._last_frame_mono = {name: 0.0 for name in self._active_names}
        self._last_image_stamp = {name: None for name in self._active_names}
        self._memory_error = ''
        self._memory_resets = 0
        self._pose: Optional[Pose2D] = None
        self._pose_mono = 0.0
        self._distance = 0.0
        self._memory = LocalRoadMemory(
            cell_size_m=float(self.get_parameter('memory_cell_size_m').value),
            retention_sec=float(self.get_parameter('memory_retention_sec').value),
        )

        self._status_pub = self.create_publisher(String, STATUS_TOPIC, 10)
        self._candidate_pubs = {
            name: self.create_publisher(
                Image, f'/v4_experimental/road/{name}/candidate', 2
            )
            for name in self._active_names
        }
        self._connected_pubs = {
            name: self.create_publisher(
                Image, f'/v4_experimental/road/{name}/connected', 2
            )
            for name in self._active_names
        }
        self._memory_pubs = {
            name: self.create_publisher(
                Image, f'/v4_experimental/road/{name}/memory', 2
            )
            for name in self._active_names
        }
        self._fused_pubs = {
            name: self.create_publisher(
                Image, f'/v4_experimental/road/{name}/fused', 2
            )
            for name in self._active_names
        }

        for name in self._active_names:
            self.create_subscription(
                Image,
                str(self.get_parameter(f'{name}_coverage_topic').value),
                lambda msg, camera=name: self._coverage_callback(camera, msg),
                qos_profile_sensor_data,
            )
            self.create_subscription(
                Image,
                str(self.get_parameter(f'{name}_bev_topic').value),
                lambda msg, camera=name: self._image_callback(camera, msg),
                qos_profile_sensor_data,
            )
        self.create_subscription(
            Odometry,
            str(self.get_parameter('odom_topic').value),
            self._odom_callback,
            qos_profile_sensor_data,
        )
        self.create_timer(1.0, self._publish_status)
        state = 'enabled' if self._enabled else 'disabled'
        self.get_logger().warning(
            f'V4 road-mask shadow is {state}; thresholds_validated='
            f'{self._thresholds_validated}; motion authority is permanently false'
        )

    def _on_parameters(self, parameters) -> SetParametersResult:
        safe = {
            'value_min', 'value_max', 'saturation_max', 'morph_open_px',
            'morph_close_px', 'min_component_px', 'seed_radius_m',
            'min_corridor_width_m', 'max_corridor_width_m',
            'corridor_row_step_px', 'memory_planning_age_sec',
            'memory_planning_distance_m', 'memory_reset_jump_m',
            'memory_reset_yaw_rad',
        }
        restart_only = {
            'enabled', 'profile_path', 'thresholds_validated',
            'process_secondary', 'primary_bev_topic',
            'primary_coverage_topic', 'secondary_bev_topic',
            'secondary_coverage_topic', 'sync_tolerance_sec', 'odom_topic',
            'odom_timeout_sec', 'memory_cell_size_m', 'memory_retention_sec',
        }
        proposed = {
            'value_min': self._config.value_min,
            'value_max': self._config.value_max,
            'saturation_max': self._config.saturation_max,
            'morph_open_px': self._config.morph_open_px,
            'morph_close_px': self._config.morph_close_px,
            'min_component_px': self._config.min_component_px,
            'seed_radius_m': self._seed_radius_m,
            'min_corridor_width_m': self._min_width_m,
            'max_corridor_width_m': self._max_width_m,
            'corridor_row_step_px': self._row_step,
            'memory_planning_age_sec': self._memory_age,
            'memory_planning_distance_m': self._memory_distance,
            'memory_reset_jump_m': self._reset_jump,
            'memory_reset_yaw_rad': self._reset_yaw,
        }
        for parameter in parameters:
            if parameter.name in restart_only:
                return SetParametersResult(
                    successful=False,
                    reason=f'{parameter.name} requires a node restart',
                )
            if parameter.name in safe:
                proposed[parameter.name] = parameter.value
        try:
            config = RoadMaskConfig(
                value_min=int(proposed['value_min']),
                value_max=int(proposed['value_max']),
                saturation_max=int(proposed['saturation_max']),
                morph_open_px=int(proposed['morph_open_px']),
                morph_close_px=int(proposed['morph_close_px']),
                min_component_px=int(proposed['min_component_px']),
            )
            config.validate()
            seed = float(proposed['seed_radius_m'])
            min_width = float(proposed['min_corridor_width_m'])
            max_width = float(proposed['max_corridor_width_m'])
            row_step = int(proposed['corridor_row_step_px'])
            memory_age = float(proposed['memory_planning_age_sec'])
            memory_distance = float(proposed['memory_planning_distance_m'])
            reset_jump = float(proposed['memory_reset_jump_m'])
            reset_yaw = float(proposed['memory_reset_yaw_rad'])
            values = (seed, min_width, max_width, memory_age, memory_distance,
                      reset_jump, reset_yaw)
            if not all(math.isfinite(value) for value in values):
                raise ValueError('live tuning values must be finite')
            if seed <= 0.0 or min_width <= 0.0 or max_width < min_width:
                raise ValueError('seed and corridor widths are invalid')
            if row_step <= 0 or min(memory_age, memory_distance, reset_jump, reset_yaw) < 0.0:
                raise ValueError('memory limits and row step are invalid')
        except (RoadMaskError, TypeError, ValueError) as exc:
            return SetParametersResult(successful=False, reason=str(exc))

        self._config = config
        self._seed_radius_m = seed
        self._min_width_m = min_width
        self._max_width_m = max_width
        self._row_step = row_step
        self._memory_age = memory_age
        self._memory_distance = memory_distance
        self._reset_jump = reset_jump
        self._reset_yaw = reset_yaw
        return SetParametersResult(successful=True)

    def _odom_callback(self, msg: Odometry) -> None:
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        values = (
            position.x, position.y, orientation.x, orientation.y,
            orientation.z, orientation.w,
        )
        if not all(math.isfinite(float(value)) for value in values):
            self._memory_error = 'non-finite odometry ignored'
            return
        siny = 2.0 * (
            float(orientation.w) * float(orientation.z)
            + float(orientation.x) * float(orientation.y)
        )
        cosy = 1.0 - 2.0 * (
            float(orientation.y) ** 2 + float(orientation.z) ** 2
        )
        pose = Pose2D(float(position.x), float(position.y), math.atan2(siny, cosy))
        if self._pose is not None:
            step = math.hypot(pose.x - self._pose.x, pose.y - self._pose.y)
            yaw_step = math.atan2(
                math.sin(pose.yaw - self._pose.yaw),
                math.cos(pose.yaw - self._pose.yaw),
            )
            if step > self._reset_jump or abs(yaw_step) > self._reset_yaw:
                self._memory.clear()
                self._distance = 0.0
                self._memory_resets += 1
                self._memory_error = 'memory cleared after odometry discontinuity'
            else:
                self._distance += step
        self._pose = pose
        self._pose_mono = time.monotonic()

    def _coverage_callback(self, name: str, msg: Image) -> None:
        if not self._enabled:
            return
        try:
            mask = self._bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        except CvBridgeError as exc:
            self._last_error[name] = str(exc)
            return
        coverage = mask.copy()
        coverage_stamp = _stamp_seconds(msg)
        self._coverage[name] = (coverage, coverage_stamp)
        pending = self._pending_images[name]
        if pending is None:
            return
        image_stamp = _stamp_seconds(pending)
        if timestamps_synchronized(
            image_stamp, coverage_stamp, self._sync_tolerance
        ):
            self._pending_images[name] = None
            self._process_image(name, pending, coverage)

    def _seed(self, name: str) -> Tuple[float, float]:
        profile = self._profiles[name]
        forward = min(max(0.0, profile.forward_bounds_m[0]), profile.forward_bounds_m[1])
        if forward == profile.forward_bounds_m[0]:
            forward = min(forward + 0.05, profile.forward_bounds_m[1])
        point = metric_to_bev(profile, np.array([[forward, 0.0]], np.float64))[0]
        return float(point[0]), float(point[1])

    def _image_callback(self, name: str, msg: Image) -> None:
        if not self._enabled or name not in self._profiles:
            return
        profile = self._profiles[name]
        if not profile.calibrated:
            self._last_error[name] = 'profile is explicitly uncalibrated'
            return
        coverage_entry = self._coverage[name]
        if coverage_entry is None:
            self._pending_images[name] = msg
            self._last_error[name] = 'waiting for matching coverage mask'
            return
        coverage, coverage_stamp = coverage_entry
        image_stamp = _stamp_seconds(msg)
        if not timestamps_synchronized(
            image_stamp, coverage_stamp, self._sync_tolerance
        ):
            self._pending_images[name] = msg
            self._last_error[name] = 'waiting for matching coverage mask'
            return
        self._pending_images[name] = None
        self._process_image(name, msg, coverage)

    def _process_image(
        self,
        name: str,
        msg: Image,
        coverage: np.ndarray,
    ) -> None:
        """Process one timestamp-matched BEV image and coverage mask."""
        profile = self._profiles[name]
        image_stamp = _stamp_seconds(msg)
        try:
            image = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            seed = self._seed(name)
            prior_mask = None
            prior_center_x = None
            if (
                self._pose is not None
                and time.monotonic() - self._pose_mono <= self._odom_timeout
                and int(self._memory.stats().get('cells', 0)) > 0
            ):
                try:
                    rendered = self._memory.render(
                        profile,
                        self._pose,
                        time.monotonic(),
                        self._distance,
                        self._memory_age,
                        self._memory_distance,
                    )
                    if bool((rendered > 0).any()):
                        prior_mask = rendered
                        prior_center_x = prior_center_from_mask(rendered)
                except (CalibrationError, RoadMaskError, ValueError):
                    prior_mask = None
                    prior_center_x = None
            result = process_bev(
                image,
                coverage,
                self._config,
                seed_xy=seed,
                seed_radius_px=max(1, int(round(
                    self._seed_radius_m * profile.pixels_per_meter
                ))),
                pixels_per_meter=profile.pixels_per_meter,
                min_width_m=self._min_width_m,
                max_width_m=self._max_width_m,
                row_step_px=self._row_step,
                prior_mask=prior_mask,
                prior_center_x=prior_center_x,
            )
        except (CalibrationError, RoadMaskError, CvBridgeError, ValueError) as exc:
            self._last_error[name] = str(exc)
            return

        candidate_msg = self._bridge.cv2_to_imgmsg(
            result['candidate'], encoding='mono8'
        )
        connected_msg = self._bridge.cv2_to_imgmsg(
            result['connected'], encoding='mono8'
        )
        candidate_msg.header = msg.header
        connected_msg.header = msg.header
        candidate_msg.header.frame_id = f'v4_road_{name}'
        connected_msg.header.frame_id = f'v4_road_{name}'
        self._candidate_pubs[name].publish(candidate_msg)
        self._connected_pubs[name].publish(connected_msg)

        memory_mask = np.zeros_like(result['connected'])
        fused_mask = result['connected']
        samples = result['samples']
        now = time.monotonic()
        pose_fresh = (
            self._pose is not None and now - self._pose_mono <= self._odom_timeout
        )
        if not pose_fresh:
            self._memory_error = 'no fresh odometry; live mask only'
        else:
            try:
                memory_mask = self._memory.render(
                    profile,
                    self._pose,
                    now,
                    self._distance,
                    self._memory_age,
                    self._memory_distance,
                )
                fused_mask = np.maximum(result['connected'], memory_mask)
                fused_coverage = np.maximum(coverage, memory_mask)
                samples = extract_corridor(
                    fused_mask,
                    fused_coverage,
                    seed_center_x=(
                        seed[0]
                        if prior_center_x is None
                        else prior_center_x
                    ),
                    pixels_per_meter=profile.pixels_per_meter,
                    min_width_m=self._min_width_m,
                    max_width_m=self._max_width_m,
                    row_step_px=self._row_step,
                )
                self._memory.integrate(
                    result['connected'],
                    profile,
                    self._pose,
                    now,
                    self._distance,
                )
                self._memory_error = ''
            except (CalibrationError, RoadMaskError, ValueError) as exc:
                self._memory_error = str(exc)

        memory_msg = self._bridge.cv2_to_imgmsg(memory_mask, encoding='mono8')
        fused_msg = self._bridge.cv2_to_imgmsg(fused_mask, encoding='mono8')
        memory_msg.header = msg.header
        fused_msg.header = msg.header
        memory_msg.header.frame_id = f'v4_road_memory_{name}'
        fused_msg.header.frame_id = f'v4_road_fused_{name}'
        self._memory_pubs[name].publish(memory_msg)
        self._fused_pubs[name].publish(fused_msg)

        if samples:
            pixels = np.array(
                [[sample.center_px, sample.row_px] for sample in samples],
                dtype=np.float64,
            )
            metric = bev_to_metric(profile, pixels)
            self._last_corridor[name] = [
                {
                    'forward_m': round(float(point[0]), 4),
                    'left_m': round(float(point[1]), 4),
                    'width_m': round(
                        float(sample.width_px) / profile.pixels_per_meter, 4
                    ),
                }
                for sample, point in zip(samples, metric)
            ]
        else:
            self._last_corridor[name] = []
        self._frames[name] += 1
        self._last_frame_mono[name] = time.monotonic()
        self._last_image_stamp[name] = image_stamp
        self._last_error[name] = ''
        # Publish the corridor metadata for this exact mask frame. Stage 4
        # compares this timestamp with the fused Image header before planning.
        self._publish_status()

    def _publish_status(self) -> None:
        now = time.monotonic()
        payload = {
            'algorithm_stage': 2,
            'mode': 'shadow',
            'enabled': self._enabled,
            'motion_authority': False,
            'can_publish_motion': False,
            'thresholds_validated': self._thresholds_validated,
            'active_profiles': list(self._active_names),
            'profile_error': self._profile_error,
            'profiles': {
                name: profile_report(profile)
                for name, profile in self._profiles.items()
                if name in self._active_names
            },
            'frames_published': self._frames,
            'last_frame_age_sec': {
                name: None if stamp == 0.0 else round(now - stamp, 3)
                for name, stamp in self._last_frame_mono.items()
            },
            'last_image_stamp_sec': self._last_image_stamp,
            'last_error': self._last_error,
            'memory_error': self._memory_error,
            'memory_resets': self._memory_resets,
            'memory': self._memory.stats(),
            'corridor': self._last_corridor,
        }
        msg = String()
        msg.data = json.dumps(payload, separators=(',', ':'), sort_keys=True)
        self._status_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RoadMaskShadow()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
