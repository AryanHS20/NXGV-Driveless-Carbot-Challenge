"""Stage 3 diagnostic local/coarse pose estimator with no control outputs."""

import json
import math
import time
from copy import deepcopy
from typing import Optional

from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String

from .pose_estimator_core import (
    EstimatorConfig,
    Pose2D,
    PoseEstimatorError,
    SeparatedPoseEstimator,
)


STATUS_TOPIC = '/v4_experimental/pose/status'
LOCAL_TOPIC = '/v4_experimental/pose/local'
COARSE_TOPIC = '/v4_experimental/pose/coarse'


def _yaw_from_odometry(msg: Odometry) -> float:
    q = msg.pose.pose.orientation
    values = (q.x, q.y, q.z, q.w)
    if not all(math.isfinite(float(value)) for value in values):
        raise PoseEstimatorError('odometry orientation must be finite')
    siny = 2.0 * (float(q.w) * float(q.z) + float(q.x) * float(q.y))
    cosy = 1.0 - 2.0 * (float(q.y) ** 2 + float(q.z) ** 2)
    return math.atan2(siny, cosy)


class PoseShadow(Node):
    def __init__(self) -> None:
        super().__init__('v4_pose_shadow')
        self.declare_parameter('enabled', False)
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('uwb_topic', '/uwb_fix')
        self.declare_parameter('odom_timeout_sec', 0.25)
        self.declare_parameter('uwb_timeout_sec', 1.0)
        self.declare_parameter('uwb_frame_alignment_validated', False)
        self.declare_parameter('uwb_frame_yaw_rad', 0.0)
        self.declare_parameter('initial_offset_sigma_m', 0.75)
        self.declare_parameter('process_sigma_m_per_s', 0.015)
        self.declare_parameter('process_sigma_m_per_m', 0.04)
        self.declare_parameter('default_uwb_sigma_m', 0.20)
        self.declare_parameter('minimum_uwb_sigma_m', 0.03)
        self.declare_parameter('maximum_uwb_sigma_m', 2.0)
        self.declare_parameter('innovation_gate', 13.82)
        self.declare_parameter('odom_reset_jump_m', 0.50)
        self.declare_parameter('odom_reset_yaw_rad', 1.0)

        self._enabled = bool(self.get_parameter('enabled').value)
        self._odom_timeout = float(self.get_parameter('odom_timeout_sec').value)
        self._uwb_timeout = float(self.get_parameter('uwb_timeout_sec').value)
        self._uwb_alignment_validated = bool(
            self.get_parameter('uwb_frame_alignment_validated').value
        )
        self._estimator = SeparatedPoseEstimator(EstimatorConfig(
            frame_yaw_rad=float(self.get_parameter('uwb_frame_yaw_rad').value),
            initial_offset_sigma_m=float(self.get_parameter('initial_offset_sigma_m').value),
            process_sigma_m_per_s=float(self.get_parameter('process_sigma_m_per_s').value),
            process_sigma_m_per_m=float(self.get_parameter('process_sigma_m_per_m').value),
            default_uwb_sigma_m=float(self.get_parameter('default_uwb_sigma_m').value),
            minimum_uwb_sigma_m=float(self.get_parameter('minimum_uwb_sigma_m').value),
            maximum_uwb_sigma_m=float(self.get_parameter('maximum_uwb_sigma_m').value),
            innovation_gate=float(self.get_parameter('innovation_gate').value),
            odom_reset_jump_m=float(self.get_parameter('odom_reset_jump_m').value),
            odom_reset_yaw_rad=float(self.get_parameter('odom_reset_yaw_rad').value),
        ))
        self._last_odom_mono: Optional[float] = None
        self._last_uwb_mono: Optional[float] = None
        self._last_error = ''
        self._uwb_invalid = 0
        self._last_odom: Optional[Odometry] = None

        self._status_pub = self.create_publisher(String, STATUS_TOPIC, 10)
        self._local_pub = self.create_publisher(Odometry, LOCAL_TOPIC, 2)
        self._coarse_pub = self.create_publisher(Odometry, COARSE_TOPIC, 2)
        self.create_subscription(
            Odometry,
            str(self.get_parameter('odom_topic').value),
            self._odom_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            String,
            str(self.get_parameter('uwb_topic').value),
            self._uwb_callback,
            qos_profile_sensor_data,
        )
        self.create_timer(0.2, self._publish_status)
        state = 'enabled' if self._enabled else 'disabled'
        self.get_logger().warning(
            f'V4 pose shadow is {state}; UWB cannot modify local pose; '
            'motion authority is permanently false'
        )

    def _odom_callback(self, msg: Odometry) -> None:
        if not self._enabled:
            return
        position = msg.pose.pose.position
        try:
            pose = Pose2D(
                float(position.x), float(position.y), _yaw_from_odometry(msg)
            )
            now = time.monotonic()
            if not self._estimator.update_local(pose, now):
                return
        except (PoseEstimatorError, TypeError, ValueError) as exc:
            self._last_error = str(exc)
            return
        self._last_odom_mono = now
        self._last_odom = msg
        self._last_error = ''
        self._publish_pose_messages()

    def _uwb_callback(self, msg: String) -> None:
        if not self._enabled:
            return
        if not self._uwb_alignment_validated:
            self._last_error = 'UWB frame alignment is not measured and validated'
            return
        try:
            payload = json.loads(msg.data)
            if not isinstance(payload, dict) or not bool(payload.get('valid', False)):
                self._uwb_invalid += 1
                return
            x = float(payload['x'])
            y = float(payload['y'])
            age = float(payload.get('age', 0.0))
            if not math.isfinite(age) or age < 0.0 or age > self._uwb_timeout:
                raise ValueError('UWB payload age is invalid or stale')
            sigma = payload.get('sigma_m')
            if sigma is None and payload.get('variance_m2') is not None:
                variance = float(payload['variance_m2'])
                sigma = math.sqrt(variance) if variance >= 0.0 else float('nan')
            now = time.monotonic()
            result = self._estimator.update_uwb(x, y, now, sigma)
            self._last_uwb_mono = now
            if not bool(result['accepted']):
                self._last_error = (
                    'UWB rejected: repeated timestamp'
                    if result['reason_code'] == 1.0
                    else 'UWB rejected by innovation gate'
                )
            else:
                self._last_error = ''
                self._publish_pose_messages()
        except (json.JSONDecodeError, KeyError, PoseEstimatorError, TypeError, ValueError) as exc:
            self._uwb_invalid += 1
            self._last_error = f'invalid UWB payload: {exc}'

    def _publish_pose_messages(self) -> None:
        if self._last_odom is None or self._estimator.local_pose is None:
            return
        local = Odometry()
        local.header = self._last_odom.header
        local.header.frame_id = 'v4_local_odom'
        local.child_frame_id = 'v4_local_base'
        local.pose = deepcopy(self._last_odom.pose)
        local.twist = deepcopy(self._last_odom.twist)
        self._local_pub.publish(local)

        coarse_pose = self._estimator.coarse_pose()
        coarse = Odometry()
        coarse.header = self._last_odom.header
        coarse.header.frame_id = 'v4_coarse_global'
        coarse.child_frame_id = 'v4_coarse_base'
        coarse.pose = deepcopy(self._last_odom.pose)
        coarse.pose.pose.position.x = coarse_pose.x
        coarse.pose.pose.position.y = coarse_pose.y
        variance = self._estimator.offset_variance
        coarse.pose.covariance[0] = max(0.0, coarse.pose.covariance[0]) + variance
        coarse.pose.covariance[7] = max(0.0, coarse.pose.covariance[7]) + variance
        coarse.twist = deepcopy(self._last_odom.twist)
        self._coarse_pub.publish(coarse)

    def _publish_status(self) -> None:
        now = time.monotonic()
        odom_age = None if self._last_odom_mono is None else now - self._last_odom_mono
        uwb_age = None if self._last_uwb_mono is None else now - self._last_uwb_mono
        payload = {
            'algorithm_stage': 3,
            'mode': 'shadow',
            'enabled': self._enabled,
            'motion_authority': False,
            'can_publish_motion': False,
            'local_pose_source': 'odometry_only',
            'uwb_effect': 'coarse_global_rigid_transform_only',
            'uwb_frame_alignment_validated': self._uwb_alignment_validated,
            'coarse_global_ready': bool(
                self._uwb_alignment_validated
                and uwb_age is not None
                and uwb_age <= self._uwb_timeout
            ),
            'ready': bool(
                self._enabled and odom_age is not None and odom_age <= self._odom_timeout
            ),
            'odom_age_sec': None if odom_age is None else round(odom_age, 3),
            'uwb_age_sec': None if uwb_age is None else round(uwb_age, 3),
            'uwb_fresh': bool(uwb_age is not None and uwb_age <= self._uwb_timeout),
            'invalid_uwb': self._uwb_invalid,
            'last_error': self._last_error,
            'estimate': self._estimator.report(),
        }
        status = String()
        status.data = json.dumps(payload, separators=(',', ':'), sort_keys=True)
        self._status_pub.publish(status)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PoseShadow()
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
