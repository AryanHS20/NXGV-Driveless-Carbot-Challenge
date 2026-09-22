"""Disabled-by-default Stage 5 parking-goal measurement source."""

import json
import time

from cv_bridge import CvBridge, CvBridgeError
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

from .bev_core import CalibrationError, load_profiles
from .parking_core import ParkingError
from .parking_goal_core import ParkingMarkingConfig, detect_parking_goal


class ParkingGoalSource(Node):
    def __init__(self) -> None:
        super().__init__('v4_parking_goal_source')
        self.declare_parameter('enabled', False)
        self.declare_parameter('profile_path', '')
        self.declare_parameter('image_topic', '/v4_experimental/bev/secondary/image')
        self.declare_parameter('coverage_topic', '/v4_experimental/bev/secondary/coverage')
        self.declare_parameter('goal_topic', '/v4_experimental/parking/goal')
        self.declare_parameter('marking_thresholds_validated', False)
        self.declare_parameter('slot_geometry_validated', False)
        self.declare_parameter('rear_coverage_validated', False)
        self.declare_parameter('sync_tolerance_sec', 0.10)
        self.declare_parameter('parking_kind', 'parallel')
        self.declare_parameter('preferred_yaw_rad', 0.0)
        self.declare_parameter('value_min', 180)
        self.declare_parameter('saturation_max', 90)
        self.declare_parameter('morph_close_px', 9)
        self._enabled = bool(self.get_parameter('enabled').value)
        self._gates = {name: bool(self.get_parameter(name).value) for name in (
            'marking_thresholds_validated', 'slot_geometry_validated',
            'rear_coverage_validated',
        )}
        self._sync = float(self.get_parameter('sync_tolerance_sec').value)
        self._kind = str(self.get_parameter('parking_kind').value)
        self._preferred_yaw = float(self.get_parameter('preferred_yaw_rad').value)
        self._config = ParkingMarkingConfig(
            value_min=int(self.get_parameter('value_min').value),
            saturation_max=int(self.get_parameter('saturation_max').value),
            morph_close_px=int(self.get_parameter('morph_close_px').value),
        )
        self._config.validate()
        self._profiles, self._profile_error = {}, ''
        try:
            self._profiles = load_profiles(str(self.get_parameter('profile_path').value))
        except (CalibrationError, OSError, ValueError) as exc:
            self._profile_error = str(exc)
        self._bridge = CvBridge()
        self._coverage = None
        self._coverage_stamp = None
        self._frames = 0
        self._last_error = ''
        self._last_goal = None
        self._goal_pub = self.create_publisher(String, str(self.get_parameter('goal_topic').value), 2)
        self._status_pub = self.create_publisher(String, '/v4_experimental/parking/goal_source_status', 10)
        self.create_subscription(Image, str(self.get_parameter('coverage_topic').value), self._coverage_cb, qos_profile_sensor_data)
        self.create_subscription(Image, str(self.get_parameter('image_topic').value), self._image_cb, qos_profile_sensor_data)
        self.create_timer(0.5, self._publish_status)

    @staticmethod
    def _stamp(msg: Image) -> float:
        return float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9

    def _coverage_cb(self, msg: Image) -> None:
        if not self._enabled:
            return
        try:
            self._coverage = self._bridge.imgmsg_to_cv2(msg, desired_encoding='mono8').copy()
            self._coverage_stamp = self._stamp(msg)
        except CvBridgeError as exc:
            self._last_error = str(exc)

    def _blockers(self):
        blockers = []
        profile = self._profiles.get('secondary')
        if profile is None or not profile.calibrated:
            blockers.append('secondary camera profile is uncalibrated')
        labels = {'marking_thresholds_validated': 'parking marking thresholds are not validated',
                  'slot_geometry_validated': 'parking slot geometry is not validated',
                  'rear_coverage_validated': 'rear coverage is not validated'}
        blockers.extend(label for gate, label in labels.items() if not self._gates[gate])
        if self._coverage is None:
            blockers.append('no rear coverage image')
        return blockers

    def _image_cb(self, msg: Image) -> None:
        if not self._enabled:
            return
        blockers = self._blockers()
        stamp = self._stamp(msg)
        if self._coverage_stamp is None or abs(stamp - self._coverage_stamp) > self._sync:
            blockers.append('parking image and coverage timestamps do not match')
        if blockers:
            self._last_error = '; '.join(blockers)
            return
        try:
            image = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            goal = detect_parking_goal(image, self._coverage, self._profiles['secondary'],
                                       stamp, self._kind, self._preferred_yaw, self._config)
            self._frames += 1
            if goal is None:
                self._last_error = 'no measured parking bay'
                self._last_goal = None
                return
            payload = {'kind': goal.kind, 'frame_id': goal.frame_id,
                       'x_m': round(goal.target.x, 4), 'y_m': round(goal.target.y, 4),
                       'yaw_rad': round(goal.target.yaw, 5),
                       'slot_length_m': round(goal.slot_length_m, 4),
                       'slot_width_m': round(goal.slot_width_m, 4),
                       'confidence': round(goal.confidence, 4),
                       'image_stamp_sec': goal.image_stamp_sec}
            self._last_goal, self._last_error = payload, ''
            self._goal_pub.publish(String(data=json.dumps(payload, separators=(',', ':'), sort_keys=True)))
        except (CalibrationError, CvBridgeError, ParkingError, ValueError) as exc:
            self._last_error = str(exc)

    def _publish_status(self) -> None:
        payload = {'algorithm_stage': 5, 'source': 'parking_markings',
                   'enabled': self._enabled, 'motion_authority': False,
                   'can_publish_motion': False, 'validation_gates': self._gates,
                   'blockers': ['node disabled'] if not self._enabled else self._blockers(),
                   'processed_frames': self._frames, 'last_goal': self._last_goal,
                   'last_error': self._last_error, 'profile_error': self._profile_error}
        self._status_pub.publish(String(data=json.dumps(payload, separators=(',', ':'), sort_keys=True)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ParkingGoalSource()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
