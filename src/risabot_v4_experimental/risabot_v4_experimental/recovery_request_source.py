"""Disabled-by-default policy source for Stage 6 recovery requests."""

import json
import math
import time

from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from .recovery_policy_core import build_recovery_request


class RecoveryRequestSource(Node):
    def __init__(self) -> None:
        super().__init__('v4_recovery_request_source')
        self.declare_parameter('enabled', False)
        self.declare_parameter('policy_source_validated', False)
        self.declare_parameter('stopped_detection_validated', False)
        self.declare_parameter('attempt_counter_validated', False)
        self.declare_parameter('trajectory_topic', '/v4_experimental/trajectory/status')
        self.declare_parameter('road_topic', '/v4_experimental/road/status')
        self.declare_parameter('dashboard_topic', '/dashboard_state')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('request_topic', '/v4_experimental/recovery/request')
        self.declare_parameter('input_timeout_sec', 0.35)
        self._enabled = bool(self.get_parameter('enabled').value)
        self._gates = {name: bool(self.get_parameter(name).value) for name in (
            'policy_source_validated', 'stopped_detection_validated',
            'attempt_counter_validated',
        )}
        self._timeout = float(self.get_parameter('input_timeout_sec').value)
        self._trajectory = self._road = None
        self._state = ''
        self._speed = 0.0
        self._seen = {'trajectory': 0.0, 'road': 0.0, 'state': 0.0, 'odom': 0.0}
        self._attempts = 0
        self._exhausted_episode = False
        self._published = 0
        self._last_request = None
        self._last_error = ''
        self._pub = self.create_publisher(String, str(self.get_parameter('request_topic').value), 10)
        self._status_pub = self.create_publisher(String, '/v4_experimental/recovery/request_source_status', 10)
        self.create_subscription(String, str(self.get_parameter('trajectory_topic').value), self._trajectory_cb, 10)
        self.create_subscription(String, str(self.get_parameter('road_topic').value), self._road_cb, 10)
        self.create_subscription(String, str(self.get_parameter('dashboard_topic').value), self._state_cb, 10)
        self.create_subscription(Odometry, str(self.get_parameter('odom_topic').value), self._odom_cb, 10)
        self.create_timer(0.1, self._evaluate)
        self.create_timer(0.5, self._publish_status)

    def _json(self, msg, name):
        try:
            value = json.loads(msg.data)
            if not isinstance(value, dict):
                raise ValueError(f'{name} must be an object')
            return value
        except (json.JSONDecodeError, TypeError, ValueError) as exc:
            self._last_error = str(exc)
            return None

    def _trajectory_cb(self, msg):
        value = self._json(msg, 'trajectory status')
        if value is not None:
            self._trajectory, self._seen['trajectory'] = value, time.monotonic()

    def _road_cb(self, msg):
        value = self._json(msg, 'road status')
        if value is not None:
            self._road, self._seen['road'] = value, time.monotonic()

    def _state_cb(self, msg):
        self._state, self._seen['state'] = str(msg.data), time.monotonic()

    def _odom_cb(self, msg):
        speed = float(msg.twist.twist.linear.x)
        if math.isfinite(speed):
            self._speed, self._seen['odom'] = speed, time.monotonic()

    def _blockers(self):
        now = time.monotonic()
        labels = {'policy_source_validated': 'recovery mission policy is not validated',
                  'stopped_detection_validated': 'stopped detection is not validated',
                  'attempt_counter_validated': 'attempt counter is not validated'}
        blockers = [label for gate, label in labels.items() if not self._gates[gate]]
        blockers.extend(f'{name} input is stale' for name, stamp in self._seen.items()
                        if not stamp or now - stamp > self._timeout)
        return blockers

    def _image_stamp(self):
        if not isinstance(self._road, dict):
            return None
        stamps = self._road.get('last_image_stamp_sec')
        if not isinstance(stamps, dict):
            return None
        try:
            value = float(stamps.get('secondary'))
            return value if math.isfinite(value) and value >= 0.0 else None
        except (TypeError, ValueError):
            return None

    def _evaluate(self):
        if not self._enabled:
            return
        blockers = self._blockers()
        stamp = self._image_stamp()
        if stamp is None:
            blockers.append('secondary image timestamp unavailable')
        if blockers or self._trajectory is None:
            self._last_error = '; '.join(blockers)
            return
        request = build_recovery_request(self._trajectory, self._state, self._speed,
                                         stamp, self._attempts)
        exhausted = request is not None
        if not exhausted and self._exhausted_episode:
            self._attempts += 1
        self._exhausted_episode = exhausted
        if request is None:
            self._last_request, self._last_error = None, ''
            return
        payload = vars(request)
        self._last_request, self._last_error = payload, ''
        self._pub.publish(String(data=json.dumps(payload, separators=(',', ':'), sort_keys=True)))
        self._published += 1

    def _publish_status(self):
        payload = {'algorithm_stage': 6, 'source': 'mission_policy',
                   'enabled': self._enabled, 'motion_authority': False,
                   'can_publish_motion': False, 'validation_gates': self._gates,
                   'blockers': ['node disabled'] if not self._enabled else self._blockers(),
                   'attempts': self._attempts, 'requests_published': self._published,
                   'last_request': self._last_request, 'last_error': self._last_error}
        self._status_pub.publish(String(data=json.dumps(payload, separators=(',', ':'), sort_keys=True)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RecoveryRequestSource()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
