#!/usr/bin/env python3
"""V4 telemetry bridge: one compact 5 Hz display payload, read-only.

Subscribes the V4 shadow topics (local/coarse pose, road corridor,
trajectory selection, LiDAR scan) and publishes a single small JSON
document on /v4_telemetry for the dashboard track map. No command
publishers, no services, no parameters that move anything: display only.
"""

import json
import math
import time
from typing import Dict, List, Optional

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

TELEMETRY_TOPIC = '/v4_telemetry'
LOCAL_POSE_TOPIC = '/v4_experimental/pose/local'
COARSE_POSE_TOPIC = '/v4_experimental/pose/coarse'
ROAD_STATUS_TOPIC = '/v4_experimental/road/status'
TRAJECTORY_STATUS_TOPIC = '/v4_experimental/trajectory/status'
SCAN_TOPIC = '/scan'

MAX_CORRIDOR_POINTS = 60
MAX_OBSTACLE_POINTS = 72
MAX_SCAN_RANGE_M = 8.0


def yaw_from_quaternion(q) -> Optional[float]:
    """Planar yaw from a quaternion. Returns None when unusable."""
    try:
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny, cosy)
    except (AttributeError, TypeError, ValueError):
        return None
    return yaw if math.isfinite(yaw) else None


def odom_pose(msg: Odometry):
    """Extract {x, y, yaw} from an Odometry message, or None."""
    try:
        position = msg.pose.pose.position
        x, y = float(position.x), float(position.y)
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)
    except (AttributeError, TypeError, ValueError):
        return None
    if yaw is None or not all(math.isfinite(v) for v in (x, y)):
        return None
    return {'x': x, 'y': y, 'yaw': yaw}


def odom_sigma(msg: Odometry) -> Optional[float]:
    """Position sigma from pose covariance[0], or None."""
    try:
        variance = float(msg.pose.covariance[0])
    except (AttributeError, TypeError, ValueError, IndexError):
        return None
    if not math.isfinite(variance) or variance < 0.0:
        return None
    return math.sqrt(variance)


def corridor_points(payload) -> List[dict]:
    """Validated forward-corridor points, capped. Never raises."""
    try:
        raw = payload.get('corridor', {}).get('primary', [])
    except AttributeError:
        return []
    if not isinstance(raw, list):
        return []
    out = []
    for point in raw[:MAX_CORRIDOR_POINTS]:
        try:
            fwd = float(point['forward_m'])
            left = float(point['left_m'])
            width = float(point['width_m'])
        except (KeyError, TypeError, ValueError):
            continue
        if all(math.isfinite(v) for v in (fwd, left, width)) and width > 0:
            out.append({'forward_m': fwd, 'left_m': left, 'width_m': width})
    return out


def selected_trajectory(payload):
    """Selected trajectory summary (endpoint only), or None. Never raises."""
    try:
        selected = payload.get('selected_diagnostic_only')
    except AttributeError:
        return None
    if not isinstance(selected, dict) or selected.get('valid') is not True:
        return None
    try:
        endpoint = selected['endpoint_m']
        fwd = float(endpoint['forward'])
        left = float(endpoint['left'])
    except (KeyError, TypeError, ValueError):
        return None
    if not all(math.isfinite(v) for v in (fwd, left)):
        return None
    return {'id': selected.get('id'), 'forward_m': fwd, 'left_m': left}


def scan_points(msg: LaserScan) -> List[list]:
    """Downsampled obstacle points [[x, y]] in metres, capped. Never raises."""
    try:
        ranges = list(msg.ranges)
        angle = float(msg.angle_min)
        step = float(msg.angle_increment)
    except (AttributeError, TypeError, ValueError):
        return []
    if not math.isfinite(angle) or not math.isfinite(step) or step <= 0.0:
        return []
    stride = max(1, len(ranges) // MAX_OBSTACLE_POINTS + 1)
    out = []
    for index in range(0, len(ranges), stride):
        try:
            dist = float(ranges[index])
        except (TypeError, ValueError):
            continue
        if not math.isfinite(dist) or dist < 0.05 or dist > MAX_SCAN_RANGE_M:
            continue
        bearing = angle + index * step
        out.append([round(dist * math.cos(bearing), 3),
                    round(dist * math.sin(bearing), 3)])
        if len(out) >= MAX_OBSTACLE_POINTS:
            break
    return out


def build_payload(latest: dict, now_mono: float, timeout_sec: float) -> dict:
    """Assemble the telemetry document from cached inputs. Pure function."""
    doc = {'t_wall': time.time(), 'motion_authority': False, 'stale': []}

    def aged(key: str):
        entry = latest.get(key)
        if not entry:
            doc['stale'].append(key)
            return None, None
        age = now_mono - entry[1]
        if age < 0.0 or age > timeout_sec:
            doc['stale'].append(key)
            return None, round(age, 3)
        return entry[0], round(age, 3)

    local, local_age = aged('local')
    coarse, coarse_age = aged('coarse')
    corridor, corridor_age = aged('corridor')
    trajectory, trajectory_age = aged('trajectory')
    obstacles, obstacles_age = aged('obstacles')
    doc['local'] = local
    doc['coarse'] = coarse
    doc['corridor'] = corridor if corridor is not None else []
    doc['trajectory'] = trajectory
    doc['obstacles'] = obstacles if obstacles is not None else []
    doc['ages'] = {'local': local_age, 'coarse': coarse_age,
                   'corridor': corridor_age, 'trajectory': trajectory_age,
                   'obstacles': obstacles_age}
    return doc


class V4TelemetryBridge(Node):
    """Cache V4 shadow inputs; publish one compact document at 5 Hz."""

    def __init__(self):
        super().__init__('v4_telemetry_bridge')
        self.declare_parameter('publish_hz', 5.0)
        self.declare_parameter('input_timeout_sec', 1.0)
        self._param_cache: Dict[str, float] = {}
        self._update_param_cache()

        self._latest: Dict[str, tuple] = {}
        self._pub = self.create_publisher(String, TELEMETRY_TOPIC, 10)
        sensor_qos = QoSPresetProfiles.SENSOR_DATA.value
        self.create_subscription(Odometry, LOCAL_POSE_TOPIC,
                                 lambda msg: self._store('local', self._sigma_of(msg)), 10)
        self.create_subscription(Odometry, COARSE_POSE_TOPIC,
                                 lambda msg: self._store('coarse', self._sigma_of(msg)), 10)
        self.create_subscription(String, ROAD_STATUS_TOPIC, self._road_cb, 10)
        self.create_subscription(String, TRAJECTORY_STATUS_TOPIC, self._traj_cb, 10)
        self.create_subscription(LaserScan, SCAN_TOPIC, self._scan_cb, sensor_qos)

        hz = max(0.5, float(self._param_cache['publish_hz']))
        self.create_timer(1.0 / hz, self._tick)
        self.get_logger().info('V4 telemetry bridge on (read-only, 5 Hz)')

    def _update_param_cache(self) -> None:
        self._param_cache = {
            'publish_hz': float(self.get_parameter('publish_hz').value),
            'input_timeout_sec': float(self.get_parameter('input_timeout_sec').value),
        }

    @staticmethod
    def _sigma_of(msg: Odometry):
        pose = odom_pose(msg)
        if pose is None:
            return None
        pose = dict(pose)
        sigma = odom_sigma(msg)
        if sigma is not None:
            pose['sigma_m'] = sigma
        return pose

    def _store(self, key: str, value) -> None:
        if value is not None:
            self._latest[key] = (value, time.monotonic())

    def _road_cb(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except (TypeError, ValueError):
            return
        if isinstance(payload, dict):
            self._latest['corridor'] = (corridor_points(payload), time.monotonic())

    def _traj_cb(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except (TypeError, ValueError):
            return
        if isinstance(payload, dict):
            selected = selected_trajectory(payload)
            if selected is not None:
                self._latest['trajectory'] = (selected, time.monotonic())

    def _scan_cb(self, msg: LaserScan) -> None:
        self._latest['obstacles'] = (scan_points(msg), time.monotonic())

    def _tick(self) -> None:
        doc = build_payload(self._latest, time.monotonic(),
                            float(self._param_cache['input_timeout_sec']))
        message = String()
        message.data = json.dumps(doc, separators=(',', ':'), sort_keys=True)
        self._pub.publish(message)


def main(args=None) -> None:
    """Entry point: spin the bridge until shutdown."""
    rclpy.init(args=args)
    node = V4TelemetryBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
