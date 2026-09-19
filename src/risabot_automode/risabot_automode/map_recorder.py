#!/usr/bin/env python3
"""Map recorder: sample every run into ~/risabot_maps for later reuse.

Runs on every bringup while ``map_enabled`` is true (default). Records odometry
pose, lane signals, mission flags and UWB fixes (when the teammate feed
exists) at ``sample_hz`` into ``run_<UTC>.jsonl``. Also maintains nothing else:
deriving ``map_best.json`` from runs is an offline step until real data shows
what the map must contain. Recording can never affect driving — this node only
subscribes (plus its own parameter callback).
"""

import json
import math
import os
import time
from typing import Dict, List, Optional

import rclpy
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String

from .map_store import MapStore, parse_uwb_fix
from .topics import (
    HILL_SIGN_TOPIC,
    LANE_ERROR_TOPIC,
    LANE_LOST_TOPIC,
    ODOM_TOPIC,
    TRAFFIC_LIGHT_TOPIC,
    TUNNEL_DETECTED_TOPIC,
    UWB_FIX_TOPIC,
)

CURVATURE_TOPIC = '/lane_curvature'  # same literal the lane node publishes
ROAD_STATUS_TOPIC = '/v4_experimental/road/status'  # V4-owned; literal like above
V4_UWB_FIX_TOPIC = '/v4_experimental/uwb/fix'  # V4 bridge output; same schema as UWB_FIX_TOPIC
CORRIDOR_MAX_AGE = 2.0  # seconds: older corridor is recorded as []
CORRIDOR_MAX_POINTS = 200


def extract_corridor(payload) -> List[dict]:
    """Pull validated forward-corridor points from a road/status payload."""
    try:
        points = payload.get('corridor', {}).get('primary', [])
    except AttributeError:
        return []
    if not isinstance(points, list):
        return []
    out = []
    for point in points[:CORRIDOR_MAX_POINTS]:
        try:
            fwd = float(point['forward_m'])
            left = float(point['left_m'])
            width = float(point['width_m'])
        except (KeyError, TypeError, ValueError):
            continue
        if all(math.isfinite(v) for v in (fwd, left, width)) and width > 0:
            out.append({'forward_m': fwd, 'left_m': left, 'width_m': width})
    return out


def _yaw_from_quaternion(q) -> Optional[float]:
    try:
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny, cosy)
    except (AttributeError, TypeError, ValueError):
        return None
    return yaw if math.isfinite(yaw) else None


class MapRecorder(Node):
    """Append-only per-run sampler. No publishers except parameters."""

    def __init__(self):
        super().__init__('map_recorder')
        self.declare_parameter('map_enabled', True)
        self.declare_parameter('map_dir', '')
        self.declare_parameter('sample_hz', 5.0)
        self._param_cache: Dict[str, object] = {}
        self._update_param_cache()
        self.add_on_set_parameters_callback(self._on_params)

        map_dir = str(self._param_cache['map_dir']) or os.path.join(
            os.path.expanduser('~'), 'risabot_maps')
        self._store = MapStore(map_dir)
        self._run_path = self._store.begin_run(
            {'node': 'map_recorder', 'contract': 'map_store-v1'})

        self._ox = self._oy = self._oyaw = None
        self._speed = 0.0
        self._lane_error = None
        self._curvature = None
        self._lane_lost = None
        self._tl = None
        self._hill = None
        self._tunnel = None
        self._uwb = None
        self._uwb_mono = 0.0
        self._corridor: List[dict] = []
        self._corridor_mono = 0.0

        self.create_subscription(Odometry, ODOM_TOPIC, self._odom_cb, 10)
        self.create_subscription(Float32, LANE_ERROR_TOPIC, self._lane_cb, 10)
        self.create_subscription(Float32, CURVATURE_TOPIC, self._curv_cb, 10)
        self.create_subscription(Bool, LANE_LOST_TOPIC, self._lost_cb, 10)
        self.create_subscription(String, TRAFFIC_LIGHT_TOPIC, self._tl_cb, 10)
        self.create_subscription(Bool, HILL_SIGN_TOPIC, self._hill_cb, 10)
        self.create_subscription(Bool, TUNNEL_DETECTED_TOPIC, self._tunnel_cb, 10)
        self.create_subscription(String, UWB_FIX_TOPIC, self._uwb_cb, 10)
        self.create_subscription(String, V4_UWB_FIX_TOPIC, self._uwb_cb, 10)
        self.create_subscription(String, ROAD_STATUS_TOPIC, self._road_cb, 10)

        hz = max(0.5, float(self._param_cache['sample_hz']))
        self._timer = self.create_timer(1.0 / hz, self._tick)
        self.get_logger().info(f'Map recorder on, writing {self._run_path}')

    # ── Parameters ──
    def _update_param_cache(self) -> None:
        self._param_cache = {
            'map_enabled': bool(self.get_parameter('map_enabled').value),
            'map_dir': str(self.get_parameter('map_dir').value),
            'sample_hz': float(self.get_parameter('sample_hz').value),
        }

    def _on_params(self, params) -> SetParametersResult:
        for p in params:
            if p.name in self._param_cache:
                self._param_cache[p.name] = p.value
        return SetParametersResult(successful=True)

    # ── Subscribers: cache latest values only ──
    def _odom_cb(self, msg: Odometry) -> None:
        try:
            self._ox = float(msg.pose.pose.position.x)
            self._oy = float(msg.pose.pose.position.y)
            self._oyaw = _yaw_from_quaternion(msg.pose.pose.orientation)
            self._speed = float(msg.twist.twist.linear.x)
        except (AttributeError, TypeError, ValueError):
            pass

    def _lane_cb(self, msg: Float32) -> None:
        try:
            self._lane_error = float(msg.data)
        except (AttributeError, TypeError, ValueError):
            pass

    def _curv_cb(self, msg: Float32) -> None:
        try:
            self._curvature = float(msg.data)
        except (AttributeError, TypeError, ValueError):
            pass

    def _lost_cb(self, msg: Bool) -> None:
        try:
            self._lane_lost = bool(msg.data)
        except (AttributeError, TypeError, ValueError):
            pass

    def _tl_cb(self, msg: String) -> None:
        try:
            self._tl = str(msg.data)
        except (AttributeError, TypeError, ValueError):
            pass

    def _hill_cb(self, msg: Bool) -> None:
        try:
            self._hill = bool(msg.data)
        except (AttributeError, TypeError, ValueError):
            pass

    def _tunnel_cb(self, msg: Bool) -> None:
        try:
            self._tunnel = bool(msg.data)
        except (AttributeError, TypeError, ValueError):
            pass

    def _uwb_cb(self, msg: String) -> None:
        try:
            fix = parse_uwb_fix(msg.data)
        except Exception:
            return
        if fix is not None:
            self._uwb = fix
            self._uwb_mono = time.monotonic()

    def _road_cb(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except (TypeError, ValueError):
            return
        if not isinstance(payload, dict):
            return
        # Store even when empty: a fresh "looked, found nothing" beats stale.
        self._corridor = extract_corridor(payload)
        self._corridor_mono = time.monotonic()

    # ── Sampling ──
    def _tick(self) -> None:
        if not self._param_cache['map_enabled']:
            return
        now_mono = time.monotonic()
        uwb = dict(self._uwb) if self._uwb is not None else None
        if uwb is not None:
            uwb['age'] = now_mono - self._uwb_mono
        if now_mono - self._corridor_mono <= CORRIDOR_MAX_AGE:
            corridor = list(self._corridor)
        else:
            corridor = []
        self._store.record({
            't_wall': time.time(), 't_mono': now_mono,
            'ox': self._ox, 'oy': self._oy, 'oyaw': self._oyaw,
            'speed': self._speed,
            'lane_error': self._lane_error, 'curvature': self._curvature,
            'lane_lost': self._lane_lost, 'tl': self._tl,
            'hill': self._hill, 'tunnel': self._tunnel,
            'uwb': uwb, 'corridor': corridor,
        })

    def close(self) -> None:
        """Flush and close the current run file."""
        self._store.close()


def main(args=None) -> None:
    """Entry point: spin the recorder until shutdown, then close the run file."""
    rclpy.init(args=args)
    node = MapRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.close()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
