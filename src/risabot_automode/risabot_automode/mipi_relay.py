#!/usr/bin/env python3
"""MIPI relay: republish a MIPI camera as /camera/color/image_raw.

The mipi_cam driver nodes must run as root (VSE driver requirement) and live
outside bringup in the risabot-cams systemd service. This node runs as a
normal user inside bringup and bridges the selected MIPI topic onto the
stack-standard color topic consumed by lane/signage/dashboard/safety/health.

Subscribes with SENSOR_DATA QoS (connects to any publisher QoS) and
republishes depth-10. Messages are forwarded untouched (no decode cost).
An optional second-camera passthrough feeds visualization/recording.
"""

import time
from typing import Dict

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import Image

from .topics import (
    CAMERA_IMAGE_TOPIC,
    MIPI_IMX219_TOPIC,
    MIPI_OV5647_TOPIC,
    MIPI_SECONDARY_TOPIC,
)


def should_publish(last_mono: float, now_mono: float, max_hz: float) -> bool:
    """Pure throttle predicate: True when a frame may be forwarded."""
    if max_hz <= 0:
        return True
    return (now_mono - last_mono) >= (1.0 / max_hz) - 1e-9


class MipiRelay(Node):
    """Forward MIPI image topics onto stack-standard topics."""

    def __init__(self):
        super().__init__('mipi_relay')
        self.declare_parameter('source_topic', MIPI_IMX219_TOPIC)
        self.declare_parameter('target_topic', CAMERA_IMAGE_TOPIC)
        self.declare_parameter('second_enabled', True)
        self.declare_parameter('second_source', MIPI_OV5647_TOPIC)
        self.declare_parameter('second_target', MIPI_SECONDARY_TOPIC)
        self.declare_parameter('max_hz', 30.0)
        self._param_cache: Dict[str, object] = {}
        self._update_param_cache()
        self.add_on_set_parameters_callback(self._on_params)

        qos = QoSPresetProfiles.SENSOR_DATA.value
        self._fwd_pub = self.create_publisher(Image, str(self._param_cache['target_topic']), 10)
        self._fwd_last = 0.0
        self._fwd_count = 0
        self.create_subscription(
            Image, str(self._param_cache['source_topic']), self._fwd_cb, qos)

        self._second_pub = None
        self._second_last = 0.0
        self._second_count = 0
        if self._param_cache['second_enabled']:
            self._second_pub = self.create_publisher(
                Image, str(self._param_cache['second_target']), 10)
            self.create_subscription(
                Image, str(self._param_cache['second_source']), self._second_cb, qos)

        self._stat_mono = time.monotonic()
        self.get_logger().info(
            'MIPI relay on: %s -> %s' % (self._param_cache['source_topic'],
                                         self._param_cache['target_topic']))

    # ── Parameters ──
    def _update_param_cache(self) -> None:
        self._param_cache = {
            'source_topic': str(self.get_parameter('source_topic').value),
            'target_topic': str(self.get_parameter('target_topic').value),
            'second_enabled': bool(self.get_parameter('second_enabled').value),
            'second_source': str(self.get_parameter('second_source').value),
            'second_target': str(self.get_parameter('second_target').value),
            'max_hz': float(self.get_parameter('max_hz').value),
        }

    def _on_params(self, params) -> SetParametersResult:
        for p in params:
            if p.name in self._param_cache:
                self._param_cache[p.name] = p.value
        return SetParametersResult(successful=True)

    # ── Forwarding ──
    def _fwd_cb(self, msg: Image) -> None:
        now = time.monotonic()
        if should_publish(self._fwd_last, now, float(self._param_cache['max_hz'])):
            self._fwd_last = now
            self._fwd_pub.publish(msg)
            self._fwd_count += 1
            self._maybe_log(now, 'forward', self._fwd_count)

    def _second_cb(self, msg: Image) -> None:
        if self._second_pub is None:
            return
        now = time.monotonic()
        if should_publish(self._second_last, now, float(self._param_cache['max_hz'])):
            self._second_last = now
            self._second_pub.publish(msg)
            self._second_count += 1
            self._maybe_log(now, 'second', self._second_count)

    def _maybe_log(self, now: float, name: str, count: int) -> None:
        if now - self._stat_mono >= 5.0:
            self._stat_mono = now
            self.get_logger().info(f'MIPI relay {name}: {count} frames forwarded')


def main(args=None) -> None:
    """Entry point: spin the relay until shutdown."""
    rclpy.init(args=args)
    node = MipiRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
