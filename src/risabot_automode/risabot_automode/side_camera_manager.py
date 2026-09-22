#!/usr/bin/env python3
"""Root-side lifecycle manager for the two optional RDK X5 MIPI cameras.

The forward Astra is intentionally outside this manager.  Side cameras are
started only under a short renewable lease from the dashboard or while the V4
executor reports real motion authority.  A failed child is not retry-spun: a
new OFF -> side request is required before another start attempt.
"""

import json
import os
import signal
import subprocess
import time
from typing import Dict, Optional

import rclpy
try:
    from rclpy.executors import ExternalShutdownException
except ImportError:  # lightweight unit-test stubs do not expose executors
    class ExternalShutdownException(Exception):
        pass
from rclpy.node import Node
from std_msgs.msg import String

from .topics import SIDE_CAMERA_REQUEST_TOPIC, SIDE_CAMERA_STATUS_TOPIC


VALID_MODES = frozenset({'off', 'right', 'left', 'both'})


def normalize_mode(value: str) -> str:
    mode = str(value).strip().lower()
    if mode not in VALID_MODES:
        raise ValueError('side camera mode must be off, right, left, or both')
    return mode


class SideCameraManager(Node):
    def __init__(self) -> None:
        super().__init__('side_camera_manager')
        self.declare_parameter('image_width', 480)
        self.declare_parameter('image_height', 272)
        self.declare_parameter('request_timeout_sec', 8.0)
        self.declare_parameter('startup_settle_sec', 3.0)
        self.declare_parameter('driver_path', '/opt/tros/humble/lib/mipi_cam/mipi_cam')
        self._width = int(self.get_parameter('image_width').value)
        self._height = int(self.get_parameter('image_height').value)
        self._timeout = float(self.get_parameter('request_timeout_sec').value)
        self._settle = float(self.get_parameter('startup_settle_sec').value)
        self._driver = str(self.get_parameter('driver_path').value)
        if self._width <= 0 or self._height <= 0 or self._timeout <= 0 or self._settle < 0:
            raise ValueError('side camera dimensions and timing must be valid')

        self._children: Dict[str, subprocess.Popen] = {}
        self._ui_mode = 'off'
        self._ui_stamp = 0.0
        self._v4_required = False
        self._v4_stamp = 0.0
        self._mode = 'off'
        self._fault = ''
        self._fault_mode = ''
        self._status_pub = self.create_publisher(String, SIDE_CAMERA_STATUS_TOPIC, 10)
        self.create_subscription(String, SIDE_CAMERA_REQUEST_TOPIC, self._request_cb, 10)
        self.create_subscription(String, '/v4_control/status', self._v4_cb, 10)
        self.create_timer(0.5, self._tick)
        self.create_timer(1.0, self._publish_status)
        self.get_logger().info('Side cameras are OFF; waiting for a renewable request')

    def _request_cb(self, msg: String) -> None:
        try:
            mode = normalize_mode(msg.data)
        except ValueError as exc:
            self.get_logger().warning(str(exc))
            return
        self._ui_mode = mode
        self._ui_stamp = time.monotonic()
        if mode == 'off':
            self._fault_mode = ''

    def _v4_cb(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            self._v4_required = bool(payload.get('motion_authority', False))
            self._v4_stamp = time.monotonic()
        except (ValueError, TypeError):
            self._v4_required = False
            self._v4_stamp = 0.0

    def _wanted_mode(self, now: float) -> str:
        ui = self._ui_mode if now - self._ui_stamp <= self._timeout else 'off'
        v4 = self._v4_required and now - self._v4_stamp <= 1.5
        if v4 and ui == 'left':
            return 'both'
        if v4:
            return 'right'
        return ui

    def _command(self, side: str):
        if side == 'left':
            return [self._driver, '--ros-args', '-r', '__ns:=/cam_ov5647',
                    '-p', 'channel:=2', '-p', f'image_width:={self._width}',
                    '-p', f'image_height:={self._height}', '--log-level', 'warn']
        return [self._driver, '--ros-args', '-r', '__ns:=/cam_imx219',
                '-p', 'channel:=0', '-p', f'image_width:={self._width}',
                '-p', f'image_height:={self._height}', '-p', 'rotation:=180.0',
                '--log-level', 'warn']

    def _start_child(self, side: str) -> bool:
        try:
            child = subprocess.Popen(self._command(side), start_new_session=True)
            self._children[side] = child
            time.sleep(0.35)
            if child.poll() is not None:
                raise RuntimeError(f'{side} camera exited with {child.returncode}')
            return True
        except (OSError, RuntimeError) as exc:
            self._fault = str(exc)
            self.get_logger().error(self._fault)
            return False

    def _stop_all(self) -> None:
        children = list(self._children.items())
        self._children = {}
        for _, child in children:
            if child.poll() is None:
                try:
                    os.killpg(child.pid, signal.SIGTERM)
                except ProcessLookupError:
                    pass
        deadline = time.monotonic() + 3.0
        for _, child in children:
            remaining = max(0.0, deadline - time.monotonic())
            try:
                child.wait(timeout=remaining)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(child.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
        self._mode = 'off'

    def _apply_mode(self, wanted: str) -> None:
        self._stop_all()
        if wanted == 'off':
            self._fault = ''
            return
        # When both are required, the OV5647 must settle before IMX219 to avoid
        # the verified VSE ret -217 initialization race.
        order = ('left', 'right') if wanted == 'both' else (wanted,)
        for index, side in enumerate(order):
            if index and self._settle:
                time.sleep(self._settle)
            if not self._start_child(side):
                self._stop_all()
                self._fault_mode = wanted
                return
        self._fault = ''
        self._fault_mode = ''
        self._mode = wanted
        self.get_logger().info(f'Side camera mode: {wanted} at {self._width}x{self._height}')

    def _tick(self) -> None:
        now = time.monotonic()
        for side, child in list(self._children.items()):
            if child.poll() is not None:
                self._fault = f'{side} camera exited with {child.returncode}; retry requires OFF then request'
                self._fault_mode = self._mode
                self.get_logger().error(self._fault)
                self._stop_all()
                return
        wanted = self._wanted_mode(now)
        if wanted == self._fault_mode:
            return
        if wanted != self._mode:
            self._apply_mode(wanted)

    def _publish_status(self) -> None:
        payload = {
            'mode': self._mode,
            'requested': self._wanted_mode(time.monotonic()),
            'resolution': [self._width, self._height],
            'active': sorted(self._children),
            'fault': self._fault,
        }
        self._status_pub.publish(String(data=json.dumps(payload, separators=(',', ':'))))

    def destroy_node(self):
        self._stop_all()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SideCameraManager()
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
