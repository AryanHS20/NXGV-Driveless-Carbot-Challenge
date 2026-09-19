"""Guarded raw-range UWB bridge for V4 coarse-global pose diagnostics."""

import json
import math
import time
from typing import List, Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String

from .uwb_core import Anchor, Fix, UwbError, UwbRangeProcessor


STATUS_TOPIC = '/v4_experimental/uwb/status'


class UwbBridgeShadow(Node):
    def __init__(self) -> None:
        super().__init__('v4_uwb_bridge_shadow')
        self.declare_parameter('enabled', False)
        self.declare_parameter('raw_topic', '/uwb3/input_json')
        self.declare_parameter('fix_topic', '/v4_experimental/uwb/fix')
        self.declare_parameter('status_topic', STATUS_TOPIC)
        self.declare_parameter('status_hz', 5.0)
        self.declare_parameter('fix_timeout_sec', 1.0)
        self.declare_parameter('maximum_range_age_sec', 0.40)
        self.declare_parameter('minimum_anchor_count', 3)
        self.declare_parameter('minimum_anchor_area_m2', 0.25)
        self.declare_parameter('maximum_residual_m', 0.30)
        self.declare_parameter('maximum_geometry_condition', 100.0)
        self.declare_parameter('minimum_sigma_m', 0.03)
        self.declare_parameter('tag_z_m', 0.0)
        self.declare_parameter('anchor_geometry_validated', False)
        self.declare_parameter('range_offsets_validated', False)
        self.declare_parameter('antenna_heights_validated', False)
        self.declare_parameter('anchor_ids', ['1782', '1783', '1786'])
        anchor_ids = [str(value) for value in self.get_parameter('anchor_ids').value]
        defaults = {
            '1782': (0.0, 0.0, 0.0),
            '1783': (7.5, 4.83, 0.0),
            '1786': (7.5, 0.0, 0.0),
        }
        anchors: List[Anchor] = []
        for anchor_id in anchor_ids:
            prefix = f'anchor_{anchor_id}'
            x, y, z = defaults.get(anchor_id, (0.0, 0.0, 0.0))
            self.declare_parameter(f'{prefix}_x_m', x)
            self.declare_parameter(f'{prefix}_y_m', y)
            self.declare_parameter(f'{prefix}_z_m', z)
            self.declare_parameter(f'{prefix}_range_offset_m', 0.0)
            anchors.append(Anchor(
                anchor_id,
                float(self.get_parameter(f'{prefix}_x_m').value),
                float(self.get_parameter(f'{prefix}_y_m').value),
                float(self.get_parameter(f'{prefix}_z_m').value),
                float(self.get_parameter(f'{prefix}_range_offset_m').value),
            ))

        self._enabled = bool(self.get_parameter('enabled').value)
        self._gates = {
            'anchor_geometry_validated': bool(
                self.get_parameter('anchor_geometry_validated').value),
            'range_offsets_validated': bool(
                self.get_parameter('range_offsets_validated').value),
            'antenna_heights_validated': bool(
                self.get_parameter('antenna_heights_validated').value),
        }
        self._fix_timeout = float(self.get_parameter('fix_timeout_sec').value)
        status_hz = float(self.get_parameter('status_hz').value)
        if not math.isfinite(status_hz) or status_hz <= 0.0:
            raise UwbError('status_hz must be finite and positive')
        if not math.isfinite(self._fix_timeout) or self._fix_timeout <= 0.0:
            raise UwbError('fix_timeout_sec must be finite and positive')
        self._processor = UwbRangeProcessor(
            anchors,
            tag_z_m=float(self.get_parameter('tag_z_m').value),
            maximum_range_age_sec=float(
                self.get_parameter('maximum_range_age_sec').value),
            minimum_anchor_count=int(self.get_parameter('minimum_anchor_count').value),
            minimum_anchor_area_m2=float(
                self.get_parameter('minimum_anchor_area_m2').value),
            maximum_residual_m=float(self.get_parameter('maximum_residual_m').value),
            maximum_geometry_condition=float(
                self.get_parameter('maximum_geometry_condition').value),
            minimum_sigma_m=float(self.get_parameter('minimum_sigma_m').value),
        )
        self._last_raw_mono: Optional[float] = None
        self._last_fix_mono: Optional[float] = None
        self._last_fix: Optional[Fix] = None
        self._last_reason = 'waiting_for_raw_ranges'

        self._fix_pub = self.create_publisher(
            String, str(self.get_parameter('fix_topic').value), 10)
        self._status_pub = self.create_publisher(
            String, str(self.get_parameter('status_topic').value), 10)
        self.create_subscription(
            String, str(self.get_parameter('raw_topic').value),
            self._raw_callback, qos_profile_sensor_data)
        self.create_timer(1.0 / status_hz, self._timer_callback)
        self.get_logger().warning(
            'V4 UWB bridge is diagnostic-only; motion authority is permanently false'
        )

    @property
    def _calibration_ready(self) -> bool:
        return all(self._gates.values())

    def _raw_callback(self, msg: String) -> None:
        if not self._enabled:
            return
        now = time.monotonic()
        self._last_raw_mono = now
        if not self._calibration_ready:
            self._last_reason = 'calibration_gates_closed'
            return
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            self._processor.invalid_count += 1
            self._last_reason = 'invalid_json'
            return
        fix, reason = self._processor.ingest(payload, now)
        self._last_reason = reason
        if fix is None:
            return
        self._last_fix = fix
        self._last_fix_mono = now
        self._publish_fix(fix)

    def _publish_fix(self, fix: Fix) -> None:
        anchors = [
            {
                'id': sample.anchor_id,
                'range_m': round(sample.raw_range_m, 5),
                'age_ms': round(sample.age_sec * 1000.0, 1),
                'sample_seq': sample.sample_seq,
            }
            for sample in sorted(fix.samples, key=lambda item: item.anchor_id)
        ]
        payload = {
            'valid': True,
            'x': fix.x_m,
            'y': fix.y_m,
            'age': fix.age_sec,
            'sigma_m': fix.sigma_m,
            'variance_m2': fix.sigma_m ** 2,
            'residual_rms_m': fix.residual_rms_m,
            'maximum_residual_m': fix.maximum_residual_m,
            'geometry_condition': fix.geometry_condition,
            'boot_id': fix.boot_id,
            'seq': fix.report_seq,
            'anchors': anchors,
        }
        message = String()
        message.data = json.dumps(payload, separators=(',', ':'), sort_keys=True)
        self._fix_pub.publish(message)

    def _publish_invalid(self, reason: str, age: Optional[float]) -> None:
        message = String()
        message.data = json.dumps({
            'valid': False,
            'reason': reason,
            'age': age,
            'anchors': [],
        }, separators=(',', ':'), sort_keys=True)
        self._fix_pub.publish(message)

    def _timer_callback(self) -> None:
        now = time.monotonic()
        raw_age = None if self._last_raw_mono is None else now - self._last_raw_mono
        fix_age = None if self._last_fix_mono is None else now - self._last_fix_mono
        if self._enabled and (fix_age is None or fix_age > self._fix_timeout):
            reason = 'calibration_gates_closed' if not self._calibration_ready else 'fix_timeout'
            self._publish_invalid(reason, fix_age)
        blockers = [name for name, passed in self._gates.items() if not passed]
        status = String()
        status.data = json.dumps({
            'algorithm_stage': 3,
            'component': 'uwb_range_bridge',
            'mode': 'shadow',
            'enabled': self._enabled,
            'motion_authority': False,
            'can_publish_motion': False,
            'ready': bool(
                self._enabled and self._calibration_ready
                and fix_age is not None and fix_age <= self._fix_timeout),
            'calibration_ready': self._calibration_ready,
            'blockers': blockers,
            'raw_age_sec': None if raw_age is None else round(raw_age, 3),
            'fix_age_sec': None if fix_age is None else round(fix_age, 3),
            'last_reason': self._last_reason,
            'boot_id': self._processor.boot_id,
            'duplicate_ranges': self._processor.duplicate_count,
            'out_of_order_ranges': self._processor.out_of_order_count,
            'invalid_ranges': self._processor.invalid_count,
        }, separators=(',', ':'), sort_keys=True)
        self._status_pub.publish(status)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = UwbBridgeShadow()
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
