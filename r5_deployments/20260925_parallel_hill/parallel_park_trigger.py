#!/usr/bin/env python3
"""One-shot, operator-armed R5 parallel sign trigger for named playback.

Arm only while R5 is stopped at the recorded starting pose. The sign is
confirmed after AUTO is selected; this node never selects AUTO itself.
"""

import json
import time

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from std_msgs.msg import Bool, String

from .topics import RECORD_PLAYBACK_CMD_TOPIC, RECORD_PLAYBACK_STATE_TOPIC


class ParallelParkTrigger(Node):
    def __init__(self):
        super().__init__('parallel_park_trigger')
        self.declare_parameter('armed', False)
        self.declare_parameter('auto_arm_on_sign', True)
        self.declare_parameter('recording_name', 'parking_movement_20260925_1602')
        self.declare_parameter('hold_sec', 2.0)
        self._armed = bool(self.get_parameter('armed').value)
        self._armed_by_sign = False
        self._fired = False
        self._saw_auto = False
        self._auto = False
        self._auto_stamp = 0.0
        self._permitted = False
        self._permit_stamp = 0.0
        self._gate_open = False
        self._gate_stamp = 0.0
        self._kind = ''
        self._kind_stamp = 0.0
        self._seen_since = None
        self._waiting_for_playback = False
        self._playback_state = ''
        self._playback_stamp = 0.0
        self._saved_names = set()
        self.add_on_set_parameters_callback(self._on_params)
        self.cmd_pub = self.create_publisher(String, RECORD_PLAYBACK_CMD_TOPIC, 10)
        self.wait_pub = self.create_publisher(Bool, '/parallel_parking_wait', 10)
        self.create_subscription(String, '/parking_sign_kind', self._kind_cb, 10)
        self.create_subscription(Bool, '/auto_mode', self._auto_cb, 10)
        self.create_subscription(Bool, '/motion_permitted', self._permit_cb, 10)
        self.create_subscription(Bool, '/boom_gate_open', self._gate_cb, 10)
        self.create_subscription(String, RECORD_PLAYBACK_STATE_TOPIC, self._playback_cb, 10)
        self.create_timer(0.1, self._tick)

    def _on_params(self, params):
        for param in params:
            if param.name == 'armed':
                if not isinstance(param.value, bool):
                    return SetParametersResult(successful=False, reason='armed must be boolean')
                if not param.value:
                    self._armed = False
                    self._armed_by_sign = False
                    self._seen_since = None
                    if not self._auto:
                        self._waiting_for_playback = False
                        self.wait_pub.publish(Bool(data=False))
                elif not self._armed:
                    if self._auto or not 0 <= time.monotonic() - self._auto_stamp < 1.0:
                        return SetParametersResult(
                            successful=False, reason='arm requires fresh MANUAL mode')
                    # Another run requires a new explicit MANUAL arm.
                    self._armed = True
                    self._armed_by_sign = False
                    self._fired = False
                    self._saw_auto = self._auto
                    self._seen_since = None
                    self.wait_pub.publish(Bool(data=True))
            elif param.name in ('auto_arm_on_sign', 'recording_name', 'hold_sec'):
                return SetParametersResult(successful=False, reason='requires restart')
        return SetParametersResult(successful=True)

    def _auto_cb(self, msg):
        now = time.monotonic()
        was_auto = self._auto
        self._auto = bool(msg.data)
        self._auto_stamp = now
        if self._auto:
            self._saw_auto = True
        elif was_auto and self._saw_auto:
            self._armed = False
            self._armed_by_sign = False
            self._seen_since = None
            self._waiting_for_playback = False
            self.wait_pub.publish(Bool(data=False))
        if self._auto != was_auto:
            self._seen_since = None

    def _permit_cb(self, msg):
        self._permitted = bool(msg.data)
        self._permit_stamp = time.monotonic()

    def _gate_cb(self, msg):
        self._gate_open = bool(msg.data)
        self._gate_stamp = time.monotonic()

    def _kind_cb(self, msg):
        self._kind = msg.data.strip().lower()
        self._kind_stamp = time.monotonic()

    def _playback_cb(self, msg):
        try:
            data = json.loads(msg.data)
            self._playback_state = str(data['state'])
            self._saved_names = {str(item['name']) for item in data.get('saved_recordings', [])}
            self._playback_stamp = time.monotonic()
            if self._playback_state == 'PLAYBACK':
                self._armed = False
                self._armed_by_sign = False
                self._waiting_for_playback = False
                self.wait_pub.publish(Bool(data=False))
        except (ValueError, KeyError, TypeError):
            self._playback_state = ''
            self._playback_stamp = 0.0

    def _tick(self):
        now = time.monotonic()
        sign = self._kind == 'parallel' and 0 <= now - self._kind_stamp < 0.5
        auto_fresh = self._auto and 0 <= now - self._auto_stamp < 1.0
        recording_ready = (self._playback_state == 'IDLE'
                           and 0 <= now - self._playback_stamp < 1.0
                           and self.get_parameter('recording_name').value in self._saved_names)
        if (self.get_parameter('auto_arm_on_sign').value
                and not self._armed and not self._fired and not self._waiting_for_playback
                and auto_fresh and sign and recording_ready):
            # The first accepted sign freezes the lane command at this pose.
            self._armed = True
            self._armed_by_sign = True
            self._seen_since = now
            self.wait_pub.publish(Bool(data=True))
        if self._armed_by_sign and (not sign or not auto_fresh):
            # A transient sign releases the hold and resumes normal lane flow.
            self._armed = False
            self._armed_by_sign = False
            self._seen_since = None
            self.wait_pub.publish(Bool(data=False))
        self.wait_pub.publish(Bool(data=self._armed or self._waiting_for_playback))
        ready = (self._armed and not self._fired and self._auto
                 and auto_fresh
                 and self._permitted and 0 <= now - self._permit_stamp < 0.4
                 and self._gate_open and 0 <= now - self._gate_stamp < 1.0
                 and recording_ready)
        if not (sign and ready):
            self._seen_since = None
            return
        if self._seen_since is None:
            self._seen_since = now
        elif now - self._seen_since >= float(self.get_parameter('hold_sec').value):
            self._fired = True
            self._armed = False
            self._armed_by_sign = False
            self._waiting_for_playback = True
            self.cmd_pub.publish(String(data='playback:parallel'))
            self.get_logger().warn('Parallel sign confirmed: selected named parallel playback once')


def main(args=None):
    rclpy.init(args=args)
    node = ParallelParkTrigger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
