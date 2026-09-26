#!/usr/bin/env python3
"""
Parking Sign Trigger Node
=============================================================================
Watches the signage classifier's /parking_sign_kind. When it reports 'parallel' or
'perpendicular' continuously for `hold_sec` (default 2.0 s) it sends
`playback:parallel` / `playback:perpendicular` to servo_controller, which replays
the recording named by its `parallel_recording` / `perpendicular_recording`
parameter (motor + servo). The playback is routed through cmd_safety_controller,
which ignores the lane/auto command while playback is active, so it overrides all
automode functions. `stop` on /record_playback_cmd aborts it.

Armed only in AUTO: the sign hold does not start (and any hold in progress is
dropped) while the car is in MANUAL, so entering AUTO always restarts the hold.

Re-arming is immediate: the moment /record_playback_state leaves PLAYBACK (finished,
aborted, or forced to MANUAL) the trigger is armed again. The sign must be held
for a fresh `hold_sec` before it fires again.

If a request does not start a playback (interlock, missing recording) the trigger
waits `start_timeout_sec`, logs the servo's reason, and then tries again on the next
hold. Trigger a playback by hand any time the servo is idle:
  ros2 topic pub --once /record_playback_cmd std_msgs/msg/String "{data: 'playback:parallel'}"
  ros2 topic pub --once /record_playback_cmd std_msgs/msg/String "{data: 'playback:perpendicular'}"

Subscribes: /parking_sign_kind (String: 'parallel' | 'perpendicular' | ''),
            /auto_mode (Bool), /record_playback_state (String JSON)
Publishes:  /record_playback_cmd (String)
"""

import json
import time
from typing import Dict

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from std_msgs.msg import Bool, String

from .topics import AUTO_MODE_TOPIC, RECORD_PLAYBACK_CMD_TOPIC, RECORD_PLAYBACK_STATE_TOPIC

PARKING_KIND_TOPIC = '/parking_sign_kind'


class ParallelParkTrigger(Node):
    """Fires a parking playback after a parking sign is held for N seconds in AUTO."""

    def __init__(self):
        super().__init__('parallel_park_trigger')

        self.declare_parameter('enabled', True)
        self.declare_parameter('hold_sec', 2.0)            # continuous detection needed
        self.declare_parameter('stale_sec', 1.5)           # no sign message this long = sign gone
        self.declare_parameter('mode_stale_sec', 2.0)      # /auto_mode heartbeat older = not AUTO
        self.declare_parameter('start_timeout_sec', 3.0)   # wait this long for PLAYBACK to begin
        self.declare_parameter('check_hz', 10.0)

        self._param_cache: Dict[str, object] = {}
        self._update_param_cache()
        self.add_on_set_parameters_callback(self._on_params)

        self.cmd_pub = self.create_publisher(String, RECORD_PLAYBACK_CMD_TOPIC, 10)
        self.create_subscription(String, PARKING_KIND_TOPIC, self._kind_cb, 10)
        self.create_subscription(Bool, AUTO_MODE_TOPIC, self._mode_cb, 10)
        self.create_subscription(String, RECORD_PLAYBACK_STATE_TOPIC, self._state_cb, 10)

        self.last_kind = ''
        self.last_kind_stamp = 0.0
        self.auto = False
        self.auto_stamp = 0.0
        self.rp_state = 'IDLE'        # servo publishes on change only; assume idle until told
        self.rp_result = ''
        self.was_playing = False
        self.seen_since = None        # monotonic time the current sign streak began
        self.seen_kind = ''           # 'parallel' | 'perpendicular' for that streak
        self.waiting_since = None     # set when a request is sent, until PLAYBACK is seen

        self.create_timer(1.0 / float(self._param_cache['check_hz']), self._tick)
        self.get_logger().info(
            'Parking Trigger ready (armed in AUTO only; waiting for parallel / perpendicular sign)')

    def _update_param_cache(self) -> None:
        self._param_cache = {
            'enabled': bool(self.get_parameter('enabled').value),
            'hold_sec': float(self.get_parameter('hold_sec').value),
            'stale_sec': float(self.get_parameter('stale_sec').value),
            'mode_stale_sec': float(self.get_parameter('mode_stale_sec').value),
            'start_timeout_sec': float(self.get_parameter('start_timeout_sec').value),
            'check_hz': float(self.get_parameter('check_hz').value),
        }

    def _on_params(self, params) -> SetParametersResult:
        for p in params:
            if p.name in self._param_cache:
                self._param_cache[p.name] = p.value
        return SetParametersResult(successful=True)

    def _kind_cb(self, msg: String) -> None:
        self.last_kind = msg.data.strip().lower()
        self.last_kind_stamp = time.monotonic()

    def _mode_cb(self, msg: Bool) -> None:
        self.auto = bool(msg.data)
        self.auto_stamp = time.monotonic()

    def _state_cb(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        self.rp_state = str(payload.get('state', 'IDLE'))
        self.rp_result = str(payload.get('result', ''))

    def _tick(self) -> None:
        now = time.monotonic()

        # Playback running: never fire. The moment it stops, arm again immediately
        # (fresh hold), whether it completed, was aborted, or dropped to MANUAL.
        playing = self.rp_state == 'PLAYBACK'
        if playing:
            self.waiting_since = None
            self.seen_since = None
        elif self.was_playing:
            self.get_logger().info(f'Playback stopped ({self.rp_result or "?"}) -> trigger re-armed')
            self.waiting_since = None
            self.seen_since = None
        self.was_playing = playing
        if playing:
            return

        # A request was sent but no playback began: report the servo's reason and retry later.
        if self.waiting_since is not None:
            if now - self.waiting_since < float(self._param_cache['start_timeout_sec']):
                return
            self.get_logger().warn(
                f'Playback did not start (servo result: {self.rp_result or "none"}); re-armed')
            self.waiting_since = None
            self.seen_since = None

        # Armed only in AUTO.
        in_auto = self.auto and now - self.auto_stamp <= float(self._param_cache['mode_stale_sec'])
        if not in_auto or self.rp_state == 'RECORDING' or not self._param_cache['enabled']:
            self.seen_since = None
            return

        kind = self.last_kind if self.last_kind in ('parallel', 'perpendicular') else ''
        seen = bool(kind) and now - self.last_kind_stamp <= float(self._param_cache['stale_sec'])
        if not seen:
            self.seen_since = None
            return

        if self.seen_since is None or kind != self.seen_kind:
            self.seen_since = now          # new sign (or sign changed): restart the hold
            self.seen_kind = kind
            return

        held = now - self.seen_since
        if held >= float(self._param_cache['hold_sec']):
            self.get_logger().info(f'{kind} sign held {held:.1f}s in AUTO -> playback:{kind}')
            self.cmd_pub.publish(String(data=f'playback:{kind}'))
            self.waiting_since = now
            self.seen_since = None


def main(args=None) -> None:
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
