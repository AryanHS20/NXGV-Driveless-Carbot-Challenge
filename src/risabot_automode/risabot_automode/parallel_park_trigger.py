#!/usr/bin/env python3
"""
Parallel Park Trigger Node
=============================================================================
Watches the signage classifier's /parking_sign_kind. When it reports 'parallel' or
'perpendicular' continuously for `hold_sec` (default 2.0 s) it sends
`playback:parallel` / `playback:perpendicular` to servo_controller, which replays
the recording named by its `parallel_recording` / `perpendicular_recording`
parameter (motor + servo). The playback is routed through cmd_safety_controller,
which ignores the lane/auto command while playback is active, so it overrides all
automode functions. `stop` on /record_playback_cmd aborts it.

Re-arming: after firing, the trigger stays latched until the sign has been
absent for `rearm_clear_sec` AND `cooldown_sec` has elapsed. Re-run by hand any
time the servo is idle:
  ros2 topic pub --once /record_playback_cmd std_msgs/msg/String "{data: 'playback:parallel'}"
  ros2 topic pub --once /record_playback_cmd std_msgs/msg/String "{data: 'playback:perpendicular'}"

Subscribes: /parking_sign_kind (String: 'parallel' | 'perpendicular' | '')
Publishes:  /record_playback_cmd (String)
"""

import time
from typing import Dict

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from std_msgs.msg import String

from .topics import RECORD_PLAYBACK_CMD_TOPIC

PARKING_KIND_TOPIC = '/parking_sign_kind'


class ParallelParkTrigger(Node):
    """Fires the park sequence after the parallel-parking sign is held for N seconds."""

    def __init__(self):
        super().__init__('parallel_park_trigger')

        self.declare_parameter('enabled', True)
        self.declare_parameter('hold_sec', 2.0)          # continuous detection needed
        self.declare_parameter('stale_sec', 1.5)         # no message this long = sign not seen
        self.declare_parameter('rearm_clear_sec', 3.0)   # sign must vanish this long to re-arm
        self.declare_parameter('cooldown_sec', 30.0)     # min gap between auto-triggers
        self.declare_parameter('check_hz', 10.0)

        self._param_cache: Dict[str, object] = {}
        self._update_param_cache()
        self.add_on_set_parameters_callback(self._on_params)

        self.cmd_pub = self.create_publisher(String, RECORD_PLAYBACK_CMD_TOPIC, 10)
        self.create_subscription(String, PARKING_KIND_TOPIC, self._kind_cb, 10)

        self.last_kind = ''
        self.last_kind_stamp = 0.0
        self.seen_since = None        # monotonic time the current sign streak began
        self.seen_kind = ''           # 'parallel' | 'perpendicular' for that streak
        self.clear_since = time.monotonic()
        self.latched = False          # True after firing until re-armed
        self.last_fire = -1e9

        self.create_timer(1.0 / float(self._param_cache['check_hz']), self._tick)
        self.get_logger().info('Parallel Park Trigger ready (waiting for parallel / perpendicular sign)')

    def _update_param_cache(self) -> None:
        self._param_cache = {
            'enabled': bool(self.get_parameter('enabled').value),
            'hold_sec': float(self.get_parameter('hold_sec').value),
            'stale_sec': float(self.get_parameter('stale_sec').value),
            'rearm_clear_sec': float(self.get_parameter('rearm_clear_sec').value),
            'cooldown_sec': float(self.get_parameter('cooldown_sec').value),
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

    def _tick(self) -> None:
        now = time.monotonic()
        kind = self.last_kind if self.last_kind in ('parallel', 'perpendicular') else ''
        seen = bool(kind) and now - self.last_kind_stamp <= float(self._param_cache['stale_sec'])

        if seen:
            if self.seen_since is None or kind != self.seen_kind:
                self.seen_since = now      # new sign (or sign changed): restart the hold
                self.seen_kind = kind
        else:
            if self.seen_since is not None:
                self.clear_since = now
            self.seen_since = None

        # Re-arm once the sign has been gone long enough and cooldown passed
        if self.latched and not seen:
            if (now - self.clear_since >= float(self._param_cache['rearm_clear_sec'])
                    and now - self.last_fire >= float(self._param_cache['cooldown_sec'])):
                self.latched = False
                self.get_logger().info('Trigger re-armed')

        if not self._param_cache['enabled'] or self.latched:
            return

        if seen and self.seen_since is not None:
            held = now - self.seen_since
            if held >= float(self._param_cache['hold_sec']):
                self.get_logger().info(f'{self.seen_kind} sign held {held:.1f}s -> playback:{self.seen_kind}')
                self.cmd_pub.publish(String(data=f'playback:{self.seen_kind}'))
                self.latched = True
                self.last_fire = now


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
