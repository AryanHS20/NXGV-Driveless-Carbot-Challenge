"""park_sequence (wiggle x3 -> pause -> recording) and the parallel park trigger.

Real node classes with inert ROS/hardware interfaces (same harness as
test_safety_logic).
"""
import tempfile
import unittest
from unittest.mock import patch

import ros_stub
ros_stub.install()
from ros_stub import Message

from control_servo.servo_controller import ServoControllerV9
from risabot_automode.parallel_park_trigger import ParallelParkTrigger

REC = [dict(motor_pwm=40, servo_angle=100)] * 7


class ParkSequenceTests(unittest.TestCase):
    def setUp(self):
        self.clock = patch('time.monotonic', return_value=100.0)
        self.clock.start()
        self.addCleanup(self.clock.stop)
        ros_stub.NOW = 100.0

    def servo(self):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        with patch('os.path.expanduser', side_effect=lambda s: tmp.name + '/' + s.removeprefix('~/')):
            v = ServoControllerV9()
        v.manual_mode = False
        v.last_auto_cmd_time = v.last_permit_time = 100.
        v.motion_permitted = True
        v.joy_unlocked = True
        v.joy_alive = True
        v.last_joy_time = 100.
        return v

    def test_buffer_is_wiggle_then_pause_then_recording(self):
        v = self.servo()
        v.record_buffer = list(REC)
        v.bot.writes = []
        v._start_park_sequence()
        self.assertEqual(v.rp_state, 'PLAYBACK')
        self.assertEqual(v.playback_kind, 'parallel')
        left = v.servo_center - v.servo_range_left
        right = v.servo_center + v.servo_range_right
        hold, delay = 20, 100          # 1.0 s and 5.0 s at 20 Hz
        buf = v.record_buffer
        self.assertEqual(len(buf), 3 * 2 * hold + delay + len(REC))
        # three left/right sweeps, motor stopped
        for cycle in range(3):
            base = cycle * 2 * hold
            self.assertTrue(all(s['servo_angle'] == left and s['motor_pwm'] == 0 for s in buf[base:base + hold]))
            self.assertTrue(all(s['servo_angle'] == right and s['motor_pwm'] == 0
                                for s in buf[base + hold:base + 2 * hold]))
        # centered pause, then the untouched recording
        pause = buf[6 * hold:6 * hold + delay]
        self.assertTrue(all(s['servo_angle'] == v.servo_center and s['motor_pwm'] == 0 for s in pause))
        self.assertEqual(buf[6 * hold + delay:], REC)
        # goes through the safety path (a request), never a direct hardware write
        self.assertFalse(v.bot.writes)
        self.assertLess(v.playback_cmd_pub.messages[-1].angular.z, 0)   # first step = full left
        self.assertEqual(v.playback_cmd_pub.messages[-1].linear.x, 0)

    def test_runs_to_completion_then_can_run_again(self):
        v = self.servo()
        v.record_buffer = list(REC)
        v._start_park_sequence()
        for _ in range(len(v.record_buffer) + 2):
            v._playback_step()
        self.assertEqual(v.rp_state, 'IDLE')
        self.assertEqual(v.playback_result, 'complete')
        self.assertEqual(v.record_buffer, REC)          # real recording restored, no wiggle prefix
        v._record_playback_cmd_cb(Message('park_sequence'))   # re-run after a failed attempt
        self.assertEqual(v.rp_state, 'PLAYBACK')

    def test_refuses_without_a_recording(self):
        v = self.servo()
        v.record_buffer = []
        v._start_park_sequence()
        self.assertEqual(v.rp_state, 'IDLE')
        self.assertEqual(v.playback_result, 'missing_recording')

    def test_interlock_refusal_restores_recording(self):
        v = self.servo()
        v.record_buffer = list(REC)
        v.motion_permitted = False
        v._start_park_sequence()
        self.assertEqual(v.rp_state, 'IDLE')
        self.assertEqual(v.playback_result, 'interlock')
        self.assertEqual(v.record_buffer, REC)

    def test_stop_aborts_and_restores(self):
        v = self.servo()
        v.record_buffer = list(REC)
        v._start_park_sequence()
        v._record_playback_cmd_cb(Message('stop'))
        self.assertEqual(v.rp_state, 'IDLE')
        self.assertEqual(v.record_buffer, REC)

    def test_ignored_while_busy(self):
        v = self.servo()
        v.record_buffer = list(REC)
        v._start_park_sequence()
        size = len(v.record_buffer)
        v._record_playback_cmd_cb(Message('park_sequence'))
        self.assertEqual(len(v.record_buffer), size)

    def test_loads_active_recording_from_disk(self):
        v = self.servo()
        v.record_buffer = list(REC)
        v._save_recording('p1')
        v._set_active_parking('p1')
        v.record_buffer = []                              # stale buffer must not matter
        v._start_park_sequence()
        self.assertEqual(v.rp_state, 'PLAYBACK')
        self.assertEqual(v.record_buffer[-len(REC):], REC)


class TriggerTests(unittest.TestCase):
    def setUp(self):
        self.now = 100.0
        self.clock = patch('time.monotonic', side_effect=lambda: self.now)
        self.clock.start()
        self.addCleanup(self.clock.stop)
        ros_stub.NOW = 100.0
        self.t = ParallelParkTrigger()

    def feed(self, kind, seconds, step=0.5):
        """Publish `kind` every `step` s for `seconds`, ticking the node each time."""
        end = self.now + seconds
        while self.now < end:
            self.t._kind_cb(Message(kind))
            self.t._tick()
            self.now += step
        self.t._tick()

    def sent(self):
        return [m.data for m in self.t.cmd_pub.messages]

    def test_fires_after_two_seconds_of_parallel(self):
        self.feed('parallel', 1.5)
        self.assertEqual(self.sent(), [])
        self.feed('parallel', 1.0)
        self.assertEqual(self.sent(), ['park_sequence'])

    def test_broken_streak_restarts_the_clock(self):
        self.feed('parallel', 1.5)
        self.feed('', 3.0)
        self.feed('parallel', 1.5)
        self.assertEqual(self.sent(), [])

    def test_perpendicular_never_fires(self):
        self.feed('perpendicular', 10.0)
        self.assertEqual(self.sent(), [])

    def test_fires_once_then_rearms_after_clear_and_cooldown(self):
        self.feed('parallel', 3.0)
        self.feed('parallel', 5.0)                 # still seen: latched, no repeat
        self.assertEqual(len(self.sent()), 1)
        self.feed('', 10.0)                        # cleared, but cooldown (30 s) not over
        self.feed('parallel', 3.0)
        self.assertEqual(len(self.sent()), 1)
        self.feed('', 40.0)                        # clear + cooldown elapsed -> re-armed
        self.feed('parallel', 3.0)
        self.assertEqual(len(self.sent()), 2)

    def test_stale_kind_counts_as_not_seen(self):
        self.t._kind_cb(Message('parallel'))
        self.now += 10.0                           # classifier stopped publishing
        self.t._tick()
        self.assertIsNone(self.t.seen_since)
        self.assertEqual(self.sent(), [])


if __name__ == '__main__':
    unittest.main()
