"""Encoder-distance parking playback: segmenting and closed-loop replay."""

import unittest

import ros_stub

ros_stub.install()
from control_servo.encoder_segments import SegmentPlayer, build_segments, describe

CENTER = 80


def record(moves, tps=1000.0, coast_ticks=0.0):
    """Fake operator: (servo_angle, motor_pwm, seconds) blocks, 20 Hz, cumulative ticks."""
    samples, enc = [], 0.0
    for angle, pwm, secs in moves:
        for _ in range(int(secs * 20)):
            samples.append({'motor_pwm': pwm, 'servo_angle': angle, 'enc_ticks': enc})
            enc += pwm / 40.0 * tps / 20.0
    return samples


def replay(segments, gain=1.0, stall_after=None, dt=0.05):
    """Closed loop against a fake car whose wheel speed is duty*gain (slip/speed error)."""
    player = SegmentPlayer(segments, CENTER)
    enc, t, trace = 0.0, 0.0, []
    for _ in range(4000):
        out = player.step(t, enc)
        if out.abort or out.done:
            return player, enc, out, trace
        moving = stall_after is None or t < stall_after
        enc += (out.motor_pwm / 40.0 * 1000.0 * gain * dt) if moving else 0.0
        trace.append((t, out.motor_pwm, out.servo_angle, enc))
        t += dt
    raise AssertionError('playback never finished')


class BuildSegmentsTest(unittest.TestCase):
    def test_splits_on_steering_and_stop(self):
        s = build_segments(record([(80, 0, 1), (80, 40, 2), (80, 0, 1), (30, 0, 1),
                                   (30, -40, 1), (30, 0, 1)]))
        self.assertEqual([(x.servo_angle, x.direction) for x in s], [(80, 1), (30, -1)])
        self.assertAlmostEqual(s[0].ticks, 2000.0 - 50.0, delta=60)
        self.assertAlmostEqual(s[1].ticks, 1000.0 - 50.0, delta=60)
        self.assertEqual(s[0].duty, 40)

    def test_steer_change_while_moving_splits(self):
        s = build_segments(record([(80, 40, 1), (30, 40, 1)]))
        self.assertEqual(len(s), 2)

    def test_tiny_blip_ignored(self):
        self.assertEqual(build_segments(record([(80, 0, 1), (80, 1, 0.1), (80, 0, 1)])), [])

    def test_describe(self):
        text = describe(build_segments(record([(30, -40, 1), (30, 0, 1)])), CENTER, 1000.0)
        self.assertIn('rev', text[0])
        self.assertIn('left', text[0])


class ReplayTest(unittest.TestCase):
    def setUp(self):
        self.segments = build_segments(record([(80, 40, 2), (80, 0, 1), (30, 0, 1),
                                               (30, -40, 1.5), (30, 0, 1), (130, 0, 1),
                                               (130, 40, 1), (130, 0, 1)]))

    def test_reaches_distance_with_slow_or_fast_wheels(self):
        for gain in (0.6, 1.0, 1.4):
            player, enc, out, _ = replay(self.segments, gain=gain)
            self.assertTrue(out.done, gain)
            # net travel = +seg1 - seg2 + seg3, independent of wheel speed
            want = self.segments[0].ticks - self.segments[1].ticks + self.segments[2].ticks
            self.assertAlmostEqual(enc, want, delta=120, msg=f'gain {gain}')

    def test_never_drives_while_steering_changes(self):
        _, _, _, trace = replay(self.segments)
        for (t0, m0, a0, _), (t1, m1, a1, _) in zip(trace, trace[1:]):
            if a0 != a1:
                self.assertEqual((m0, m1), (0, 0), f'moved while steering at t={t1:.2f}')

    def test_steers_before_moving(self):
        _, _, _, trace = replay(self.segments)
        first_reverse = next(x for x in trace if x[1] < 0)
        self.assertEqual(first_reverse[2], 30)

    def test_stall_aborts(self):
        _, _, out, _ = replay(self.segments, stall_after=1.5)
        self.assertEqual(out.abort, 'encoder_stall')
        self.assertEqual(out.motor_pwm, 0)

    def test_duty_override(self):
        player = SegmentPlayer(self.segments, CENTER, duty_override=25)
        t, enc = 0.0, 0.0
        for _ in range(60):
            out = player.step(t, enc)
            t += 0.05
        self.assertIn(out.motor_pwm, (0, 25))


class ServoIntegrationTest(unittest.TestCase):
    """The real servo node: recorded encoder ticks drive playback distance and stop."""

    def test_playback_uses_encoder_and_keeps_state_heartbeat(self):
        import tempfile
        from unittest.mock import patch
        from test_track_test_pipeline import TrackTestPipelineTests
        base = TrackTestPipelineTests('test_lane_chain_slows_for_turn_without_stopping')
        base.setUp()
        try:
            node = base.servo()
            node.joy_unlocked = True
            node.record_buffer = record([(80, 0, 1), (30, 0, 1), (30, -40, 2), (30, 0, 1)])
            node.rp_state = 'IDLE'
            node.enc_ticks_total = 0.0
            target = build_segments(node.record_buffer)[0].ticks
            t, enc = 200.0, 0.0
            with patch('time.monotonic') as clock:
                def tick(now):
                    clock.return_value = now
                    node.last_joy_time = node.last_permit_time = node.last_auto_cmd_time = now
                clock.return_value = t
                tick(t)
                node._start_playback()
                self.assertIsNotNone(node.enc_player)
                states_before = len(node.rp_state_pub.messages)
                cmds = []
                for _ in range(400):
                    if node.rp_state != 'PLAYBACK':
                        break
                    tick(t)
                    node._playback_step()
                    cmd = node.playback_cmd_pub.messages[-1]
                    cmds.append(cmd.linear.x * 255.0)
                    enc += cmd.linear.x * 255.0 / 40.0 * 1000.0 * 0.05 * 0.7   # wheels 30% slow
                    node.enc_ticks_total = enc
                    t += 0.05
            self.assertEqual(node.playback_result, 'complete')
            self.assertGreater(len(node.rp_state_pub.messages), states_before + 10)
            self.assertLess(min(cmds), -30)                       # drove in reverse
            self.assertAlmostEqual(abs(enc), target, delta=150)   # despite slow wheels
        finally:
            base.doCleanups()


if __name__ == '__main__':
    unittest.main()
