"""Parking sign trigger: armed in AUTO only, re-armed the moment playback stops."""

import json
import unittest
from unittest.mock import patch

import ros_stub

ros_stub.install()
from ros_stub import Message
from risabot_automode.parallel_park_trigger import ParallelParkTrigger


class Rig:
    """Scripted world: mode, sign and servo playback state, ticking at 10 Hz."""

    def __init__(self, test):
        self.patcher = patch('time.monotonic')
        self.clock = self.patcher.start()
        test.addCleanup(self.patcher.stop)
        self.t = 100.0
        self.clock.return_value = self.t
        self.node = ParallelParkTrigger()
        self.auto, self.sign = True, ''

    def state(self, name, result=''):
        self.node._state_cb(Message(json.dumps({'state': name, 'result': result})))

    def run(self, seconds):
        for _ in range(int(round(seconds * 10))):
            self.t += 0.1
            self.clock.return_value = self.t
            self.node._mode_cb(Message(self.auto))            # heartbeat
            if self.sign is not None:
                self.node._kind_cb(Message(self.sign))
            self.node._tick()

    @property
    def sent(self):
        return [m.data for m in self.node.cmd_pub.messages]


class ParkingTriggerTests(unittest.TestCase):
    def setUp(self):
        self.r = Rig(self)

    def test_parallel_fires_parallel_in_auto(self):
        self.r.sign = 'parallel'
        self.r.run(4)
        self.assertEqual(self.r.sent, ['playback:parallel'])

    def test_perpendicular_fires_perpendicular_in_auto(self):
        self.r.sign = 'perpendicular'
        self.r.run(4)
        self.assertEqual(self.r.sent, ['playback:perpendicular'])

    def test_never_fires_in_manual(self):
        self.r.auto, self.r.sign = False, 'parallel'
        self.r.run(10)
        self.assertEqual(self.r.sent, [])

    def test_entering_auto_restarts_the_hold(self):
        self.r.auto, self.r.sign = False, 'parallel'
        self.r.run(10)                    # sign visible for ages in MANUAL
        self.r.auto = True
        self.r.run(1.0)
        self.assertEqual(self.r.sent, [])  # not yet: hold starts on entering AUTO
        self.r.run(1.5)
        self.assertEqual(self.r.sent, ['playback:parallel'])

    def test_dropping_to_manual_cancels_hold(self):
        self.r.sign = 'parallel'
        self.r.run(1.5)
        self.r.auto = False
        self.r.run(1.0)
        self.r.auto = True
        self.r.run(1.5)                   # only 1.5 s since re-entering AUTO
        self.assertEqual(self.r.sent, [])

    def test_needs_hold_time(self):
        self.r.sign = 'perpendicular'
        self.r.run(1.0)
        self.r.sign = ''
        self.r.run(3.0)
        self.assertEqual(self.r.sent, [])

    def test_sign_change_restarts_hold_and_fires_once(self):
        self.r.sign = 'parallel'
        self.r.run(1.5)
        self.r.sign = 'perpendicular'
        self.r.run(3.5)
        self.assertEqual(self.r.sent, ['playback:perpendicular'])

    def test_no_second_fire_while_playing(self):
        self.r.sign = 'parallel'
        self.r.run(3)
        self.r.state('PLAYBACK', 'running')
        self.r.run(20)                    # sign still in view the whole time
        self.assertEqual(self.r.sent, ['playback:parallel'])

    def test_rearms_immediately_when_playback_stops(self):
        self.r.sign = 'parallel'
        self.r.run(3)
        self.r.state('PLAYBACK', 'running')
        self.r.run(5)
        self.r.state('IDLE', 'complete')
        self.r.run(1.0)
        self.assertEqual(len(self.r.sent), 1)   # fresh hold still counting
        self.r.run(1.5)
        self.assertEqual(self.r.sent, ['playback:parallel', 'playback:parallel'])

    def test_rearms_after_abort_and_takes_other_sign(self):
        self.r.sign = 'parallel'
        self.r.run(3)
        self.r.state('PLAYBACK', 'running')
        self.r.run(2)
        self.r.state('IDLE', 'aborted')
        self.r.sign = 'perpendicular'
        self.r.run(3)
        self.assertEqual(self.r.sent, ['playback:parallel', 'playback:perpendicular'])

    def test_playback_that_never_starts_is_retried(self):
        self.r.sign = 'parallel'
        self.r.run(3)                     # fires once; servo answers interlock
        self.r.state('IDLE', 'interlock')
        self.r.run(2.5)
        self.assertEqual(len(self.r.sent), 1)   # still inside start timeout
        self.r.run(6)
        self.assertGreaterEqual(len(self.r.sent), 2)

    def test_does_not_fire_while_recording(self):
        self.r.state('RECORDING')
        self.r.sign = 'parallel'
        self.r.run(6)
        self.assertEqual(self.r.sent, [])

    def test_disabled_never_fires(self):
        self.r.node._param_cache['enabled'] = False
        self.r.sign = 'parallel'
        self.r.run(6)
        self.assertEqual(self.r.sent, [])


if __name__ == '__main__':
    unittest.main()
