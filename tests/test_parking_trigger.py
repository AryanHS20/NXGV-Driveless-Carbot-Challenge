"""Parking sign trigger: parallel and perpendicular each start their own playback."""

import unittest
from unittest.mock import patch

import ros_stub

ros_stub.install()
from ros_stub import Message
from risabot_automode.parallel_park_trigger import ParallelParkTrigger


class ParkingTriggerTests(unittest.TestCase):
    def run_signs(self, signs, seconds=4.0, dt=0.1):
        """signs: kind shown while t < seconds ('' = no sign)."""
        with patch('time.monotonic') as clock:
            clock.return_value = 100.0
            node = ParallelParkTrigger()
            t = 100.0
            while t < 100.0 + seconds:
                clock.return_value = t
                kind = signs(t - 100.0)
                if kind is not None:
                    node._kind_cb(Message(kind))
                node._tick()
                t += dt
        return [m.data for m in node.cmd_pub.messages]

    def test_parallel_fires_parallel(self):
        self.assertEqual(self.run_signs(lambda t: 'parallel'), ['playback:parallel'])

    def test_perpendicular_fires_perpendicular(self):
        self.assertEqual(self.run_signs(lambda t: 'perpendicular'), ['playback:perpendicular'])

    def test_needs_hold_time(self):
        self.assertEqual(self.run_signs(lambda t: 'perpendicular' if t < 1.0 else '', 4.0), [])

    def test_sign_change_restarts_hold_and_fires_once(self):
        msgs = self.run_signs(lambda t: 'parallel' if t < 1.5 else 'perpendicular', 5.0)
        self.assertEqual(msgs, ['playback:perpendicular'])

    def test_fires_once_while_sign_stays(self):
        self.assertEqual(len(self.run_signs(lambda t: 'perpendicular', 10.0)), 1)


if __name__ == '__main__':
    unittest.main()
