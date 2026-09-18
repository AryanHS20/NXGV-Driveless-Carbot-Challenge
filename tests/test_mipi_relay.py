"""Unit tests for the MIPI relay (pure python, no ROS)."""
import unittest

import ros_stub

ros_stub.install()

from risabot_automode.mipi_relay import MipiRelay, should_publish
from risabot_automode.topics import (
    CAMERA_IMAGE_TOPIC,
    MIPI_IMX219_TOPIC,
    MIPI_OV5647_TOPIC,
    MIPI_SECONDARY_TOPIC,
)


class ShouldPublishTests(unittest.TestCase):
    def test_first_frame_always_passes(self):
        # Node starts with last=0.0 while the monotonic clock is large.
        self.assertTrue(should_publish(0.0, 1000.0, 30.0))

    def test_throttles_fast_frames(self):
        self.assertFalse(should_publish(10.0, 10.001, 30.0))
        self.assertTrue(should_publish(10.0, 10.05, 30.0))

    def test_nonpositive_hz_means_unlimited(self):
        self.assertTrue(should_publish(10.0, 10.0, 0.0))
        self.assertTrue(should_publish(10.0, 10.0, -5.0))


class TopicConstantTests(unittest.TestCase):
    def test_mipi_topics_sane(self):
        self.assertTrue(MIPI_IMX219_TOPIC.endswith('/image_raw'))
        self.assertTrue(MIPI_OV5647_TOPIC.endswith('/image_raw'))
        self.assertNotEqual(MIPI_IMX219_TOPIC, MIPI_OV5647_TOPIC)
        self.assertNotEqual(MIPI_SECONDARY_TOPIC, CAMERA_IMAGE_TOPIC)


class RelayWiringTests(unittest.TestCase):
    def test_forward_and_second_publish(self):
        node = MipiRelay()
        fwd_pub = node._fwd_pub
        second_pub = node._second_pub
        self.assertEqual(len(fwd_pub.messages), 0)
        node._fwd_cb(ros_stub.Message())
        node._second_cb(ros_stub.Message())
        self.assertEqual(len(fwd_pub.messages), 1)
        self.assertEqual(len(second_pub.messages), 1)

    def test_throttle_drops_immediate_second_frame(self):
        node = MipiRelay()
        node._param_cache['max_hz'] = 1.0
        node._fwd_cb(ros_stub.Message())
        node._fwd_cb(ros_stub.Message())
        self.assertEqual(len(node._fwd_pub.messages), 1)

    def test_second_cb_without_pub_is_noop(self):
        node = MipiRelay()
        node._second_pub = None
        node._second_cb(ros_stub.Message())  # must not raise


if __name__ == '__main__':
    unittest.main()
