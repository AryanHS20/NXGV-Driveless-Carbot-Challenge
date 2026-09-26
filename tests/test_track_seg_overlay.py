"""Tests for the r5_deployments/20260926_track_seg overlay (seg road source)."""

import importlib.util
import json
import sys
import types
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np

import ros_stub

ros_stub.install()
from ros_stub import Message

OVERLAY = Path(__file__).resolve().parents[1] / 'r5_deployments' / '20260926_track_seg'
PKG = 'risabot_v4_experimental'


def _load(name):
    spec = importlib.util.spec_from_file_location(f'{PKG}.{name}', OVERLAY / f'{name}.py')
    module = importlib.util.module_from_spec(spec)
    sys.modules[f'{PKG}.{name}'] = module
    spec.loader.exec_module(module)
    return module


def _msg(sec, nanosec=0, data=None):
    return Message(data=data, header=types.SimpleNamespace(
        stamp=types.SimpleNamespace(sec=sec, nanosec=nanosec), frame_id='camera'))


def _profile():
    from risabot_v4_experimental.bev_core import CameraProfile
    return CameraProfile(
        name='primary', calibrated=True, resolution=(60, 60),
        camera_matrix=np.eye(3), distortion=np.zeros(5),
        source_points_px=np.array([[0, 0], [59, 0], [59, 59], [0, 59]], float),
        ground_points_m=np.array([[0, -.3], [.59, -.3], [.59, .29], [0, .29]], float),
        pixels_per_meter=100., forward_bounds_m=(0., .59), left_bounds_m=(-.3, .29))


class OverlayCase(unittest.TestCase):
    def setUp(self):
        modules = patch.dict(sys.modules)
        modules.start()
        self.addCleanup(modules.stop)
        self.core = _load('road_mask_core')
        self.shadow = _load('road_mask_shadow')
        self.clock = [100.0]
        patcher = patch('time.monotonic', side_effect=lambda: self.clock[0])
        patcher.start()
        self.addCleanup(patcher.stop)

    def node(self, **params):
        merged = {'v4_road_mask_shadow': {'ros__parameters': {
            'enabled': True, 'use_road_memory': False, 'process_secondary': False, **params}}}
        patcher = patch.dict(ros_stub.PARAMS, merged)
        patcher.start()
        self.addCleanup(patcher.stop)
        node = self.shadow.RoadMaskShadow()
        node._profiles = {'primary': _profile()}
        self.bev = np.full((60, 60, 3), 220, np.uint8)  # bright: classical finds no road
        self.bev[:, 20:40] = 40                          # ...except this dark strip
        node._bridge = types.SimpleNamespace(
            imgmsg_to_cv2=lambda msg, **kw: msg.data,
            cv2_to_imgmsg=lambda data, **kw: Message(data.copy(), header=None))
        node._publish_status = lambda: None
        return node

    def feed(self, node, stamp, bev=None):
        node._dispatch('primary', _msg(stamp, data=self.bev if bev is None else bev),
                       np.full((60, 60), 255, np.uint8))


class CoreTests(OverlayCase):
    def test_override_replaces_threshold_and_is_limited_to_coverage(self):
        seg = np.zeros((60, 60), np.uint8)
        seg[:, 5:15] = 255
        coverage = np.full((60, 60), 255, np.uint8)
        coverage[:, :8] = 0
        out = self.core.seg_candidate_from_mask(seg, coverage)
        self.assertEqual(int(out[:, 5:8].max()), 0)
        self.assertEqual(int(out[:, 8:15].min()), 255)
        self.assertEqual(int(out[:, 15:].max()), 0)

    def test_override_rejects_bad_shape(self):
        with self.assertRaises(self.core.RoadMaskError):
            self.core.seg_candidate_from_mask(np.zeros((10, 10), np.uint8), np.zeros((60, 60), np.uint8))
        with self.assertRaises(self.core.RoadMaskError):
            self.core.seg_candidate_from_mask(np.zeros((60, 60, 3), np.uint8), np.zeros((60, 60), np.uint8))

    def test_process_bev_without_override_is_classical(self):
        bev = np.full((60, 60, 3), 220, np.uint8)
        bev[:, 20:40] = 40
        cov = np.full((60, 60), 255, np.uint8)
        cfg = self.core.RoadMaskConfig()
        kwargs = dict(seed_xy=(30., 30.), seed_radius_px=5, pixels_per_meter=100.)
        base = self.core.process_bev(bev, cov, cfg, **kwargs)
        np.testing.assert_array_equal(base['candidate'], self.core.segment_dark_road(bev, cov, cfg))
        seg = np.zeros((60, 60), np.uint8)
        seg[:, 25:35] = 255
        over = self.core.process_bev(bev, cov, cfg, candidate_override=seg, **kwargs)
        self.assertEqual(int(over['candidate'][:, 25:35].min()), 255)
        self.assertEqual(int(over['candidate'][:, :25].max()), 0)


class ShadowTests(OverlayCase):
    def test_default_is_classical_and_has_no_seg_subscription(self):
        node = self.node()
        self.assertFalse(node._seg_mode)
        self.assertNotIn('/v4_experimental/road/primary/seg_candidate', node.subs)
        self.feed(node, 1)
        self.assertEqual(node._last_source['primary'], 'classical')

    def test_invalid_source_falls_back_to_classical(self):
        self.assertFalse(self.node(mask_source='bogus')._seg_mode)

    def test_seg_mode_subscribes(self):
        node = self.node(mask_source='seg')
        self.assertTrue(node._seg_mode)
        self.assertIn('/v4_experimental/road/primary/seg_candidate', node.subs)

    def test_seg_arrives_after_bev_frame_and_is_used(self):
        node = self.node(mask_source='seg')
        seg = np.zeros((60, 60), np.uint8)
        seg[:, 10:20] = 255
        node._seg_callback(_msg(5, data=np.zeros((60, 60), np.uint8)))  # older frame: fresh, wrong stamp
        self.feed(node, 6)
        self.assertIsNotNone(node._seg_wait)
        self.assertEqual(node._frames['primary'], 0)
        node._seg_callback(_msg(6, data=seg))
        self.assertIsNone(node._seg_wait)
        self.assertEqual(node._last_source['primary'], 'seg')
        cand = node._candidate_pubs['primary'].messages[-1].data
        self.assertEqual(int(cand[:, 10:20].min()), 255)
        self.assertEqual(int(cand[:, 20:].max()), 0)  # classical dark strip (20:40) NOT used
        self.assertEqual(node._seg_counts['used'], 1)

    def test_seg_arrives_before_bev_frame(self):
        node = self.node(mask_source='seg')
        seg = np.zeros((60, 60), np.uint8)
        seg[:, 10:20] = 255
        node._seg_callback(_msg(7, data=seg))
        self.feed(node, 7)
        self.assertEqual(node._last_source['primary'], 'seg')

    def test_frame_without_seg_is_dropped_not_mixed_with_classical(self):
        node = self.node(mask_source='seg')
        seg = np.zeros((60, 60), np.uint8)
        seg[:, 10:20] = 255
        node._seg_callback(_msg(1, data=seg))
        self.feed(node, 2)   # rate-capped seg skipped this frame
        self.feed(node, 3)   # next frame arrives; frame 2 is dropped, seg still fresh
        self.assertEqual(node._seg_counts['dropped'], 1)
        self.assertEqual(node._seg_counts['fallback'], 0)
        self.assertEqual(node._frames['primary'], 0)

    def test_stale_seg_falls_back_to_classical(self):
        node = self.node(mask_source='seg')
        node._seg_callback(_msg(1, data=np.zeros((60, 60), np.uint8)))
        self.clock[0] += 1.0  # > seg_timeout_sec (0.5)
        self.feed(node, 2)
        self.assertEqual(node._last_source['primary'], 'classical')
        self.assertEqual(node._seg_counts['fallback'], 1)

    def test_no_seg_ever_falls_back_to_classical(self):
        node = self.node(mask_source='seg')
        self.feed(node, 1)
        self.assertEqual(node._last_source['primary'], 'classical')

    def test_fallback_disabled_publishes_nothing(self):
        node = self.node(mask_source='seg', seg_fallback_to_classical=False)
        self.feed(node, 1)
        self.assertEqual(node._frames['primary'], 0)
        self.assertIn('stale', node._last_error['primary'])

    def test_status_reports_source(self):
        node = self.node(mask_source='seg')
        del node._publish_status  # use the real one
        node._profiles = {}
        node._status_pub.messages = []
        node._publish_status()
        payload = json.loads(node._status_pub.messages[-1].data)
        self.assertEqual(payload['mask_source'], 'seg')
        self.assertEqual(set(payload['seg']), {'used', 'fallback', 'dropped', 'fresh'})


class NodeHelperTests(unittest.TestCase):
    def test_logits_to_mask_layouts(self):
        # The board node imports rclpy/cv_bridge at module level; load just the helpers.
        source = (OVERLAY / 'track_seg_node.py').read_text(encoding='utf-8')
        start = source.index('def bgr_to_nv12')
        end = source.index('class TrackSegNode')
        ns = {'np': np, 'cv2': __import__('cv2')}
        exec(source[start:end], ns)
        chw = np.zeros((1, 2, 4, 6), np.float32)
        chw[0, 1, :, :3] = 5
        chw[0, 0, :, 3:] = 5
        mask = ns['logits_to_mask'](chw)
        self.assertEqual(mask.shape, (4, 6))
        self.assertEqual(int(mask[:, :3].min()), 1)
        self.assertEqual(int(mask[:, 3:].max()), 0)
        np.testing.assert_array_equal(ns['logits_to_mask'](np.transpose(chw[0], (1, 2, 0))), mask)
        with self.assertRaises(ValueError):
            ns['logits_to_mask'](np.zeros((1, 3, 4, 4), np.float32))
        nv12 = ns['bgr_to_nv12'](np.zeros((288, 512, 3), np.uint8))
        self.assertEqual(nv12.shape, (432, 512))


if __name__ == '__main__':
    unittest.main()
