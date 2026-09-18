"""Unit tests for persistent run mapping (pure python, no ROS)."""
import json
import math
import os
import tempfile
import time
import unittest

import numpy as np

import ros_stub

ros_stub.install()

from risabot_automode.map_recorder import MapRecorder, extract_corridor
from risabot_automode.map_store import MapStore, parse_uwb_fix, solve_anchors


def make_store():
    """MapStore rooted at a fresh temp dir."""
    return MapStore(tempfile.mkdtemp(prefix='risabot_maps_'))


class ParseUwbFixTests(unittest.TestCase):
    def test_valid_fix_normalized(self):
        raw = json.dumps({'t': 12.5, 'x': 1.0, 'y': 2.0, 'valid': True,
                          'anchors': [{'id': 'A1', 'range_m': 3.0, 'age_ms': 40.0}]})
        fix = parse_uwb_fix(raw)
        self.assertTrue(fix['valid'])
        self.assertEqual(fix['x'], 1.0)
        self.assertEqual(len(fix['anchors']), 1)
        self.assertEqual(fix['anchors'][0]['id'], 'A1')

    def test_garbage_returns_none(self):
        for bad in ('not json', '42', '[1,2]', '{"anchors": "nope"}', ''):
            self.assertIsNone(parse_uwb_fix(bad))

    def test_bad_ranges_dropped_not_fatal(self):
        raw = json.dumps({'x': 0.0, 'y': 0.0, 'valid': True, 'anchors': [
            {'id': 'A1', 'range_m': 2.0},
            {'id': 'A2', 'range_m': -1.0},
            {'id': 'A3'},
            {'id': 'A4', 'range_m': float('inf')},
        ]})
        fix = parse_uwb_fix(raw)
        self.assertEqual([a['id'] for a in fix['anchors']], ['A1'])

    def test_missing_position_is_invalid(self):
        fix = parse_uwb_fix(json.dumps({'valid': True, 'anchors': []}))
        self.assertFalse(fix['valid'])
        self.assertIsNone(fix['x'])


class MapStoreRoundTripTests(unittest.TestCase):
    def test_record_and_reload(self):
        store = make_store()
        path = store.begin_run({'tag': 'unit'})
        self.assertTrue(os.path.isfile(path))
        for i in range(3):
            store.record({'t_wall': float(i), 'ox': 0.1 * i, 'oy': 0.0})
        store.close()
        rows = MapStore.load_run(path)
        self.assertEqual(rows[0]['meta'], {'tag': 'unit'})
        self.assertEqual(len(rows) - 1, 3)
        self.assertAlmostEqual(rows[2]['ox'], 0.1)

    def test_corrupt_lines_skipped(self):
        store = make_store()
        path = store.begin_run()
        store.record({'ok': True})
        store.close()
        with open(path, 'a', encoding='utf-8') as fh:
            fh.write('{broken json\n\n')
        rows = MapStore.load_run(path)
        self.assertEqual(len(rows), 2)

    def test_best_round_trip_and_backup(self):
        store = make_store()
        self.assertEqual(MapStore.load_best(store.map_dir), {})
        store.save_best({'version': 1, 'track': [[0.0, 0.0, 0.0]]})
        self.assertEqual(MapStore.load_best(store.map_dir)['version'], 1)
        store.save_best({'version': 1, 'track': []})
        self.assertTrue(os.path.isfile(store.best_path() + '.prev'))
        self.assertEqual(MapStore.load_best(store.map_dir)['track'], [])


class SolveAnchorsTests(unittest.TestCase):
    def test_recovers_synthetic_anchors(self):
        rng = np.random.default_rng(7)
        truth = {'A1': (5.0, 0.5), 'A2': (0.5, 5.0), 'A3': (5.0, 5.0)}
        samples = []
        for i in range(40):
            # L-shaped traverse (leg along x, then leg along y): a straight
            # line would leave a mirror ambiguity no range-only solver can fix.
            if i < 20:
                rx, ry = 0.25 * i, 0.05 * (i % 5)
            else:
                rx, ry = 5.0 + 0.05 * (i % 3), 0.25 * (i - 20)
            ranges = []
            for aid, (ax, ay) in truth.items():
                ranges.append((aid, math.hypot(ax - rx, ay - ry) + rng.normal(0, 0.02)))
            samples.append((rx, ry, ranges))
        solved = solve_anchors(samples)
        self.assertEqual(set(solved), set(truth))
        for aid, (ax, ay) in truth.items():
            self.assertLess(math.hypot(solved[aid][0] - ax, solved[aid][1] - ay), 0.15)

    def test_poorly_observed_anchor_skipped(self):
        samples = [(0.0, 0.0, [('A9', 1.0)]), (0.0, 0.0, [('A9', 1.0)])]
        self.assertEqual(solve_anchors(samples), {})


class SkeletonTests(unittest.TestCase):
    def test_track_landmarks_anchors(self):
        samples = []
        for i in range(40):
            ox = 0.02 * i
            samples.append({
                'ox': ox, 'oy': 0.0, 'oyaw': 0.0,
                'lm': [{'type': 'hill', 'dx': 1.0, 'dy': 0.0}],
                'uwb': {'x': ox, 'y': 0.0, 'valid': True, 'anchors': [
                    {'id': 'A1', 'range_m': math.hypot(5.0 - ox, 0.5), 'age_ms': 10.0},
                    {'id': 'A2', 'range_m': math.hypot(0.5 - ox, 5.0), 'age_ms': 10.0},
                ]},
            })
        skel = MapStore.build_map_skeleton(samples)
        self.assertEqual(skel['version'], 1)
        # 0.8 m traverse at 2 cm steps decimated to >= 5 cm: fewer, but >= 2.
        self.assertLess(len(skel['track']), 40)
        self.assertGreaterEqual(len(skel['track']), 2)
        for (x0, _y0, _w0), (x1, _y1, _w1) in zip(skel['track'], skel['track'][1:]):
            self.assertGreaterEqual(math.hypot(x1 - x0, 0.0), 0.05 - 1e-9)
        self.assertIn('hill', skel['landmarks'])
        cells = skel['landmarks']['hill']
        # 0.3 m cells split the 0.8 m sweep; counts must cover all 40 sightings
        # and the count-weighted mean must match the true sweep centre.
        self.assertEqual(sum(c['n'] for c in cells), 40)
        mean_x = sum(c['x'] * c['n'] for c in cells) / 40
        self.assertAlmostEqual(mean_x, np.mean([0.02 * i + 1.0 for i in range(40)]), places=6)
        self.assertEqual(set(skel['anchors']), {'A1', 'A2'})


class ExtractCorridorTests(unittest.TestCase):
    def test_valid_points_kept_bad_dropped(self):
        payload = {'corridor': {'primary': [
            {'forward_m': 0.5, 'left_m': 0.02, 'width_m': 0.3},
            {'forward_m': 1.0, 'left_m': 0.0},  # missing width
            {'forward_m': float('inf'), 'left_m': 0.0, 'width_m': 0.3},
            {'forward_m': 1.5, 'left_m': 0.0, 'width_m': -0.1},
            'junk',
        ]}}
        got = extract_corridor(payload)
        self.assertEqual(len(got), 1)
        self.assertAlmostEqual(got[0]['forward_m'], 0.5)

    def test_garbage_payloads_yield_empty(self):
        for bad in (None, [], 'x', {}, {'corridor': None},
                    {'corridor': {'primary': 'nope'}}):
            self.assertEqual(extract_corridor(bad), [])

    def test_point_cap(self):
        payload = {'corridor': {'primary': [
            {'forward_m': float(i), 'left_m': 0.0, 'width_m': 0.3}
            for i in range(500)]}}
        self.assertEqual(len(extract_corridor(payload)), 200)


class RecorderCorridorTests(unittest.TestCase):
    def _node_in_tmp_home(self):
        home = tempfile.mkdtemp(prefix='fakehome_')
        saved = {key: os.environ.get(key) for key in ('HOME', 'USERPROFILE')}
        os.environ['HOME'] = home
        os.environ['USERPROFILE'] = home
        self.addCleanup(self._restore_env, saved)
        node = MapRecorder()
        self.addCleanup(node.close)
        return node

    @staticmethod
    def _restore_env(saved):
        for key, value in saved.items():
            if value is None:
                os.environ.pop(key, None)
            else:
                os.environ[key] = value

    def _last_sample(self, node):
        node.close()  # flush buffered writes before reading back
        rows = MapStore.load_run(node._run_path)
        return rows[-1]

    def test_fresh_corridor_recorded(self):
        node = self._node_in_tmp_home()
        node._road_cb(ros_stub.Message(json.dumps({'corridor': {'primary': [
            {'forward_m': 0.5, 'left_m': 0.02, 'width_m': 0.3}]}})))
        node._tick()
        sample = self._last_sample(node)
        self.assertEqual(len(sample['corridor']), 1)
        self.assertAlmostEqual(sample['corridor'][0]['width_m'], 0.3)

    def test_stale_corridor_recorded_empty(self):
        node = self._node_in_tmp_home()
        node._road_cb(ros_stub.Message(json.dumps({'corridor': {'primary': [
            {'forward_m': 0.5, 'left_m': 0.02, 'width_m': 0.3}]}})))
        node._corridor_mono = time.monotonic() - 5.0
        node._tick()
        self.assertEqual(self._last_sample(node)['corridor'], [])

    def test_garbage_status_ignored(self):
        node = self._node_in_tmp_home()
        node._road_cb(ros_stub.Message('not json'))
        node._tick()
        self.assertEqual(self._last_sample(node)['corridor'], [])


if __name__ == '__main__':
    unittest.main()
