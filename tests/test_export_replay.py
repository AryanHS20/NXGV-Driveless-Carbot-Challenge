"""Unit tests for the replay exporter and replay_store helpers (no ROS)."""
import json
import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'tools'))

import export_replay

import ros_stub

ros_stub.install()

from risabot_automode.replay_store import (
    list_replays,
    load_replay_json,
    mime_for,
    safe_replay_path,
    safe_static_path,
    summarize_replay,
)


def write_lines(path, lines):
    with open(path, 'w', encoding='utf-8') as handle:
        for line in lines:
            handle.write(line + '\n')


class ExporterTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp(prefix='replay_')
        self.run = os.path.join(self.tmp, 'run_20260918_082444.jsonl')
        write_lines(self.run, [
            json.dumps({'meta': {'node': 'map_recorder'}, 't_wall': 100.0}),
            json.dumps({'t_wall': 101.0, 'ox': 0.1, 'oy': 0.0, 'oyaw': 0.0,
                        'speed': 0.15, 'lane_error': -0.04, 'tl': 'green'}),
            '{broken json',
            '',
            json.dumps({'t_wall': 102.0, 'ox': None, 'oy': None, 'speed': 0.0}),
        ])
        self.status = os.path.join(self.tmp, 'status.jsonl')
        write_lines(self.status, [
            json.dumps({'t_wall': 101.1, 'status': {'corridor': {'primary': [
                {'forward_m': 0.5, 'left_m': 0.02, 'width_m': 0.3}]}}}),
            json.dumps({'t_wall': 500.0, 'status': {'corridor': {'primary': []}}}),
        ])

    def test_export_schema_and_merge(self):
        out = os.path.join(self.tmp, 'run.replay.json')
        doc, path = export_replay.export_run(self.run, self.status, out)
        self.assertEqual(path, out)
        self.assertEqual(doc['version'], 1)
        self.assertEqual(doc['source_run'], 'run_20260918_082444.jsonl')
        # meta + corrupt + blank lines skipped: 2 frames.
        self.assertEqual(len(doc['frames']), 2)
        first = doc['frames'][0]
        self.assertEqual(first['odom'], {'x': 0.1, 'y': 0.0, 'yaw': 0.0})
        self.assertEqual(len(first['corridor']), 1)
        self.assertAlmostEqual(first['corridor'][0]['forward_m'], 0.5)
        # Far snapshot must not merge into the second frame.
        self.assertEqual(doc['frames'][1]['corridor'], [])
        self.assertIsNone(doc['frames'][1]['odom'])

    def test_export_without_sidecar(self):
        out = os.path.join(self.tmp, 'plain.replay.json')
        doc, _ = export_replay.export_run(self.run, None, out)
        self.assertEqual(len(doc['frames']), 2)
        self.assertEqual(doc['frames'][0]['corridor'], [])

    def test_downsample_stride(self):
        out = os.path.join(self.tmp, 'small.replay.json')
        doc, _ = export_replay.export_run(self.run, None, out, max_frames=1)
        self.assertEqual(len(doc['frames']), 1)

    def test_default_out_path(self):
        doc, path = export_replay.export_run(self.run)
        self.assertTrue(path.endswith('.replay.json'))
        self.assertTrue(os.path.isfile(path))
        self.assertEqual(doc['version'], 1)


class ReplayStoreTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp(prefix='store_')
        self.sim = os.path.join(self.tmp, 'sim_views')
        os.makedirs(self.sim)
        with open(os.path.join(self.sim, 'index.html'), 'w') as handle:
            handle.write('<html></html>')
        with open(os.path.join(self.sim, 'app.bundle.js'), 'w') as handle:
            handle.write('var x = 1;')
        with open(os.path.join(self.tmp, 'run_a.jsonl'), 'w') as handle:
            handle.write('{}\n')
        with open(os.path.join(self.tmp, 'run_a.replay.json'), 'w') as handle:
            json.dump({'version': 1, 'source_run': 'run_a.jsonl', 'frames': [
                {'t': 1.0, 'odom': {'x': 0, 'y': 0, 'yaw': 0}, 'corridor': []},
                {'t': 3.0, 'odom': None, 'corridor': [{'forward_m': 1}]}]},
                handle)

    def test_static_safe_paths(self):
        self.assertTrue(safe_static_path(self.sim, 'index.html').endswith('index.html'))
        self.assertIsNone(safe_static_path(self.sim, '../secret'))
        self.assertIsNone(safe_static_path(self.sim, 'missing.js'))
        self.assertIsNone(safe_static_path(self.sim, 'index.html '))
        self.assertEqual(mime_for('a.js'), 'application/javascript')
        self.assertIsNone(mime_for('a.exe'))

    def test_replay_paths_and_list(self):
        good = safe_replay_path(self.tmp, 'run_a.replay.json')
        self.assertTrue(good and good.endswith('.replay.json'))
        self.assertIsNone(safe_replay_path(self.tmp, '../x.json'))
        self.assertIsNone(safe_replay_path(self.tmp, 'x.txt'))
        self.assertIsNone(safe_replay_path(self.tmp, 'nope.json'))
        listed = list_replays(self.tmp)
        self.assertEqual(listed['runs'], ['run_a.jsonl'])
        self.assertEqual(listed['replays'], ['run_a.replay.json'])
        self.assertEqual(list_replays(os.path.join(self.tmp, 'nope')),
                         {'runs': [], 'replays': []})

    def test_summarize_and_load(self):
        doc, err = load_replay_json(os.path.join(self.tmp, 'run_a.replay.json'))
        self.assertEqual(err, '')
        summary = summarize_replay(doc)
        self.assertEqual(summary['frames'], 2)
        self.assertEqual(summary['duration_s'], 2.0)
        self.assertTrue(summary['has_odom'])
        self.assertEqual(summary['corridor_frames'], 1)
        bad, err = load_replay_json(os.path.join(self.tmp, 'run_a.jsonl'))
        self.assertIsNone(bad)
        self.assertTrue(err)
        self.assertEqual(summarize_replay({})['frames'], 0)


if __name__ == '__main__':
    unittest.main()
