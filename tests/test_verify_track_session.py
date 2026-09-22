import csv
import json
import os
import sys
import tempfile
import unittest

import yaml

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'tools'))
import verify_track_session


class VerifyTrackSessionTests(unittest.TestCase):
    def make_session(self, counts, events=True):
        root = tempfile.TemporaryDirectory()
        bag = os.path.join(root.name, 'bag')
        os.makedirs(bag)
        metadata = {
            'rosbag2_bagfile_information': {
                'topics_with_message_count': [
                    {'topic_metadata': {'name': name}, 'message_count': count}
                    for name, count in counts.items()
                ]
            }
        }
        with open(os.path.join(bag, 'metadata.yaml'), 'w', encoding='utf-8') as f:
            yaml.safe_dump(metadata, f)
        with open(os.path.join(root.name, 'events.jsonl'), 'w', encoding='utf-8') as f:
            if events:
                f.write(json.dumps({'event': 'straight_normal'}) + '\n')
        with open(os.path.join(root.name, 'system_health.csv'), 'w',
                  encoding='utf-8', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=['temperature_c'])
            writer.writeheader()
            writer.writerow({'temperature_c': '74.5'})
        return root

    def test_critical_topics_pass(self):
        counts = {name: 10 for name in verify_track_session.CRITICAL_TOPICS}
        counts.update({name: 3 for name in verify_track_session.CORE_V4_TOPICS})
        root = self.make_session(counts)
        self.addCleanup(root.cleanup)
        report = verify_track_session.verify(root.name)
        self.assertTrue(report['ok'])
        self.assertEqual(report['events'], ['straight_normal'])
        self.assertEqual(report['max_temperature_c'], 74.5)

    def test_zero_count_critical_topic_fails(self):
        counts = {name: 10 for name in verify_track_session.CRITICAL_TOPICS}
        counts['/scan'] = 0
        root = self.make_session(counts)
        self.addCleanup(root.cleanup)
        report = verify_track_session.verify(root.name)
        self.assertFalse(report['ok'])
        self.assertIn('/scan has no recorded messages', report['errors'])

    def test_missing_bag_metadata_fails(self):
        with tempfile.TemporaryDirectory() as root:
            report = verify_track_session.verify(root)
        self.assertFalse(report['ok'])
        self.assertIn('bag/metadata.yaml is missing', report['errors'])


if __name__ == '__main__':
    unittest.main()
