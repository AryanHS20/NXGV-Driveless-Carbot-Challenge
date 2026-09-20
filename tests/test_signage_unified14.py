"""Consistency tests for the unified14 signage forward-port (no BPU needed)."""
import hashlib
import json
import os
import unittest

import yaml

import ros_stub

ros_stub.install()

from risabot_automode.signage_detector import CLASS_NAMES, SignageDetector

ROOT = os.path.join(os.path.dirname(__file__), '..')
MANIFEST = os.path.join(ROOT, 'tools', 'bpu_model', 'model_manifest_unified14.json')
PARAMS = os.path.join(ROOT, 'src', 'risabot_automode', 'config', 'params.yaml')


class Unified14ContractTests(unittest.TestCase):
    def test_fourteen_classes_no_lamp_no_partial(self):
        self.assertEqual(len(CLASS_NAMES), 14)
        self.assertNotIn('traffic_light', CLASS_NAMES)
        self.assertNotIn('boom_partial', CLASS_NAMES)
        self.assertEqual(CLASS_NAMES[7], 'traffic_warn_sign')
        self.assertEqual(CLASS_NAMES[8], 'tunnel_sign')
        self.assertEqual(CLASS_NAMES[9], 'traffic_red')
        self.assertEqual(CLASS_NAMES[12], 'boom_closed')
        self.assertEqual(CLASS_NAMES[13], 'boom_open')

    def test_threshold_keys_match_classes(self):
        keys = SignageDetector._THRESH_KEYS
        self.assertEqual(len(keys), 14)
        self.assertNotIn('thresh_tl_lamp', keys)
        self.assertNotIn('thresh_boom_partial', keys)
        self.assertIn('thresh_traffic_red', keys)
        self.assertIn('thresh_boom_open', keys)

    def test_params_yaml_matches(self):
        with open(PARAMS, encoding='utf-8') as handle:
            params = yaml.safe_load(handle)['signage_detector']['ros__parameters']
        self.assertIn('unified14_', params['model_path'])
        for key in SignageDetector._THRESH_KEYS:
            self.assertIn(key, params, key)
        self.assertNotIn('thresh_tl_lamp', params)
        self.assertNotIn('thresh_boom_partial', params)

    def test_manifest_matches_decoder_and_hash(self):
        with open(MANIFEST, encoding='utf-8') as handle:
            manifest = json.load(handle)
        self.assertEqual(manifest['classes'], CLASS_NAMES)
        self.assertEqual(len(manifest['classes']), 14)
        model = os.path.join(ROOT, 'tools', 'bpu_model', 'model_output',
                             manifest['file'].split('/')[-1])
        digest = hashlib.sha256(open(model, 'rb').read()).hexdigest()
        self.assertEqual(digest, manifest['sha256'])


if __name__ == '__main__':
    unittest.main()
