import math
import unittest

from risabot_v4_experimental.uwb_core import Anchor, UwbError, UwbRangeProcessor, validate_anchor_geometry


ANCHORS = [Anchor('1782', 0.0, 0.0), Anchor('1783', 7.5, 4.83), Anchor('1786', 7.5, 0.0)]


def payload(x=4.0, y=1.5, *, boot='boot-a', report_seq=20, sample_seq=10,
            ages=(30, 40, 50), bad_anchor=None):
    links = []
    for index, anchor in enumerate(ANCHORS):
        distance = math.hypot(x - anchor.x_m, y - anchor.y_m)
        if anchor.anchor_id == bad_anchor:
            distance += 1.0
        links.append({'A': anchor.anchor_id, 'R': distance, 'age_ms': ages[index],
                      'sample_seq': sample_seq + index})
    return {'boot_id': boot, 'seq': report_seq, 'links': links}


class UwbCoreTests(unittest.TestCase):
    def processor(self, anchors=ANCHORS, **kwargs):
        return UwbRangeProcessor(anchors, tag_z_m=kwargs.pop('tag_z_m', 0.0),
                                 minimum_anchor_area_m2=0.25, **kwargs)

    def test_exact_ranges_recover_position_and_contract_metadata(self):
        fix, reason = self.processor().ingest(payload(), 100.0)
        self.assertEqual(reason, 'accepted')
        self.assertAlmostEqual(fix.x_m, 4.0, places=5)
        self.assertAlmostEqual(fix.y_m, 1.5, places=5)
        self.assertEqual((fix.boot_id, fix.report_seq, len(fix.samples)), ('boot-a', 20, 3))
        self.assertAlmostEqual(fix.age_sec, 0.05, places=6)
        self.assertGreaterEqual(fix.sigma_m, 0.03)

    def test_range_offsets_and_height_correction_are_applied(self):
        x, y, tag_z = 4.0, 1.5, 0.25
        configured = [Anchor(a.anchor_id, a.x_m, a.y_m, 1.20, 0.12) for a in ANCHORS]
        links = []
        for index, anchor in enumerate(configured):
            true_3d = math.sqrt((x-anchor.x_m)**2 + (y-anchor.y_m)**2 + (tag_z-anchor.z_m)**2)
            links.append({'A': anchor.anchor_id, 'R': true_3d + 0.12,
                          'age_ms': 20, 'sample_seq': index + 1})
        fix, reason = self.processor(configured, tag_z_m=tag_z).ingest(
            {'boot_id': 'height', 'seq': 1, 'links': links}, 10.0)
        self.assertEqual(reason, 'accepted')
        self.assertAlmostEqual(fix.x_m, x, places=5)
        self.assertAlmostEqual(fix.y_m, y, places=5)

    def test_duplicate_bundle_is_not_fused_again(self):
        processor = self.processor()
        self.assertIsNotNone(processor.ingest(payload(), 10.0)[0])
        fix, reason = processor.ingest(payload(report_seq=21), 10.1)
        self.assertIsNone(fix)
        self.assertEqual(reason, 'no_new_ranges')
        self.assertEqual(processor.duplicate_count, 3)

    def test_one_new_anchor_reuses_only_still_fresh_cached_ranges(self):
        processor = self.processor(maximum_range_age_sec=0.4)
        processor.ingest(payload(sample_seq=10), 10.0)
        update = payload(sample_seq=10)
        update['links'] = [dict(update['links'][0], sample_seq=20, age_ms=10)]
        self.assertEqual(processor.ingest(update, 10.2)[1], 'accepted')
        later = payload(sample_seq=10)
        later['links'] = [dict(later['links'][0], sample_seq=21, age_ms=10)]
        fix, reason = processor.ingest(later, 10.6)
        self.assertIsNone(fix)
        self.assertEqual(reason, 'insufficient_fresh_anchors')

    def test_boot_change_resets_sequence_and_cached_ranges(self):
        processor = self.processor()
        processor.ingest(payload(sample_seq=100), 10.0)
        fix, reason = processor.ingest(payload(boot='boot-b', sample_seq=1), 10.1)
        self.assertEqual(reason, 'accepted')
        self.assertEqual(fix.boot_id, 'boot-b')

    def test_sequence_wrap_is_accepted_but_old_sequence_is_rejected(self):
        processor = self.processor()
        processor.ingest(payload(sample_seq=0xFFFFFFFD), 10.0)
        self.assertEqual(processor.ingest(payload(sample_seq=1), 10.1)[1], 'accepted')
        fix, reason = processor.ingest(payload(sample_seq=0), 10.2)
        self.assertIsNone(fix)
        self.assertEqual(reason, 'no_new_ranges')
        self.assertEqual(processor.out_of_order_count, 3)

    def test_stale_nonfinite_and_missing_fields_never_form_fix(self):
        processor = self.processor()
        self.assertIsNone(processor.ingest(payload(ages=(500, 500, 500)), 1.0)[0])
        malformed = payload(boot='boot-b')
        malformed['links'][0]['R'] = float('nan')
        malformed['links'][1].pop('sample_seq')
        self.assertIsNone(processor.ingest(malformed, 2.0)[0])
        self.assertGreaterEqual(processor.invalid_count, 5)

    def test_large_single_anchor_error_is_rejected_by_residual_gate(self):
        fix, reason = self.processor(maximum_residual_m=0.05).ingest(
            payload(bad_anchor='1786'), 10.0)
        self.assertIsNone(fix)
        self.assertEqual(reason, 'residual_gate')

    def test_unknown_anchor_is_ignored(self):
        data = payload()
        data['links'].append({'A': '9999', 'R': 1.0, 'age_ms': 0, 'sample_seq': 1})
        fix, reason = self.processor().ingest(data, 10.0)
        self.assertEqual(reason, 'accepted')
        self.assertEqual(len(fix.samples), 3)

    def test_collinear_or_duplicate_anchor_geometry_is_rejected(self):
        with self.assertRaises(UwbError):
            validate_anchor_geometry([Anchor('a', 0, 0), Anchor('b', 1, 0), Anchor('c', 2, 0)], 0.1)
        with self.assertRaises(UwbError):
            validate_anchor_geometry([Anchor('a', 0, 0), Anchor('b', 0, 0), Anchor('c', 1, 1)], 0.1)


if __name__ == '__main__':
    unittest.main()
