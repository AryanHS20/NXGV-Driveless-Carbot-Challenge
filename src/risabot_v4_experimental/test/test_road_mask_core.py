import unittest

import numpy as np

from risabot_v4_experimental.road_mask_core import (
    RoadMaskConfig,
    extract_corridor,
    prior_center_from_mask,
    process_bev,
    seed_connected_component,
    segment_dark_road,
    timestamps_synchronized,
)


class RoadMaskCoreTests(unittest.TestCase):
    def setUp(self):
        self.config = RoadMaskConfig(
            value_max=120,
            morph_open_px=0,
            morph_close_px=0,
            min_component_px=50,
        )

    def test_unobserved_black_pixels_are_not_road(self):
        image = np.zeros((80, 100, 3), np.uint8)
        coverage = np.zeros((80, 100), np.uint8)
        coverage[:, 30:70] = 255
        candidate = segment_dark_road(image, coverage, self.config)
        self.assertTrue(np.all(candidate[:, :30] == 0))
        self.assertTrue(np.all(candidate[:, 30:70] == 255))
        self.assertTrue(np.all(candidate[:, 70:] == 0))

    def test_seed_growth_rejects_disconnected_dark_region(self):
        candidate = np.zeros((100, 120), np.uint8)
        candidate[:, 45:75] = 255
        candidate[5:30, 2:25] = 255
        connected = seed_connected_component(candidate, (60, 92), 5, 50)
        self.assertTrue(np.all(connected[:, 45:75] == 255))
        self.assertTrue(np.all(connected[5:30, 2:25] == 0))

    def test_synthetic_road_produces_stable_corridor(self):
        image = np.full((120, 140, 3), 220, np.uint8)
        coverage = np.full((120, 140), 255, np.uint8)
        for row in range(120):
            center = 70 + int(round((119 - row) * 0.08))
            image[row, center - 20:center + 21] = 45
        result = process_bev(
            image,
            coverage,
            self.config,
            seed_xy=(70, 114),
            seed_radius_px=8,
            pixels_per_meter=100.0,
            min_width_m=0.30,
            max_width_m=0.60,
            row_step_px=6,
        )
        samples = result['samples']
        self.assertGreaterEqual(len(samples), 18)
        self.assertAlmostEqual(samples[0].center_px, 70.0, delta=1.0)
        self.assertGreater(samples[-1].center_px, samples[0].center_px + 6.0)
        self.assertTrue(all(39 <= sample.width_px <= 42 for sample in samples))

    def test_corridor_chooses_run_nearest_previous_center(self):
        mask = np.zeros((40, 100), np.uint8)
        mask[:, 5:25] = 255
        mask[:, 55:80] = 255
        coverage = np.full_like(mask, 255)
        samples = extract_corridor(
            mask,
            coverage,
            seed_center_x=65.0,
            pixels_per_meter=100.0,
            min_width_m=0.15,
            max_width_m=0.30,
            row_step_px=5,
        )
        self.assertTrue(samples)
        self.assertTrue(all(sample.left_px == 55 for sample in samples))

    def test_corridor_accepts_road_inside_narrow_camera_coverage(self):
        mask = np.zeros((40, 100), np.uint8)
        mask[:, 42:68] = 255
        coverage = np.zeros_like(mask)
        coverage[:, 38:72] = 255
        # Only 34% of the complete BEV row is visible, but the complete road
        # segment is observed. A triangular forward-camera FOV must pass.
        samples = extract_corridor(
            mask,
            coverage,
            seed_center_x=55.0,
            pixels_per_meter=100.0,
            min_width_m=0.20,
            max_width_m=0.40,
            row_step_px=5,
        )
        self.assertEqual(len(samples), 8)
        self.assertTrue(all(sample.left_px == 42 for sample in samples))

    def test_timestamp_pairing_rejects_previous_frame(self):
        self.assertFalse(timestamps_synchronized(10.2, 10.0, 0.1))
        self.assertTrue(timestamps_synchronized(10.2, 10.2, 0.1))

    def test_timestamp_pairing_accepts_unstamped_inputs(self):
        self.assertTrue(timestamps_synchronized(0.0, 10.0, 0.1))
        self.assertTrue(timestamps_synchronized(10.0, 0.0, 0.1))

    def _impostor_candidate(self):
        # Big centered impostor plus a smaller offset true-road strip; the
        # 30 px seed circle touches both, and legacy overlap voting picks
        # the big one.
        candidate = np.zeros((100, 120), np.uint8)
        candidate[:, 45:75] = 255
        candidate[:, 88:102] = 255
        return candidate

    def _impostor_seed(self):
        return (60, 92), 30

    def test_seed_without_prior_keeps_legacy_biggest_overlap(self):
        connected = seed_connected_component(
            self._impostor_candidate(), (60, 92), 8, 50
        )
        self.assertTrue(np.all(connected[:, 45:75] == 255))
        self.assertTrue(np.all(connected[:, 88:102] == 0))

    def test_seed_prior_prefers_remembered_road(self):
        candidate = self._impostor_candidate()
        prior = np.zeros_like(candidate)
        prior[:, 88:102] = 255
        seed_xy, seed_radius = self._impostor_seed()
        connected = seed_connected_component(
            candidate, seed_xy, seed_radius, 50, prior_mask=prior
        )
        self.assertTrue(np.all(connected[:, 88:102] == 255))
        self.assertTrue(np.all(connected[:, 45:75] == 0))

    def test_seed_empty_prior_falls_back_to_legacy(self):
        candidate = self._impostor_candidate()
        connected = seed_connected_component(
            candidate, (60, 92), 8, 50,
            prior_mask=np.zeros_like(candidate),
        )
        self.assertTrue(np.all(connected[:, 45:75] == 255))

    def test_seed_prior_outside_seed_falls_back_to_legacy(self):
        candidate = self._impostor_candidate()
        prior = np.zeros_like(candidate)
        prior[:, 0:10] = 255
        connected = seed_connected_component(
            candidate, (60, 92), 8, 50, prior_mask=prior
        )
        self.assertTrue(np.all(connected[:, 45:75] == 255))

    def test_seed_prior_below_min_component_falls_back(self):
        candidate = np.zeros((100, 120), np.uint8)
        candidate[:, 45:75] = 255
        candidate[80:90, 88:92] = 255
        prior = np.zeros_like(candidate)
        prior[80:90, 88:92] = 255
        connected = seed_connected_component(
            candidate, (60, 92), 8, 50, prior_mask=prior
        )
        self.assertTrue(np.all(connected[:, 45:75] == 255))

    def test_prior_center_from_mask_uses_nearest_rows(self):
        mask = np.zeros((100, 120), np.uint8)
        mask[90:, 80:100] = 255
        mask[:50, 0:20] = 255
        self.assertAlmostEqual(
            prior_center_from_mask(mask), 89.5, delta=1.0
        )

    def test_prior_center_from_empty_mask_is_none(self):
        self.assertIsNone(
            prior_center_from_mask(np.zeros((40, 60), np.uint8))
        )

    def test_process_bev_prior_selects_true_road(self):
        image = np.full((100, 120, 3), 220, np.uint8)
        image[:, 45:75] = 40
        image[:, 88:102] = 40
        coverage = np.full((100, 120), 255, np.uint8)
        prior = np.zeros((100, 120), np.uint8)
        prior[:, 88:102] = 255
        result = process_bev(
            image,
            coverage,
            self.config,
            seed_xy=(60, 92),
            seed_radius_px=30,
            pixels_per_meter=100.0,
            min_width_m=0.10,
            max_width_m=0.35,
            row_step_px=5,
            prior_mask=prior,
            prior_center_x=95.0,
        )
        self.assertTrue(result['prior_used'])
        samples = result['samples']
        self.assertTrue(samples)
        self.assertTrue(
            all(abs(sample.center_px - 95.0) < 8.0 for sample in samples)
        )


if __name__ == '__main__':
    unittest.main()
