import unittest

import numpy as np

from risabot_v4_experimental.road_mask_core import (
    RoadMaskConfig,
    extract_corridor,
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


if __name__ == '__main__':
    unittest.main()
