#!/usr/bin/env python3
"""
Unit Test Suite for Path Planning, Heading Fusion, and Kinematics
Covers:
  - 2nd-degree Polynomial curve fitting & Pure Pursuit tracker
  - NaN/Degenerate fallback guards
  - Angle normalization and shortest wrap difference
  - VFH+ Polar sector histogram evaluation with [0.2, 0.6, 0.2] smoothing and 15% hysteresis
  - 3rd-order Bezier spline generation & apex lateral clearance
"""

import math
import unittest
import numpy as np


# ──────────────────────────────────────────────────────────────────────────────
# 1. Polynomial Curve Fitting & Pure Pursuit Tracker
# ──────────────────────────────────────────────────────────────────────────────

def fit_lane_polynomial(y_norm: np.ndarray, x_norm: np.ndarray):
    """Fit x(y) = a*y^2 + b*y + c with NaN/Inf bounds checking."""
    try:
        poly = np.polyfit(y_norm, x_norm, 2)
        if np.isnan(poly).any() or np.isinf(poly).any():
            return None
        # Physical clamping
        poly[0] = float(np.clip(poly[0], -1.5, 1.5))
        poly[1] = float(np.clip(poly[1], -2.0, 2.0))
        poly[2] = float(np.clip(poly[2], -1.0, 1.0))
        return poly
    except Exception:
        return None


def compute_pure_pursuit(poly: np.ndarray, y_L: float, wheelbase: float = 0.14):
    """Compute lookahead steering angle using Ackermann Pure Pursuit."""
    x_L = float(poly[0] * (y_L**2) + poly[1] * y_L + poly[2])
    alpha = math.atan2(x_L, max(0.1, y_L))
    l_d = max(0.15, math.sqrt(x_L**2 + y_L**2))
    curv = (2.0 * math.sin(alpha)) / l_d
    steer_rad = math.atan(curv * wheelbase)
    normalized_error = float(np.clip(steer_rad / 0.61, -1.0, 1.0))
    return x_L, steer_rad, normalized_error


# ──────────────────────────────────────────────────────────────────────────────
# 2. Angle Normalization & Wrap-Around
# ──────────────────────────────────────────────────────────────────────────────

def normalize_angle_deg(deg: float) -> float:
    while deg > 180.0: deg -= 360.0
    while deg < -180.0: deg += 360.0
    return deg


def angle_diff_deg(a: float, b: float) -> float:
    d = a - b
    while d > 180.0: d -= 360.0
    while d < -180.0: d += 360.0
    return d


# ──────────────────────────────────────────────────────────────────────────────
# 3. VFH+ Polar Sector & Gaussian Smoothing
# ──────────────────────────────────────────────────────────────────────────────

def vfh_smooth(sector_density: list) -> list:
    n = len(sector_density)
    smoothed = [0.0] * n
    for k in range(n):
        l = sector_density[k-1] if k > 0 else sector_density[0]
        r = sector_density[k+1] if k < n - 1 else sector_density[-1]
        smoothed[k] = 0.2 * l + 0.6 * sector_density[k] + 0.2 * r
    return smoothed


# ──────────────────────────────────────────────────────────────────────────────
# 4. 3rd-Order Bezier Spline Generation
def eval_bezier(t: float, lat_m: float, len_m: float):
    t = max(0.0, min(1.0, t))
    omt = 1.0 - t
    scale_lat = (4.0 / 3.0) * lat_m
    p0 = (0.0, 0.0)
    p1 = (scale_lat, 0.33 * len_m)
    p2 = (scale_lat, 0.66 * len_m)
    p3 = (0.0, len_m)

    x = (omt**3)*p0[0] + 3*(omt**2)*t*p1[0] + 3*omt*(t**2)*p2[0] + (t**3)*p3[0]
    y = (omt**3)*p0[1] + 3*(omt**2)*t*p1[1] + 3*omt*(t**2)*p2[1] + (t**3)*p3[1]
    dx = 3*(omt**2)*(p1[0] - p0[0]) + 6*omt*t*(p2[0] - p1[0]) + 3*(t**2)*(p3[0] - p2[0])
    dy = 3*(omt**2)*(p1[1] - p0[1]) + 6*omt*t*(p2[1] - p1[1]) + 3*(t**2)*(p3[1] - p2[1])
    tangent_angle = math.atan2(dx, max(0.01, dy))
    return x, y, tangent_angle


class TestPathPlanning(unittest.TestCase):

    def test_straight_line_polynomial(self):
        y = np.array([0.1, 0.3, 0.5, 0.7, 0.9])
        x = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
        poly = fit_lane_polynomial(y, x)
        self.assertIsNotNone(poly)
        self.assertAlmostEqual(poly[0], 0.0, delta=1e-3)
        self.assertAlmostEqual(poly[1], 0.0, delta=1e-3)
        self.assertAlmostEqual(poly[2], 0.0, delta=1e-3)

        _, steer_rad, norm_err = compute_pure_pursuit(poly, y_L=0.5)
        self.assertAlmostEqual(steer_rad, 0.0, delta=1e-3)
        self.assertAlmostEqual(norm_err, 0.0, delta=1e-3)

    def test_curved_lane_polynomial(self):
        y = np.array([0.0, 0.25, 0.5, 0.75, 1.0])
        x = 0.8 * (y ** 2)
        poly = fit_lane_polynomial(y, x)
        self.assertIsNotNone(poly)
        self.assertAlmostEqual(poly[0], 0.8, delta=0.05)

        x_L, steer_rad, norm_err = compute_pure_pursuit(poly, y_L=0.5)
        self.assertGreater(x_L, 0.0)
        self.assertGreater(steer_rad, 0.0)
        self.assertGreater(norm_err, 0.0)

    def test_polynomial_nan_guards(self):
        y = np.array([0.0, np.nan, 0.5])
        x = np.array([0.0, 0.2, 0.4])
        poly = fit_lane_polynomial(y, x)
        self.assertIsNone(poly)

    def test_angle_normalization(self):
        self.assertEqual(normalize_angle_deg(190.0), -170.0)
        self.assertEqual(normalize_angle_deg(-200.0), 160.0)
        self.assertEqual(normalize_angle_deg(360.0), 0.0)
        self.assertEqual(angle_diff_deg(175.0, -175.0), -10.0)
        self.assertEqual(angle_diff_deg(-175.0, 175.0), 10.0)

    def test_vfh_narrow_gap_preservation(self):
        raw = [5.0]*8 + [0.0]*4 + [5.0]*8
        smoothed = vfh_smooth(raw)
        self.assertLess(smoothed[9], 1.0)
        self.assertLess(smoothed[10], 1.0)
        self.assertGreater(smoothed[2], 4.0)
        self.assertGreater(smoothed[17], 4.0)

    def test_vfh_hysteresis_selection(self):
        hyst = 0.15
        left_density = 5.2
        right_density = 5.0
        switch = right_density < left_density * (1.0 - hyst)
        self.assertFalse(switch)

        right_density_clear = 2.0
        switch = right_density_clear < left_density * (1.0 - hyst)
        self.assertTrue(switch)

    def test_bezier_endpoints_and_apex(self):
        lat_offset = 0.18
        spline_len = 0.80

        x0, y0, _ = eval_bezier(0.0, lat_offset, spline_len)
        self.assertAlmostEqual(x0, 0.0, delta=1e-3)
        self.assertAlmostEqual(y0, 0.0, delta=1e-3)

        x1, y1, _ = eval_bezier(1.0, lat_offset, spline_len)
        self.assertAlmostEqual(x1, 0.0, delta=1e-3)
        self.assertAlmostEqual(y1, 0.80, delta=1e-3)

        x_mid, y_mid, tangent = eval_bezier(0.50, lat_offset, spline_len)
        self.assertAlmostEqual(x_mid, 0.18, delta=1e-3)  # apex displacement exactly equals lateral_offset_m (18cm)
        self.assertGreaterEqual(x_mid, 0.15)              # strictly >= 15cm lateral clearance spec
        self.assertTrue(0.20 <= y_mid <= 0.60)
        self.assertIsInstance(tangent, float)

    def test_curvature_inflection_point_detection(self):
        """Verify analytical y* = -b/(2a) finds exact peak curvature within [0, y_L]."""
        # Parabola: x = 1.2*y^2 - 0.6*y + 0.1
        # Inflection point y* = -(-0.6) / (2*1.2) = 0.25 (within typical lookahead 0.0-0.7)
        a, b = 1.2, -0.6
        y_L = 0.7

        def exact_curv(y_pos):
            slope = 2.0 * a * y_pos + b
            return abs(2.0 * a) / ((1.0 + slope**2)**1.5)

        # At y* = 0.25, slope = 2*1.2*0.25 + (-0.6) = 0.0, so kappa = |2a| = 2.4
        y_star = -b / (2.0 * a)
        self.assertAlmostEqual(y_star, 0.25, delta=1e-6)
        self.assertTrue(0.0 <= y_star <= y_L)

        kappa_peak = exact_curv(y_star)
        self.assertAlmostEqual(kappa_peak, abs(2.0 * a), delta=1e-6)  # = 2.4

        # Confirm endpoints have strictly lower curvature than the inflection point
        kappa_0 = exact_curv(0.0)
        kappa_yL = exact_curv(y_L)
        self.assertGreater(kappa_peak, kappa_0)
        self.assertGreater(kappa_peak, kappa_yL)

        # Confirm the 3-point fixed sampling (0, 0.5*y_L, y_L) would miss the true peak
        kappa_mid_sample = exact_curv(0.5 * y_L)
        kappa_3pt_max = max(kappa_0, kappa_mid_sample, kappa_yL)
        self.assertGreater(kappa_peak, kappa_3pt_max,
                           "Analytical y* must find a higher curvature than 3-point fixed sampling")


if __name__ == '__main__':
    unittest.main()
