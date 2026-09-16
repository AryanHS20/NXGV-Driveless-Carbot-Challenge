"""Unit tests for the depth ground-plane mask (pure geometry, no ROS)."""
import math
import unittest

import numpy as np

import ros_stub

ros_stub.install()

from risabot_automode.line_follower_camera import compute_floor_mask


def reference_floor_mask(depth_m, fx, fy, cx, cy, h, tilt_deg, tol):
    """Independent per-pixel reference (explicit loop, same geometry)."""
    tilt = math.radians(tilt_deg)
    ct, st = math.cos(tilt), math.sin(tilt)
    rows, cols = depth_m.shape
    out = np.zeros((rows, cols), dtype=bool)
    for v in range(rows):
        for u in range(cols):
            z = float(depth_m[v, u])
            if not math.isfinite(z) or z <= 0:
                continue
            denom = ((v - cy) / fy) * ct + st
            if denom <= 1e-6:
                continue
            if abs(z - h / denom) <= tol:
                out[v, u] = True
    return out


def make_floor(h=0.085, tilt=0.0, fx=300.0, fy=300.0, cx=160.0, cy=120.0,
               shape=(240, 320), noise=0.0, seed=0):
    """Synthetic z-depth image of a perfect flat floor (+ optional noise)."""
    rng = np.random.default_rng(seed)
    tilt_r = math.radians(tilt)
    ct, st = math.cos(tilt_r), math.sin(tilt_r)
    rows, cols = shape
    depth = np.zeros(shape, dtype=np.float32)
    for v in range(rows):
        denom = ((v - cy) / fy) * ct + st
        if denom > 1e-6:
            depth[v, :] = h / denom
    if noise > 0:
        nz = depth > 0
        depth[nz] += rng.normal(0.0, noise, size=int(nz.sum())).astype(np.float32)
    return depth


class GroundMaskTests(unittest.TestCase):
    def test_flat_floor_passes_below_horizon(self):
        kw = dict(fx=300.0, fy=300.0, cx=160.0, cy=120.0,
                  cam_height_m=0.085, cam_tilt_deg=0.0, tol_m=0.03)
        depth = make_floor(h=kw['cam_height_m'], tilt=kw['cam_tilt_deg'], noise=0.002)
        got = compute_floor_mask(depth, kw['fx'], kw['fy'], kw['cx'], kw['cy'],
                                 kw['cam_height_m'], kw['cam_tilt_deg'], kw['tol_m'])
        want = reference_floor_mask(depth, kw['fx'], kw['fy'], kw['cx'], kw['cy'],
                                    kw['cam_height_m'], kw['cam_tilt_deg'], kw['tol_m'])
        self.assertTrue(np.array_equal(got, want))
        # Below the horizon (v > cy) the exact floor must pass.
        self.assertTrue(got[200, :].all())
        # At/above the horizon nothing can be floor.
        self.assertFalse(got[30, :].any())

    def test_box_above_floor_rejected(self):
        kw = dict(fx=300.0, fy=300.0, cx=160.0, cy=120.0,
                  cam_height_m=0.085, cam_tilt_deg=0.0, tol_m=0.03)
        depth = make_floor()
        depth[150:200, 100:220] *= 0.4  # box standing on the floor
        got = compute_floor_mask(depth, kw['fx'], kw['fy'], kw['cx'], kw['cy'],
                                 kw['cam_height_m'], kw['cam_tilt_deg'], kw['tol_m'])
        self.assertFalse(got[150:200, 100:220].any())
        rest = np.ones_like(got, dtype=bool)
        rest[150:200, 100:220] = False
        rest[:121, :] = False  # horizon band excluded from the claim
        self.assertTrue(got[rest].mean() > 0.99)

    def test_no_return_invalid(self):
        kw = dict(fx=300.0, fy=300.0, cx=160.0, cy=120.0,
                  cam_height_m=0.085, cam_tilt_deg=0.0, tol_m=0.03)
        depth = make_floor()
        depth[180:200, :] = 0.0
        depth[200:210, :] = np.nan
        depth[210:220, :] = np.inf
        got = compute_floor_mask(depth, kw['fx'], kw['fy'], kw['cx'], kw['cy'],
                                 kw['cam_height_m'], kw['cam_tilt_deg'], kw['tol_m'])
        self.assertFalse(got[180:220, :].any())

    def test_tilt_shifts_horizon(self):
        kw = dict(fx=300.0, fy=300.0, cx=160.0, cy=120.0,
                  cam_height_m=0.085, cam_tilt_deg=10.0, tol_m=0.03)
        depth = make_floor(tilt=10.0)
        got = compute_floor_mask(depth, kw['fx'], kw['fy'], kw['cx'], kw['cy'],
                                 kw['cam_height_m'], kw['cam_tilt_deg'], kw['tol_m'])
        # Horizon at v = cy - fy*tan(10deg) ~= 67: above fails, deep below passes.
        self.assertFalse(got[30, :].any())
        self.assertTrue(got[200, :].all())

    def test_tolerance_boundary(self):
        kw = dict(fx=300.0, fy=300.0, cx=160.0, cy=120.0,
                  cam_height_m=0.085, cam_tilt_deg=0.0, tol_m=0.03)
        base = make_floor()
        got_in = compute_floor_mask(base + 0.01, kw['fx'], kw['fy'], kw['cx'], kw['cy'],
                                    kw['cam_height_m'], kw['cam_tilt_deg'], kw['tol_m'])
        got_out = compute_floor_mask(base + 0.10, kw['fx'], kw['fy'], kw['cx'], kw['cy'],
                                     kw['cam_height_m'], kw['cam_tilt_deg'], kw['tol_m'])
        self.assertTrue(got_in[200, :].all())
        self.assertFalse(got_out[200, :].any())

    def test_invalid_inputs_raise(self):
        kw = dict(fx=300.0, fy=300.0, cx=160.0, cy=120.0,
                  cam_height_m=0.085, cam_tilt_deg=0.0, tol_m=0.03)
        with self.assertRaises(ValueError):
            compute_floor_mask(np.zeros((10, 10, 3)), **kw)
        with self.assertRaises(ValueError):
            compute_floor_mask(np.zeros((10, 10)), fx=0.0, fy=300.0, cx=5.0,
                               cy=5.0, cam_height_m=0.085, cam_tilt_deg=0.0,
                               tol_m=0.03)


if __name__ == '__main__':
    unittest.main()
