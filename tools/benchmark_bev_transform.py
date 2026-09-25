#!/usr/bin/env python3
"""Compare the installed BEV transform with a proposed cached implementation."""

import importlib.util
import sys
import time

import numpy as np

from risabot_v4_experimental.bev_core import load_profiles, warp_to_bev


def main():
    profile_path, proposed_path = sys.argv[1:3]
    spec = importlib.util.spec_from_file_location('proposed_bev_core', proposed_path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    profile = load_profiles(profile_path)['primary']
    width, height = profile.resolution
    frame = np.random.default_rng(23).integers(
        0, 256, (height, width, 3), dtype=np.uint8,
    )
    transformer = module.BevTransformer(profile)
    reference = warp_to_bev(frame, profile)
    proposed = transformer.warp(frame)
    for old, new in zip(reference, proposed):
        np.testing.assert_array_equal(old, new)
    for _ in range(3):
        warp_to_bev(frame, profile)
        transformer.warp(frame)
    for name, function in [('installed', lambda: warp_to_bev(frame, profile)),
                           ('cached', lambda: transformer.warp(frame))]:
        start = time.perf_counter()
        for _ in range(40):
            function()
        print(f'{name}: {(time.perf_counter() - start) / 40 * 1000:.1f} ms/frame')


if __name__ == '__main__':
    main()
