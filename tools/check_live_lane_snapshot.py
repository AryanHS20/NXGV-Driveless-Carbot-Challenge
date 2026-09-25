"""Read-only regression/benchmark; images are never runtime control inputs."""
import argparse
import json
import time
from pathlib import Path

import cv2

from risabot_v4_control.track_test_config import track_test_overrides
from risabot_v4_experimental.bev_core import load_profiles
from risabot_v4_experimental.trajectory_core import (
    TrajectoryConfig, VehicleGeometry, near_field_bootstrap_from_corridor,
    steering_reference_from_corridor,
)
from risabot_v4_experimental.live_lane_core import select_live_lane_arc


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('directory', type=Path)
    parser.add_argument('--profile', required=True)
    args = parser.parse_args()
    status = json.loads((args.directory / 'dashboard_status.json').read_text())
    rows = status['v4_status']['road']['corridor']['primary']
    settings = track_test_overrides('risabot1')['v4_trajectory_shadow']
    config = TrajectoryConfig(**{k: v for k, v in settings.items()
                                 if k in TrajectoryConfig.__dataclass_fields__})
    geometry = VehicleGeometry(
        length_m=settings['vehicle_length_m'], width_m=settings['vehicle_width_m'],
        wheelbase_m=settings['wheelbase_m'],
        minimum_turn_radius_m=settings['minimum_turn_radius_m'],
        footprint_padding_m=settings['footprint_padding_m'],
    )
    near = near_field_bootstrap_from_corridor(
        [(r['forward_m'], r['left_m'], r['width_m']) for r in rows], geometry,
        config.near_field_max_gap_m, config.near_field_settle_m,
    )
    reference = steering_reference_from_corridor(
        [r for r in rows if r['forward_m'] >= near.forward_m],
        config.expected_lane_width_m,
    )
    mask = cv2.imread(str(args.directory / 'v4_fused.png'), 0)
    profile = load_profiles(args.profile)['primary']
    start = time.perf_counter()
    selected, candidates = select_live_lane_arc(
        reference, mask, profile, geometry, config, near_field=near,
    )
    print(json.dumps({'elapsed_ms': 1000 * (time.perf_counter() - start),
                      'reference_rows': len(reference), 'selected': selected,
                      'valid_arcs': sum(c['valid'] for c in candidates),
                      'total_arcs': len(candidates)}, indent=2))


if __name__ == '__main__':
    main()
