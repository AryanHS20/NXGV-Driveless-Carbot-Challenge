"""Compatibility checks for the user's Setup Carbot planning algorithms.

These tests use no ROS, robot processes, display windows, or motor commands.
The console/backend integration is separate from these vendored algorithms.
"""
import copy
import hashlib
import math
import sys
from pathlib import Path

import numpy as np
import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/risabot_automode'))
from risabot_automode.dashboard_panels import map_builder as mb
from risabot_automode.dashboard_panels import mission_planner as mp


@pytest.fixture(scope='module')
def road():
    return mp.Road(copy.deepcopy(mb.TEMPLATE))


def test_saved_map_rebuilds_identical_geometry_and_exact_fingerprint(tmp_path):
    path = tmp_path / 'track_map.yaml'
    mb.save_yaml(path, mb.to_yaml_dict(mb.TEMPLATE, (1.2, -0.4, 0.1), {}))
    template, fingerprint = mp.load_map(path)
    assert fingerprint == hashlib.sha1(path.read_bytes()).hexdigest()
    original, rebuilt = mb.centrelines(mb.TEMPLATE), mb.centrelines(template)
    assert original.keys() == rebuilt.keys()
    for section in original:
        # YAML quantization can change ceil(length / step) by one sample.
        # Compare the curves at the same normalized positions, not array size.
        positions = np.linspace(0, 1, 100)
        def resample(points):
            return np.column_stack([
                np.interp(positions, np.linspace(0, 1, len(points)), points[:, axis])
                for axis in (0, 1)
            ])
        assert np.allclose(resample(original[section]), resample(rebuilt[section]), atol=0.001)
    path.write_bytes(path.read_bytes() + b'\n# changed map\n')
    assert mp.load_map(path)[1] != fingerprint


def test_flip_preserves_body_location_and_reverses_heading(road):
    pose = mp.default_pose(road, 0)
    flipped = mp.flip(pose)
    assert np.allclose(mp.body_corners(pose).mean(0), mp.body_corners(flipped).mean(0))
    assert abs(abs(mp.wrap(flipped[2] - pose[2])) - math.pi) < 1e-8
    assert np.allclose(mp.flip(flipped)[:2], pose[:2])


def test_off_road_pose_is_explicitly_rejected(road):
    fits, clearance = mp.fit_check(road, (100, 100, 0))
    assert not fits
    assert clearance < 0


def test_lane_change_handle_keeps_linked_geometry():
    template = copy.deepcopy(mb.TEMPLATE)
    old = np.array(template['lane_change']['centre'])
    old_handle = np.array(template['lane_change']['length_handle'])
    target = old + [0.05, 0.02]
    mb.move_handle(template, 'lane_change.centre', 'lc_centre', target)
    assert np.allclose(template['lane_change']['length_handle'], old_handle + [0.05, 0.02])


def test_partial_mission_does_not_claim_success(road):
    poses = mp.default_poses(road)
    document = mp.mission_dict('track_map.yaml', 'map-fingerprint', poses, road, [None] * 3)
    assert document['version'] == 2
    assert document['map']['sha1'] == 'map-fingerprint'
    assert document['frame'] == 'track'
    assert len(document['poses']) == 4
    assert all(not leg['ok'] for leg in document['legs'])
    assert all('pieces' not in leg for leg in document['legs'])
    assert yaml.safe_load(yaml.safe_dump(document)) == document


def test_venue_transform_roundtrip():
    points = np.array([[0.0, 0.0], [1.0, 2.0], [-0.5, 1.5]])
    transform = (1.2, -0.4, 0.3)
    assert np.allclose(mb.apply(mb.invert(transform), mb.apply(transform, points)), points)
