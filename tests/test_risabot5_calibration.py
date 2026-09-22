"""Synthetic geometry tests; no physical calibration claims and no robot access."""
import importlib.util
import json
from pathlib import Path
import tempfile
import sys

sys.dont_write_bytecode = True
import cv2
import numpy as np
import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('r5_calibration', ROOT / 'tools/calibrate_risabot5_camera.py')
c = importlib.util.module_from_spec(spec)
spec.loader.exec_module(c)
from carbot_perception.camera_model import Mount
from risabot_v4_experimental.bev_core import CalibrationError, load_profiles


def synthetic():
    # Deliberately synthetic values, unrelated to car 1 or car 2 measurements.
    board = dict(inner_corners=[6, 4], square_m=.043, centre_m=[.83, .04],
                 yaw_deg=83., ordering='flip_cols')
    ground = c.board_ground(board)
    k = np.array([[530., 0, 319.5], [0, 535., 239.5], [0, 0, 1.]])
    d = np.array([-.06, .01, .0004, -.0008, 0.])
    mount = Mount.from_yaml(dict(x_m=.17, y_m=.015, z_m=.18,
                                yaw_deg=1., pitch_down_deg=12., roll_deg=.7))
    rotation = mount.r_base_opt().T
    rvec, _ = cv2.Rodrigues(rotation)
    tvec = -rotation @ mount.position

    def project(g):
        xyz = np.column_stack((g, np.zeros(len(g))))
        p, _ = cv2.projectPoints(xyz, rvec, tvec, k, d)
        return p.reshape(-1, 2)

    vg = np.array([[.64, .19], [1.13, -.18], [1.05, .24]])
    return board, ground, k, d, project(ground), project(vg), vg


def info(k, d):
    return dict(width=640, height=480, k=k.ravel().tolist(), d=d.tolist(),
                distortion_model='plumb_bob', binning_x=0, binning_y=0,
                roi=dict(x_offset=0, y_offset=0, width=0, height=0, do_rectify=False),
                r=np.eye(3).ravel().tolist(), p=np.column_stack((k, np.zeros(3))).ravel().tolist())


def test_pixel_centres_and_intrinsics_scale_together():
    t = c.pixel_transform([640, 480], [320, 240])
    np.testing.assert_allclose(c.transform_points([[0, 0], [319.5, 239.5]], t),
                               [[-.25, -.25], [159.5, 119.5]])
    _, _, k, _, _, _, _ = synthetic()
    rays = np.array([[.1, -.2, 1], [-.2, .1, 1]])
    pixels = (rays @ k.T)[:, :2]
    np.testing.assert_allclose(c.transform_points(pixels, t), (rays @ (t @ k).T)[:, :2])
    np.testing.assert_allclose(c.transform_points(c.transform_points(pixels, t), np.linalg.inv(t)), pixels)


def test_distorted_raw_points_recover_metric_ground_and_resize_invariance():
    _, ground, k, d, raw, vp, vg = synthetic()
    h, result = c.fit_geometry(raw, ground, k, d, [640, 480], vp, vg)
    assert result['held_out_board']['max_m'] < 1e-5
    assert result['independent_markers']['max_m'] < 1e-5
    t = c.pixel_transform([640, 480], [320, 240])
    _, scaled = c.fit_geometry(c.transform_points(raw, t), ground, t @ k, d,
                               [320, 240], c.transform_points(vp, t), vg)
    assert scaled['independent_markers']['max_m'] < 1e-5
    # Distortion correction is material, not a no-op fixture.
    naive = cv2.getPerspectiveTransform(raw[c.OUTER].astype(np.float32), ground[c.OUTER].astype(np.float32))
    assert np.max(np.linalg.norm(c.transform_points(vp, naive) - vg, axis=1)) > 1e-4


def test_board_origin_orientation_and_inner_span():
    b = dict(inner_corners=[6, 4], square_m=.04, centre_m=[.9, .1], yaw_deg=90, ordering='flip_cols')
    g = c.board_ground(b)
    np.testing.assert_allclose(g[c.OUTER], [[.96, .20], [.96, 0], [.84, 0], [.84, .20]], atol=1e-12)
    np.testing.assert_allclose(g.mean(axis=0), [.9, .1])


def test_wrong_order_is_exposed_by_independent_markers():
    board, ground, k, d, raw, vp, vg = synthetic()
    wrong = dict(board, ordering='identity')
    _, result = c.fit_geometry(raw, c.board_ground(wrong), k, d, [640, 480], vp, vg)
    assert result['held_out_board']['max_m'] < 1e-5  # symmetric board alone cannot disambiguate
    assert result['independent_markers']['max_m'] > .02


def test_held_out_error_is_not_training_error():
    _, ground, k, d, raw, vp, vg = synthetic()
    raw[8] += [0, 12]
    _, result = c.fit_geometry(raw, ground, k, d, [640, 480], vp, vg)
    assert result['construction']['max_m'] < 1e-5
    assert result['held_out_board']['max_m'] > .02


@pytest.mark.parametrize('mutation', ['size', 'fisheye', 'roi', 'binning', 'nan', 'zero_focal', 'missing_d'])
def test_invalid_intrinsics_fail_closed(mutation):
    _, _, k, d, _, _, _ = synthetic()
    data = info(k, d)
    if mutation == 'size': data['width'] = 320
    if mutation == 'fisheye': data['distortion_model'] = 'equidistant'
    if mutation == 'roi': data['roi']['x_offset'] = 10
    if mutation == 'binning': data['binning_x'] = 2
    if mutation == 'nan': data['k'][0] = float('nan')
    if mutation == 'zero_focal': data['k'][0] = 0
    if mutation == 'missing_d': data['d'] = []
    with pytest.raises(ValueError): c.camera_info(data, [640, 480])


@pytest.mark.parametrize('mutation', ['duplicate', 'crossed', 'out_of_bounds', 'no_validation', 'reused_corner'])
def test_bad_correspondences_rejected(mutation):
    _, ground, k, d, raw, vp, vg = synthetic()
    if mutation == 'duplicate': raw[c.OUTER[1]] = raw[c.OUTER[0]]
    if mutation == 'crossed': raw[[5, 23]] = raw[[23, 5]]
    if mutation == 'out_of_bounds': raw[2, 0] = 640
    if mutation == 'no_validation': vp, vg = vp[:1], vg[:1]
    if mutation == 'reused_corner': vp[0] = raw[0]
    with pytest.raises(ValueError): c.fit_geometry(raw, ground, k, d, [640, 480], vp, vg)


def test_reviewed_front_profile_and_unmeasured_secondary():
    profiles = load_profiles(str(c.PROFILE))
    front = profiles['primary']
    assert front.calibrated
    assert front.resolution == (320, 240)
    frame = cv2.imread(str(c.EVIDENCE / 'front_session/aligned_current320/raw.png'))
    bev, coverage = c.warp_to_bev(frame, front)
    assert bev.shape[:2] == coverage.shape == (351, 361)
    assert coverage.any()
    secondary = profiles['secondary']
    assert not secondary.calibrated and secondary.camera_matrix.size == 0
    with pytest.raises(CalibrationError, match='not calibrated'):
        c.build_homography(secondary)
    disabled = c.profile_from_mapping('primary_disabled', {
        'calibrated': False, 'resolution': [320, 240],
        'ground_bounds_m': {'forward': [.05, 1.8], 'left': [-.9, .9]},
        'pixels_per_meter': 200.,
    })
    with pytest.raises(CalibrationError, match='not calibrated'):
        c.warp_to_bev(frame, disabled)


def test_end_to_end_candidate_stays_locked_and_tracks_failures():
    board, ground, k, d, raw, vp, vg = synthetic()
    with tempfile.TemporaryDirectory(prefix='synthetic_test_', dir=c.EVIDENCE) as folder:
        p = Path(folder)
        cv2.imwrite(str(p / 'frame.png'), np.zeros((480, 640, 3), np.uint8))
        c.save_json(p / 'info.json', info(k, d))
        digest = c.sha256(p / 'frame.png')
        c.save_json(p / 'detections.json', {'images': [dict(sha256=digest, found=True,
                    resolution=[640, 480], corners_raw_px=raw.tolist())]})
        session = dict(vehicle='Risabot 5 (car 2)', profile='primary',
            origin='rear_axle_midpoint_on_ground', image_space='raw_unrotated', image='frame.png',
            image_sha256=digest, camera_info='info.json', detections='detections.json',
            capture_provenance='SYNTHETIC TEST', intrinsics_provenance='SYNTHETIC TEST',
            measurement_provenance='SYNTHETIC TEST', ordering_evidence='SYNTHETIC TEST',
            corners_reviewed=True, board=board, validation=dict(raw_pixels=vp.tolist(), ground_m=vg.tolist()),
            target_resolution=[320, 240], software_resize_only=True, resize_provenance='SYNTHETIC TEST',
            ground_bounds_m=dict(forward=[.05, 1.8], left=[-.9, .9]), pixels_per_meter=200.)
        c.save_json(p / 'session.json', session)
        result = c.fit_session(p / 'session.json', p / 'pass')
        assert result['geometry_pass'] and result['runtime_agreement_max_m'] < 1e-4
        assert not result['calibrated']
        candidate = yaml.safe_load((p / 'pass/candidate_camera_profiles.yaml').read_text())
        assert not candidate['profiles']['primary']['calibrated']
        pending = dict(session, validation={'raw_pixels': [], 'ground_m': []})
        c.save_json(p / 'pending.json', pending)
        preliminary = c.fit_session(p / 'pending.json', p / 'board_only', allow_board_only=True)
        assert preliminary['checks']['held_out_board']['max_m'] < 1e-5
        assert preliminary['checks']['independent_markers'] is None
        assert not preliminary['geometry_pass'] and not preliminary['calibrated']
        with pytest.raises(ValueError, match='at least two'):
            c.fit_session(p / 'pending.json', p / 'strict')
        session['validation']['ground_m'][0][0] += .05
        c.save_json(p / 'session.json', session)
        assert not c.fit_session(p / 'session.json', p / 'fail')['geometry_pass']
        session['software_resize_only'] = False
        c.save_json(p / 'session.json', session)
        with pytest.raises(ValueError, match='software resize'):
            c.fit_session(p / 'session.json', p / 'mode_change')
        session['image_sha256'] = 'wrong'
        c.save_json(p / 'session.json', session)
        with pytest.raises(ValueError, match='hash mismatch'):
            c.fit_session(p / 'session.json', p / 'bad_hash')


def test_recovery_coordinates_inverse_crop_and_scale(monkeypatch):
    with tempfile.TemporaryDirectory(prefix='synthetic_test_', dir=c.EVIDENCE) as folder:
        p = Path(folder) / 'image.png'
        cv2.imwrite(str(p), np.zeros((240, 320, 3), np.uint8))
        raw = np.array([[50 + col * 20, 130 + row * 5] for row in range(4) for col in range(6)], float)
        t = c.pixel_transform([240, 41], [240, 164])
        processed = c.transform_points(raw - [35, 125], t)
        monkeypatch.setattr(c, 'detect_chessboard', lambda *args: processed)
        record, _ = c.detect_image(p, [35, 125, 240, 41], [1, 4])
        np.testing.assert_allclose(record['corners_raw_px'], raw)


def test_outputs_cannot_escape_evidence():
    with pytest.raises(ValueError, match='below calibration_risabot5'):
        c.output_dir(ROOT / 'forbidden')
