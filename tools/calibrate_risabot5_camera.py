#!/usr/bin/env python3
"""Offline-only R5 evidence inspection and measured ground-plane candidate fitting.

Never activates a profile or accesses ROS/network/robot processes. See
RISABOT5_CALIBRATION.md and calibration_risabot5/session.template.json.
"""
import argparse
from datetime import datetime, timezone
import glob
import hashlib
import json
from pathlib import Path
import sys

# Imported checkout modules must not create files outside the evidence directory.
sys.dont_write_bytecode = True
import cv2
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
EVIDENCE = ROOT / 'calibration_risabot5'
PROFILE = ROOT / 'src/risabot_v4_experimental/config/risabot5_camera_profiles.yaml'
sys.path.insert(0, str(ROOT / 'carbot_calibration_2-4/src/carbot_perception'))
sys.path.insert(0, str(ROOT / 'src/risabot_v4_experimental'))
from carbot_perception.calib_core import FloorBoard, detect_chessboard
from risabot_v4_experimental.bev_core import (
    bev_to_metric, build_homography, profile_from_mapping, warp_to_bev,
)

OUTER = np.array([0, 5, 23, 18])
HELD_OUT = np.array([i for i in range(24) if i not in OUTER])


def require(condition, message):
    if not condition:
        raise ValueError(message)


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def output_dir(path, evidence_root=EVIDENCE):
    path = Path(path).resolve()
    evidence_root = Path(evidence_root).resolve()
    require(path.is_relative_to(evidence_root),
            f'outputs must be below {evidence_root.name}/')
    require(not path.exists(), 'output directory already exists; use a fresh run name')
    path.mkdir(parents=True)
    return path


def save_json(path, value):
    Path(path).write_text(json.dumps(value, indent=2, allow_nan=False) + '\n', encoding='utf-8')


def finite(value, shape, label):
    result = np.asarray(value, dtype=np.float64)
    require(result.shape == shape and np.isfinite(result).all(),
            f'{label} must be finite with shape {shape}')
    return result


def pixel_transform(source_size, target_size):
    """OpenCV resize pixel centres: p' = (p + 0.5) * scale - 0.5.

    This describes software resizing of the SAME image, not a sensor mode change.
    """
    source = finite(source_size, (2,), 'source size')
    target = finite(target_size, (2,), 'target size')
    require(np.all(source > 0) and np.all(target > 0), 'sizes must be positive')
    sx, sy = target / source
    return np.array([[sx, 0, (sx - 1) / 2], [0, sy, (sy - 1) / 2], [0, 0, 1.]])


def transform_points(points, matrix):
    points = np.asarray(points, np.float64)
    homogeneous = np.column_stack((points, np.ones(len(points)))) @ matrix.T
    require(np.isfinite(homogeneous).all() and np.all(np.abs(homogeneous[:, 2]) > 1e-10),
            'projection is non-finite or at the horizon')
    return homogeneous[:, :2] / homogeneous[:, 2, None]


def check_points(points, size, count, label):
    points = finite(points, (count, 2), label)
    require(np.all(points >= 0) and np.all(points < np.asarray(size)),
            f'{label} outside the raw image')
    return points


def camera_info(document, size):
    """Accept a saved full ROS CameraInfo mapping; reject unsupported processing."""
    require([document['width'], document['height']] == list(size),
            'CameraInfo resolution must match the raw capture; no automatic K scaling')
    model = document.get('distortion_model')
    require(model in ('plumb_bob', 'rational_polynomial'),
            'bev_core uses pinhole undistortion; equidistant/fisheye is unsupported')
    k = finite(document['k'], (9,), 'CameraInfo.k').reshape(3, 3)
    d = np.asarray(document['d'], np.float64)
    require(d.ndim == 1 and len(d) in ((4, 5) if model == 'plumb_bob' else (8,)),
            'distortion coefficient count does not match the supported ROS model')
    require(np.isfinite(d).all(), 'non-finite distortion')
    require(k[0, 0] > 0 and k[1, 1] > 0 and np.allclose(k[2], [0, 0, 1])
            and k[0, 1] == 0 and k[1, 0] == 0, 'invalid pinhole camera matrix')
    require(0 <= k[0, 2] < size[0] and 0 <= k[1, 2] < size[1],
            'principal point outside image; investigate crop/mode')
    require(document['binning_x'] in (0, 1) and document['binning_y'] in (0, 1),
            'binned CameraInfo needs explicit effective-intrinsic analysis')
    roi = document['roi']
    require(roi['x_offset'] == 0 and roi['y_offset'] == 0
            and roi['width'] in (0, size[0]) and roi['height'] in (0, size[1])
            and roi['do_rectify'] is False, 'cropped/rectified ROI is unsupported')
    finite(document['r'], (9,), 'CameraInfo.r')
    finite(document['p'], (12,), 'CameraInfo.p')
    return k, d


def board_ground(board, name='measured_floor_board'):
    require(board['inner_corners'] == [6, 4], 'floor target must have 6x4 INNER corners')
    square = float(board['square_m'])
    require(np.isfinite(square) and square > 0, 'measured square_m must be positive')
    finite(board['centre_m'], (2,), 'measured board centre')
    require(np.isfinite(float(board['yaw_deg'])), 'measured board yaw must be finite')
    floor = FloorBoard.from_yaml(dict(board, name=name, roles=['front']))
    choices = dict(floor.orderings())
    require(board['ordering'] in choices, 'unknown board ordering')
    return choices[board['ordering']][:, :2]


def convex_quad(points, label):
    contour = np.asarray(points, np.float32).reshape(-1, 1, 2)
    require(cv2.isContourConvex(contour) and abs(cv2.contourArea(contour)) > 1e-8,
            f'{label} must be a non-crossed, non-degenerate convex perimeter')


def fit_geometry(raw_corners, ground, k, d, size, validation_px, validation_ground,
                 allow_board_only=False):
    raw = check_points(raw_corners, size, 24, 'raw corners')
    ground = finite(ground, (24, 2), 'ground grid')
    convex_quad(raw[OUTER], 'image quad')
    convex_quad(ground[OUTER], 'ground quad')
    undistorted = cv2.undistortPoints(raw.reshape(-1, 1, 2), k, d, P=k).reshape(-1, 2)
    convex_quad(undistorted[OUTER], 'undistorted image quad')
    h = cv2.getPerspectiveTransform(undistorted[OUTER].astype(np.float32),
                                    ground[OUTER].astype(np.float32))
    require(np.isfinite(h).all() and np.linalg.matrix_rank(h) == 3,
            'singular ground homography')
    # Require one side of the horizon over the calibration board.
    denom = np.column_stack((undistorted, np.ones(24))) @ h[2]
    require(np.all(denom > 1e-10) or np.all(denom < -1e-10),
            'board crosses the homography horizon')
    n = len(validation_px)
    board_prediction = transform_points(undistorted[HELD_OUT], h)
    board_errors = np.linalg.norm(board_prediction - ground[HELD_OUT], axis=1)
    if allow_board_only and n == 0:
        require(len(validation_ground) == 0, 'validation pixels and ground must be paired')
        construction = transform_points(undistorted[OUTER], h)
        construction_errors = np.linalg.norm(construction - ground[OUTER], axis=1)
        return h, {
            'construction': {'rms_m': float(np.sqrt(np.mean(construction_errors ** 2))),
                             'max_m': float(construction_errors.max()),
                             'per_point_m': construction_errors.tolist(),
                             'predicted_ground_m': construction.tolist()},
            'held_out_board': {'rms_m': float(np.sqrt(np.mean(board_errors ** 2))),
                              'max_m': float(board_errors.max()),
                              'per_point_m': board_errors.tolist(),
                              'predicted_ground_m': board_prediction.tolist()},
            'independent_markers': None,
        }
    require(n >= 2, 'at least two independently measured validation markers required')
    vp = check_points(validation_px, size, n, 'validation pixels')
    vg = finite(validation_ground, (n, 2), 'validation ground')
    require(np.min(np.linalg.norm(vp[:, None] - raw[None], axis=2)) > 1,
            'independent markers must not reuse board corners')
    require(len(np.unique(vp, axis=0)) == n and len(np.unique(vg, axis=0)) == n,
            'duplicate independent validation markers')
    vu = cv2.undistortPoints(vp.reshape(-1, 1, 2), k, d, P=k).reshape(-1, 2)

    def errors(predicted, expected):
        e = np.linalg.norm(predicted - expected, axis=1)
        return {'rms_m': float(np.sqrt(np.mean(e ** 2))), 'max_m': float(e.max()),
                'per_point_m': e.tolist(), 'predicted_ground_m': predicted.tolist()}

    return h, {
        'construction': errors(transform_points(undistorted[OUTER], h), ground[OUTER]),
        'held_out_board': errors(transform_points(undistorted[HELD_OUT], h), ground[HELD_OUT]),
        'independent_markers': errors(transform_points(vu, h), vg),
    }


def detect_image(path, roi=None, scale=(1., 1.)):
    image = cv2.imread(str(path))
    require(image is not None, f'cannot read {path}')
    height, width = image.shape[:2]
    x, y, w, h = (0, 0, width, height) if roi is None else tuple(roi)
    require(x >= 0 and y >= 0 and w > 0 and h > 0 and x + w <= width and y + h <= height,
            'ROI must be wholly inside image')
    scales = finite(scale, (2,), 'diagnostic scale')
    require(np.all(scales > 0) and np.all(scales <= 8), 'diagnostic scale must be in (0, 8]')
    sw, sh = int(round(w * scales[0])), int(round(h * scales[1]))
    require(sw >= 2 and sh >= 2, 'scaled ROI too small')
    crop = image[y:y+h, x:x+w]
    processed = crop if (sw, sh) == (w, h) else cv2.resize(crop, (sw, sh), interpolation=cv2.INTER_CUBIC)
    corners = detect_chessboard(processed, 6, 4)
    record = {'image': str(Path(path).resolve()), 'sha256': sha256(path),
              'resolution': [width, height], 'roi_xywh': [x, y, w, h],
              'processed_resolution': [sw, sh], 'found': corners is not None,
              'method': 'bundled detect_chessboard(fast=False): SB then classic',
              'diagnostic_resampling': roi is not None or tuple(scale) != (1., 1.),
              'corners_raw_px': None}
    if corners is not None:
        corners = transform_points(corners, pixel_transform((sw, sh), (w, h))) + [x, y]
        record['corners_raw_px'] = corners.tolist()
        spacing = np.linalg.norm(np.diff(corners.reshape(4, 6, 2), axis=0), axis=2)
        record['row_spacing_px_min_median_max'] = [float(spacing.min()), float(np.median(spacing)), float(spacing.max())]
        cv2.drawChessboardCorners(image, (6, 4), corners.reshape(-1, 1, 2).astype(np.float32), True)
        for i in OUTER:
            cv2.putText(image, str(i), tuple(np.rint(corners[i]).astype(int)),
                        cv2.FONT_HERSHEY_SIMPLEX, .4, (0, 0, 255), 1, cv2.LINE_AA)
    return record, image


def inspect(args, evidence_root=EVIDENCE):
    paths = sorted({Path(p).resolve() for pattern in args.images for p in glob.glob(pattern)})
    require(paths, 'no images matched')
    out = output_dir(args.output, evidence_root)
    records = []
    for index, path in enumerate(paths):
        record, image = detect_image(path, args.roi, args.scale)
        if record['found']:
            name = f'{index:02d}_{path.stem}_corners.png'
            require(cv2.imwrite(str(out / name), image), 'could not write annotation')
            record['annotation'] = name
        records.append(record)
        print(f'{path.name}: {record["resolution"]}, found={record["found"]}', flush=True)
    save_json(out / 'detections.json', {'opencv': cv2.__version__, 'numpy': np.__version__,
              'detector_sha256': sha256(ROOT / 'carbot_calibration_2-4/src/carbot_perception/carbot_perception/calib_core.py'),
              'images': records})


def fit_session(manifest_path, destination, allow_board_only=False,
                vehicle='Risabot 5 (car 2)', profile_path=PROFILE,
                evidence_root=EVIDENCE):
    manifest_path = Path(manifest_path).resolve()
    session = json.loads(manifest_path.read_text(encoding='utf-8'))
    require(session['vehicle'] == vehicle, 'wrong vehicle identity')
    require(session['origin'] == 'rear_axle_midpoint_on_ground', 'wrong ground origin')
    require(session['image_space'] == 'raw_unrotated', 'only raw unrotated captures are supported')
    for key in ('capture_provenance', 'intrinsics_provenance', 'measurement_provenance', 'ordering_evidence'):
        require(isinstance(session.get(key), str) and bool(session[key].strip()), f'missing {key}')

    def relative(value):
        return (manifest_path.parent / value).resolve()

    image_path = relative(session['image'])
    image = cv2.imread(str(image_path))
    require(image is not None, 'missing capture')
    require(sha256(image_path) == session['image_sha256'], 'capture hash mismatch')
    size = image.shape[1::-1]
    info_path = relative(session['camera_info'])
    info = yaml.safe_load(info_path.read_text(encoding='utf-8'))
    k, d = camera_info(info, size)
    detection_path = relative(session['detections'])
    detections = json.loads(detection_path.read_text(encoding='utf-8'))
    match = [r for r in detections['images'] if r['sha256'] == session['image_sha256']]
    require(len(match) == 1 and match[0]['found'], 'need one successful detection for this capture')
    record = match[0]
    require(record['resolution'] == list(size), 'detection resolution mismatch')
    require(session.get('corners_reviewed') is True, 'review numbered raw-pixel overlay first')
    ground = board_ground(session['board'], name=vehicle.replace(' ', '_').lower())
    vp = session['validation']['raw_pixels']
    vg = session['validation']['ground_m']
    h, checks = fit_geometry(record['corners_raw_px'], ground, k, d, size, vp, vg,
                            allow_board_only=allow_board_only)
    target = session['target_resolution']
    require(len(target) == 2 and all(isinstance(x, int) and x > 0 for x in target),
            'target_resolution must be two positive integers')
    if list(size) != target:
        require(session.get('resize_provenance') and session.get('software_resize_only') is True,
                'resolution conversion requires documented software resize of the SAME raw image')
    resize = pixel_transform(size, target)
    points = transform_points(np.asarray(record['corners_raw_px'])[OUTER], resize)
    raw = yaml.safe_load(Path(profile_path).read_text(encoding='utf-8'))
    name = session['profile']
    require(name in ('primary', 'secondary'), 'profile must be primary or secondary')
    candidate = raw['profiles'][name]
    candidate.update(resolution=target, camera_matrix=(resize @ k).tolist(),
                     distortion_coefficients=d.tolist(), source_points_px=points.tolist(),
                     ground_points_m=ground[OUTER].tolist())
    candidate['ground_bounds_m'] = session['ground_bounds_m']
    candidate['pixels_per_meter'] = session['pixels_per_meter']
    # In-memory true flag only to exercise the exact runtime pipeline.
    runtime = profile_from_mapping(name, dict(candidate, calibrated=True))
    frame = image if list(size) == target else cv2.resize(image, tuple(target), interpolation=cv2.INTER_AREA)
    bev, coverage = warp_to_bev(frame, runtime)
    raw_check = np.vstack((record['corners_raw_px'], np.asarray(vp).reshape(-1, 2)))
    target_points = transform_points(raw_check, resize)
    undistorted = cv2.undistortPoints(target_points.reshape(-1, 1, 2), resize @ k, d, P=resize @ k).reshape(-1, 2)
    runtime_ground = bev_to_metric(runtime, transform_points(undistorted, build_homography(runtime)))
    original_undistorted = cv2.undistortPoints(raw_check.reshape(-1, 1, 2), k, d, P=k).reshape(-1, 2)
    runtime_delta = float(np.max(np.linalg.norm(runtime_ground - transform_points(original_undistorted, h), axis=1)))
    require(runtime_delta < 1e-4, 'runtime BEV and offline ground geometry disagree')
    independent = checks['independent_markers']
    passes = independent is not None and checks['held_out_board']['max_m'] <= .02 and independent['max_m'] <= .02
    out = output_dir(destination, evidence_root)
    for filename, data in [('bev.png', bev), ('coverage.png', coverage)]:
        require(cv2.imwrite(str(out / filename), data), f'could not save {filename}')
    candidate['calibrated'] = False
    candidate['calibration_evidence'] = str((out / 'fit_report.json').relative_to(ROOT))
    candidate['calibration_status'] = 'candidate_requires_visual_and_provenance_review' if passes else 'validation_failed'
    if independent is None:
        candidate['calibration_status'] = 'board_fit_only_independent_validation_pending'
    (out / 'candidate_camera_profiles.yaml').write_text(yaml.safe_dump(raw, sort_keys=False), encoding='utf-8')
    report = {'created_utc': datetime.now(timezone.utc).isoformat(), 'opencv': cv2.__version__,
              'vehicle': session['vehicle'], 'calibrated': False, 'geometry_pass': passes,
              'maximum_error_threshold_m': .02, 'source_resolution': list(size),
              'target_resolution': target, 'construction_indices': OUTER.tolist(),
              'held_out_indices': HELD_OUT.tolist(), 'checks': checks,
              'runtime_agreement_max_m': runtime_delta, 'homography_undistorted_to_ground': h.tolist(),
              'inputs': {str(p): sha256(p) for p in (manifest_path, image_path, info_path, detection_path, Path(__file__))},
              'session': session, 'detection': record,
              'remaining_review': ['identity and intrinsic source', 'physical corner orientation',
                                   'straight-line undistortion', 'useful-area coverage and extrapolation']}
    if independent is None:
        report['remaining_review'].append('independently measured validation markers')
    save_json(out / 'fit_report.json', report)
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command', required=True)
    p = sub.add_parser('inspect', help='audit dimensions/hashes and detect 6x4 inner corners')
    p.add_argument('images', nargs='+', help='paths or quoted glob patterns')
    p.add_argument('--roi', nargs=4, type=int, metavar=('X', 'Y', 'WIDTH', 'HEIGHT'))
    p.add_argument('--scale', nargs=2, type=float, default=[1., 1.], metavar=('SX', 'SY'))
    p.add_argument('--output', required=True, help='new directory under calibration_risabot5')
    p = sub.add_parser('fit', help='produce a measured candidate; always leaves calibrated false')
    p.add_argument('manifest')
    p.add_argument('--output', required=True)
    p.add_argument('--board-only', action='store_true',
                   help='save a preliminary board fit with missing independent validation explicitly reported')
    args = parser.parse_args(argv)
    try:
        if args.command == 'inspect':
            inspect(args)
            return 0
        report = fit_session(args.manifest, args.output, allow_board_only=args.board_only)
        print(json.dumps({'geometry_pass': report['geometry_pass'], 'calibrated': False}))
        return 0 if report['geometry_pass'] else 2
    except (ValueError, KeyError, TypeError, OSError, cv2.error) as error:
        parser.exit(2, f'Calibration blocked: {error}\n')


if __name__ == '__main__':
    raise SystemExit(main())
