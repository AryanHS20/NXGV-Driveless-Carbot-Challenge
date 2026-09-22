"""Offline checkerboard intrinsic calibration for Stage 1 profiles."""

import argparse
from glob import glob
from pathlib import Path
from typing import Dict, Iterable, List, Sequence, Tuple

import cv2
import numpy as np
import yaml

from .bev_core import CalibrationError


def checkerboard_object_points(
    columns: int, rows: int, square_size_m: float
) -> np.ndarray:
    if columns < 3 or rows < 3 or square_size_m <= 0.0:
        raise CalibrationError('checkerboard dimensions and square size must be positive')
    points = np.zeros((columns * rows, 3), dtype=np.float32)
    points[:, :2] = np.mgrid[0:columns, 0:rows].T.reshape(-1, 2)
    points[:, :2] *= float(square_size_m)
    return points


def calibrate_from_observations(
    object_points: Sequence[np.ndarray],
    image_points: Sequence[np.ndarray],
    image_size: Tuple[int, int],
) -> Dict[str, object]:
    """Calibrate from already detected corners for deterministic testing/replay."""
    if len(object_points) < 5 or len(object_points) != len(image_points):
        raise CalibrationError('at least five matched checkerboard observations are required')
    width, height = int(image_size[0]), int(image_size[1])
    if width <= 0 or height <= 0:
        raise CalibrationError('image size must be positive')
    rms, matrix, distortion, _, _ = cv2.calibrateCamera(
        [np.asarray(item, dtype=np.float32) for item in object_points],
        [np.asarray(item, dtype=np.float32) for item in image_points],
        (width, height),
        None,
        None,
    )
    if not np.isfinite(rms) or not np.all(np.isfinite(matrix)):
        raise CalibrationError('OpenCV returned a non-finite calibration')
    return {
        'resolution': [width, height],
        'camera_matrix': matrix.tolist(),
        'distortion_coefficients': distortion.reshape(-1).tolist(),
        'intrinsic_rms_px': float(rms),
        'frames_used': len(object_points),
    }


def detect_observations(
    image_paths: Iterable[str],
    columns: int,
    rows: int,
    square_size_m: float,
) -> Tuple[List[np.ndarray], List[np.ndarray], Tuple[int, int], List[str]]:
    template = checkerboard_object_points(columns, rows, square_size_m)
    objects: List[np.ndarray] = []
    images: List[np.ndarray] = []
    accepted: List[str] = []
    image_size = None
    for raw_path in image_paths:
        path = str(Path(raw_path))
        frame = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
        if frame is None:
            continue
        size = (frame.shape[1], frame.shape[0])
        if image_size is None:
            image_size = size
        elif size != image_size:
            raise CalibrationError(
                f'all calibration images must have one resolution; {path} is {size}'
            )
        found, corners = cv2.findChessboardCornersSB(
            frame,
            (columns, rows),
            flags=cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY,
        )
        if not found:
            continue
        objects.append(template.copy())
        images.append(corners.reshape(-1, 1, 2).astype(np.float32))
        accepted.append(path)
    if image_size is None:
        raise CalibrationError('no readable calibration images found')
    return objects, images, image_size, accepted


def main(args=None) -> None:
    parser = argparse.ArgumentParser(
        description='Estimate OpenCV intrinsics and print a reviewed YAML fragment.'
    )
    parser.add_argument('images', help='Image glob, for example captures/primary/*.png')
    parser.add_argument('--columns', type=int, required=True, help='Checkerboard inner corners across')
    parser.add_argument('--rows', type=int, required=True, help='Checkerboard inner corners down')
    parser.add_argument('--square-m', type=float, required=True, help='Measured checker square size in metres')
    parser.add_argument('--camera', choices=('primary', 'secondary'), required=True)
    parsed = parser.parse_args(args=args)

    paths = sorted(glob(parsed.images))
    objects, images, size, accepted = detect_observations(
        paths, parsed.columns, parsed.rows, parsed.square_m
    )
    result = calibrate_from_observations(objects, images, size)
    result['camera'] = parsed.camera
    result['images_considered'] = len(paths)
    result['accepted_images'] = accepted
    result['calibrated'] = False
    print(yaml.safe_dump(result, sort_keys=False))


if __name__ == '__main__':
    main()
