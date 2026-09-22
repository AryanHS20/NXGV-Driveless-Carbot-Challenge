"""Calibration profile validation and metric bird's-eye-view transforms.

This module has no ROS dependency so the geometry can be tested on a laptop
and replayed against recorded frames before it is used on the board.
"""

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Mapping, Sequence, Tuple

import cv2
import numpy as np
import yaml


class CalibrationError(ValueError):
    """Raised when a profile cannot support a measured BEV transform."""


@dataclass(frozen=True)
class CameraProfile:
    name: str
    calibrated: bool
    resolution: Tuple[int, int]
    camera_matrix: np.ndarray
    distortion: np.ndarray
    source_points_px: np.ndarray
    ground_points_m: np.ndarray
    forward_bounds_m: Tuple[float, float]
    left_bounds_m: Tuple[float, float]
    pixels_per_meter: float

    @property
    def output_size(self) -> Tuple[int, int]:
        """Return output width and height, retaining both metric endpoints."""
        forward_span = self.forward_bounds_m[1] - self.forward_bounds_m[0]
        left_span = self.left_bounds_m[1] - self.left_bounds_m[0]
        width = int(round(left_span * self.pixels_per_meter)) + 1
        height = int(round(forward_span * self.pixels_per_meter)) + 1
        return width, height


def _pair(value: Sequence[Any], label: str) -> Tuple[float, float]:
    if not isinstance(value, (list, tuple)) or len(value) != 2:
        raise CalibrationError(f'{label} must contain exactly two values')
    return float(value[0]), float(value[1])


def _matrix(value: Any, shape: Tuple[int, int], label: str) -> np.ndarray:
    array = np.asarray(value, dtype=np.float64)
    if array.shape != shape or not np.all(np.isfinite(array)):
        raise CalibrationError(f'{label} must be a finite {shape[0]}x{shape[1]} matrix')
    return array


def _points(value: Any, label: str) -> np.ndarray:
    array = np.asarray(value, dtype=np.float64)
    if array.shape != (4, 2) or not np.all(np.isfinite(array)):
        raise CalibrationError(f'{label} must contain four finite [x, y] points')
    if abs(float(cv2.contourArea(array.astype(np.float32)))) < 1e-6:
        raise CalibrationError(f'{label} is degenerate')
    return array


def profile_from_mapping(name: str, raw: Mapping[str, Any]) -> CameraProfile:
    """Parse one YAML profile, preserving explicit uncalibrated placeholders."""
    if not isinstance(raw, Mapping):
        raise CalibrationError(f'{name} profile must be a mapping')
    resolution_raw = raw.get('resolution', [])
    if not isinstance(resolution_raw, (list, tuple)) or len(resolution_raw) != 2:
        raise CalibrationError(f'{name}.resolution must be [width, height]')
    resolution = int(resolution_raw[0]), int(resolution_raw[1])
    if resolution[0] <= 0 or resolution[1] <= 0:
        raise CalibrationError(f'{name}.resolution must be positive')

    bounds = raw.get('ground_bounds_m', {})
    if not isinstance(bounds, Mapping):
        raise CalibrationError(f'{name}.ground_bounds_m must be a mapping')
    forward = _pair(bounds.get('forward', []), f'{name}.ground_bounds_m.forward')
    left = _pair(bounds.get('left', []), f'{name}.ground_bounds_m.left')
    pixels_per_meter = float(raw.get('pixels_per_meter', 0.0))
    if forward[1] <= forward[0] or left[1] <= left[0]:
        raise CalibrationError(f'{name} ground bounds must increase')
    if not np.isfinite(pixels_per_meter) or pixels_per_meter <= 0.0:
        raise CalibrationError(f'{name}.pixels_per_meter must be positive')

    calibrated = bool(raw.get('calibrated', False))
    if not calibrated:
        empty_points = np.empty((0, 2), dtype=np.float64)
        return CameraProfile(
            name=name,
            calibrated=False,
            resolution=resolution,
            camera_matrix=np.empty((0, 0), dtype=np.float64),
            distortion=np.empty((0,), dtype=np.float64),
            source_points_px=empty_points,
            ground_points_m=empty_points.copy(),
            forward_bounds_m=forward,
            left_bounds_m=left,
            pixels_per_meter=pixels_per_meter,
        )

    camera_matrix = _matrix(raw.get('camera_matrix'), (3, 3), f'{name}.camera_matrix')
    if camera_matrix[0, 0] <= 0.0 or camera_matrix[1, 1] <= 0.0:
        raise CalibrationError(f'{name}.camera_matrix focal lengths must be positive')
    distortion = np.asarray(raw.get('distortion_coefficients'), dtype=np.float64).reshape(-1)
    if distortion.size not in (4, 5, 8, 12, 14) or not np.all(np.isfinite(distortion)):
        raise CalibrationError(
            f'{name}.distortion_coefficients must have 4, 5, 8, 12 or 14 values'
        )
    source = _points(raw.get('source_points_px'), f'{name}.source_points_px')
    ground = _points(raw.get('ground_points_m'), f'{name}.ground_points_m')
    width, height = resolution
    if np.any(source[:, 0] < 0) or np.any(source[:, 0] >= width):
        raise CalibrationError(f'{name}.source_points_px contains x outside the image')
    if np.any(source[:, 1] < 0) or np.any(source[:, 1] >= height):
        raise CalibrationError(f'{name}.source_points_px contains y outside the image')
    if np.any(ground[:, 0] < forward[0]) or np.any(ground[:, 0] > forward[1]):
        raise CalibrationError(f'{name}.ground_points_m contains forward distance outside bounds')
    if np.any(ground[:, 1] < left[0]) or np.any(ground[:, 1] > left[1]):
        raise CalibrationError(f'{name}.ground_points_m contains lateral distance outside bounds')

    return CameraProfile(
        name=name,
        calibrated=True,
        resolution=resolution,
        camera_matrix=camera_matrix,
        distortion=distortion,
        source_points_px=source,
        ground_points_m=ground,
        forward_bounds_m=forward,
        left_bounds_m=left,
        pixels_per_meter=pixels_per_meter,
    )


def load_profiles(path: str) -> Dict[str, CameraProfile]:
    """Load the versioned primary and secondary profiles from YAML."""
    source = Path(path)
    with source.open('r', encoding='utf-8') as handle:
        document = yaml.safe_load(handle)
    if not isinstance(document, Mapping) or int(document.get('version', 0)) != 1:
        raise CalibrationError('camera profile file must have version: 1')
    raw_profiles = document.get('profiles')
    if not isinstance(raw_profiles, Mapping):
        raise CalibrationError('camera profile file must contain profiles')
    missing = {'primary', 'secondary'} - set(raw_profiles)
    if missing:
        raise CalibrationError('missing profiles: ' + ', '.join(sorted(missing)))
    return {
        name: profile_from_mapping(name, raw_profiles[name])
        for name in ('primary', 'secondary')
    }


def ground_to_bev_points(profile: CameraProfile) -> np.ndarray:
    """Map [forward, left] ground coordinates to BEV pixel coordinates."""
    if not profile.calibrated:
        raise CalibrationError(f'{profile.name} is not calibrated')
    return metric_to_bev(profile, profile.ground_points_m)


def metric_to_bev(profile: CameraProfile, points_m: np.ndarray) -> np.ndarray:
    """Map arbitrary [forward, left] metric points into the profile's BEV."""
    if not profile.calibrated:
        raise CalibrationError(f'{profile.name} is not calibrated')
    points = np.asarray(points_m, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 2 or not np.all(np.isfinite(points)):
        raise CalibrationError('metric points must be a finite Nx2 array')
    width, height = profile.output_size
    f0, f1 = profile.forward_bounds_m
    l0, l1 = profile.left_bounds_m
    forward = points[:, 0]
    left = points[:, 1]
    u = (l1 - left) * (width - 1) / (l1 - l0)
    v = (f1 - forward) * (height - 1) / (f1 - f0)
    return np.column_stack((u, v)).astype(np.float32)


def bev_to_metric(profile: CameraProfile, points_px: np.ndarray) -> np.ndarray:
    """Map arbitrary BEV [u, v] pixels back to [forward, left] metres."""
    if not profile.calibrated:
        raise CalibrationError(f'{profile.name} is not calibrated')
    points = np.asarray(points_px, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 2 or not np.all(np.isfinite(points)):
        raise CalibrationError('BEV points must be a finite Nx2 array')
    width, height = profile.output_size
    f0, f1 = profile.forward_bounds_m
    l0, l1 = profile.left_bounds_m
    left = l1 - points[:, 0] * (l1 - l0) / (width - 1)
    forward = f1 - points[:, 1] * (f1 - f0) / (height - 1)
    return np.column_stack((forward, left)).astype(np.float32)


def undistorted_source_points(profile: CameraProfile) -> np.ndarray:
    """Convert raw-image calibration marks to coordinates in the undistorted image."""
    if not profile.calibrated:
        raise CalibrationError(f'{profile.name} is not calibrated')
    points = profile.source_points_px.reshape(-1, 1, 2).astype(np.float64)
    return cv2.undistortPoints(
        points,
        profile.camera_matrix,
        profile.distortion,
        P=profile.camera_matrix,
    ).reshape(-1, 2).astype(np.float32)


def build_homography(profile: CameraProfile) -> np.ndarray:
    """Build the raw-ground correspondence homography after lens correction."""
    source = undistorted_source_points(profile)
    destination = ground_to_bev_points(profile)
    matrix = cv2.getPerspectiveTransform(source, destination)
    if not np.all(np.isfinite(matrix)) or abs(float(np.linalg.det(matrix))) < 1e-12:
        raise CalibrationError(f'{profile.name} homography is singular')
    return matrix


def warp_to_bev(image: np.ndarray, profile: CameraProfile) -> Tuple[np.ndarray, np.ndarray]:
    """Undistort and warp an image, returning the BEV and observed-pixel mask."""
    if not profile.calibrated:
        raise CalibrationError(f'{profile.name} is not calibrated')
    if image is None or image.ndim not in (2, 3):
        raise CalibrationError('input image must be a 2-D or 3-D array')
    expected_width, expected_height = profile.resolution
    if image.shape[1] != expected_width or image.shape[0] != expected_height:
        raise CalibrationError(
            f'{profile.name} expected {expected_width}x{expected_height}, '
            f'got {image.shape[1]}x{image.shape[0]}'
        )
    undistorted = cv2.undistort(
        image, profile.camera_matrix, profile.distortion, None, profile.camera_matrix
    )
    matrix = build_homography(profile)
    output_size = profile.output_size
    bev = cv2.warpPerspective(
        undistorted,
        matrix,
        output_size,
        flags=cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_CONSTANT,
    )
    raw_mask = np.full((expected_height, expected_width), 255, dtype=np.uint8)
    undistorted_mask = cv2.undistort(
        raw_mask, profile.camera_matrix, profile.distortion, None, profile.camera_matrix
    )
    coverage = cv2.warpPerspective(
        undistorted_mask,
        matrix,
        output_size,
        flags=cv2.INTER_NEAREST,
        borderMode=cv2.BORDER_CONSTANT,
    )
    return bev, coverage


def profile_report(profile: CameraProfile) -> Dict[str, Any]:
    width, height = profile.output_size
    return {
        'calibrated': profile.calibrated,
        'input_resolution': list(profile.resolution),
        'output_resolution': [width, height],
        'forward_bounds_m': list(profile.forward_bounds_m),
        'left_bounds_m': list(profile.left_bounds_m),
        'pixels_per_meter': profile.pixels_per_meter,
    }
