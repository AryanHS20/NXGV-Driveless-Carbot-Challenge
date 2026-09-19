"""Coverage-aware road segmentation and corridor extraction for Stage 2."""

from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np


class RoadMaskError(ValueError):
    """Raised when a road-mask input or configuration is invalid."""


@dataclass(frozen=True)
class RoadMaskConfig:
    value_min: int = 0
    value_max: int = 135
    saturation_max: int = 255
    morph_open_px: int = 3
    morph_close_px: int = 7
    min_component_px: int = 100

    def validate(self) -> None:
        for name, value in (
            ('value_min', self.value_min),
            ('value_max', self.value_max),
            ('saturation_max', self.saturation_max),
        ):
            if int(value) < 0 or int(value) > 255:
                raise RoadMaskError(f'{name} must be in [0, 255]')
        if self.value_min > self.value_max:
            raise RoadMaskError('value_min must not exceed value_max')
        for name, value in (
            ('morph_open_px', self.morph_open_px),
            ('morph_close_px', self.morph_close_px),
        ):
            if int(value) < 0 or (int(value) > 0 and int(value) % 2 == 0):
                raise RoadMaskError(f'{name} must be zero or a positive odd number')
        if int(self.min_component_px) < 1:
            raise RoadMaskError('min_component_px must be positive')


@dataclass(frozen=True)
class CorridorSample:
    row_px: int
    left_px: int
    right_px: int
    center_px: float
    width_px: int


def _validate_images(image: np.ndarray, coverage: np.ndarray) -> None:
    if image is None or image.ndim != 3 or image.shape[2] != 3:
        raise RoadMaskError('BEV image must be HxWx3 BGR')
    if coverage is None or coverage.ndim != 2:
        raise RoadMaskError('coverage must be a single-channel mask')
    if coverage.shape != image.shape[:2]:
        raise RoadMaskError('coverage shape must match the BEV image')


def segment_dark_road(
    bev_bgr: np.ndarray,
    coverage: np.ndarray,
    config: RoadMaskConfig,
) -> np.ndarray:
    """Create a candidate mask without treating unobserved black pixels as road."""
    _validate_images(bev_bgr, coverage)
    config.validate()
    hsv = cv2.cvtColor(bev_bgr, cv2.COLOR_BGR2HSV)
    saturation = hsv[:, :, 1]
    value = hsv[:, :, 2]
    candidate = (
        (value >= config.value_min)
        & (value <= config.value_max)
        & (saturation <= config.saturation_max)
        & (coverage > 0)
    ).astype(np.uint8) * 255
    if config.morph_open_px:
        kernel = np.ones((config.morph_open_px, config.morph_open_px), np.uint8)
        candidate = cv2.morphologyEx(candidate, cv2.MORPH_OPEN, kernel)
    if config.morph_close_px:
        kernel = np.ones((config.morph_close_px, config.morph_close_px), np.uint8)
        candidate = cv2.morphologyEx(candidate, cv2.MORPH_CLOSE, kernel)
    return candidate


def seed_connected_component(
    candidate: np.ndarray,
    seed_xy: Tuple[float, float],
    seed_radius_px: int,
    min_component_px: int,
) -> np.ndarray:
    """Keep the four-connected candidate component with most seed overlap."""
    if candidate is None or candidate.ndim != 2:
        raise RoadMaskError('candidate must be a single-channel mask')
    if seed_radius_px < 1 or min_component_px < 1:
        raise RoadMaskError('seed radius and component size must be positive')
    height, width = candidate.shape
    seed_x = int(round(float(seed_xy[0])))
    seed_y = int(round(float(seed_xy[1])))
    if seed_x < 0 or seed_x >= width or seed_y < 0 or seed_y >= height:
        raise RoadMaskError('seed lies outside the BEV image')

    count, labels, stats, _ = cv2.connectedComponentsWithStats(
        (candidate > 0).astype(np.uint8), connectivity=4
    )
    seed_mask = np.zeros_like(candidate, dtype=np.uint8)
    cv2.circle(seed_mask, (seed_x, seed_y), int(seed_radius_px), 1, thickness=-1)
    touched = labels[seed_mask > 0]
    if touched.size == 0:
        return np.zeros_like(candidate)
    label_counts = np.bincount(touched, minlength=count)
    label_counts[0] = 0
    chosen = int(np.argmax(label_counts))
    if chosen == 0 or int(stats[chosen, cv2.CC_STAT_AREA]) < min_component_px:
        return np.zeros_like(candidate)
    return (labels == chosen).astype(np.uint8) * 255


def _runs(row: np.ndarray) -> List[Tuple[int, int]]:
    indices = np.flatnonzero(row > 0)
    if indices.size == 0:
        return []
    splits = np.where(np.diff(indices) > 1)[0] + 1
    return [(int(part[0]), int(part[-1])) for part in np.split(indices, splits)]


def extract_corridor(
    connected_mask: np.ndarray,
    coverage: np.ndarray,
    seed_center_x: float,
    pixels_per_meter: float,
    min_width_m: float = 0.12,
    max_width_m: float = 0.80,
    row_step_px: int = 8,
    min_coverage_fraction: float = 0.75,
) -> List[CorridorSample]:
    """Follow the contiguous road run nearest the previous centre, near to far."""
    if connected_mask is None or connected_mask.ndim != 2:
        raise RoadMaskError('connected mask must be single-channel')
    if coverage.shape != connected_mask.shape:
        raise RoadMaskError('coverage shape must match connected mask')
    if pixels_per_meter <= 0.0 or row_step_px < 1:
        raise RoadMaskError('pixel scale and row step must be positive')
    if not 0.0 <= min_coverage_fraction <= 1.0:
        raise RoadMaskError('min coverage fraction must be in [0, 1]')
    min_width_px = max(1, int(round(min_width_m * pixels_per_meter)))
    max_width_px = max(min_width_px, int(round(max_width_m * pixels_per_meter)))
    previous = float(seed_center_x)
    samples: List[CorridorSample] = []
    for row_index in range(connected_mask.shape[0] - 1, -1, -row_step_px):
        observed = float((coverage[row_index] > 0).mean())
        if observed < min_coverage_fraction:
            continue
        candidates = []
        for left, right in _runs(connected_mask[row_index]):
            width = right - left + 1
            if min_width_px <= width <= max_width_px:
                center = 0.5 * (left + right)
                candidates.append((abs(center - previous), left, right, center, width))
        if not candidates:
            continue
        _, left, right, center, width = min(candidates, key=lambda item: item[0])
        samples.append(CorridorSample(row_index, left, right, center, width))
        previous = center
    return samples


def process_bev(
    bev_bgr: np.ndarray,
    coverage: np.ndarray,
    config: RoadMaskConfig,
    seed_xy: Tuple[float, float],
    seed_radius_px: int,
    pixels_per_meter: float,
    min_width_m: float = 0.12,
    max_width_m: float = 0.80,
    row_step_px: int = 8,
) -> Dict[str, object]:
    candidate = segment_dark_road(bev_bgr, coverage, config)
    connected = seed_connected_component(
        candidate, seed_xy, seed_radius_px, config.min_component_px
    )
    samples = extract_corridor(
        connected,
        coverage,
        seed_center_x=seed_xy[0],
        pixels_per_meter=pixels_per_meter,
        min_width_m=min_width_m,
        max_width_m=max_width_m,
        row_step_px=row_step_px,
    )
    return {
        'candidate': candidate,
        'connected': connected,
        'samples': samples,
        'connected_fraction': float((connected > 0).mean()),
    }
