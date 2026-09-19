"""Parking-bay marking measurement for the Stage 5 goal contract."""

from dataclasses import dataclass
import math
from typing import Optional

import cv2
import numpy as np

from .bev_core import CameraProfile, bev_to_metric
from .parking_core import ParkingError, ParkingGoal, wrap_angle
from .trajectory_core import PathPoint


@dataclass(frozen=True)
class ParkingMarkingConfig:
    value_min: int = 180
    saturation_max: int = 90
    morph_close_px: int = 9
    minimum_length_m: float = 0.32
    maximum_length_m: float = 1.20
    minimum_width_m: float = 0.22
    maximum_width_m: float = 0.80
    minimum_confidence: float = 0.65

    def validate(self) -> None:
        if not 0 <= self.value_min <= 255 or not 0 <= self.saturation_max <= 255:
            raise ParkingError('parking marking thresholds must be bytes')
        if self.morph_close_px <= 0 or self.morph_close_px % 2 == 0:
            raise ParkingError('morph_close_px must be a positive odd integer')
        if not (0 < self.minimum_length_m <= self.maximum_length_m):
            raise ParkingError('parking length limits are invalid')
        if not (0 < self.minimum_width_m <= self.maximum_width_m):
            raise ParkingError('parking width limits are invalid')
        if not 0.0 <= self.minimum_confidence <= 1.0:
            raise ParkingError('minimum confidence must be in [0, 1]')


def _angle_distance(a: float, b: float) -> float:
    return abs(wrap_angle(a - b))


def detect_parking_goal(
    image_bgr: np.ndarray,
    coverage_mask: np.ndarray,
    profile: CameraProfile,
    image_stamp_sec: float,
    kind: str = 'parallel',
    preferred_yaw_rad: float = 0.0,
    config: ParkingMarkingConfig = ParkingMarkingConfig(),
) -> Optional[ParkingGoal]:
    """Measure the best closed bright bay marking in a calibrated BEV image."""
    config.validate()
    if kind not in ('parallel', 'perpendicular'):
        raise ParkingError('parking kind must be parallel or perpendicular')
    if not profile.calibrated:
        raise ParkingError('parking camera profile is uncalibrated')
    if image_bgr is None or image_bgr.ndim != 3 or image_bgr.shape[2] != 3:
        raise ParkingError('parking BEV must be a BGR image')
    if coverage_mask is None or coverage_mask.ndim != 2:
        raise ParkingError('parking coverage must be single-channel')
    if image_bgr.shape[:2] != coverage_mask.shape:
        raise ParkingError('parking image and coverage shapes differ')
    if tuple(reversed(coverage_mask.shape)) != profile.output_size:
        raise ParkingError('parking image shape does not match profile')
    if not math.isfinite(image_stamp_sec) or image_stamp_sec < 0.0:
        raise ParkingError('image timestamp must be finite and non-negative')
    if not math.isfinite(preferred_yaw_rad):
        raise ParkingError('preferred yaw must be finite')

    hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
    marking = cv2.inRange(
        hsv, np.array([0, 0, config.value_min], np.uint8),
        np.array([179, config.saturation_max, 255], np.uint8),
    )
    marking = cv2.bitwise_and(marking, coverage_mask)
    kernel = np.ones((config.morph_close_px, config.morph_close_px), np.uint8)
    marking = cv2.morphologyEx(marking, cv2.MORPH_CLOSE, kernel)
    contours, _ = cv2.findContours(marking, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    choices = []
    for contour in contours:
        if len(contour) < 4:
            continue
        rectangle = cv2.minAreaRect(contour)
        box_px = cv2.boxPoints(rectangle).astype(np.float64)
        box_m = bev_to_metric(profile, box_px)
        edges = [box_m[(i + 1) % 4] - box_m[i] for i in range(4)]
        lengths = [float(np.linalg.norm(edge)) for edge in edges]
        long_index = int(np.argmax(lengths))
        length_m = lengths[long_index]
        width_m = min(lengths)
        if not (config.minimum_length_m <= length_m <= config.maximum_length_m):
            continue
        if not (config.minimum_width_m <= width_m <= config.maximum_width_m):
            continue
        rect_area_px = max(1.0, float(rectangle[1][0] * rectangle[1][1]))
        rectangularity = min(1.0, float(cv2.contourArea(contour)) / rect_area_px)
        perimeter_ratio = min(1.0, float(cv2.arcLength(contour, True)) /
                              max(1.0, 2.0 * sum(rectangle[1])))
        confidence = 0.45 * rectangularity + 0.55 * perimeter_ratio
        if confidence < config.minimum_confidence:
            continue
        center = box_m.mean(axis=0)
        edge = edges[long_index]
        yaw = math.atan2(float(edge[1]), float(edge[0]))
        if _angle_distance(yaw + math.pi, preferred_yaw_rad) < _angle_distance(yaw, preferred_yaw_rad):
            yaw = wrap_angle(yaw + math.pi)
        choices.append((confidence, length_m * width_m, ParkingGoal(
            kind=kind, target=PathPoint(float(center[0]), float(center[1]), yaw),
            slot_length_m=length_m, slot_width_m=width_m,
            confidence=confidence, image_stamp_sec=image_stamp_sec,
            frame_id='base_link',
        )))
    if not choices:
        return None
    return max(choices, key=lambda item: (item[0], item[1]))[2]
