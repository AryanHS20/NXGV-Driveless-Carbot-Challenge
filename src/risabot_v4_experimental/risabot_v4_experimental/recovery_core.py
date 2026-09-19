"""Bounded reverse-first recovery proposals for V4 Stage 6.

This ROS-free module only plans and validates paths.  It has no actuator or
vehicle-command interface.  Recovery is deliberately stricter than parking:
one short reverse section must be followed by one forward rejoin section.
"""

from dataclasses import dataclass, replace
import math
from typing import List, Mapping, Sequence, Tuple

import cv2
import numpy as np

from .bev_core import CameraProfile
from .parking_core import (
    ParkingCandidate,
    ParkingConfig,
    ParkingError,
    reeds_shepp_candidates,
    validate_candidate,
)
from .trajectory_core import PathPoint, VehicleGeometry, reference_from_corridor, road_support


class RecoveryError(ValueError):
    """Raised when a recovery request or planner input is invalid."""


@dataclass(frozen=True)
class RecoveryRequest:
    problem: str
    hard_hold: str
    permitted: bool
    stopped: bool
    attempts: int
    image_stamp_sec: float
    frame_id: str


@dataclass(frozen=True)
class RecoveryConfig:
    sample_step_m: float = 0.006
    footprint_sample_spacing_m: float = 0.02
    minimum_road_support: float = 0.98
    obstacle_margin_m: float = 0.015
    minimum_reverse_distance_m: float = 0.03
    maximum_reverse_distance_m: float = 0.20
    maximum_path_length_m: float = 1.60
    minimum_rear_evidence_fraction: float = 0.55
    road_tolerance_m: float = 0.05
    first_goal_distance_m: float = 0.25
    goal_spacing_m: float = 0.15
    maximum_goal_count: int = 5
    maximum_attempts: int = 3

    def validate(self) -> None:
        positive = (
            ('sample_step_m', self.sample_step_m),
            ('footprint_sample_spacing_m', self.footprint_sample_spacing_m),
            ('minimum_reverse_distance_m', self.minimum_reverse_distance_m),
            ('maximum_reverse_distance_m', self.maximum_reverse_distance_m),
            ('maximum_path_length_m', self.maximum_path_length_m),
            ('first_goal_distance_m', self.first_goal_distance_m),
            ('goal_spacing_m', self.goal_spacing_m),
        )
        for name, value in positive:
            if not math.isfinite(value) or value <= 0.0:
                raise RecoveryError(f'{name} must be finite and positive')
        if self.minimum_reverse_distance_m > self.maximum_reverse_distance_m:
            raise RecoveryError('minimum reverse distance exceeds maximum')
        if not 0.0 < self.minimum_road_support <= 1.0:
            raise RecoveryError('minimum road support must be in (0, 1]')
        if not 0.0 < self.minimum_rear_evidence_fraction <= 1.0:
            raise RecoveryError('minimum rear evidence fraction must be in (0, 1]')
        if not math.isfinite(self.obstacle_margin_m) or self.obstacle_margin_m < 0.0:
            raise RecoveryError('obstacle margin must be finite and non-negative')
        if not math.isfinite(self.road_tolerance_m) or self.road_tolerance_m < 0.0:
            raise RecoveryError('road tolerance must be finite and non-negative')
        if self.maximum_goal_count <= 0 or self.maximum_attempts <= 0:
            raise RecoveryError('goal and attempt limits must be positive')


@dataclass
class RecoveryCandidate:
    goal_index: int
    goal: PathPoint
    path: ParkingCandidate
    rear_evidence_fraction: float
    valid: bool
    reject_reason: str


def recovery_request_from_mapping(
    value: Mapping[str, object], required_frame: str = 'base_link'
) -> RecoveryRequest:
    if not isinstance(value, Mapping):
        raise RecoveryError('recovery request must be an object')
    if not isinstance(value.get('permitted'), bool) or not isinstance(value.get('stopped'), bool):
        raise RecoveryError('permitted and stopped must be booleans')
    if isinstance(value.get('attempts'), bool) or not isinstance(value.get('attempts'), int):
        raise RecoveryError('attempts must be an integer')
    try:
        request = RecoveryRequest(
            problem=str(value['problem']).strip(),
            hard_hold=str(value.get('hard_hold', '')).strip(),
            permitted=value['permitted'],
            stopped=value['stopped'],
            attempts=int(value['attempts']),
            image_stamp_sec=float(value['image_stamp_sec']),
            frame_id=str(value['frame_id']),
        )
    except (KeyError, TypeError, ValueError) as exc:
        raise RecoveryError(f'invalid recovery request field: {exc}') from exc
    if not request.problem:
        raise RecoveryError('problem must be non-empty')
    if request.problem != 'no_forward_candidate':
        raise RecoveryError('unsupported recovery problem')
    if request.frame_id != required_frame:
        raise RecoveryError(f'recovery request frame must be {required_frame}')
    if request.attempts < 0:
        raise RecoveryError('attempts must be non-negative')
    if not math.isfinite(request.image_stamp_sec) or request.image_stamp_sec < 0.0:
        raise RecoveryError('image timestamp must be finite and non-negative')
    return request


def request_blockers(request: RecoveryRequest, config: RecoveryConfig) -> List[str]:
    """Return policy blockers; any hard hold always wins."""
    config.validate()
    blockers = []
    if request.hard_hold:
        blockers.append(f'hard hold active: {request.hard_hold}')
    if not request.permitted:
        blockers.append('recovery is not permitted by mission logic')
    if not request.stopped:
        blockers.append('vehicle is not confirmed stopped')
    if request.attempts >= config.maximum_attempts:
        blockers.append('recovery attempt limit reached')
    return blockers


def forward_candidates_exhausted(value: Mapping[str, object]) -> bool:
    """Accept only a Stage 4 report that evaluated and rejected every path."""
    if not isinstance(value, Mapping) or value.get('algorithm_stage') != 4:
        return False
    candidates = value.get('candidates')
    if not isinstance(candidates, list) or not candidates:
        return False
    if value.get('selected_diagnostic_only') is not None:
        return False
    return all(
        isinstance(candidate, Mapping) and candidate.get('valid') is False
        for candidate in candidates
    )


def select_rejoin_goals(
    corridor_samples: Sequence[Tuple[float, float]],
    config: RecoveryConfig = RecoveryConfig(),
) -> List[PathPoint]:
    """Pick a small set of increasingly distant goals along the road centre."""
    config.validate()
    reference = reference_from_corridor(corridor_samples)
    distances = []
    previous = PathPoint(0.0, 0.0, 0.0)
    accumulated = 0.0
    for point in reference:
        accumulated += math.hypot(point.x - previous.x, point.y - previous.y)
        distances.append(accumulated)
        previous = point
    goals = []
    target = config.first_goal_distance_m
    for _ in range(config.maximum_goal_count):
        eligible = [index for index, distance in enumerate(distances) if distance >= target]
        if not eligible:
            break
        point = reference[eligible[0]]
        if not goals or math.hypot(point.x - goals[-1].x, point.y - goals[-1].y) > 1e-6:
            goals.append(point)
        target += config.goal_spacing_m
    return goals


def _directions(candidate: ParkingCandidate) -> List[int]:
    sections = []
    for point in candidate.points:
        direction = int(point.direction)
        if direction not in (-1, 1):
            continue
        if not sections or sections[-1] != direction:
            sections.append(direction)
    return sections


def _rear_evidence_fraction(
    candidate: ParkingCandidate,
    evidence_mask: np.ndarray,
    profile: CameraProfile,
    geometry: VehicleGeometry,
) -> float:
    """Measure observed coverage behind the body throughout reverse motion."""
    fractions = []
    rear_distance = geometry.rear_overhang_m + geometry.footprint_padding_m
    half_width = 0.5 * geometry.width_m + geometry.footprint_padding_m
    for point in candidate.points:
        if point.direction != -1:
            continue
        cosine = math.cos(point.yaw)
        sine = math.sin(point.yaw)
        samples = []
        for lateral in (-half_width, 0.0, half_width):
            longitudinal = -(rear_distance + 0.02)
            samples.append((
                point.x + longitudinal * cosine - lateral * sine,
                point.y + longitudinal * sine + lateral * cosine,
            ))
        fractions.append(road_support(np.asarray(samples), evidence_mask, profile))
    return float(sum(fractions) / len(fractions)) if fractions else 0.0


def plan_recovery(
    corridor_samples: Sequence[Tuple[float, float]],
    drivable_mask: np.ndarray,
    rear_evidence_mask: np.ndarray,
    profile: CameraProfile,
    geometry: VehicleGeometry = VehicleGeometry(),
    config: RecoveryConfig = RecoveryConfig(),
    obstacles_m: Sequence[Tuple[float, float]] = (),
) -> List[RecoveryCandidate]:
    """Return checked reverse-then-forward rejoin paths, valid first."""
    geometry.validate()
    config.validate()
    parking_config = ParkingConfig(
        sample_step_m=config.sample_step_m,
        footprint_sample_spacing_m=config.footprint_sample_spacing_m,
        minimum_road_support=config.minimum_road_support,
        obstacle_margin_m=config.obstacle_margin_m,
        maximum_gear_changes=1,
        maximum_reverse_distance_m=config.maximum_reverse_distance_m,
    )
    checked_mask = drivable_mask
    tolerance_px = int(round(config.road_tolerance_m * profile.pixels_per_meter))
    if tolerance_px > 0:
        kernel_size = 2 * tolerance_px + 1
        checked_mask = cv2.dilate(
            drivable_mask, np.ones((kernel_size, kernel_size), np.uint8)
        )
        # Edge tolerance may bridge paint/noise only where the rear camera has
        # actually observed the ground. Unobserved pixels never become road.
        checked_mask = cv2.bitwise_and(checked_mask, rear_evidence_mask)
    evaluated = []
    start = PathPoint(0.0, 0.0, 0.0)
    for goal_index, goal in enumerate(select_rejoin_goals(corridor_samples, config)):
        for candidate in reeds_shepp_candidates(
            start, goal, geometry.minimum_turn_radius_m, parking_config
        ):
            checked = validate_candidate(
                candidate, checked_mask, profile, geometry, parking_config,
                obstacles_m=obstacles_m, require_reverse=True,
            )
            reasons = [checked.reject_reason] if checked.reject_reason else []
            directions = _directions(checked)
            if directions != [-1, 1]:
                reasons.append('path must contain one reverse section then one forward section')
            if checked.reverse_distance_m < config.minimum_reverse_distance_m:
                reasons.append('minimum reverse distance not reached')
            if checked.path_length_m > config.maximum_path_length_m:
                reasons.append('path-length limit exceeded')
            evidence = _rear_evidence_fraction(
                checked, rear_evidence_mask, profile, geometry
            )
            if evidence < config.minimum_rear_evidence_fraction:
                reasons.append('insufficient observed rear coverage')
            unique_reasons = list(dict.fromkeys(reason for reason in reasons if reason))
            evaluated.append(RecoveryCandidate(
                goal_index=goal_index,
                goal=goal,
                path=replace(checked, valid=not unique_reasons,
                             reject_reason='; '.join(unique_reasons)),
                rear_evidence_fraction=evidence,
                valid=not unique_reasons,
                reject_reason='; '.join(unique_reasons),
            ))
    return sorted(evaluated, key=lambda item: (not item.valid, item.path.cost))
