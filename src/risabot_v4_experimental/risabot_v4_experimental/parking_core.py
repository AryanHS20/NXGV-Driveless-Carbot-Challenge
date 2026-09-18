"""Reeds-Shepp parking path proposals for V4 Stage 5.

The analytic families follow the MIT-licensed PythonRobotics formulation used
by the simulator reference. Every endpoint is independently reconstructed with
the bicycle model and every accepted path is checked with the full vehicle
footprint. This module has no ROS dependency.
"""

from dataclasses import dataclass, replace
import math
from typing import Callable, List, Mapping, Optional, Sequence, Tuple

import numpy as np

from .bev_core import CameraProfile
from .trajectory_core import (
    PathPoint,
    TrajectoryError,
    VehicleGeometry,
    footprint_points,
    obstacle_in_footprint,
    road_support,
)


class ParkingError(ValueError):
    """Raised when parking geometry or planner input is invalid."""


@dataclass(frozen=True)
class ParkingGoal:
    kind: str
    target: PathPoint
    slot_length_m: float
    slot_width_m: float
    confidence: float
    image_stamp_sec: float
    frame_id: str


def parking_goal_from_mapping(
    value: Mapping[str, object],
    required_frame: str = 'base_link',
) -> ParkingGoal:
    """Validate a measured rear-axle goal expressed in the current base frame."""
    if not isinstance(value, Mapping):
        raise ParkingError('parking goal must be an object')
    try:
        kind = str(value['kind']).lower()
        frame_id = str(value['frame_id'])
        goal = ParkingGoal(
            kind=kind,
            target=PathPoint(
                float(value['x_m']), float(value['y_m']),
                wrap_angle(float(value['yaw_rad'])),
            ),
            slot_length_m=float(value['slot_length_m']),
            slot_width_m=float(value['slot_width_m']),
            confidence=float(value['confidence']),
            image_stamp_sec=float(value['image_stamp_sec']),
            frame_id=frame_id,
        )
    except (KeyError, TypeError, ValueError) as exc:
        raise ParkingError(f'invalid parking goal field: {exc}') from exc
    if kind not in ('parallel', 'perpendicular'):
        raise ParkingError('parking kind must be parallel or perpendicular')
    if frame_id != required_frame:
        raise ParkingError(f'parking goal frame must be {required_frame}')
    numeric = (
        goal.target.x, goal.target.y, goal.target.yaw,
        goal.slot_length_m, goal.slot_width_m,
        goal.confidence, goal.image_stamp_sec,
    )
    if not all(math.isfinite(item) for item in numeric):
        raise ParkingError('parking goal values must be finite')
    if goal.slot_length_m <= 0.0 or goal.slot_width_m <= 0.0:
        raise ParkingError('parking slot dimensions must be positive')
    if not 0.0 <= goal.confidence <= 1.0:
        raise ParkingError('parking goal confidence must be in [0, 1]')
    if goal.image_stamp_sec < 0.0:
        raise ParkingError('parking goal image timestamp must be non-negative')
    return goal


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(float(angle)), math.cos(float(angle)))


@dataclass(frozen=True)
class ParkingPathPoint:
    x: float
    y: float
    yaw: float
    direction: int
    curvature: float


@dataclass
class ParkingCandidate:
    candidate_id: int
    family: str
    segment_lengths_m: Tuple[float, ...]
    points: List[ParkingPathPoint]
    path_length_m: float
    reverse_distance_m: float
    gear_changes: int
    cost: float
    endpoint_position_error_m: float
    endpoint_yaw_error_rad: float
    valid: bool = False
    road_blocked_samples: int = 0
    obstacle_blocked_samples: int = 0
    minimum_road_support: float = 0.0
    reject_reason: str = ''
    stage: str = 'direct'


@dataclass(frozen=True)
class ParkingConfig:
    sample_step_m: float = 0.006
    footprint_sample_spacing_m: float = 0.02
    minimum_road_support: float = 0.98
    obstacle_margin_m: float = 0.015
    endpoint_position_tolerance_m: float = 1e-5
    endpoint_yaw_tolerance_rad: float = 1e-5
    maximum_gear_changes: int = 4
    maximum_reverse_distance_m: float = 1.20
    gear_change_penalty_m: float = 0.08
    reverse_penalty_scale: float = 0.07
    docking_distances_m: Tuple[float, ...] = (0.15, 0.12, 0.08, 0.0)

    def validate(self) -> None:
        for name, value in (
            ('sample_step_m', self.sample_step_m),
            ('footprint_sample_spacing_m', self.footprint_sample_spacing_m),
            ('endpoint_position_tolerance_m', self.endpoint_position_tolerance_m),
            ('endpoint_yaw_tolerance_rad', self.endpoint_yaw_tolerance_rad),
            ('maximum_reverse_distance_m', self.maximum_reverse_distance_m),
        ):
            if not math.isfinite(value) or value <= 0.0:
                raise ParkingError(f'{name} must be finite and positive')
        if not 0.0 < self.minimum_road_support <= 1.0:
            raise ParkingError('minimum road support must be in (0, 1]')
        if not math.isfinite(self.obstacle_margin_m) or self.obstacle_margin_m < 0.0:
            raise ParkingError('obstacle margin must be finite and non-negative')
        if self.maximum_gear_changes < 0:
            raise ParkingError('maximum gear changes must be non-negative')
        if self.gear_change_penalty_m < 0.0 or self.reverse_penalty_scale < 0.0:
            raise ParkingError('parking cost penalties must be non-negative')
        if not self.docking_distances_m or not all(
            math.isfinite(value) and value >= 0.0
            for value in self.docking_distances_m
        ):
            raise ParkingError('docking distances must be finite and non-negative')


def _polar(x: float, y: float) -> Tuple[float, float]:
    return math.hypot(x, y), math.atan2(y, x)


def _acos(value: float) -> float:
    return math.acos(max(-1.0, min(1.0, value)))


def _asin(value: float) -> float:
    return math.asin(max(-1.0, min(1.0, value)))


FamilyResult = Optional[Tuple[List[float], str]]


def _lsl(x: float, y: float, phi: float) -> FamilyResult:
    u, theta = _polar(x - math.sin(phi), y - 1.0 + math.cos(phi))
    v = wrap_angle(phi - theta)
    if 0.0 <= theta <= math.pi and 0.0 <= v <= math.pi:
        return [theta, u, v], 'LSL'
    return None


def _lsr(x: float, y: float, phi: float) -> FamilyResult:
    radius, beta = _polar(x + math.sin(phi), y - 1.0 - math.cos(phi))
    if radius * radius >= 4.0:
        u = math.sqrt(radius * radius - 4.0)
        theta = wrap_angle(beta + math.atan2(2.0, u))
        v = wrap_angle(theta - phi)
        if theta >= 0.0 and v >= 0.0:
            return [theta, u, v], 'LSR'
    return None


def _lrl_a(x: float, y: float, phi: float) -> FamilyResult:
    radius, beta = _polar(x - math.sin(phi), y - 1.0 + math.cos(phi))
    if radius <= 4.0:
        alpha = _acos(radius / 4.0)
        theta = wrap_angle(alpha + beta + math.pi / 2.0)
        u = wrap_angle(math.pi - 2.0 * alpha)
        v = wrap_angle(phi - theta - u)
        return [theta, -u, v], 'LRL'
    return None


def _lrl_b(x: float, y: float, phi: float) -> FamilyResult:
    radius, beta = _polar(x - math.sin(phi), y - 1.0 + math.cos(phi))
    if radius <= 4.0:
        alpha = _acos(radius / 4.0)
        theta = wrap_angle(alpha + beta + math.pi / 2.0)
        u = wrap_angle(math.pi - 2.0 * alpha)
        v = wrap_angle(-phi + theta + u)
        return [theta, -u, -v], 'LRL'
    return None


def _lrl_c(x: float, y: float, phi: float) -> FamilyResult:
    radius, beta = _polar(x - math.sin(phi), y - 1.0 + math.cos(phi))
    if 1e-8 < radius <= 4.0:
        u = _acos(1.0 - radius * radius / 8.0)
        alpha = _asin(2.0 * math.sin(u) / radius)
        theta = wrap_angle(-alpha + beta + math.pi / 2.0)
        v = wrap_angle(theta - u - phi)
        return [theta, u, -v], 'LRL'
    return None


def _lrlr_a(x: float, y: float, phi: float) -> FamilyResult:
    radius, beta = _polar(x + math.sin(phi), y - 1.0 - math.cos(phi))
    if radius <= 2.0:
        alpha = _acos((radius + 2.0) / 4.0)
        theta = wrap_angle(beta + alpha + math.pi / 2.0)
        u = wrap_angle(alpha)
        v = wrap_angle(phi - theta + 2.0 * u)
        if theta >= 0.0 and u >= 0.0 and v >= 0.0:
            return [theta, u, -u, -v], 'LRLR'
    return None


def _lrlr_b(x: float, y: float, phi: float) -> FamilyResult:
    radius, beta = _polar(x + math.sin(phi), y - 1.0 - math.cos(phi))
    u2 = (20.0 - radius * radius) / 16.0
    if 0.0 <= u2 <= 1.0 and radius > 1e-8:
        u = _acos(u2)
        alpha = _asin(2.0 * math.sin(u) / radius)
        theta = wrap_angle(beta + alpha + math.pi / 2.0)
        v = wrap_angle(theta - phi)
        if theta >= 0.0 and v >= 0.0:
            return [theta, -u, -u, v], 'LRLR'
    return None


def _lrsl(x: float, y: float, phi: float) -> FamilyResult:
    radius, beta = _polar(x - math.sin(phi), y - 1.0 + math.cos(phi))
    if radius >= 2.0:
        q = math.sqrt(radius * radius - 4.0)
        u = q - 2.0
        alpha = math.atan2(2.0, q)
        theta = wrap_angle(beta + alpha + math.pi / 2.0)
        v = wrap_angle(theta - phi + math.pi / 2.0)
        if theta >= 0.0 and v >= 0.0:
            return [theta, -math.pi / 2.0, -u, -v], 'LRSL'
    return None


def _lrsr(x: float, y: float, phi: float) -> FamilyResult:
    radius, beta = _polar(x + math.sin(phi), y - 1.0 - math.cos(phi))
    if radius >= 2.0:
        theta = wrap_angle(beta + math.pi / 2.0)
        u = radius - 2.0
        v = wrap_angle(phi - theta - math.pi / 2.0)
        if theta >= 0.0 and v >= 0.0:
            return [theta, -math.pi / 2.0, -u, -v], 'LRSR'
    return None


def _lsrl(x: float, y: float, phi: float) -> FamilyResult:
    radius, beta = _polar(x - math.sin(phi), y - 1.0 + math.cos(phi))
    if radius >= 2.0:
        q = math.sqrt(radius * radius - 4.0)
        u = q - 2.0
        alpha = math.atan2(q, 2.0)
        theta = wrap_angle(beta - alpha + math.pi / 2.0)
        v = wrap_angle(theta - phi - math.pi / 2.0)
        if theta >= 0.0 and v >= 0.0:
            return [theta, u, math.pi / 2.0, -v], 'LSRL'
    return None


def _lslr(x: float, y: float, phi: float) -> FamilyResult:
    radius, beta = _polar(x + math.sin(phi), y - 1.0 - math.cos(phi))
    if radius >= 2.0:
        theta = wrap_angle(beta)
        u = radius - 2.0
        v = wrap_angle(phi - theta - math.pi / 2.0)
        if theta >= 0.0 and v >= 0.0:
            return [theta, u, math.pi / 2.0, -v], 'LSLR'
    return None


def _lrslr(x: float, y: float, phi: float) -> FamilyResult:
    radius, beta = _polar(x + math.sin(phi), y - 1.0 - math.cos(phi))
    if radius >= 4.0:
        q = math.sqrt(radius * radius - 4.0)
        u = q - 4.0
        alpha = math.atan2(2.0, q)
        theta = wrap_angle(beta + alpha + math.pi / 2.0)
        v = wrap_angle(theta - phi)
        if theta >= 0.0 and v >= 0.0:
            return [theta, -math.pi / 2.0, -u, -math.pi / 2.0, v], 'LRSLR'
    return None


_FAMILIES: Tuple[Callable[[float, float, float], FamilyResult], ...] = (
    _lsl, _lsr, _lrl_a, _lrl_b, _lrl_c, _lrlr_a, _lrlr_b,
    _lrsl, _lrsr, _lsrl, _lslr, _lrslr,
)


def _integrate(pose: PathPoint, distance: float, curvature: float) -> PathPoint:
    if abs(curvature) < 1e-12:
        return PathPoint(
            pose.x + distance * math.cos(pose.yaw),
            pose.y + distance * math.sin(pose.yaw),
            pose.yaw,
        )
    yaw = pose.yaw + distance * curvature
    return PathPoint(
        pose.x + (math.sin(yaw) - math.sin(pose.yaw)) / curvature,
        pose.y + (-math.cos(yaw) + math.cos(pose.yaw)) / curvature,
        wrap_angle(yaw),
    )


def _local_goal(start: PathPoint, goal: PathPoint) -> Tuple[float, float, float]:
    dx = goal.x - start.x
    dy = goal.y - start.y
    cosine = math.cos(start.yaw)
    sine = math.sin(start.yaw)
    return (
        dx * cosine + dy * sine,
        -dx * sine + dy * cosine,
        wrap_angle(goal.yaw - start.yaw),
    )


def _gear_changes(points: Sequence[ParkingPathPoint]) -> int:
    directions = [point.direction for point in points if point.direction]
    return sum(a != b for a, b in zip(directions, directions[1:]))


def _build_candidate(
    start: PathPoint,
    goal: PathPoint,
    radius_m: float,
    normalized_lengths: Sequence[float],
    types: Sequence[str],
    reverse_all: bool,
    step_m: float,
    candidate_id: int,
    config: ParkingConfig,
) -> Optional[ParkingCandidate]:
    lengths = tuple(
        float(value) * radius_m * (-1.0 if reverse_all else 1.0)
        for value in normalized_lengths
    )
    pose = start
    points: List[ParkingPathPoint] = []
    for distance, segment_type in zip(lengths, types):
        if abs(distance) < 1e-9:
            continue
        direction = 1 if distance > 0.0 else -1
        curvature = (
            1.0 / radius_m if segment_type == 'L'
            else -1.0 / radius_m if segment_type == 'R'
            else 0.0
        )
        steps = max(1, int(math.ceil(abs(distance) / step_m)))
        origin = pose
        if not points:
            points.append(ParkingPathPoint(
                pose.x, pose.y, pose.yaw, direction, curvature
            ))
        else:
            points.append(ParkingPathPoint(
                pose.x, pose.y, pose.yaw, direction, curvature
            ))
        for index in range(1, steps + 1):
            pose = _integrate(origin, distance * index / steps, curvature)
            points.append(ParkingPathPoint(
                pose.x, pose.y, pose.yaw, direction, curvature
            ))
    if not points:
        return None
    position_error = math.hypot(pose.x - goal.x, pose.y - goal.y)
    yaw_error = abs(wrap_angle(pose.yaw - goal.yaw))
    if (
        position_error > config.endpoint_position_tolerance_m
        or yaw_error > config.endpoint_yaw_tolerance_rad
    ):
        return None
    reverse_distance = sum(-distance for distance in lengths if distance < 0.0)
    changes = _gear_changes(points)
    path_length = sum(abs(distance) for distance in lengths)
    return ParkingCandidate(
        candidate_id=candidate_id,
        family=''.join(types),
        segment_lengths_m=lengths,
        points=points,
        path_length_m=path_length,
        reverse_distance_m=reverse_distance,
        gear_changes=changes,
        cost=(
            path_length
            + changes * config.gear_change_penalty_m
            + reverse_distance * config.reverse_penalty_scale
        ),
        endpoint_position_error_m=position_error,
        endpoint_yaw_error_rad=yaw_error,
    )


def reeds_shepp_candidates(
    start: PathPoint,
    goal: PathPoint,
    minimum_turn_radius_m: float,
    config: ParkingConfig = ParkingConfig(),
) -> List[ParkingCandidate]:
    """Generate the 12 analytic families with time reversal and reflection."""
    config.validate()
    if not math.isfinite(minimum_turn_radius_m) or minimum_turn_radius_m <= 0.0:
        raise ParkingError('minimum turn radius must be finite and positive')
    if not all(
        math.isfinite(value)
        for pose in (start, goal)
        for value in (pose.x, pose.y, pose.yaw)
    ):
        raise ParkingError('start and goal poses must be finite')
    local_x, local_y, phi = _local_goal(start, goal)
    x = local_x / minimum_turn_radius_m
    y = local_y / minimum_turn_radius_m
    candidates = []
    seen = set()
    for family in _FAMILIES:
        for variant in range(4):
            reverse_all = variant in (1, 3)
            reflect = variant >= 2
            result = family(
                -x if reverse_all else x,
                -y if reflect else y,
                -phi if reverse_all != reflect else phi,
            )
            if result is None:
                continue
            normalized_lengths, family_name = result
            types = [
                ('R' if value == 'L' else 'L' if value == 'R' else 'S')
                if reflect else value
                for value in family_name
            ]
            candidate = _build_candidate(
                start, goal, minimum_turn_radius_m, normalized_lengths, types,
                reverse_all, config.sample_step_m, len(candidates), config,
            )
            if candidate is not None:
                key = (
                    candidate.family,
                    tuple(round(value, 8) for value in candidate.segment_lengths_m),
                )
                if key in seen:
                    continue
                seen.add(key)
                candidates.append(candidate)
    return sorted(candidates, key=lambda item: item.cost)


def _append_reverse_docking(
    candidate: ParkingCandidate,
    pre_goal: PathPoint,
    goal: PathPoint,
    distance_m: float,
    config: ParkingConfig,
) -> ParkingCandidate:
    if distance_m <= 0.0:
        return candidate
    steps = max(1, int(math.ceil(distance_m / config.sample_step_m)))
    points = list(candidate.points)
    for index in range(1, steps + 1):
        ratio = index / steps
        points.append(ParkingPathPoint(
            pre_goal.x + (goal.x - pre_goal.x) * ratio,
            pre_goal.y + (goal.y - pre_goal.y) * ratio,
            goal.yaw,
            -1,
            0.0,
        ))
    lengths = candidate.segment_lengths_m + (-distance_m,)
    reverse_distance = candidate.reverse_distance_m + distance_m
    changes = _gear_changes(points)
    return replace(
        candidate,
        family=candidate.family + 'S',
        segment_lengths_m=lengths,
        points=points,
        path_length_m=candidate.path_length_m + distance_m,
        reverse_distance_m=reverse_distance,
        gear_changes=changes,
        cost=(
            candidate.path_length_m + distance_m
            + changes * config.gear_change_penalty_m
            + reverse_distance * config.reverse_penalty_scale
        ),
        endpoint_position_error_m=0.0,
        endpoint_yaw_error_rad=0.0,
        stage=f'{round(distance_m * 100.0)} cm reverse docking straight',
    )


def validate_candidate(
    candidate: ParkingCandidate,
    drivable_mask: np.ndarray,
    profile: CameraProfile,
    geometry: VehicleGeometry,
    config: ParkingConfig,
    obstacles_m: Sequence[Tuple[float, float]] = (),
    require_reverse: bool = False,
) -> ParkingCandidate:
    geometry.validate()
    config.validate()
    road_blocked = 0
    obstacle_blocked = 0
    minimum_support = 1.0
    for point in candidate.points:
        pose = PathPoint(point.x, point.y, point.yaw)
        samples = footprint_points(
            pose, geometry, config.footprint_sample_spacing_m
        )
        support = road_support(samples, drivable_mask, profile)
        minimum_support = min(minimum_support, support)
        if support < config.minimum_road_support:
            road_blocked += 1
        if obstacle_in_footprint(
            pose, obstacles_m, geometry, config.obstacle_margin_m
        ):
            obstacle_blocked += 1
    reasons = []
    if road_blocked:
        reasons.append('swept footprint leaves observed parking area')
    if obstacle_blocked:
        reasons.append('obstacle intersects swept footprint')
    if candidate.gear_changes > config.maximum_gear_changes:
        reasons.append('gear-change limit exceeded')
    if candidate.reverse_distance_m > config.maximum_reverse_distance_m:
        reasons.append('reverse-distance limit exceeded')
    if require_reverse and candidate.reverse_distance_m <= 0.0:
        reasons.append('parking maneuver requires a reverse segment')
    return replace(
        candidate,
        valid=not reasons,
        road_blocked_samples=road_blocked,
        obstacle_blocked_samples=obstacle_blocked,
        minimum_road_support=minimum_support,
        reject_reason='; '.join(reasons),
    )


def goal_fits_slot(
    slot_length_m: float,
    slot_width_m: float,
    geometry: VehicleGeometry,
    clearance_m: float = 0.005,
) -> bool:
    geometry.validate()
    values = (slot_length_m, slot_width_m, clearance_m)
    if not all(math.isfinite(value) for value in values):
        raise ParkingError('slot dimensions and clearance must be finite')
    if slot_length_m <= 0.0 or slot_width_m <= 0.0 or clearance_m < 0.0:
        raise ParkingError('slot dimensions must be positive')
    return (
        geometry.length_m + 2.0 * clearance_m <= slot_length_m
        and geometry.width_m + 2.0 * clearance_m <= slot_width_m
    )


def plan_parking(
    start: PathPoint,
    goal: PathPoint,
    drivable_mask: np.ndarray,
    profile: CameraProfile,
    geometry: VehicleGeometry = VehicleGeometry(),
    config: ParkingConfig = ParkingConfig(),
    obstacles_m: Sequence[Tuple[float, float]] = (),
    require_reverse: bool = True,
) -> List[ParkingCandidate]:
    """Return all checked docking/direct candidates, valid-first by cost."""
    geometry.validate()
    config.validate()
    evaluated = []
    for docking_distance in config.docking_distances_m:
        if docking_distance > 0.0:
            pre_goal = _integrate(goal, docking_distance, 0.0)
        else:
            pre_goal = goal
        raw = reeds_shepp_candidates(
            start, pre_goal, geometry.minimum_turn_radius_m, config
        )
        for candidate in raw:
            candidate = _append_reverse_docking(
                candidate, pre_goal, goal, docking_distance, config
            )
            candidate.candidate_id = len(evaluated)
            evaluated.append(validate_candidate(
                candidate, drivable_mask, profile, geometry, config,
                obstacles_m, require_reverse,
            ))
    return sorted(evaluated, key=lambda item: (not item.valid, item.cost))
