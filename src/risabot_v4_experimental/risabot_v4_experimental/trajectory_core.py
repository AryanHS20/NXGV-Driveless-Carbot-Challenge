"""Footprint-aware short-horizon trajectory candidates for V4 Stage 4."""

from dataclasses import dataclass
import math
from typing import List, Sequence, Tuple

import numpy as np

from .bev_core import CameraProfile, metric_to_bev


class TrajectoryError(ValueError):
    """Raised when trajectory input or geometry is invalid."""


@dataclass(frozen=True)
class PathPoint:
    x: float
    y: float
    yaw: float = 0.0


@dataclass(frozen=True)
class VehicleGeometry:
    length_m: float = 0.300
    width_m: float = 0.192
    wheelbase_m: float = 0.216
    rear_overhang_m: float = 0.042
    minimum_turn_radius_m: float = 0.400
    footprint_padding_m: float = 0.005

    def validate(self) -> None:
        values = vars(self)
        if not all(math.isfinite(value) for value in values.values()):
            raise TrajectoryError('vehicle geometry must be finite')
        if min(self.length_m, self.width_m, self.wheelbase_m, self.minimum_turn_radius_m) <= 0.0:
            raise TrajectoryError('vehicle dimensions and turn radius must be positive')
        if self.rear_overhang_m < 0.0 or self.rear_overhang_m >= self.length_m:
            raise TrajectoryError('rear overhang must lie within the vehicle length')
        if self.footprint_padding_m < 0.0:
            raise TrajectoryError('footprint padding must be non-negative')


@dataclass(frozen=True)
class TrajectoryConfig:
    lateral_offsets_m: Tuple[float, ...] = (
        -0.060, -0.040, -0.020, -0.008, 0.0, 0.008, 0.020, 0.040, 0.060,
    )
    horizon_m: float = 0.55
    step_m: float = 0.01
    lookahead_m: float = 0.095
    rollout_speed_mps: float = 0.08
    steering_lag_sec: float = 0.12
    steering_rate_rad_sec: float = math.pi
    footprint_sample_spacing_m: float = 0.02
    minimum_road_support: float = 0.98
    obstacle_margin_m: float = 0.015

    def validate(self) -> None:
        if not self.lateral_offsets_m:
            raise TrajectoryError('at least one lateral offset is required')
        if not all(math.isfinite(value) for value in self.lateral_offsets_m):
            raise TrajectoryError('lateral offsets must be finite')
        for name, value in (
            ('horizon_m', self.horizon_m), ('step_m', self.step_m),
            ('lookahead_m', self.lookahead_m),
            ('rollout_speed_mps', self.rollout_speed_mps),
            ('steering_lag_sec', self.steering_lag_sec),
            ('steering_rate_rad_sec', self.steering_rate_rad_sec),
            ('footprint_sample_spacing_m', self.footprint_sample_spacing_m),
        ):
            if not math.isfinite(value) or value <= 0.0:
                raise TrajectoryError(f'{name} must be finite and positive')
        if not 0.0 < self.minimum_road_support <= 1.0:
            raise TrajectoryError('minimum road support must be in (0, 1]')
        if not math.isfinite(self.obstacle_margin_m) or self.obstacle_margin_m < 0.0:
            raise TrajectoryError('obstacle margin must be finite and non-negative')


@dataclass
class Candidate:
    candidate_id: int
    offset_m: float
    points: List[PathPoint]
    curvatures: List[float]
    cost: float
    valid: bool
    road_blocked: int
    obstacle_blocked: int
    minimum_support: float
    command_steer_rad: float
    reject_reason: str


def reference_from_corridor(samples: Sequence[Tuple[float, float]]) -> List[PathPoint]:
    """Convert near-to-far [forward, left] corridor samples to a headed path."""
    clean = []
    for sample in samples:
        if len(sample) != 2:
            raise TrajectoryError('corridor samples must be [forward, left] pairs')
        x, y = float(sample[0]), float(sample[1])
        if not math.isfinite(x) or not math.isfinite(y):
            raise TrajectoryError('corridor samples must be finite')
        clean.append((x, y))
    clean.sort(key=lambda item: item[0])
    if len(clean) < 2:
        raise TrajectoryError('at least two corridor samples are required')
    result = []
    for index, (x, y) in enumerate(clean):
        before = clean[max(0, index - 1)]
        after = clean[min(len(clean) - 1, index + 1)]
        result.append(PathPoint(x, y, math.atan2(after[1] - before[1], after[0] - before[0])))
    return result


def footprint_points(
    pose: PathPoint,
    geometry: VehicleGeometry,
    spacing_m: float,
) -> np.ndarray:
    """Sample the complete rectangular body, referenced at the rear axle."""
    geometry.validate()
    if not math.isfinite(spacing_m) or spacing_m <= 0.0:
        raise TrajectoryError('footprint sample spacing must be positive')
    rear = -geometry.rear_overhang_m - geometry.footprint_padding_m
    front = geometry.length_m - geometry.rear_overhang_m + geometry.footprint_padding_m
    half_width = 0.5 * geometry.width_m + geometry.footprint_padding_m
    nx = max(2, int(math.ceil((front - rear) / spacing_m)) + 1)
    ny = max(2, int(math.ceil((2.0 * half_width) / spacing_m)) + 1)
    longitudinal, lateral = np.meshgrid(
        np.linspace(rear, front, nx), np.linspace(-half_width, half_width, ny)
    )
    cosine = math.cos(pose.yaw)
    sine = math.sin(pose.yaw)
    world_x = pose.x + longitudinal * cosine - lateral * sine
    world_y = pose.y + longitudinal * sine + lateral * cosine
    return np.column_stack((world_x.ravel(), world_y.ravel()))


def road_support(
    points_m: np.ndarray,
    drivable_mask: np.ndarray,
    profile: CameraProfile,
) -> float:
    if drivable_mask is None or drivable_mask.ndim != 2:
        raise TrajectoryError('drivable mask must be single-channel')
    if tuple(reversed(drivable_mask.shape)) != profile.output_size:
        raise TrajectoryError('drivable mask shape does not match BEV profile')
    pixels = metric_to_bev(profile, points_m)
    columns = np.rint(pixels[:, 0]).astype(int)
    rows = np.rint(pixels[:, 1]).astype(int)
    inside = (
        (columns >= 0) & (columns < drivable_mask.shape[1])
        & (rows >= 0) & (rows < drivable_mask.shape[0])
    )
    supported = np.zeros(points_m.shape[0], dtype=bool)
    supported[inside] = drivable_mask[rows[inside], columns[inside]] > 0
    return float(supported.mean()) if supported.size else 0.0


def obstacle_in_footprint(
    pose: PathPoint,
    obstacles_m: Sequence[Tuple[float, float]],
    geometry: VehicleGeometry,
    margin_m: float,
) -> bool:
    cosine = math.cos(pose.yaw)
    sine = math.sin(pose.yaw)
    rear = -geometry.rear_overhang_m - margin_m
    front = geometry.length_m - geometry.rear_overhang_m + margin_m
    half_width = 0.5 * geometry.width_m + margin_m
    for obstacle in obstacles_m:
        if len(obstacle) != 2:
            raise TrajectoryError('obstacles must be [forward, left] pairs')
        dx = float(obstacle[0]) - pose.x
        dy = float(obstacle[1]) - pose.y
        local_x = dx * cosine + dy * sine
        local_y = -dx * sine + dy * cosine
        if rear <= local_x <= front and abs(local_y) <= half_width:
            return True
    return False


def _closest(reference: Sequence[PathPoint], pose: PathPoint, start: int) -> int:
    return min(
        range(max(0, start), len(reference)),
        key=lambda index: (reference[index].x - pose.x) ** 2 + (reference[index].y - pose.y) ** 2,
    )


def generate_candidates(
    reference: Sequence[PathPoint],
    drivable_mask: np.ndarray,
    profile: CameraProfile,
    geometry: VehicleGeometry = VehicleGeometry(),
    config: TrajectoryConfig = TrajectoryConfig(),
    obstacles_m: Sequence[Tuple[float, float]] = (),
) -> List[Candidate]:
    geometry.validate()
    config.validate()
    if len(reference) < 2:
        raise TrajectoryError('reference path needs at least two points')
    if not all(math.isfinite(value) for point in reference for value in (point.x, point.y, point.yaw)):
        raise TrajectoryError('reference path must be finite')

    maximum_steer = math.atan(geometry.wheelbase_m / geometry.minimum_turn_radius_m)
    maximum_curvature = 1.0 / geometry.minimum_turn_radius_m
    steps = max(1, int(math.ceil(config.horizon_m / config.step_m)))
    dt = config.step_m / config.rollout_speed_mps
    candidates = []
    for candidate_id, offset in enumerate(config.lateral_offsets_m):
        pose = PathPoint(0.0, 0.0, 0.0)
        points = [pose]
        curvatures = []
        steer = 0.0
        reference_index = 0
        cost = abs(offset) * 25.0
        road_blocked = 0
        obstacle_blocked = 0
        minimum_support = 1.0
        command_steer = 0.0
        for step_index in range(steps):
            reference_index = _closest(reference, pose, reference_index)
            target_index = reference_index
            while target_index < len(reference) - 1:
                target = reference[target_index]
                if math.hypot(target.x - pose.x, target.y - pose.y) >= config.lookahead_m:
                    break
                target_index += 1
            target = reference[target_index]
            target_x = target.x - offset * math.sin(target.yaw)
            target_y = target.y + offset * math.cos(target.yaw)
            dx = target_x - pose.x
            dy = target_y - pose.y
            local_x = dx * math.cos(pose.yaw) + dy * math.sin(pose.yaw)
            local_y = -dx * math.sin(pose.yaw) + dy * math.cos(pose.yaw)
            desired = math.atan(
                geometry.wheelbase_m * 2.0 * local_y
                / max(0.002, local_x ** 2 + local_y ** 2)
            )
            desired = max(-maximum_steer, min(maximum_steer, desired))
            if step_index == 0:
                command_steer = desired
            rate_limit = config.steering_rate_rad_sec * dt
            exponential_step = (desired - steer) * (
                1.0 - math.exp(-dt / config.steering_lag_sec)
            )
            steer += max(-rate_limit, min(rate_limit, exponential_step))
            curvature = max(
                -maximum_curvature,
                min(maximum_curvature, math.tan(steer) / geometry.wheelbase_m),
            )
            midpoint_yaw = pose.yaw + 0.5 * config.step_m * curvature
            pose = PathPoint(
                pose.x + config.step_m * math.cos(midpoint_yaw),
                pose.y + config.step_m * math.sin(midpoint_yaw),
                math.atan2(
                    math.sin(pose.yaw + config.step_m * curvature),
                    math.cos(pose.yaw + config.step_m * curvature),
                ),
            )
            points.append(pose)
            curvatures.append(curvature)

            samples = footprint_points(pose, geometry, config.footprint_sample_spacing_m)
            support = road_support(samples, drivable_mask, profile)
            minimum_support = min(minimum_support, support)
            if support < config.minimum_road_support:
                road_blocked += 1
            if obstacle_in_footprint(pose, obstacles_m, geometry, config.obstacle_margin_m):
                obstacle_blocked += 1
            tracking = math.hypot(target.x - pose.x, target.y - pose.y)
            cost += tracking ** 2 * 8.0 + (1.0 - support) * 40.0 + abs(curvature) * 0.001

        reasons = []
        if road_blocked:
            reasons.append('swept footprint leaves observed road')
        if obstacle_blocked:
            reasons.append('obstacle intersects swept footprint')
        candidates.append(Candidate(
            candidate_id=candidate_id,
            offset_m=offset,
            points=points,
            curvatures=curvatures,
            cost=cost,
            valid=not road_blocked and not obstacle_blocked,
            road_blocked=road_blocked,
            obstacle_blocked=obstacle_blocked,
            minimum_support=minimum_support,
            command_steer_rad=command_steer,
            reject_reason='; '.join(reasons),
        ))
    return sorted(candidates, key=lambda candidate: (not candidate.valid, candidate.cost))
