"""Footprint-aware short-horizon trajectory candidates for V4 Stage 4."""

from dataclasses import dataclass
import math
from typing import List, Optional, Sequence, Tuple

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
    wheelbase_m: float = 0.210
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
class NearFieldBootstrap:
    """Bounded road evidence for the camera blind strip below the vehicle.

    The rear axle is the metric origin.  Road pixels remain authoritative from
    ``forward_m`` onward; this object only describes the continuous strip from
    the current vehicle footprint to that first stable observed row.
    """

    rear_m: float
    forward_m: float
    end_left_m: float
    width_m: float

    def validate(self) -> None:
        if not all(math.isfinite(value) for value in vars(self).values()):
            raise TrajectoryError('near-field bootstrap must be finite')
        if self.rear_m >= 0.0 or self.forward_m <= 0.0 or self.width_m <= 0.0:
            raise TrajectoryError('near-field bootstrap bounds are invalid')


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
    steering_gain: float = 1.0
    footprint_sample_spacing_m: float = 0.02
    minimum_road_support: float = 0.98
    road_support_cost_weight: float = 40.0
    expected_lane_width_m: float = 0.32
    centerline_filter_alpha: float = 0.60
    cross_track_gain: float = 1.10
    heading_gain: float = 0.85
    curvature_feedforward_gain: float = 0.90
    reliable_support_threshold: float = 0.75
    minimum_observed_centerline_fraction: float = 0.50
    low_support_direction_hold_sec: float = 2.50
    low_support_steer_decay_sec: float = 1.50
    obstacle_margin_m: float = 0.015
    near_field_max_gap_m: float = 0.55
    near_field_settle_m: float = 0.04

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
            ('steering_gain', self.steering_gain),
            ('footprint_sample_spacing_m', self.footprint_sample_spacing_m),
            ('road_support_cost_weight', self.road_support_cost_weight),
            ('expected_lane_width_m', self.expected_lane_width_m),
            ('cross_track_gain', self.cross_track_gain),
            ('heading_gain', self.heading_gain),
            ('curvature_feedforward_gain', self.curvature_feedforward_gain),
            ('low_support_direction_hold_sec', self.low_support_direction_hold_sec),
            ('low_support_steer_decay_sec', self.low_support_steer_decay_sec),
            ('near_field_max_gap_m', self.near_field_max_gap_m),
            ('near_field_settle_m', self.near_field_settle_m),
        ):
            if not math.isfinite(value) or value <= 0.0:
                raise TrajectoryError(f'{name} must be finite and positive')
        if not 0.0 < self.minimum_road_support <= 1.0:
            raise TrajectoryError('minimum road support must be in (0, 1]')
        if not 0.0 < self.centerline_filter_alpha <= 1.0:
            raise TrajectoryError('centerline filter alpha must be in (0, 1]')
        if not 0.0 <= self.reliable_support_threshold <= 1.0:
            raise TrajectoryError('reliable support threshold must be in [0, 1]')
        if not 0.0 <= self.minimum_observed_centerline_fraction <= 1.0:
            raise TrajectoryError(
                'minimum observed centerline fraction must be in [0, 1]'
            )
        if not math.isfinite(self.obstacle_margin_m) or self.obstacle_margin_m < 0.0:
            raise TrajectoryError('obstacle margin must be finite and non-negative')


def steering_reference_from_corridor(
    samples: Sequence[dict], expected_lane_width_m: float
) -> List[PathPoint]:
    """Build a lane-centre reference from two edges or one observed edge.

    A turn can move one white boundary outside the camera coverage. In that
    case the visible run midpoint is biased, but its remaining observed edge
    and the measured lane width still define the lane centre.
    """
    if not math.isfinite(expected_lane_width_m) or expected_lane_width_m <= 0.0:
        raise TrajectoryError('expected lane width must be finite and positive')
    centers = []
    for sample in samples:
        forward = float(sample['forward_m'])
        visible_center = float(sample['left_m'])
        visible_width = float(sample['width_m'])
        if not all(math.isfinite(value) for value in (
                forward, visible_center, visible_width)) or visible_width <= 0.0:
            continue
        both = bool(sample.get('boundaries_observed', True))
        left_seen = bool(sample.get('left_boundary_observed', both))
        right_seen = bool(sample.get('right_boundary_observed', both))
        if left_seen and right_seen:
            center = visible_center
        elif left_seen:
            center = visible_center + 0.5 * visible_width - 0.5 * expected_lane_width_m
        elif right_seen:
            center = visible_center - 0.5 * visible_width + 0.5 * expected_lane_width_m
        else:
            continue
        centers.append((forward, center))
    return reference_from_corridor(centers)


def smooth_centerline_reference(
    reference: Sequence[PathPoint],
    previous_coefficients: Sequence[float] = (),
    alpha: float = 0.60,
) -> Tuple[List[PathPoint], Tuple[float, float, float]]:
    """Fit and temporally filter y=c0+c1*x+c2*x^2 in vehicle coordinates."""
    if len(reference) < 2:
        raise TrajectoryError('centerline smoothing requires at least two points')
    if not math.isfinite(alpha) or not 0.0 < alpha <= 1.0:
        raise TrajectoryError('centerline filter alpha must be in (0, 1]')
    x = np.asarray([point.x for point in reference], dtype=np.float64)
    y = np.asarray([point.y for point in reference], dtype=np.float64)
    if np.unique(x).size < 2:
        raise TrajectoryError('centerline samples need two forward positions')
    if len(reference) >= 3:
        design = np.column_stack((np.ones_like(x), x, x * x))
        coefficients = np.linalg.lstsq(design, y, rcond=None)[0]
    else:
        design = np.column_stack((np.ones_like(x), x))
        linear = np.linalg.lstsq(design, y, rcond=None)[0]
        coefficients = np.asarray((linear[0], linear[1], 0.0))
    if len(previous_coefficients) == 3 and all(
            math.isfinite(float(value)) for value in previous_coefficients):
        previous = np.asarray(previous_coefficients, dtype=np.float64)
        coefficients = alpha * coefficients + (1.0 - alpha) * previous
    smoothed = reference_from_corridor([
        (float(value), float(coefficients[0] + coefficients[1] * value
                             + coefficients[2] * value * value))
        for value in x
    ])
    return smoothed, tuple(float(value) for value in coefficients)


def centerline_steering_command(
    reference: Sequence[PathPoint],
    coefficients: Sequence[float],
    geometry: VehicleGeometry,
    lookahead_m: float,
    cross_track_gain: float,
    heading_gain: float,
    curvature_feedforward_gain: float,
    expected_lane_width_m: float = 0.32,
) -> Tuple[float, dict]:
    """Return a Stanley-style steering angle with curvature feedforward."""
    if len(reference) < 2 or len(coefficients) != 3:
        raise TrajectoryError('centerline control needs a fitted reference')
    values = (lookahead_m, cross_track_gain, heading_gain,
              curvature_feedforward_gain, expected_lane_width_m, *coefficients)
    if not all(math.isfinite(float(value)) for value in values):
        raise TrajectoryError('centerline control values must be finite')
    if lookahead_m <= 0.0 or expected_lane_width_m <= 0.0 or min(cross_track_gain, heading_gain,
                                 curvature_feedforward_gain) < 0.0:
        raise TrajectoryError('centerline control gains are invalid')
    c0, c1, c2 = (float(value) for value in coefficients)
    minimum_x = min(point.x for point in reference)
    maximum_x = max(point.x for point in reference)
    evaluation_x = max(minimum_x, min(maximum_x, lookahead_m))
    slope = c1 + 2.0 * c2 * evaluation_x
    heading_error = math.atan(slope)
    curvature = 2.0 * c2 / ((1.0 + slope * slope) ** 1.5)
    # The closest observed corridor is commonly 35-45 cm ahead. Using the
    # fitted intercept at x=0 extrapolates through that blind strip and can
    # invent a very large error as a bend enters view. Track the fitted lane
    # at the nearest evaluated lookahead and bound only the feedback term.
    lateral_error = c0 + c1 * evaluation_x + c2 * evaluation_x * evaluation_x
    padded_body_width = geometry.width_m + 2.0 * geometry.footprint_padding_m
    feasible_cross_track = max(
        0.02, 0.5 * (expected_lane_width_m - padded_body_width)
    )
    control_lateral_error = max(
        -feasible_cross_track, min(feasible_cross_track, lateral_error)
    )
    feedforward = math.atan(geometry.wheelbase_m * curvature)
    cross_track = math.atan2(cross_track_gain * control_lateral_error,
                             max(0.12, evaluation_x))
    command = (curvature_feedforward_gain * feedforward
               + heading_gain * heading_error + cross_track)
    maximum_steer = math.atan(
        geometry.wheelbase_m / geometry.minimum_turn_radius_m
    )
    command = max(-maximum_steer, min(maximum_steer, command))
    diagnostics = {
        'lateral_error_m': lateral_error,
        'control_lateral_error_m': control_lateral_error,
        'intercept_lateral_error_m': c0,
        'heading_error_rad': heading_error,
        'curvature_per_m': curvature,
        'feedforward_steer_rad': feedforward,
        'evaluation_forward_m': evaluation_x,
    }
    return command, diagnostics


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
    local = _footprint_template(geometry, spacing_m)
    cosine = math.cos(pose.yaw)
    sine = math.sin(pose.yaw)
    world_x = pose.x + local[:, 0] * cosine - local[:, 1] * sine
    world_y = pose.y + local[:, 0] * sine + local[:, 1] * cosine
    return np.column_stack((world_x, world_y))


def _footprint_template(
    geometry: VehicleGeometry,
    spacing_m: float,
) -> np.ndarray:
    """Return body samples in vehicle coordinates for reuse across poses."""
    rear = -geometry.rear_overhang_m - geometry.footprint_padding_m
    front = geometry.length_m - geometry.rear_overhang_m + geometry.footprint_padding_m
    half_width = 0.5 * geometry.width_m + geometry.footprint_padding_m
    nx = max(2, int(math.ceil((front - rear) / spacing_m)) + 1)
    ny = max(2, int(math.ceil((2.0 * half_width) / spacing_m)) + 1)
    longitudinal, lateral = np.meshgrid(
        np.linspace(rear, front, nx), np.linspace(-half_width, half_width, ny)
    )
    return np.column_stack((longitudinal.ravel(), lateral.ravel()))


def _candidate_supports(
    poses: Sequence[PathPoint],
    local_footprint: np.ndarray,
    drivable_mask: np.ndarray,
    profile: CameraProfile,
    near_field: Optional[NearFieldBootstrap],
) -> np.ndarray:
    """Calculate support for every pose in one vectorized BEV lookup."""
    pose_x = np.asarray([pose.x for pose in poses], dtype=np.float64)
    pose_y = np.asarray([pose.y for pose in poses], dtype=np.float64)
    yaw = np.asarray([pose.yaw for pose in poses], dtype=np.float64)
    cosine = np.cos(yaw)[:, None]
    sine = np.sin(yaw)[:, None]
    local_x = local_footprint[:, 0][None, :]
    local_y = local_footprint[:, 1][None, :]
    world_x = pose_x[:, None] + local_x * cosine - local_y * sine
    world_y = pose_y[:, None] + local_x * sine + local_y * cosine
    flattened = np.column_stack((world_x.ravel(), world_y.ravel()))

    if drivable_mask is None or drivable_mask.ndim != 2:
        raise TrajectoryError('drivable mask must be single-channel')
    if tuple(reversed(drivable_mask.shape)) != profile.output_size:
        raise TrajectoryError('drivable mask shape does not match BEV profile')
    pixels = metric_to_bev(profile, flattened)
    columns = np.rint(pixels[:, 0]).astype(int)
    rows = np.rint(pixels[:, 1]).astype(int)
    inside = (
        (columns >= 0) & (columns < drivable_mask.shape[1])
        & (rows >= 0) & (rows < drivable_mask.shape[0])
    )
    supported = np.zeros(flattened.shape[0], dtype=bool)
    supported[inside] = drivable_mask[rows[inside], columns[inside]] > 0
    if near_field is not None:
        near_field.validate()
        forward = flattened[:, 0]
        left = flattened[:, 1]
        in_strip = (
            (forward >= near_field.rear_m)
            & (forward <= near_field.forward_m)
        )
        alpha = np.clip(forward / near_field.forward_m, 0.0, 1.0)
        center = alpha * near_field.end_left_m
        supported |= (
            in_strip
            & (np.abs(left - center) <= 0.5 * near_field.width_m)
        )
    return supported.reshape(len(poses), -1).mean(axis=1)


def _candidate_obstacle_hits(
    poses: Sequence[PathPoint],
    obstacles_m: Sequence[Tuple[float, float]],
    geometry: VehicleGeometry,
    margin_m: float,
) -> np.ndarray:
    """Return one collision flag per pose using a batch body-frame test."""
    if not obstacles_m:
        return np.zeros(len(poses), dtype=bool)
    obstacles = np.asarray(obstacles_m, dtype=np.float64)
    if obstacles.ndim != 2 or obstacles.shape[1] != 2 or not np.all(np.isfinite(obstacles)):
        raise TrajectoryError('obstacles must be finite [forward, left] pairs')
    pose_x = np.asarray([pose.x for pose in poses], dtype=np.float64)[:, None]
    pose_y = np.asarray([pose.y for pose in poses], dtype=np.float64)[:, None]
    yaw = np.asarray([pose.yaw for pose in poses], dtype=np.float64)[:, None]
    dx = obstacles[None, :, 0] - pose_x
    dy = obstacles[None, :, 1] - pose_y
    local_x = dx * np.cos(yaw) + dy * np.sin(yaw)
    local_y = -dx * np.sin(yaw) + dy * np.cos(yaw)
    rear = -geometry.rear_overhang_m - margin_m
    front = geometry.length_m - geometry.rear_overhang_m + margin_m
    half_width = 0.5 * geometry.width_m + margin_m
    return np.any(
        (local_x >= rear) & (local_x <= front)
        & (np.abs(local_y) <= half_width),
        axis=1,
    )


def road_support(
    points_m: np.ndarray,
    drivable_mask: np.ndarray,
    profile: CameraProfile,
    near_field: Optional[NearFieldBootstrap] = None,
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
    if near_field is not None:
        near_field.validate()
        x = points_m[:, 0]
        y = points_m[:, 1]
        in_strip = (x >= near_field.rear_m) & (x <= near_field.forward_m)
        alpha = np.clip(x / near_field.forward_m, 0.0, 1.0)
        center = alpha * near_field.end_left_m
        supported |= in_strip & (np.abs(y - center) <= 0.5 * near_field.width_m)
    return float(supported.mean()) if supported.size else 0.0


def near_field_bootstrap_from_corridor(
    samples: Sequence[Tuple[float, float, float]],
    geometry: VehicleGeometry,
    maximum_gap_m: float,
    settle_m: float,
) -> NearFieldBootstrap:
    """Build a conservative blind-strip continuation from measured corridor rows.

    The first row at the camera coverage edge is commonly clipped.  The anchor
    therefore starts ``settle_m`` farther forward and uses the minimum width of
    the next four measured rows.  Nothing is extrapolated when that stable row
    is beyond ``maximum_gap_m`` or cannot contain the padded vehicle.
    """
    geometry.validate()
    if not math.isfinite(maximum_gap_m) or maximum_gap_m <= 0.0:
        raise TrajectoryError('near-field maximum gap must be finite and positive')
    if not math.isfinite(settle_m) or settle_m < 0.0:
        raise TrajectoryError('near-field settle distance must be finite and non-negative')
    clean = []
    for sample in samples:
        if len(sample) != 3:
            raise TrajectoryError('corridor bootstrap samples must be [forward, left, width]')
        x, y, width = map(float, sample)
        if not all(math.isfinite(value) for value in (x, y, width)) or x <= 0.0 or width <= 0.0:
            continue
        clean.append((x, y, width))
    clean.sort(key=lambda item: item[0])
    if not clean:
        raise TrajectoryError('no finite corridor is available for the near-field blind strip')
    target_x = clean[0][0] + settle_m
    anchor_index = next((index for index, item in enumerate(clean) if item[0] >= target_x), None)
    if anchor_index is None:
        raise TrajectoryError('corridor does not reach the near-field stable row')
    anchor = clean[anchor_index]
    if anchor[0] > maximum_gap_m:
        raise TrajectoryError('first stable corridor row exceeds the near-field maximum gap')
    width = min(item[2] for item in clean[anchor_index:anchor_index + 4])
    required_width = geometry.width_m + 2.0 * geometry.footprint_padding_m
    if width < required_width:
        raise TrajectoryError('near-field corridor is narrower than the padded vehicle')
    bootstrap = NearFieldBootstrap(
        rear_m=-geometry.rear_overhang_m - geometry.footprint_padding_m,
        forward_m=anchor[0],
        end_left_m=anchor[1],
        width_m=width,
    )
    bootstrap.validate()
    return bootstrap


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
    near_field: Optional[NearFieldBootstrap] = None,
    enforce_road_support: bool = True,
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
    local_footprint = _footprint_template(
        geometry, config.footprint_sample_spacing_m
    )
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
            # Apply correction gain before rolling out the trajectory: the
            # evaluated path and the command must use the same steering.
            desired = max(-maximum_steer, min(maximum_steer, desired * config.steering_gain))
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

            tracking = math.hypot(target.x - pose.x, target.y - pose.y)
            cost += tracking ** 2 * 8.0 + abs(curvature) * 0.001

        supports = _candidate_supports(
            points[1:], local_footprint, drivable_mask, profile, near_field
        )
        obstacle_hits = _candidate_obstacle_hits(
            points[1:], obstacles_m, geometry, config.obstacle_margin_m
        )
        minimum_support = float(np.min(supports))
        road_blocked = int(np.count_nonzero(
            supports < config.minimum_road_support
        ))
        obstacle_blocked = int(np.count_nonzero(obstacle_hits))
        cost += float(np.sum(
            (1.0 - supports) * config.road_support_cost_weight
        ))

        reasons = []
        if road_blocked and enforce_road_support:
            reasons.append('swept footprint leaves observed road')
        if obstacle_blocked:
            reasons.append('obstacle intersects swept footprint')
        candidates.append(Candidate(
            candidate_id=candidate_id,
            offset_m=offset,
            points=points,
            curvatures=curvatures,
            cost=cost,
            valid=(not road_blocked or not enforce_road_support) and not obstacle_blocked,
            road_blocked=road_blocked,
            obstacle_blocked=obstacle_blocked,
            minimum_support=minimum_support,
            command_steer_rad=command_steer,
            reject_reason='; '.join(reasons),
        ))
    return sorted(candidates, key=lambda candidate: (not candidate.valid, candidate.cost))
