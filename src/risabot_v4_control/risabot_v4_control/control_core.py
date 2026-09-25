"""Pure, ROS-free contracts for the V4 motion executor."""

from dataclasses import dataclass
import math
from typing import Mapping, Optional, Sequence, Tuple


STOP_STATES = frozenset({
    '', 'MANUAL', 'FINISHED', 'TRAFFIC_LIGHT', 'BOOM_GATE',
    'EMERGENCY_STOP', 'REVERSE_ADJUST', 'PARKING_IDLE', 'PARKING_PLAYBACK',
})
LEGACY_CHALLENGE_STATES = frozenset({'TUNNEL', 'OBSTRUCTION'})
V4_TRAJECTORY_STATES = frozenset({'LANE_FOLLOW', 'ROUNDABOUT', 'HILL'})
V4_PATH_STATES = {
    'LANE_RECOVERY': 'recovery',
    'PARALLEL_PARK': 'parking',
    'PERPENDICULAR_PARK': 'parking',
}


class ControlContractError(ValueError):
    """Raised when an input cannot safely become a motion request."""


@dataclass(frozen=True)
class PathPoint:
    x: float
    y: float
    yaw: float
    direction: int
    curvature: float


@dataclass(frozen=True)
class CommandDecision:
    speed: float
    steering: float
    source: str
    reason: str


def dashboard_state(value: str) -> str:
    return str(value).split('|', 1)[0].strip().upper()


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def steering_from_curvature(curvature: float, wheelbase: float,
                            maximum_steer_rad: float) -> float:
    """Return the car contract: normalized steering, positive means right."""
    values = (curvature, wheelbase, maximum_steer_rad)
    if not all(math.isfinite(v) for v in values):
        raise ControlContractError('curvature conversion values must be finite')
    if wheelbase <= 0.0 or maximum_steer_rad <= 0.0:
        raise ControlContractError('wheelbase and steering limit must be positive')
    left_positive_angle = math.atan(wheelbase * curvature)
    return clamp(-left_positive_angle / maximum_steer_rad, -1.0, 1.0)


def parse_path(payload: Mapping[str, object], maximum_points: int = 160) -> Tuple[PathPoint, ...]:
    raw = payload.get('points')
    if not isinstance(raw, Sequence) or isinstance(raw, (str, bytes)):
        raise ControlContractError('path points are missing')
    if len(raw) < 2 or len(raw) > maximum_points:
        raise ControlContractError('path point count is outside limits')
    points = []
    for item in raw:
        if not isinstance(item, Mapping):
            raise ControlContractError('path point must be an object')
        try:
            point = PathPoint(
                float(item['forward_m']), float(item['left_m']),
                float(item['yaw_rad']), int(item['direction']),
                float(item['curvature_per_m']),
            )
        except (KeyError, TypeError, ValueError) as exc:
            raise ControlContractError(f'invalid path point: {exc}') from exc
        if not all(math.isfinite(v) for v in (
                point.x, point.y, point.yaw, point.curvature)):
            raise ControlContractError('path values must be finite')
        if point.direction not in (-1, 1):
            raise ControlContractError('path direction must be -1 or 1')
        points.append(point)
    return tuple(points)


def transform_path(points: Sequence[PathPoint], pose: Tuple[float, float, float]) -> Tuple[PathPoint, ...]:
    px, py, yaw = pose
    if not all(math.isfinite(v) for v in pose):
        raise ControlContractError('anchor pose must be finite')
    cosine, sine = math.cos(yaw), math.sin(yaw)
    return tuple(PathPoint(
        px + p.x * cosine - p.y * sine,
        py + p.x * sine + p.y * cosine,
        math.atan2(math.sin(yaw + p.yaw), math.cos(yaw + p.yaw)),
        p.direction, p.curvature,
    ) for p in points)


def nearest_path_index(points: Sequence[PathPoint], x: float, y: float,
                       start: int = 0, search_ahead: int = 35) -> int:
    if not points:
        raise ControlContractError('path is empty')
    first = max(0, min(int(start), len(points) - 1))
    last = min(len(points), first + max(1, int(search_ahead)))
    return min(range(first, last), key=lambda i: (points[i].x - x) ** 2 + (points[i].y - y) ** 2)


def path_command(points: Sequence[PathPoint], pose: Tuple[float, float, float],
                 start_index: int, wheelbase: float, maximum_steer_rad: float,
                 forward_speed: float, reverse_speed: float,
                 completion_distance: float) -> Tuple[CommandDecision, int, bool]:
    x, y, _ = pose
    index = nearest_path_index(points, x, y, start_index)
    final_distance = math.hypot(points[-1].x - x, points[-1].y - y)
    complete = index >= len(points) - 2 and final_distance <= completion_distance
    if complete:
        return CommandDecision(0.0, 0.0, 'v4_path', 'path complete'), index, True
    target = points[min(index + 2, len(points) - 1)]
    steering = steering_from_curvature(target.curvature, wheelbase, maximum_steer_rad)
    speed = forward_speed if target.direction > 0 else -abs(reverse_speed)
    return CommandDecision(speed, steering, 'v4_path', 'following validated path'), index, False


def trajectory_command(reference: Mapping[str, object], wheelbase: float,
                       maximum_steer_rad: float, forward_speed: float,
                       minimum_speed_scale: float,
                       steering_slowdown_gain: float = 0.65,
                       boundary_slowdown_margin_m: float = 0.03) -> CommandDecision:
    if reference.get('valid') is not True:
        raise ControlContractError('trajectory reference is not valid')
    try:
        steer_rad = float(reference['command_steer_rad_diagnostic_only'])
    except (KeyError, TypeError, ValueError) as exc:
        raise ControlContractError('trajectory steering is missing') from exc
    if not math.isfinite(steer_rad):
        raise ControlContractError('trajectory steering is non-finite')
    values = (forward_speed, minimum_speed_scale, steering_slowdown_gain,
              boundary_slowdown_margin_m)
    if not all(math.isfinite(value) for value in values):
        raise ControlContractError('trajectory speed settings must be finite')
    if forward_speed <= 0.0 or not 0.0 < minimum_speed_scale <= 1.0:
        raise ControlContractError('trajectory speed and minimum scale are invalid')
    if steering_slowdown_gain < 0.0:
        raise ControlContractError('steering slowdown gain must be non-negative')
    if boundary_slowdown_margin_m <= 0.0:
        raise ControlContractError('boundary slowdown margin must be positive')
    curvature = math.tan(steer_rad) / wheelbase
    steering = steering_from_curvature(curvature, wheelbase, maximum_steer_rad)
    scale = max(minimum_speed_scale,
                1.0 - steering_slowdown_gain * abs(steering))
    if 'boundary_clearance_m' in reference:
        try:
            clearance = float(reference['boundary_clearance_m'])
        except (TypeError, ValueError) as exc:
            raise ControlContractError('boundary clearance is invalid') from exc
        if not math.isfinite(clearance):
            raise ControlContractError('boundary clearance is non-finite')
        clearance_scale = minimum_speed_scale + (1.0 - minimum_speed_scale) * clamp(
            clearance / boundary_slowdown_margin_m, 0.0, 1.0
        )
        scale = min(scale, clearance_scale)
    if 'preview_lateral_shift_m' in reference:
        try:
            preview_shift = float(reference['preview_lateral_shift_m'])
        except (TypeError, ValueError) as exc:
            raise ControlContractError('lane preview shift is invalid') from exc
        if not math.isfinite(preview_shift):
            raise ControlContractError('lane preview shift is non-finite')
        # The 1 m camera preview can reveal a bend before the near-lane
        # controller needs steering. Give the actuator time to turn before
        # the front corner reaches the white line.
        preview_scale = max(
            minimum_speed_scale,
            1.0 - 8.0 * max(0.0, abs(preview_shift) - 0.02),
        )
        scale = min(scale, preview_scale)
    return CommandDecision(forward_speed * scale, steering, 'v4_trajectory', 'validated trajectory')


def proposal_contract(payload: Mapping[str, object]) -> Tuple[str, str, Optional[Mapping[str, object]]]:
    if not isinstance(payload, Mapping):
        raise ControlContractError('proposal must be an object')
    source = str(payload.get('source', '')).strip().lower()
    action = str(payload.get('action', '')).strip().lower()
    reference = payload.get('reference')
    if action not in ('stop', 'follow_curvature', 'follow_path'):
        raise ControlContractError('unsupported proposal action')
    if action == 'stop':
        return source, action, None
    if source not in ('trajectory', 'parking', 'recovery') or not isinstance(reference, Mapping):
        raise ControlContractError('moving proposal has no valid reference')
    if reference.get('valid') is not True:
        raise ControlContractError('moving proposal reference is invalid')
    return source, action, reference
