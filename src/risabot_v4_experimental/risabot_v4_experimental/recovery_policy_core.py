"""Mission-policy adapter for bounded Stage 6 recovery requests."""

from dataclasses import dataclass
import math
from typing import Mapping, Optional

from .recovery_core import RecoveryRequest, forward_candidates_exhausted


@dataclass(frozen=True)
class RecoveryPolicyConfig:
    stopped_speed_mps: float = 0.015
    maximum_attempts: int = 3
    allowed_states: tuple = ('LANE_FOLLOW', 'LANE_RECOVERY')


def dashboard_state(value: str) -> str:
    if not isinstance(value, str) or not value.strip():
        return ''
    return value.split('|', 1)[0].strip().upper()


def build_recovery_request(
    trajectory_status: Mapping[str, object],
    state_value: str,
    speed_mps: float,
    image_stamp_sec: float,
    attempts: int,
    config: RecoveryPolicyConfig = RecoveryPolicyConfig(),
) -> Optional[RecoveryRequest]:
    if not forward_candidates_exhausted(trajectory_status):
        return None
    if not all(math.isfinite(value) for value in (speed_mps, image_stamp_sec)):
        return None
    if attempts < 0:
        return None
    state = dashboard_state(state_value)
    hard_hold = '' if state in config.allowed_states else (state or 'UNKNOWN_STATE')
    return RecoveryRequest(
        problem='no_forward_candidate', hard_hold=hard_hold,
        permitted=state in config.allowed_states and attempts < config.maximum_attempts,
        stopped=abs(speed_mps) <= config.stopped_speed_mps,
        attempts=attempts, image_stamp_sec=image_stamp_sec, frame_id='base_link',
    )
