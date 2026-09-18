"""Pure diagnostic source arbitration for V4 Stage 7."""

from dataclasses import dataclass
from typing import Mapping, Optional


@dataclass(frozen=True)
class ArbitrationDecision:
    source: str
    action: str
    reason: str
    reference: Optional[Mapping[str, object]]


HARD_HOLD_STATES = frozenset({
    'EMERGENCY_STOP', 'TRAFFIC_LIGHT', 'BOOM_GATE', 'OBSTRUCTION',
    'TUNNEL', 'REVERSE_ADJUST', 'ROUNDABOUT', 'HILL', 'PARKING_IDLE',
    'PARKING_PLAYBACK', 'MANUAL', 'FINISHED', 'IDLE',
})


def _state(value: str) -> str:
    return str(value).split('|', 1)[0].strip().upper()


def select_diagnostic_intent(
    dashboard_value: str,
    motion_permitted: bool,
    trajectory_status: Mapping[str, object],
    parking_status: Mapping[str, object],
    recovery_status: Mapping[str, object],
) -> ArbitrationDecision:
    """Choose a proposed source. The result is data, never an actuator command."""
    state = _state(dashboard_value)
    if not motion_permitted:
        return ArbitrationDecision('hold', 'stop', 'safety controller denies motion', None)
    if state in HARD_HOLD_STATES or not state:
        return ArbitrationDecision('hold', 'stop', f'mission hard hold: {state or "UNKNOWN"}', None)
    if state in ('PARALLEL_PARK', 'PERPENDICULAR_PARK'):
        selected = parking_status.get('selected_diagnostic_only') if isinstance(parking_status, Mapping) else None
        if isinstance(selected, Mapping) and selected.get('valid') is True:
            return ArbitrationDecision('parking', 'follow_path', 'valid parking proposal', selected)
        return ArbitrationDecision('hold', 'stop', 'parking has no valid proposal', None)
    selected_recovery = recovery_status.get('selected_diagnostic_only') if isinstance(recovery_status, Mapping) else None
    if state == 'LANE_RECOVERY' and isinstance(selected_recovery, Mapping) and selected_recovery.get('valid') is True:
        return ArbitrationDecision('recovery', 'follow_path', 'valid bounded recovery proposal', selected_recovery)
    selected = trajectory_status.get('selected_diagnostic_only') if isinstance(trajectory_status, Mapping) else None
    if isinstance(selected, Mapping) and selected.get('valid') is True:
        return ArbitrationDecision('trajectory', 'follow_curvature', 'valid forward trajectory', selected)
    return ArbitrationDecision('hold', 'stop', 'no valid proposal', None)
