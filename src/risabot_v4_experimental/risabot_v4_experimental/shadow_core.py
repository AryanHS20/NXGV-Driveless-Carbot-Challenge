"""ROS-independent safety and freshness logic for V4 shadow mode."""

from dataclasses import dataclass
from typing import Dict, Iterable, Mapping, Optional


@dataclass(frozen=True)
class InputState:
    name: str
    age_seconds: Optional[float]
    timeout_seconds: float

    @property
    def fresh(self) -> bool:
        return (
            self.age_seconds is not None
            and self.age_seconds >= 0.0
            and self.age_seconds <= self.timeout_seconds
        )


def evaluate_inputs(
    now: float,
    last_seen: Mapping[str, Optional[float]],
    timeouts: Mapping[str, float],
    required: Iterable[str],
) -> Dict[str, object]:
    """Return a deterministic shadow-mode readiness report.

    Readiness is diagnostic only. It is deliberately not a motion permission.
    """
    states = []
    missing = []
    stale = []
    for name in required:
        stamp = last_seen.get(name)
        timeout = float(timeouts[name])
        age = None if stamp is None else max(0.0, now - float(stamp))
        state = InputState(name=name, age_seconds=age, timeout_seconds=timeout)
        states.append(state)
        if stamp is None:
            missing.append(name)
        elif not state.fresh:
            stale.append(name)

    return {
        'ready_for_shadow_evaluation': not missing and not stale,
        'motion_authority': False,
        'missing': missing,
        'stale': stale,
        'inputs': {
            state.name: {
                'age_seconds': state.age_seconds,
                'timeout_seconds': state.timeout_seconds,
                'fresh': state.fresh,
            }
            for state in states
        },
    }
