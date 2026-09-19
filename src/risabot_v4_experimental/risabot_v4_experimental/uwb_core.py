"""ROS-free UWB range validation, de-duplication, and multilateration."""

from dataclasses import dataclass
import math
from typing import Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

import numpy as np


class UwbError(ValueError):
    """Raised when a UWB configuration or measurement is unsafe to use."""


@dataclass(frozen=True)
class Anchor:
    anchor_id: str
    x_m: float
    y_m: float
    z_m: float = 0.0
    range_offset_m: float = 0.0

    def __post_init__(self) -> None:
        values = (self.x_m, self.y_m, self.z_m, self.range_offset_m)
        if not self.anchor_id or not all(math.isfinite(float(v)) for v in values):
            raise UwbError('anchor id and coordinates must be finite and non-empty')


@dataclass(frozen=True)
class RangeSample:
    anchor_id: str
    planar_range_m: float
    raw_range_m: float
    age_sec: float
    sample_seq: int
    measured_mono: float


@dataclass(frozen=True)
class Fix:
    x_m: float
    y_m: float
    sigma_m: float
    residual_rms_m: float
    maximum_residual_m: float
    geometry_condition: float
    age_sec: float
    boot_id: str
    report_seq: Optional[int]
    samples: Tuple[RangeSample, ...]


def _finite(value: object, label: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise UwbError(f'{label} must be numeric') from exc
    if not math.isfinite(result):
        raise UwbError(f'{label} must be finite')
    return result


def validate_anchor_geometry(anchors: Sequence[Anchor], minimum_area_m2: float) -> None:
    if len(anchors) < 3:
        raise UwbError('at least three anchors are required')
    if not math.isfinite(minimum_area_m2) or minimum_area_m2 <= 0.0:
        raise UwbError('minimum anchor area must be positive')
    points = np.asarray([(a.x_m, a.y_m) for a in anchors], dtype=np.float64)
    if len(np.unique(points, axis=0)) != len(points):
        raise UwbError('anchor positions must be unique')
    maximum_twice_area = 0.0
    for i in range(len(points) - 2):
        for j in range(i + 1, len(points) - 1):
            for k in range(j + 1, len(points)):
                first = points[j] - points[i]
                second = points[k] - points[i]
                twice_area = abs(float(first[0] * second[1] - first[1] * second[0]))
                maximum_twice_area = max(maximum_twice_area, twice_area)
    if maximum_twice_area < 2.0 * minimum_area_m2:
        raise UwbError('anchor geometry is collinear or too compact')


def _initial_position(samples: Sequence[RangeSample], anchors: Mapping[str, Anchor]) -> np.ndarray:
    reference = samples[0]
    a0 = anchors[reference.anchor_id]
    rows = []
    values = []
    for sample in samples[1:]:
        anchor = anchors[sample.anchor_id]
        rows.append([2.0 * (anchor.x_m - a0.x_m), 2.0 * (anchor.y_m - a0.y_m)])
        values.append(
            reference.planar_range_m ** 2 - sample.planar_range_m ** 2
            - a0.x_m ** 2 - a0.y_m ** 2
            + anchor.x_m ** 2 + anchor.y_m ** 2
        )
    matrix = np.asarray(rows, dtype=np.float64)
    if np.linalg.matrix_rank(matrix) < 2:
        raise UwbError('fresh anchor geometry cannot determine a 2D fix')
    result, _, _, _ = np.linalg.lstsq(matrix, np.asarray(values), rcond=None)
    return result


def solve_multilateration(
    samples: Sequence[RangeSample],
    anchors: Mapping[str, Anchor],
    *,
    huber_delta_m: float = 0.20,
    maximum_iterations: int = 12,
    minimum_sigma_m: float = 0.03,
) -> Fix:
    """Solve a robust 2D range fix and report residual/geometry diagnostics."""
    if len(samples) < 3:
        raise UwbError('at least three fresh ranges are required')
    if huber_delta_m <= 0.0 or maximum_iterations < 1 or minimum_sigma_m <= 0.0:
        raise UwbError('solver parameters must be positive')
    for sample in samples:
        if sample.anchor_id not in anchors:
            raise UwbError(f'unknown anchor {sample.anchor_id}')
        if not math.isfinite(sample.planar_range_m) or sample.planar_range_m <= 0.0:
            raise UwbError('planar ranges must be finite and positive')

    position = _initial_position(samples, anchors)
    for _ in range(maximum_iterations):
        jacobian = []
        residuals = []
        weights = []
        for sample in samples:
            anchor = anchors[sample.anchor_id]
            delta = position - np.asarray([anchor.x_m, anchor.y_m])
            distance = float(np.linalg.norm(delta))
            if distance < 1e-9:
                distance = 1e-9
            residual = distance - sample.planar_range_m
            jacobian.append(delta / distance)
            residuals.append(residual)
            absolute = abs(residual)
            weights.append(1.0 if absolute <= huber_delta_m else huber_delta_m / absolute)
        j = np.asarray(jacobian, dtype=np.float64)
        r = np.asarray(residuals, dtype=np.float64)
        w = np.sqrt(np.asarray(weights, dtype=np.float64))
        step, _, rank, _ = np.linalg.lstsq(j * w[:, None], -r * w, rcond=None)
        if rank < 2:
            raise UwbError('range geometry is singular at the solution')
        position += step
        if float(np.linalg.norm(step)) < 1e-7:
            break

    residuals = []
    jacobian = []
    for sample in samples:
        anchor = anchors[sample.anchor_id]
        delta = position - np.asarray([anchor.x_m, anchor.y_m])
        distance = float(np.linalg.norm(delta))
        if distance < 1e-9:
            raise UwbError('solution coincides with an anchor and has singular geometry')
        residuals.append(distance - sample.planar_range_m)
        jacobian.append(delta / distance)
    residual_array = np.asarray(residuals, dtype=np.float64)
    information = np.asarray(jacobian, dtype=np.float64).T @ np.asarray(jacobian, dtype=np.float64)
    condition = float(np.linalg.cond(information))
    if not math.isfinite(condition):
        raise UwbError('solution geometry condition is not finite')
    rms = float(np.sqrt(np.mean(residual_array ** 2)))
    geometry_scale = math.sqrt(max(1.0, condition))
    sigma = max(minimum_sigma_m, rms) * geometry_scale
    newest_age = max(sample.age_sec for sample in samples)
    return Fix(
        x_m=float(position[0]), y_m=float(position[1]), sigma_m=sigma,
        residual_rms_m=rms,
        maximum_residual_m=float(np.max(np.abs(residual_array))),
        geometry_condition=condition, age_sec=newest_age,
        boot_id='', report_seq=None, samples=tuple(samples),
    )


class UwbRangeProcessor:
    """Maintain one latest unique sample per anchor and emit fresh range fixes."""

    def __init__(
        self,
        anchors: Iterable[Anchor],
        *,
        tag_z_m: float,
        maximum_range_age_sec: float = 0.40,
        minimum_anchor_count: int = 3,
        minimum_anchor_area_m2: float = 0.25,
        maximum_residual_m: float = 0.30,
        maximum_geometry_condition: float = 100.0,
        minimum_sigma_m: float = 0.03,
    ) -> None:
        anchor_list = list(anchors)
        validate_anchor_geometry(anchor_list, minimum_anchor_area_m2)
        self.anchors: Dict[str, Anchor] = {a.anchor_id: a for a in anchor_list}
        if len(self.anchors) != len(anchor_list):
            raise UwbError('anchor ids must be unique')
        self.tag_z_m = _finite(tag_z_m, 'tag z')
        self.maximum_range_age_sec = _finite(maximum_range_age_sec, 'maximum range age')
        self.maximum_residual_m = _finite(maximum_residual_m, 'maximum residual')
        self.maximum_geometry_condition = _finite(
            maximum_geometry_condition, 'maximum geometry condition'
        )
        self.minimum_sigma_m = _finite(minimum_sigma_m, 'minimum sigma')
        self.minimum_anchor_count = int(minimum_anchor_count)
        if self.maximum_range_age_sec <= 0.0 or self.maximum_residual_m <= 0.0:
            raise UwbError('age and residual limits must be positive')
        if self.maximum_geometry_condition <= 0.0 or self.minimum_sigma_m <= 0.0:
            raise UwbError('geometry condition and sigma limits must be positive')
        if self.minimum_anchor_count < 3 or self.minimum_anchor_count > len(self.anchors):
            raise UwbError('minimum anchor count is outside the configured anchor set')
        self.boot_id: Optional[str] = None
        self._last_sequence: Dict[str, int] = {}
        self._latest: Dict[str, RangeSample] = {}
        self.duplicate_count = 0
        self.out_of_order_count = 0
        self.invalid_count = 0

    @staticmethod
    def _is_newer_sequence(current: int, previous: int) -> bool:
        difference = (current - previous) & 0xFFFFFFFF
        return 0 < difference < 0x80000000

    def ingest(self, payload: object, arrival_mono: float) -> Tuple[Optional[Fix], str]:
        arrival = _finite(arrival_mono, 'arrival time')
        if not isinstance(payload, dict):
            self.invalid_count += 1
            return None, 'payload_not_object'
        boot_id = str(payload.get('boot_id', '')).strip()
        if not boot_id:
            self.invalid_count += 1
            return None, 'missing_boot_id'
        if boot_id != self.boot_id:
            self.boot_id = boot_id
            self._last_sequence.clear()
            self._latest.clear()

        links = payload.get('links')
        if not isinstance(links, list):
            self.invalid_count += 1
            return None, 'links_not_array'
        accepted_new = 0
        for link in links:
            try:
                if not isinstance(link, dict):
                    raise UwbError('link is not an object')
                anchor_id = str(link.get('A', ''))
                anchor = self.anchors.get(anchor_id)
                if anchor is None:
                    continue
                sequence = int(link['sample_seq'])
                if sequence < 0 or sequence > 0xFFFFFFFF:
                    raise UwbError('sample sequence outside uint32 range')
                previous = self._last_sequence.get(anchor_id)
                if previous is not None:
                    if sequence == previous:
                        self.duplicate_count += 1
                        continue
                    if not self._is_newer_sequence(sequence, previous):
                        self.out_of_order_count += 1
                        continue
                raw_range = _finite(link['R'], 'range')
                age_sec = _finite(link['age_ms'], 'range age') / 1000.0
                if raw_range <= 0.0 or age_sec < 0.0 or age_sec > self.maximum_range_age_sec:
                    raise UwbError('range is non-positive or stale')
                corrected = raw_range - anchor.range_offset_m
                vertical = anchor.z_m - self.tag_z_m
                planar_squared = corrected * corrected - vertical * vertical
                if corrected <= 0.0 or planar_squared <= 0.0:
                    raise UwbError('range cannot support configured height correction')
                sample = RangeSample(
                    anchor_id=anchor_id,
                    planar_range_m=math.sqrt(planar_squared),
                    raw_range_m=raw_range,
                    age_sec=age_sec,
                    sample_seq=sequence,
                    measured_mono=arrival - age_sec,
                )
                self._last_sequence[anchor_id] = sequence
                self._latest[anchor_id] = sample
                accepted_new += 1
            except (KeyError, TypeError, ValueError, UwbError):
                self.invalid_count += 1

        if accepted_new == 0:
            return None, 'no_new_ranges'
        fresh = [
            sample for sample in self._latest.values()
            if 0.0 <= arrival - sample.measured_mono <= self.maximum_range_age_sec
        ]
        if len(fresh) < self.minimum_anchor_count:
            return None, 'insufficient_fresh_anchors'
        try:
            solved = solve_multilateration(
                sorted(fresh, key=lambda item: item.anchor_id), self.anchors,
                minimum_sigma_m=self.minimum_sigma_m,
            )
        except UwbError:
            self.invalid_count += 1
            return None, 'solver_rejected'
        if solved.maximum_residual_m > self.maximum_residual_m:
            return None, 'residual_gate'
        if solved.geometry_condition > self.maximum_geometry_condition:
            return None, 'geometry_gate'
        report_seq = payload.get('seq')
        try:
            report_seq = None if report_seq is None else int(report_seq)
        except (TypeError, ValueError):
            report_seq = None
        fix = Fix(
            x_m=solved.x_m, y_m=solved.y_m, sigma_m=solved.sigma_m,
            residual_rms_m=solved.residual_rms_m,
            maximum_residual_m=solved.maximum_residual_m,
            geometry_condition=solved.geometry_condition,
            age_sec=max(arrival - sample.measured_mono for sample in fresh),
            boot_id=boot_id, report_seq=report_seq, samples=tuple(fresh),
        )
        return fix, 'accepted'
