"""Encoder-distance record/playback for fixed-steering parking moves (ROS-free).

A recording is a list of 20 Hz samples {motor_pwm, servo_angle, enc_ticks} where
enc_ticks is the cumulative signed drive-wheel encoder count. Playback does not
replay motor timing. It splits the samples into segments -- "hold this steering,
drive this many encoder ticks in this direction" -- and closes the loop on the
encoder, so wheel slip or a different speed cannot change how far the car goes.

Distances stay in raw ticks so record and playback always agree, whatever
ticks_per_meter is set to. Nothing here needs a calibrated odometer.
"""

from dataclasses import dataclass
from typing import List, Optional

SAMPLE_DT = 0.05            # recording rate (s)
STEER_TOL = 3               # servo units; a bigger change starts a new segment
MIN_SEGMENT_TICKS = 20.0    # drop moves shorter than this (stick noise)


@dataclass
class Segment:
    servo_angle: int
    direction: int          # +1 forward, -1 reverse
    duty: int               # motor duty percent while driving
    ticks: float            # encoder ticks travelled while the motor was on
    duration_s: float       # how long it took when recorded


def build_segments(samples, steer_tol: int = STEER_TOL,
                   min_ticks: float = MIN_SEGMENT_TICKS) -> List[Segment]:
    """Split recorded samples into (steering, direction, distance) moves."""
    segments: List[Segment] = []
    run: list = []

    def flush(end_ticks):
        if not run:
            return
        start_ticks = float(run[0].get('enc_ticks', 0.0))
        ticks = abs(float(end_ticks) - start_ticks)
        duties = sorted(abs(int(s['motor_pwm'])) for s in run)
        angles = sorted(int(s['servo_angle']) for s in run)
        direction = 1 if sum(int(s['motor_pwm']) for s in run) >= 0 else -1
        if ticks >= min_ticks:
            segments.append(Segment(
                servo_angle=angles[len(angles) // 2], direction=direction,
                duty=duties[len(duties) // 2], ticks=ticks,
                duration_s=len(run) * SAMPLE_DT))
        run.clear()

    for i, sample in enumerate(samples):
        pwm = int(sample['motor_pwm'])
        nxt_ticks = float(samples[i + 1].get('enc_ticks', 0.0)) if i + 1 < len(samples) \
            else float(sample.get('enc_ticks', 0.0))
        if pwm == 0:
            flush(float(sample.get('enc_ticks', 0.0)))
            continue
        if run:
            same_dir = (pwm > 0) == (int(run[-1]['motor_pwm']) > 0)
            same_steer = abs(int(sample['servo_angle']) - int(run[-1]['servo_angle'])) <= steer_tol
            if not (same_dir and same_steer):
                flush(float(sample.get('enc_ticks', 0.0)))
        run.append(sample)
        if i + 1 == len(samples):
            flush(nxt_ticks)
    return segments


@dataclass
class Output:
    motor_pwm: int = 0
    servo_angle: Optional[int] = None
    done: bool = False
    abort: str = ''


class SegmentPlayer:
    """Steer -> drive to the encoder target -> settle, once per segment."""

    def __init__(self, segments: List[Segment], servo_center: int, *,
                 duty_override: int = 0, lead_ticks: float = 0.0,
                 steer_settle_s: float = 0.7, same_steer_settle_s: float = 0.2,
                 still_ticks: float = 2.0, still_s: float = 0.35, settle_max_s: float = 1.5,
                 stall_ticks: float = 10.0, stall_s: float = 2.0):
        self.segments = segments
        self.servo_center = int(servo_center)
        self.duty_override = int(duty_override)
        self.lead_ticks = float(lead_ticks)
        self.steer_settle_s = steer_settle_s
        self.same_steer_settle_s = same_steer_settle_s
        self.still_ticks = still_ticks
        self.still_s = still_s
        self.settle_max_s = settle_max_s
        self.stall_ticks = stall_ticks
        self.stall_s = stall_s
        self.index = 0
        self.phase = 'STEER'
        self.phase_t = None
        self.start_ticks = 0.0
        self.progress_ticks = 0.0
        self.progress_t = 0.0
        self.still_ref = 0.0
        self.still_t = 0.0
        self.angle = self.servo_center      # steering held right now
        self.log: List[str] = []

    @property
    def current(self) -> Optional[Segment]:
        return self.segments[self.index] if self.index < len(self.segments) else None

    def step(self, now: float, enc_ticks: float) -> Output:
        seg = self.current
        if seg is None:
            return Output(motor_pwm=0, servo_angle=self.angle, done=True)
        if self.phase_t is None:
            self.phase_t = now

        if self.phase == 'STEER':
            wait = (self.steer_settle_s if abs(seg.servo_angle - self.angle) > STEER_TOL
                    else self.same_steer_settle_s)
            self.angle = seg.servo_angle
            if now - self.phase_t >= wait:
                self.phase = 'DRIVE'
                self.phase_t = now
                self.start_ticks = enc_ticks
                self.progress_ticks = enc_ticks
                self.progress_t = now
            return Output(0, self.angle)

        if self.phase == 'DRIVE':
            travelled = abs(enc_ticks - self.start_ticks)
            if travelled >= max(0.0, seg.ticks - self.lead_ticks):
                self.log.append(f'segment {self.index + 1}/{len(self.segments)}: '
                                f'target {seg.ticks:.0f} ticks, motor off at {travelled:.0f}')
                self.phase = 'SETTLE'
                self.phase_t = now
                self.still_ref = enc_ticks
                self.still_t = now
                return Output(0, self.angle)
            if abs(enc_ticks - self.progress_ticks) >= self.stall_ticks:
                self.progress_ticks = enc_ticks
                self.progress_t = now
            elif now - self.progress_t > self.stall_s:
                return Output(0, self.angle, abort='encoder_stall')
            if now - self.phase_t > 3.0 * seg.duration_s + 3.0:
                return Output(0, self.angle, abort='segment_timeout')
            duty = self.duty_override or seg.duty
            return Output(seg.direction * duty, self.angle)

        # SETTLE: motor off until the wheel has stopped rolling
        if abs(enc_ticks - self.still_ref) > self.still_ticks:
            self.still_ref = enc_ticks
            self.still_t = now
        if now - self.still_t >= self.still_s or now - self.phase_t >= self.settle_max_s:
            self.index += 1
            self.phase = 'STEER'
            self.phase_t = now
            if self.current is None:
                return Output(0, self.angle, done=True)
        return Output(0, self.angle)


def describe(segments: List[Segment], servo_center: int, ticks_per_meter: float) -> List[str]:
    """Human-readable lines; metres are only as good as ticks_per_meter."""
    lines = []
    for n, s in enumerate(segments, 1):
        steer = s.servo_angle - servo_center
        side = 'center' if abs(steer) <= STEER_TOL else ('right' if steer > 0 else 'left')
        lines.append(f'{n}. {"fwd" if s.direction > 0 else "rev"} {s.ticks:.0f} ticks '
                     f'(~{s.ticks / max(1.0, ticks_per_meter):.2f} m) '
                     f'steer {side} servo={s.servo_angle} duty={s.duty}')
    return lines
