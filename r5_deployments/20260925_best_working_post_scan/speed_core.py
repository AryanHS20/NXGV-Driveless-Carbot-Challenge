"""Forward-only encoder speed regulation; output remains bounded motor duty.

The odometry scale is supplied by the existing encoder configuration. This
controller cannot establish ground-speed accuracy or detect wheel slip.
"""
import math


class SpeedRegulator:
    def __init__(self, duty_limit=65.0, kp=100.0, ki=80.0):
        if (not all(math.isfinite(v) for v in (duty_limit, kp, ki))
                or not 0 < duty_limit <= 100 or kp <= 0 or ki < 0):
            raise ValueError('invalid speed regulator limits')
        self.limit, self.kp, self.ki = duty_limit, kp, ki
        self.fault = ''
        self.reset()

    def reset(self, clear_fault=False):
        self.integral = 0.0
        self.last_time = None
        self.duty = 0.0
        self.filtered_speed = None
        if clear_fault:
            self.fault = ''

    def update(self, target, measured, age, now):
        if target == 0.0:
            self.reset()
            return 0.0
        if self.fault:
            return 0.0
        if (not all(math.isfinite(v) for v in (target, measured, age, now))
                or target < 0 or not 0 <= age <= 0.25):
            self.fault = 'invalid target or stale encoder odometry'
        elif measured > 1.2:
            self.fault = 'encoder speed outside supported range'
        if self.fault:
            self.reset()
            return 0.0
        dt = 0.0 if self.last_time is None else now - self.last_time
        self.last_time = now
        if not 0 <= dt <= 0.25:
            self.fault = 'speed control timing gap'
            self.reset()
            return 0.0
        # The existing wheel-odometry filter can briefly report a small
        # negative speed while the stationary car enters AUTO. That is not
        # evidence of reverse travel. Treat negative feedback as zero while
        # retaining the independent motor-duty ceiling.
        measured = max(0.0, measured)
        # Raw encoder velocity jumps between zero and values well above the
        # 0.3 m/s request during the recorded R5 run. Filter that feedback
        # before the PI calculation; a target-zero/fault still stops at once.
        if self.filtered_speed is None:
            self.filtered_speed = measured
        elif dt > 0.0:
            fraction = 1.0 - math.exp(-dt / 0.35)
            self.filtered_speed += fraction * (measured - self.filtered_speed)
        error = target - self.filtered_speed
        tentative = max(-self.limit, min(self.limit, self.integral + self.ki * error * dt))
        raw = self.kp * error + tentative
        # Integrate only when unsaturated or when error unwinds saturation.
        if 0 <= raw <= self.limit or (raw > self.limit and error < 0) or (raw < 0 and error > 0):
            self.integral = tentative
        requested = max(0.0, min(self.limit, self.kp * error + self.integral))
        # Avoid alternating motor-off and high duty between camera frames.
        # This is a drive slew, not a delay on explicit target-zero or faults.
        max_change = 90.0 * dt
        self.duty += max(-max_change, min(max_change, requested - self.duty))
        return self.duty
