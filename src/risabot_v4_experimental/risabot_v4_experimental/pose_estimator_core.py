"""Separated local and coarse-global pose estimation for V4 Stage 3.

Local pose is owned by odometry. UWB measurements may update only the
coarse-global translation offset; they can never rewrite local pose.
"""

from dataclasses import dataclass
import math
from typing import Dict, Optional


class PoseEstimatorError(ValueError):
    """Raised when estimator input or configuration is invalid."""


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(float(angle)), math.cos(float(angle)))


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float

    def validate(self) -> None:
        if not all(math.isfinite(value) for value in (self.x, self.y, self.yaw)):
            raise PoseEstimatorError('pose values must be finite')


@dataclass(frozen=True)
class EstimatorConfig:
    frame_yaw_rad: float = 0.0
    initial_offset_sigma_m: float = 0.75
    process_sigma_m_per_s: float = 0.015
    process_sigma_m_per_m: float = 0.04
    default_uwb_sigma_m: float = 0.20
    minimum_uwb_sigma_m: float = 0.03
    maximum_uwb_sigma_m: float = 2.0
    innovation_gate: float = 13.82
    odom_reset_jump_m: float = 0.50
    odom_reset_yaw_rad: float = 1.0

    def validate(self) -> None:
        if not math.isfinite(self.frame_yaw_rad):
            raise PoseEstimatorError('frame_yaw_rad must be finite')
        positive = (
            ('initial_offset_sigma_m', self.initial_offset_sigma_m),
            ('default_uwb_sigma_m', self.default_uwb_sigma_m),
            ('minimum_uwb_sigma_m', self.minimum_uwb_sigma_m),
            ('maximum_uwb_sigma_m', self.maximum_uwb_sigma_m),
            ('innovation_gate', self.innovation_gate),
            ('odom_reset_jump_m', self.odom_reset_jump_m),
            ('odom_reset_yaw_rad', self.odom_reset_yaw_rad),
        )
        for name, value in positive:
            if not math.isfinite(value) or value <= 0.0:
                raise PoseEstimatorError(f'{name} must be finite and positive')
        for name, value in (
            ('process_sigma_m_per_s', self.process_sigma_m_per_s),
            ('process_sigma_m_per_m', self.process_sigma_m_per_m),
        ):
            if not math.isfinite(value) or value < 0.0:
                raise PoseEstimatorError(f'{name} must be finite and non-negative')
        if self.minimum_uwb_sigma_m > self.maximum_uwb_sigma_m:
            raise PoseEstimatorError('minimum UWB sigma must not exceed maximum')


class SeparatedPoseEstimator:
    """Track exact local odometry plus a filtered UWB translation offset."""

    def __init__(self, config: EstimatorConfig = EstimatorConfig()) -> None:
        config.validate()
        self.config = config
        self.local_pose: Optional[Pose2D] = None
        self.local_stamp: Optional[float] = None
        self.offset_x = 0.0
        self.offset_y = 0.0
        self.offset_variance = config.initial_offset_sigma_m ** 2
        self.last_uwb_stamp: Optional[float] = None
        self.last_gain = 0.0
        self.last_innovation_score: Optional[float] = None
        self.accepted_uwb = 0
        self.rejected_uwb = 0
        self.odom_resets = 0

    @staticmethod
    def _stamp(value: float) -> float:
        value = float(value)
        if not math.isfinite(value) or value < 0.0:
            raise PoseEstimatorError('timestamp must be finite and non-negative')
        return value

    def update_local(self, pose: Pose2D, stamp: float) -> bool:
        pose.validate()
        stamp = self._stamp(stamp)
        if self.local_stamp is not None and stamp <= self.local_stamp:
            return False

        if self.local_pose is not None and self.local_stamp is not None:
            distance = math.hypot(
                pose.x - self.local_pose.x, pose.y - self.local_pose.y
            )
            yaw_step = abs(wrap_angle(pose.yaw - self.local_pose.yaw))
            elapsed = stamp - self.local_stamp
            if (
                distance > self.config.odom_reset_jump_m
                or yaw_step > self.config.odom_reset_yaw_rad
            ):
                self.offset_x = 0.0
                self.offset_y = 0.0
                self.offset_variance = self.config.initial_offset_sigma_m ** 2
                self.last_uwb_stamp = None
                self.odom_resets += 1
            else:
                time_sigma = self.config.process_sigma_m_per_s * elapsed
                distance_sigma = self.config.process_sigma_m_per_m * distance
                self.offset_variance += time_sigma ** 2 + distance_sigma ** 2

        self.local_pose = Pose2D(pose.x, pose.y, wrap_angle(pose.yaw))
        self.local_stamp = stamp
        return True

    def update_uwb(
        self,
        x: float,
        y: float,
        stamp: float,
        sigma_m: Optional[float] = None,
    ) -> Dict[str, float]:
        if self.local_pose is None:
            raise PoseEstimatorError('local pose is required before UWB')
        stamp = self._stamp(stamp)
        if self.last_uwb_stamp is not None and stamp <= self.last_uwb_stamp:
            self.rejected_uwb += 1
            return {'accepted': 0.0, 'reason_code': 1.0, 'gain': 0.0}
        x = float(x)
        y = float(y)
        if not math.isfinite(x) or not math.isfinite(y):
            raise PoseEstimatorError('UWB position must be finite')
        sigma = self.config.default_uwb_sigma_m if sigma_m is None else float(sigma_m)
        if not math.isfinite(sigma) or sigma <= 0.0:
            raise PoseEstimatorError('UWB sigma must be finite and positive')
        sigma = min(
            max(sigma, self.config.minimum_uwb_sigma_m),
            self.config.maximum_uwb_sigma_m,
        )

        predicted = self.coarse_pose()
        predicted_x = predicted.x
        predicted_y = predicted.y
        residual_x = x - predicted_x
        residual_y = y - predicted_y
        innovation_variance = self.offset_variance + sigma ** 2
        score = (residual_x ** 2 + residual_y ** 2) / innovation_variance
        self.last_innovation_score = score
        self.last_uwb_stamp = stamp
        if score > self.config.innovation_gate:
            self.last_gain = 0.0
            self.rejected_uwb += 1
            return {
                'accepted': 0.0,
                'reason_code': 2.0,
                'gain': 0.0,
                'innovation_score': score,
            }

        gain = self.offset_variance / innovation_variance
        self.offset_x += gain * residual_x
        self.offset_y += gain * residual_y
        self.offset_variance = max(
            (1.0 - gain) * self.offset_variance,
            self.config.minimum_uwb_sigma_m ** 2 * 1e-3,
        )
        self.last_gain = gain
        self.accepted_uwb += 1
        return {
            'accepted': 1.0,
            'reason_code': 0.0,
            'gain': gain,
            'innovation_score': score,
        }

    def coarse_pose(self) -> Optional[Pose2D]:
        if self.local_pose is None:
            return None
        cosine = math.cos(self.config.frame_yaw_rad)
        sine = math.sin(self.config.frame_yaw_rad)
        return Pose2D(
            self.local_pose.x * cosine - self.local_pose.y * sine + self.offset_x,
            self.local_pose.x * sine + self.local_pose.y * cosine + self.offset_y,
            wrap_angle(self.local_pose.yaw + self.config.frame_yaw_rad),
        )

    def report(self) -> Dict[str, object]:
        coarse = self.coarse_pose()
        return {
            'local_pose': None if self.local_pose is None else vars(self.local_pose),
            'coarse_pose': None if coarse is None else vars(coarse),
            'global_offset_m': {'x': self.offset_x, 'y': self.offset_y},
            'frame_yaw_rad': self.config.frame_yaw_rad,
            'offset_sigma_m': math.sqrt(self.offset_variance),
            'last_gain': self.last_gain,
            'last_innovation_score': self.last_innovation_score,
            'accepted_uwb': self.accepted_uwb,
            'rejected_uwb': self.rejected_uwb,
            'odom_resets': self.odom_resets,
        }
