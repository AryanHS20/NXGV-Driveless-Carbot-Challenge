"""Sparse odometry-fixed memory for recent Stage 2 road evidence."""

from dataclasses import dataclass
import math
from typing import Dict, Tuple

import numpy as np

from .bev_core import CameraProfile, bev_to_metric, metric_to_bev
from .road_mask_core import RoadMaskError


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class MemoryCell:
    x: float
    y: float
    stamp: float
    distance: float


class LocalRoadMemory:
    """Quantized road observations in an odometry-fixed metric frame."""

    def __init__(
        self,
        cell_size_m: float = 0.025,
        retention_sec: float = 25.0,
        max_cells: int = 50000,
    ) -> None:
        if cell_size_m <= 0.0 or retention_sec <= 0.0 or max_cells < 1:
            raise RoadMaskError('memory dimensions and limits must be positive')
        self.cell_size_m = float(cell_size_m)
        self.retention_sec = float(retention_sec)
        self.max_cells = int(max_cells)
        self.cells: Dict[Tuple[int, int], MemoryCell] = {}

    def clear(self) -> None:
        self.cells.clear()

    def _key(self, x: float, y: float) -> Tuple[int, int]:
        return (
            int(round(x / self.cell_size_m)),
            int(round(y / self.cell_size_m)),
        )

    def expire(self, now: float) -> int:
        expired = [
            key for key, value in self.cells.items()
            if now - value.stamp > self.retention_sec
        ]
        for key in expired:
            del self.cells[key]
        return len(expired)

    def integrate(
        self,
        connected_mask: np.ndarray,
        profile: CameraProfile,
        pose: Pose2D,
        stamp: float,
        distance: float,
    ) -> int:
        if connected_mask is None or connected_mask.ndim != 2:
            raise RoadMaskError('memory integration mask must be single-channel')
        expected = (profile.output_size[1], profile.output_size[0])
        if connected_mask.shape != expected:
            raise RoadMaskError(
                f'memory mask shape {connected_mask.shape} does not match {expected}'
            )
        if not all(math.isfinite(v) for v in (pose.x, pose.y, pose.yaw, stamp, distance)):
            raise RoadMaskError('memory pose, stamp and distance must be finite')

        stride = max(1, int(round(self.cell_size_m * profile.pixels_per_meter)))
        sampled = connected_mask[::stride, ::stride]
        rows, columns = np.nonzero(sampled > 0)
        if rows.size == 0:
            self.expire(stamp)
            return 0
        pixels = np.column_stack((columns * stride, rows * stride)).astype(np.float64)
        local = bev_to_metric(profile, pixels)
        cosine = math.cos(pose.yaw)
        sine = math.sin(pose.yaw)
        world_x = pose.x + local[:, 0] * cosine - local[:, 1] * sine
        world_y = pose.y + local[:, 0] * sine + local[:, 1] * cosine
        for x, y in zip(world_x, world_y):
            key = self._key(float(x), float(y))
            self.cells[key] = MemoryCell(float(x), float(y), float(stamp), float(distance))
        self.expire(stamp)
        if len(self.cells) > self.max_cells:
            oldest = sorted(self.cells, key=lambda key: self.cells[key].stamp)
            for key in oldest[:len(self.cells) - self.max_cells]:
                del self.cells[key]
        return int(rows.size)

    def render(
        self,
        profile: CameraProfile,
        pose: Pose2D,
        now: float,
        distance: float,
        max_age_sec: float,
        max_distance_m: float,
    ) -> np.ndarray:
        if max_age_sec <= 0.0 or max_distance_m < 0.0:
            raise RoadMaskError('memory planning limits are invalid')
        width, height = profile.output_size
        output = np.zeros((height, width), dtype=np.uint8)
        cosine = math.cos(pose.yaw)
        sine = math.sin(pose.yaw)
        local_points = []
        for cell in self.cells.values():
            if now - cell.stamp > max_age_sec:
                continue
            if abs(distance - cell.distance) > max_distance_m:
                continue
            dx = cell.x - pose.x
            dy = cell.y - pose.y
            forward = dx * cosine + dy * sine
            left = -dx * sine + dy * cosine
            if not (
                profile.forward_bounds_m[0] <= forward <= profile.forward_bounds_m[1]
                and profile.left_bounds_m[0] <= left <= profile.left_bounds_m[1]
            ):
                continue
            local_points.append((forward, left))
        if not local_points:
            return output
        pixels = metric_to_bev(profile, np.asarray(local_points, dtype=np.float64))
        columns = np.rint(pixels[:, 0]).astype(int)
        rows = np.rint(pixels[:, 1]).astype(int)
        valid = (
            (columns >= 0) & (columns < width)
            & (rows >= 0) & (rows < height)
        )
        output[rows[valid], columns[valid]] = 255
        radius = max(1, int(round(0.5 * self.cell_size_m * profile.pixels_per_meter)))
        if radius > 0:
            import cv2
            kernel = np.ones((radius * 2 + 1, radius * 2 + 1), np.uint8)
            output = cv2.dilate(output, kernel)
        return output

    def stats(self) -> Dict[str, float]:
        return {
            'cells': len(self.cells),
            'cell_size_m': self.cell_size_m,
            'retention_sec': self.retention_sec,
        }
