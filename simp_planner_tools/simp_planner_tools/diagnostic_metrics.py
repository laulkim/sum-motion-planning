from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Optional

import numpy as np

from .path_geometry import PathProjection, project_open_path, wrap_angle


@dataclass(frozen=True)
class TrackingError:
    projection: PathProjection
    lateral_error: float
    motion_direction_error: float


@dataclass(frozen=True)
class OpenPathGeometry:
    x: np.ndarray
    y: np.ndarray
    yaw: np.ndarray
    kappa: np.ndarray
    s: np.ndarray
    segment_length: np.ndarray

    @classmethod
    def from_arrays(cls, x, y, yaw, kappa) -> "OpenPathGeometry":
        x = np.asarray(x, dtype=float)
        y = np.asarray(y, dtype=float)
        yaw = np.unwrap(np.asarray(yaw, dtype=float))
        kappa = np.asarray(kappa, dtype=float)
        if not (len(x) == len(y) == len(yaw) == len(kappa)):
            raise ValueError("Open-path diagnostic arrays must have equal length")
        if len(x) < 2:
            raise ValueError("Open-path diagnostics require at least two points")
        if not all(np.all(np.isfinite(value)) for value in (x, y, yaw, kappa)):
            raise ValueError("Open-path diagnostics contain NaN or Inf")
        segment_length = np.hypot(np.diff(x), np.diff(y))
        if np.any(segment_length <= 1.0e-9):
            raise ValueError("Open-path diagnostics contain zero-length segments")
        s = np.r_[0.0, np.cumsum(segment_length)]
        return cls(x=x, y=y, yaw=yaw, kappa=kappa, s=s, segment_length=segment_length)

    def tracking_error(
        self,
        x: float,
        y: float,
        motion_direction: float,
        *,
        previous_segment: Optional[int] = None,
    ) -> TrackingError:
        projection = project_open_path(
            float(x),
            float(y),
            x=self.x,
            y=self.y,
            yaw=self.yaw,
            kappa=self.kappa,
            s=self.s,
            segment_length=self.segment_length,
            previous_segment=previous_segment,
            search_back=8,
            search_forward=40,
            fallback_distance=2.0,
        )
        return TrackingError(
            projection=projection,
            lateral_error=float(projection.lateral_error),
            motion_direction_error=float(
                wrap_angle(float(motion_direction) - float(projection.yaw))
            ),
        )


def tracking_error_to_executed_segment(
    x: float,
    y: float,
    motion_direction: float,
    *,
    start_x: float,
    start_y: float,
    start_heading: float,
    end_x: float,
    end_y: float,
    end_heading: float,
) -> TrackingError:
    """Track the body against the command segment that is actually active.

    Comparing odometry with the newest complete replanned trajectory can create
    artificial spikes whenever a plan is replaced.  The executable command is a
    zero-order-held segment between two consecutive trajectory states, so this
    metric projects only onto that segment and interpolates its motion heading.
    """
    px = float(x)
    py = float(y)
    x0 = float(start_x)
    y0 = float(start_y)
    x1 = float(end_x)
    y1 = float(end_y)
    dx = x1 - x0
    dy = y1 - y0
    length2 = dx * dx + dy * dy
    if length2 <= 1.0e-12:
        tau = 0.0
        projected_x = x0
        projected_y = y0
        segment_yaw = float(start_heading)
        segment_length = 0.0
    else:
        tau = float(np.clip(((px - x0) * dx + (py - y0) * dy) / length2, 0.0, 1.0))
        projected_x = x0 + tau * dx
        projected_y = y0 + tau * dy
        heading_delta = float(wrap_angle(float(end_heading) - float(start_heading)))
        segment_yaw = float(start_heading) + tau * heading_delta
        segment_length = math.sqrt(length2)
    ex = px - projected_x
    ey = py - projected_y
    lateral_error = -math.sin(segment_yaw) * ex + math.cos(segment_yaw) * ey
    projection = PathProjection(
        segment_index=0,
        t=float(tau),
        x=float(projected_x),
        y=float(projected_y),
        s=float(tau * segment_length),
        yaw=float(wrap_angle(segment_yaw)),
        kappa=0.0,
        lateral_error=float(lateral_error),
        distance=float(math.hypot(ex, ey)),
    )
    return TrackingError(
        projection=projection,
        lateral_error=float(lateral_error),
        motion_direction_error=float(wrap_angle(float(motion_direction) - segment_yaw)),
    )


@dataclass
class TimingStatistics:
    deadline_ms: float
    samples_ms: list[float] = field(default_factory=list)

    def add(self, value_ms: float) -> None:
        value = float(value_ms)
        if not math.isfinite(value) or value < 0.0:
            raise ValueError("Timing sample must be finite and non-negative")
        self.samples_ms.append(value)

    def summary(self) -> dict[str, float | int]:
        if not self.samples_ms:
            return {
                "count": 0,
                "mean_ms": math.nan,
                "p95_ms": math.nan,
                "max_ms": math.nan,
                "deadline_ms": float(self.deadline_ms),
                "deadline_miss_count": 0,
                "deadline_miss_ratio": 0.0,
            }
        values = np.asarray(self.samples_ms, dtype=float)
        misses = int(np.count_nonzero(values > float(self.deadline_ms)))
        return {
            "count": int(len(values)),
            "mean_ms": float(np.mean(values)),
            "p95_ms": float(np.percentile(values, 95.0)),
            "max_ms": float(np.max(values)),
            "deadline_ms": float(self.deadline_ms),
            "deadline_miss_count": misses,
            "deadline_miss_ratio": float(misses / len(values)),
        }


@dataclass
class PlanContinuityStatistics:
    previous_candidate_id: Optional[int] = None
    previous_n_target: Optional[float] = None
    candidate_switch_count: int = 0
    terminal_offset_switch_count: int = 0
    maximum_terminal_offset_jump: float = 0.0
    maximum_curvature_jump: float = 0.0
    maximum_heading_rate_jump: float = 0.0

    def update(
        self,
        *,
        candidate_id: int,
        n_target: float,
        curvature_jump: float,
        heading_rate_jump: float,
    ) -> dict[str, float | int | bool]:
        candidate_id = int(candidate_id)
        n_target = float(n_target)
        candidate_switched = (
            self.previous_candidate_id is not None
            and candidate_id != self.previous_candidate_id
        )
        n_target_jump = (
            0.0
            if self.previous_n_target is None
            else n_target - self.previous_n_target
        )
        terminal_offset_switched = (
            self.previous_n_target is not None and abs(n_target_jump) > 1.0e-6
        )
        if candidate_switched:
            self.candidate_switch_count += 1
        if terminal_offset_switched:
            self.terminal_offset_switch_count += 1
        self.maximum_terminal_offset_jump = max(
            self.maximum_terminal_offset_jump, abs(n_target_jump)
        )
        self.maximum_curvature_jump = max(
            self.maximum_curvature_jump, abs(float(curvature_jump))
        )
        self.maximum_heading_rate_jump = max(
            self.maximum_heading_rate_jump, abs(float(heading_rate_jump))
        )
        self.previous_candidate_id = candidate_id
        self.previous_n_target = n_target
        return {
            "candidate_switched": bool(candidate_switched),
            "terminal_offset_switched": bool(terminal_offset_switched),
            "n_target_jump": float(n_target_jump),
            "candidate_switch_count": int(self.candidate_switch_count),
            "terminal_offset_switch_count": int(self.terminal_offset_switch_count),
            "maximum_terminal_offset_jump": float(self.maximum_terminal_offset_jump),
            "maximum_curvature_jump": float(self.maximum_curvature_jump),
            "maximum_heading_rate_jump": float(self.maximum_heading_rate_jump),
        }
