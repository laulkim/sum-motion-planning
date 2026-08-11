from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

import numpy as np


def wrap_angle(angle):
    return (np.asarray(angle) + np.pi) % (2.0 * np.pi) - np.pi


@dataclass(frozen=True)
class PathProjection:
    segment_index: int
    t: float
    x: float
    y: float
    s: float
    yaw: float
    kappa: float
    lateral_error: float
    distance: float


def _candidate_indices(
    point_count: int,
    previous_segment: Optional[int],
    search_back: int,
    search_forward: int,
) -> np.ndarray:
    if previous_segment is None or point_count <= search_back + search_forward + 1:
        return np.arange(point_count, dtype=int)
    offsets = np.arange(-search_back, search_forward + 1, dtype=int)
    return np.unique((int(previous_segment) + offsets) % point_count)


def _project_candidates(
    px: float,
    py: float,
    x: np.ndarray,
    y: np.ndarray,
    yaw: np.ndarray,
    kappa: np.ndarray,
    s: np.ndarray,
    segment_length: np.ndarray,
    total_length: float,
    candidate_indices: np.ndarray,
) -> PathProjection:
    next_indices = (candidate_indices + 1) % len(x)
    ax = x[candidate_indices]
    ay = y[candidate_indices]
    bx = x[next_indices]
    by = y[next_indices]
    dx = bx - ax
    dy = by - ay
    denominator = dx * dx + dy * dy
    t = np.clip(((px - ax) * dx + (py - ay) * dy) / denominator, 0.0, 1.0)
    qx = ax + t * dx
    qy = ay + t * dy
    distance_squared = (px - qx) ** 2 + (py - qy) ** 2
    local_best = int(np.argmin(distance_squared))

    segment_index = int(candidate_indices[local_best])
    ratio = float(t[local_best])
    projected_x = float(qx[local_best])
    projected_y = float(qy[local_best])

    next_index = (segment_index + 1) % len(x)
    yaw_delta = float(wrap_angle(yaw[next_index] - yaw[segment_index]))
    projected_yaw = float(wrap_angle(yaw[segment_index] + ratio * yaw_delta))
    projected_kappa = float(
        kappa[segment_index]
        + ratio * (kappa[next_index] - kappa[segment_index])
    )
    projected_s = float(
        s[segment_index] + ratio * segment_length[segment_index]
    )
    if projected_s >= total_length:
        projected_s -= total_length

    error_x = px - projected_x
    error_y = py - projected_y
    lateral_error = float(
        -math.sin(projected_yaw) * error_x
        + math.cos(projected_yaw) * error_y
    )
    distance = math.hypot(error_x, error_y)

    return PathProjection(
        segment_index=segment_index,
        t=ratio,
        x=projected_x,
        y=projected_y,
        s=projected_s,
        yaw=projected_yaw,
        kappa=projected_kappa,
        lateral_error=lateral_error,
        distance=distance,
    )


def project_closed_path(
    px: float,
    py: float,
    *,
    x: np.ndarray,
    y: np.ndarray,
    yaw: np.ndarray,
    kappa: np.ndarray,
    s: np.ndarray,
    segment_length: np.ndarray,
    total_length: float,
    previous_segment: Optional[int] = None,
    search_back: int = 20,
    search_forward: int = 80,
    fallback_distance: float = 3.0,
) -> PathProjection:
    arrays = [
        np.asarray(value, dtype=float)
        for value in (x, y, yaw, kappa, s, segment_length)
    ]
    x, y, yaw, kappa, s, segment_length = arrays
    if not (
        len(x)
        == len(y)
        == len(yaw)
        == len(kappa)
        == len(s)
        == len(segment_length)
    ):
        raise ValueError("Projection arrays must have equal length")
    if len(x) < 2:
        raise ValueError("Projection requires at least two path points")
    if not all(math.isfinite(value) for value in (px, py, total_length)):
        raise ValueError("Projection input contains NaN or Inf")

    candidates = _candidate_indices(
        len(x), previous_segment, search_back, search_forward
    )
    projection = _project_candidates(
        float(px),
        float(py),
        x,
        y,
        yaw,
        kappa,
        s,
        segment_length,
        float(total_length),
        candidates,
    )

    if previous_segment is not None and projection.distance > fallback_distance:
        projection = _project_candidates(
            float(px),
            float(py),
            x,
            y,
            yaw,
            kappa,
            s,
            segment_length,
            float(total_length),
            np.arange(len(x), dtype=int),
        )
    return projection



def _candidate_indices_open(
    segment_count: int,
    previous_segment: Optional[int],
    search_back: int,
    search_forward: int,
) -> np.ndarray:
    if segment_count < 1:
        raise ValueError("segment_count must be positive")
    if previous_segment is None or segment_count <= search_back + search_forward + 1:
        return np.arange(segment_count, dtype=int)
    start = max(0, int(previous_segment) - int(search_back))
    stop = min(segment_count, int(previous_segment) + int(search_forward) + 1)
    return np.arange(start, stop, dtype=int)


def _project_open_candidates(
    px: float,
    py: float,
    x: np.ndarray,
    y: np.ndarray,
    yaw: np.ndarray,
    kappa: np.ndarray,
    s: np.ndarray,
    segment_length: np.ndarray,
    candidate_indices: np.ndarray,
) -> PathProjection:
    next_indices = candidate_indices + 1
    ax = x[candidate_indices]
    ay = y[candidate_indices]
    bx = x[next_indices]
    by = y[next_indices]
    dx = bx - ax
    dy = by - ay
    denominator = dx * dx + dy * dy
    if np.any(denominator <= 1.0e-12):
        raise ValueError("Open path contains a zero-length segment")
    t = np.clip(((px - ax) * dx + (py - ay) * dy) / denominator, 0.0, 1.0)
    qx = ax + t * dx
    qy = ay + t * dy
    distance_squared = (px - qx) ** 2 + (py - qy) ** 2
    local_best = int(np.argmin(distance_squared))

    segment_index = int(candidate_indices[local_best])
    ratio = float(t[local_best])
    projected_x = float(qx[local_best])
    projected_y = float(qy[local_best])
    next_index = segment_index + 1

    yaw_delta = float(wrap_angle(yaw[next_index] - yaw[segment_index]))
    projected_yaw = float(wrap_angle(yaw[segment_index] + ratio * yaw_delta))
    projected_kappa = float(
        kappa[segment_index]
        + ratio * (kappa[next_index] - kappa[segment_index])
    )
    projected_s = float(s[segment_index] + ratio * segment_length[segment_index])

    error_x = px - projected_x
    error_y = py - projected_y
    lateral_error = float(
        -math.sin(projected_yaw) * error_x
        + math.cos(projected_yaw) * error_y
    )
    distance = math.hypot(error_x, error_y)

    return PathProjection(
        segment_index=segment_index,
        t=ratio,
        x=projected_x,
        y=projected_y,
        s=projected_s,
        yaw=projected_yaw,
        kappa=projected_kappa,
        lateral_error=lateral_error,
        distance=distance,
    )


def project_open_path(
    px: float,
    py: float,
    *,
    x: np.ndarray,
    y: np.ndarray,
    yaw: np.ndarray,
    kappa: np.ndarray,
    s: np.ndarray,
    segment_length: np.ndarray,
    previous_segment: Optional[int] = None,
    search_back: int = 20,
    search_forward: int = 80,
    fallback_distance: float = 3.0,
) -> PathProjection:
    """Project a point onto an ordered, non-cyclic reference path."""
    x = np.asarray(x, dtype=float)
    y = np.asarray(y, dtype=float)
    yaw = np.asarray(yaw, dtype=float)
    kappa = np.asarray(kappa, dtype=float)
    s = np.asarray(s, dtype=float)
    segment_length = np.asarray(segment_length, dtype=float)
    if not (len(x) == len(y) == len(yaw) == len(kappa) == len(s)):
        raise ValueError("Open projection point arrays must have equal length")
    if len(segment_length) != len(x) - 1:
        raise ValueError("Open projection requires one segment length per path segment")
    if len(x) < 2:
        raise ValueError("Open projection requires at least two path points")
    if not all(math.isfinite(value) for value in (px, py)):
        raise ValueError("Projection input contains NaN or Inf")

    segment_count = len(x) - 1
    candidates = _candidate_indices_open(
        segment_count,
        previous_segment,
        search_back,
        search_forward,
    )
    projection = _project_open_candidates(
        float(px),
        float(py),
        x,
        y,
        yaw,
        kappa,
        s,
        segment_length,
        candidates,
    )
    if previous_segment is not None and projection.distance > fallback_distance:
        projection = _project_open_candidates(
            float(px),
            float(py),
            x,
            y,
            yaw,
            kappa,
            s,
            segment_length,
            np.arange(segment_count, dtype=int),
        )
    return projection
