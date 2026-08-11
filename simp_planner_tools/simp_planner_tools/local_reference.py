from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .path_geometry import PathProjection
from .track_map import TrackMap


@dataclass(frozen=True)
class LocalReferenceSlice:
    indices: np.ndarray
    x: np.ndarray
    y: np.ndarray
    yaw: np.ndarray
    kappa: np.ndarray
    mode: np.ndarray


def reference_index_from_projection(
    projection: PathProjection,
    point_count: int,
) -> int:
    if point_count < 2:
        raise ValueError("point_count must be at least two")
    return int(
        projection.segment_index
        if projection.t < 0.5
        else (projection.segment_index + 1) % point_count
    )


def directional_arc_progress(
    previous_s: float,
    current_s: float,
    total_length: float,
    *,
    reverse: bool = False,
) -> float:
    """Return forward route progress while rejecting backward projection noise."""
    if total_length <= 0.0:
        raise ValueError("total_length must be positive")
    if reverse:
        progress = (float(previous_s) - float(current_s)) % float(total_length)
    else:
        progress = (float(current_s) - float(previous_s)) % float(total_length)

    # A jump larger than half a lap is interpreted as a small backward move or
    # projection jitter rather than genuine forward progress.
    if progress > 0.5 * float(total_length):
        return 0.0
    return float(progress)


def should_publish_local_reference(
    previous_s: float | None,
    current_s: float,
    total_length: float,
    minimum_distance: float,
    *,
    reverse: bool = False,
    mode_changed: bool = False,
) -> bool:
    if minimum_distance <= 0.0:
        raise ValueError("minimum_distance must be positive")
    if previous_s is None or mode_changed:
        return True
    return (
        directional_arc_progress(
            previous_s,
            current_s,
            total_length,
            reverse=reverse,
        )
        >= minimum_distance
    )


def move_backward_index(track: TrackMap, index: int, distance: float) -> int:
    if distance < 0.0:
        raise ValueError("distance must be non-negative")
    current = int(index)
    accumulated = 0.0
    for _ in range(len(track.x) - 1):
        previous = (current - 1) % len(track.x)
        accumulated += float(track.segment_length[previous])
        current = previous
        if accumulated >= distance:
            break
    return current


def local_reference_indices(
    track: TrackMap,
    reference_index: int,
    path_back_length: float,
    path_ahead_length: float,
) -> np.ndarray:
    if path_back_length < 0.0:
        raise ValueError("path_back_length must be non-negative")
    if path_ahead_length <= 0.0:
        raise ValueError("path_ahead_length must be positive")

    start = move_backward_index(track, reference_index, path_back_length)
    required_length = path_back_length + path_ahead_length
    indices = [start]
    accumulated = 0.0
    current = start
    for _ in range(len(track.x) - 1):
        accumulated += float(track.segment_length[current])
        current = (current + 1) % len(track.x)
        indices.append(current)
        if accumulated >= required_length and len(indices) >= 4:
            break

    if len(indices) < 4:
        raise RuntimeError("Local path generation produced too few points")
    return np.asarray(indices, dtype=int)


def build_local_reference_slice(
    track: TrackMap,
    reference_index: int,
    path_back_length: float,
    path_ahead_length: float,
) -> LocalReferenceSlice:
    indices = local_reference_indices(
        track,
        reference_index,
        path_back_length,
        path_ahead_length,
    )
    return LocalReferenceSlice(
        indices=indices,
        x=track.x[indices].astype(float),
        y=track.y[indices].astype(float),
        yaw=np.unwrap(track.yaw[indices]).astype(float),
        kappa=track.kappa[indices].astype(float),
        mode=track.mode[indices].astype(np.uint8),
    )
