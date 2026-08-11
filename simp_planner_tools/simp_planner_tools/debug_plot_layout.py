from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, Optional

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.axes import Axes
from matplotlib.figure import Figure


@dataclass(frozen=True)
class DebugPlotAxes:
    map: Axes
    status: Axes
    speed: Axes
    global_error: Axes
    tracking: Axes
    acceleration: Axes
    jerk: Axes
    curvature: Axes
    continuity: Axes
    timing: Axes
    allocation_velocity: Axes
    allocation_rate_split: Axes
    allocation_beta: Axes
    allocation_acceleration: Axes
    allocation_consistency: Axes


def create_debug_figure() -> tuple[Figure, DebugPlotAxes]:
    """Create the main dashboard plus direct allocator-state diagnostics."""

    figure = plt.figure(figsize=(24.0, 19.2), constrained_layout=True)
    grid = figure.add_gridspec(
        5,
        4,
        width_ratios=(1.0, 1.0, 1.0, 0.88),
        height_ratios=(1.10, 1.0, 0.94, 0.94, 0.56),
        wspace=0.12,
        hspace=0.13,
    )
    axes = DebugPlotAxes(
        map=figure.add_subplot(grid[0, 0:3]),
        status=figure.add_subplot(grid[0, 3]),
        speed=figure.add_subplot(grid[1, 0]),
        global_error=figure.add_subplot(grid[1, 1]),
        tracking=figure.add_subplot(grid[1, 2]),
        timing=figure.add_subplot(grid[1, 3]),
        acceleration=figure.add_subplot(grid[2, 0]),
        jerk=figure.add_subplot(grid[2, 1]),
        curvature=figure.add_subplot(grid[2, 2]),
        continuity=figure.add_subplot(grid[2, 3]),
        allocation_velocity=figure.add_subplot(grid[3, 0]),
        allocation_rate_split=figure.add_subplot(grid[3, 1]),
        allocation_beta=figure.add_subplot(grid[3, 2]),
        allocation_acceleration=figure.add_subplot(grid[3, 3]),
        allocation_consistency=figure.add_subplot(grid[4, 0:4]),
    )
    return figure, axes


def symmetric_limit(
    values: Iterable[float],
    *,
    minimum: float,
    padding: float = 1.15,
) -> float:
    array = np.asarray(list(values), dtype=float)
    finite = array[np.isfinite(array)]
    if finite.size == 0:
        return float(minimum)
    return max(float(minimum), float(np.max(np.abs(finite))) * float(padding))


def first_true_time(times: Iterable[float], flags: Iterable[bool]) -> Optional[float]:
    for value, flag in zip(times, flags):
        if bool(flag):
            return float(value)
    return None


def add_zero_line(axis: Axes) -> None:
    axis.axhline(0.0, linewidth=0.8, alpha=0.35)
