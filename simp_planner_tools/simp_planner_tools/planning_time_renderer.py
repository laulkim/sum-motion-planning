from __future__ import annotations

from typing import Any

import matplotlib

matplotlib.use("Agg")

import numpy as np


def draw_planning_time_panel(
    axis: Any,
    plan_time_history: list[float],
    plan_compute_history: list[float],
    deadline_ms: float,
) -> bool:
    """Draw the planning-compute-time panel into an existing axis.

    Used as one panel of combined_snapshot_renderer's stacked figure.
    Returns False (and draws nothing) if no plan was ever logged.
    """
    time = np.asarray(plan_time_history, dtype=float)
    compute = np.asarray(plan_compute_history, dtype=float)
    finite = np.isfinite(time) & np.isfinite(compute)
    time, compute = time[finite], compute[finite]
    if compute.size == 0:
        return False

    minimum = float(compute.min())
    maximum = float(compute.max())
    mean = float(compute.mean())

    axis.plot(time, compute, linewidth=1.0, marker=".", markersize=3,
              label="Compute time per cycle")
    axis.axhline(deadline_ms, color="red", linestyle="--", linewidth=1.5,
                 label=f"Deadline ({deadline_ms:.1f} ms)")
    axis.axhline(maximum, color="tab:orange", linestyle=":", linewidth=1.3,
                 label=f"Max ({maximum:.2f} ms)")
    axis.axhline(mean, color="tab:blue", linestyle="-.", linewidth=1.3,
                 label=f"Mean ({mean:.2f} ms)")
    axis.axhline(minimum, color="tab:green", linestyle=":", linewidth=1.3,
                 label=f"Min ({minimum:.2f} ms)")
    axis.set_xlabel("time [s]")
    axis.set_ylabel("planning compute time [ms]")
    axis.set_title("Planning compute time per cycle")
    axis.grid(True)
    axis.legend(loc="upper right", fontsize=8.0)
    return True
