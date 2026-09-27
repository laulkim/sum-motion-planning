from __future__ import annotations

import shutil
from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt

from .candidate_snapshot_renderer import draw_candidate_panel
from .paper_plot_style import apply_paper_style
from .planning_time_renderer import draw_planning_time_panel

# height_ratios=[1, 1] with this figsize gives each panel the same 13x6
# footprint candidate_snapshot_renderer's own standalone figure uses, so the
# two stack at equal size rather than just equal width.
_FIGSIZE = (13.0, 12.0)


def render_combined_snapshot(
    session_dir_value: str,
    snapshot: dict[str, Any],
    snapshot_index: int,
    elapsed_seconds: float,
    plan_time_history: list[float],
    plan_compute_history: list[float],
    deadline_ms: float,
) -> str | None:
    """Render one combined snapshot: candidate map on top, planning time below.

    Produced on the same 3 s cadence as candidate_snapshots and
    local_costmap_snapshots (called from DebugPlotNode.save_candidate_snapshot),
    using whichever compute-time samples have accumulated so far -- this is a
    live, per-cycle pairing, not an end-of-run summary. candidate_snapshots
    itself is untouched; this is an additional, separate output. Returns None
    (and writes nothing) if no plan has been logged yet.
    """
    session_dir = Path(session_dir_value)
    figure, (top_axis, bottom_axis) = plt.subplots(
        2, 1, figsize=_FIGSIZE, gridspec_kw={"height_ratios": [1, 1]},
    )
    draw_candidate_panel(top_axis, snapshot, snapshot_index, elapsed_seconds)
    if not draw_planning_time_panel(
        bottom_axis, plan_time_history, plan_compute_history, deadline_ms
    ):
        plt.close(figure)
        return None

    for axis in (top_axis, bottom_axis):
        apply_paper_style(axis)

    snapshot_path = session_dir / f"snapshot_{int(snapshot_index):06d}.png"
    latest_path = session_dir / "latest.png"
    figure.tight_layout()
    figure.savefig(snapshot_path, dpi=150)
    plt.close(figure)
    shutil.copy2(snapshot_path, latest_path)
    return str(snapshot_path)
