from __future__ import annotations

from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np

from .paper_plot_style import apply_paper_style

MODE_NAMES = {0: "FORWARD", 1: "REVERSE", 2: "LEFT", 3: "RIGHT", 4: "SPOT_TURN"}


def render_mode_transition_summary(
    session_dir_value: str,
    time_history: list[float],
    mode_history: list[int],
) -> str | None:
    """Render one step-plot of drive-mode transitions over the whole run.

    Unlike the periodic candidate/local-costmap snapshots, this is a single
    end-of-run summary -- called once from DebugPlotNode.destroy_node(),
    i.e. when the launch is stopped (Ctrl+C). Returns None (and writes
    nothing) if no odometry was ever recorded.
    """
    if not time_history:
        return None

    session_dir = Path(session_dir_value)
    time = np.asarray(time_history, dtype=float)
    mode = np.asarray(mode_history, dtype=float)
    mode = np.where(mode < 0, np.nan, mode)

    figure, axis = plt.subplots(figsize=(13.0, 4.5))
    axis.step(time, mode, where="post", linewidth=2.0, label="Confirmed mode")

    tick_codes = sorted(MODE_NAMES.keys())
    axis.set_yticks(tick_codes)
    axis.set_yticklabels([MODE_NAMES[code] for code in tick_codes])
    axis.set_ylim(min(tick_codes) - 0.5, max(tick_codes) + 0.5)
    axis.set_xlabel("time [s]")
    axis.set_title("Drive mode transition history")
    axis.grid(True, axis="x")
    apply_paper_style(axis)

    summary_path = session_dir / "mode_transition_summary.png"
    figure.tight_layout()
    figure.savefig(summary_path, dpi=150)
    plt.close(figure)
    return str(summary_path)
