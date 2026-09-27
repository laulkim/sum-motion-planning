from __future__ import annotations

import shutil
from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Polygon

from .debug_plot_renderer import (
    _array,
    _costmap_geometry,
    _draw_costmap,
    _draw_scenario_obstacles,
    _vehicle_polygon,
)
from .paper_plot_style import apply_paper_style

# Every candidate that never made the shortlist is a faint "sampled" line;
# candidates that reached full trajectory/cost evaluation (core.cpp's
# `full` vector, capped at normal_shortlist_size=9) are a darker "Top-K"
# line -- the one that got activated this cycle is one of these 9 and is
# additionally re-drawn on top as the "Best-ranked" line, not excluded
# from the Top-K group.
_SAMPLED_STYLE = {"color": "steelblue", "linewidth": 0.8, "alpha": 0.35, "zorder": 3}
_TOPK_STYLE = {"color": "orange", "linewidth": 1.6, "alpha": 0.55, "zorder": 4}
_BEST_STYLE = {"color": "black", "linewidth": 2.2, "alpha": 0.95, "zorder": 5,
               "label": "Best-ranked path"}


def _draw_candidates(map_ax: Any, snapshot: dict[str, Any]) -> None:
    """Draw sampled and Top-K candidate paths, with the best one re-drawn on top.

    Populated only when the planner's export_debug_candidates parameter is
    on (see core.cpp's PathVelocityPlanner::plan); otherwise draws nothing.
    """
    labelled_sampled = False
    labelled_topk = False
    for candidate in snapshot.get("candidates", []) or []:
        x_values = np.asarray(candidate.get("x", []), dtype=float)
        y_values = np.asarray(candidate.get("y", []), dtype=float)
        if x_values.size < 2 or y_values.size < 2:
            continue
        if candidate.get("trajectory_evaluated"):
            map_ax.plot(
                x_values, y_values,
                label=None if labelled_topk else "Top-K candidate paths",
                **_TOPK_STYLE,
            )
            labelled_topk = True
        else:
            map_ax.plot(
                x_values, y_values,
                label=None if labelled_sampled else "Sampled candidate paths",
                **_SAMPLED_STYLE,
            )
            labelled_sampled = True

    for candidate in snapshot.get("candidates", []) or []:
        if not candidate.get("selected"):
            continue
        x_values = np.asarray(candidate.get("x", []), dtype=float)
        y_values = np.asarray(candidate.get("y", []), dtype=float)
        if x_values.size < 2 or y_values.size < 2:
            continue
        map_ax.plot(x_values, y_values, **_BEST_STYLE)
        break


def draw_local_costmap_panel(map_ax: Any, snapshot: dict[str, Any]) -> None:
    """Draw the full local-costmap panel (grid, candidates, reference,
    vehicle, projections) into an existing axis.

    Shared by render_local_costmap_snapshot (its own figure) and
    combined_summary_renderer (as one panel of a stacked figure) so both
    stay pixel-identical.
    """
    _draw_costmap(map_ax, snapshot)
    obstacle_artists = _draw_scenario_obstacles(map_ax, snapshot)
    _draw_candidates(map_ax, snapshot)

    reference_x = _array(snapshot, "reference_x")
    reference_y = _array(snapshot, "reference_y")
    if reference_x.size:
        map_ax.plot(reference_x, reference_y, ":", linewidth=1.8, zorder=6,
                    label="Reference path")

    state = snapshot.get("current_state") or {}
    projection = snapshot.get("current_projection") or {}
    selected_projection = snapshot.get("selected_projection") or {}
    vehicle_artist: Polygon | None = None
    if state:
        vehicle_artist = Polygon(
            _vehicle_polygon(
                float(state["x"]),
                float(state["y"]),
                float(state["body_yaw"]),
                float(snapshot["vehicle_length"]),
                float(snapshot["vehicle_width"]),
            ),
            closed=True,
            fill=False,
            linewidth=2.0,
            zorder=7,
            label="Vehicle",
        )
        map_ax.add_patch(vehicle_artist)
    if projection:
        map_ax.scatter(
            [projection["x"]], [projection["y"]], marker="x", s=70,
            zorder=8, label="Global projection",
        )
    if selected_projection:
        map_ax.scatter(
            [selected_projection["x"]], [selected_projection["y"]],
            marker="+", s=70, zorder=8, label="Selected projection",
        )

    # Fixed to the received costmap's own extent -- not autoscaled from
    # path data -- so the whole local costmap is always visible, matching
    # what the planner actually sees rather than whatever happens to be
    # plotted this cycle.
    costmap_data = snapshot.get("costmap_data")
    expected_shape = np.asarray(costmap_data).shape if costmap_data is not None else None
    geometry = _costmap_geometry(snapshot, expected_shape)
    if geometry is not None:
        corners = geometry["corners"]
        x_min, y_min = corners.min(axis=0)
        x_max, y_max = corners.max(axis=0)
        padding = 2.0
        map_ax.set_xlim(x_min - padding, x_max + padding)
        map_ax.set_ylim(y_min - padding, y_max + padding)
    elif vehicle_artist is not None and state:
        half_window = 45.0
        map_ax.set_xlim(float(state["x"]) - half_window, float(state["x"]) + half_window)
        map_ax.set_ylim(float(state["y"]) - half_window, float(state["y"]) + half_window)

    map_ax.set_aspect("equal", adjustable="box")
    map_ax.set_title("Candidate path sampling and Top-K selection")
    map_ax.set_xlabel("x [m]")
    map_ax.set_ylabel("y [m]")
    map_ax.grid(True)
    map_ax.legend(loc="upper left", ncol=2, fontsize=7.6)


def render_local_costmap_snapshot(
    snapshot: dict[str, Any],
    session_dir_value: str,
    snapshot_index: int,
    elapsed_seconds: float,
) -> str:
    """Render one local, costmap-scale snapshot into ``<session_dir>``.

    Unlike candidate_snapshot_renderer's world-scale map (global path,
    full odometry trail), this is zoomed to exactly the vehicle-centred
    costmap window (see costmap_size_m in simulation.launch.py, 90x90 m by
    default) so everything inside the local costmap is visible: the grid
    itself, obstacles, the sampled/Top-K/best-ranked candidate tiers (see
    _draw_candidates), the reference path, and vehicle.
    """
    session_dir = Path(session_dir_value)
    figure, map_ax = plt.subplots(figsize=(9.0, 9.0))
    draw_local_costmap_panel(map_ax, snapshot)
    apply_paper_style(map_ax)

    snapshot_path = session_dir / f"snapshot_{int(snapshot_index):06d}.png"
    latest_path = session_dir / "latest.png"
    figure.tight_layout()
    figure.savefig(snapshot_path, dpi=150)
    plt.close(figure)
    shutil.copy2(snapshot_path, latest_path)
    return str(snapshot_path)
