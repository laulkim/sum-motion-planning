from __future__ import annotations

from typing import Any

import matplotlib

matplotlib.use("Agg")

import numpy as np
from matplotlib.patches import Polygon

from .debug_plot_renderer import _array, _draw_scenario_obstacles, _vehicle_polygon


def draw_candidate_panel(
    map_ax: Any,
    snapshot: dict[str, Any],
    snapshot_index: int,
    elapsed_seconds: float,
) -> None:
    """Draw the world-frame scenario-map panel into an existing axis.

    Same layers as the main dashboard's map panel (global path, odometry,
    active reference, selected trajectory, obstacles, vehicle, global
    projection) but without the costmap/local-window layers. Used as one
    panel of combined_snapshot_renderer's stacked figure.
    """
    global_x = _array(snapshot, "global_x")
    global_y = _array(snapshot, "global_y")
    reference_x = _array(snapshot, "reference_x")
    reference_y = _array(snapshot, "reference_y")
    selected_x = _array(snapshot, "selected_x")
    selected_y = _array(snapshot, "selected_y")
    x_history = _array(snapshot, "x_history")
    y_history = _array(snapshot, "y_history")

    if global_x.size:
        map_ax.plot(global_x, global_y, label="Global path")
    if x_history.size:
        map_ax.plot(x_history, y_history, label="Odometry")
    if reference_x.size:
        map_ax.plot(reference_x, reference_y, "--", label="Active reference")
    if selected_x.size:
        map_ax.plot(selected_x, selected_y, ":", linewidth=2.0, label="Selected trajectory")

    obstacle_artists = _draw_scenario_obstacles(map_ax, snapshot)

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

    map_x_parts: list[np.ndarray] = []
    map_y_parts: list[np.ndarray] = []
    for x_values, y_values in (
        (global_x, global_y), (x_history, y_history),
        (reference_x, reference_y), (selected_x, selected_y),
    ):
        if x_values.size and y_values.size:
            map_x_parts.append(x_values)
            map_y_parts.append(y_values)
    for obstacle in obstacle_artists:
        vertices = np.asarray(obstacle.get_xy(), dtype=float)
        map_x_parts.append(vertices[:, 0])
        map_y_parts.append(vertices[:, 1])
    if vehicle_artist is not None:
        vertices = np.asarray(vehicle_artist.get_xy(), dtype=float)
        map_x_parts.append(vertices[:, 0])
        map_y_parts.append(vertices[:, 1])
    if map_x_parts:
        map_x = np.concatenate(map_x_parts)
        map_y = np.concatenate(map_y_parts)
        finite = np.isfinite(map_x) & np.isfinite(map_y)
        if np.any(finite):
            x_min, x_max = float(np.min(map_x[finite])), float(np.max(map_x[finite]))
            y_min, y_max = float(np.min(map_y[finite])), float(np.max(map_y[finite]))
            map_ax.set_xlim(
                x_min - max(4.0, 0.08 * max(x_max - x_min, 1.0)),
                x_max + max(4.0, 0.08 * max(x_max - x_min, 1.0)),
            )
            map_ax.set_ylim(
                y_min - max(24.0, 0.96 * max(y_max - y_min, 1.0)),
                y_max + max(24.0, 0.96 * max(y_max - y_min, 1.0)),
            )
    map_ax.set_aspect("equal", adjustable="box")
    map_ax.set_title(
        "Scenario map, obstacles, and executed motion "
        f"(candidate snapshot #{snapshot_index}, t={elapsed_seconds:.2f}s)"
    )
    map_ax.set_xlabel("x [m]")
    map_ax.set_ylabel("y [m]")
    map_ax.grid(True)
    map_ax.legend(loc="upper left", ncol=2, fontsize=8.2)
