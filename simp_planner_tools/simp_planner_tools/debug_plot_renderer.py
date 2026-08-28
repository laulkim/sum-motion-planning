from __future__ import annotations

import math
import shutil
from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Polygon
from matplotlib.transforms import Affine2D

from .debug_plot_layout import add_zero_line, create_debug_figure, first_true_time, symmetric_limit


def _array(snapshot: dict[str, Any], name: str) -> np.ndarray:
    return np.asarray(snapshot.get(name, []), dtype=float)


def _format_number(value: object, digits: int = 3) -> str:
    try:
        numeric = float(value)
    except (TypeError, ValueError):
        return "N/A"
    if not math.isfinite(numeric):
        return "N/A"
    return f"{numeric:.{digits}f}"


def _vehicle_polygon(x: float, y: float, yaw: float, length: float, width: float) -> np.ndarray:
    corners = np.asarray(
        [
            [0.5 * length, 0.5 * width],
            [0.5 * length, -0.5 * width],
            [-0.5 * length, -0.5 * width],
            [-0.5 * length, 0.5 * width],
        ],
        dtype=float,
    )
    c = math.cos(yaw)
    s = math.sin(yaw)
    return corners @ np.asarray([[c, -s], [s, c]]).T + np.asarray([x, y])


def _costmap_geometry(
    snapshot: dict[str, Any],
    expected_shape: tuple[int, int] | None = None,
    prefix: str = "costmap",
) -> dict[str, Any] | None:
    """Return validated grid geometry expressed in odom coordinates.

    ``prefix`` selects which set of snapshot keys to read: "costmap" is the
    full grid as received from the sensor/scenario side; "costmap_crop" is
    the (usually smaller, reference-path-slice-sized) window the planner
    actually ran its distance transform over -- see
    LOCAL_COSTMAP_REDESIGN_KR.md. Both share this same key shape:
    ``{prefix}_origin`` (dict with x/y/yaw), ``{prefix}_resolution``,
    ``{prefix}_width``, ``{prefix}_height``.
    """
    origin = snapshot.get(f"{prefix}_origin")
    try:
        origin_x = float(origin["x"])
        origin_y = float(origin["y"])
        origin_yaw = float(origin["yaw"])
        resolution = float(snapshot.get(f"{prefix}_resolution"))
        default_height, default_width = expected_shape or (0, 0)
        width = int(snapshot.get(f"{prefix}_width", default_width))
        height = int(snapshot.get(f"{prefix}_height", default_height))
    except (KeyError, TypeError, ValueError, OverflowError):
        return None
    if not (
        math.isfinite(origin_x)
        and math.isfinite(origin_y)
        and math.isfinite(origin_yaw)
        and math.isfinite(resolution)
        and resolution > 0.0
        and width > 0
        and height > 0
        and (expected_shape is None or expected_shape == (height, width))
    ):
        return None

    width_m = width * resolution
    height_m = height * resolution
    local_corners = np.asarray(
        [[0.0, 0.0], [width_m, 0.0], [width_m, height_m], [0.0, height_m]],
        dtype=float,
    )
    c = math.cos(origin_yaw)
    s = math.sin(origin_yaw)
    rotation = np.asarray([[c, -s], [s, c]], dtype=float)
    corners = local_corners @ rotation.T + np.asarray([origin_x, origin_y])
    center = np.asarray([[0.5 * width_m, 0.5 * height_m]]) @ rotation.T
    center = center[0] + np.asarray([origin_x, origin_y])
    return {
        "origin_x": origin_x,
        "origin_y": origin_y,
        "yaw": origin_yaw,
        "resolution": resolution,
        "width": width,
        "height": height,
        "width_m": width_m,
        "height_m": height_m,
        "corners": corners,
        "center": center,
    }


def _draw_costmap(map_ax: Any, snapshot: dict[str, Any]) -> Any:
    """Draw a costmap in odom coordinates, with legacy extent fallback."""
    costmap = snapshot.get("costmap_data")
    if costmap is None:
        return None

    costmap_array = np.asarray(costmap)
    if costmap_array.ndim != 2:
        return None
    occupied = np.ma.masked_where(costmap_array < 50, costmap_array)

    geometry = _costmap_geometry(snapshot, costmap_array.shape)
    if geometry is not None:
        local_extent = (
            0.0,
            geometry["width_m"],
            0.0,
            geometry["height_m"],
        )
        grid_to_odom = (
            Affine2D()
            .rotate(geometry["yaw"])
            .translate(geometry["origin_x"], geometry["origin_y"])
        )
        return map_ax.imshow(
            occupied,
            origin="lower",
            extent=local_extent,
            transform=grid_to_odom + map_ax.transData,
            interpolation="nearest",
            alpha=0.45,
        )

    # Snapshots written before rotated-grid metadata was introduced contain
    # only an odom-axis-aligned extent. Keep rendering those unchanged.
    extent = snapshot.get("costmap_extent")
    if extent is None:
        return None
    return map_ax.imshow(
        occupied,
        origin="lower",
        extent=extent,
        interpolation="nearest",
        alpha=0.45,
    )


def _draw_scenario_obstacles(
    map_ax: Any, snapshot: dict[str, Any]
) -> list[Polygon]:
    """Draw every authored scenario obstacle, including those outside the local map."""
    valid: list[np.ndarray] = []
    for vertices in snapshot.get("scenario_obstacles", ()) or ():
        array = np.asarray(vertices, dtype=float)
        if array.ndim == 2 and array.shape[0] >= 3 and array.shape[1] == 2:
            if np.all(np.isfinite(array)):
                valid.append(array)

    artists: list[Polygon] = []
    for index, vertices in enumerate(valid):
        artist = Polygon(
            vertices,
            closed=True,
            facecolor="tab:red",
            edgecolor="darkred",
            linewidth=1.1,
            alpha=0.24,
            zorder=3,
            label=(f"Scenario obstacles ({len(valid)} total)" if index == 0 else None),
        )
        map_ax.add_patch(artist)
        artists.append(artist)
    return artists


def _costmap_is_vehicle_centered(snapshot: dict[str, Any]) -> bool:
    semantic = snapshot.get("costmap_vehicle_centered")
    if semantic is not None:
        return bool(semantic)
    return str(snapshot.get("scenario_name", "")).strip().lower() != "track_map"


def _draw_costmap_boundary(map_ax: Any, snapshot: dict[str, Any]) -> Polygon | None:
    """Outline the region the planner actually ran its distance transform over.

    Prefers ``costmap_crop_*`` (the planner's reference-path-slice crop of
    the received grid, see LOCAL_COSTMAP_REDESIGN_KR.md) and falls back to
    the raw received ``costmap_*`` grid when a snapshot has no crop metadata
    (older snapshots, or before the planner has both a path and odom to crop
    with). The crop is generally NOT centred on the vehicle -- it can reach
    much further ahead than behind, or further to one side than the other --
    so the marker/annotation report the vehicle's actual local position
    rather than assuming it sits at the box's geometric centre.
    """
    geometry = _costmap_geometry(snapshot, prefix="costmap_crop")
    is_planner_window = geometry is not None
    if geometry is None:
        geometry = _costmap_geometry(snapshot)
    if geometry is None:
        return None
    width_m = float(geometry["width_m"])
    height_m = float(geometry["height_m"])
    vehicle_centered = _costmap_is_vehicle_centered(snapshot)
    label = f"Costmap {width_m:.1f}×{height_m:.1f} m"
    if vehicle_centered:
        qualifier = "planner window" if is_planner_window else "as received"
        label = f"Local costmap {width_m:.1f}×{height_m:.1f} m ({qualifier})"
    boundary = Polygon(
        geometry["corners"],
        closed=True,
        fill=False,
        edgecolor="tab:cyan",
        linestyle="--",
        linewidth=1.8,
        zorder=5,
        label=label,
    )
    map_ax.add_patch(boundary)
    first_corner = geometry["corners"][0]
    annotation = f"{width_m:.1f} × {height_m:.1f} m"
    if vehicle_centered:
        state = snapshot.get("current_state") or {}
        try:
            vehicle_x = float(state["x"])
            vehicle_y = float(state["y"])
        except (KeyError, TypeError, ValueError):
            vehicle_x = vehicle_y = None
        if vehicle_x is not None:
            c = math.cos(geometry["yaw"])
            s = math.sin(geometry["yaw"])
            dx = vehicle_x - geometry["origin_x"]
            dy = vehicle_y - geometry["origin_y"]
            local_x = c * dx + s * dy
            local_y = -s * dx + c * dy
            map_ax.scatter(
                [vehicle_x], [vehicle_y], marker="+", s=80,
                color="tab:cyan", linewidths=1.6, zorder=6,
                label="Costmap capture vehicle pose",
            )
            annotation += (
                f"\nvehicle: ahead {width_m - local_x:.1f} m, "
                f"behind {local_x:.1f} m, "
                f"left {height_m - local_y:.1f} m, right {local_y:.1f} m"
            )
        else:
            # Snapshot predates per-state vehicle pose, or state is missing
            # this tick -- fall back to the box's own geometric centre.
            center = geometry["center"]
            map_ax.scatter(
                [center[0]], [center[1]], marker="+", s=80,
                color="tab:cyan", linewidths=1.6, zorder=6,
                label="Costmap centre",
            )
    map_ax.annotate(
        annotation,
        xy=(first_corner[0], first_corner[1]),
        xytext=(5, 5),
        textcoords="offset points",
        fontsize=7.0,
        color="teal",
        bbox={"boxstyle": "round", "facecolor": "white", "alpha": 0.72},
        zorder=7,
    )
    return boundary


def render_debug_snapshot(
    snapshot: dict[str, Any],
    session_dir_value: str,
    elapsed_int: int,
) -> str:
    """Render one immutable history snapshot in a worker process."""

    session_dir = Path(session_dir_value)
    figure, dashboard = create_debug_figure()
    map_ax = dashboard.map
    status_ax = dashboard.status
    speed_ax = dashboard.speed
    global_error_ax = dashboard.global_error
    tracking_ax = dashboard.tracking
    acceleration_ax = dashboard.acceleration
    jerk_ax = dashboard.jerk
    curvature_ax = dashboard.curvature
    continuity_ax = dashboard.continuity
    timing_ax = dashboard.timing
    allocation_velocity_ax = dashboard.allocation_velocity
    allocation_rate_ax = dashboard.allocation_rate_split
    allocation_beta_ax = dashboard.allocation_beta
    allocation_accel_ax = dashboard.allocation_acceleration
    allocation_consistency_ax = dashboard.allocation_consistency

    _draw_costmap(map_ax, snapshot)

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
    costmap_boundary = _draw_costmap_boundary(map_ax, snapshot)

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
    for x_values, y_values in ((global_x, global_y), (x_history, y_history), (reference_x, reference_y), (selected_x, selected_y)):
        if x_values.size and y_values.size:
            map_x_parts.append(x_values)
            map_y_parts.append(y_values)
    for obstacle in obstacle_artists:
        vertices = np.asarray(obstacle.get_xy(), dtype=float)
        map_x_parts.append(vertices[:, 0])
        map_y_parts.append(vertices[:, 1])
    if costmap_boundary is not None:
        vertices = np.asarray(costmap_boundary.get_xy(), dtype=float)
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
            map_ax.set_xlim(x_min - max(2.0, 0.04 * max(x_max - x_min, 1.0)), x_max + max(2.0, 0.04 * max(x_max - x_min, 1.0)))
            map_ax.set_ylim(y_min - max(2.0, 0.08 * max(y_max - y_min, 1.0)), y_max + max(2.0, 0.08 * max(y_max - y_min, 1.0)))
    map_ax.set_aspect("equal", adjustable="box")
    map_ax.set_title("Scenario map, obstacles, and executed motion")
    map_ax.set_xlabel("x [m]")
    map_ax.set_ylabel("y [m]")
    map_ax.grid(True)
    map_ax.legend(loc="upper left", ncol=2, fontsize=8.2)

    odom_time = _array(snapshot, "odom_time_history")
    cmd_time = _array(snapshot, "cmd_time_history")
    terminal_hold_time = first_true_time(odom_time, snapshot.get("terminal_hold_history", []))
    if odom_time.size:
        speed_ax.plot(odom_time, _array(snapshot, "speed_history"), linewidth=1.8, label="Measured")
        speed_ax.plot(odom_time, _array(snapshot, "target_history"), "--", label="Requested")
        speed_ax.plot(odom_time, _array(snapshot, "applied_target_history"), "-.", label="Feasible-speed upper bound")
    if cmd_time.size:
        speed_ax.step(cmd_time, _array(snapshot, "command_speed_history"), where="post", linestyle=":", label="Body command magnitude")
    if terminal_hold_time is not None:
        speed_ax.axvline(terminal_hold_time, linestyle="--", linewidth=1.0, alpha=0.65, label="Terminal hold")
    speed_ax.set_title("Timestamp-aligned requested, feasible, command, and measured speed")
    speed_ax.set_xlabel("time [s]")
    speed_ax.set_ylabel("speed [m/s]")
    speed_ax.grid(True)
    speed_ax.legend(loc="best", fontsize=7.7)

    if odom_time.size:
        global_error_ax.plot(odom_time, _array(snapshot, "global_lateral_history"), label="Lateral deviation [m]")
        angle_ax = global_error_ax.twinx()
        angle_ax.plot(odom_time, _array(snapshot, "global_motion_history"), "--", label="Motion-direction deviation [deg]")
        angle_ax.plot(odom_time, _array(snapshot, "beta_history"), ":", label="Body offset beta [deg]")
        global_error_ax.set_ylim(-symmetric_limit(snapshot.get("global_lateral_history", []), minimum=1.0), symmetric_limit(snapshot.get("global_lateral_history", []), minimum=1.0))
        angle_limit = symmetric_limit([*snapshot.get("global_motion_history", []), *snapshot.get("beta_history", [])], minimum=10.0)
        angle_ax.set_ylim(-angle_limit, angle_limit)
        lines1, labels1 = global_error_ax.get_legend_handles_labels()
        lines2, labels2 = angle_ax.get_legend_handles_labels()
        global_error_ax.legend(lines1 + lines2, labels1 + labels2, loc="best", fontsize=7.4)
        angle_ax.set_ylabel("angle [deg]")
    add_zero_line(global_error_ax)
    global_error_ax.set_title("Global-path deviation (intentional avoidance included)")
    global_error_ax.set_xlabel("time [s]")
    global_error_ax.set_ylabel("lateral deviation [m]")
    global_error_ax.grid(True)

    if odom_time.size:
        lateral_cm = 100.0 * _array(snapshot, "tracking_lateral_history")
        tracking_ax.plot(odom_time, lateral_cm, label="Lateral error [cm]")
        tracking_angle_ax = tracking_ax.twinx()
        tracking_angle_ax.plot(odom_time, _array(snapshot, "tracking_motion_history"), "--", label="Direction error [deg]")
        tracking_ax.set_ylim(-symmetric_limit(lateral_cm, minimum=1.0), symmetric_limit(lateral_cm, minimum=1.0))
        angle_limit = symmetric_limit(snapshot.get("tracking_motion_history", []), minimum=0.25)
        tracking_angle_ax.set_ylim(-angle_limit, angle_limit)
        lines1, labels1 = tracking_ax.get_legend_handles_labels()
        lines2, labels2 = tracking_angle_ax.get_legend_handles_labels()
        tracking_ax.legend(lines1 + lines2, labels1 + labels2, loc="best", fontsize=7.7)
        tracking_angle_ax.set_ylabel("direction error [deg]")
    add_zero_line(tracking_ax)
    tracking_ax.set_title("Executed-command segment tracking error")
    tracking_ax.set_xlabel("time [s]")
    tracking_ax.set_ylabel("lateral error [cm]")
    tracking_ax.grid(True)

    if cmd_time.size:
        acceleration_ax.step(
            cmd_time, _array(snapshot, "command_acceleration_history"),
            where="post", label="Planned trajectory acceleration"
        )
    add_zero_line(acceleration_ax)
    acceleration_ax.set_title("Timestamp-aligned planned acceleration (direct state)")
    acceleration_ax.set_xlabel("time [s]")
    acceleration_ax.set_ylabel("acceleration [m/s²]")
    acceleration_ax.grid(True)
    acceleration_ax.legend(loc="best", fontsize=8.0)

    if cmd_time.size:
        jerk_ax.step(
            cmd_time, _array(snapshot, "command_jerk_history"),
            where="post", label="Planned longitudinal jerk U[:,0]"
        )
    add_zero_line(jerk_ax)
    jerk_ax.set_title("Timestamp-aligned planned jerk (direct control input)")
    jerk_ax.set_xlabel("time [s]")
    jerk_ax.set_ylabel("jerk [m/s³]")
    jerk_ax.grid(True)
    jerk_ax.legend(loc="best", fontsize=8.0)

    if odom_time.size:
        curvature_ax.plot(odom_time, _array(snapshot, "reference_kappa_history"), label="Global reference")
        curvature_ax.plot(odom_time, _array(snapshot, "executed_kappa_history"), "--", label="Executed trajectory")
        limit = max(0.22, symmetric_limit([*snapshot.get("reference_kappa_history", []), *snapshot.get("executed_kappa_history", [])], minimum=0.05))
        curvature_ax.set_ylim(-limit, limit)
    curvature_ax.axhline(0.20, linestyle=":", linewidth=1.0, label="Hard limit ±0.20")
    curvature_ax.axhline(-0.20, linestyle=":", linewidth=1.0)
    add_zero_line(curvature_ax)
    curvature_ax.set_title("Reference and executed curvature")
    curvature_ax.set_xlabel("time [s]")
    curvature_ax.set_ylabel("curvature [1/m]")
    curvature_ax.grid(True)
    curvature_ax.legend(loc="best", fontsize=8.0)

    # Allocation diagnostics use only direct ExecutedCommand fields and exact
    # identities from the allocator: v=sqrt(vx^2+vy^2), chi_dot=r+beta_dot,
    # and chi_ddot=psi_ddot+beta_ddot.  No finite differences are used.
    if cmd_time.size:
        allocation_velocity_ax.step(
            cmd_time, _array(snapshot, "cmd_vx_history"), where="post", label="vx"
        )
        allocation_velocity_ax.step(
            cmd_time, _array(snapshot, "cmd_vy_history"), where="post", label="vy"
        )
        allocation_velocity_ax.step(
            cmd_time, _array(snapshot, "command_speed_history"), where="post",
            linestyle=":", label="Planned speed"
        )
    add_zero_line(allocation_velocity_ax)
    allocation_velocity_ax.set_title("Allocation: body-frame velocity commands")
    allocation_velocity_ax.set_xlabel("time [s]")
    allocation_velocity_ax.set_ylabel("velocity [m/s]")
    allocation_velocity_ax.grid(True)
    allocation_velocity_ax.legend(loc="best", fontsize=7.5)

    if cmd_time.size:
        allocation_rate_ax.step(
            cmd_time, _array(snapshot, "command_motion_heading_rate_history"),
            where="post", label="motion rate chi_dot"
        )
        allocation_rate_ax.step(
            cmd_time, np.degrees(_array(snapshot, "cmd_yaw_rate_history")),
            where="post", label="body yaw rate psi_dot"
        )
        allocation_rate_ax.step(
            cmd_time, _array(snapshot, "command_beta_rate_history"),
            where="post", label="beta rate"
        )
    add_zero_line(allocation_rate_ax)
    allocation_rate_ax.set_title("Allocation: chi_dot = psi_dot + beta_dot")
    allocation_rate_ax.set_xlabel("time [s]")
    allocation_rate_ax.set_ylabel("angular rate [deg/s]")
    allocation_rate_ax.grid(True)
    allocation_rate_ax.legend(loc="best", fontsize=7.2)

    if cmd_time.size:
        allocation_beta_ax.step(
            cmd_time, _array(snapshot, "command_beta_history"),
            where="post", label="beta"
        )
        allocation_beta_ax.step(
            cmd_time, _array(snapshot, "command_beta_center_history"),
            where="post", linestyle="--", label="mode center"
        )
        allocation_beta_ax.step(
            cmd_time, _array(snapshot, "command_beta_deviation_history"),
            where="post", linestyle=":", label="center deviation"
        )
    add_zero_line(allocation_beta_ax)
    allocation_beta_ax.set_title("Allocation: body-offset state")
    allocation_beta_ax.set_xlabel("time [s]")
    allocation_beta_ax.set_ylabel("angle [deg]")
    allocation_beta_ax.grid(True)
    allocation_beta_ax.legend(loc="best", fontsize=7.2)

    if cmd_time.size:
        allocation_accel_ax.step(
            cmd_time,
            _array(snapshot, "command_motion_heading_acceleration_history"),
            where="post", label="motion heading accel"
        )
        allocation_accel_ax.step(
            cmd_time, _array(snapshot, "command_yaw_acceleration_history"),
            where="post", label="body yaw accel"
        )
        allocation_accel_ax.step(
            cmd_time, _array(snapshot, "command_beta_acceleration_history"),
            where="post", label="beta accel"
        )
    add_zero_line(allocation_accel_ax)
    allocation_accel_ax.set_title("Allocation: angular acceleration split")
    allocation_accel_ax.set_xlabel("time [s]")
    allocation_accel_ax.set_ylabel("angular acceleration [deg/s²]")
    allocation_accel_ax.grid(True)
    allocation_accel_ax.legend(loc="best", fontsize=7.0)

    if cmd_time.size:
        speed_error_mmps = 1000.0 * _array(
            snapshot, "allocation_speed_reconstruction_error_history"
        )
        allocation_consistency_ax.step(
            cmd_time, speed_error_mmps, where="post",
            label="sqrt(vx²+vy²)-v [mm/s]"
        )
        residual_ax = allocation_consistency_ax.twinx()
        residual_ax.step(
            cmd_time,
            _array(snapshot, "allocation_rate_split_residual_history"),
            where="post", linestyle="--",
            label="chi_dot-psi_dot-beta_dot [deg/s]"
        )
        speed_limit = symmetric_limit(speed_error_mmps, minimum=1.0e-6)
        residual_limit = symmetric_limit(
            snapshot.get("allocation_rate_split_residual_history", []),
            minimum=1.0e-6,
        )
        allocation_consistency_ax.set_ylim(-speed_limit, speed_limit)
        residual_ax.set_ylim(-residual_limit, residual_limit)
        lines1, labels1 = allocation_consistency_ax.get_legend_handles_labels()
        lines2, labels2 = residual_ax.get_legend_handles_labels()
        allocation_consistency_ax.legend(
            lines1 + lines2, labels1 + labels2, loc="best", fontsize=7.5, ncol=2
        )
        residual_ax.set_ylabel("angular identity residual [deg/s]")
    add_zero_line(allocation_consistency_ax)
    allocation_consistency_ax.set_title(
        "Allocation consistency residuals (direct algebraic identities)"
    )
    allocation_consistency_ax.set_xlabel("time [s]")
    allocation_consistency_ax.set_ylabel("speed identity residual [mm/s]")
    allocation_consistency_ax.grid(True)

    plan_time = _array(snapshot, "plan_time_history")
    if plan_time.size:
        continuity_ax.step(plan_time, _array(snapshot, "n_target_history"), where="post", label="Selected lateral target [m]")
        jump_ax = continuity_ax.twinx()
        jump_ax.plot(plan_time, _array(snapshot, "curvature_jump_history"), ".-", markersize=3.0, label="Curvature jump [1/m]")
        continuity_ax.set_ylim(-symmetric_limit(snapshot.get("n_target_history", []), minimum=1.0), symmetric_limit(snapshot.get("n_target_history", []), minimum=1.0))
        jump_limit = symmetric_limit(snapshot.get("curvature_jump_history", []), minimum=0.005)
        jump_ax.set_ylim(-jump_limit, jump_limit)
        lines1, labels1 = continuity_ax.get_legend_handles_labels()
        lines2, labels2 = jump_ax.get_legend_handles_labels()
        continuity_ax.legend(lines1 + lines2, labels1 + labels2, loc="best", fontsize=8.0)
        jump_ax.set_ylabel("curvature jump [1/m]")
    add_zero_line(continuity_ax)
    continuity_ax.set_title("Candidate selection and plan-switch continuity")
    continuity_ax.set_xlabel("time [s]")
    continuity_ax.set_ylabel("lateral target [m]")
    continuity_ax.grid(True)

    if plan_time.size:
        compute = _array(snapshot, "plan_compute_history")
        timing_ax.plot(plan_time, compute, ".-", markersize=3.0, label="Total planning pipeline")
        deadline = float(snapshot["planning_deadline_ms"])
        timing_ax.axhline(deadline, linestyle="--", label=f"Deadline {deadline:.0f} ms")
        misses = np.asarray(snapshot.get("plan_deadline_miss_history", []), dtype=bool)
        if misses.size and np.any(misses):
            timing_ax.scatter(plan_time[misses], compute[misses], marker="x", s=50, label="Deadline miss")
        finite = compute[np.isfinite(compute)]
        observed_max = float(np.max(finite)) if finite.size else 0.0
        timing_ax.set_ylim(0.0, max(10.0, 1.10 * deadline, 1.15 * observed_max))
    timing_ax.set_title("Planning computation time")
    timing_ax.set_xlabel("time [s]")
    timing_ax.set_ylabel("time [ms]")
    timing_ax.grid(True)
    timing_ax.legend(loc="best", fontsize=8.0)

    status_ax.axis("off")
    plan = snapshot.get("plan") or {}
    timing = snapshot.get("timing") or {}
    execution = snapshot.get("execution") or {}
    scenario = snapshot.get("scenario_status") or {}
    diagnosis = str(snapshot.get("diagnosis", "UNKNOWN"))
    detail = str(snapshot.get("detail", ""))
    max_odom_delay = max(snapshot.get("odom_callback_delay_history", [0.0]) or [0.0])
    costmap_geometry = _costmap_geometry(snapshot)
    if costmap_geometry is None:
        costmap_status = ["[LOCAL COSTMAP]", "not received"]
    else:
        width_m = float(costmap_geometry["width_m"])
        height_m = float(costmap_geometry["height_m"])
        costmap_status = [
            "[LOCAL COSTMAP]",
            f"size      {width_m:.1f} x {height_m:.1f} m",
        ]
        if _costmap_is_vehicle_centered(snapshot):
            costmap_status.extend(
                [
                    f"body x    {-0.5 * width_m:.1f} .. {0.5 * width_m:.1f} m",
                    f"body y    {-0.5 * height_m:.1f} .. {0.5 * height_m:.1f} m",
                ]
            )
        costmap_status.append(
            f"obstacles {len(snapshot.get('scenario_obstacles', ()) or ())} total"
        )
    status_left = [
        f"DIAGNOSIS  {diagnosis}", detail[:72], "",
        "[SCENARIO / STATE]",
        f"scenario  {scenario.get('scenario', snapshot.get('scenario_name', 'N/A'))}",
        f"phase     {scenario.get('phase_name', 'N/A')}",
        f"scenario  {scenario.get('state', 'N/A')}",
        f"planner   {snapshot.get('planner_state', 'N/A')}",
        f"hold      {snapshot.get('hold_latched', False)}",
        f"mode req/actual {snapshot.get('requested_drive_mode', 'N/A')} / {snapshot.get('vehicle_drive_mode', 'N/A')}",
        f"mode transition {snapshot.get('vehicle_transition_in_progress', False)} complete={snapshot.get('vehicle_transition_complete', False)}", "",
        *costmap_status, "",
        "[TIMESTAMP LOGGING]",
        f"odom samples {len(odom_time)}",
        f"cmd samples  {len(cmd_time)}",
        f"max odom callback delay {max_odom_delay:.3f} s",
        f"render mode worker process",
    ]
    status_right = [
        "[PLAN / CONTINUITY]",
        f"plan id    {execution.get('current_plan_id', 'N/A')}",
        f"candidate  {plan.get('selected_candidate_id', 'N/A')}",
        f"offset     {_format_number(plan.get('selected_n_target'))} m",
        f"select     {plan.get('lateral_selection_basis', 'N/A')}",
        f"center lock {plan.get('reference_center_lock_active', False)}", "",
        "[TIMING]",
        f"current {_format_number(plan.get('total_compute_time_ms'), 2)} ms",
        f"mean    {_format_number(timing.get('mean_ms'), 2)} ms",
        f"p95     {_format_number(timing.get('p95_ms'), 2)} ms",
        f"maximum {_format_number(timing.get('max_ms'), 2)} ms",
        f"misses  {timing.get('deadline_miss_count', 0)} / {timing.get('count', 0)}", "",
        "[CURRENT TRACKING]",
        f"global lat {_format_number(state.get('global_lateral'))} m",
        f"track lat  {_format_number(100.0 * float(state.get('tracking_lateral', math.nan)))} cm",
        f"track src  {state.get('tracking_source', 'N/A')}", "",
        "[CURRENT ALLOCATION]",
        f"vx/vy {_format_number(snapshot.get('latest_cmd_vx'))} / {_format_number(snapshot.get('latest_cmd_vy'))} m/s",
        f"yaw rate {_format_number(math.degrees(float(snapshot.get('latest_cmd_yaw_rate', math.nan))))} deg/s",
        f"beta {_format_number((snapshot.get('command_beta_history') or [math.nan])[-1])} deg",
        f"beta rate {_format_number((snapshot.get('command_beta_rate_history') or [math.nan])[-1])} deg/s",
    ]
    text_style = {
        "transform": status_ax.transAxes, "va": "top", "ha": "left",
        "family": "monospace", "fontsize": 7.2, "linespacing": 1.08,
        "clip_on": True, "bbox": {"boxstyle": "round", "alpha": 0.82},
    }
    status_ax.text(0.01, 0.985, "\n".join(status_left), **text_style)
    status_ax.text(0.51, 0.985, "\n".join(status_right), **text_style)
    status_ax.set_title("Current state and timestamp diagnostics")

    figure.suptitle(f"SIMP Planner - {snapshot.get('scenario_name', 'N/A')} - {diagnosis}")
    snapshot_path = session_dir / f"snapshot_{int(elapsed_int):06d}.png"
    latest_path = session_dir / "latest.png"
    figure.savefig(snapshot_path, dpi=150)
    plt.close(figure)
    shutil.copy2(snapshot_path, latest_path)
    return str(snapshot_path)
