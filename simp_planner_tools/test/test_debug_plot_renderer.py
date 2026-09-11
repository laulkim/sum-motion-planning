from __future__ import annotations

import math
from pathlib import Path

import matplotlib
import numpy as np

matplotlib.use("Agg")

import matplotlib.pyplot as plt

from simp_planner_tools.debug_plot_renderer import (
    _draw_costmap,
    _draw_costmap_boundary,
    _draw_scenario_obstacles,
    render_debug_snapshot,
)


def test_rotated_costmap_affine_places_grid_corners_in_odom() -> None:
    figure, axis = plt.subplots()
    snapshot = {
        "costmap_data": np.asarray([[100, 0], [0, 0]], dtype=np.int8),
        "costmap_extent": (10.0, 14.0, 20.0, 24.0),
        "costmap_origin": {"x": 10.0, "y": 20.0, "yaw": 0.5 * math.pi},
        "costmap_resolution": 2.0,
        "costmap_width": 2,
        "costmap_height": 2,
    }

    image = _draw_costmap(axis, snapshot)
    assert image is not None
    local_corners = np.asarray(
        [[0.0, 0.0], [4.0, 0.0], [4.0, 4.0], [0.0, 4.0]]
    )
    display_corners = image.get_transform().transform(local_corners)
    odom_corners = axis.transData.inverted().transform(display_corners)
    np.testing.assert_allclose(
        odom_corners,
        np.asarray([[10.0, 20.0], [10.0, 24.0], [6.0, 24.0], [6.0, 20.0]]),
        atol=1.0e-12,
    )
    plt.close(figure)


def test_costmap_renderer_accepts_legacy_extent_only_snapshot() -> None:
    figure, axis = plt.subplots()
    snapshot = {
        "costmap_data": np.asarray([[0, 100], [0, 0]], dtype=np.int8),
        "costmap_extent": (-3.0, 1.0, 7.0, 11.0),
    }

    image = _draw_costmap(axis, snapshot)
    assert image is not None
    assert tuple(image.get_extent()) == snapshot["costmap_extent"]
    plt.close(figure)


def test_all_scenario_obstacles_and_vehicle_relative_costmap_size_are_drawn() -> None:
    figure, axis = plt.subplots()
    snapshot = {
        "scenario_obstacles": (
            np.asarray([[1.0, 2.0], [3.0, 2.0], [3.0, 4.0], [1.0, 4.0]]),
            np.asarray([[-8.0, 5.0], [-6.0, 5.0], [-6.0, 7.0], [-8.0, 7.0]]),
        ),
        "costmap_origin": {"x": 10.0, "y": 20.0, "yaw": 0.5 * math.pi},
        "costmap_resolution": 1.0,
        "costmap_width": 60,
        "costmap_height": 60,
        # Sits at this box's geometric centre, so the (symmetric, raw
        # as-received) costmap reports equal ahead/behind/left/right numbers
        # here -- a planner crop window is generally NOT symmetric like this.
        "current_state": {"x": -20.0, "y": 50.0, "body_yaw": 0.5 * math.pi},
    }

    obstacles = _draw_scenario_obstacles(axis, snapshot)
    boundary = _draw_costmap_boundary(axis, snapshot)

    assert len(obstacles) == 2
    assert obstacles[0].get_label() == "Scenario obstacles (2 total)"
    assert boundary is not None
    assert "60.0×60.0 m" in boundary.get_label()
    assert "as received" in boundary.get_label()
    np.testing.assert_allclose(
        np.asarray(boundary.get_xy())[:4],
        np.asarray([[10.0, 20.0], [10.0, 80.0], [-50.0, 80.0], [-50.0, 20.0]]),
        atol=1.0e-12,
    )
    np.testing.assert_allclose(
        axis.collections[-1].get_offsets(), np.asarray([[-20.0, 50.0]])
    )
    assert any(
        "ahead 30.0 m, behind 30.0 m, left 30.0 m, right 30.0 m" in annotation.get_text()
        for annotation in axis.texts
    )
    plt.close(figure)


def test_costmap_boundary_prefers_planner_crop_window_over_raw_grid() -> None:
    # The planner crops the raw received grid down to a reference-path-slice
    # window before running its distance transform (see
    # LOCAL_COSTMAP_REDESIGN_KR.md) -- the debug plot should show that
    # smaller/asymmetric window, not the full grid the sensor/scenario side
    # published.
    figure, axis = plt.subplots()
    snapshot = {
        "costmap_origin": {"x": -100.0, "y": -100.0, "yaw": 0.0},
        "costmap_resolution": 1.0,
        "costmap_width": 60,
        "costmap_height": 60,
        "costmap_crop_origin": {"x": 0.0, "y": 0.0, "yaw": 0.0},
        "costmap_crop_resolution": 1.0,
        "costmap_crop_width": 10,
        "costmap_crop_height": 6,
        "current_state": {"x": 2.0, "y": 3.0, "body_yaw": 0.0},
    }

    boundary = _draw_costmap_boundary(axis, snapshot)

    assert boundary is not None
    assert "10.0×6.0 m" in boundary.get_label()
    assert "planner window" in boundary.get_label()
    np.testing.assert_allclose(
        np.asarray(boundary.get_xy())[:4],
        np.asarray([[0.0, 0.0], [10.0, 0.0], [10.0, 6.0], [0.0, 6.0]]),
        atol=1.0e-12,
    )
    np.testing.assert_allclose(
        axis.collections[-1].get_offsets(), np.asarray([[2.0, 3.0]])
    )
    assert any(
        "ahead 8.0 m, behind 2.0 m, left 3.0 m, right 3.0 m" in annotation.get_text()
        for annotation in axis.texts
    )
    plt.close(figure)


def test_track_map_boundary_does_not_claim_vehicle_centered_semantics() -> None:
    figure, axis = plt.subplots()
    snapshot = {
        "scenario_name": "track_map",
        "costmap_vehicle_centered": False,
        "costmap_origin": {"x": -10.0, "y": -5.0, "yaw": 0.0},
        "costmap_resolution": 1.0,
        "costmap_width": 20,
        "costmap_height": 10,
    }

    boundary = _draw_costmap_boundary(axis, snapshot)

    assert boundary is not None
    assert boundary.get_label() == "Costmap 20.0×10.0 m"
    assert not axis.collections
    assert all("body" not in annotation.get_text() for annotation in axis.texts)
    plt.close(figure)


def test_renderer_creates_timestamp_aligned_dashboard(tmp_path: Path) -> None:
    times = [0.0, 0.1, 0.2, 0.3]
    snapshot = {
        "scenario_name": "test",
        "vehicle_length": 3.0,
        "vehicle_width": 2.0,
        "planning_deadline_ms": 100.0,
        "latest_cmd_vx": 0.0, "latest_cmd_vy": 0.0,
        "latest_cmd_yaw_rate": 0.0,
        "diagnosis": "OK",
        "detail": "timestamp test",
        "global_x": [0.0, 1.0], "global_y": [0.0, 0.0],
        "reference_x": [0.0, 1.0], "reference_y": [0.0, 0.0],
        "selected_x": [0.0, 1.0], "selected_y": [0.0, 0.0],
        "costmap_data": None, "costmap_extent": None,
        "current_state": {
            "x": 0.3, "y": 0.0, "body_yaw": 0.0,
            "global_lateral": 0.0, "tracking_lateral": 0.0,
            "tracking_source": "EXECUTED_COMMAND_SEGMENT",
        },
        "current_projection": {"x": 0.3, "y": 0.0},
        "selected_projection": {},
        "scenario_status": {"scenario": "test", "phase_name": "p", "state": "RUNNING"},
        "planner_state": "TRACKING", "hold_latched": False,
        "plan": {"selected_candidate_id": 0, "selected_n_target": 0.0},
        "timing": {}, "execution": {"current_plan_id": 1},
        "odom_time_history": times,
        "odom_receive_time_history": times,
        "odom_callback_delay_history": [0.0] * 4,
        "x_history": times, "y_history": [0.0] * 4,
        "speed_history": [0.0, 1.0, 0.8, 0.0],
        "target_history": [1.0] * 4, "applied_target_history": [1.0] * 4,
        "cmd_time_history": times,
        "cmd_receive_time_history": times,
        "cmd_callback_delay_history": [0.0] * 4,
        "cmd_vx_history": [0.0, 1.0, 0.8, 0.0],
        "cmd_vy_history": [0.0] * 4, "cmd_yaw_rate_history": [0.0] * 4,
        "command_speed_history": [0.0, 1.0, 0.8, 0.0],
        "command_acceleration_history": [0.0, 0.5, -0.2, 0.0],
        "command_jerk_history": [1.0, 0.0, -1.0, 0.0],
        "command_motion_heading_rate_history": [0.0, 5.0, -3.0, 0.0],
        "command_motion_heading_acceleration_history": [0.0, 2.0, -1.0, 0.0],
        "command_beta_history": [0.0, 2.0, 1.0, 0.0],
        "command_beta_center_history": [0.0] * 4,
        "command_beta_deviation_history": [0.0, 2.0, 1.0, 0.0],
        "command_beta_rate_history": [0.0, 1.0, -0.5, 0.0],
        "command_beta_acceleration_history": [0.0, 0.5, -0.2, 0.0],
        "command_yaw_acceleration_history": [0.0, 1.5, -0.8, 0.0],
        "allocation_rate_split_residual_history": [0.0] * 4,
        "allocation_speed_reconstruction_error_history": [0.0] * 4,
        "terminal_hold_history": [False] * 4,
        "global_lateral_history": [0.0] * 4,
        "global_motion_history": [0.0] * 4,
        "tracking_lateral_history": [0.0] * 4,
        "tracking_motion_history": [0.0] * 4,
        "beta_history": [0.0] * 4,
        "reference_kappa_history": [0.0] * 4,
        "executed_kappa_history": [0.0] * 4,
        "plan_time_history": [0.0, 0.3],
        "n_target_history": [0.0, 0.0],
        "curvature_jump_history": [0.0, 0.0],
        "plan_compute_history": [10.0, 12.0],
        "plan_deadline_miss_history": [0, 0],
    }
    result = Path(render_debug_snapshot(snapshot, str(tmp_path), 1))
    assert result.exists()
    assert (tmp_path / "latest.png").exists()
