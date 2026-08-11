from __future__ import annotations

from pathlib import Path

from simp_planner_tools.debug_plot_renderer import render_debug_snapshot


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
