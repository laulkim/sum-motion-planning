import csv
import math

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pytest

from simp_planner_tools.spot_turn_analysis import (
    analyze_turns, create_turn_figures, read_series, write_summary,
)
from simp_planner_tools.tracking_shutdown_plot import COLUMNS, show_recording


def recording():
    data = np.zeros((7, len(COLUMNS)))
    data[:, 0] = np.arange(7) * 0.1
    data[:, 19] = 1e9 + data[:, 0] * 1e9
    data[:, 20] = data[:, 19] - 0.1e9  # delayed snapshot, not the raw odom clock
    data[:, 21] = data[:, 19]
    data[:, 22] = 1
    data[:, 1], data[:, 4] = 999.0, 999.1  # must not be used for measured movement
    data[1:4, 15] = 0.3
    data[1:4, 16] = [0.1, 0.2, 0.3]
    data[1:4, 9] = np.radians([1, -1, 2])
    odom = np.array([
        [1e9, 100, 0, 0, 0, 0, 0],  # old acquisition sample must be excluded
        [1.1e9, 0, 0, 0, 0, 0, 0],
        [1.2e9, 0.1, 0, 0, 1, 0, 0],
        [1.2e9, 0.1, 0, 0, 1, 0, 0],  # duplicate acquisition timestamp
        [1.3e9, 0, 0, 0, -1, 0, 0],
        [1.4e9, 0, 0, 0, 0, 0, 0],
        [1.5e9, 0, 0, 0, 0, 0, 0],
    ])
    modes = np.array([[1e9, 4, 4, 1]])
    return data, odom, modes


def test_metrics_use_acquisition_pose_and_zero_order_held_command(tmp_path):
    turns = analyze_turns(*recording())
    assert len(turns) == 1
    metric = turns[0]["metrics"]
    assert metric["duration_s"] == pytest.approx(0.3)
    assert metric["command_distance_m"] == pytest.approx(0.06)  # includes last held command
    assert metric["odom_samples"] == 4
    assert metric["odom_net_displacement_m"] == 0
    assert metric["odom_max_displacement_m"] == pytest.approx(0.1)
    assert metric["odom_path_length_m"] == pytest.approx(0.2)
    assert metric["yaw_rmse_deg"] == pytest.approx(math.sqrt(2))
    assert metric["initial_position_error_m"] == pytest.approx(0.1)
    assert metric["partial"] == 0 and metric["mode_verified"] == 1
    write_summary(tmp_path / "summary.csv", turns)
    with (tmp_path / "summary.csv").open() as file:
        row = next(csv.DictReader(file))
    assert float(row["command_distance_m"]) == pytest.approx(0.06)


def test_mode_feedback_is_not_used_before_its_timestamp():
    data, odom, _ = recording()
    modes = np.array([[1e9, 0, 4, 0], [1.2e9, 4, 4, 1]])
    turns = analyze_turns(data, odom, modes)
    assert len(turns) == 1
    assert turns[0]["metrics"]["start_s"] == pytest.approx(0.2)
    assert turns[0]["metrics"]["control_samples"] == 2
    assert not analyze_turns(data, odom, np.array([[1e9, 4, 0, 0]]))


def test_hold_and_translating_curve_are_excluded():
    data, odom, modes = recording()
    data[2, 13] = 1e-12  # exact zero condition, not a low-speed threshold
    turns = analyze_turns(data, odom, modes)
    assert [turn["metrics"]["control_samples"] for turn in turns] == [1, 1]
    data[1:4, 15] = 0
    assert not analyze_turns(data, odom, modes)


def test_partial_and_legacy_recordings_do_not_claim_measured_motion(tmp_path):
    data, odom, modes = recording()
    partial = analyze_turns(data[:3], odom, modes)[0]["metrics"]
    assert partial["partial"] == 1
    assert partial["command_distance_m"] == pytest.approx(0.01)
    legacy_turns = analyze_turns(data[:, :19])
    legacy = legacy_turns[0]["metrics"]
    assert legacy["mode_verified"] == 0
    assert math.isnan(legacy["odom_max_displacement_m"])
    figures = create_turn_figures(legacy_turns, predicted=False)
    try:
        assert figures[1].axes[6].lines[0].get_label() == "reference minus measured"
    finally:
        for figure in figures:
            plt.close(figure)
    assert read_series(tmp_path / "missing.csv", 7).shape == (0, 7)
    (tmp_path / "empty.csv").write_text("stamp_ns,x,y,yaw,vx,vy,omega\n")
    assert read_series(tmp_path / "empty.csv", 7).shape == (0, 7)


def test_turn_panels_display_raw_center_path():
    figures = create_turn_figures(analyze_turns(*recording()))
    try:
        assert len(figures) == 2 and len(figures[1].axes) == 8
        assert figures[1].axes[0].lines[0].get_drawstyle() == "steps-post"
        assert figures[1].axes[0].lines[0].get_xdata()[-1] == pytest.approx(0.3)
        np.testing.assert_allclose(figures[1].axes[4].lines[0].get_xdata(), [0, 10, 0, 0])
        np.testing.assert_allclose(figures[1].axes[5].lines[1].get_ydata(), [0, 10, 20, 20])
    finally:
        for figure in figures:
            plt.close(figure)


def test_viewer_saves_existing_plots_summary_and_turn_panel(tmp_path, monkeypatch):
    monkeypatch.delenv("DISPLAY", raising=False)
    data, odom, modes = recording()
    for name, array, columns in (
        ("tracking.csv", data, COLUMNS),
        ("odometry.csv", odom, ("stamp_ns", "x", "y", "yaw", "vx", "vy", "omega")),
        ("drive_mode.csv", modes, ("stamp_ns", "current_mode", "requested_mode", "status")),
    ):
        with (tmp_path / name).open("w", newline="") as file:
            writer = csv.writer(file)
            writer.writerow(columns)
            writer.writerows(array)
    show_recording(tmp_path / "tracking.csv")
    for name in ("tracking_errors.png", "tracking_states_inputs.png", "tracking_world_errors.png",
                 "spot_turn_summary.png", "spot_turn_01.png", "spot_turn_summary.csv"):
        assert (tmp_path / name).stat().st_size > 0
    assert not plt.get_fignums()
