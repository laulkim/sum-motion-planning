import csv
import math

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from simp_planner_msgs.msg import TrackingControl, TrackingTrajectoryPoint

from simp_planner_tools.tracking_shutdown_plot import COLUMNS, create_figures, sample_row, show_recording


def test_shutdown_figures_use_controller_snapshot(tmp_path):
    message = TrackingControl(valid=True)
    message.header.stamp.sec = 1
    message.header.stamp.nanosec = 14000000
    message.odom_stamp.sec = 1
    message.reference_stamp.sec = 1
    message.reference_stamp.nanosec = 20000000
    message.predicted_x = 0.04
    message.reference = TrackingTrajectoryPoint(x=0.04, vx=2.0)
    message.measured_velocity.linear.x = 2.0
    message.command.linear.x = 2.0
    row = sample_row(message, 1000000000)
    assert len(row) == len(COLUMNS)
    assert math.isclose(row[0], 0.014)
    assert row[1:10] == (0.04, 0.0, 0.0, 0.04, 0.0, 0.0, 0.0, 0.0, 0.0)
    assert row[10] == row[13] == row[16] == 2.0
    assert row[19:] == (1014000000, 1000000000, 1020000000, 1)
    # Keep the controller's error verbatim; never select/recompute it in the plotter.
    message.ex, message.ey, message.etheta = 0.1, -0.2, 0.3
    assert sample_row(message, 1000000000)[7:10] == (0.1, -0.2, 0.3)
    message.valid = False
    message.command.linear.x = 0.0
    invalid = sample_row(message, 1000000000)
    assert all(math.isnan(value) for value in invalid[1:16])
    assert invalid[16:19] == (0.0, 0.0, 0.0)
    assert invalid[-1] == 0
    figures = create_figures(np.asarray([row, invalid]))
    assert [len(figure.axes) for figure in figures] == [3, 6, 3]
    np.testing.assert_allclose(figures[0].axes[0].lines[0].get_ydata(), [0.0, np.nan])
    np.testing.assert_allclose(figures[1].axes[1].lines[0].get_ydata(), [2.0, 0.0])
    assert figures[1].axes[0].lines[0].get_label() == "predicted"
    for index, figure in enumerate(figures):
        figure.savefig(tmp_path / f"figure{index}.png")
        plt.close(figure)
    # Old CSVs remain identifiable as the old, unpredicted calculation.
    legacy = create_figures(np.asarray([row[:19]]), predicted=False)
    assert legacy[1].axes[0].lines[0].get_label() == "actual"
    for figure in legacy:
        plt.close(figure)


def test_single_sample_recording_saves_three_figures_without_display(tmp_path, monkeypatch):
    monkeypatch.delenv("DISPLAY", raising=False)
    message = TrackingControl(valid=True)
    path = tmp_path / "tracking.csv"
    with path.open("w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(COLUMNS)
        writer.writerow(sample_row(message, 0))
    show_recording(path)
    for name in ("tracking_errors.png", "tracking_states_inputs.png", "tracking_world_errors.png"):
        assert (tmp_path / name).stat().st_size > 0
    assert not plt.get_fignums()


def test_world_errors_are_direct_differences_even_when_stopped_or_rotating():
    message = TrackingControl(valid=True)
    message.predicted_x, message.predicted_y = 1.0, 2.0
    message.reference = TrackingTrajectoryPoint(x=2.0, y=4.0)
    message.ex, message.ey = -3.0, 5.0  # existing controller plot stays verbatim
    # vx, vy, reference yaw, predicted yaw, expected yaw error in degrees.
    cases = [
        (1, 0, 0, 0, 0),
        (0, 1, math.pi / 2, 0, 90),
        (-1, 0, math.pi, math.pi, 0),
        (0, 0, 0, 0, 0),  # stationary: no artificial gap
        (0, 0, 1, 0, math.degrees(1)),  # spot turn: no artificial gap
        (1, 0, -math.pi + 0.1, math.pi - 0.1, math.degrees(0.2)),
    ]
    rows = []
    for vx, vy, yaw_ref, yaw_pred, _ in cases:
        message.reference.vx, message.reference.vy = float(vx), float(vy)
        message.reference.body_yaw, message.predicted_yaw = float(yaw_ref), float(yaw_pred)
        message.reference.yaw_rate = 0.3 if yaw_ref == 1 else 0.0
        rows.append(sample_row(message, 0))
    message.valid = False
    rows.append(sample_row(message, 0))
    figures = create_figures(np.asarray(rows))
    try:
        np.testing.assert_allclose(figures[0].axes[0].lines[0].get_ydata(),
                                   [-3] * len(cases) + [np.nan])
        np.testing.assert_allclose(figures[2].axes[0].lines[0].get_ydata(),
                                   [1] * len(cases) + [np.nan])
        np.testing.assert_allclose(figures[2].axes[1].lines[0].get_ydata(),
                                   [2] * len(cases) + [np.nan])
        np.testing.assert_allclose(figures[2].axes[2].lines[0].get_ydata(),
                                   [case[4] for case in cases] + [np.nan], atol=1e-12)
    finally:
        for figure in figures:
            plt.close(figure)
