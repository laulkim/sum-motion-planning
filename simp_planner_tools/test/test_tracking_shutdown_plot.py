import csv
import math

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from simp_planner_msgs.msg import TrackingControl, TrackingTrajectoryPoint

from simp_planner_tools.tracking_shutdown_plot import (
    ALLOCATION_COLUMNS, ALLOCATION_FILE, COLUMNS, VX_PRIORITY, VY_PRIORITY, allocation_event,
    allocation_profiles, create_figures, motion_heading, motion_heading_rate, sample_row,
    show_recording)


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
    assert [len(figure.axes) for figure in figures] == [3, 9]
    np.testing.assert_allclose(figures[0].axes[0].lines[0].get_ydata(), [0.0, np.nan])
    np.testing.assert_allclose(figures[1].axes[1].lines[0].get_ydata(), [2.0, 0.0])
    assert figures[1].axes[0].lines[0].get_label() == "predicted"
    chi_axis = figures[1].axes[6]
    assert chi_axis.get_ylabel() == "chi [deg]"
    np.testing.assert_allclose(chi_axis.lines[0].get_ydata(), [0.0, np.nan])
    assert figures[1].axes[7].get_ylabel() == "chi_rate [deg/s]"
    for index, figure in enumerate(figures):
        figure.savefig(tmp_path / f"figure{index}.png")
        plt.close(figure)
    # Old CSVs remain identifiable as the old, unpredicted calculation.
    legacy = create_figures(np.asarray([row[:19]]), predicted=False)
    assert legacy[1].axes[0].lines[0].get_label() == "actual"
    # No allocation record: the state graph says so instead of drawing a guess.
    assert not legacy[1].axes[8].lines and legacy[1].axes[8].texts
    for figure in legacy:
        plt.close(figure)


def test_single_sample_recording_saves_both_figures_without_display(tmp_path, monkeypatch):
    monkeypatch.delenv("DISPLAY", raising=False)
    message = TrackingControl(valid=True)
    path = tmp_path / "tracking.csv"
    with path.open("w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(COLUMNS)
        writer.writerow(sample_row(message, 0))
    with (tmp_path / ALLOCATION_FILE).open("w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(ALLOCATION_COLUMNS)
        writer.writerow((0, VY_PRIORITY))
    show_recording(path)
    for name in ("tracking_errors.png", "tracking_states_inputs.png"):
        assert (tmp_path / name).stat().st_size > 0
    assert not plt.get_fignums()


def test_motion_heading_and_rate():
    # Crab-like motion: body yaw 0.1 rad, velocity along body +y -> chi = 0.1 + pi/2.
    np.testing.assert_allclose(motion_heading(np.array([0.1]), np.array([0.0]), np.array([1.5])),
                               [0.1 + math.pi / 2])
    # Below the heading speed the motion direction is undefined.
    assert math.isnan(motion_heading(np.array([0.0]), np.array([0.01]), np.array([0.0]))[0])

    # yaw rate 0.1 rad/s and slip angle turning at 0.2 rad/s -> chi_dot = 0.3 rad/s,
    # including across the +-pi wrap of beta.
    time = np.arange(6) * 0.01
    beta = math.pi - 0.002 + 0.2 * time
    rate = motion_heading_rate(time, np.full(6, 0.1), 2.0 * np.cos(beta), 2.0 * np.sin(beta))
    assert math.isnan(rate[0])
    np.testing.assert_allclose(rate[1:], 0.3)

    # A repeated sample (held odometry) keeps the previous rate, not a zero-time spike;
    # a stationary sample breaks the series.
    time = np.array([0.00, 0.01, 0.01, 0.02, 0.03, 0.04])
    beta = 0.2 * time
    speed = np.array([1.0, 1.0, 1.0, 0.0, 1.0, 1.0])
    rate = motion_heading_rate(time, np.zeros(6), speed * np.cos(beta), speed * np.sin(beta))
    np.testing.assert_allclose(rate[1:3], 0.2)
    assert math.isnan(rate[3]) and math.isnan(rate[4])
    np.testing.assert_allclose(rate[5], 0.2)


def test_allocation_state_graph():
    assert allocation_event('{"allocation_profile":"MINIMUM_VY","trajectory_start_ns":5}') == (
        5, VX_PRIORITY)
    # Statuses that committed no new trajectory content carry no event.
    assert allocation_event('{"allocation_profile":"NONE","state":"STALE_PLAN_DISCARDED"}') is None
    assert allocation_event("not json") is None

    # The last-published content replaces earlier content from its own start on,
    # even when it starts before content published earlier (shorter handover lead).
    events = [(10, VY_PRIORITY), (20, "NONE"), (30, VX_PRIORITY), (25, VY_PRIORITY)]
    assert allocation_profiles([5, 10, 22, 26, 35], events) == [
        None, VY_PRIORITY, "NONE", VY_PRIORITY, VY_PRIORITY]

    data = np.zeros((5, len(COLUMNS)))
    data[:, 0] = np.arange(5) * 0.01
    data[:, 19] = [5, 15, 25, 35, 45]
    events = [(10, VY_PRIORITY), (20, VX_PRIORITY), (30, "NONE"), (40, VY_PRIORITY)]
    figures = create_figures(data, allocation=events)
    # One state graph: none=0, Vy-priority=1, Vx-priority=2, unknown before the first event.
    state = figures[1].axes[8]
    assert state.get_ylabel() == "allocation"
    assert len(state.lines) == 1
    np.testing.assert_allclose(state.lines[0].get_ydata(), [np.nan, 1, 2, 0, 1])
    for figure in figures:
        plt.close(figure)
