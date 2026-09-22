import math

import numpy as np
import pytest
from builtin_interfaces.msg import Time
from nav_msgs.msg import Odometry
from simp_planner_msgs.msg import ExecutedCommand, Trajectory, TrajectoryPoint

from simp_planner_tools.debug_position_history import PositionComparison
from simp_planner_tools.tracking_result_viewer import save_history, create_figures


def trajectory(plan_id=1, start=10):
    plan = Trajectory(plan_id=plan_id, start_time=Time(sec=start))
    plan.header.frame_id = "odom"
    plan.points = [
        TrajectoryPoint(time_from_start=0.0, x=0.0, y=0.0),
        TrajectoryPoint(time_from_start=0.4, x=0.4, y=0.8),
    ]
    return plan


def command(plan_id=1, sec=10, ns=0, state="ACTIVE_PLAN"):
    message = ExecutedCommand(plan_id=plan_id, execution_state=state)
    message.header.stamp = Time(sec=sec, nanosec=ns)
    return message


def odom(sec=10, ns=200_000_000, frame="odom"):
    message = Odometry()
    message.header.stamp = Time(sec=sec, nanosec=ns)
    message.header.frame_id = frame
    return message


def test_uses_measurement_time_and_matching_plan_not_latest_arrival():
    history = PositionComparison()
    history.on_trajectory(trajectory())
    history.on_trajectory(trajectory(2, 11))
    history.on_command(command(2, 11))  # Future command received first.
    history.on_command(command())
    assert history.reference_xy(odom()) == pytest.approx((0.2, 0.4))
    assert history.reference_xy(odom(11, 100_000_000)) == pytest.approx((0.1, 0.2))


@pytest.mark.parametrize("case", ["missing_plan", "frame", "stale", "stopped", "expired", "future"])
def test_unavailable_target_is_a_gap_not_zero_error(case):
    history = PositionComparison()
    history.on_trajectory(trajectory())
    message = command()
    measurement = odom()
    if case == "missing_plan":
        message.plan_id = 99
    elif case == "frame":
        measurement.header.frame_id = "map"
    elif case == "stale":
        measurement.header.stamp.nanosec = 300_000_000
    elif case == "stopped":
        message.execution_state = "TERMINAL_HOLD"
    elif case == "expired":
        message.header.stamp.nanosec = 450_000_000
        measurement.header.stamp.nanosec = 500_000_000
    else:
        message.header.stamp.sec = 9
        measurement.header.stamp = Time(sec=9, nanosec=100_000_000)
    history.on_command(message)
    assert all(math.isnan(value) for value in history.reference_xy(measurement))


def test_clock_reset_discards_old_plans_and_commands():
    history = PositionComparison()
    history.on_trajectory(trajectory())
    history.on_command(command())
    history.reference_xy(odom())
    assert all(math.isnan(value) for value in history.reference_xy(odom(1)))
    assert not history.plans and not history.commands


@pytest.mark.parametrize("tracker_enabled", [True, False])
def test_position_graph_and_csv_use_same_errors_for_on_and_off(tmp_path, tracker_enabled):
    snapshot = {
        "scenario_name": "position-test",
        "use_tracking_controller": tracker_enabled,
        "position_history": [(0, 1, 2, 1.3, 2.4, 0, 0, 0, 0),
                             (1, math.nan, math.nan, 2, 3, math.nan, math.nan, 0, 0)],
    }
    save_history(snapshot["position_history"], tmp_path / "position_comparison.csv")
    values = np.loadtxt(tmp_path / "position_comparison.csv", delimiter=",", skiprows=1)
    np.testing.assert_allclose(values[0, 5:8], [0.3, 0.4, 0.5])
    assert np.isnan(values[1, 5:8]).all()
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    data = np.genfromtxt(tmp_path / "position_comparison.csv", delimiter=",", names=True)
    figures = create_figures(data, "Tracker " + str(tracker_enabled))
    assert [len(figure.axes) for figure in figures] == [4, 5]
    # The final sample is present, even though its target is unavailable.
    assert figures[0].axes[0].lines[1].get_xdata()[-1] == 1
    assert figures[0].axes[0].lines[1].get_ydata()[-1] == 2
    for figure in figures:
        plt.close(figure)
    assert not list(tmp_path.glob("*.png"))


def test_reference_interpolates_yaw_and_yaw_rate_across_angle_boundary():
    history = PositionComparison()
    plan = trajectory()
    plan.points[0].yaw = math.radians(179)
    plan.points[1].yaw = math.radians(-179)
    plan.points[0].yaw_rate = 0.1
    plan.points[1].yaw_rate = 0.3
    history.on_trajectory(plan)
    history.on_command(command())
    x, y, yaw, yaw_rate = history.reference_state(odom())
    assert (x, y, abs(yaw), yaw_rate) == pytest.approx((0.2, 0.4, math.pi, 0.2))
    assert all(math.isnan(value) for value in history.reference_state(odom(frame="map")))


def test_angular_errors_use_measured_rate_and_wrapped_yaw(tmp_path):
    snapshot = {"position_history": [
        (0, 1, 2, 1.3, 2.4, math.radians(179), 0.2, math.radians(-179), 0.3),
        (1, math.nan, math.nan, 2, 3, math.nan, math.nan, 0.0, 0.1),
    ]}
    save_history(snapshot["position_history"], tmp_path / "position_comparison.csv")
    values = np.genfromtxt(tmp_path / "position_comparison.csv", delimiter=",", names=True)
    assert values["yaw_error_rad"][0] == pytest.approx(math.radians(2))
    assert values["yaw_rate_error_rad_s"][0] == pytest.approx(0.1)
    assert math.isnan(values["yaw_error_rad"][1])
    assert math.isnan(values["yaw_rate_error_rad_s"][1])
