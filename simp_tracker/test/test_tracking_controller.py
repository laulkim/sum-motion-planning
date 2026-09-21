import math

import pytest
import rclpy
from builtin_interfaces.msg import Time
from nav_msgs.msg import Odometry
from simp_planner_msgs.msg import DriveModeState, ExecutedCommand, Trajectory, TrajectoryPoint

from simp_tracker.tracking_controller_node import (
    TrackingController, ZERO, limit_command, reference_at, tracking_command,
    validate_trajectory, wrap_angle,
)


def point(t, x=0.0, y=0.0, yaw=0.0, vx=1.0, vy=0.0, yaw_rate=0.0, mode=0):
    return TrajectoryPoint(time_from_start=float(t), x=float(x), y=float(y),
                           yaw=float(yaw), vx=float(vx), vy=float(vy),
                           yaw_rate=float(yaw_rate), mode=mode)


def plan(plan_id=1, start_sec=10):
    message = Trajectory(plan_id=plan_id, start_time=Time(sec=start_sec))
    message.header.frame_id = "odom"
    message.points = [point(0), point(0.1, x=0.1), point(0.35, x=0.35)]
    return message


def test_interpolation_time_origin_and_yaw_boundary():
    message = plan()
    message.points[1].yaw = math.radians(179)
    message.points[2].yaw = math.radians(-179)
    validate_trajectory(message)
    ref = reference_at(message, 0.225)
    assert ref.x == pytest.approx(0.225)
    assert abs(ref.yaw) == pytest.approx(math.pi)
    assert reference_at(message, -0.01) is None
    assert reference_at(message, 0.3501) is None
    assert reference_at(message, 0.35).x == pytest.approx(0.35)


@pytest.mark.parametrize("bad", ["empty", "nan", "duplicate_time", "mode_change", "no_frame"])
def test_reject_invalid_horizon(bad):
    message = plan()
    if bad == "empty":
        message.points = []
    elif bad == "nan":
        message.points[1].vx = math.nan
    elif bad == "duplicate_time":
        message.points[1].time_from_start = 0.0
    elif bad == "mode_change":
        message.points[-1].mode = 2
    else:
        message.header.frame_id = ""
    with pytest.raises(ValueError):
        validate_trajectory(message)


@pytest.mark.parametrize("vx,vy,mode", [(1.0, 0.0, 0), (-1.0, 0.0, 1), (0.0, 1.0, 2), (0.0, -1.0, 3)])
def test_zero_error_returns_body_feedforward(vx, vy, mode):
    ref = point(0, x=1, y=2, yaw=0.7, vx=vx, vy=vy, yaw_rate=0.3, mode=mode)
    command, error = tracking_command(ref, (1.0, 2.0, 0.7), (3, 4, 2))
    assert command == pytest.approx((vx, vy, 0.3))
    assert error == pytest.approx(ZERO)


def test_body_frame_transform_and_lyapunov_derivative():
    ref = point(0, x=1.3, y=-0.4, yaw=0.8, vx=0.9, vy=-0.3, yaw_rate=0.2)
    command, error = tracking_command(ref, (0.2, 0.7, -0.6), (3, 4, 2))
    ex, ey, et = error
    vx, vy, omega = command
    # Equation (13) should yield equation (18), including the yaw coupling.
    dex = -vx + ey * omega + ref.vx * math.cos(et) - ref.vy * math.sin(et)
    dey = -vy - ex * omega + ref.vx * math.sin(et) + ref.vy * math.cos(et)
    det = ref.yaw_rate - omega
    assert ex * dex + ey * dey + et * det == pytest.approx(-3 * ex**2 - 4 * ey**2 - 2 * et**2)
    command, _ = tracking_command(point(0, yaw=-math.pi + 0.01, vx=0),
                                   (0, 0, math.pi - 0.01), (3, 4, 2))
    assert command[2] == pytest.approx(0.04)
    assert limit_command((6, 8, -2), 5, 1) == pytest.approx((3, 4, -1))


@pytest.mark.parametrize("hz", [20, 50, 100])
def test_paper_figure_eight_converges_from_initial_pose_error(hz):
    # Paper eq. (28), section 5 initial condition and gains.
    x, y, yaw = -0.2, -0.5, 0.5
    dt = 1.0 / hz
    errors = []
    for step in range(10 * hz):
        t = step * dt
        ref = point(t, x=-0.24 + math.sin(t / 5), y=-0.24 + 0.5 * math.sin(2 * t / 5),
                    vx=0.2 * math.cos(t / 5), vy=0.2 * math.cos(2 * t / 5))
        command, error = tracking_command(ref, (x, y, yaw), (3, 4, 2))
        vx, vy, omega = command
        x += (math.cos(yaw) * vx - math.sin(yaw) * vy) * dt
        y += (math.sin(yaw) * vx + math.cos(yaw) * vy) * dt
        yaw = wrap_angle(yaw + omega * dt)
        errors.append(math.sqrt(sum(e * e for e in error)))
    assert errors[-1] < 0.005
    assert errors[-1] < errors[0] / 50


@pytest.fixture
def controller():
    rclpy.init()
    node = TrackingController()
    now = Time(sec=10, nanosec=150000000)
    node.odom = Odometry()
    node.odom.header.stamp = now
    node.odom.header.frame_id = "odom"
    node.odom.child_frame_id = "base_link"
    node.odom.pose.pose.orientation.w = 1.0
    node.nominal = ExecutedCommand(plan_id=1, execution_state="ACTIVE_PLAN")
    node.nominal.header.stamp = now
    node.nominal.header.frame_id = "base_link"
    node.mode = DriveModeState(current_mode=0, requested_mode=0, status=1)
    node.mode.header.stamp = now
    node.on_trajectory(plan())
    yield node
    node.destroy_node()
    rclpy.shutdown()


def test_delayed_arrival_selects_current_time_and_waits_for_matching_plan(controller):
    command, state = controller.calculate(10_150_000_000)
    assert state == "TRACKING"
    assert controller.reference.x == pytest.approx(0.15)
    assert command[0] == pytest.approx(1.45)
    controller.on_trajectory(plan(2, 11))
    assert controller.calculate(10_150_000_000)[1] == "TRACKING"
    controller.nominal.plan_id = 2
    assert controller.calculate(10_150_000_000) == (ZERO, "TRAJECTORY_NOT_STARTED")
    controller.on_trajectory(plan())  # Out-of-order delivery must not replace plan 2.
    assert max(controller.plans) == 2
    controller.nominal.plan_id = 9
    assert controller.calculate(10_150_000_000) == (ZERO, "WAIT_TRAJECTORY")


@pytest.mark.parametrize("source,state", [("nominal", "WAIT_COMMAND"), ("odom", "WAIT_ODOM"), ("mode", "WAIT_MODE")])
def test_stale_inputs_stop(controller, source, state):
    getattr(controller, source).header.stamp = Time(sec=9)
    assert controller.calculate(10_150_000_000) == (ZERO, state)


def test_expiration_frame_mismatch_and_clock_reset(controller):
    for message in (controller.odom, controller.nominal, controller.mode):
        message.header.stamp = Time(sec=10, nanosec=400000000)
    assert controller.calculate(10_400_000_000) == (ZERO, "TRAJECTORY_EXPIRED")
    assert controller.calculate(1_000_000_000) == (ZERO, "CLOCK_RESET")
    assert not controller.plans and controller.nominal is None


def test_mode_and_frame_guards(controller):
    controller.mode.status = 0
    assert controller.calculate(10_150_000_000) == (ZERO, "MODE_ALIGNING")
    controller.mode.status = 1
    controller.mode.current_mode = controller.mode.requested_mode = 2
    assert controller.calculate(10_150_000_000) == (ZERO, "MODE_MISMATCH")
    controller.mode.current_mode = controller.mode.requested_mode = 0
    controller.odom.header.frame_id = "map"
    assert controller.calculate(10_150_000_000) == (ZERO, "ODOM_FRAME_MISMATCH")


@pytest.mark.parametrize("state", ["SAFETY_STOP", "MODE_STOP", "SPOT_TURN_ROTATING"])
def test_special_commands_survive_without_a_trajectory(controller, state):
    controller.plans.clear()
    controller.nominal.execution_state = state
    if state == "SPOT_TURN_ROTATING":
        controller.mode.current_mode = controller.mode.requested_mode = 4
        controller.nominal.yaw_rate = 0.3
    else:
        controller.nominal.vx = 0.2
    command, actual_state = controller.calculate(10_150_000_000)
    assert actual_state == state
    assert command == (controller.nominal.vx, 0.0, controller.nominal.yaw_rate)
    controller.nominal.execution_state = "TERMINAL_HOLD"
    assert controller.calculate(10_150_000_000) == (ZERO, "TERMINAL_HOLD")
