"""Exercise real ROS node callbacks with a deterministic clock, without spinning."""

import math
from types import SimpleNamespace

import pytest
import rclpy
from geometry_msgs.msg import Twist
from rclpy.clock import ClockType
from rclpy.time import Time
from std_msgs.msg import UInt8

from planar_velocity_sim.planar_velocity_sim_node import PlanarVelocitySimNode
from planar_velocity_sim.vehicle_response import SensorDelay


@pytest.fixture
def simulator(monkeypatch):
    rclpy.init()
    node = PlanarVelocitySimNode()
    now = SimpleNamespace(sec=0.0)
    clock = SimpleNamespace(now=lambda: Time(
        nanoseconds=round(now.sec * 1e9), clock_type=ClockType.ROS_TIME
    ))
    monkeypatch.setattr(node, "get_clock", lambda: clock)
    node.last_update_time = clock.now()
    node.reset_response(0.0)
    node.sensor = SensorDelay(node.sensor.delay_sec)
    node.sensor.update(0.0, node.actual_state())
    odometry = []
    node.odom_pub = SimpleNamespace(publish=odometry.append)
    try:
        yield node, now, odometry
    finally:
        node.destroy_node()
        rclpy.shutdown()


def command(node, vx=0.0, vy=0.0, yaw_rate=0.0):
    msg = Twist()
    msg.linear.x, msg.linear.y, msg.angular.z = vx, vy, yaw_rate
    node.cmd_vel_callback(msg)


def tick(node, clock, t):
    clock.sec = t
    node.update()


def test_actual_pose_and_delayed_odometry(simulator):
    node, clock, odometry = simulator
    command(node, vx=1.0, vy=0.5, yaw_rate=0.2)
    history = {0: node.actual_state()}
    for i in range(1, 31):
        tick(node, clock, i * 0.01)
        history[i] = node.actual_state()
        if i < 10:
            assert not odometry
            continue
        msg = odometry[-1]
        x, y, yaw, vx, vy, yaw_rate = history[i - 10]
        assert msg.pose.pose.position.x == pytest.approx(x)
        assert msg.pose.pose.position.y == pytest.approx(y)
        assert msg.pose.pose.orientation.z == pytest.approx(math.sin(yaw / 2))
        assert msg.twist.twist.linear.x == pytest.approx(vx)
        assert msg.twist.twist.linear.y == pytest.approx(vy)
        assert msg.twist.twist.angular.z == pytest.approx(yaw_rate)
        assert msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9 == pytest.approx(
            (i - 10) * 0.01
        )
    # Ground truth responds before sensor delivery and uses actual, not reference.
    assert history[1][0] == 0.0  # vx still in dead time
    expected_vy = 0.5 * 0.94555 * (1 - math.exp(-0.01 / 0.02441))
    assert history[1][1] == pytest.approx(expected_vy * 0.01)
    assert node.x > odometry[-1].pose.pose.position.x


def test_aligning_forces_actual_zero_and_clears_old_response(simulator):
    node, clock, odometry = simulator
    command(node, vx=0.02, vy=0.01, yaw_rate=0.2)
    tick(node, clock, 0.04)
    assert node.applied_vx > 0.0 and node.applied_yaw_rate > 0.0
    # Accepted using actual velocity below the standstill threshold.
    node.mode_command_callback(UInt8(data=2))
    assert node.mode_model.transition_in_progress
    assert node.actual_state()[3:] == (0.0, 0.0, 0.0)
    pose = node.actual_state()[:3]
    for t in (0.05, 0.16, 0.8, 2.03):
        clock.sec = t
        command(node, vx=1.0, vy=2.0, yaw_rate=0.5)
        node.update()
        assert node.actual_state()[:3] == pytest.approx(pose)
        assert node.actual_state()[3:] == (0.0, 0.0, 0.0)
        assert all(not response.pending for response in node.responses)
    tick(node, clock, 2.04)
    assert not node.mode_model.transition_in_progress
    assert node.actual_state()[3:] == (0.0, 0.0, 0.0)
    tick(node, clock, 2.05)
    assert node.applied_vx == node.applied_yaw_rate == 0.0
    assert node.applied_vy > 0.0
    tick(node, clock, 2.07)
    assert node.applied_vx > 0.0 and node.applied_yaw_rate > 0.0


def test_mode_change_checks_actual_speed_not_delayed_sensor(simulator):
    node, clock, odometry = simulator
    command(node, vx=1.0)
    tick(node, clock, 0.04)
    assert not odometry  # sensor has no measurement yet
    node.mode_command_callback(UInt8(data=2))
    assert not node.mode_model.transition_in_progress
    assert node.mode_model.current_mode == 0


def test_cancelling_alignment_resumes_latest_reference(simulator):
    node, clock, odometry = simulator
    node.mode_command_callback(UInt8(data=2))
    clock.sec = 0.05
    command(node, vx=1.0)
    node.update()
    assert node.applied_vx == 0.0
    node.mode_command_callback(UInt8(data=0))
    assert not node.mode_model.transition_in_progress
    tick(node, clock, 0.06)
    assert node.applied_vx == 0.0
    tick(node, clock, 0.08)
    assert node.applied_vx > 0.0
