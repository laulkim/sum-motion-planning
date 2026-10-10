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
from planar_velocity_sim.vehicle_response import AR2Noise, AR2_NOISE_DEFAULTS, SensorDelay


@pytest.fixture
def simulator(monkeypatch, request):
    # Existing deterministic response tests explicitly disable measurement noise.
    rclpy.init(args=getattr(request, "param", [
        "--ros-args", "-p", "sensor_noise_enabled:=false"
    ]))
    node = PlanarVelocitySimNode()
    now = SimpleNamespace(sec=0.0)
    clock = SimpleNamespace(now=lambda: Time(
        nanoseconds=round(now.sec * 1e9), clock_type=ClockType.ROS_TIME
    ))
    monkeypatch.setattr(node, "get_clock", lambda: clock)
    node.last_update_time = clock.now()
    for response in node.responses:
        response.reset(0.0)
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


@pytest.mark.parametrize("simulator", [
    ["--ros-args", "-p", "sensor_noise_enabled:=false"],
    [],  # default enabled, scale=1, seed=42
    ["--ros-args", "-p", "sensor_noise_scale:=0.0"],
], indirect=True)
def test_actual_pose_and_delayed_odometry(simulator):
    node, clock, odometry = simulator
    expected_noise = [
        AR2Noise(a1, a2, std * node.get_parameter("sensor_noise_scale").value, 42 + i)
        for i, (a1, a2, std) in enumerate(AR2_NOISE_DEFAULTS.values())
    ]
    command(node, vx=1.0, vy=0.5, yaw_rate=0.2)
    history = {0: node.actual_state()}
    for i in range(1, 31):
        tick(node, clock, i * 0.01)
        history[i] = node.actual_state()
        # Noisy measurements must never affect the ground-truth FOPDT response.
        for actual, reference, response in zip(history[i][3:], (1.0, 0.5, 0.2), node.responses):
            elapsed = max(0.0, i * 0.01 - response.delay_sec)
            assert actual == pytest.approx(
                reference * response.gain * (1.0 - math.exp(-elapsed / response.tau_sec))
            )
        if i < 10:
            assert not odometry
            continue
        msg = odometry[-1]
        x, y, yaw, vx, vy, yaw_rate = history[i - 10]
        if node.sensor_noise_enabled:
            vx += expected_noise[0].sample()
            vy += expected_noise[1].sample()
            yaw_rate += expected_noise[2].sample()
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


def test_aligning_stops_through_response_and_integrates_residual_motion(simulator):
    node, clock, odometry = simulator
    command(node, vx=0.02, vy=0.01, yaw_rate=0.2)
    tick(node, clock, 0.04)
    before = node.actual_state()
    node.mode_command_callback(UInt8(data=2))
    assert node.mode_model.transition_in_progress
    assert node.actual_state() == before  # no instantaneous velocity reset
    clock.sec = 0.05
    command(node, vx=1.0, vy=2.0, yaw_rate=0.5)  # retained, not applied yet
    node.update()
    vy_response = node.responses[1]
    # vx/yaw stop commands still have 20 ms dead time; vy decays immediately.
    assert node.applied_vx > before[3]
    assert node.applied_yaw_rate > before[5]
    assert node.applied_vy == pytest.approx(before[4] * math.exp(-0.01 / vy_response.tau_sec))
    assert node.x > before[0] and node.y > before[1] and node.yaw > before[2]
    tick(node, clock, 0.06)
    at_stop = node.actual_state()[3:]
    tick(node, clock, 0.07)
    for actual, previous, response in zip(node.actual_state()[3:], at_stop, node.responses):
        assert actual == pytest.approx(previous * math.exp(-0.01 / response.tau_sec))
        assert actual > 0.0
    assert all(response.reference == 0.0 for response in node.responses)
    # Delayed odometry eventually reports the residual motion, not a reset pose.
    tick(node, clock, 0.15)
    assert odometry[-1].pose.pose.position.x > before[0]
    assert odometry[-1].twist.twist.linear.x > 0.0
    tick(node, clock, 2.04)
    assert not node.mode_model.transition_in_progress
    settled_vx = node.applied_vx
    tick(node, clock, 2.05)
    assert node.applied_vx <= settled_vx  # resumed vx is still in dead time
    assert node.applied_vy > 0.0
    tick(node, clock, 2.07)
    assert node.applied_vx > 0.0 and node.applied_yaw_rate > 0.0


def test_aligning_preserves_commands_already_in_dead_time(simulator):
    node, clock, odometry = simulator
    command(node, vx=0.02)
    tick(node, clock, 0.03)
    clock.sec = 0.035
    command(node, vx=1.0)  # will arrive at 55 ms
    tick(node, clock, 0.04)
    before = node.applied_vx
    node.mode_command_callback(UInt8(data=2))  # zero will arrive at 60 ms
    assert node.mode_model.transition_in_progress
    tick(node, clock, 0.057)
    assert node.applied_vx > before  # in-flight command was not erased
    tick(node, clock, 0.06)
    at_stop = node.applied_vx
    tick(node, clock, 0.07)
    assert node.applied_vx == pytest.approx(
        at_stop * math.exp(-0.01 / node.responses[0].tau_sec)
    )


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
