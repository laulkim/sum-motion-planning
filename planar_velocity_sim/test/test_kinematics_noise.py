import math
import random

import pytest

import planar_velocity_sim.kinematics_noise as kinematics_noise
from planar_velocity_sim.kinematics import integrate_body_velocity
from planar_velocity_sim.kinematics_noise import KinematicsNoiseModel


def test_zero_command_keeps_vehicle_stopped() -> None:
    state = KinematicsNoiseModel().step(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

    assert state.x == 0.0
    assert state.y == 0.0
    assert state.yaw == 0.0


def test_longitudinal_response_converges_to_allocated_command() -> None:
    model = KinematicsNoiseModel()
    pose = (0.0, 0.0, 0.0)
    for _ in range(1000):
        state = model.step(*pose, 1.0, 0.0, 0.0)
        pose = state[:3]

    assert model.actual_vx == pytest.approx(1.0, abs=1.0e-6)
    assert model.actual_vy == 0.0
    assert model.actual_yaw_rate == 0.0


def test_lateral_response_starts_after_013_second_delay() -> None:
    model = KinematicsNoiseModel()
    pose = (0.0, 0.0, 0.0)

    for _ in range(13):
        state = model.step(*pose, 0.0, 1.0, 0.0)
        pose = state[:3]
        assert model.actual_vy == 0.0

    model.step(*pose, 0.0, 1.0, 0.0)
    assert model.actual_vy > 0.0


def test_yaw_rate_is_the_weighted_sum_of_both_branches() -> None:
    model = KinematicsNoiseModel()
    for _ in range(50):
        model.step(0.0, 0.0, 0.0, 0.0, 0.0, 0.4)

    expected = (
        kinematics_noise.DRIVE_YAW_WEIGHT * model.drive_yaw_rate
        + kinematics_noise.STEERING_YAW_WEIGHT * model.steering_yaw_rate
    )
    assert kinematics_noise.DRIVE_YAW_WEIGHT == pytest.approx(
        0.285469, abs=1.0e-6
    )
    assert kinematics_noise.STEERING_YAW_WEIGHT == pytest.approx(
        0.714531, abs=1.0e-6
    )
    assert model.actual_yaw_rate == pytest.approx(expected)


def test_pose_uses_actual_vehicle_response() -> None:
    model = KinematicsNoiseModel()
    state = model.step(0.0, 0.0, 0.0, 1.0, 0.5, 0.2)
    expected = integrate_body_velocity(
        0.0,
        0.0,
        0.0,
        0.5 * model.actual_vx,
        0.5 * model.actual_vy,
        0.5 * model.actual_yaw_rate,
        0.01,
    )

    assert state[:3] == pytest.approx(expected)
    assert state.beta == pytest.approx(
        math.atan2(model.actual_vy, model.actual_vx)
    )


def test_sensor_delays_are_applied_to_actual_vehicle_output(monkeypatch) -> None:
    monkeypatch.setattr(
        kinematics_noise,
        "SENSOR_PARAMETERS",
        ((0.06, 0.0, 0.0, 0.0), (0.06, 0.0, 0.0, 0.0), (0.04, 0.0, 0.0, 0.0)),
    )
    model = KinematicsNoiseModel()
    actual_vx = []
    actual_yaw_rate = []
    states = []

    for _ in range(7):
        states.append(model.step(0.0, 0.0, 0.0, 1.0, 0.0, 0.3))
        actual_vx.append(model.actual_vx)
        actual_yaw_rate.append(model.actual_yaw_rate)

    assert states[3].yaw_rate == 0.0
    assert states[4].yaw_rate == pytest.approx(actual_yaw_rate[0])
    assert states[5].vx == 0.0
    assert states[6].vx == pytest.approx(actual_vx[0])


def test_reference_noise_equation_and_reset_are_deterministic() -> None:
    model = KinematicsNoiseModel(random_seed=7)
    first = model.step(0.0, 0.0, 0.0, 1.0, 2.0, 0.3, 0.02)

    expected_random = random.Random(7)
    expected = []
    for _, correlation, error_sigma, bias_sigma in kinematics_noise.SENSOR_PARAMETERS:
        error = (
            error_sigma
            * math.sqrt(1.0 - correlation**2)
            * expected_random.gauss(0.0, 1.0)
        )
        bias = bias_sigma * expected_random.gauss(0.0, 1.0)
        expected.append(error + bias)

    assert first[3:6] == pytest.approx(expected)
    model.reset()
    assert model.step(0.0, 0.0, 0.0, 1.0, 2.0, 0.3, 0.02) == first


def test_non_positive_dt_is_rejected() -> None:
    with pytest.raises(ValueError):
        KinematicsNoiseModel().step(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
