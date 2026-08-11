import math

import pytest

from planar_velocity_sim.kinematics import integrate_body_velocity


def test_forward_and_lateral_motion() -> None:
    x, y, yaw = integrate_body_velocity(0.0, 0.0, 0.0, 1.0, 2.0, 0.0, 0.5)
    assert x == pytest.approx(0.5)
    assert y == pytest.approx(1.0)
    assert yaw == pytest.approx(0.0)


def test_midpoint_yaw_integration() -> None:
    x, y, yaw = integrate_body_velocity(0.0, 0.0, 0.0, 1.0, 0.0, 1.0, 0.2)
    assert x == pytest.approx(math.cos(0.1) * 0.2)
    assert y == pytest.approx(math.sin(0.1) * 0.2)
    assert yaw == pytest.approx(0.2)


def test_negative_dt_rejected() -> None:
    with pytest.raises(ValueError):
        integrate_body_velocity(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.1)
