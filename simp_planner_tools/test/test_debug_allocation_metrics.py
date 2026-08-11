from __future__ import annotations

import math

import pytest

from simp_planner_tools.debug_allocation_metrics import compute_allocation_debug_sample


def test_direct_allocation_identities_are_zero_without_differentiation() -> None:
    beta = math.radians(30.0)
    speed = 2.0
    beta_rate = math.radians(4.0)
    yaw_rate = math.radians(6.0)
    result = compute_allocation_debug_sample(
        vx=speed * math.cos(beta),
        vy=speed * math.sin(beta),
        yaw_rate=yaw_rate,
        planned_speed=speed,
        motion_heading_rate=yaw_rate + beta_rate,
        motion_heading_acceleration=math.radians(3.0),
        beta=beta,
        beta_rate=beta_rate,
        yaw_acceleration=math.radians(2.0),
        mode=0,
    )
    assert result.speed_reconstruction_error_mps == pytest.approx(0.0, abs=1e-12)
    assert result.rate_split_residual_degps == pytest.approx(0.0, abs=1e-12)
    assert result.beta_acceleration_degps2 == pytest.approx(1.0)
    assert result.beta_deviation_deg == pytest.approx(30.0)


def test_crab_mode_center_is_reported_directly() -> None:
    result = compute_allocation_debug_sample(
        vx=0.0,
        vy=1.0,
        yaw_rate=0.0,
        planned_speed=1.0,
        motion_heading_rate=0.0,
        motion_heading_acceleration=0.0,
        beta=math.pi / 2.0,
        beta_rate=0.0,
        yaw_acceleration=0.0,
        mode=2,
    )
    assert result.beta_center_deg == pytest.approx(90.0)
    assert result.beta_deviation_deg == pytest.approx(0.0)
