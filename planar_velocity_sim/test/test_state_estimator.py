import math

import pytest

from planar_velocity_sim.state_estimator import (
    PlanarState,
    StateEstimatorConfig,
    StateEstimatorModel,
)


def sample(stamp: float, x: float, yaw: float = 0.0) -> PlanarState:
    return PlanarState(stamp, x, 2.0 * x, yaw, x, -x, 0.1 * x)


def test_disabled_estimator_returns_exact_truth() -> None:
    model = StateEstimatorModel(StateEstimatorConfig(enabled=False))
    truth = sample(1.0, 3.0)
    model.record_truth(truth)

    estimate = model.estimate(1.0)

    assert estimate is not None
    assert estimate.state == truth
    assert estimate.pose_covariance == tuple([0.0] * 36)


def test_estimator_interpolates_delayed_truth_and_wraps_yaw() -> None:
    model = StateEstimatorModel(
        StateEstimatorConfig(enabled=False, delay_sec=0.5)
    )
    model.record_truth(sample(0.0, 0.0, math.radians(179.0)))
    model.record_truth(sample(1.0, 2.0, math.radians(-179.0)))

    estimate = model.estimate(1.0)

    assert estimate is not None
    assert estimate.state.stamp_sec == pytest.approx(0.5)
    assert estimate.state.x == pytest.approx(1.0)
    assert abs(abs(estimate.state.yaw) - math.pi) < 1.0e-12


def test_estimator_noise_is_reproducible_and_covariance_is_reported() -> None:
    config = StateEstimatorConfig(
        enabled=True,
        position_noise_std_m=0.1,
        position_bias_std_m=0.2,
        random_seed=7,
    )
    first = StateEstimatorModel(config)
    second = StateEstimatorModel(config)
    truth = sample(0.0, 0.0)
    first.record_truth(truth)
    second.record_truth(truth)

    first_estimate = first.estimate(0.1)
    second_estimate = second.estimate(0.1)

    assert first_estimate is not None
    assert second_estimate is not None
    assert first_estimate.state == second_estimate.state
    assert first_estimate.state != truth
    assert first_estimate.pose_covariance[0] == pytest.approx(0.1**2 + 0.2**2)


def test_estimator_can_drop_an_output_sample() -> None:
    model = StateEstimatorModel(
        StateEstimatorConfig(enabled=True, dropout_probability=1.0)
    )
    model.record_truth(sample(0.0, 0.0))

    assert model.estimate(0.1) is None

