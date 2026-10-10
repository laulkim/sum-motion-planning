import math

import pytest

from planar_velocity_sim.vehicle_response import (
    AR2Noise,
    AR2_NOISE_DEFAULTS,
    FirstOrderResponse,
    RESPONSE_PARAMETER_DEFAULTS,
    SensorDelay,
)


@pytest.mark.parametrize("parameters", AR2_NOISE_DEFAULTS.values())
def test_ar2_seed_reproduces_sequence(parameters):
    first = AR2Noise(*parameters, seed=42)
    second = AR2Noise(*parameters, seed=42)
    other = AR2Noise(*parameters, seed=43)
    sequence = [first.sample() for _ in range(20)]
    assert sequence == [second.sample() for _ in range(20)]
    assert sequence != [other.sample() for _ in range(20)]


def test_ar2_zero_innovation():
    noise = AR2Noise(0.4, 0.5, 0.0, seed=42)
    assert [noise.sample() for _ in range(20)] == [0.0] * 20


def test_ar2_recurrence(monkeypatch):
    noise = AR2Noise(0.4, 0.5, 0.01, seed=42)
    previous1, previous2 = noise.previous1, noise.previous2
    monkeypatch.setattr(noise.rng, "gauss", lambda mean, std: std)
    expected = 0.4 * previous1 + 0.5 * previous2 + 0.01
    assert noise.sample() == pytest.approx(expected)
    assert noise.sample() == pytest.approx(0.4 * expected + 0.5 * previous1 + 0.01)


@pytest.mark.parametrize("a1,a2,std", [
    (0.4, 0.5, -0.01), (0.4, 0.5, math.nan), (0.4, 0.5, math.inf),
    (math.nan, 0.5, 0.01), (0.4, math.inf, 0.01),
    (0.5, 0.5, 0.01), (0.4, -1.0, 0.01),
])
def test_invalid_ar2_parameters(a1, a2, std):
    with pytest.raises(ValueError):
        AR2Noise(a1, a2, std, seed=42)


def channel_response(channel, now_sec=0.0):
    return FirstOrderResponse(
        RESPONSE_PARAMETER_DEFAULTS[f"{channel}_gain"],
        RESPONSE_PARAMETER_DEFAULTS[f"{channel}_tau_sec"],
        RESPONSE_PARAMETER_DEFAULTS[f"{channel}_delay_sec"],
        now_sec,
    )


@pytest.mark.parametrize("channel", ["vx", "vy", "yaw_rate"])
def test_step_response_and_steady_state_gain(channel):
    response = channel_response(channel)
    response.command(2.0, 0.0)
    for elapsed in (0.003, 0.010, 0.027, 0.100, 1.000):
        t = response.delay_sec + elapsed
        expected = 2.0 * response.gain * (1.0 - math.exp(-elapsed / response.tau_sec))
        assert response.update(t) == pytest.approx(expected)
    assert response.actual == pytest.approx(2.0 * response.gain)


@pytest.mark.parametrize("channel", ["vx", "yaw_rate"])
def test_twenty_ms_delay_uses_command_receipt_time(channel):
    response = channel_response(channel)
    response.command(1.0, 0.003)
    assert response.update(0.019) == 0.0
    assert response.update(0.023) == 0.0
    # The event at 23 ms is inside this irregular timer interval.
    expected = response.gain * (1.0 - math.exp(-0.004 / response.tau_sec))
    assert response.update(0.027) == pytest.approx(expected)


def test_multiple_delayed_changes_within_one_update():
    response = FirstOrderResponse(1.0, 0.01, 0.02, 0.0)
    response.command(1.0, 0.0)
    response.command(0.0, 0.005)
    response.command(-1.0, 0.010)
    at_25 = 1.0 - math.exp(-0.005 / 0.01)
    at_30 = at_25 * math.exp(-0.005 / 0.01)
    expected = at_30 * math.exp(-0.01 / 0.01) - (1.0 - math.exp(-0.01 / 0.01))
    assert response.update(0.04) == pytest.approx(expected)


@pytest.mark.parametrize("dt", [0.005, 0.010, 0.025])
def test_sensor_hundred_ms_delay_preserves_pose_velocity_and_timestamp(dt):
    sensor = SensorDelay(RESPONSE_PARAMETER_DEFAULTS["sensor_delay_sec"])
    for i in range(round(0.2 / dt) + 1):
        t = i * dt
        state = (t, 2 * t, 3 * t, 4 * t, 5 * t, 6 * t)
        measurement = sensor.update(t, state)
        if t < 0.1 - 1e-12:
            assert measurement is None
        else:
            stamp, delayed = measurement
            assert stamp == pytest.approx(t - 0.1)
            assert delayed == pytest.approx(tuple(n * stamp for n in range(1, 7)))
    assert len(sensor.pending) <= math.ceil(0.1 / dt)


def test_sensor_delay_with_irregular_sampling_and_zero_delay():
    sensor = SensorDelay(0.1)
    assert sensor.update(0.0, (0.0,)) is None
    assert sensor.update(0.037, (1.0,)) is None
    assert sensor.update(0.099, (2.0,)) is None
    assert sensor.update(0.101, (3.0,)) == (0.0, (0.0,))
    assert sensor.update(0.138, (4.0,)) == (0.037, (1.0,))
    assert SensorDelay(0.0).update(0.5, (7.0,)) == (0.5, (7.0,))


@pytest.mark.parametrize("gain,tau,delay", [
    (1.0, 0.0, 0.02), (1.0, -0.1, 0.02), (1.0, 0.01, -0.02),
    (math.nan, 0.01, 0.02), (1.0, math.inf, 0.02),
])
def test_invalid_response_parameters(gain, tau, delay):
    with pytest.raises(ValueError):
        FirstOrderResponse(gain, tau, delay, 0.0)
