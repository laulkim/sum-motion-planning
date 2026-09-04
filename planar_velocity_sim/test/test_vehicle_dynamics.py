import pytest

from planar_velocity_sim.vehicle_dynamics import (
    BodyVelocity,
    CommandChannel,
    CommandChannelConfig,
    PlanarVehicleDynamics,
    VehicleDynamicsConfig,
)


def test_command_channel_applies_delay_and_watchdog() -> None:
    channel = CommandChannel(CommandChannelConfig(delay_sec=0.1, timeout_sec=0.5))
    command = BodyVelocity(1.0, 2.0, 0.3)
    channel.push(command, 1.0)

    assert channel.sample(1.09) == BodyVelocity()
    assert channel.sample(1.10) == command
    assert channel.sample(1.50) == command
    assert channel.sample(1.5001) == BodyVelocity()


def test_disabled_dynamics_preserves_ideal_velocity_interface() -> None:
    model = PlanarVehicleDynamics(VehicleDynamicsConfig(enabled=False))

    velocity = model.step(BodyVelocity(10.0, -8.0, 2.0), 0.01)

    assert velocity == BodyVelocity(10.0, -8.0, 2.0)


def test_dynamics_respects_acceleration_and_jerk_limits() -> None:
    config = VehicleDynamicsConfig(
        enabled=True,
        vx_time_constant_sec=0.1,
        max_vx_acceleration_mps2=1.0,
        max_vx_jerk_mps3=2.0,
    )
    model = PlanarVehicleDynamics(config)

    first = model.step(BodyVelocity(vx=5.0), 0.1)
    second = model.step(BodyVelocity(vx=5.0), 0.1)

    assert model.vx_acceleration <= config.max_vx_acceleration_mps2
    assert first.vx == pytest.approx(0.02)
    assert second.vx == pytest.approx(0.06)


def test_zero_target_settles_without_velocity_sign_chatter() -> None:
    model = PlanarVehicleDynamics(
        VehicleDynamicsConfig(
            enabled=True,
            rolling_resistance_acceleration_mps2=0.1,
            stop_speed_threshold_mps=0.01,
        )
    )
    model.velocity = BodyVelocity(vx=0.02)

    for _ in range(100):
        velocity = model.step(BodyVelocity(), 0.01)

    assert velocity.vx == 0.0
    assert model.vx_acceleration == 0.0
