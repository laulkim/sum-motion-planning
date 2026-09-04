from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass


@dataclass(frozen=True)
class BodyVelocity:
    vx: float = 0.0
    vy: float = 0.0
    yaw_rate: float = 0.0


@dataclass(frozen=True)
class CommandChannelConfig:
    delay_sec: float = 0.0
    timeout_sec: float = 0.0

    def __post_init__(self) -> None:
        if self.delay_sec < 0.0:
            raise ValueError("delay_sec must be non-negative")
        if self.timeout_sec < 0.0:
            raise ValueError("timeout_sec must be non-negative")


class CommandChannel:
    """Timestamped body-velocity command delay and watchdog model."""

    def __init__(self, config: CommandChannelConfig) -> None:
        self.config = config
        self._history: deque[tuple[float, BodyVelocity]] = deque()
        self._last_receive_sec: float | None = None

    def push(self, command: BodyVelocity, receive_sec: float) -> None:
        receive_sec = float(receive_sec)
        if self._last_receive_sec is not None and receive_sec < self._last_receive_sec:
            raise ValueError("command timestamps must be monotonic")
        self._history.append((receive_sec, command))
        self._last_receive_sec = receive_sec

    def reset(self, command: BodyVelocity, receive_sec: float) -> None:
        self._history.clear()
        self._last_receive_sec = None
        self.push(command, receive_sec)

    def sample(self, now_sec: float) -> BodyVelocity:
        now_sec = float(now_sec)
        if self._last_receive_sec is None:
            return BodyVelocity()
        if (
            self.config.timeout_sec > 0.0
            and now_sec - self._last_receive_sec > self.config.timeout_sec
        ):
            return BodyVelocity()

        apply_before_sec = now_sec - self.config.delay_sec
        while len(self._history) >= 2 and self._history[1][0] <= apply_before_sec:
            self._history.popleft()
        if self._history and self._history[0][0] <= apply_before_sec:
            return self._history[0][1]
        return BodyVelocity()


@dataclass(frozen=True)
class VehicleDynamicsConfig:
    enabled: bool = False
    vx_time_constant_sec: float = 0.20
    vy_time_constant_sec: float = 0.30
    yaw_rate_time_constant_sec: float = 0.20
    vx_command_gain: float = 1.0
    vy_command_gain: float = 1.0
    yaw_rate_command_gain: float = 1.0
    max_vx_mps: float = 6.0
    max_vy_mps: float = 6.0
    max_yaw_rate_radps: float = 1.0
    max_vx_acceleration_mps2: float = 1.5
    max_vy_acceleration_mps2: float = 1.2
    max_yaw_acceleration_radps2: float = 1.5
    max_vx_jerk_mps3: float = 4.0
    max_vy_jerk_mps3: float = 3.0
    max_yaw_jerk_radps3: float = 5.0
    rolling_resistance_acceleration_mps2: float = 0.0
    command_deadband_mps: float = 0.0
    yaw_rate_deadband_radps: float = 0.0
    stop_speed_threshold_mps: float = 1.0e-3

    def __post_init__(self) -> None:
        positive = {
            "vx_time_constant_sec": self.vx_time_constant_sec,
            "vy_time_constant_sec": self.vy_time_constant_sec,
            "yaw_rate_time_constant_sec": self.yaw_rate_time_constant_sec,
            "max_vx_mps": self.max_vx_mps,
            "max_vy_mps": self.max_vy_mps,
            "max_yaw_rate_radps": self.max_yaw_rate_radps,
            "max_vx_acceleration_mps2": self.max_vx_acceleration_mps2,
            "max_vy_acceleration_mps2": self.max_vy_acceleration_mps2,
            "max_yaw_acceleration_radps2": self.max_yaw_acceleration_radps2,
            "max_vx_jerk_mps3": self.max_vx_jerk_mps3,
            "max_vy_jerk_mps3": self.max_vy_jerk_mps3,
            "max_yaw_jerk_radps3": self.max_yaw_jerk_radps3,
        }
        for name, value in positive.items():
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive")
        non_negative = {
            "rolling_resistance_acceleration_mps2": (
                self.rolling_resistance_acceleration_mps2
            ),
            "command_deadband_mps": self.command_deadband_mps,
            "yaw_rate_deadband_radps": self.yaw_rate_deadband_radps,
            "stop_speed_threshold_mps": self.stop_speed_threshold_mps,
        }
        for name, value in non_negative.items():
            if not math.isfinite(value) or value < 0.0:
                raise ValueError(f"{name} must be finite and non-negative")
        for name, value in (
            ("vx_command_gain", self.vx_command_gain),
            ("vy_command_gain", self.vy_command_gain),
            ("yaw_rate_command_gain", self.yaw_rate_command_gain),
        ):
            if not math.isfinite(value) or value < 0.0:
                raise ValueError(f"{name} must be finite and non-negative")


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


def _move_towards(value: float, target: float, maximum_change: float) -> float:
    return value + _clamp(target - value, maximum_change)


class PlanarVehicleDynamics:
    """Actuator-level dynamics for a holonomic planar velocity interface.

    The planner commands body-frame vx, vy, and yaw rate rather than wheel
    angle or torque.  This model therefore represents the closed low-level
    velocity loops, while retaining support for forward, reverse, and crab
    drive modes.
    """

    def __init__(self, config: VehicleDynamicsConfig) -> None:
        self.config = config
        self.velocity = BodyVelocity()
        self.vx_acceleration = 0.0
        self.vy_acceleration = 0.0
        self.yaw_acceleration = 0.0

    def _step_axis(
        self,
        value: float,
        acceleration: float,
        target: float,
        *,
        time_constant: float,
        maximum_value: float,
        maximum_acceleration: float,
        maximum_jerk: float,
        resistance: float,
        stop_threshold: float,
        dt: float,
    ) -> tuple[float, float]:
        target = _clamp(target, maximum_value)
        desired_acceleration = (target - value) / time_constant
        if resistance > 0.0 and abs(value) > stop_threshold:
            desired_acceleration -= math.copysign(resistance, value)
        desired_acceleration = _clamp(desired_acceleration, maximum_acceleration)
        next_acceleration = _move_towards(
            acceleration, desired_acceleration, maximum_jerk * dt
        )
        next_value = _clamp(value + next_acceleration * dt, maximum_value)

        if abs(target) <= stop_threshold:
            crossed_zero = value != 0.0 and value * next_value <= 0.0
            settled = (
                abs(next_value) <= stop_threshold
                and abs(next_acceleration) <= max(resistance, maximum_jerk * dt)
            )
            if crossed_zero or settled:
                return 0.0, 0.0
        if abs(next_value) >= maximum_value and next_value * next_acceleration > 0.0:
            next_acceleration = 0.0
        return next_value, next_acceleration

    def step(self, command: BodyVelocity, dt: float) -> BodyVelocity:
        dt = float(dt)
        if dt < 0.0:
            raise ValueError("dt must be non-negative")
        if dt == 0.0:
            return self.velocity

        cfg = self.config
        if not cfg.enabled:
            next_velocity = BodyVelocity(
                float(command.vx),
                float(command.vy),
                float(command.yaw_rate),
            )
            self.vx_acceleration = (next_velocity.vx - self.velocity.vx) / dt
            self.vy_acceleration = (next_velocity.vy - self.velocity.vy) / dt
            self.yaw_acceleration = (
                next_velocity.yaw_rate - self.velocity.yaw_rate
            ) / dt
            self.velocity = next_velocity
            return self.velocity

        vx_target = cfg.vx_command_gain * float(command.vx)
        vy_target = cfg.vy_command_gain * float(command.vy)
        yaw_target = cfg.yaw_rate_command_gain * float(command.yaw_rate)
        if abs(vx_target) < cfg.command_deadband_mps:
            vx_target = 0.0
        if abs(vy_target) < cfg.command_deadband_mps:
            vy_target = 0.0
        if abs(yaw_target) < cfg.yaw_rate_deadband_radps:
            yaw_target = 0.0

        vx, self.vx_acceleration = self._step_axis(
            self.velocity.vx,
            self.vx_acceleration,
            vx_target,
            time_constant=cfg.vx_time_constant_sec,
            maximum_value=cfg.max_vx_mps,
            maximum_acceleration=cfg.max_vx_acceleration_mps2,
            maximum_jerk=cfg.max_vx_jerk_mps3,
            resistance=cfg.rolling_resistance_acceleration_mps2,
            stop_threshold=cfg.stop_speed_threshold_mps,
            dt=dt,
        )
        vy, self.vy_acceleration = self._step_axis(
            self.velocity.vy,
            self.vy_acceleration,
            vy_target,
            time_constant=cfg.vy_time_constant_sec,
            maximum_value=cfg.max_vy_mps,
            maximum_acceleration=cfg.max_vy_acceleration_mps2,
            maximum_jerk=cfg.max_vy_jerk_mps3,
            resistance=cfg.rolling_resistance_acceleration_mps2,
            stop_threshold=cfg.stop_speed_threshold_mps,
            dt=dt,
        )
        yaw_rate, self.yaw_acceleration = self._step_axis(
            self.velocity.yaw_rate,
            self.yaw_acceleration,
            yaw_target,
            time_constant=cfg.yaw_rate_time_constant_sec,
            maximum_value=cfg.max_yaw_rate_radps,
            maximum_acceleration=cfg.max_yaw_acceleration_radps2,
            maximum_jerk=cfg.max_yaw_jerk_radps3,
            resistance=0.0,
            stop_threshold=cfg.stop_speed_threshold_mps,
            dt=dt,
        )
        self.velocity = BodyVelocity(vx, vy, yaw_rate)
        return self.velocity
