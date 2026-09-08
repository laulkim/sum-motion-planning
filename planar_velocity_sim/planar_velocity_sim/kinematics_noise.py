from __future__ import annotations

import math
import random
from collections import deque
from typing import NamedTuple

from .kinematics import integrate_body_velocity


# Vehicle command-response parameters from the supplied model.
SECOND_ORDER_TIME_CONSTANT_SEC = 0.24
SECOND_ORDER_GAIN = 1.4
STEERING_TIME_CONSTANT_SEC = 0.106
STEERING_DELAY_SEC = 0.13
WHEELBASE_M = 2.120
TRACK_WIDTH_M = 1.340
DRIVE_YAW_WEIGHT = TRACK_WIDTH_M**2 / (WHEELBASE_M**2 + TRACK_WIDTH_M**2)
STEERING_YAW_WEIGHT = WHEELBASE_M**2 / (WHEELBASE_M**2 + TRACK_WIDTH_M**2)

# Fused-state sensor parameters. Each row is delay, correlation, error sigma,
# and bias random-walk sigma for vx, vy, and yaw-rate respectively.
SENSOR_REFERENCE_DT = 0.02
SENSOR_PARAMETERS = (
    (0.06, 0.97, 0.015, 0.00008),
    (0.06, 0.965, 0.020, 0.00010),
    (0.04, 0.94, 0.0012, 0.00001),
)


class KinematicsState(NamedTuple):
    x: float
    y: float
    yaw: float
    vx: float
    vy: float
    yaw_rate: float
    beta: float


def _delayed_sample(
    history: deque[tuple[float, float, float, float]],
    target_time: float,
) -> tuple[float, float, float]:
    """Return the latest sample at or before target_time (zero-order hold)."""
    for stamp, vx, vy, yaw_rate in reversed(history):
        if stamp <= target_time + 1.0e-12:
            return vx, vy, yaw_rate
    return 0.0, 0.0, 0.0


def _trim_history(
    history: deque[tuple[float, float, float, float]],
    oldest_time: float,
) -> None:
    while len(history) >= 2 and history[1][0] <= oldest_time:
        history.popleft()


class KinematicsNoiseModel:
    """Vehicle response, pose integration, and fused-state sensor error."""

    def __init__(self, random_seed: int = 42) -> None:
        self.random_seed = int(random_seed)
        self._random = random.Random(self.random_seed)
        self.reset()

    def reset(self) -> None:
        self._random.seed(self.random_seed)
        self._time = 0.0

        # Second-order states for vx and the drive yaw-rate branch.
        self._vx_z1 = 0.0
        self._vx_z2 = 0.0
        self._drive_yaw_z1 = 0.0
        self._drive_yaw_z2 = 0.0

        # First-order states for vy and the steering yaw-rate branch.
        self.actual_vx = 0.0
        self.actual_vy = 0.0
        self.drive_yaw_rate = 0.0
        self.steering_yaw_rate = 0.0
        self.actual_yaw_rate = 0.0
        self.beta = 0.0

        self._command_history: deque[tuple[float, float, float, float]] = deque()
        self._sensor_history: deque[tuple[float, float, float, float]] = deque(
            [(0.0, 0.0, 0.0, 0.0)]
        )
        self._errors = [0.0, 0.0, 0.0]
        self._biases = [0.0, 0.0, 0.0]

    @property
    def actual_speed(self) -> float:
        return math.hypot(self.actual_vx, self.actual_vy)

    @staticmethod
    def _second_order_step(
        z1: float,
        z2: float,
        command: float,
        dt: float,
    ) -> tuple[float, float, float]:
        gain_over_time = SECOND_ORDER_GAIN / SECOND_ORDER_TIME_CONSTANT_SEC
        inverse_time = 1.0 / SECOND_ORDER_TIME_CONSTANT_SEC
        z1_dot = z2
        z2_dot = -gain_over_time * z1 - inverse_time * z2 + command
        z1 += z1_dot * dt
        z2 += z2_dot * dt
        output = gain_over_time * z1 + inverse_time * z2
        return z1, z2, output

    def _advance_vehicle(
        self,
        command_vx: float,
        command_vy: float,
        command_yaw_rate: float,
        dt: float,
    ) -> None:
        self._command_history.append(
            (self._time, command_vx, command_vy, command_yaw_rate)
        )
        _, delayed_vy, delayed_yaw_rate = _delayed_sample(
            self._command_history, self._time - STEERING_DELAY_SEC
        )
        _trim_history(
            self._command_history, self._time - STEERING_DELAY_SEC
        )

        self._vx_z1, self._vx_z2, self.actual_vx = self._second_order_step(
            self._vx_z1, self._vx_z2, command_vx, dt
        )
        (
            self._drive_yaw_z1,
            self._drive_yaw_z2,
            self.drive_yaw_rate,
        ) = self._second_order_step(
            self._drive_yaw_z1,
            self._drive_yaw_z2,
            command_yaw_rate,
            dt,
        )

        first_order_fraction = 1.0 - math.exp(-dt / STEERING_TIME_CONSTANT_SEC)
        self.actual_vy += first_order_fraction * (delayed_vy - self.actual_vy)
        self.steering_yaw_rate += first_order_fraction * (
            delayed_yaw_rate - self.steering_yaw_rate
        )
        self.actual_yaw_rate = (
            DRIVE_YAW_WEIGHT * self.drive_yaw_rate
            + STEERING_YAW_WEIGHT * self.steering_yaw_rate
        )
        self.beta = math.atan2(self.actual_vy, self.actual_vx)

    def _advance_sensor_error(self, dt: float) -> None:
        sample_ratio = dt / SENSOR_REFERENCE_DT
        for index, (_, correlation, error_sigma, bias_sigma) in enumerate(
            SENSOR_PARAMETERS
        ):
            correlation_dt = correlation**sample_ratio
            innovation_sigma = error_sigma * math.sqrt(
                max(0.0, 1.0 - correlation_dt**2)
            )
            self._errors[index] = (
                correlation_dt * self._errors[index]
                + innovation_sigma * self._random.gauss(0.0, 1.0)
            )
            self._biases[index] += (
                bias_sigma
                * math.sqrt(sample_ratio)
                * self._random.gauss(0.0, 1.0)
            )

    def _sensor_output(self) -> tuple[float, float, float]:
        delayed = [
            _delayed_sample(self._sensor_history, self._time - parameters[0])[
                index
            ]
            for index, parameters in enumerate(SENSOR_PARAMETERS)
        ]
        return tuple(
            delayed[index] + self._errors[index] + self._biases[index]
            for index in range(3)
        )

    def step(
        self,
        x: float,
        y: float,
        yaw: float,
        command_vx: float,
        command_vy: float,
        command_yaw_rate: float,
        dt: float = 0.01,
    ) -> KinematicsState:
        dt = float(dt)
        if not math.isfinite(dt) or dt <= 0.0:
            raise ValueError("dt must be finite and positive")

        previous_vx = self.actual_vx
        previous_vy = self.actual_vy
        previous_yaw_rate = self.actual_yaw_rate
        self._advance_vehicle(
            float(command_vx),
            float(command_vy),
            float(command_yaw_rate),
            dt,
        )

        next_x, next_y, next_yaw = integrate_body_velocity(
            x,
            y,
            yaw,
            0.5 * (previous_vx + self.actual_vx),
            0.5 * (previous_vy + self.actual_vy),
            0.5 * (previous_yaw_rate + self.actual_yaw_rate),
            dt,
        )

        self._time += dt
        self._sensor_history.append(
            (self._time, self.actual_vx, self.actual_vy, self.actual_yaw_rate)
        )
        self._advance_sensor_error(dt)
        estimated_vx, estimated_vy, estimated_yaw_rate = self._sensor_output()
        longest_sensor_delay = max(row[0] for row in SENSOR_PARAMETERS)
        _trim_history(self._sensor_history, self._time - longest_sensor_delay)

        return KinematicsState(
            next_x,
            next_y,
            next_yaw,
            estimated_vx,
            estimated_vy,
            estimated_yaw_rate,
            self.beta,
        )
