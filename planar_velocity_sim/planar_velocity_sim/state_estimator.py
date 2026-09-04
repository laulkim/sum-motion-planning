from __future__ import annotations

import math
import random
from collections import deque
from dataclasses import dataclass

from .kinematics import wrap_angle


@dataclass(frozen=True)
class PlanarState:
    stamp_sec: float
    x: float
    y: float
    yaw: float
    vx: float
    vy: float
    yaw_rate: float


@dataclass(frozen=True)
class StateEstimatorConfig:
    enabled: bool = False
    delay_sec: float = 0.0
    dropout_probability: float = 0.0
    position_noise_std_m: float = 0.0
    yaw_noise_std_rad: float = 0.0
    velocity_noise_std_mps: float = 0.0
    yaw_rate_noise_std_radps: float = 0.0
    position_bias_std_m: float = 0.0
    yaw_bias_std_rad: float = 0.0
    velocity_bias_std_mps: float = 0.0
    yaw_rate_bias_std_radps: float = 0.0
    bias_correlation_time_sec: float = 30.0
    position_resolution_m: float = 0.0
    yaw_resolution_rad: float = 0.0
    velocity_resolution_mps: float = 0.0
    yaw_rate_resolution_radps: float = 0.0
    random_seed: int = 42

    def __post_init__(self) -> None:
        non_negative = {
            "delay_sec": self.delay_sec,
            "position_noise_std_m": self.position_noise_std_m,
            "yaw_noise_std_rad": self.yaw_noise_std_rad,
            "velocity_noise_std_mps": self.velocity_noise_std_mps,
            "yaw_rate_noise_std_radps": self.yaw_rate_noise_std_radps,
            "position_bias_std_m": self.position_bias_std_m,
            "yaw_bias_std_rad": self.yaw_bias_std_rad,
            "velocity_bias_std_mps": self.velocity_bias_std_mps,
            "yaw_rate_bias_std_radps": self.yaw_rate_bias_std_radps,
            "bias_correlation_time_sec": self.bias_correlation_time_sec,
            "position_resolution_m": self.position_resolution_m,
            "yaw_resolution_rad": self.yaw_resolution_rad,
            "velocity_resolution_mps": self.velocity_resolution_mps,
            "yaw_rate_resolution_radps": self.yaw_rate_resolution_radps,
        }
        for name, value in non_negative.items():
            if not math.isfinite(value) or value < 0.0:
                raise ValueError(f"{name} must be finite and non-negative")
        if not 0.0 <= self.dropout_probability <= 1.0:
            raise ValueError("dropout_probability must be in [0, 1]")


@dataclass(frozen=True)
class StateEstimate:
    state: PlanarState
    pose_covariance: tuple[float, ...]
    twist_covariance: tuple[float, ...]


def _quantize(value: float, resolution: float) -> float:
    if resolution <= 0.0:
        return value
    return round(value / resolution) * resolution


class StateEstimatorModel:
    """Delayed, biased, noisy localization output derived from true state."""

    def __init__(self, config: StateEstimatorConfig) -> None:
        self.config = config
        self._random = random.Random(config.random_seed)
        self._history: deque[PlanarState] = deque()
        self._last_update_sec: float | None = None
        self._bias = [0.0] * 6

    def record_truth(self, state: PlanarState) -> None:
        if self._history and state.stamp_sec < self._history[-1].stamp_sec:
            raise ValueError("truth timestamps must be monotonic")
        self._history.append(state)
        retain_after = state.stamp_sec - max(2.0, 2.0 * self.config.delay_sec + 1.0)
        while len(self._history) >= 2 and self._history[1].stamp_sec < retain_after:
            self._history.popleft()

    @staticmethod
    def _interpolate(first: PlanarState, second: PlanarState, stamp: float) -> PlanarState:
        duration = second.stamp_sec - first.stamp_sec
        ratio = 0.0 if duration <= 0.0 else (stamp - first.stamp_sec) / duration
        ratio = max(0.0, min(1.0, ratio))

        def blend(a: float, b: float) -> float:
            return a + ratio * (b - a)

        yaw_difference = wrap_angle(second.yaw - first.yaw)
        return PlanarState(
            stamp_sec=stamp,
            x=blend(first.x, second.x),
            y=blend(first.y, second.y),
            yaw=wrap_angle(first.yaw + ratio * yaw_difference),
            vx=blend(first.vx, second.vx),
            vy=blend(first.vy, second.vy),
            yaw_rate=blend(first.yaw_rate, second.yaw_rate),
        )

    def _truth_at(self, stamp_sec: float) -> PlanarState | None:
        if not self._history:
            return None
        if stamp_sec <= self._history[0].stamp_sec:
            return self._history[0]
        if stamp_sec >= self._history[-1].stamp_sec:
            return self._history[-1]
        for index in range(1, len(self._history)):
            second = self._history[index]
            if second.stamp_sec >= stamp_sec:
                return self._interpolate(self._history[index - 1], second, stamp_sec)
        return self._history[-1]

    def _update_biases(self, now_sec: float) -> None:
        if self._last_update_sec is None:
            dt = 0.0
        else:
            dt = max(0.0, now_sec - self._last_update_sec)
        self._last_update_sec = now_sec
        if dt == 0.0:
            return
        correlation_time = self.config.bias_correlation_time_sec
        if correlation_time > 0.0:
            decay = math.exp(-dt / correlation_time)
            random_scale = math.sqrt(max(0.0, 1.0 - decay * decay))
        else:
            decay = 0.0
            random_scale = 1.0
        standard_deviations = (
            self.config.position_bias_std_m,
            self.config.position_bias_std_m,
            self.config.yaw_bias_std_rad,
            self.config.velocity_bias_std_mps,
            self.config.velocity_bias_std_mps,
            self.config.yaw_rate_bias_std_radps,
        )
        for index, standard_deviation in enumerate(standard_deviations):
            self._bias[index] = (
                decay * self._bias[index]
                + standard_deviation * random_scale * self._random.gauss(0.0, 1.0)
            )

    @staticmethod
    def _covariance(
        x_variance: float, y_variance: float, yaw_variance: float
    ) -> tuple[float, ...]:
        covariance = [0.0] * 36
        covariance[0] = x_variance
        covariance[7] = y_variance
        covariance[14] = 1.0e6
        covariance[21] = 1.0e6
        covariance[28] = 1.0e6
        covariance[35] = yaw_variance
        return tuple(covariance)

    def estimate(self, now_sec: float) -> StateEstimate | None:
        now_sec = float(now_sec)
        truth = self._truth_at(now_sec - self.config.delay_sec)
        if truth is None:
            return None

        if not self.config.enabled:
            return StateEstimate(
                state=truth,
                pose_covariance=tuple([0.0] * 36),
                twist_covariance=tuple([0.0] * 36),
            )

        self._update_biases(now_sec)
        if self._random.random() < self.config.dropout_probability:
            return None

        cfg = self.config
        state = PlanarState(
            stamp_sec=truth.stamp_sec,
            x=_quantize(
                truth.x + self._bias[0] + self._random.gauss(0.0, cfg.position_noise_std_m),
                cfg.position_resolution_m,
            ),
            y=_quantize(
                truth.y + self._bias[1] + self._random.gauss(0.0, cfg.position_noise_std_m),
                cfg.position_resolution_m,
            ),
            yaw=wrap_angle(
                _quantize(
                    truth.yaw + self._bias[2] + self._random.gauss(0.0, cfg.yaw_noise_std_rad),
                    cfg.yaw_resolution_rad,
                )
            ),
            vx=_quantize(
                truth.vx + self._bias[3] + self._random.gauss(0.0, cfg.velocity_noise_std_mps),
                cfg.velocity_resolution_mps,
            ),
            vy=_quantize(
                truth.vy + self._bias[4] + self._random.gauss(0.0, cfg.velocity_noise_std_mps),
                cfg.velocity_resolution_mps,
            ),
            yaw_rate=_quantize(
                truth.yaw_rate
                + self._bias[5]
                + self._random.gauss(0.0, cfg.yaw_rate_noise_std_radps),
                cfg.yaw_rate_resolution_radps,
            ),
        )
        position_variance = cfg.position_noise_std_m**2 + cfg.position_bias_std_m**2
        yaw_variance = cfg.yaw_noise_std_rad**2 + cfg.yaw_bias_std_rad**2
        velocity_variance = cfg.velocity_noise_std_mps**2 + cfg.velocity_bias_std_mps**2
        yaw_rate_variance = (
            cfg.yaw_rate_noise_std_radps**2 + cfg.yaw_rate_bias_std_radps**2
        )
        return StateEstimate(
            state=state,
            pose_covariance=self._covariance(
                position_variance, position_variance, yaw_variance
            ),
            twist_covariance=self._covariance(
                velocity_variance, velocity_variance, yaw_rate_variance
            ),
        )
