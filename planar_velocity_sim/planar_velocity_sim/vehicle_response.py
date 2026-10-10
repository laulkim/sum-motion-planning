"""Empirical closed-loop body-velocity response, not open-loop vehicle physics.

Each reference passes through the identified lower controller + actuator +
vehicle response: tau * dy/dt + y = gain * u(t - delay).
Inputs are zero-order held from their receipt timestamps.
"""

from collections import deque
import math
import random


# Shared by the ROS node and simulation launch; times are in seconds.
RESPONSE_PARAMETER_DEFAULTS = {
    "vx_gain": 1.00073,
    "vx_tau_sec": 0.00939,
    "vx_delay_sec": 0.020,
    "vy_gain": 0.94555,
    "vy_tau_sec": 0.02441,
    "vy_delay_sec": 0.000,
    "yaw_rate_gain": 0.93197,
    "yaw_rate_tau_sec": 0.00527,
    "yaw_rate_delay_sec": 0.020,
    "sensor_delay_sec": 0.100,
}


# Vmax3/Vmax4 residual fits, validated on Vmax5; (a1, a2, sigma_w).
# Velocity units are m/s, m/s, rad/s respectively; no intercept is added.
AR2_NOISE_DEFAULTS = {
    "vx": (0.407455642, 0.589895709, 0.00399662414),
    "vy": (0.611780077, 0.381494641, 0.00321297667),
    "yaw_rate": (0.303096072, 0.680895634, 0.00239455092),
}


class AR2Noise:
    """Zero-mean measurement noise, advanced once per published odometry sample."""

    def __init__(self, a1, a2, innovation_std, seed):
        if not all(math.isfinite(v) for v in (a1, a2, innovation_std)):
            raise ValueError("AR(2) parameters must be finite")
        if innovation_std < 0.0:
            raise ValueError("innovation_std must be non-negative")
        if abs(a2) >= 1.0 or abs(a1) >= 1.0 - a2:
            raise ValueError("AR(2) coefficients must be stationary")
        self.a1, self.a2, self.innovation_std = a1, a2, innovation_std
        self.rng = random.Random(seed)
        # Yule-Walker: rho(1) = a1/(1-a2), variance = sigma_w^2 /
        # ((1-a2^2)*(1-rho(1)^2)). Draw the correlated stationary state pair.
        rho = a1 / (1.0 - a2)
        std = innovation_std / math.sqrt((1.0 - a2 * a2) * (1.0 - rho * rho))
        self.previous1 = self.rng.gauss(0.0, std)
        self.previous2 = rho * self.previous1 + self.rng.gauss(
            0.0, std * math.sqrt(1.0 - rho * rho)
        )

    def sample(self):
        noise = (self.a1 * self.previous1 + self.a2 * self.previous2
                 + self.rng.gauss(0.0, self.innovation_std))
        self.previous2, self.previous1 = self.previous1, noise
        return noise


class FirstOrderResponse:
    """One channel with timestamped dead time and exact exponential updates."""

    def __init__(
        self, gain: float, tau_sec: float, delay_sec: float, now_sec: float
    ) -> None:
        if not all(math.isfinite(v) for v in (gain, tau_sec, delay_sec)):
            raise ValueError("response parameters must be finite")
        if tau_sec <= 0.0 or delay_sec < 0.0:
            raise ValueError("tau must be positive and delay non-negative")
        self.gain = gain
        self.tau_sec = tau_sec
        self.delay_sec = delay_sec
        self.reset(now_sec)

    def reset(self, now_sec: float) -> None:
        self.time_sec = now_sec
        self.actual = 0.0
        self.input = 0.0
        self.reference = 0.0
        self.pending = deque()

    def command(self, reference: float, now_sec: float) -> None:
        if now_sec < self.time_sec:
            raise ValueError("command timestamp precedes response state")
        if reference != self.reference:
            self.pending.append((now_sec + self.delay_sec, reference))
            self.reference = reference

    def _advance(self, now_sec: float) -> None:
        alpha = math.exp(-(now_sec - self.time_sec) / self.tau_sec)
        self.actual = alpha * self.actual + (1.0 - alpha) * self.gain * self.input
        self.time_sec = now_sec

    def update(self, now_sec: float) -> float:
        if now_sec < self.time_sec:
            raise ValueError("response time must be monotonic")
        # Split the interval at delayed input changes, including those between
        # timer ticks. A 20 ms dead time stays 20 ms at any update rate.
        while self.pending and self.pending[0][0] <= now_sec:
            event_sec, next_input = self.pending.popleft()
            self._advance(event_sec)
            self.input = next_input
        self._advance(now_sec)
        return self.actual


class SensorDelay:
    """Deliver timestamped pose/velocity samples once their latency has elapsed.

    Retains the newest eligible sample (zero-order hold); measurement timestamp
    is preserved, so sampling resolution is explicit even with irregular ticks.
    No sample is available during the initial delay window.
    """

    def __init__(self, delay_sec: float) -> None:
        if not math.isfinite(delay_sec) or delay_sec < 0.0:
            raise ValueError("sensor delay must be finite and non-negative")
        self.delay_sec = delay_sec
        self.pending = deque()
        self.latest = None

    def update(self, now_sec: float, state: tuple):
        self.pending.append((now_sec, state))
        cutoff = now_sec - self.delay_sec
        while self.pending and self.pending[0][0] <= cutoff + 1.0e-12:
            self.latest = self.pending.popleft()
        return self.latest
