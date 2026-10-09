"""Empirical closed-loop body-velocity response, not open-loop vehicle physics.

Each reference passes through the identified lower controller + actuator +
vehicle response: tau * dy/dt + y = gain * u(t - delay).
Inputs are zero-order held from their receipt timestamps.
"""

from collections import deque
import math


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
