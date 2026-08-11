from __future__ import annotations

from dataclasses import dataclass


VALID_DRIVE_MODES = (0, 1, 2, 3)


@dataclass(frozen=True)
class DriveModeFeedback:
    current_mode: int
    requested_mode: int
    transition_in_progress: bool
    transition_complete: bool


class DriveModeTransitionModel:
    """Vehicle-side mode transition model.

    A mode command is accepted only at standstill. During the transition the
    vehicle is held at zero velocity. The current mode changes only after the
    configured dwell has elapsed.
    """

    def __init__(
        self,
        *,
        initial_mode: int = 0,
        transition_duration_sec: float = 2.0,
        stop_speed_threshold: float = 0.03,
    ) -> None:
        if initial_mode not in VALID_DRIVE_MODES:
            raise ValueError("initial_mode must be in [0, 3]")
        if transition_duration_sec < 0.0:
            raise ValueError("transition_duration_sec must be non-negative")
        if stop_speed_threshold < 0.0:
            raise ValueError("stop_speed_threshold must be non-negative")
        self.current_mode = int(initial_mode)
        self.requested_mode = int(initial_mode)
        self.transition_duration_sec = float(transition_duration_sec)
        self.stop_speed_threshold = float(stop_speed_threshold)
        self.transition_in_progress = False
        self.transition_start_sec: float | None = None

    @property
    def transition_complete(self) -> bool:
        return (
            not self.transition_in_progress
            and self.current_mode == self.requested_mode
        )

    def command(self, requested_mode: int, measured_speed: float, now_sec: float) -> bool:
        requested_mode = int(requested_mode)
        if requested_mode not in VALID_DRIVE_MODES:
            raise ValueError("requested_mode must be in [0, 3]")
        if measured_speed > self.stop_speed_threshold:
            return False
        if self.transition_in_progress and requested_mode == self.requested_mode:
            return True
        self.requested_mode = requested_mode
        if requested_mode == self.current_mode:
            self.transition_in_progress = False
            self.transition_start_sec = None
            return True
        self.transition_in_progress = True
        self.transition_start_sec = float(now_sec)
        return True

    def update(self, now_sec: float) -> bool:
        if not self.transition_in_progress:
            return False
        assert self.transition_start_sec is not None
        if float(now_sec) - self.transition_start_sec + 1.0e-12 < self.transition_duration_sec:
            return False
        self.current_mode = self.requested_mode
        self.transition_in_progress = False
        self.transition_start_sec = None
        return True

    def feedback(self) -> DriveModeFeedback:
        return DriveModeFeedback(
            current_mode=self.current_mode,
            requested_mode=self.requested_mode,
            transition_in_progress=self.transition_in_progress,
            transition_complete=self.transition_complete,
        )

    def applied_velocity(
        self, vx: float, vy: float, yaw_rate: float
    ) -> tuple[float, float, float]:
        if self.transition_in_progress:
            return 0.0, 0.0, 0.0
        return float(vx), float(vy), float(yaw_rate)
