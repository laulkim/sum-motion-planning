from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum


VALID_DRIVE_MODES = (0, 1, 2, 3, 4)


class VehicleModeStatus(IntEnum):
    """DriveModeState.msg의 STATUS_ALIGNING(0)/STATUS_READY(1)와 1:1 대응.

    이전에는 transition_in_progress/transition_complete 두 bool로 나뉘어
    있었지만, complete는 항상 (not in_progress) and current_mode==requested_mode
    로만 계산돼서 독립된 정보가 아니었다. 필드 하나로 합친다.
    """

    ALIGNING = 0
    READY = 1


@dataclass(frozen=True)
class DriveModeFeedback:
    current_mode: int
    requested_mode: int
    status: VehicleModeStatus


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
            raise ValueError("initial_mode must be in [0, 4]")
        if transition_duration_sec < 0.0:
            raise ValueError("transition_duration_sec must be non-negative")
        if stop_speed_threshold < 0.0:
            raise ValueError("stop_speed_threshold must be non-negative")
        self.current_mode = int(initial_mode)
        self.requested_mode = int(initial_mode)
        self.transition_duration_sec = float(transition_duration_sec)
        self.stop_speed_threshold = float(stop_speed_threshold)
        self.status = VehicleModeStatus.READY
        self.transition_start_sec: float | None = None

    @property
    def transition_in_progress(self) -> bool:
        return self.status == VehicleModeStatus.ALIGNING

    def command(self, requested_mode: int, measured_speed: float, now_sec: float) -> bool:
        requested_mode = int(requested_mode)
        if requested_mode not in VALID_DRIVE_MODES:
            raise ValueError("requested_mode must be in [0, 4]")
        if measured_speed > self.stop_speed_threshold:
            return False
        if self.status == VehicleModeStatus.ALIGNING and requested_mode == self.requested_mode:
            return True
        self.requested_mode = requested_mode
        if requested_mode == self.current_mode:
            self.status = VehicleModeStatus.READY
            self.transition_start_sec = None
            return True
        self.status = VehicleModeStatus.ALIGNING
        self.transition_start_sec = float(now_sec)
        return True

    def update(self, now_sec: float) -> bool:
        if self.status == VehicleModeStatus.READY:
            return False
        assert self.transition_start_sec is not None
        if float(now_sec) - self.transition_start_sec + 1.0e-12 < self.transition_duration_sec:
            return False
        self.current_mode = self.requested_mode
        self.status = VehicleModeStatus.READY
        self.transition_start_sec = None
        return True

    def feedback(self) -> DriveModeFeedback:
        return DriveModeFeedback(
            current_mode=self.current_mode,
            requested_mode=self.requested_mode,
            status=self.status,
        )

    def applied_velocity(
        self, vx: float, vy: float, yaw_rate: float
    ) -> tuple[float, float, float]:
        if self.status == VehicleModeStatus.ALIGNING:
            return 0.0, 0.0, 0.0
        return float(vx), float(vy), float(yaw_rate)
