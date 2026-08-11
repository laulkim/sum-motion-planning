from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
import numpy as np


class DriveMode(IntEnum):
    """Discrete vehicle motion mode supplied by the supervisory layer."""

    FORWARD = 0
    REVERSE = 1
    LEFT = 2
    RIGHT = 3


MODE_LABEL = {
    DriveMode.FORWARD: "FORWARD",
    DriveMode.REVERSE: "REVERSE",
    DriveMode.LEFT: "LEFT",
    DriveMode.RIGHT: "RIGHT",
}

MODE_BETA_CENTER = {
    DriveMode.FORWARD: 0.0,
    DriveMode.REVERSE: np.pi,
    DriveMode.LEFT: 0.5 * np.pi,
    DriveMode.RIGHT: -0.5 * np.pi,
}


@dataclass(frozen=True)
class PlanningCommand:
    """Command from the supervisory mode manager to Planner V11."""

    target_speed: float
    drive_mode: DriveMode = DriveMode.FORWARD

    def __post_init__(self) -> None:
        if not np.isfinite(self.target_speed):
            raise ValueError("target_speed must be finite")
        if self.target_speed < 0.0:
            raise ValueError("target_speed must be a non-negative speed magnitude")
        object.__setattr__(self, "drive_mode", DriveMode(int(self.drive_mode)))


@dataclass
class PlannerMotionTrajectory:
    """Planner output interface consumed by the Motion-Body Allocator."""

    name: str
    description: str
    t: np.ndarray
    x: np.ndarray
    y: np.ndarray
    chi: np.ndarray
    kappa: np.ndarray
    kappa_s: np.ndarray
    v: np.ndarray
    a: np.ndarray
    longitudinal_jerk: np.ndarray
    motion_heading_rate: np.ndarray
    motion_heading_acceleration: np.ndarray
    drive_mode: np.ndarray

    def __post_init__(self) -> None:
        self.t = np.asarray(self.t, dtype=float)
        self.x = np.asarray(self.x, dtype=float)
        self.y = np.asarray(self.y, dtype=float)
        self.chi = np.asarray(self.chi, dtype=float)
        self.kappa = np.asarray(self.kappa, dtype=float)
        self.kappa_s = np.asarray(self.kappa_s, dtype=float)
        self.v = np.asarray(self.v, dtype=float)
        self.a = np.asarray(self.a, dtype=float)
        self.longitudinal_jerk = np.asarray(self.longitudinal_jerk, dtype=float)
        self.motion_heading_rate = np.asarray(self.motion_heading_rate, dtype=float)
        self.motion_heading_acceleration = np.asarray(self.motion_heading_acceleration, dtype=float)
        self.drive_mode = np.asarray(self.drive_mode, dtype=int)
        n = len(self.t)
        arrays = [self.x, self.y, self.chi, self.kappa, self.kappa_s, self.v, self.a, self.longitudinal_jerk, self.motion_heading_rate, self.motion_heading_acceleration, self.drive_mode]
        if n < 2 or any(len(arr) != n for arr in arrays):
            raise ValueError("All PlannerMotionTrajectory arrays must have equal length >= 2")
        if np.any(np.diff(self.t) <= 0.0):
            raise ValueError("Trajectory time must be strictly increasing")
        if np.any(self.v < -1.0e-9):
            raise ValueError("v must be a non-negative speed magnitude")
        if not all(np.all(np.isfinite(arr)) for arr in arrays[:-1]):
            raise ValueError("Trajectory contains NaN or Inf")
        valid_modes = {int(m) for m in DriveMode}
        if any(int(m) not in valid_modes for m in self.drive_mode):
            raise ValueError("Unsupported drive mode")

    @property
    def dt(self) -> float:
        return float(np.median(np.diff(self.t)))


@dataclass
class AllocatorInitialState:
    """State used to keep allocation continuous across replans."""

    beta: float
    beta_rate: float = 0.0
    yaw_rate: float = 0.0
    yaw_accel: float = 0.0
