from __future__ import annotations

import csv
import math
from dataclasses import dataclass, replace
from pathlib import Path

import numpy as np

from .models import DriveMode, MODE_BETA_CENTER
from .path_geometry import PathProjection, project_closed_path, project_open_path, wrap_angle


VALID_MODES = {0, 1, 2, 3}


def motion_yaw_from_map_yaw(map_yaw, mode) -> np.ndarray:
    """Convert one-direction map heading to motion direction using drive mode."""
    map_yaw = np.asarray(map_yaw, dtype=float)
    mode = np.asarray(mode, dtype=np.uint8)
    if len(map_yaw) != len(mode):
        raise ValueError("map_yaw and mode must have equal length")
    offsets = np.asarray(
        [MODE_BETA_CENTER[DriveMode(int(value))] for value in mode],
        dtype=float,
    )
    return np.unwrap(map_yaw + offsets)


@dataclass(frozen=True)
class ScenarioPath:
    x: np.ndarray
    y: np.ndarray
    yaw: np.ndarray
    kappa: np.ndarray
    mode: np.ndarray
    s: np.ndarray
    segment_length: np.ndarray
    total_length: float
    map_yaw: np.ndarray
    heading_semantics: str
    closed_loop: bool = False

    @classmethod
    def from_arrays(
        cls,
        x,
        y,
        yaw,
        kappa,
        mode,
        *,
        map_yaw=None,
        heading_semantics: str = "MOTION_DIRECTION",
        closed_loop: bool = False,
        tangent_tolerance_deg: float = 5.0,
        curvature_limit: float = 0.2,
        yaw_step_limit_deg: float = 15.0,
    ) -> "ScenarioPath":
        x = np.asarray(x, dtype=float)
        y = np.asarray(y, dtype=float)
        yaw = np.unwrap(np.asarray(yaw, dtype=float))
        kappa = np.asarray(kappa, dtype=float)
        mode = np.asarray(mode, dtype=np.uint8)
        if map_yaw is None:
            map_yaw = yaw.copy()
        map_yaw = np.unwrap(np.asarray(map_yaw, dtype=float))
        count = len(x)
        if count < 4:
            raise ValueError("Scenario path requires at least four points")
        if not (
            len(y) == len(yaw) == len(kappa) == len(mode) == len(map_yaw) == count
        ):
            raise ValueError("Scenario-path arrays must have equal length")
        if not np.all(np.isfinite(np.c_[x, y, yaw, kappa, map_yaw])):
            raise ValueError("Scenario path contains NaN or Inf")
        if not set(mode.astype(int)).issubset(VALID_MODES):
            raise ValueError("Scenario path contains an unsupported drive mode")
        if np.max(np.abs(kappa)) > curvature_limit + 1.0e-9:
            raise ValueError("Scenario path exceeds the curvature limit")

        if closed_loop:
            next_x = np.roll(x, -1)
            next_y = np.roll(y, -1)
            segment_length = np.hypot(next_x - x, next_y - y)
            chord_yaw = np.arctan2(next_y - y, next_x - x)
            yaw_step = np.asarray(wrap_angle(np.roll(yaw, -1) - yaw), dtype=float)
            s = np.r_[0.0, np.cumsum(segment_length[:-1])]
        else:
            dx = np.diff(x)
            dy = np.diff(y)
            segment_length = np.hypot(dx, dy)
            chord_yaw = np.arctan2(dy, dx)
            yaw_step = np.diff(yaw)
            s = np.r_[0.0, np.cumsum(segment_length)]

        if np.any(segment_length <= 1.0e-4):
            raise ValueError("Scenario path contains a zero-length segment")
        if np.max(np.abs(yaw_step)) > math.radians(yaw_step_limit_deg):
            raise ValueError("Scenario path contains a heading discontinuity")

        if closed_loop:
            motion_tangent_yaw = yaw
        else:
            motion_tangent_yaw = 0.5 * (yaw[:-1] + yaw[1:])
        tangent_error = np.asarray(
            wrap_angle(chord_yaw - motion_tangent_yaw), dtype=float
        )
        if np.max(np.abs(tangent_error)) > math.radians(tangent_tolerance_deg):
            raise ValueError(
                "Scenario path motion yaw is inconsistent with x-y geometry: "
                f"{math.degrees(float(np.max(np.abs(tangent_error)))):.2f} deg"
            )

        total_length = float(np.sum(segment_length))
        return cls(
            x=x.copy(),
            y=y.copy(),
            yaw=yaw.copy(),
            kappa=kappa.copy(),
            mode=mode.copy(),
            s=s,
            segment_length=segment_length,
            total_length=total_length,
            map_yaw=map_yaw.copy(),
            heading_semantics=str(heading_semantics),
            closed_loop=bool(closed_loop),
        )

    @classmethod
    def load_csv(
        cls,
        path: Path | str,
        *,
        closed_loop: bool = False,
        mode_aware_heading: bool = False,
    ) -> "ScenarioPath":
        map_path = Path(path).expanduser()
        if not map_path.is_file():
            raise FileNotFoundError(f"Scenario path not found: {map_path}")
        rows: list[tuple[float, float, float, float, int]] = []
        with map_path.open("r", newline="", encoding="utf-8") as file:
            reader = csv.DictReader(file)
            if reader.fieldnames != ["x", "y", "yaw", "kappa", "mode"]:
                raise ValueError("CSV header must be exactly: x,y,yaw,kappa,mode")
            for line_number, row in enumerate(reader, start=2):
                try:
                    rows.append(
                        (
                            float(row["x"]),
                            float(row["y"]),
                            float(row["yaw"]),
                            float(row["kappa"]),
                            int(row["mode"]),
                        )
                    )
                except (TypeError, ValueError) as exc:
                    raise ValueError(
                        f"Invalid scenario path at line {line_number}"
                    ) from exc
        array = np.asarray(rows, dtype=float)
        map_yaw = array[:, 2]
        mode = array[:, 4].astype(np.uint8)
        if mode_aware_heading:
            motion_yaw = motion_yaw_from_map_yaw(map_yaw, mode)
            semantics = "MAP_HEADING_PLUS_MODE_OFFSET"
        else:
            motion_yaw = map_yaw
            semantics = "MOTION_DIRECTION"
        return cls.from_arrays(
            array[:, 0],
            array[:, 1],
            motion_yaw,
            array[:, 3],
            mode,
            map_yaw=map_yaw,
            heading_semantics=semantics,
            closed_loop=closed_loop,
        )

    def project(
        self,
        x: float,
        y: float,
        *,
        previous_segment: int | None = None,
        search_back: int = 20,
        search_forward: int = 80,
        fallback_distance: float = 3.0,
    ) -> PathProjection:
        common = dict(
            x=self.x,
            y=self.y,
            yaw=self.yaw,
            kappa=self.kappa,
            s=self.s,
            segment_length=self.segment_length,
            previous_segment=previous_segment,
            search_back=search_back,
            search_forward=search_forward,
            fallback_distance=fallback_distance,
        )
        if self.closed_loop:
            return project_closed_path(
                x,
                y,
                total_length=self.total_length,
                **common,
            )
        return project_open_path(x, y, **common)

    def map_yaw_at(self, projection: PathProjection) -> float:
        index = int(projection.segment_index)
        next_index = (index + 1) % len(self.map_yaw)
        delta = float(wrap_angle(self.map_yaw[next_index] - self.map_yaw[index]))
        return float(wrap_angle(self.map_yaw[index] + projection.t * delta))

    def local_slice(
        self,
        projection_s: float,
        back_length: float,
        ahead_length: float,
    ) -> "ScenarioPath":
        if back_length < 0.0 or ahead_length <= 0.0:
            raise ValueError("Invalid local path length")
        if self.closed_loop:
            reference_index = int(
                np.searchsorted(self.s, projection_s, side="left")
            ) % len(self.x)
            start = reference_index
            accumulated = 0.0
            while accumulated < back_length:
                previous = (start - 1) % len(self.x)
                accumulated += float(self.segment_length[previous])
                start = previous
                if start == reference_index:
                    break
            indices = [start]
            accumulated = 0.0
            current = start
            required = back_length + ahead_length
            for _ in range(len(self.x) - 1):
                accumulated += float(self.segment_length[current])
                current = (current + 1) % len(self.x)
                indices.append(current)
                if accumulated >= required and len(indices) >= 4:
                    break
            idx = np.asarray(indices, dtype=int)
        else:
            start_s = max(0.0, float(projection_s) - float(back_length))
            end_s = min(
                self.total_length, float(projection_s) + float(ahead_length)
            )
            start = max(
                0, int(np.searchsorted(self.s, start_s, side="right") - 1)
            )
            stop = min(
                len(self.x), int(np.searchsorted(self.s, end_s, side="left") + 1)
            )
            if stop - start < 4:
                if start == 0:
                    stop = min(len(self.x), 4)
                else:
                    start = max(0, stop - 4)
            idx = np.arange(start, stop, dtype=int)

        return ScenarioPath.from_arrays(
            self.x[idx],
            self.y[idx],
            self.yaw[idx],
            self.kappa[idx],
            self.mode[idx],
            map_yaw=self.map_yaw[idx],
            heading_semantics=self.heading_semantics,
            closed_loop=False,
        )


    def clipped(self, end_s: float) -> "ScenarioPath":
        """Return an open path ending exactly at ``end_s``.

        The terminal point is interpolated and becomes the real end of the
        reference supplied to the planner.  This lets the planner perform
        position-aware braking for phase changes instead of receiving a late
        zero-speed request.
        """
        if self.closed_loop:
            raise ValueError("Closed-loop paths cannot be terminal-clipped")
        end_s = float(np.clip(end_s, 0.0, self.total_length))
        if end_s >= self.total_length - 1.0e-9:
            return self
        upper = int(np.searchsorted(self.s, end_s, side="right"))
        upper = max(1, min(upper, len(self.s) - 1))
        lower = upper - 1
        ds = float(self.s[upper] - self.s[lower])
        alpha = 0.0 if ds <= 1.0e-12 else (end_s - float(self.s[lower])) / ds
        alpha = float(np.clip(alpha, 0.0, 1.0))

        x_end = float((1.0 - alpha) * self.x[lower] + alpha * self.x[upper])
        y_end = float((1.0 - alpha) * self.y[lower] + alpha * self.y[upper])
        yaw_end = float(self.yaw[lower] + alpha * wrap_angle(self.yaw[upper] - self.yaw[lower]))
        map_yaw_end = float(self.map_yaw[lower] + alpha * wrap_angle(self.map_yaw[upper] - self.map_yaw[lower]))
        kappa_end = float((1.0 - alpha) * self.kappa[lower] + alpha * self.kappa[upper])
        mode_end = int(self.mode[lower])

        exact_existing = abs(end_s - float(self.s[lower])) <= 1.0e-9
        count = lower + 1 if exact_existing else lower + 2
        if count < 4:
            count = min(4, len(self.x))
            if end_s < float(self.s[count - 1]) - 1.0e-9:
                raise ValueError("Terminal clipping leaves fewer than four path points")

        if exact_existing:
            idx = slice(0, lower + 1)
            return ScenarioPath.from_arrays(
                self.x[idx], self.y[idx], self.yaw[idx], self.kappa[idx], self.mode[idx],
                map_yaw=self.map_yaw[idx], heading_semantics=self.heading_semantics,
                closed_loop=False,
            )

        return ScenarioPath.from_arrays(
            np.r_[self.x[:upper], x_end],
            np.r_[self.y[:upper], y_end],
            np.r_[self.yaw[:upper], yaw_end],
            np.r_[self.kappa[:upper], kappa_end],
            np.r_[self.mode[:upper], np.uint8(mode_end)],
            map_yaw=np.r_[self.map_yaw[:upper], map_yaw_end],
            heading_semantics=self.heading_semantics,
            closed_loop=False,
        )

    def translated(self, dx: float, dy: float) -> "ScenarioPath":
        if not math.isfinite(dx) or not math.isfinite(dy):
            raise ValueError("Translation must be finite")
        return replace(
            self,
            x=self.x + float(dx),
            y=self.y + float(dy),
        )

    @property
    def end_xy(self) -> tuple[float, float]:
        return float(self.x[-1]), float(self.y[-1])
