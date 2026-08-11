from __future__ import annotations

import csv
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Callable

import numpy as np


VALID_MODES = {0, 1, 2, 3}


def wrap_angle(angle):
    return (np.asarray(angle) + np.pi) % (2.0 * np.pi) - np.pi


@dataclass(frozen=True)
class TrackMap:
    x: np.ndarray
    y: np.ndarray
    yaw: np.ndarray
    kappa: np.ndarray
    mode: np.ndarray
    s: np.ndarray
    segment_length: np.ndarray
    total_length: float

    @classmethod
    def from_arrays(
        cls,
        x,
        y,
        yaw,
        kappa,
        mode,
        *,
        tangent_tolerance_deg: float = 5.0,
        curvature_limit: float = 0.2,
        yaw_step_limit_deg: float = 15.0,
    ) -> "TrackMap":
        x = np.asarray(x, dtype=float)
        y = np.asarray(y, dtype=float)
        yaw = np.asarray(yaw, dtype=float)
        kappa = np.asarray(kappa, dtype=float)
        mode = np.asarray(mode, dtype=np.uint8)

        length = len(x)
        if length < 8:
            raise ValueError("Track map requires at least eight points")
        if not (len(y) == len(yaw) == len(kappa) == len(mode) == length):
            raise ValueError("Track-map arrays must have equal length")
        if not np.all(np.isfinite(np.c_[x, y, yaw, kappa])):
            raise ValueError("Track map contains NaN or Inf")
        if not set(mode.astype(int)).issubset(VALID_MODES):
            raise ValueError("Track map contains an unsupported drive mode")
        if np.max(np.abs(kappa)) > float(curvature_limit) + 1.0e-9:
            raise ValueError(
                f"Track curvature exceeds {float(curvature_limit):.3f} 1/m"
            )

        next_x = np.roll(x, -1)
        next_y = np.roll(y, -1)
        segment_length = np.hypot(next_x - x, next_y - y)
        if np.any(segment_length <= 1.0e-4):
            raise ValueError("Track map contains duplicate or near-duplicate points")

        continuous_yaw = np.unwrap(yaw)
        yaw_step = wrap_angle(np.roll(yaw, -1) - yaw)
        if np.max(np.abs(yaw_step)) > math.radians(yaw_step_limit_deg):
            raise ValueError(
                f"Adjacent map yaw change exceeds {yaw_step_limit_deg:.1f} deg"
            )

        chord_yaw = np.arctan2(next_y - y, next_x - x)
        outgoing_error = wrap_angle(chord_yaw - yaw)
        if np.max(np.abs(outgoing_error)) > math.radians(tangent_tolerance_deg):
            raise ValueError(
                "Map yaw is inconsistent with x-y geometry: "
                f"max error={math.degrees(float(np.max(np.abs(outgoing_error)))):.2f} deg"
            )

        s = np.r_[0.0, np.cumsum(segment_length[:-1])]
        total_length = float(np.sum(segment_length))
        if total_length <= 1.0:
            raise ValueError("Track-map length is too short")

        return cls(
            x=x.copy(),
            y=y.copy(),
            yaw=continuous_yaw,
            kappa=kappa.copy(),
            mode=mode.copy(),
            s=s,
            segment_length=segment_length,
            total_length=total_length,
        )

    @classmethod
    def load_csv(cls, path: Path | str) -> "TrackMap":
        map_path = Path(path).expanduser()
        if not map_path.is_file():
            raise FileNotFoundError(f"Track map not found: {map_path}")

        rows: list[tuple[float, float, float, float, int]] = []
        with map_path.open("r", newline="", encoding="utf-8") as file:
            reader = csv.DictReader(file)
            expected = ["x", "y", "yaw", "kappa", "mode"]
            if reader.fieldnames != expected:
                raise ValueError("CSV header must be exactly: x,y,yaw,kappa,mode")

            for line_number, row in enumerate(reader, start=2):
                try:
                    values = (
                        float(row["x"]),
                        float(row["y"]),
                        float(row["yaw"]),
                        float(row["kappa"]),
                        int(row["mode"]),
                    )
                except (TypeError, ValueError) as exc:
                    raise ValueError(
                        f"Invalid map value at CSV line {line_number}"
                    ) from exc
                rows.append(values)

        array = np.asarray(rows, dtype=float)
        return cls.from_arrays(
            array[:, 0],
            array[:, 1],
            array[:, 2],
            array[:, 3],
            array[:, 4].astype(np.uint8),
        )

    def save_csv(self, path: Path | str) -> None:
        map_path = Path(path)
        map_path.parent.mkdir(parents=True, exist_ok=True)
        with map_path.open("w", newline="", encoding="utf-8") as file:
            writer = csv.writer(file)
            writer.writerow(["x", "y", "yaw", "kappa", "mode"])
            for x, y, yaw, kappa, mode in zip(
                self.x, self.y, self.yaw, self.kappa, self.mode
            ):
                writer.writerow(
                    [
                        f"{float(x):.9f}",
                        f"{float(y):.9f}",
                        f"{float(wrap_angle(yaw)):.12f}",
                        f"{float(kappa):.12f}",
                        int(mode),
                    ]
                )


def generate_clothoid_stadium(
    *,
    straight_length: float = 40.0,
    radius: float = 10.0,
    clothoid_length: float = 6.0,
    ds: float = 0.2,
    mode: int = 0,
) -> TrackMap:
    if straight_length <= 0.0:
        raise ValueError("straight_length must be positive")
    if radius <= 0.0:
        raise ValueError("radius must be positive")
    if clothoid_length <= 0.0:
        raise ValueError("clothoid_length must be positive")
    if ds <= 0.0:
        raise ValueError("ds must be positive")
    if mode not in VALID_MODES:
        raise ValueError("Unsupported mode")

    maximum_curvature = 1.0 / float(radius)
    circular_arc_length = math.pi * float(radius) - float(clothoid_length)
    if circular_arc_length <= 0.0:
        raise ValueError("clothoid_length must be smaller than pi*radius")

    state = np.asarray([0.0, 0.0, 0.0], dtype=float)
    samples: list[tuple[float, float, float, float, int]] = [
        (0.0, 0.0, 0.0, 0.0, mode)
    ]

    def integrate(length: float, curvature: Callable[[float], float]) -> None:
        nonlocal state
        count = max(1, int(math.ceil(float(length) / float(ds))))
        step = float(length) / count
        for index in range(count):
            midpoint_s = (index + 0.5) * step
            midpoint_curvature = float(curvature(midpoint_s))
            midpoint_yaw = float(state[2] + 0.5 * midpoint_curvature * step)
            state[0] += math.cos(midpoint_yaw) * step
            state[1] += math.sin(midpoint_yaw) * step
            state[2] += midpoint_curvature * step
            endpoint_s = (index + 1) * step
            samples.append(
                (
                    float(state[0]),
                    float(state[1]),
                    float(state[2]),
                    float(curvature(endpoint_s)),
                    mode,
                )
            )

    straight = lambda _: 0.0
    ramp_up = lambda s: maximum_curvature * s / clothoid_length
    constant = lambda _: maximum_curvature
    ramp_down = lambda s: maximum_curvature * (1.0 - s / clothoid_length)

    segment_definition = (
        (straight_length, straight),
        (clothoid_length, ramp_up),
        (circular_arc_length, constant),
        (clothoid_length, ramp_down),
        (straight_length, straight),
        (clothoid_length, ramp_up),
        (circular_arc_length, constant),
        (clothoid_length, ramp_down),
    )
    for length, curvature in segment_definition:
        integrate(length, curvature)

    array = np.asarray(samples, dtype=float)
    closure_error = float(np.linalg.norm(array[-1, :2] - array[0, :2]))
    yaw_closure_error = abs(
        float(wrap_angle(array[-1, 2] - array[0, 2]))
    )
    if closure_error > 1.0e-6 or yaw_closure_error > 1.0e-8:
        raise RuntimeError(
            "Generated stadium did not close: "
            f"position={closure_error:.3e}, yaw={yaw_closure_error:.3e}"
        )

    # Remove the duplicate final point. The final map segment connects the last
    # retained point to the first point.
    array = array[:-1]
    return TrackMap.from_arrays(
        array[:, 0],
        array[:, 1],
        array[:, 2],
        array[:, 3],
        array[:, 4].astype(np.uint8),
    )
