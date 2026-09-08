#!/usr/bin/env python3

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


COMMAND_THRESHOLD = 1.0e-4
MOVING_SPEED_THRESHOLD = 0.2


def _numeric_column(rows: list[dict[str, str]], name: str) -> np.ndarray:
    return np.asarray(
        [float(row[name]) if row[name] else math.nan for row in rows], dtype=float
    )


def _read_rows(path: Path) -> list[dict[str, str]]:
    if not path.is_file():
        raise FileNotFoundError(path)
    with path.open(encoding="utf-8") as file:
        return list(csv.DictReader(file))


def _first_command_time(rows: list[dict[str, str]]) -> float:
    for row in rows:
        if (
            abs(float(row["vx"]))
            + abs(float(row["vy"]))
            + abs(float(row["yaw_rate"]))
            > COMMAND_THRESHOLD
        ):
            return float(row["publish_time"])
    return float(rows[0]["publish_time"])


def load_run(run_directory: Path) -> dict[str, object]:
    odometry_rows = _read_rows(run_directory / "odom_history.csv")
    command_rows = _read_rows(run_directory / "command_history.csv")
    if not odometry_rows or not command_rows:
        raise ValueError(f"Empty run data: {run_directory}")

    start_time = _first_command_time(command_rows)
    time = _numeric_column(odometry_rows, "time") - start_time
    keep = np.isfinite(time) & (time >= 0.0)
    time = time[keep]

    speed = _numeric_column(odometry_rows, "speed")[keep]
    beta = _numeric_column(odometry_rows, "beta")[keep]
    yaw = np.unwrap(_numeric_column(odometry_rows, "body_yaw")[keep])
    # vx and vy are exactly recoverable because the logger stored speed and
    # beta=atan2(vy, vx). The pose-derived yaw-rate is used because the old CSV
    # format did not store odometry twist.angular.z.
    vx = speed * np.cos(beta)
    vy = speed * np.sin(beta)
    yaw_rate = np.gradient(yaw, time)

    command_time = _numeric_column(command_rows, "publish_time") - start_time
    command_keep = np.isfinite(command_time) & (command_time >= 0.0)
    command_time = command_time[command_keep]

    return {
        "directory": str(run_directory),
        "scenario": odometry_rows[0]["scenario"],
        "time": time,
        "x": _numeric_column(odometry_rows, "x")[keep],
        "y": _numeric_column(odometry_rows, "y")[keep],
        "yaw": yaw,
        "speed": speed,
        "beta": beta,
        "vx": vx,
        "vy": vy,
        "yaw_rate": yaw_rate,
        "lateral": _numeric_column(
            odometry_rows, "global_lateral_deviation"
        )[keep],
        "direction": _numeric_column(
            odometry_rows, "global_direction_deviation"
        )[keep],
        "command_time": command_time,
        "command_vx": _numeric_column(command_rows, "vx")[command_keep],
        "command_vy": _numeric_column(command_rows, "vy")[command_keep],
        "command_yaw_rate": _numeric_column(command_rows, "yaw_rate")[
            command_keep
        ],
    }


def _interpolate(
    run: dict[str, object], time: np.ndarray
) -> dict[str, np.ndarray]:
    source_time = run["time"]
    result = {
        name: np.interp(time, source_time, run[name])
        for name in (
            "x",
            "y",
            "yaw",
            "speed",
            "beta",
            "vx",
            "vy",
            "yaw_rate",
            "lateral",
            "direction",
        )
    }
    command_time = run["command_time"]
    for name in ("vx", "vy", "yaw_rate"):
        result[f"command_{name}"] = np.interp(
            time, command_time, run[f"command_{name}"]
        )
    return result


def _wrap_angle(angle: np.ndarray) -> np.ndarray:
    return np.arctan2(np.sin(angle), np.cos(angle))


def _statistics(values: np.ndarray) -> dict[str, float]:
    finite = values[np.isfinite(values)]
    return {
        "rmse": float(np.sqrt(np.mean(finite**2))),
        "mean_abs": float(np.mean(np.abs(finite))),
        "max_abs": float(np.max(np.abs(finite))),
        "bias": float(np.mean(finite)),
    }


def _run_statistics(run: dict[str, np.ndarray]) -> dict[str, float]:
    return {
        "lateral_rmse_m": _statistics(run["lateral"])["rmse"],
        "lateral_max_abs_m": _statistics(run["lateral"])["max_abs"],
        "direction_rmse_deg": _statistics(np.degrees(run["direction"]))[
            "rmse"
        ],
        "direction_max_abs_deg": _statistics(
            np.degrees(run["direction"])
        )["max_abs"],
    }


def _write_aligned_csv(
    output_path: Path,
    time: np.ndarray,
    ideal: dict[str, np.ndarray],
    noisy: dict[str, np.ndarray],
    errors: dict[str, np.ndarray],
) -> None:
    columns = {"time": time}
    for prefix, values in (("ideal", ideal), ("noisy", noisy)):
        for name, data in values.items():
            columns[f"{prefix}_{name}"] = data
    for name, data in errors.items():
        columns[f"error_{name}"] = data

    with output_path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.writer(file)
        writer.writerow(columns)
        writer.writerows(zip(*columns.values()))


def _render_overlay(
    output_path: Path | None,
    scenario: str,
    time: np.ndarray,
    ideal: dict[str, np.ndarray],
    noisy: dict[str, np.ndarray],
    position_error: np.ndarray,
) -> None:
    figure, axes = plt.subplots(3, 2, figsize=(16, 13), constrained_layout=True)
    figure.suptitle(f"{scenario}: ideal vs noisy", fontsize=16)

    axes[0, 0].plot(ideal["x"], ideal["y"], label="ideal")
    axes[0, 0].plot(noisy["x"], noisy["y"], label="noisy", alpha=0.85)
    axes[0, 0].set_title("XY trajectory")
    axes[0, 0].set_xlabel("x [m]")
    axes[0, 0].set_ylabel("y [m]")
    axes[0, 0].axis("equal")

    axes[0, 1].plot(time, position_error, color="tab:red")
    axes[0, 1].set_title("Position separation")
    axes[0, 1].set_ylabel("m")

    for key, title, unit, axis in (
        ("speed", "Speed", "m/s", axes[1, 0]),
        ("vx", "Body vx", "m/s", axes[1, 1]),
        ("vy", "Body vy", "m/s", axes[2, 0]),
        ("yaw_rate", "Pose-derived yaw-rate", "rad/s", axes[2, 1]),
    ):
        axis.plot(time, ideal[key], label="ideal")
        axis.plot(time, noisy[key], label="noisy", alpha=0.8)
        axis.set_title(title)
        axis.set_ylabel(unit)
        axis.legend()

    for axis in axes.flat:
        axis.grid(True, alpha=0.3)
        if axis is not axes[0, 0]:
            axis.set_xlabel("time from first command [s]")
    axes[0, 0].legend()
    if output_path is not None:
        figure.savefig(output_path, dpi=160)


def _render_errors(
    output_path: Path | None,
    scenario: str,
    time: np.ndarray,
    ideal: dict[str, np.ndarray],
    noisy: dict[str, np.ndarray],
    errors: dict[str, np.ndarray],
) -> None:
    figure, axes = plt.subplots(3, 2, figsize=(16, 13), constrained_layout=True)
    figure.suptitle(f"{scenario}: noisy - ideal", fontsize=16)

    axes[0, 0].plot(time, errors["x"], label="x")
    axes[0, 0].plot(time, errors["y"], label="y")
    axes[0, 0].plot(time, errors["position"], "k", label="distance")
    axes[0, 0].set_title("Position error")
    axes[0, 0].set_ylabel("m")
    axes[0, 0].legend()

    axes[0, 1].plot(time, np.degrees(errors["yaw"]), color="tab:purple")
    axes[0, 1].set_title("Yaw error")
    axes[0, 1].set_ylabel("deg")

    axes[1, 0].plot(time, errors["vx"], label="vx")
    axes[1, 0].plot(time, errors["vy"], label="vy")
    axes[1, 0].set_title("Body velocity error")
    axes[1, 0].set_ylabel("m/s")
    axes[1, 0].legend()

    axes[1, 1].plot(time, errors["yaw_rate"], color="tab:red")
    axes[1, 1].set_title("Pose-derived yaw-rate error")
    axes[1, 1].set_ylabel("rad/s")

    moving = np.maximum(ideal["speed"], noisy["speed"]) > MOVING_SPEED_THRESHOLD
    axes[2, 0].plot(
        time[moving], np.degrees(errors["beta"][moving]), color="tab:green"
    )
    axes[2, 0].set_title("Beta error while moving")
    axes[2, 0].set_ylabel("deg")

    axes[2, 1].plot(time, errors["command_vx"], label="cmd vx")
    axes[2, 1].plot(time, errors["command_vy"], label="cmd vy")
    axes[2, 1].plot(time, errors["command_yaw_rate"], label="cmd yaw-rate")
    axes[2, 1].set_title("Closed-loop command difference")
    axes[2, 1].set_ylabel("mixed units")
    axes[2, 1].legend()

    for axis in axes.flat:
        axis.grid(True, alpha=0.3)
        axis.set_xlabel("time from first command [s]")
    if output_path is not None:
        figure.savefig(output_path, dpi=160)


def compare_runs(
    ideal_directory: Path,
    noisy_directory: Path,
    output_directory: Path | None,
    sample_period: float = 0.01,
    show: bool = False,
) -> dict[str, object]:
    if sample_period <= 0.0:
        raise ValueError("sample_period must be positive")
    ideal_raw = load_run(ideal_directory)
    noisy_raw = load_run(noisy_directory)
    if ideal_raw["scenario"] != noisy_raw["scenario"]:
        raise ValueError("Ideal and noisy runs use different scenarios")

    start_time = max(float(ideal_raw["time"][0]), float(noisy_raw["time"][0]))
    end_time = min(float(ideal_raw["time"][-1]), float(noisy_raw["time"][-1]))
    if end_time <= start_time:
        raise ValueError("Runs do not have an overlapping time range")
    time = np.arange(start_time, end_time, sample_period)
    ideal = _interpolate(ideal_raw, time)
    noisy = _interpolate(noisy_raw, time)

    errors = {
        name: noisy[name] - ideal[name]
        for name in ("x", "y", "speed", "vx", "vy", "yaw_rate")
    }
    errors["position"] = np.hypot(errors["x"], errors["y"])
    errors["yaw"] = _wrap_angle(noisy["yaw"] - ideal["yaw"])
    errors["beta"] = _wrap_angle(noisy["beta"] - ideal["beta"])
    for name in ("vx", "vy", "yaw_rate"):
        errors[f"command_{name}"] = (
            noisy[f"command_{name}"] - ideal[f"command_{name}"]
        )

    moving = np.maximum(ideal["speed"], noisy["speed"]) > MOVING_SPEED_THRESHOLD
    metrics = {
        "scenario": ideal_raw["scenario"],
        "ideal_directory": str(ideal_directory),
        "noisy_directory": str(noisy_directory),
        "alignment": "first non-zero command",
        "duration_sec": float(time[-1] - time[0]),
        "sample_period_sec": sample_period,
        "derived_fields": {
            "vx_vy": "speed and beta",
            "yaw_rate": "numerical derivative of body_yaw",
        },
        "ideal_reference_tracking": _run_statistics(ideal),
        "noisy_reference_tracking": _run_statistics(noisy),
        "noisy_minus_ideal": {
            "position_m": _statistics(errors["position"]),
            "yaw_deg": _statistics(np.degrees(errors["yaw"])),
            "speed_mps": _statistics(errors["speed"]),
            "vx_mps": _statistics(errors["vx"]),
            "vy_mps": _statistics(errors["vy"]),
            "yaw_rate_radps": _statistics(errors["yaw_rate"]),
            "beta_deg_while_moving": _statistics(
                np.degrees(errors["beta"][moving])
            ),
        },
        "closed_loop_command_difference": {
            name: _statistics(errors[f"command_{name}"])
            for name in ("vx", "vy", "yaw_rate")
        },
    }

    if output_directory is not None:
        output_directory.mkdir(parents=True, exist_ok=True)
        _write_aligned_csv(
            output_directory / "aligned_comparison.csv", time, ideal, noisy, errors
        )
    _render_overlay(
        None if output_directory is None else output_directory / "ideal_noisy_overlay.png",
        str(ideal_raw["scenario"]),
        time,
        ideal,
        noisy,
        errors["position"],
    )
    _render_errors(
        None if output_directory is None else output_directory / "ideal_noisy_errors.png",
        str(ideal_raw["scenario"]),
        time,
        ideal,
        noisy,
        errors,
    )
    if output_directory is not None:
        with (output_directory / "metrics.json").open(
            "w", encoding="utf-8"
        ) as file:
            json.dump(metrics, file, indent=2, ensure_ascii=False)
    if show:
        plt.show()
    plt.close("all")
    return metrics


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Compare ideal and noisy simulation debug-run directories."
    )
    parser.add_argument("--ideal", required=True, type=Path)
    parser.add_argument("--noisy", required=True, type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--sample-period", type=float, default=0.01)
    parser.add_argument(
        "--show", action="store_true", help="Open interactive plot windows."
    )
    args = parser.parse_args()
    if args.output is None and not args.show:
        parser.error("use --show, --output, or both")

    output_directory = None if args.output is None else args.output.resolve()

    metrics = compare_runs(
        args.ideal.resolve(),
        args.noisy.resolve(),
        output_directory,
        args.sample_period,
        args.show,
    )
    print(json.dumps(metrics, indent=2, ensure_ascii=False))
    if output_directory is not None:
        print(f"Saved comparison: {output_directory}")


if __name__ == "__main__":
    main()
