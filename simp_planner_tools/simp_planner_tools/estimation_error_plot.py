from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
from typing import Iterable

import matplotlib.pyplot as plt
import numpy as np


NUMERIC_COLUMNS = (
    "time",
    "receive_time",
    "x",
    "y",
    "body_yaw",
    "speed",
    "global_lateral_deviation",
)

OUTPUT_COLUMNS = (
    "time_sec",
    "estimate_x_m",
    "estimate_y_m",
    "ground_truth_x_m",
    "ground_truth_y_m",
    "error_x_m",
    "error_y_m",
    "position_error_m",
    "estimate_yaw_rad",
    "ground_truth_yaw_rad",
    "yaw_error_deg",
    "estimate_speed_mps",
    "ground_truth_speed_mps",
    "speed_error_mps",
    "estimate_global_lateral_m",
    "ground_truth_global_lateral_m",
    "global_lateral_error_m",
)


def load_odom_csv(path: Path) -> dict[str, np.ndarray]:
    with path.open(newline="", encoding="utf-8") as file:
        reader = csv.DictReader(file)
        if reader.fieldnames is None:
            raise ValueError(f"CSV has no header: {path}")
        missing = [name for name in NUMERIC_COLUMNS if name not in reader.fieldnames]
        if missing:
            raise ValueError(f"CSV is missing columns {missing}: {path}")
        values = {name: [] for name in NUMERIC_COLUMNS}
        for row in reader:
            try:
                sample = {name: float(row[name]) for name in NUMERIC_COLUMNS}
            except (TypeError, ValueError):
                continue
            if not all(math.isfinite(value) for value in sample.values()):
                continue
            for name, value in sample.items():
                values[name].append(value)
    if not values["receive_time"]:
        raise ValueError(f"CSV contains no usable odometry samples: {path}")
    return {name: np.asarray(column, dtype=float) for name, column in values.items()}


def _sorted_unique_time_series(
    data: dict[str, np.ndarray], time_column: str
) -> dict[str, np.ndarray]:
    order = np.argsort(data[time_column], kind="stable")
    sorted_time = data[time_column][order]
    _, reverse_unique_indices = np.unique(sorted_time[::-1], return_index=True)
    keep = len(sorted_time) - 1 - reverse_unique_indices
    keep.sort()
    return {name: values[order][keep] for name, values in data.items()}


def wrap_angle_array(values: np.ndarray) -> np.ndarray:
    return np.arctan2(np.sin(values), np.cos(values))


def build_error_series(
    estimate: dict[str, np.ndarray],
    ground_truth: dict[str, np.ndarray],
    time_column: str = "receive_time",
) -> dict[str, np.ndarray]:
    if time_column not in {"receive_time", "time"}:
        raise ValueError("time_column must be 'receive_time' or 'time'")
    estimate = _sorted_unique_time_series(estimate, time_column)
    ground_truth = _sorted_unique_time_series(ground_truth, time_column)

    estimate_time = estimate[time_column]
    truth_time = ground_truth[time_column]
    overlap = (estimate_time >= truth_time[0]) & (estimate_time <= truth_time[-1])
    if not np.any(overlap):
        raise ValueError("Estimate and ground-truth CSV files do not overlap in time")

    time = estimate_time[overlap]

    def estimate_values(name: str) -> np.ndarray:
        return estimate[name][overlap]

    def interpolate_truth(name: str) -> np.ndarray:
        return np.interp(time, truth_time, ground_truth[name])

    estimate_x = estimate_values("x")
    estimate_y = estimate_values("y")
    truth_x = interpolate_truth("x")
    truth_y = interpolate_truth("y")
    error_x = estimate_x - truth_x
    error_y = estimate_y - truth_y

    estimate_yaw = estimate_values("body_yaw")
    unwrapped_truth_yaw = np.unwrap(ground_truth["body_yaw"])
    truth_yaw = wrap_angle_array(np.interp(time, truth_time, unwrapped_truth_yaw))
    yaw_error_deg = np.degrees(wrap_angle_array(estimate_yaw - truth_yaw))

    estimate_speed = estimate_values("speed")
    truth_speed = interpolate_truth("speed")
    estimate_lateral = estimate_values("global_lateral_deviation")
    truth_lateral = interpolate_truth("global_lateral_deviation")

    return {
        "time_sec": time,
        "estimate_x_m": estimate_x,
        "estimate_y_m": estimate_y,
        "ground_truth_x_m": truth_x,
        "ground_truth_y_m": truth_y,
        "error_x_m": error_x,
        "error_y_m": error_y,
        "position_error_m": np.hypot(error_x, error_y),
        "estimate_yaw_rad": estimate_yaw,
        "ground_truth_yaw_rad": truth_yaw,
        "yaw_error_deg": yaw_error_deg,
        "estimate_speed_mps": estimate_speed,
        "ground_truth_speed_mps": truth_speed,
        "speed_error_mps": estimate_speed - truth_speed,
        "estimate_global_lateral_m": estimate_lateral,
        "ground_truth_global_lateral_m": truth_lateral,
        "global_lateral_error_m": estimate_lateral - truth_lateral,
    }


def resolve_input_paths(
    input_path: Path, ground_truth_csv: Path | None
) -> tuple[Path, Path, Path]:
    input_path = input_path.expanduser().resolve()
    if input_path.is_dir():
        estimate_path = input_path / "odom_history_estimate.csv"
        truth_path = input_path / "odom_history_ground_truth.csv"
        session_dir = input_path
    else:
        estimate_path = input_path
        if ground_truth_csv is not None:
            truth_path = ground_truth_csv.expanduser().resolve()
        elif "estimate" in estimate_path.name:
            truth_path = estimate_path.with_name(
                estimate_path.name.replace("estimate", "ground_truth")
            )
        else:
            raise ValueError(
                "Pass --ground-truth-csv when the input is not a session directory"
            )
        session_dir = estimate_path.parent
    for path in (estimate_path, truth_path):
        if not path.is_file():
            raise FileNotFoundError(path)
    return estimate_path, truth_path, session_dir


def write_error_csv(path: Path, series: dict[str, np.ndarray]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.writer(file)
        writer.writerow(OUTPUT_COLUMNS)
        for row in zip(*(series[name] for name in OUTPUT_COLUMNS)):
            writer.writerow(f"{float(value):.9f}" for value in row)


def percentile_summary(
    series: dict[str, np.ndarray], min_speed_mps: float
) -> dict[str, tuple[float, float, float, float]]:
    mask = series["ground_truth_speed_mps"] >= min_speed_mps
    if not np.any(mask):
        mask = np.ones_like(series["time_sec"], dtype=bool)
    metrics = {
        "position_cm": 100.0 * series["position_error_m"],
        "yaw_deg": np.abs(series["yaw_error_deg"]),
        "speed_mps": np.abs(series["speed_error_mps"]),
        "global_lateral_cm": 100.0 * np.abs(series["global_lateral_error_m"]),
    }
    summary = {}
    for name, values in metrics.items():
        selected = values[mask]
        summary[name] = (
            float(np.sqrt(np.mean(selected * selected))),
            float(np.percentile(selected, 50.0)),
            float(np.percentile(selected, 95.0)),
            float(np.max(selected)),
        )
    return summary


def _format_summary(
    summary: dict[str, tuple[float, float, float, float]],
    min_speed_mps: float,
) -> str:
    labels = {
        "position_cm": ("position", "cm"),
        "yaw_deg": ("yaw", "deg"),
        "speed_mps": ("speed", "m/s"),
        "global_lateral_cm": ("global lateral", "cm"),
    }
    lines = [
        f"Statistics for ground-truth speed >= {min_speed_mps:.3f} m/s",
        "metric                 RMSE        p50        p95        max",
    ]
    for name, values in summary.items():
        label, unit = labels[name]
        lines.append(
            f"{label:<18} {values[0]:>8.3f} {values[1]:>10.3f} "
            f"{values[2]:>10.3f} {values[3]:>10.3f} {unit}"
        )
    return "\n".join(lines)


def _add_percentile_lines(
    axis: plt.Axes,
    values: np.ndarray,
    statistics_mask: np.ndarray,
    label_prefix: str = "",
) -> None:
    absolute = np.abs(values[statistics_mask])
    p50 = float(np.percentile(absolute, 50.0))
    p95 = float(np.percentile(absolute, 95.0))
    prefix = f"{label_prefix} " if label_prefix else ""
    axis.axhline(p50, color="tab:green", linestyle=":", linewidth=1.0,
                 label=f"{prefix}|error| p50={p50:.3f}")
    axis.axhline(p95, color="tab:red", linestyle="--", linewidth=1.0,
                 label=f"{prefix}|error| p95={p95:.3f}")
    if np.any(values < 0.0):
        axis.axhline(-p50, color="tab:green", linestyle=":", linewidth=1.0)
        axis.axhline(-p95, color="tab:red", linestyle="--", linewidth=1.0)


def _connect_interaction(
    figure: plt.Figure,
    time_axes: Iterable[plt.Axes],
    series: dict[str, np.ndarray],
) -> None:
    time_axes = tuple(time_axes)
    vertical_lines = [axis.axvline(math.nan, color="black", alpha=0.35) for axis in time_axes]
    readout = figure.text(0.01, 0.008, "Move the mouse over a time plot for exact values.",
                          family="monospace", fontsize=8)
    time = series["time_sec"]

    def on_motion(event) -> None:
        if event.inaxes not in time_axes or event.xdata is None:
            return
        index = int(np.clip(np.searchsorted(time, event.xdata), 0, len(time) - 1))
        if index > 0 and abs(time[index - 1] - event.xdata) < abs(time[index] - event.xdata):
            index -= 1
        for line in vertical_lines:
            line.set_xdata([time[index], time[index]])
        readout.set_text(
            f"t={time[index]:.3f} s  "
            f"position={100.0 * series['position_error_m'][index]:.2f} cm  "
            f"yaw={series['yaw_error_deg'][index]:+.3f} deg  "
            f"speed={series['speed_error_mps'][index]:+.4f} m/s  "
            f"global_lateral={100.0 * series['global_lateral_error_m'][index]:+.2f} cm"
        )
        figure.canvas.draw_idle()

    def on_scroll(event) -> None:
        if event.inaxes is None or event.xdata is None or event.ydata is None:
            return
        scale = 0.8 if event.button == "up" else 1.25
        x_min, x_max = event.inaxes.get_xlim()
        y_min, y_max = event.inaxes.get_ylim()
        event.inaxes.set_xlim(
            event.xdata - (event.xdata - x_min) * scale,
            event.xdata + (x_max - event.xdata) * scale,
        )
        event.inaxes.set_ylim(
            event.ydata - (event.ydata - y_min) * scale,
            event.ydata + (y_max - event.ydata) * scale,
        )
        figure.canvas.draw_idle()

    figure.canvas.mpl_connect("motion_notify_event", on_motion)
    figure.canvas.mpl_connect("scroll_event", on_scroll)


def create_figure(
    series: dict[str, np.ndarray],
    session_name: str,
    min_stat_speed_mps: float,
) -> plt.Figure:
    figure = plt.figure(figsize=(17, 11), constrained_layout=True)
    grid = figure.add_gridspec(3, 2)
    position_axis = figure.add_subplot(grid[0, 0])
    yaw_axis = figure.add_subplot(grid[0, 1], sharex=position_axis)
    speed_axis = figure.add_subplot(grid[1, 0], sharex=position_axis)
    lateral_axis = figure.add_subplot(grid[1, 1], sharex=position_axis)
    lateral_value_axis = figure.add_subplot(grid[2, 0], sharex=position_axis)
    trajectory_axis = figure.add_subplot(grid[2, 1])
    time = series["time_sec"]
    statistics_mask = series["ground_truth_speed_mps"] >= min_stat_speed_mps
    if not np.any(statistics_mask):
        statistics_mask = np.ones_like(time, dtype=bool)

    position_cm = 100.0 * series["position_error_m"]
    position_axis.plot(time, position_cm, label="|estimate - GT|")
    position_axis.plot(time, 100.0 * series["error_x_m"], alpha=0.5, label="dx")
    position_axis.plot(time, 100.0 * series["error_y_m"], alpha=0.5, label="dy")
    _add_percentile_lines(position_axis, position_cm, statistics_mask)
    position_axis.set_title("Effective position error (delay included)")
    position_axis.set_ylabel("error [cm]")
    position_axis.legend(fontsize=8)

    yaw_axis.plot(time, series["yaw_error_deg"], label="estimate - GT")
    _add_percentile_lines(yaw_axis, series["yaw_error_deg"], statistics_mask)
    yaw_axis.set_title("Yaw error")
    yaw_axis.set_ylabel("error [deg]")
    yaw_axis.legend(fontsize=8)

    speed_axis.plot(time, series["speed_error_mps"], label="estimate - GT")
    _add_percentile_lines(speed_axis, series["speed_error_mps"], statistics_mask)
    speed_axis.set_title("Speed error")
    speed_axis.set_ylabel("error [m/s]")
    speed_axis.legend(fontsize=8)

    lateral_cm = 100.0 * series["global_lateral_error_m"]
    lateral_axis.plot(time, lateral_cm, label="estimate - GT")
    _add_percentile_lines(lateral_axis, lateral_cm, statistics_mask)
    lateral_axis.set_title("Global-path lateral-error difference")
    lateral_axis.set_ylabel("difference [cm]")
    lateral_axis.legend(fontsize=8)

    lateral_value_axis.plot(
        time, series["estimate_global_lateral_m"], label="estimate"
    )
    lateral_value_axis.plot(
        time, series["ground_truth_global_lateral_m"], "--", label="ground truth"
    )
    lateral_value_axis.set_title("Global-path lateral deviation")
    lateral_value_axis.set_xlabel("receive time [s]")
    lateral_value_axis.set_ylabel("deviation [m]")
    lateral_value_axis.legend(fontsize=8)

    trajectory_axis.plot(
        series["ground_truth_x_m"], series["ground_truth_y_m"],
        label="ground truth", linewidth=2.0,
    )
    trajectory_axis.plot(
        series["estimate_x_m"], series["estimate_y_m"],
        "--", label="estimate", linewidth=1.2,
    )
    trajectory_axis.set_title("Trajectory overlay (zoom here for spatial detail)")
    trajectory_axis.set_xlabel("x [m]")
    trajectory_axis.set_ylabel("y [m]")
    trajectory_axis.axis("equal")
    trajectory_axis.legend(fontsize=8)

    for axis in (position_axis, yaw_axis, speed_axis, lateral_axis, lateral_value_axis):
        axis.grid(True)
        axis.set_xlabel("receive time [s]")
    trajectory_axis.grid(True)
    figure.suptitle(
        f"GT vs estimate error analysis - {session_name}\n"
        f"p50/p95 use GT speed >= {min_stat_speed_mps:.3f} m/s | "
        "mouse wheel: zoom | toolbar: rectangle zoom/pan/home"
    )
    _connect_interaction(
        figure,
        (position_axis, yaw_axis, speed_axis, lateral_axis, lateral_value_axis),
        series,
    )
    return figure


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Compare recorded estimate and ground-truth odometry and open an "
            "interactive Python/Matplotlib error dashboard."
        )
    )
    parser.add_argument(
        "input",
        type=Path,
        help="Session directory or odom_history_estimate.csv",
    )
    parser.add_argument(
        "--ground-truth-csv",
        type=Path,
        help="Ground-truth CSV when input is an estimate CSV",
    )
    parser.add_argument(
        "--alignment",
        choices=("receive", "source"),
        default="receive",
        help=(
            "receive compares values available at the same wall time and includes "
            "estimator delay; source uses each logger's aligned header time"
        ),
    )
    parser.add_argument(
        "--min-stat-speed",
        type=float,
        default=0.1,
        help="Only samples at or above this GT speed contribute to summary statistics",
    )
    parser.add_argument("--output-csv", type=Path)
    parser.add_argument("--output-image", type=Path)
    parser.add_argument("--no-show", action="store_true")
    return parser.parse_args()


def main() -> None:
    arguments = parse_arguments()
    if arguments.min_stat_speed < 0.0:
        raise ValueError("--min-stat-speed must be non-negative")
    estimate_path, truth_path, session_dir = resolve_input_paths(
        arguments.input, arguments.ground_truth_csv
    )
    estimate = load_odom_csv(estimate_path)
    ground_truth = load_odom_csv(truth_path)
    time_column = "receive_time" if arguments.alignment == "receive" else "time"
    series = build_error_series(estimate, ground_truth, time_column=time_column)

    output_csv = arguments.output_csv or session_dir / "estimation_error_comparison.csv"
    output_image = arguments.output_image or session_dir / "estimation_error_overview.png"
    write_error_csv(output_csv.expanduser().resolve(), series)
    summary = percentile_summary(series, arguments.min_stat_speed)
    print(_format_summary(summary, arguments.min_stat_speed))
    print(f"Aligned samples: {len(series['time_sec'])}")
    print(f"Comparison CSV: {output_csv.expanduser().resolve()}")

    figure = create_figure(series, session_dir.name, arguments.min_stat_speed)
    output_image = output_image.expanduser().resolve()
    output_image.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output_image, dpi=150)
    print(f"Overview image: {output_image}")
    if arguments.no_show:
        plt.close(figure)
    else:
        plt.show()


if __name__ == "__main__":
    main()
