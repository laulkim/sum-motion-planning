"""Each run's own time-trajectory target minus its odometry pose.

Reference samples come from ExecutedCommand, so replans do not require matching
the latest displayed path to old odometry. Source stamps, not receipt times,
align the two streams. This module only evaluates/plots recorded data.
"""

from __future__ import annotations

import csv
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


ERRORS = (
    ("longitudinal_m", "Longitudinal error", "m"),
    ("lateral_m", "Lateral error", "m"),
    ("heading_deg", "Body heading error", "deg"),
)


def _rows(path: Path) -> list[dict[str, str]]:
    with path.open(encoding="utf-8") as file:
        return list(csv.DictReader(file))


def load_tracking_errors(directory: Path) -> dict[str, np.ndarray]:
    odom = _rows(directory / "odom_history.csv")
    commands = _rows(directory / "command_history.csv")
    required = {"source_stamp_ns", "reference_valid", "reference_x", "reference_y",
                "reference_chi_rad", "reference_body_yaw_rad"}
    if not odom or not commands or "source_stamp_ns" not in odom[0] or not required <= commands[0].keys():
        raise ValueError(f"Tracking evaluation needs newly recorded reference/timestamp columns: {directory}")
    first_command = next((row for row in commands if
                          sum(abs(float(row[key])) for key in ("vx", "vy", "yaw_rate")) > 1e-4), commands[0])
    origin = int(first_command["source_stamp_ns"])
    # Keep the latest pose for duplicate timestamps and interpolate only within
    # the recorded odometry range. Subtract integer ns before converting to s.
    poses = {int(row["source_stamp_ns"]): row for row in odom if int(row["source_stamp_ns"]) > 0}
    stamps = sorted(poses)
    if len(stamps) < 2:
        raise ValueError(f"Not enough timestamped odometry: {directory}")
    odom_t = np.array([(stamp - origin) * 1e-9 for stamp in stamps])
    x = np.array([float(poses[stamp]["x"]) for stamp in stamps])
    y = np.array([float(poses[stamp]["y"]) for stamp in stamps])
    yaw = np.unwrap([float(poses[stamp]["body_yaw"]) for stamp in stamps])
    samples = sorted((row for row in commands if int(row["source_stamp_ns"]) >= origin),
                     key=lambda row: int(row["source_stamp_ns"]))
    time = np.array([(int(row["source_stamp_ns"]) - origin) * 1e-9 for row in samples])
    ref_x = np.array([float(row["reference_x"]) for row in samples])
    ref_y = np.array([float(row["reference_y"]) for row in samples])
    chi = np.array([float(row["reference_chi_rad"]) for row in samples])
    ref_yaw = np.array([float(row["reference_body_yaw_rad"]) for row in samples])
    valid = np.array([row["reference_valid"].lower() in ("true", "1") for row in samples])
    valid &= (time >= odom_t[0]) & (time <= odom_t[-1])
    valid &= np.isfinite(ref_x) & np.isfinite(ref_y) & np.isfinite(chi) & np.isfinite(ref_yaw)
    actual_x = np.interp(time, odom_t, x)
    actual_y = np.interp(time, odom_t, y)
    actual_yaw = np.interp(time, odom_t, yaw)
    valid &= np.isfinite(actual_x) & np.isfinite(actual_y) & np.isfinite(actual_yaw)
    dx = ref_x - actual_x
    dy = ref_y - actual_y
    heading = ref_yaw - actual_yaw
    heading = np.arctan2(np.sin(heading), np.cos(heading))
    errors = {
        "longitudinal_m": np.cos(chi) * dx + np.sin(chi) * dy,
        "lateral_m": -np.sin(chi) * dx + np.cos(chi) * dy,
        "heading_deg": np.degrees(heading),
    }
    signals = {
        "reference_x": ref_x, "odom_x": actual_x,
        "reference_y": ref_y, "odom_y": actual_y,
        # Display both headings on the same angular branch. A +/-pi crossing
        # must not look like a 360-degree tracking error.
        "reference_yaw_deg": np.degrees(actual_yaw + heading),
        "odom_yaw_deg": np.degrees(actual_yaw),
    }
    for values in (*errors.values(), *signals.values()):
        values[~valid] = np.nan  # no target during safety/idle/hold: never fake zero error
    return {"time": time, **errors, **signals}


def render_tracking_run(run: dict[str, np.ndarray], label: str, output_path: Path | None) -> None:
    """One run: target/feedback overlays and reference-frame pose errors."""
    figure, axes = plt.subplots(3, 2, figsize=(14, 10), sharex=True, constrained_layout=True)
    figure.suptitle(f"{label}: planning target vs odometry (source-time aligned)")
    for row, (coordinate, title, unit) in enumerate((
        ("x", "Map X", "m"), ("y", "Map Y", "m"),
        ("yaw_deg", "Body heading", "deg"),
    )):
        axis = axes[row, 0]
        axis.plot(run["time"], run[f"reference_{coordinate}"], "--", label="Planning target")
        axis.plot(run["time"], run[f"odom_{coordinate}"], label="Odometry feedback", alpha=0.8)
        axis.set_title(title)
        axis.set_ylabel(unit)
        axis.legend()
        key, error_title, error_unit = ERRORS[row]
        axis = axes[row, 1]
        axis.plot(run["time"], run[key])
        axis.axhline(0.0, color="black", linewidth=0.6)
        axis.set_title(f"{error_title} (target - feedback)")
        axis.set_ylabel(error_unit)
    for axis in axes.flat:
        axis.grid(True, alpha=0.3)
    for axis in axes[-1]:
        axis.set_xlabel("time from first non-zero command [s]")
    if output_path is not None:
        figure.savefig(output_path, dpi=160)


def compare_tracking_runs(
    ideal_directory: Path, noisy_off_directory: Path, noisy_on_directory: Path,
    output_directory: Path | None, scenario: str,
) -> dict[str, object]:
    runs = {
        "ideal (P ON)": load_tracking_errors(ideal_directory),
        "noisy P OFF": load_tracking_errors(noisy_off_directory),
        "noisy P ON": load_tracking_errors(noisy_on_directory),
    }
    start = max(run["time"][0] for run in runs.values())
    end = min(run["time"][-1] for run in runs.values())
    if end <= start:
        raise ValueError("Tracking runs have no common elapsed-time range")
    metrics = {}
    figure, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True, constrained_layout=True)
    figure.suptitle(f"{scenario}: each run's OWN target minus odometry pose")
    summary, bars = plt.subplots(1, 3, figsize=(15, 5), constrained_layout=True)
    summary.suptitle(f"{scenario}: own-target tracking error statistics")
    for index, (label, run) in enumerate(runs.items()):
        keep = (run["time"] >= start) & (run["time"] <= end)
        metrics[label] = {}
        for (key, title, unit), axis, bar in zip(ERRORS, axes, bars):
            values = run[key][keep]
            finite = values[np.isfinite(values)]
            stats = {
                "sample_count": int(finite.size),
                "rmse": float(np.sqrt(np.mean(finite**2))) if finite.size else None,
                "max_abs": float(np.max(np.abs(finite))) if finite.size else None,
            }
            metrics[label][key] = stats
            axis.plot(run["time"][keep], values, label=label, alpha=0.85)
            axis.set_title(title)
            axis.set_ylabel(unit)
            heights = [stats[name] if stats[name] is not None else np.nan for name in ("rmse", "max_abs")]
            rectangles = bar.bar(np.arange(2) + (index - 1) * 0.25, heights, width=0.25, label=label)
            bar.bar_label(rectangles, fmt="%.3g", fontsize=8)
            bar.set_xticks([0, 1], ["RMSE", "Max |error|"])
            bar.set_title(title)
            bar.set_ylabel(unit)
    for axis in axes:
        axis.axhline(0.0, color="black", linewidth=0.6)
        axis.grid(True, alpha=0.3)
        axis.legend()
    axes[-1].set_xlabel("time from each run's first non-zero command [s]")
    for axis in bars:
        axis.grid(True, axis="y", alpha=0.3)
        axis.legend(fontsize=8)
    result = {
        "comparison": "each run's own target minus odometry pose",
        "alignment": "source header timestamps within each run; first non-zero command across runs",
        "range_sec": [float(start), float(end)],
        "runs": metrics,
    }
    if output_directory is not None:
        output_directory.mkdir(parents=True, exist_ok=True)
        figure.savefig(output_directory / "tracking_errors.png", dpi=160)
        summary.savefig(output_directory / "tracking_error_metrics.png", dpi=160)
        with (output_directory / "tracking_metrics.json").open("w", encoding="utf-8") as file:
            json.dump(result, file, indent=2, ensure_ascii=False)
    for label, filename in (("ideal (P ON)", "ideal_target_feedback.png"),
                            ("noisy P ON", "noisy_target_feedback.png")):
        render_tracking_run(runs[label], f"{scenario}: {label}",
                            None if output_directory is None else output_directory / filename)
    # The comparison caller shows all figures with one plt.show().
    return result
