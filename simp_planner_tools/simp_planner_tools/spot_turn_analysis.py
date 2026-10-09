"""Shutdown analysis of pure-rotation references and timestamped odometry.

No control decisions are made here. Odom displacement uses measured samples,
not predicted pose; with simulator sensor_delay_sec=0 these are truth samples.
"""
from __future__ import annotations

import csv
import math

import numpy as np


POST_TURN_SECONDS = 3.0
SUMMARY_COLUMNS = (
    "turn", "start_s", "duration_s", "partial", "mode_verified", "control_samples",
    "odom_samples", "odom_span_s", "initial_position_error_m", "vx_cmd_peak_mps",
    "vy_cmd_peak_mps", "translation_cmd_rms_mps", "command_distance_m",
    "odom_net_displacement_m", "odom_max_displacement_m", "odom_path_length_m",
    "yaw_rmse_deg", "yaw_peak_deg", "post_position_rmse_m", "post_cmd_peak_mps",
)


def read_series(path, columns):
    """Old recordings may have no sidecars, and a topic may never publish."""
    if not path.exists():
        return np.empty((0, columns))
    with path.open() as file:
        next(file, None)
        rows = [row for row in csv.reader(file) if row]
    return np.asarray(rows, dtype=float).reshape(-1, columns)


def analyze_turns(data, odometry=None, modes=None):
    """Tracking columns retain their existing layout; sidecars start with ns."""
    data = np.atleast_2d(data)
    odometry = np.empty((0, 7)) if odometry is None else np.asarray(odometry)
    modes = np.empty((0, 4)) if modes is None else np.asarray(modes)
    valid = np.isfinite(data[:, :19]).all(axis=1)
    if data.shape[1] >= 23:
        valid &= data[:, 22] == 1
    # Exactly the planner-reference condition used by gain_scheduler.hpp.
    mask = valid & (data[:, 13] == 0) & (data[:, 14] == 0) & (data[:, 15] != 0)
    verified = bool(len(modes) and data.shape[1] >= 23)
    if verified:
        modes = modes[np.isfinite(modes).all(axis=1)]
        modes = modes[np.argsort(modes[:, 0], kind="stable")]
        index = np.searchsorted(modes[:, 0], data[:, 19], side="right") - 1
        ready = np.zeros(len(data), dtype=bool)
        known = index >= 0
        # DriveModeState: SPOT_TURN=4, STATUS_READY=1. Never use future feedback.
        ready[known] = (modes[index[known], 1] == 4) & (modes[index[known], 3] == 1)
        mask &= ready

    if len(odometry) and data.shape[1] >= 23:
        odometry = odometry[np.isfinite(odometry).all(axis=1)].copy()
        _, unique = np.unique(odometry[:, 0], return_index=True)
        odometry = odometry[unique]
        origin_ns = data[0, 19] - data[0, 0] * 1e9
        odometry[:, 0] = (odometry[:, 0] - origin_ns) * 1e-9
    else:
        odometry = np.empty((0, 7))

    edges = np.diff(np.r_[0, mask.astype(int), 0])
    starts, stops = np.flatnonzero(edges == 1), np.flatnonzero(edges == -1)
    turns = []
    for number, (start, stop) in enumerate(zip(starts, stops), 1):
        samples = data[start:stop]
        begin = samples[0, 0]
        end = data[stop, 0] if stop < len(data) else samples[-1, 0]
        measured = odometry[(odometry[:, 0] >= begin) & (odometry[:, 0] <= end)]
        post_stop = starts[number] if number < len(starts) else len(data)
        post = data[stop:post_stop]
        post = post[(post[:, 0] <= end + POST_TURN_SECONDS) & valid[stop:post_stop]]
        post_odom = odometry[(odometry[:, 0] >= end) &
                            (odometry[:, 0] <= end + POST_TURN_SECONDS)]
        if number < len(starts):
            post_odom = post_odom[post_odom[:, 0] < data[starts[number], 0]]
        speed = np.hypot(samples[:, 16], samples[:, 17])
        # Commands are zero-order held up to the next control timestamp.
        intervals = np.diff(np.r_[samples[:, 0], end])
        yaw_error = np.degrees(samples[:, 9])
        metrics = dict.fromkeys(SUMMARY_COLUMNS, math.nan)
        metrics.update(
            turn=number, start_s=begin, duration_s=end - begin,
            partial=int(start == 0 or stop == len(data)), mode_verified=int(verified),
            control_samples=len(samples), odom_samples=len(measured),
            odom_span_s=measured[-1, 0] - measured[0, 0] if len(measured) > 1 else 0.0,
            initial_position_error_m=np.hypot(samples[0, 4] - samples[0, 1],
                                              samples[0, 5] - samples[0, 2]),
            vx_cmd_peak_mps=np.max(np.abs(samples[:, 16])),
            vy_cmd_peak_mps=np.max(np.abs(samples[:, 17])),
            translation_cmd_rms_mps=np.sqrt(np.mean(speed ** 2)),
            command_distance_m=np.sum(speed * intervals),
            yaw_rmse_deg=np.sqrt(np.mean(yaw_error ** 2)),
            yaw_peak_deg=np.max(np.abs(yaw_error)),
        )
        if len(measured) > 1:
            offset = measured[:, 1:3] - measured[0, 1:3]
            distance = np.hypot(offset[:, 0], offset[:, 1])
            path = np.sum(np.hypot(*np.diff(measured[:, 1:3], axis=0).T))
            metrics.update(odom_net_displacement_m=distance[-1],
                           odom_max_displacement_m=np.max(distance), odom_path_length_m=path)
        if len(post):
            error = np.hypot(post[:, 4] - post[:, 1], post[:, 5] - post[:, 2])
            metrics.update(post_position_rmse_m=np.sqrt(np.mean(error ** 2)),
                           post_cmd_peak_mps=np.max(np.hypot(post[:, 16], post[:, 17])))
        turns.append(dict(metrics=metrics, samples=samples, odometry=measured,
                          post=post, post_odometry=post_odom, end=end))
    return turns


def create_turn_figures(turns, predicted=True):
    import matplotlib.pyplot as plt

    summary, ax = plt.subplots(figsize=(18, max(3, 1.8 + 0.55 * len(turns))), layout="constrained")
    summary.canvas.manager.set_window_title("Spot-turn summary")
    ax.axis("off")
    ax.set_title("Spot-turn analysis: command suppression, measured movement and yaw tracking")
    if not turns:
        ax.text(0.5, 0.5, "No valid pure-rotation interval found.\n"
                "Reference must have vx=vy=0 and yaw_rate!=0; recorded mode must be SPOT_TURN/READY.",
                ha="center", va="center", transform=ax.transAxes)
        return [summary]
    fields = (
        ("duration_s", "Duration\n[s]", 1), ("odom_span_s", "Odom span\n[s]", 1),
        ("initial_position_error_m", f"Entry {'predicted' if predicted else 'measured'}\nerror [cm]", 100),
        ("command_distance_m", "Command integral\n[cm]", 100),
        ("odom_max_displacement_m", "Max center shift\n[cm]", 100),
        ("odom_net_displacement_m", "End center shift\n[cm]", 100),
        ("odom_path_length_m", "Odom travel\n[cm]", 100),
        ("yaw_rmse_deg", "Yaw RMSE\n[deg]", 1),
        ("post_position_rmse_m", f"Post {'predicted' if predicted else 'measured'}\nRMSE [cm]", 100),
        ("post_cmd_peak_mps", "Post cmd peak\n[m/s]", 1),
    )
    rows = []
    for turn in turns:
        metric = turn["metrics"]
        label = f"{metric['turn']}" + ("*" if metric["partial"] else "")
        label += " / READY" if metric["mode_verified"] else " / ref only"
        rows.append([label] + [f"{metric[key] * scale:.4f}" if math.isfinite(metric[key]) else "--"
                               for key, _, scale in fields])
    table = ax.table(cellText=rows, colLabels=["Turn / mode"] + [label for _, label, _ in fields],
                     loc="center", cellLoc="center")
    table.auto_set_font_size(False)
    table.set_fontsize(9)
    table.scale(1, 2)
    ax.text(0.5, 0.02, "* Recording starts/ends inside an active interval. Odom shift uses first available "
            "acquisition sample; travel sums pose increments.\n"
            "Post window: first 3 s, including alignment/next motion. Missing odometry is --. "
            "Coverage/sample counts are in spot_turn_summary.csv.",
            ha="center", va="bottom", fontsize=9, transform=ax.transAxes)
    figures = [summary]
    for turn in turns:
        metric, samples, measured = turn["metrics"], turn["samples"], turn["odometry"]
        begin, end = metric["start_s"], turn["end"]
        figure, axes = plt.subplots(4, 2, figsize=(13, 10), layout="constrained")
        figure.canvas.manager.set_window_title(f"Spot turn {metric['turn']}")
        figure.suptitle(f"Spot turn {metric['turn']} | {begin:.3f}--{end:.3f} s | "
                       f"{'SPOT_TURN/READY' if metric['mode_verified'] else 'reference only; mode unknown'}"
                       + (" | PARTIAL" if metric["partial"] else ""))
        for i, ax in enumerate((axes[0, 0], axes[0, 1], axes[1, 0])):
            scale = 180 / math.pi if i == 2 else 1
            ax.step(np.r_[samples[:, 0] - begin, end - begin],
                    np.r_[samples[:, 16 + i], samples[-1, 16 + i]] * scale,
                    where="post", label="command")
            ax.plot(samples[:, 0] - begin, samples[:, 13 + i] * scale, "--", label="reference")
            if len(measured):
                ax.plot(measured[:, 0] - begin, measured[:, 4 + i] * scale, label="odom measurement")
            ax.set_ylabel(("vx [m/s]", "vy [m/s]", "yaw rate [deg/s]")[i])
        axes[1, 1].plot(samples[:, 0] - begin, np.degrees(samples[:, 9]), label="controller yaw error")
        axes[1, 1].set_ylabel("Yaw error [deg]")
        if len(measured):
            offset = (measured[:, 1:3] - measured[0, 1:3]) * 100
            axes[2, 0].plot(offset[:, 0], offset[:, 1], ".-", label="measured center path")
            axes[2, 0].scatter(*offset[0], marker="o", label="first sample")
            axes[2, 0].scatter(*offset[-1], marker="x", label="last sample")
            distance = np.hypot(offset[:, 0], offset[:, 1])
            travel = np.r_[0, np.cumsum(np.hypot(*np.diff(offset, axis=0).T))]
            axes[2, 1].plot(measured[:, 0] - begin, distance, label="distance from first sample")
            axes[2, 1].plot(measured[:, 0] - begin, travel, "--", label="cumulative travel")
        else:
            for ax in axes[2]:
                ax.text(0.5, 0.5, "No odometry samples", ha="center", transform=ax.transAxes)
        axes[2, 0].set_xlabel("World delta x [cm]")
        axes[2, 0].set_ylabel("World delta y [cm]")
        axes[2, 0].set_aspect("equal", adjustable="datalim")
        axes[2, 1].set_ylabel("Measured movement [cm]")
        if end > begin:
            for ax in (*axes[0], *axes[1], axes[2, 1]):
                ax.set_xlim(0, end - begin)
        post, post_odom = turn["post"], turn["post_odometry"]
        if len(post):
            error = np.hypot(post[:, 4] - post[:, 1], post[:, 5] - post[:, 2])
            axes[3, 0].plot(post[:, 0] - end, error * 100,
                            label="reference minus predicted" if predicted else "reference minus measured")
            axes[3, 1].plot(post[:, 0] - end, np.hypot(post[:, 16], post[:, 17]), label="command speed")
            axes[3, 1].plot(post[:, 0] - end, np.hypot(post[:, 13], post[:, 14]), "--", label="reference speed")
        if len(post_odom):
            axes[3, 1].plot(post_odom[:, 0] - end, np.hypot(post_odom[:, 4], post_odom[:, 5]),
                            label="odom speed")
        if not len(post):
            axes[3, 0].text(0.5, 0.5, "Post-turn control not recorded", ha="center", transform=axes[3, 0].transAxes)
        axes[3, 0].set_ylabel("Post position error [cm]")
        axes[3, 1].set_ylabel("Post translational speed [m/s]")
        for row in axes:
            for ax in row:
                ax.grid(True, alpha=0.3)
                if ax.lines or ax.collections:
                    ax.legend(fontsize=8)
                if ax is not axes[2, 0]:
                    ax.set_xlabel("Time after turn end [s]" if ax in axes[3] else "Time after turn start [s]")
        figures.append(figure)
    return figures


def write_summary(path, turns):
    with path.open("w", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=SUMMARY_COLUMNS)
        writer.writeheader()
        writer.writerows(turn["metrics"] for turn in turns)
