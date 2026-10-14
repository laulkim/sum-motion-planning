"""종료 후 추종 오차, 상태/입력, yaw-rate projection을 Matplotlib 창으로 표시한다."""
from __future__ import annotations

import csv
import json
import math
import os
import signal
from pathlib import Path
import subprocess
import sys
from datetime import datetime

import numpy as np


COLUMNS = (
    "time_s", "predicted_x", "predicted_y", "predicted_yaw", "xr", "yr", "yaw_r", "ex", "ey", "etheta",
    "vx", "vy", "omega", "vx_ref", "vy_ref", "omega_ref", "vx_cmd", "vy_cmd", "omega_cmd",
    "control_stamp_ns", "odom_stamp_ns", "reference_stamp_ns", "valid",
)


# Below this speed the motion direction atan2(vy, vx) is undefined.
HEADING_MIN_SPEED = 0.05

# Allocation profiles the planner selects between (see
# allocate_with_oriented_collision_search). NONE marks trajectory content that
# no allocation produced: stops, holds and spot-turn rotations.
VY_PRIORITY = "LATERAL_PRIORITY"  # beta and vy carry the maneuver, body heading lags
VX_PRIORITY = "MINIMUM_VY"        # body heading follows the motion, vy kept small
# Level of each profile in the allocation state graph; "NONE" is trajectory
# content that no allocation produced (stops, holds and spot-turn rotation).
ALLOCATION_STATES = {"NONE": 0, VY_PRIORITY: 1, VX_PRIORITY: 2}
ALLOCATION_COLUMNS = ("start_ns", "allocation_profile")
ALLOCATION_FILE = "allocation.csv"
# The planner's yaw-rate projection at the reference point each control cycle
# used, one row per tracking.csv row (TrackingDiagnosticsPoint fields).
PROJECTION_COLUMNS = ("control_stamp_ns", "projection_yaw_rate_delta", "projection_score",
                      "projection_yaw_rate_limit")
PROJECTION_FILE = "projection.csv"
# Diagnostics kept for the lookup: a few planning cycles of trajectories.
PROJECTION_HISTORY = 32


def stamp_ns(stamp):
    return stamp.sec * 1000000000 + stamp.nanosec


def wrap_angle(angle):
    return (np.asarray(angle) + math.pi) % (2.0 * math.pi) - math.pi


def motion_heading(yaw, vx, vy):
    """chi = body yaw + slip angle atan2(vy, vx); NaN while (nearly) stationary."""
    chi = wrap_angle(yaw + np.arctan2(vy, vx))
    with np.errstate(invalid="ignore"):
        return np.where(np.hypot(vx, vy) >= HEADING_MIN_SPEED, chi, math.nan)


def motion_heading_rate(time, yaw_rate, vx, vy):
    """chi_dot = yaw rate + beta_dot. beta_dot is the difference of atan2(vy, vx)
    between distinct sample times; a repeated sample (held odometry) keeps the
    previous rate instead of producing a zero-time spike."""
    beta = np.arctan2(vy, vx)
    with np.errstate(invalid="ignore"):
        valid = np.isfinite(time) & (np.hypot(vx, vy) >= HEADING_MIN_SPEED)
    beta_rate = np.full(len(time), math.nan)
    last = None
    for k in range(len(time)):
        if not valid[k]:
            last = None
            continue
        if last is not None:
            dt = time[k] - time[last]
            if dt <= 0.0:
                beta_rate[k] = beta_rate[last]
                continue
            beta_rate[k] = wrap_angle(beta[k] - beta[last]) / dt
        last = k
    return yaw_rate + beta_rate


def allocation_event(status_json):
    """(start_ns, profile) when a planner status reports newly committed
    trajectory content, otherwise None."""
    try:
        status = json.loads(status_json)
    except json.JSONDecodeError:
        return None
    if not isinstance(status, dict) or "trajectory_start_ns" not in status:
        return None
    return int(status["trajectory_start_ns"]), str(status.get("allocation_profile", "NONE"))


def allocation_profiles(control_ns, events):
    """Allocation profile of the trajectory content playing at each control time.

    `events` are (start_ns, profile) in publish order. Content published later
    replaces earlier content from its own start onward, so the profile at t is
    the one from the last-published event starting at or before t; None before
    the first event."""
    starts, profiles = [], []
    for start, profile in events:
        while starts and starts[-1] >= start:
            starts.pop()
            profiles.pop()
        starts.append(start)
        profiles.append(profile)
    index = np.searchsorted(np.asarray(starts, dtype=float), np.asarray(control_ns, dtype=float),
                            side="right") - 1
    return [profiles[i] if i >= 0 else None for i in index]


def projection_at(diagnostics, reference_ns, period_ns):
    """Projection values of the diagnostics point at `reference_ns`.

    `diagnostics` maps a tracking trajectory's stamp to per-point value lists,
    one list per PROJECTION_COLUMNS value column. The trajectory in effect is
    the newest one stamped at or before the reference time; points are one
    command period apart. NaN when none covers it."""
    missing = (math.nan,) * (len(PROJECTION_COLUMNS) - 1)
    stamps = [stamp for stamp in diagnostics if stamp <= reference_ns]
    if not stamps:
        return missing
    stamp = max(stamps)
    columns = diagnostics[stamp]
    index = round((reference_ns - stamp) / period_ns)
    if not 0 <= index < len(columns[0]):
        return missing
    return tuple(values[index] for values in columns)


def sample_row(message, origin_ns):
    # The controller owns prediction, reference selection and error calculation.
    state = [math.nan] * 15
    if message.valid:
        reference = message.reference
        velocity = message.measured_velocity
        state = [message.predicted_x, message.predicted_y, message.predicted_yaw,
                 reference.x, reference.y, reference.body_yaw,
                 message.ex, message.ey, message.etheta,
                 velocity.linear.x, velocity.linear.y, velocity.angular.z,
                 reference.vx, reference.vy, reference.yaw_rate]
    command = message.command
    return ((stamp_ns(message.header.stamp) - origin_ns) * 1e-9, *state,
            command.linear.x, command.linear.y, command.angular.z,
            stamp_ns(message.header.stamp),
            stamp_ns(message.odom_stamp) if message.valid else math.nan,
            stamp_ns(message.reference_stamp) if message.valid else math.nan, int(message.valid))


def create_figures(data, predicted=True, allocation=None):
    import matplotlib.pyplot as plt

    data = np.atleast_2d(data)
    time = data[:, 0]
    errors, axes = plt.subplots(3, 1, sharex=True, figsize=(11, 8), layout="constrained")
    errors.canvas.manager.set_window_title("Tracking errors")
    errors.suptitle("Controller errors (predicted odometry at selected reference time)" if predicted
                    else "Body-frame tracking errors (reference selected at odometry timestamp)")
    for ax, column, label, scale in zip(axes, (7, 8, 9), ("ex [m]", "ey [m]", "yaw error [deg]"), (1, 1, 180 / math.pi)):
        ax.plot(time, data[:, column] * scale)
        ax.axhline(0, color="black", linewidth=0.6)
        ax.set_ylabel(label)
        ax.grid(True, alpha=0.3)
    axes[-1].set_xlabel("Elapsed time [s]")

    # Four rows of state/input pairs, then one full-width allocation state row.
    # The full-width row has its own horizontal scale, so the last pair row keeps
    # its time labels too.
    states = plt.figure(figsize=(14, 13), layout="constrained")
    grid = states.add_gridspec(5, 2)
    axes = np.empty((4, 2), dtype=object)
    for i in range(4):
        for j in range(2):
            axes[i, j] = states.add_subplot(grid[i, j], sharex=axes[0, 0] if i or j else None)
            if i < 3:
                axes[i, j].tick_params(labelbottom=False)
    allocation_axis = states.add_subplot(grid[4, :], sharex=axes[0, 0])
    states.canvas.manager.set_window_title("Vehicle states and control inputs")
    states.suptitle("Controller snapshot: predicted pose and body-frame velocities" if predicted
                    else "Vehicle states and body-frame velocity commands")
    for i, (state_label, input_label) in enumerate(zip(("x [m]", "y [m]", "yaw [deg]"), ("vx [m/s]", "vy [m/s]", "omega [deg/s]"))):
        scale = 180 / math.pi if i == 2 else 1
        axes[i, 0].plot(time, data[:, 1 + i] * scale, label="predicted" if predicted else "actual")
        axes[i, 0].plot(time, data[:, 4 + i] * scale, "--", label="reference")
        axes[i, 0].set_ylabel(state_label)
        axes[i, 1].plot(time, data[:, 16 + i] * scale, label="command")
        axes[i, 1].plot(time, data[:, 10 + i] * scale, label="measured at odom time" if predicted else "actual", alpha=0.7)
        axes[i, 1].plot(time, data[:, 13 + i] * scale, "--", label="reference")
        axes[i, 1].set_ylabel(input_label)
    # Motion direction chi = yaw + atan2(vy, vx) and its rate chi_dot = omega + beta_dot.
    # beta_dot is differenced on the reference stamps when recorded, the control
    # time otherwise (old CSVs).
    deg = 180 / math.pi
    has_stamps = data.shape[1] > 21
    reference_time = data[:, 21] * 1e-9 if has_stamps else time
    axes[3, 0].plot(time, motion_heading(data[:, 3], data[:, 10], data[:, 11]) * deg,
                    label="predicted" if predicted else "actual")
    axes[3, 0].plot(time, motion_heading(data[:, 6], data[:, 13], data[:, 14]) * deg,
                    label="reference")
    axes[3, 0].set_ylabel("chi [deg]")
    # command/measured chi_rate는 계단형 신호의 수치 미분이라 스파이크만 커서 reference만 그린다.
    axes[3, 1].plot(time, motion_heading_rate(reference_time, data[:, 15], data[:, 13],
                                              data[:, 14]) * deg,
                    label="reference")
    axes[3, 1].set_ylabel("chi_rate [deg/s]")
    # Which allocation profile produced the command at each control time, as one
    # state graph: no allocation (stops, holds and turns), Vy-priority, Vx-priority.
    allocation_axis.set_ylabel("allocation")
    allocation_axis.set_ylim(-0.4, 2.4)
    allocation_axis.set_yticks(list(ALLOCATION_STATES.values()),
                               ["none (stop/hold/turn)", "Vy-priority", "Vx-priority"])
    if allocation is not None and has_stamps:
        state = [ALLOCATION_STATES.get(profile, math.nan)
                 for profile in allocation_profiles(data[:, 19], allocation)]
        allocation_axis.plot(time, state, drawstyle="steps-post")
    else:
        allocation_axis.text(0.5, 0.5, "no allocation record", transform=allocation_axis.transAxes,
                             ha="center", va="center")
    for ax in axes.flat:
        ax.legend()
        ax.grid(True, alpha=0.3)
    allocation_axis.grid(True, alpha=0.3)
    allocation_axis.set_xlabel("Elapsed time [s]")
    return errors, states


def create_projection_figure(data, projection):
    """How much the planner's yaw-rate projection acted on the executed commands.

    `data` are tracking.csv rows, `projection` the matching projection.csv rows."""
    import matplotlib.pyplot as plt

    data = np.atleast_2d(data)
    projection = np.atleast_2d(projection)
    time = data[:, 0]
    deg = 180 / math.pi
    commanded = data[:, 15] * deg
    delta = projection[:, 1] * deg
    score = projection[:, 2]
    # Recordings before the allowed yaw rate was logged have no exceedance panel.
    has_limit = projection.shape[1] > 3
    figure, axes = plt.subplots(4 if has_limit else 3, 1, sharex=True,
                                figsize=(12, 12 if has_limit else 9), layout="constrained")
    figure.canvas.manager.set_window_title("Yaw-rate projection")
    with np.errstate(invalid="ignore"):
        active = np.abs(delta) > 1.0e-3
        outside = score > 1.0 + 1.0e-6
    known = np.isfinite(delta)
    share = 100.0 * np.count_nonzero(active) / max(np.count_nonzero(known), 1)
    summary = (f"Planner yaw-rate projection: active {share:.1f}% of recorded time, "
               f"max |change| {np.nanmax(np.abs(delta), initial=0.0):.2f} deg/s, "
               f"commands outside the vehicle set {np.count_nonzero(outside)} samples")
    if has_limit:
        # How far the commanded yaw rate goes past what the vehicle keeps at
        # that slip angle; zero inside. An unprojected (spot-turn) command has
        # an infinite limit and never exceeds.
        limit = projection[:, 3] * deg
        with np.errstate(invalid="ignore"):
            exceedance = np.maximum(np.abs(commanded) - limit, 0.0)
        summary += f"\nmax yaw-rate exceedance {np.nanmax(exceedance, initial=0.0):.2f} deg/s"
    figure.suptitle(summary)

    axes[0].plot(time, commanded + delta, color="0.6", label="without projection (reference + change)")
    axes[0].plot(time, commanded, label="reference (projected)")
    axes[0].set_ylabel("yaw rate [deg/s]")
    axes[0].legend(loc="upper right")

    axes[1].plot(time, delta, drawstyle="steps-post")
    axes[1].axhline(0, color="black", linewidth=0.6)
    axes[1].set_ylabel("projection change\n(without - with) [deg/s]")

    # A yaw rate at standstill scores infinity (the vehicle zeroes it): show it at the top.
    axes[2].plot(time, np.clip(score, 0.0, 2.0), drawstyle="steps-post")
    axes[2].axhline(1.0, color="black", linestyle="--", linewidth=1.0, label="vehicle limit")
    axes[2].set_ylim(0.0, 2.05)
    axes[2].set_ylabel("|gamma|/delta_max\n+ |r|/(kappa_max V)")
    axes[2].legend(loc="upper right")
    if has_limit:
        axes[3].plot(time, exceedance, drawstyle="steps-post", color="tab:red")
        axes[3].axhline(0, color="black", linewidth=0.6)
        axes[3].set_ylabel("yaw-rate exceedance\n|r| - allowed [deg/s]")
    axes[-1].set_xlabel("Elapsed time [s]")
    for ax in axes:
        ax.grid(True, alpha=0.3)
        for start, end in _spans(time, active):
            ax.axvspan(start, end, color="tab:orange", alpha=0.12, linewidth=0)
    return figure


def _spans(time, mask):
    """(start, end) times of consecutive True runs in `mask`."""
    spans = []
    start = None
    for t, on in zip(time, mask):
        if on and start is None:
            start = t
        elif not on and start is not None:
            spans.append((start, t))
            start = None
    if start is not None:
        spans.append((start, time[-1]))
    return spans


def show_recording(path):
    import matplotlib

    interactive = bool(os.environ.get("DISPLAY"))
    matplotlib.use("TkAgg" if interactive else "Agg")
    import matplotlib.pyplot as plt

    with path.open() as file:
        predicted = "predicted_x" in next(csv.reader(file))
    allocation = None
    allocation_path = path.parent / ALLOCATION_FILE
    if allocation_path.exists():
        with allocation_path.open() as file:
            allocation = [(int(row["start_ns"]), row["allocation_profile"])
                          for row in csv.DictReader(file)]
    data = np.loadtxt(path, delimiter=",", skiprows=1)
    figures = create_figures(data, predicted=predicted, allocation=allocation)
    names = ["tracking_errors.png", "tracking_states_inputs.png"]
    projection_path = path.parent / PROJECTION_FILE
    if projection_path.exists():
        projection = np.loadtxt(projection_path, delimiter=",", skiprows=1, ndmin=2)
        if len(projection) == len(np.atleast_2d(data)):
            figures = (*figures, create_projection_figure(data, projection))
            names.append("tracking_projection.png")
    for figure, name in zip(figures, names):
        figure.savefig(path.parent / name, dpi=140)
    if interactive:
        plt.show()
    plt.close("all")


def main(args=None):
    if len(sys.argv) == 3 and sys.argv[1] == "--show":
        show_recording(Path(sys.argv[2]))
        return

    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from simp_planner_msgs.msg import TrackingControl, TrackingDiagnostics
    from std_msgs.msg import String

    rclpy.init(args=args)
    node = Node("tracking_shutdown_plot")
    directory = Path(node.declare_parameter("output_dir", "simp_planner_debug").value).expanduser()
    # Must match the planner's: tracking trajectory points are one command period apart.
    period_ns = round(1.0e9 / float(node.declare_parameter("command_frequency_hz", 100.0).value))
    directory = directory / ("tracking_" + datetime.now().strftime("%Y%m%d_%H%M%S_%f"))
    directory.mkdir(parents=True, exist_ok=True)
    path = directory / "tracking.csv"
    file = path.open("w", newline="")
    writer = csv.writer(file)
    writer.writerow(COLUMNS)
    allocation_file = (directory / ALLOCATION_FILE).open("w", newline="")
    allocation_writer = csv.writer(allocation_file)
    allocation_writer.writerow(ALLOCATION_COLUMNS)
    projection_file = (directory / PROJECTION_FILE).open("w", newline="")
    projection_writer = csv.writer(projection_file)
    projection_writer.writerow(PROJECTION_COLUMNS)
    diagnostics = {}
    origin_ns = None
    count = 0

    def receive_control(message):
        nonlocal origin_ns, count
        if origin_ns is None:
            origin_ns = stamp_ns(message.header.stamp)
        writer.writerow(sample_row(message, origin_ns))
        values = (projection_at(diagnostics, stamp_ns(message.reference_stamp), period_ns)
                  if message.valid else (math.nan,) * (len(PROJECTION_COLUMNS) - 1))
        projection_writer.writerow((stamp_ns(message.header.stamp), *values))
        count += 1

    def receive_diagnostics(message):
        diagnostics[stamp_ns(message.header.stamp)] = (
            [point.projection_yaw_rate_delta for point in message.points],
            [point.projection_score for point in message.points],
            [point.projection_yaw_rate_limit for point in message.points])
        while len(diagnostics) > PROJECTION_HISTORY:
            diagnostics.pop(min(diagnostics))

    def receive_status(message):
        event = allocation_event(message.data)
        if event is not None:
            allocation_writer.writerow(event)

    node.create_subscription(TrackingControl, "/tracker/control", receive_control, qos_profile_sensor_data)
    # Every committed trajectory's start and allocation profile is needed, not
    # only the latest status, so keep a queue.
    node.create_subscription(String, "/planner/status", receive_status,
                             QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE))
    # Every trajectory's diagnostics are needed for the lookup, so keep a queue.
    node.create_subscription(TrackingDiagnostics, "/planner/tracking_diagnostics",
                             receive_diagnostics,
                             QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE))
    node.get_logger().info(f"Ctrl+C 후 추종 플롯 표시 / 기록: {path}")
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # 터미널과 launch가 SIGINT를 연달아 보내도 저장/창 실행은 끝낸다.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        file.close()
        allocation_file.close()
        projection_file.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if count:
            # launch의 종료 신호와 분리해 사용자가 창을 닫을 때까지 유지한다.
            with (directory / "viewer.log").open("w") as log:
                subprocess.Popen(
                    [sys.executable, "-m", "simp_planner_tools.tracking_shutdown_plot", "--show", str(path.resolve())],
                    start_new_session=True, stdin=subprocess.DEVNULL, stdout=log, stderr=log,
                )


if __name__ == "__main__":
    main()
