"""종료 후 추종 오차와 상태/입력을 세 Matplotlib 창으로 표시한다."""
from __future__ import annotations

import csv
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


def stamp_ns(stamp):
    return stamp.sec * 1000000000 + stamp.nanosec


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


def create_figures(data, predicted=True):
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

    states, axes = plt.subplots(3, 2, sharex=True, figsize=(14, 9), layout="constrained")
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
        for ax in axes[i]:
            ax.legend()
            ax.grid(True, alpha=0.3)
    for ax in axes[-1]:
        ax.set_xlabel("Elapsed time [s]")
    # Direct differences in the shared world/odom frame, including standstill.
    # Display only: controller feedback remains in its original body frame.
    yaw_difference = data[:, 6] - data[:, 3]
    world_errors = (
        data[:, 4] - data[:, 1],
        data[:, 5] - data[:, 2],
        np.arctan2(np.sin(yaw_difference), np.cos(yaw_difference)),
    )
    world_frame, axes = plt.subplots(3, 1, sharex=True, figsize=(11, 8), layout="constrained")
    world_frame.canvas.manager.set_window_title("World-frame tracking errors")
    world_frame.suptitle("World-frame errors (reference minus predicted pose)" if predicted
                            else "World-frame errors (reference minus measured pose)")
    for ax, values, label, scale in zip(
        axes, world_errors, ("x_ref - x_predicted [m]" if predicted else "x_ref - x_measured [m]",
                             "y_ref - y_predicted [m]" if predicted else "y_ref - y_measured [m]",
                             "yaw error [deg]"),
        (1, 1, 180 / math.pi),
    ):
        ax.plot(time, values * scale)
        ax.axhline(0, color="black", linewidth=0.6)
        ax.set_ylabel(label)
        ax.grid(True, alpha=0.3)
    axes[-1].set_xlabel("Elapsed time [s]")
    return errors, states, world_frame


def show_recording(path):
    import matplotlib

    interactive = bool(os.environ.get("DISPLAY"))
    matplotlib.use("TkAgg" if interactive else "Agg")
    import matplotlib.pyplot as plt

    with path.open() as file:
        predicted = "predicted_x" in next(csv.reader(file))
    figures = create_figures(np.loadtxt(path, delimiter=",", skiprows=1), predicted=predicted)
    for figure, name in zip(figures, ("tracking_errors.png", "tracking_states_inputs.png", "tracking_world_errors.png")):
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
    from rclpy.qos import qos_profile_sensor_data
    from simp_planner_msgs.msg import TrackingControl

    rclpy.init(args=args)
    node = Node("tracking_shutdown_plot")
    directory = Path(node.declare_parameter("output_dir", "simp_planner_debug").value).expanduser()
    directory = directory / ("tracking_" + datetime.now().strftime("%Y%m%d_%H%M%S_%f"))
    directory.mkdir(parents=True, exist_ok=True)
    path = directory / "tracking.csv"
    file = path.open("w", newline="")
    writer = csv.writer(file)
    writer.writerow(COLUMNS)
    origin_ns = None
    count = 0

    def receive_control(message):
        nonlocal origin_ns, count
        if origin_ns is None:
            origin_ns = stamp_ns(message.header.stamp)
        writer.writerow(sample_row(message, origin_ns))
        count += 1

    node.create_subscription(TrackingControl, "/tracker/control", receive_control, qos_profile_sensor_data)
    node.get_logger().info(f"Ctrl+C 후 추종 플롯 표시 / 기록: {path}")
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # 터미널과 launch가 SIGINT를 연달아 보내도 저장/창 실행은 끝낸다.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        file.close()
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
