"""종료 후 추종 오차와 상태/입력을 두 Matplotlib 창으로 표시한다."""
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
    "time_s", "x", "y", "yaw", "xr", "yr", "yaw_r", "ex", "ey", "etheta",
    "vx", "vy", "omega", "vx_ref", "vy_ref", "omega_ref", "vx_cmd", "vy_cmd", "omega_cmd",
)


def reference_at(trajectory, stamp_ns, frame):
    if trajectory is None or not frame or trajectory.header.frame_id != frame:
        return None
    period = trajectory.sample_period
    if period.sec < 0 or period.nanosec >= 1000000000:
        return None
    dt = period.sec * 1000000000 + period.nanosec
    stamp = trajectory.header.stamp
    elapsed = stamp_ns - (stamp.sec * 1000000000 + stamp.nanosec)
    if dt <= 0 or elapsed < 0:
        return None
    index = elapsed // dt + (elapsed % dt != 0)
    return trajectory.points[index] if index < len(trajectory.points) else None


def sample_row(odom, trajectory, command, origin_ns):
    stamp = odom.header.stamp
    stamp_ns = stamp.sec * 1000000000 + stamp.nanosec
    pose = odom.pose.pose
    q = pose.orientation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    x, y = pose.position.x, pose.position.y
    reference = reference_at(trajectory, stamp_ns, odom.header.frame_id)
    xr, yr, yaw_r, ex, ey, etheta, vxr, vyr, wr = [math.nan] * 9
    if reference is not None:
        xr, yr, yaw_r = reference.x, reference.y, reference.body_yaw
        dx, dy = xr - x, yr - y
        ex = math.cos(yaw) * dx + math.sin(yaw) * dy
        ey = -math.sin(yaw) * dx + math.cos(yaw) * dy
        etheta = math.remainder(yaw_r - yaw, 2 * math.pi)
        vxr, vyr, wr = reference.vx, reference.vy, reference.yaw_rate
    velocity = odom.twist.twist
    cmd = (command.linear.x, command.linear.y, command.angular.z) if command else (math.nan,) * 3
    return ((stamp_ns - origin_ns) * 1e-9, x, y, yaw, xr, yr, yaw_r, ex, ey, etheta,
            velocity.linear.x, velocity.linear.y, velocity.angular.z, vxr, vyr, wr, *cmd)


def create_figures(data):
    import matplotlib.pyplot as plt

    data = np.atleast_2d(data)
    time = data[:, 0]
    errors, axes = plt.subplots(3, 1, sharex=True, figsize=(11, 8), layout="constrained")
    errors.canvas.manager.set_window_title("Tracking errors")
    errors.suptitle("Body-frame tracking errors (reference selected at odometry timestamp)")
    for ax, column, label, scale in zip(axes, (7, 8, 9), ("ex [m]", "ey [m]", "yaw error [deg]"), (1, 1, 180 / math.pi)):
        ax.plot(time, data[:, column] * scale)
        ax.axhline(0, color="black", linewidth=0.6)
        ax.set_ylabel(label)
        ax.grid(True, alpha=0.3)
    axes[-1].set_xlabel("Elapsed time [s]")

    states, axes = plt.subplots(3, 2, sharex=True, figsize=(14, 9), layout="constrained")
    states.canvas.manager.set_window_title("Vehicle states and control inputs")
    states.suptitle("Vehicle states and body-frame velocity commands")
    for i, (state_label, input_label) in enumerate(zip(("x [m]", "y [m]", "yaw [deg]"), ("vx [m/s]", "vy [m/s]", "omega [deg/s]"))):
        scale = 180 / math.pi if i == 2 else 1
        axes[i, 0].plot(time, data[:, 1 + i] * scale, label="Actual")
        axes[i, 0].plot(time, data[:, 4 + i] * scale, "--", label="Planner reference")
        axes[i, 0].set_ylabel(state_label)
        axes[i, 1].plot(time, data[:, 16 + i] * scale, label="Tracker command")
        axes[i, 1].plot(time, data[:, 10 + i] * scale, label="Actual", alpha=0.7)
        axes[i, 1].plot(time, data[:, 13 + i] * scale, "--", label="Planner reference")
        axes[i, 1].set_ylabel(input_label)
        for ax in axes[i]:
            ax.legend()
            ax.grid(True, alpha=0.3)
    for ax in axes[-1]:
        ax.set_xlabel("Elapsed time [s]")
    return errors, states


def show_recording(path):
    import matplotlib

    interactive = bool(os.environ.get("DISPLAY"))
    matplotlib.use("TkAgg" if interactive else "Agg")
    import matplotlib.pyplot as plt

    figures = create_figures(np.loadtxt(path, delimiter=",", skiprows=1))
    for figure, name in zip(figures, ("tracking_errors.png", "tracking_states_inputs.png")):
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
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from simp_planner_msgs.msg import TrackingTrajectory

    rclpy.init(args=args)
    node = Node("tracking_shutdown_plot")
    directory = Path(node.declare_parameter("output_dir", "simp_planner_debug").value)
    directory = directory / ("tracking_" + datetime.now().strftime("%Y%m%d_%H%M%S_%f"))
    directory.mkdir(parents=True, exist_ok=True)
    path = directory / "tracking.csv"
    file = path.open("w", newline="")
    writer = csv.writer(file)
    writer.writerow(COLUMNS)
    trajectory, command, origin_ns = None, None, None
    count = 0

    def receive_trajectory(message):
        nonlocal trajectory
        stamp = message.header.stamp
        if trajectory is None or (stamp.sec, stamp.nanosec) > (
                trajectory.header.stamp.sec, trajectory.header.stamp.nanosec):
            trajectory = message

    def receive_command(message):
        nonlocal command
        command = message

    def receive_odom(message):
        nonlocal origin_ns, count
        if origin_ns is None:
            origin_ns = message.header.stamp.sec * 1000000000 + message.header.stamp.nanosec
        writer.writerow(sample_row(message, trajectory, command, origin_ns))
        count += 1

    node.create_subscription(TrackingTrajectory, "/planner/tracking_trajectory", receive_trajectory, 1)
    node.create_subscription(Twist, "/cmd_vel", receive_command, 10)
    node.create_subscription(Odometry, "/odom", receive_odom, qos_profile_sensor_data)
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
