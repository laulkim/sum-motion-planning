"""Controller/odom/mode → CSV → Ctrl+C → plots and turn summary, with real ROS."""
import csv
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import rclpy
from ament_index_python.packages import get_package_prefix
from nav_msgs.msg import Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from simp_planner_msgs.msg import DriveModeState, TrackingControl, TrackingTrajectory, TrackingTrajectoryPoint


def test_controller_recording_survives_sigint(tmp_path, monkeypatch):
    monkeypatch.setenv("ROS_DOMAIN_ID", str(150 + os.getpid() % 70))
    monkeypatch.delenv("DISPLAY", raising=False)
    rclpy.init()
    node = rclpy.create_node("tracking_shutdown_test")
    trajectory_pub = node.create_publisher(
        TrackingTrajectory, "/planner/tracking_trajectory",
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    odom_pub = node.create_publisher(Odometry, "/odom", 1)
    mode_pub = node.create_publisher(
        DriveModeState, "/vehicle/drive_mode_state",
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    snapshots = []
    node.create_subscription(TrackingControl, "/tracker/control", snapshots.append, qos_profile_sensor_data)
    executable = Path(get_package_prefix("simp_controller")) / "lib/simp_controller/tracking_controller_node"
    processes = []
    with (tmp_path / "processes.log").open("w") as log:
        try:
            for command in ([str(executable), "--ros-args", "-p", "kx:=0.3",
                             "-p", "ky:=0.2", "-p", "ktheta:=0.2"],
                            [sys.executable, "-m", "simp_planner_tools.tracking_shutdown_plot",
                             "--ros-args", "-p", f"output_dir:={tmp_path}"]):
                processes.append(subprocess.Popen(command, stdout=log, stderr=log, start_new_session=True))

            odom = Odometry()
            odom.header.frame_id = "odom"
            odom.child_frame_id = "base_link"
            odom.pose.pose.orientation.w = 1.0

            def wait_for(predicate, timeout=15.0):
                deadline = time.monotonic() + timeout
                while time.monotonic() < deadline:
                    odom.header.stamp = node.get_clock().now().to_msg()
                    odom_pub.publish(odom)
                    rclpy.spin_once(node, timeout_sec=0.01)
                    if predicate():
                        return
                raise AssertionError((tmp_path / "processes.log").read_text())

            wait_for(lambda: trajectory_pub.get_subscription_count() > 0
                     and odom_pub.get_subscription_count() >= 2
                     and mode_pub.get_subscription_count() > 0
                     and node.count_subscribers("/tracker/control") >= 2)
            odom = Odometry()
            odom.header.stamp = node.get_clock().now().to_msg()
            odom.header.frame_id = "odom"
            odom.child_frame_id = "base_link"
            odom.pose.pose.orientation.w = 1.0
            odom_pub.publish(odom)
            trajectory = TrackingTrajectory()
            trajectory.header.frame_id = "odom"
            trajectory.header.stamp = node.get_clock().now().to_msg()
            trajectory.points = [TrackingTrajectoryPoint(x=1.0, y=-0.5, body_yaw=0.2) for _ in range(101)]
            trajectory_pub.publish(trajectory)
            wait_for(lambda: sum(message.valid for message in snapshots) >= 10)
            message = next(message for message in snapshots if message.valid)
            assert (message.ex, message.ey, message.etheta) == (1.0, -0.5, 0.2)
            assert message.reference.x == 1.0 and message.predicted_x == 0.0
            assert abs(message.command.linear.x - 0.3) < 1e-12
            assert abs(message.command.linear.y + 0.1) < 1e-12
            assert abs(message.command.angular.z - 0.04) < 1e-12
            wait_for(lambda: not snapshots[-1].valid and any(message.valid for message in snapshots))
            mode = DriveModeState(current_mode=4, requested_mode=4, status=1)
            mode.header.stamp = node.get_clock().now().to_msg()
            mode_pub.publish(mode)
            trajectory.header.stamp = node.get_clock().now().to_msg()
            trajectory.points = [TrackingTrajectoryPoint(x=1.0, y=-0.5, body_yaw=0.2,
                                                         yaw_rate=0.3, requested_mode=4) for _ in range(51)]
            trajectory_pub.publish(trajectory)
            wait_for(lambda: sum(message.valid and message.reference.yaw_rate == 0.3
                                 for message in snapshots) >= 10)
            wait_for(lambda: not snapshots[-1].valid)
            recorder = processes[1]
            recorder.send_signal(signal.SIGINT)
            assert recorder.wait(timeout=10) == 0
            directory = next(tmp_path.glob("tracking_*"))
            wait_for(lambda: all((directory / name).exists() for name in
                                 ("tracking_errors.png", "tracking_states_inputs.png", "tracking_world_errors.png",
                                  "spot_turn_summary.png", "spot_turn_01.png", "spot_turn_summary.csv")))
            with (directory / "tracking.csv").open() as file:
                rows = list(csv.DictReader(file))
            valid = [row for row in rows if row["valid"] == "1"]
            assert valid and float(valid[0]["ex"]) == 1.0
            assert abs(float(valid[0]["vx_cmd"]) - 0.3) < 1e-12
            assert any(row["valid"] == "0" and float(row["vx_cmd"]) == 0.0 for row in rows)
            with (directory / "spot_turn_summary.csv").open() as file:
                summary = next(csv.DictReader(file))
            assert summary["mode_verified"] == "1"
            assert int(summary["odom_samples"]) >= 2
            assert float(summary["odom_max_displacement_m"]) == 0
            assert (directory / "odometry.csv").stat().st_size > 100
            assert (directory / "drive_mode.csv").stat().st_size > 50
            assert not (directory / "viewer.log").read_text()
        finally:
            for process in processes:
                if process.poll() is None:
                    process.terminate()
                    try:
                        process.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait()
            node.destroy_node()
            rclpy.shutdown()
