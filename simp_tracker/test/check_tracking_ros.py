"""Manual integration check: source install/setup.bash and use an unused ROS_DOMAIN_ID.

Runs the real Tracker and Simulator with a timed sideways trajectory and initial
pose error, then checks mode switching, passthrough and command-loss stopping.
"""
import math
import os
import signal
import subprocess
import time

import rclpy
from geometry_msgs.msg import Twist, Vector3Stamped
from nav_msgs.msg import Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String, UInt8
from simp_planner_msgs.msg import DriveModeState, ExecutedCommand, Trajectory, TrajectoryPoint


def stop(process):
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=5)


rclpy.init()
node = rclpy.create_node("tracking_integration_check")
latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
trajectory_pub = node.create_publisher(Trajectory, "/planner/trajectory", latched)
command_pub = node.create_publisher(ExecutedCommand, "/planner/executed_command", 1)
mode_pub = node.create_publisher(UInt8, "/vehicle/drive_mode_command", latched)
errors, odometry, modes, commands, statuses = [], [], [], [], []
node.create_subscription(Vector3Stamped, "/tracker/error", errors.append, 10)
node.create_subscription(Odometry, "/odom", odometry.append, 10)
node.create_subscription(DriveModeState, "/vehicle/drive_mode_state", modes.append, latched)
node.create_subscription(Twist, "/cmd_vel", commands.append, 10)
node.create_subscription(String, "/tracker/status", lambda msg: statuses.append(msg.data), latched)
nominal = ExecutedCommand(plan_id=1, execution_state="ACTIVE_PLAN")
nominal.header.frame_id = "base_link"


def spin(seconds, heartbeat=True):
    until = time.monotonic() + seconds
    while time.monotonic() < until:
        if heartbeat:
            nominal.header.stamp = node.get_clock().now().to_msg()
            command_pub.publish(nominal)
        rclpy.spin_once(node, timeout_sec=0.01)


with open("/tmp/simp_tracker_ros_check.log", "w") as log:
    simulator = subprocess.Popen([
        "ros2", "run", "planar_velocity_sim", "planar_velocity_sim_node", "--ros-args",
        "-p", "initial_x:=0.3", "-p", "initial_y:=-0.4", "-p", "initial_yaw:=0.9",
        "-p", "initial_drive_mode:=2", "-p", "mode_transition_duration_sec:=0.1",
    ], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    tracker = subprocess.Popen([
        "ros2", "run", "simp_tracker", "tracking_controller_node", "--ros-args",
        "-p", "control_frequency_hz:=50.0",
    ], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    try:
        ready_deadline = time.monotonic() + 10.0
        while time.monotonic() < ready_deadline:
            spin(0.1)
            if odometry and modes and commands and trajectory_pub.get_subscription_count():
                break
        assert odometry and modes and commands, "ROS nodes did not become ready"
        assert node.count_publishers("/cmd_vel") == 1, "multiple command publishers"
        trajectory = Trajectory(plan_id=1)
        trajectory.header.frame_id = "odom"
        trajectory.header.stamp = node.get_clock().now().to_msg()
        trajectory.start_time = (node.get_clock().now() + rclpy.duration.Duration(seconds=0.3)).to_msg()
        theta, speed = 0.4, 0.3
        for i in range(81):
            t = i * 0.1
            trajectory.points.append(TrajectoryPoint(
                time_from_start=t, x=-math.sin(theta) * speed * t,
                y=math.cos(theta) * speed * t, yaw=theta, vx=0.0, vy=speed, mode=2,
            ))
        trajectory_pub.publish(trajectory)
        spin(4.0)
        assert len(errors) > 50 and "TRACKING" in statuses
        norms = [math.sqrt(e.vector.x**2 + e.vector.y**2 + e.vector.z**2) for e in errors]
        assert norms[0] > 0.4, norms[0]
        assert max(norms[-10:]) < 0.025, norms[-10:]
        print(f"Tracking PASS: initial error norm={norms[0]:.4f}, final={norms[-1]:.4f}")

        spin(0.5, heartbeat=False)
        assert "WAIT_COMMAND" in statuses
        assert abs(commands[-1].linear.x) + abs(commands[-1].linear.y) + abs(commands[-1].angular.z) == 0.0
        nominal.execution_state = "MODE_WAIT"
        spin(0.1)
        mode_pub.publish(UInt8(data=4))
        spin(0.5)
        assert modes[-1].current_mode == 4 and modes[-1].status == 1
        nominal.execution_state = "SPOT_TURN_ROTATING"
        nominal.yaw_rate = 0.3
        spin(0.5)
        assert "SPOT_TURN_ROTATING" in statuses
        assert abs(odometry[-1].twist.twist.angular.z - 0.3) < 1e-9
        stop(tracker)
        spin(0.8)
        assert abs(odometry[-1].twist.twist.angular.z) < 1e-9
        print("Integration PASS: mode change, spot-turn passthrough, heartbeat loss and simulator watchdog")
    finally:
        stop(tracker)
        stop(simulator)
        node.destroy_node()
        rclpy.try_shutdown()
