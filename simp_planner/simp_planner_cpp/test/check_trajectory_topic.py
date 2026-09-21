"""Run after sourcing install/setup.bash, in an unused ROS_DOMAIN_ID.

Starts the existing s_curve simulation and checks the actual trajectory topic.
Requires simp_planner_tools and planar_velocity_sim to be built as well.
"""

import math
import os
import signal
import subprocess
import time

import rclpy
from geometry_msgs.msg import Twist, Vector3Stamped
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, DurabilityPolicy
from simp_planner_msgs.msg import Trajectory, ExecutedCommand


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


rclpy.init()
node = rclpy.create_node('trajectory_topic_check')
trajectories, commands, odometry, executed = [], [], [], []
tracking_errors = []
qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
node.create_subscription(Trajectory, '/planner/trajectory', trajectories.append, qos)
node.create_subscription(Twist, '/cmd_vel', commands.append, 10)
node.create_subscription(Odometry, '/odom', odometry.append, 10)
node.create_subscription(ExecutedCommand, '/planner/executed_command', executed.append, 100)
node.create_subscription(Vector3Stamped, '/tracker/error', tracking_errors.append, 10)
with open('/tmp/trajectory_topic_check.log', 'w') as log:
    process = subprocess.Popen(
        ['ros2', 'launch', 'simp_planner_tools', 'simulation.launch.py',
         'scenario:=s_curve', 'use_rviz:=false',
         'debug_output_dir:=/tmp/trajectory_topic_check_plots', 'save_period:=60.0'],
        stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    try:
        deadline = time.monotonic() + 35.0
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if len(trajectories) >= 3 and commands and len(odometry) >= 2 and tracking_errors:
                if any(e.plan_id == trajectories[-1].plan_id for e in executed):
                    break
        assert len(trajectories) >= 3, 'fewer than three activated plans received'
        assert node.count_publishers('/cmd_vel') == 1, 'cmd_vel must have exactly one publisher'
        assert tracking_errors, 'controller did not enter tracking'
        assert all(math.isfinite(v) for error in tracking_errors
                   for v in (error.vector.x, error.vector.y, error.vector.z))
        ids = [msg.plan_id for msg in trajectories]
        assert all(a < b for a, b in zip(ids, ids[1:])), ids
        for msg in trajectories:
            assert msg.header.frame_id == 'odom'
            assert stamp_ns(msg.start_time) <= stamp_ns(msg.header.stamp)
            assert len(msg.points) >= 2 and msg.points[0].time_from_start == 0.0
            assert all(a.time_from_start < b.time_from_start
                       for a, b in zip(msg.points, msg.points[1:]))
            assert all(math.isfinite(v) for p in msg.points
                       for v in (p.x, p.y, p.yaw, p.vx, p.vy, p.yaw_rate, p.time_from_start))
            assert all(0 <= p.mode <= 4 for p in msg.points)
        assert any(abs(c.linear.x) + abs(c.linear.y) > 0.0 for c in commands)
        assert any(abs(o.twist.twist.linear.x) + abs(o.twist.twist.linear.y) > 0.0
                   for o in odometry)
        msg = trajectories[-1]
        print(f'PASS: {len(trajectories)} plans, last id={msg.plan_id}, '
              f'{len(msg.points)} points, horizon={msg.points[-1].time_from_start:.3f}s; '
              'single cmd_vel publisher, tracking errors and simulator odometry confirmed')
    finally:
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=5)
        node.destroy_node()
        rclpy.shutdown()
