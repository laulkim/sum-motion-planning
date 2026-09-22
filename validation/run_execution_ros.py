#!/usr/bin/env python3
"""Exercise ROS-time execution without planner/vehicle dependencies, on a private domain.
Build and source install/setup.bash first. No physical vehicle topics are used.
"""
import argparse
import os
from pathlib import Path
import signal
import subprocess
import time


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--domain', type=int, default=163)
    parser.add_argument('--output', type=Path, default=Path('/tmp/simp_execution_ros'))
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    os.environ['ROS_DOMAIN_ID'] = str(args.domain)
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    os.environ['ROS_LOG_DIR'] = str(args.output / 'ros_logs')
    import rclpy
    from ament_index_python.packages import get_package_prefix
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from rosgraph_msgs.msg import Clock
    from simp_planner_msgs.msg import Trajectory, TrajectoryPoint, DriveModeState, ExecutionStatus, ExecutedCommand
    rclpy.init()
    node = rclpy.create_node('execution_protocol_test')
    clock_pub = node.create_publisher(Clock, '/clock', 10)
    plan_pub = node.create_publisher(Trajectory, '/planner/trajectory', 10)
    mode_pub = node.create_publisher(DriveModeState, '/vehicle/drive_mode_state',
                                    QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    statuses, commands = [], []
    node.create_subscription(ExecutionStatus, '/tracker/execution_status', statuses.append, 100)
    node.create_subscription(ExecutedCommand, '/planner/executed_command', commands.append, 200)
    binary = Path(get_package_prefix('simp_tracker')) / 'lib/simp_tracker/tracker_node'
    log = (args.output / 'tracker.log').open('w')
    process = subprocess.Popen([str(binary), '--ros-args', '-p', 'use_sim_time:=true'], stdout=log, stderr=log)
    def pump(seconds=.08):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.005)
    def clock(t):
        msg = Clock(); ns = round(t * 1e9)
        msg.clock.sec, msg.clock.nanosec = divmod(ns, 10**9)
        clock_pub.publish(msg)
        mode = DriveModeState(); mode.status = DriveModeState.STATUS_READY
        mode_pub.publish(mode)
        pump()
    def trajectory(plan_id, base, epoch, start, x, speed=1.):
        msg = Trajectory(); msg.header.frame_id = 'map'; msg.session_id = 'protocol-test'
        msg.plan_id = plan_id; msg.base_plan_id = base; msg.execution_epoch = epoch
        msg.start_time.sec, msg.start_time.nanosec = divmod(round(start * 1e9), 10**9)
        msg.duration = 4.; msg.trajectory_type = 'ACTIVE_PLAN'
        msg.fallback_deceleration = 1.; msg.fallback_jerk = .8
        for t in (0., 4.):
            p = TrajectoryPoint(); p.time = t; p.x = x + speed * t
            p.command.vx = speed; p.command.planned_speed = speed
            msg.points.append(p)
        return msg
    try:
        deadline = time.monotonic() + 5
        while plan_pub.get_subscription_count() == 0 and time.monotonic() < deadline:
            pump()
        assert plan_pub.get_subscription_count(), 'tracker discovery failed'
        # Discover /clock as well as the trajectory endpoint before accepting
        # a plan; otherwise the initial 0 -> 100 clock initialization arrives late.
        for _ in range(10):
            clock(100.)
            if clock_pub.get_subscription_count():
                break
        clock(100.1); clock(100.2)
        assert commands and commands[-1].header.stamp.sec == 100, 'ROS clock not initialized'
        a = trajectory(1, 0, 1, 100.5, 0.)
        plan_pub.publish(a); pump(.2)
        assert statuses[-1].acceptance == 'ACCEPTED', statuses[-1].acceptance
        clock(100.4)
        assert not commands or commands[-1].vx == 0., 'executed before start'
        clock(100.5); clock(100.59)
        assert abs(commands[-1].vx - 1.) < 1e-9, '90ms no-message gap interrupted execution'
        assert abs(commands[-1].segment_start_x - .09) < 1e-6, 'wrong ROS-time sample'
        before = len(commands); pump(.2)
        assert len(commands) == before, 'ROS timer advanced while simulated time paused'
        b = trajectory(2, 1, 1, 101., .5)
        plan_pub.publish(b); pump()
        assert statuses[-1].acceptance == 'ACCEPTED', statuses[-1].acceptance
        replacement = trajectory(3, 1, 2, 100.8, .3)
        plan_pub.publish(replacement); pump()
        assert statuses[-1].acceptance == 'ACCEPTED', statuses[-1].acceptance
        plan_pub.publish(b); pump()
        assert statuses[-1].acceptance == 'OLD_EPOCH', statuses[-1].acceptance
        clock(100.8)
        assert commands[-1].plan_id == 3, 'wrong active plan'
        plan_pub.publish(replacement); pump()
        assert statuses[-1].acceptance == 'DUPLICATE', 'duplicate not idempotent'
        # No further plans: expiry must brake and retain its final position.
        for t in (101.8, 102.8, 103.8, 104.8, 105.8, 106.8, 107.8):
            clock(t)
        assert abs(commands[-1].vx) < 1e-8, 'expiry did not brake'
        assert statuses[-1].execution_state == 'SAFETY_STOP_EXPIRED'
        clock(99.)
        clock(99.1)
        assert 'CLOCK_RESET' in statuses[-1].execution_state, 'clock reset not latched'
        print('PASS: scheduled activation, 90ms gap, ROS pause, supersession, duplicate, expiry, clock reset')
    finally:
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill(); process.wait()
        log.close(); node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
