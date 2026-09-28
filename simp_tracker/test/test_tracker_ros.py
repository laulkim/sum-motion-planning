"""실제 ROS 노드의 독립 100 Hz 제어, 모드 전달, 만료 정지를 확인한다."""
import os
import statistics
import subprocess
import sys
import tempfile
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile
from simp_planner_msgs.msg import TrackingTrajectory, TrackingTrajectoryPoint
from std_msgs.msg import UInt8


def main():
    os.environ["ROS_DOMAIN_ID"] = str(150 + os.getpid() % 70)
    rclpy.init()
    node = rclpy.create_node("tracker_ros_test")
    trajectory_pub = node.create_publisher(TrackingTrajectory, "/planner/tracking_trajectory", 1)
    odom_pub = node.create_publisher(Odometry, "/odom", 1)
    commands, modes = [], []
    node.create_subscription(Twist, "/cmd_vel", lambda msg: commands.append((time.monotonic(), msg)), 100)
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(UInt8, "/vehicle/drive_mode_command", lambda msg: modes.append(msg.data), qos)

    def wait_for(predicate, timeout=5.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            assert process.poll() is None, "tracker terminated"
            rclpy.spin_once(node, timeout_sec=0.005)
            if predicate():
                return
        raise AssertionError("tracker output timeout")

    def stopped(command):
        return command.linear.x == command.linear.y == command.angular.z == 0.0

    with tempfile.TemporaryFile(mode="w+") as log:
        process = subprocess.Popen([sys.argv[1]], stdout=log, stderr=log)
        try:
            wait_for(lambda: len(commands) >= 15 and trajectory_pub.get_subscription_count() > 0
                     and odom_pub.get_subscription_count() > 0)
            assert all(stopped(msg) for _, msg in commands)
            odom = Odometry()
            odom.header.frame_id = "odom"
            odom.pose.pose.orientation.w = 1.0
            odom_pub.publish(odom)
            trajectory = TrackingTrajectory()
            trajectory.header.frame_id = "odom"
            start = node.get_clock().now().nanoseconds + 120000000
            trajectory.header.stamp.sec, trajectory.header.stamp.nanosec = divmod(start, 1000000000)
            trajectory.sample_period.nanosec = 10000000
            trajectory.points = [TrackingTrajectoryPoint(x=1.0, requested_mode=2) for _ in range(31)]
            trajectory_pub.publish(trajectory)
            count = len(commands)
            wait_for(lambda: len(commands) >= count + 4)
            assert all(stopped(msg) for _, msg in commands[count:]), "command before trajectory start"
            wait_for(lambda: commands[-1][1].linear.x == 3.0 and modes and modes[-1] == 2)
            # 입력은 한 번만 보냈어도 타이머가 계속 제어하고 horizon 종료 후 정지한다.
            wait_for(lambda: node.get_clock().now().nanoseconds > start + 350000000
                     and stopped(commands[-1][1]))
            valid = [t for t, msg in commands if msg.linear.x == 3.0]
            assert len(valid) >= 15, "control loop depends on input callbacks"
            median = statistics.median(b - a for a, b in zip(valid, valid[1:]))
            assert 0.006 < median < 0.016, f"control median period={median}"
            count = len(commands)
            wait_for(lambda: len(commands) >= count + 10)
            assert all(stopped(msg) for _, msg in commands[count:]), "stale command persisted"
            print(f"tracker ROS passed: {len(valid)} controls from one input, median {median:.4f}s")
        except Exception:
            log.seek(0)
            print(log.read(), file=sys.stderr)
            raise
        finally:
            process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
