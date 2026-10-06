"""실제 ROS 노드의 독립 100 Hz 제어, 모드 전달, 만료 정지를 확인한다."""
import copy
import os
import statistics
import subprocess
import sys
import tempfile
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from simp_planner_msgs.msg import TrackingControl, TrackingTrajectory, TrackingTrajectoryPoint
from std_msgs.msg import UInt8


def main():
    os.environ["ROS_DOMAIN_ID"] = str(150 + os.getpid() % 70)
    rclpy.init()
    node = rclpy.create_node("tracker_ros_test")
    trajectory_pub = node.create_publisher(TrackingTrajectory, "/planner/tracking_trajectory", 1)
    odom_pub = node.create_publisher(Odometry, "/odom", 1)
    commands, modes = [], []
    diagnostics = []
    node.create_subscription(TrackingControl, "/tracker/control", diagnostics.append, qos_profile_sensor_data)
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

    odom = Odometry()
    odom.header.frame_id = "odom"
    odom.child_frame_id = "base_link"
    odom.pose.pose.orientation.w = 1.0
    send_odom = True
    moving_start = None

    def publish_odom():
        if not send_odom:
            return
        stamp = node.get_clock().now().nanoseconds
        if moving_start is not None:
            stamp -= 20000000
            odom.pose.pose.position.x = 1.0 + 2.0 * (stamp - moving_start) * 1.0e-9
            odom.twist.twist.linear.x = 2.0
        odom.header.stamp.sec, odom.header.stamp.nanosec = divmod(stamp, 1000000000)
        odom_pub.publish(odom)

    node.create_timer(0.01, publish_odom)
    with tempfile.TemporaryFile(mode="w+") as log:
        process = subprocess.Popen(
            [sys.argv[1], "--ros-args", "-p", "kx:=3.0", "-p", "ky:=4.0", "-p", "ktheta:=2.0"],
            stdout=log, stderr=log)
        try:
            wait_for(lambda: len(commands) >= 15 and trajectory_pub.get_subscription_count() > 0
                     and odom_pub.get_subscription_count() > 0)
            assert all(stopped(msg) for _, msg in commands)
            publish_odom()
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
            # 궤적은 한 번만 보내고 odom은 갱신한다. horizon 종료 후 정지한다.
            wait_for(lambda: node.get_clock().now().nanoseconds > start + 350000000
                     and stopped(commands[-1][1]))
            valid = [t for t, msg in commands if msg.linear.x == 3.0]
            assert len(valid) >= 15, "control loop depends on input callbacks"
            median = statistics.median(b - a for a, b in zip(valid, valid[1:]))
            assert 0.006 < median < 0.016, f"control median period={median}"
            count = len(commands)
            wait_for(lambda: len(commands) >= count + 10)
            assert all(stopped(msg) for _, msg in commands[count:]), "stale command persisted"

            # 새 참조를 저장한 뒤 늦게 도착한 과거/동일 stamp는 제어에 반영하지 않는다.
            stamp = node.get_clock().now().nanoseconds
            trajectory.header.stamp.sec, trajectory.header.stamp.nanosec = divmod(stamp, 1000000000)
            trajectory.points = [TrackingTrajectoryPoint(x=2.0, requested_mode=2) for _ in range(31)]
            trajectory_pub.publish(trajectory)
            wait_for(lambda: commands[-1][1].linear.x == 6.0)
            rejected = copy.deepcopy(trajectory)
            rejected.points = [TrackingTrajectoryPoint(x=9.0, requested_mode=4) for _ in range(31)]
            for rejected_stamp in (stamp - 10000000, stamp):
                rejected.header.stamp.sec, rejected.header.stamp.nanosec = divmod(rejected_stamp, 1000000000)
                trajectory_pub.publish(rejected)
                count = len(commands)
                wait_for(lambda: len(commands) >= count + 5)
                assert all(msg.linear.x == 6.0 for _, msg in commands[count:]), \
                    "older/equal stamp replaced latest trajectory"
                assert modes[-1] == 2, "rejected trajectory changed mode relay"
            trajectory.header.stamp.sec, trajectory.header.stamp.nanosec = divmod(stamp + 1, 1000000000)
            trajectory.points = [TrackingTrajectoryPoint(x=4.0, requested_mode=3) for _ in range(31)]
            trajectory_pub.publish(trajectory)
            wait_for(lambda: commands[-1][1].linear.x == 12.0 and modes[-1] == 3)

            # 동일 frame에서 제어하다 odom frame이 바뀌면 즉시 zero로 바뀐다.
            stamp = node.get_clock().now().nanoseconds
            trajectory.header.stamp.sec, trajectory.header.stamp.nanosec = divmod(stamp, 1000000000)
            trajectory_pub.publish(trajectory)
            odom.header.frame_id = "map"
            publish_odom()
            wait_for(lambda: stopped(commands[-1][1]))
            assert modes[-1] == 3, "frame mismatch changed mode relay"
            odom.header.frame_id = "odom"
            publish_odom()
            wait_for(lambda: commands[-1][1].linear.x == 12.0)

            # 역순/동일/미래 stamp의 odom은 최신 상태를 덮어쓰지 않는다.
            for i, offset in enumerate((-1000000, 0, 1000000000)):
                odom.pose.pose.position.x = float(i + 1)
                publish_odom()
                send_odom = False
                expected = 3.0 * (4.0 - odom.pose.pose.position.x)
                # depth=1 큐에서 정상 odom이 밀리지 않도록 먼저 반영을 확인한다.
                wait_for(lambda: commands[-1][1].linear.x == expected)
                rejected_odom = copy.deepcopy(odom)
                odom_stamp = odom.header.stamp.sec * 1000000000 + odom.header.stamp.nanosec
                rejected_odom.header.stamp.sec, rejected_odom.header.stamp.nanosec = divmod(
                    odom_stamp + offset, 1000000000)
                rejected_odom.pose.pose.position.x = -100.0
                odom_pub.publish(rejected_odom)
                count = len(commands)
                wait_for(lambda: len(commands) >= count + 2)
                assert all(msg.linear.x == expected for _, msg in commands[count:]), \
                    "invalid odom stamp replaced latest state"
                send_odom = True

            # 20ms 지연 측위도 선택한 궤적점 시각으로 예측하면 feedforward만 남는다.
            moving_start = node.get_clock().now().nanoseconds
            trajectory.header.stamp.sec, trajectory.header.stamp.nanosec = divmod(
                moving_start, 1000000000)
            trajectory.points = [TrackingTrajectoryPoint(x=1.0 + 2.0 * i * 0.01, vx=2.0,
                                                         requested_mode=2) for i in range(31)]
            trajectory_pub.publish(trajectory)
            publish_odom()
            wait_for(lambda: abs(commands[-1][1].linear.x - 2.0) < 1.0e-9)
            count = len(commands)
            wait_for(lambda: len(commands) >= count + 5)
            assert all(abs(msg.linear.x - 2.0) < 1.0e-9 for _, msg in commands[count:]), \
                "prediction did not align odometry to selected reference time"
            wait_for(lambda: diagnostics and diagnostics[-1].valid
                     and abs(diagnostics[-1].command.linear.x - 2.0) < 1.0e-9)
            diagnostic = diagnostics[-1]
            assert abs(diagnostic.ex) < 1.0e-9 and abs(diagnostic.ey) < 1.0e-9
            assert abs(diagnostic.predicted_x - diagnostic.reference.x) < 1.0e-9
            ns = lambda stamp: stamp.sec * 1000000000 + stamp.nanosec
            assert ns(diagnostic.odom_stamp) < ns(diagnostic.header.stamp) <= ns(diagnostic.reference_stamp)
            measured_x = 1.0 + 2.0 * (ns(diagnostic.odom_stamp) - moving_start) * 1.0e-9
            assert diagnostic.reference.x - measured_x > 0.01

            # 측위 갱신이 80ms 이상 중단돼도 궤적이 유효하면 계속 예측/제어한다.
            send_odom = False
            count = len(commands)
            wait_for(lambda: len(commands) >= count + 8)
            assert all(abs(msg.linear.x - 2.0) < 1.0e-9 for _, msg in commands[count:]), \
                "old odom triggered a time-limit stop"
            wait_for(lambda: diagnostics and diagnostics[-1].valid
                     and ns(diagnostics[-1].reference_stamp) - ns(diagnostics[-1].odom_stamp) > 50000000)
            assert modes[-1] == 2, "old odom changed mode relay"
            send_odom = True
            publish_odom()
            wait_for(lambda: abs(commands[-1][1].linear.x - 2.0) < 1.0e-9)
            print(f"tracker ROS passed: {len(valid)} controls from one trajectory, median {median:.4f}s")
            print("timestamp ordering/equality and frame mismatch/recovery passed")
            print("delayed-odom prediction, unlimited old-odom prediction and refresh passed")
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
