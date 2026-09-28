"""ROS 환경을 source한 뒤 python3 이파일 planner_node_cpp경로 로 실행한다."""

import math
import os
import statistics
import subprocess
import sys
import tempfile
import time

import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile
from simp_planner_msgs.msg import DriveModeState, ReferencePath, TrackingTrajectory
from std_msgs.msg import Float64, String, UInt8


def main():
    # 실행 중인 차량/시뮬레이터와 테스트 토픽을 분리한다.
    os.environ["ROS_DOMAIN_ID"] = str(150 + os.getpid() % 70)
    rclpy.init()
    node = rclpy.create_node("tracking_publication_test")
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    odom_pub = node.create_publisher(Odometry, "/odom", 10)
    mode_pub = node.create_publisher(DriveModeState, "/vehicle/drive_mode_state", qos)
    request_pub = node.create_publisher(UInt8, "/requested_drive_mode", qos)
    path_pub = node.create_publisher(ReferencePath, "/reference_path_data", qos)
    map_pub = node.create_publisher(OccupancyGrid, "/costmap", qos)
    speed_pub = node.create_publisher(Float64, "/target_speed", qos)
    messages = []
    states = []
    node.create_subscription(TrackingTrajectory, "/planner/tracking_trajectory", messages.append, 100)
    node.create_subscription(String, "/planner/execution_state", lambda msg: states.append(msg.data), qos)

    odom = Odometry()
    odom.header.frame_id = "map"
    odom.pose.pose.position.x = 5.0
    odom.pose.pose.position.y = 2.0
    odom.pose.pose.orientation.w = 1.0
    mode = DriveModeState()
    mode.status = DriveModeState.STATUS_READY
    request = UInt8(data=DriveModeState.FORWARD)
    path = ReferencePath()
    path.header.frame_id = "map"
    path.x = [5.0 + i for i in range(61)]
    path.y = [2.0] * 61
    path.yaw = [0.0] * 61
    path.mode = [DriveModeState.FORWARD] * 61
    grid = OccupancyGrid()
    grid.header.frame_id = "map"
    grid.info.resolution = 0.5
    grid.info.width = 180
    grid.info.height = 80
    grid.info.origin.position.x = -10.0
    grid.info.origin.position.y = -10.0
    grid.info.origin.orientation.w = 1.0
    grid.data = [0] * (grid.info.width * grid.info.height)

    def wait_for(predicate, timeout=15.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            assert process.poll() is None, "플래너가 종료됨"
            odom.header.stamp = node.get_clock().now().to_msg()
            odom_pub.publish(odom)
            rclpy.spin_once(node, timeout_sec=0.02)
            if predicate():
                return
        raise AssertionError("참조 배열 대기 시간 초과")

    def stationary(message):
        return all(abs(p.vx) + abs(p.vy) + abs(p.yaw_rate) < 1e-9 for p in message.points)

    with tempfile.TemporaryFile(mode="w+") as log:
        process = subprocess.Popen([sys.argv[1]], stdout=log, stderr=log)
        try:
            mode_pub.publish(mode)
            request_pub.publish(request)
            wait_for(lambda: len(messages) >= 12)
            for message in messages:
                assert stationary(message), "계획 없는 상태에서 주행 참조 발생"
                assert all(abs(p.x - 5.0) < 1e-9 and abs(p.y - 2.0) < 1e-9
                           for p in message.points), "정지 위치가 원점으로 초기화됨"
            for topic in ("/cmd_vel", "/planner/cmd_vel_stamped", "/planner/executed_command",
                          "/vehicle/drive_mode_command"):
                assert node.count_publishers(topic) == 0, f"Planner가 직접 발행함: {topic}"
            odom.pose.pose.position.x = 6.0
            wait_for(lambda: messages[-1].points[0].x == 6.0)
            assert stationary(messages[-1]), "새 odometry 반영에 명령 콜백이 필요함"

            path_pub.publish(path)
            map_pub.publish(grid)
            speed_pub.publish(Float64(data=1.0))
            wait_for(lambda: messages[-1].points[0].vx > 0.1)
            moving = messages[-1]
            assert moving.points[-1].x > moving.points[0].x, "미래 위치가 진행하지 않음"
            assert all(abs(p.y - 2.0) < 1e-5 for p in moving.points), "직선 경로 이탈"

            # 실제 주행 중 모드 변경은 감속 참조를 만들되 미래 정렬 완료를 가정하지 않는다.
            odom.twist.twist.linear.x = 0.5
            mode.requested_mode = DriveModeState.LEFT
            mode.status = DriveModeState.STATUS_ALIGNING
            request.data = DriveModeState.LEFT
            mode_pub.publish(mode)
            request_pub.publish(request)
            wait_for(lambda: messages[-1].points[0].requested_mode == DriveModeState.LEFT
                     and messages[-1].points[0].vx > 0.1
                     and messages[-1].points[-1].vx < messages[-1].points[0].vx)
            odom.twist.twist.linear.x = 0.0
            wait_for(lambda: messages[-1].points[0].requested_mode == DriveModeState.LEFT
                     and stationary(messages[-1]))
            held = messages[-1]
            assert all(p.current_mode == DriveModeState.FORWARD and
                       p.requested_mode == DriveModeState.LEFT for p in held.points), "모드 정보 불일치"
            assert all(p.x == held.points[0].x and p.y == held.points[0].y
                       for p in held.points), "정렬 대기 중 위치 변화"

            # 실제 종점 도착과 회전 완료 피드백으로만 다음 단계로 넘어간다.
            odom.pose.pose.position.x = 64.0  # 마지막 curvature pad를 뺀 경로 끝
            request.data = DriveModeState.FORWARD
            mode.requested_mode = DriveModeState.FORWARD
            mode.status = DriveModeState.STATUS_READY
            request_pub.publish(request)
            mode_pub.publish(mode)
            wait_for(lambda: states and states[-1] == "TERMINAL_HOLD"
                     and stationary(messages[-1]) and messages[-1].points[0].x == 64.0)
            path.x = [64.0] * 21
            path.y = [2.0 + i for i in range(21)]
            path.yaw = [math.pi / 2] * 21
            path.mode = [DriveModeState.FORWARD] * 21
            path_pub.publish(path)
            wait_for(lambda: messages[-1].points[0].requested_mode == DriveModeState.SPOT_TURN)
            assert stationary(messages[-1]), "휠 정렬 완료 전에 회전 참조가 발생함"
            mode.current_mode = mode.requested_mode = DriveModeState.SPOT_TURN
            mode_pub.publish(mode)
            wait_for(lambda: messages[-1].points[0].yaw_rate > 0.0)
            turn_start = len(messages)
            wait_for(lambda: len(messages) >= turn_start + 4)
            assert all(abs(msg.points[0].body_yaw) < 1e-9 for msg in messages[turn_start:]), \
                "미리보기가 실제 현재 자세를 진행시킴"
            assert messages[-1].points[-1].body_yaw > messages[-1].points[0].body_yaw
            odom.pose.pose.orientation.z = math.sin(math.pi / 4)
            odom.pose.pose.orientation.w = math.cos(math.pi / 4)
            wait_for(lambda: messages[-1].points[0].requested_mode == DriveModeState.FORWARD)
            assert stationary(messages[-1]), "회전 후 일반 모드 정렬 전에 이동함"
            mode.current_mode = mode.requested_mode = DriveModeState.FORWARD
            mode_pub.publish(mode)
            wait_for(lambda: messages[-1].points[0].vx > 0.01
                     and messages[-1].points[-1].y > messages[-1].points[0].y)

            # 충돌로 인한 강제 정지도 이전 /cmd_vel 기록 없이 측정 상태로 생성한다.
            odom.twist.twist.linear.x = 0.5
            grid.data = [100] * len(grid.data)
            map_pub.publish(grid)
            wait_for(lambda: states[-1] == "SAFETY_STOP"
                     and messages[-1].points[0].vx > 0.1
                     and messages[-1].points[-1].vx < messages[-1].points[0].vx)
            odom.twist.twist.linear.x = 0.0
            wait_for(lambda: stationary(messages[-1]))
            grid.data = [0] * len(grid.data)
            map_pub.publish(grid)
            wait_for(lambda: messages[-1].points[0].vx > 0.01)

            stamps = []
            for message in messages:
                assert message.header.frame_id == "map"
                assert message.sample_period.sec == 0 and message.sample_period.nanosec == 10000000
                assert len(message.points) == 31, "0.3초 구간은 시작점 포함 31점"
                assert all(math.isfinite(value) for p in message.points
                           for value in (p.x, p.y, p.body_yaw, p.vx, p.vy, p.yaw_rate))
                stamps.append(message.header.stamp.sec + message.header.stamp.nanosec * 1e-9)
            intervals = [b - a for a, b in zip(stamps, stamps[1:])]
            assert all(dt > 0 for dt in intervals), "시간 역전"
            median = statistics.median(intervals)
            assert 0.08 < median < 0.12, f"발행 중앙 주기: {median}"
            print(f"tracking publication passed: {len(messages)} arrays, median period {median:.4f}s")
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
