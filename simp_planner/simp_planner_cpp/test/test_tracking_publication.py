"""ROS 환경을 source한 뒤 python3 이파일 planner_node_cpp경로 로 실행한다."""

import json
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
    plans = []
    node.create_subscription(TrackingTrajectory, "/planner/tracking_trajectory", messages.append, 100)
    node.create_subscription(String, "/planner/execution_state", lambda msg: states.append(msg.data), qos)

    def receive_status(message):
        plan = json.loads(message.data).get("plan", {})
        if "start_ns" in plan:
            plans.append(plan)

    node.create_subscription(String, "/planner/status", receive_status, 100)

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
    path.x = [5.0 + 0.1 * i for i in range(31)]
    path.y = [2.0] * 31
    path.yaw = [0.0] * 31
    path.mode = [DriveModeState.FORWARD] * 31
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
        process = subprocess.Popen([
            sys.argv[1], "--ros-args",
            "-p", "planning_handover_min_lead_sec:=0.6",
            "-p", "planning_handover_initial_lead_sec:=0.6",
        ], stdout=log, stderr=log)
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
            odom.twist.twist.linear.x = 9.0
            odom.pose.pose.orientation.z = 1.0
            odom.pose.pose.orientation.w = 0.0
            count = len(messages)
            wait_for(lambda: len(messages) >= count + 4)
            assert messages[-1].points[0].x == 5.0
            assert stationary(messages[-1]), "초기화 이후 odometry가 nominal 상태를 덮어씀"

            path_pub.publish(path)
            map_pub.publish(grid)
            speed_pub.publish(Float64(data=1.0))
            wait_for(lambda: len(plans) >= 1)
            first_plan = plans[-1]
            speed_pub.publish(Float64(data=0.8))
            wait_for(lambda: plans[-1]["id"] > first_plan["id"])
            assert plans[-1]["ready_ns"] < first_plan["start_ns"], \
                "미래 계획 존재로 다음 planning이 차단됨"
            assert plans[-1]["start_ns"] == first_plan["start_ns"], \
                "재계획이 handover 시각을 계속 뒤로 미룸"
            wait_for(lambda: messages[-1].points[0].vx > 0.1)
            moving = messages[-1]
            assert moving.points[-1].x > moving.points[0].x, "미래 위치가 진행하지 않음"
            assert all(abs(p.y - 2.0) < 1e-5 for p in moving.points), "직선 경로 이탈"

            # 주행 중에도 새 계획을 즉시 등록하고 한 배열 안에서 경계를 연결한다.
            plan_id = plans[-1]["id"]
            speed_pub.publish(Float64(data=0.6))
            wait_for(lambda: plans[-1]["id"] > plan_id)
            boundary = plans[-1]["start_ns"]

            def stamp_ns(message):
                return message.header.stamp.sec * 1000000000 + message.header.stamp.nanosec

            wait_for(lambda: stamp_ns(messages[-1]) < boundary <
                     stamp_ns(messages[-1]) + 300000000)
            crossing = messages[-1]
            assert crossing.points[0].vx > 0.0, "handover 이전 계획 구간이 사라짐"
            assert all(0 <= b.x - a.x < 0.03 and abs(b.vx - a.vx) < 0.05
                       for a, b in zip(crossing.points, crossing.points[1:])), \
                "handover 경계에서 참조 위치/속도 단절"
            wait_for(lambda: stamp_ns(messages[-1]) > boundary + 100000000)
            assert messages[-1].points[0].x > crossing.points[0].x, \
                "이전 계획 정리 후 nominal 진행 중단"

            # nominal 주행 중 모드 변경은 감속 참조를 만들되 미래 정렬 완료를 가정하지 않는다.
            mode.requested_mode = DriveModeState.LEFT
            mode.status = DriveModeState.STATUS_ALIGNING
            request.data = DriveModeState.LEFT
            mode_pub.publish(mode)
            request_pub.publish(request)
            wait_for(lambda: messages[-1].points[0].requested_mode == DriveModeState.LEFT
                     and messages[-1].points[0].vx > 0.1
                     and messages[-1].points[-1].vx < messages[-1].points[0].vx)
            wait_for(lambda: messages[-1].points[0].requested_mode == DriveModeState.LEFT
                     and stationary(messages[-1]))
            held = messages[-1]
            assert all(p.current_mode == DriveModeState.FORWARD and
                       p.requested_mode == DriveModeState.LEFT for p in held.points), "모드 정보 불일치"
            assert all(p.x == held.points[0].x and p.y == held.points[0].y
                       for p in held.points), "정렬 대기 중 위치 변화"

            # nominal 종점 도착·회전 완료와 실제 모드 정렬 확인으로 진행한다.
            request.data = DriveModeState.FORWARD
            mode.requested_mode = DriveModeState.FORWARD
            mode.status = DriveModeState.STATUS_READY
            request_pub.publish(request)
            mode_pub.publish(mode)
            wait_for(lambda: states and states[-1] == "TERMINAL_HOLD"
                     and stationary(messages[-1]), timeout=30.0)
            origin = messages[-1].points[0]
            path.x = [origin.x + i * math.cos(0.6) for i in range(21)]
            path.y = [origin.y + i * math.sin(0.6) for i in range(21)]
            path.yaw = [0.6] * 21
            path.mode = [DriveModeState.FORWARD] * 21
            path_pub.publish(path)
            wait_for(lambda: messages[-1].points[0].requested_mode == DriveModeState.SPOT_TURN)
            assert stationary(messages[-1]), "휠 정렬 완료 전에 회전 참조가 발생함"
            mode.current_mode = mode.requested_mode = DriveModeState.SPOT_TURN
            mode_pub.publish(mode)
            wait_for(lambda: messages[-1].points[0].yaw_rate > 0.0)
            turn_start = len(messages)
            wait_for(lambda: len(messages) >= turn_start + 4)
            assert messages[-1].points[0].body_yaw > messages[turn_start].points[0].body_yaw, \
                "odometry 없이 nominal 회전이 진행되지 않음"
            assert messages[-1].points[-1].body_yaw > messages[-1].points[0].body_yaw
            wait_for(lambda: messages[-1].points[0].requested_mode == DriveModeState.FORWARD)
            assert stationary(messages[-1]), "회전 후 일반 모드 정렬 전에 이동함"
            mode.current_mode = mode.requested_mode = DriveModeState.FORWARD
            mode_pub.publish(mode)
            wait_for(lambda: messages[-1].points[0].vx > 0.3
                     and messages[-1].points[-1].y > messages[-1].points[0].y)

            # 충돌로 인한 강제 정지도 nominal 상태에서 생성한다.
            grid.data = [100] * len(grid.data)
            map_pub.publish(grid)
            wait_for(lambda: states[-1] == "SAFETY_STOP"
                     and messages[-1].points[0].vx > 0.1
                     and messages[-1].points[-1].vx < messages[-1].points[0].vx)
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
