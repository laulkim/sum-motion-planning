"""차량 footprint 사각형 + 주행 궤적(odometry trail)을 RViz용으로 publish한다.

  /viz/vehicle_footprint  (visualization_msgs/MarkerArray)
      /odom을 받을 때마다 차량 중심(x,y) + body yaw 기준으로 직사각형 2겹을
      다시 그려서 publish한다: 채운 CUBE 몸체(진한 남색) + 위에 겹치는 굵은
      LINE_STRIP 테두리(검정에 가까운 진한 색). debug_plot_node.py의
      vehicle_polygon()과 동일한 규약: 위치는 차량 중심, 앞뒤/좌우 대칭
      (vehicle_length x vehicle_width). 같은 MarkerArray에 차량↔플래너 사이
      드라이브 모드 상태 전이 값(아래 참고)을 보여주는 텍스트 마커도 함께
      publish한다.
  /viz/traveled_path      (nav_msgs/Path)
      /odom 포즈가 min_recorded_distance(m) 이상 움직일 때마다 누적해서,
      지금까지 실제로 지나온 경로를 계속 자라나는 Path로 publish한다.

  드라이브 모드 상태 전이 텍스트는 /vehicle/drive_mode_state(차량->플래너
  피드백)의 current_mode/requested_mode/transition_in_progress/
  transition_complete 4개 필드를 그대로 보여준다.
"""
from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path as PathMessage
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from simp_planner_msgs.msg import DriveModeState
from visualization_msgs.msg import Marker, MarkerArray

MODE_NAMES = {0: "FORWARD", 1: "REVERSE", 2: "LEFT", 3: "RIGHT"}


def mode_name(value: int | None) -> str:
    if value is None:
        return "?"
    return MODE_NAMES.get(int(value), str(value))


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def vehicle_footprint_points(
    x: float, y: float, yaw: float, length: float, width: float
) -> list[tuple[float, float]]:
    half_l = 0.5 * length
    half_w = 0.5 * width
    corners = [
        (half_l, half_w),
        (half_l, -half_w),
        (-half_l, -half_w),
        (-half_l, half_w),
        (half_l, half_w),  # close the loop
    ]
    c, s = math.cos(yaw), math.sin(yaw)
    return [(x + c * lx - s * ly, y + s * lx + c * ly) for lx, ly in corners]


class VehicleVisualizerNode(Node):
    def __init__(self) -> None:
        super().__init__("vehicle_visualizer_node")
        self.declare_parameter("vehicle_length", 3.0)
        self.declare_parameter("vehicle_width", 2.0)
        self.declare_parameter("min_recorded_distance", 0.10)
        self.declare_parameter("max_path_points", 20000)
        self.declare_parameter("status_text_offset_m", 8.0)
        self.declare_parameter("status_text_scale", 2.5)

        self.vehicle_length = float(self.get_parameter("vehicle_length").value)
        self.vehicle_width = float(self.get_parameter("vehicle_width").value)
        self.min_recorded_distance = float(self.get_parameter("min_recorded_distance").value)
        self.max_path_points = int(self.get_parameter("max_path_points").value)
        self.status_text_offset_m = float(self.get_parameter("status_text_offset_m").value)
        self.status_text_scale = float(self.get_parameter("status_text_scale").value)

        static_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.footprint_pub = self.create_publisher(MarkerArray, "/viz/vehicle_footprint", 1)
        self.trail_pub = self.create_publisher(PathMessage, "/viz/traveled_path", static_qos)

        self.traveled_path = PathMessage()
        self.last_recorded_xy: tuple[float, float] | None = None
        self.vehicle_mode_state: DriveModeState | None = None

        self.create_subscription(Odometry, "/odom", self.odom_callback, 50)
        self.create_subscription(
            DriveModeState, "/vehicle/drive_mode_state", self.mode_state_callback, static_qos
        )

    def mode_state_callback(self, message: DriveModeState) -> None:
        self.vehicle_mode_state = message

    def drive_mode_status_text(self) -> str:
        lines: list[str] = []
        state = self.vehicle_mode_state
        if state is None:
            lines.append("vehicle: (no feedback yet)")
        else:
            lines.append(f"vehicle current: {mode_name(state.current_mode)}")
            lines.append(f"vehicle requested: {mode_name(state.requested_mode)}")
            lines.append(f"in_progress: {state.transition_in_progress}")
            lines.append(f"complete: {state.transition_complete}")
        return "\n".join(lines)

    def odom_callback(self, message: Odometry) -> None:
        frame_id = message.header.frame_id or "odom"
        x = message.pose.pose.position.x
        y = message.pose.pose.position.y
        q = message.pose.pose.orientation
        yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)

        body = Marker()
        body.header = message.header
        body.header.frame_id = frame_id
        body.ns = "vehicle_footprint"
        body.id = 0
        body.type = Marker.CUBE
        body.action = Marker.ADD
        body.pose.position.x = x
        body.pose.position.y = y
        body.pose.position.z = 0.15
        body.pose.orientation.z = math.sin(0.5 * yaw)
        body.pose.orientation.w = math.cos(0.5 * yaw)
        body.scale.x = self.vehicle_length
        body.scale.y = self.vehicle_width
        body.scale.z = 0.3
        body.color.r = 0.02
        body.color.g = 0.08
        body.color.b = 0.45
        body.color.a = 0.95

        outline = Marker()
        outline.header = body.header
        outline.ns = "vehicle_footprint"
        outline.id = 1
        outline.type = Marker.LINE_STRIP
        outline.action = Marker.ADD
        outline.scale.x = 0.12
        outline.color.r = 0.0
        outline.color.g = 0.0
        outline.color.b = 0.0
        outline.color.a = 1.0
        outline.points = [
            Point(x=px, y=py, z=0.32)
            for px, py in vehicle_footprint_points(
                x, y, yaw, self.vehicle_length, self.vehicle_width
            )
        ]

        # Deliberately offset away from the vehicle footprint (not stacked on
        # top of it) in a fixed world-frame direction, and rendered large --
        # a marker sized/placed to sit inside the vehicle box is unreadable
        # once the view is zoomed out to see the whole scenario.
        status_text = Marker()
        status_text.header = body.header
        status_text.ns = "drive_mode_status"
        status_text.id = 0
        status_text.type = Marker.TEXT_VIEW_FACING
        status_text.action = Marker.ADD
        status_text.pose.position.x = x
        status_text.pose.position.y = y + self.status_text_offset_m
        status_text.pose.position.z = 2.0
        status_text.scale.z = self.status_text_scale
        status_text.color.r = 0.85
        status_text.color.g = 0.05
        status_text.color.b = 0.05
        status_text.color.a = 1.0
        status_text.text = self.drive_mode_status_text()

        self.footprint_pub.publish(MarkerArray(markers=[body, outline, status_text]))

        moved_enough = (
            self.last_recorded_xy is None
            or math.hypot(x - self.last_recorded_xy[0], y - self.last_recorded_xy[1])
            >= self.min_recorded_distance
        )
        if moved_enough:
            self.last_recorded_xy = (x, y)
            pose = PoseStamped()
            pose.header = message.header
            pose.header.frame_id = frame_id
            pose.pose = message.pose.pose
            self.traveled_path.header = pose.header
            self.traveled_path.poses.append(pose)
            if len(self.traveled_path.poses) > self.max_path_points:
                self.traveled_path.poses = self.traveled_path.poses[-self.max_path_points:]
            self.trail_pub.publish(self.traveled_path)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VehicleVisualizerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
