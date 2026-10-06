"""차량 footprint 사각형 + 주행 궤적(odometry trail)을 RViz용으로 publish한다.

  /viz/vehicle_footprint  (visualization_msgs/MarkerArray)
      /odom을 받을 때마다 차량 중심(x,y) + body yaw 기준으로 몸체 + footprint
      테두리를 다시 그려서 publish한다: 박스형 화물 차량 메시 몸체(바퀴 제외,
      meshes/build_cargo_mesh.py; vehicle_mesh_resource로 SIMP03 CAD 메시
      simp03_body.dae도 쓸 수 있고, 빈 문자열이면 진한 남색 CUBE) + 위에 겹치는 굵은
      LINE_STRIP 테두리(검정에 가까운 진한 색). debug_plot_node.py의
      vehicle_polygon()과 동일한 규약: 위치는 차량 중심, 앞뒤/좌우 대칭
      (vehicle_length x vehicle_width). 메시는 footprint가 1x1로 정규화돼
      있어서 CUBE와 똑같이 scale.x/y = vehicle_length/width로 늘려 그리므로
      항상 코드상의 차량 크기와 일치한다.
  /viz/drive_mode_status_overlay  (rviz_2d_overlay_msgs/OverlayText)
      차량↔플래너 사이 드라이브 모드 상태 전이 값(아래 참고)을 RViz 화면 왼쪽 위에
      고정된 2D 글자로 publish한다. 3D 장면 위에 그려지므로 어느 시점에서도 벽,
      램프, 차량에 가려지지 않는다. rviz_2d_overlay_msgs가 없으면(아래 설치 명령)
      대신 /viz/vehicle_footprint에 3D 텍스트 마커를 넣는다.
        sudo apt install ros-jazzy-rviz-2d-overlay-plugins ros-jazzy-rviz-2d-overlay-msgs
  /viz/traveled_path      (nav_msgs/Path)
      /odom 포즈가 min_recorded_distance(m) 이상 움직일 때마다 누적해서,
      지금까지 실제로 지나온 경로를 계속 자라나는 Path로 publish한다.

  드라이브 모드 상태 전이 텍스트는 /vehicle/drive_mode_state(차량->플래너
  피드백)의 current_mode/requested_mode/status 3개 필드를 그대로 보여준다.
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
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

try:
    from rviz_2d_overlay_msgs.msg import OverlayText
except ImportError:  # 오버레이 플러그인 미설치: 3D 텍스트 마커로 대체
    OverlayText = None

MODE_NAMES = {0: "FORWARD", 1: "REVERSE", 2: "LEFT", 3: "RIGHT", 4: "SPOT_TURN"}
STATUS_NAMES = {
    DriveModeState.STATUS_ALIGNING: "ALIGNING",
    DriveModeState.STATUS_READY: "READY",
}
# 모드가 바뀌어도 상자 크기가 변하지 않도록 가장 긴 줄 기준으로 폭을 고정한다
STATUS_BOX_WIDTH = len("vehicle requested: ") + max(len(name) for name in MODE_NAMES.values())


# 상태 글자 색: 라벨과 테두리 검정, 값 진한 파랑, ALIGNING 값만 빨강
TEXT_BLUE = (0.05, 0.17, 0.55)
LABEL_BLACK = (0.0, 0.0, 0.0)
ALERT_RED = (0.82, 0.08, 0.08)


def html_span(text: str, rgb: tuple[float, float, float]) -> str:
    """오버레이 문구 안에서 색을 지정하는 조각. style은 ';'로 끝나야 플러그인이 그림자용
    문구에서 이 색을 지운다 (아래 publish_status_overlay 참고). 공백은 &nbsp;로 바꿔
    HTML이 연속 공백을 하나로 합치지 않게 한다 (상자 테두리 정렬)."""
    hex_color = "#" + "".join(f"{round(255 * c):02x}" for c in rgb)
    return f'<span style="color: {hex_color};">{text.replace(" ", "&nbsp;")}</span>'


def mode_name(value: int | None) -> str:
    if value is None:
        return "?"
    return MODE_NAMES.get(int(value), str(value))


def status_name(value: int | None) -> str:
    if value is None:
        return "?"
    return STATUS_NAMES.get(int(value), str(value))


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
        self.declare_parameter("status_text_scale", 2.8)
        self.declare_parameter(
            "vehicle_mesh_resource", "package://simp_planner_tools/meshes/cargo_body.dae"
        )

        self.vehicle_length = float(self.get_parameter("vehicle_length").value)
        self.vehicle_width = float(self.get_parameter("vehicle_width").value)
        self.min_recorded_distance = float(self.get_parameter("min_recorded_distance").value)
        self.max_path_points = int(self.get_parameter("max_path_points").value)
        self.status_text_offset_m = float(self.get_parameter("status_text_offset_m").value)
        self.status_text_scale = float(self.get_parameter("status_text_scale").value)
        self.vehicle_mesh_resource = str(self.get_parameter("vehicle_mesh_resource").value)

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

        self.status_overlay_pub = None
        if OverlayText is not None:
            self.status_overlay_pub = self.create_publisher(
                OverlayText, "/viz/drive_mode_status_overlay", static_qos
            )
            # RViz를 나중에 켜도 바로 보이도록 주기적으로도 다시 보낸다
            self.create_timer(0.5, self.publish_status_overlay)
        else:
            self.get_logger().warn(
                "rviz_2d_overlay_msgs not installed: drive-mode status falls back to a 3D "
                "text marker that scene objects can hide. Install with: sudo apt install "
                "ros-jazzy-rviz-2d-overlay-plugins ros-jazzy-rviz-2d-overlay-msgs"
            )

        self.create_subscription(Odometry, "/odom", self.odom_callback, 50)
        self.create_subscription(
            DriveModeState, "/vehicle/drive_mode_state", self.mode_state_callback, static_qos
        )

    def mode_state_callback(self, message: DriveModeState) -> None:
        self.vehicle_mode_state = message
        if self.status_overlay_pub is not None:
            self.publish_status_overlay()

    def publish_status_overlay(self) -> None:
        overlay = OverlayText()
        overlay.action = OverlayText.ADD
        overlay.horizontal_alignment = OverlayText.LEFT
        overlay.vertical_alignment = OverlayText.TOP
        overlay.horizontal_distance = 10
        overlay.vertical_distance = 10
        overlay.width = 470
        overlay.height = 140
        overlay.text_size = 16.0
        overlay.line_width = 2
        overlay.font = "DejaVu Sans Mono"
        overlay.bg_color = ColorRGBA(r=0.0, g=0.0, b=0.0, a=0.0)  # 배경 없음 (투명)
        # 플러그인은 글자를 fg_color로 감싸 그리고, 그림자(1px 어긋난 검은 글자)를
        # fg_color의 투명도로 한 번 더 그린다. fg_color를 완전 투명으로 두면 그림자가
        # 사라지고, 실제 글자색은 문구 안의 <span> 색이 덮어써서 그대로 보인다.
        overlay.fg_color = ColorRGBA(r=0.0, g=0.0, b=0.0, a=0.0)
        overlay.text = self.drive_mode_status_overlay_text()
        self.status_overlay_pub.publish(overlay)

    def drive_mode_status_overlay_text(self) -> str:
        """drive_mode_status_text()에 색을 입히고 검은 상자 테두리로 감싼 오버레이 문구.

        플러그인은 글자만 그리므로(HTML 표 테두리 등은 그려지지 않는다) 테두리는 고정폭
        글꼴(overlay.font)의 상자 선 문자로 만든다.
        """
        rows = [line.partition(": ")[::2] for line in self.drive_mode_status_text().split("\n")]
        width = max([STATUS_BOX_WIDTH] + [len(label) + 2 + len(value) for label, value in rows])
        lines = [html_span("┌" + "─" * (width + 2) + "┐", LABEL_BLACK)]
        for label, value in rows:
            value_color = ALERT_RED if value == "ALIGNING" else TEXT_BLUE
            padding = " " * (width - len(label) - 2 - len(value))
            lines.append(html_span(f"│ {label}: ", LABEL_BLACK) + html_span(value, value_color)
                         + html_span(padding + " │", LABEL_BLACK))
        lines.append(html_span("└" + "─" * (width + 2) + "┘", LABEL_BLACK))
        return "\n".join(lines)

    def drive_mode_status_text(self) -> str:
        lines: list[str] = []
        state = self.vehicle_mode_state
        if state is None:
            lines.append("vehicle: (no feedback yet)")
        else:
            lines.append(f"vehicle current: {mode_name(state.current_mode)}")
            lines.append(f"vehicle requested: {mode_name(state.requested_mode)}")
            lines.append(f"status: {status_name(state.status)}")
        return "\n".join(lines)

    def status_text_marker(self, header, x: float, y: float) -> Marker:
        """오버레이를 쓸 수 없을 때의 3D 상태 글자 (다른 물체에 가려질 수 있다)."""
        # Deliberately offset away from the vehicle footprint (not stacked on
        # top of it) in a fixed world-frame direction, and rendered large --
        # a marker sized/placed to sit inside the vehicle box is unreadable
        # once the view is zoomed out to see the whole scenario.
        status_text = Marker()
        status_text.header = header
        status_text.ns = "drive_mode_status"
        status_text.id = 0
        status_text.type = Marker.TEXT_VIEW_FACING
        status_text.action = Marker.ADD
        status_text.pose.position.x = x
        status_text.pose.position.y = y + self.status_text_offset_m
        status_text.pose.position.z = 2.0
        status_text.scale.z = self.status_text_scale
        # 3D 텍스트 마커는 색이 하나뿐이라 ALIGNING이면 문구 전체를 빨강으로 한다.
        aligning = (self.vehicle_mode_state is not None
                    and self.vehicle_mode_state.status == DriveModeState.STATUS_ALIGNING)
        r, g, b = ALERT_RED if aligning else TEXT_BLUE
        status_text.color = ColorRGBA(r=r, g=g, b=b, a=1.0)
        status_text.text = self.drive_mode_status_text()
        return status_text

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
        body.action = Marker.ADD
        body.pose.position.x = x
        body.pose.position.y = y
        body.pose.orientation.z = math.sin(0.5 * yaw)
        body.pose.orientation.w = math.cos(0.5 * yaw)
        body.scale.x = self.vehicle_length
        body.scale.y = self.vehicle_width
        if self.vehicle_mesh_resource:
            # 메시는 x/y가 [-0.5, 0.5]로 정규화돼 있고 z는 실제 높이(m, 지면=0).
            # color를 전부 0으로 두면 RViz가 메시에 내장된 부품별 색을 쓴다.
            body.type = Marker.MESH_RESOURCE
            body.mesh_resource = self.vehicle_mesh_resource
            body.mesh_use_embedded_materials = True
            body.scale.z = 1.0
        else:
            body.type = Marker.CUBE
            body.pose.position.z = 0.15
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
        # 몸체 위(메시 최고점 ~0.48 m, CUBE 0.3 m)에 그려야 TopDown 뷰에서 가려지지 않는다.
        outline.points = [
            Point(x=px, y=py, z=0.5)
            for px, py in vehicle_footprint_points(
                x, y, yaw, self.vehicle_length, self.vehicle_width
            )
        ]

        markers = [body, outline]
        if self.status_overlay_pub is None:
            markers.append(self.status_text_marker(body.header, x, y))
        self.footprint_pub.publish(MarkerArray(markers=markers))

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
