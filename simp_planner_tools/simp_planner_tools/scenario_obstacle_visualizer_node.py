"""시나리오 장애물을 물류센터 느낌(팔레트 위에 쌓인 골판지 상자)으로 RViz에 publish한다.

  /viz/scenario_obstacles  (visualization_msgs/MarkerArray)
      장애물 직사각형(x, y, length, width, yaw -- costmap에 rasterize되는 것과
      같은 ScenarioObstacle 값)을 표준 팔레트(1.2 x 1.0 m)에 가까운 칸으로
      등분하고, 칸마다 [나무 팔레트 + 골판지 상자 1~3단 + 테이프]를 그린다.
      칸이 장애물을 정확히 등분하므로 팔레트 바깥 경계 = 코드상의 장애물
      크기다. 칸이 팔레트를 놓기에 너무 좁으면 팔레트 없이 상자만 바닥에
      놓는다. kind="wall"인 장애물(창고 외벽, 램프 벽, 도크 칸막이 등)은 같은
      직사각형 그대로 회색 콘크리트 벽(CUBE)으로 그린다. 높이(상자 단수, 벽
      높이)는 시각용일 뿐 플래너와 무관하다.
      시나리오에 바닥 표현(FloorMarking: 실내 바닥/구역 칠, 차선, 화살표, 글자)이
      있으면 같은 MarkerArray에 바닥 아래층(z<0, costmap 아래)으로 함께 그린다.

  장애물은 정적이므로 hdmap_lane_visualizer_node처럼 TRANSIENT_LOCAL QoS로 한 번만
  publish한다. 장애물마다 부품별 CUBE_LIST 마커(받침/블록/상판/상자/테이프)를 하나씩
  쓰고, 마커 pose = 장애물 중심 + yaw, points = 장애물 로컬 좌표다.
"""
from __future__ import annotations

import math
import random
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from .scenario_definition import FloorMarking, ScenarioObstacle, load_scenario_definition

PALLET_LENGTH = 1.2  # m, EUR 팔레트
PALLET_WIDTH = 1.0
MIN_PALLET_SIDE = 0.6  # 칸의 짧은 변이 이보다 작으면 팔레트 없이 상자만
MIN_CARTON_SIDE = 0.3  # 칸 한 변을 상자 2개로 나눌 최소 길이

RUNNER_HEIGHT = 0.022  # 하부 받침 판
BLOCK_HEIGHT = 0.100
DECK_HEIGHT = 0.022  # 상판
PALLET_HEIGHT = RUNNER_HEIGHT + BLOCK_HEIGHT + DECK_HEIGHT
CARTON_HEIGHT = 0.38
CARTON_GAP = 0.03
TAPE_WIDTH = 0.06
TAPE_HEIGHT = 0.004
# 칸마다 상자 단수 1~3 (벽처럼 보이도록 높은 단에 가중치)
LAYER_CHOICES = (1, 2, 3)
LAYER_WEIGHTS = (0.15, 0.40, 0.45)

WOOD_DARK = (0.55, 0.40, 0.24)
WOOD_LIGHT = (0.78, 0.62, 0.40)
CARDBOARD = (0.76, 0.58, 0.38)
CARDBOARD_JITTER = 0.05
TAPE_COLOR = (0.94, 0.88, 0.74)

WALL_HEIGHT = 1.5
WALL_COLOR = (0.60, 0.61, 0.63)

# 바닥 표현. 모두 costmap(z=0) 아래에 깔아서 costmap/경로/차량을 가리지 않는다.
FLOOR_COLORS = {
    "floor": (0.95, 0.94, 0.91),    # 실내 바닥 (매우 연한 웜그레이)
    "loading": (0.99, 0.92, 0.62),  # 하역장
    "hatch": (0.93, 0.78, 0.33),
    "room": (0.86, 0.89, 0.93),     # 사무실/창고
    "core": (0.70, 0.71, 0.73),     # 램프 코어
    "ramp": (0.74, 0.76, 0.78),     # 램프 노면
    "white": (1.0, 1.0, 1.0),
    "yellow": (0.98, 0.78, 0.12),
    "navy": (0.22, 0.25, 0.52),
    "green": (0.25, 0.70, 0.35),    # 도크 레벨러
    "label": (0.20, 0.20, 0.24),
}
FLOOR_SURFACE_COLORS = {"loading", "room", "core", "ramp"}
FLOOR_BASE_Z = -0.05
FLOOR_SURFACE_Z = -0.04
FLOOR_PAINT_Z = -0.03
DASH_LENGTH = 2.0
DASH_GAP = 2.0
LABEL_HEIGHT = 1.6
LABEL_Z = 2.0


def _rgba(rgb: tuple[float, float, float], alpha: float = 1.0) -> ColorRGBA:
    return ColorRGBA(r=float(rgb[0]), g=float(rgb[1]), b=float(rgb[2]), a=alpha)


def _cube_list(
    frame_id: str, ns: str, marker_id: int, obstacle: ScenarioObstacle,
    size: tuple[float, float, float], color: tuple[float, float, float],
) -> Marker:
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.ns = ns
    marker.id = marker_id
    marker.type = Marker.CUBE_LIST
    marker.action = Marker.ADD
    marker.pose.position.x = float(obstacle.x)
    marker.pose.position.y = float(obstacle.y)
    marker.pose.orientation.z = math.sin(0.5 * float(obstacle.yaw))
    marker.pose.orientation.w = math.cos(0.5 * float(obstacle.yaw))
    marker.scale.x, marker.scale.y, marker.scale.z = (float(v) for v in size)
    marker.color = _rgba(color)
    return marker


def _spread(count: int, half_extent: float, half_size: float) -> list[float]:
    """칸 안에서 양 끝이 칸 경계에 닿도록 count개 부재 중심을 균등 배치."""
    if count == 1:
        return [0.0]
    edge = half_extent - half_size
    return [-edge + 2.0 * edge * k / (count - 1) for k in range(count)]


def obstacle_pallet_markers(
    obstacle: ScenarioObstacle, index: int, frame_id: str
) -> list[Marker]:
    length = float(obstacle.length)
    width = float(obstacle.width)
    nx = max(1, round(length / PALLET_LENGTH))
    ny = max(1, round(width / PALLET_WIDTH))
    tile_l = length / nx
    tile_w = width / ny
    tile_centers = [
        (-0.5 * length + (i + 0.5) * tile_l, -0.5 * width + (j + 0.5) * tile_w, i, j)
        for i in range(nx)
        for j in range(ny)
    ]
    has_pallet = min(tile_l, tile_w) >= MIN_PALLET_SIDE
    markers: list[Marker] = []

    if has_pallet:
        runner_w, block_l, slat_w = 0.14 * tile_w, 0.12 * tile_l, 0.12 * tile_w
        runners = _cube_list(frame_id, "obstacle_pallet_runners", index, obstacle,
                             (tile_l, runner_w, RUNNER_HEIGHT), WOOD_DARK)
        blocks = _cube_list(frame_id, "obstacle_pallet_blocks", index, obstacle,
                            (block_l, runner_w, BLOCK_HEIGHT), WOOD_DARK)
        deck = _cube_list(frame_id, "obstacle_pallet_deck", index, obstacle,
                          (tile_l, slat_w, DECK_HEIGHT), WOOD_LIGHT)
        block_z = RUNNER_HEIGHT + 0.5 * BLOCK_HEIGHT
        deck_z = RUNNER_HEIGHT + BLOCK_HEIGHT + 0.5 * DECK_HEIGHT
        for cx, cy, _, _ in tile_centers:
            for dy in _spread(3, 0.5 * tile_w, 0.5 * runner_w):
                runners.points.append(Point(x=cx, y=cy + dy, z=0.5 * RUNNER_HEIGHT))
                for dx in _spread(3, 0.5 * tile_l, 0.5 * block_l):
                    blocks.points.append(Point(x=cx + dx, y=cy + dy, z=block_z))
            for dy in _spread(5, 0.5 * tile_w, 0.5 * slat_w):
                deck.points.append(Point(x=cx, y=cy + dy, z=deck_z))
        markers.extend((runners, blocks, deck))

    cartons_x = 2 if tile_l >= 2.0 * MIN_CARTON_SIDE else 1
    cartons_y = 2 if tile_w >= 2.0 * MIN_CARTON_SIDE else 1
    carton_l = tile_l / cartons_x - CARTON_GAP
    carton_w = tile_w / cartons_y - CARTON_GAP
    base_z = PALLET_HEIGHT if has_pallet else 0.0
    cartons = _cube_list(frame_id, "obstacle_cartons", index, obstacle,
                         (carton_l, carton_w, CARTON_HEIGHT), CARDBOARD)
    tape = _cube_list(frame_id, "obstacle_carton_tape", index, obstacle,
                      (carton_l, min(TAPE_WIDTH, 0.5 * carton_w), TAPE_HEIGHT), TAPE_COLOR)
    for cx, cy, i, j in tile_centers:
        # 같은 시나리오는 실행할 때마다 같은 모양이 되도록 칸 단위로 고정 seed
        rng = random.Random(index * 1_000_003 + i * 1_009 + j)
        layers = rng.choices(LAYER_CHOICES, LAYER_WEIGHTS)[0]
        top_z = base_z + layers * CARTON_HEIGHT
        for dx in _spread(cartons_x, 0.5 * tile_l, 0.5 * tile_l / cartons_x):
            for dy in _spread(cartons_y, 0.5 * tile_w, 0.5 * tile_w / cartons_y):
                for layer in range(layers):
                    cartons.points.append(Point(
                        x=cx + dx, y=cy + dy, z=base_z + (layer + 0.5) * CARTON_HEIGHT))
                    shade = rng.uniform(-CARDBOARD_JITTER, CARDBOARD_JITTER)
                    cartons.colors.append(_rgba(tuple(c + shade for c in CARDBOARD)))
                tape.points.append(Point(x=cx + dx, y=cy + dy, z=top_z + 0.5 * TAPE_HEIGHT))
    markers.extend((cartons, tape))
    return markers


def obstacle_wall_marker(obstacle: ScenarioObstacle, index: int, frame_id: str) -> Marker:
    wall = Marker()
    wall.header.frame_id = frame_id
    wall.ns = "obstacle_walls"
    wall.id = index
    wall.type = Marker.CUBE
    wall.action = Marker.ADD
    wall.pose.position.x = float(obstacle.x)
    wall.pose.position.y = float(obstacle.y)
    wall.pose.position.z = 0.5 * WALL_HEIGHT
    wall.pose.orientation.z = math.sin(0.5 * float(obstacle.yaw))
    wall.pose.orientation.w = math.cos(0.5 * float(obstacle.yaw))
    wall.scale.x = float(obstacle.length)
    wall.scale.y = float(obstacle.width)
    wall.scale.z = WALL_HEIGHT
    wall.color = _rgba(WALL_COLOR)
    return wall


def scenario_obstacle_markers(
    obstacles: tuple[ScenarioObstacle, ...], frame_id: str
) -> MarkerArray:
    array = MarkerArray()
    for index, obstacle in enumerate(obstacles):
        if obstacle.kind == "wall":
            array.markers.append(obstacle_wall_marker(obstacle, index, frame_id))
        else:
            array.markers.extend(obstacle_pallet_markers(obstacle, index, frame_id))
    return array


def _band_triangles(points, width: float) -> list[tuple[float, float]]:
    """폴리라인을 width 폭의 띠 삼각형으로. 꼭짓점마다 앞뒤 점을 잇는 방향의 법선을 써서
    곡선(램프 원)에서도 이음매가 벌어지지 않는다."""
    count = len(points)
    closed = count > 2 and math.dist(points[0], points[-1]) < 1.0e-6
    half = 0.5 * width
    left, right = [], []
    for i in range(count):
        if closed and i in (0, count - 1):
            previous, following = points[-2], points[1]
        else:
            previous, following = points[max(i - 1, 0)], points[min(i + 1, count - 1)]
        dx, dy = following[0] - previous[0], following[1] - previous[1]
        norm = math.hypot(dx, dy) or 1.0
        nx, ny = -dy / norm * half, dx / norm * half
        x, y = points[i]
        left.append((x + nx, y + ny))
        right.append((x - nx, y - ny))
    triangles = []
    for i in range(count - 1):
        triangles += [left[i], right[i], right[i + 1], left[i], right[i + 1], left[i + 1]]
    return triangles


def _dash_pieces(points, dash: float, gap: float) -> list[list[tuple[float, float]]]:
    pieces, current, travelled, drawing = [], [points[0]], 0.0, True
    for a, b in zip(points, points[1:]):
        length, t = math.dist(a, b), 0.0
        while length - t > 1.0e-9:
            period = dash if drawing else gap
            step = min(period - travelled, length - t)
            t += step
            travelled += step
            point = (a[0] + (b[0] - a[0]) * t / length, a[1] + (b[1] - a[1]) * t / length)
            if drawing:
                current.append(point)
            if travelled >= period - 1.0e-9:
                if drawing and len(current) > 1:
                    pieces.append(current)
                drawing, travelled, current = not drawing, 0.0, [point]
    if drawing and len(current) > 1:
        pieces.append(current)
    return pieces


def _arrow_triangles(tail, head, width: float) -> list[tuple[float, float]]:
    length = math.dist(tail, head)
    ux, uy = (head[0] - tail[0]) / length, (head[1] - tail[1]) / length
    head_length, head_half = min(3.0 * width, 0.5 * length), 1.5 * width
    neck = (head[0] - ux * head_length, head[1] - uy * head_length)
    return _band_triangles((tail, neck), width) + [
        head,
        (neck[0] - uy * head_half, neck[1] + ux * head_half),
        (neck[0] + uy * head_half, neck[1] - ux * head_half),
    ]


def floor_marking_markers(markings: tuple[FloorMarking, ...], frame_id: str) -> list[Marker]:
    """바닥 표현을 층(바닥/구역/도색)별 TRIANGLE_LIST 하나씩 + 글자 마커로 만든다."""
    layers = {"floor_base": (FLOOR_BASE_Z, []), "floor_surface": (FLOOR_SURFACE_Z, []),
              "floor_paint": (FLOOR_PAINT_Z, [])}
    labels = []
    for marking in markings:
        if marking.kind == "label":
            labels.append(marking)
            continue
        if marking.kind == "area":  # 볼록 다각형 -> 부채꼴 분할
            triangles = [vertex for i in range(1, len(marking.points) - 1)
                         for vertex in (marking.points[0], marking.points[i], marking.points[i + 1])]
        elif marking.kind == "line":
            triangles = _band_triangles(marking.points, marking.width)
        elif marking.kind == "dashed":
            triangles = [vertex for piece in _dash_pieces(marking.points, DASH_LENGTH, DASH_GAP)
                         for vertex in _band_triangles(piece, marking.width)]
        elif marking.kind == "arrow":
            triangles = _arrow_triangles(*marking.points, marking.width)
        else:
            raise ValueError(f"unknown floor marking kind '{marking.kind}'")
        if marking.color == "floor":
            layer = "floor_base"
        elif marking.color in FLOOR_SURFACE_COLORS:
            layer = "floor_surface"
        else:
            layer = "floor_paint"
        layers[layer][1].append((triangles, FLOOR_COLORS[marking.color]))

    markers: list[Marker] = []
    for ns, (z, items) in layers.items():
        if not items:
            continue
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.ns = ns
        marker.type = Marker.TRIANGLE_LIST
        marker.action = Marker.ADD
        marker.pose.position.z = z
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 1.0
        marker.color = _rgba((1.0, 1.0, 1.0))
        for triangles, rgb in items:
            color = _rgba(rgb)
            for x, y in triangles:
                marker.points.append(Point(x=float(x), y=float(y), z=0.0))
                marker.colors.append(color)
        markers.append(marker)
    for index, label in enumerate(labels):
        text = Marker()
        text.header.frame_id = frame_id
        text.ns = "floor_labels"
        text.id = index
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x, text.pose.position.y = (float(v) for v in label.points[0])
        text.pose.position.z = LABEL_Z
        text.pose.orientation.w = 1.0
        text.scale.z = LABEL_HEIGHT
        text.color = _rgba(FLOOR_COLORS[label.color])
        text.text = label.text
        markers.append(text)
    return markers


class ScenarioObstacleVisualizerNode(Node):
    def __init__(self) -> None:
        super().__init__("scenario_obstacle_visualizer_node")
        self.declare_parameter("scenario", "stadium")
        self.declare_parameter("frame_id", "odom")
        scenario_name = str(self.get_parameter("scenario").value)
        frame_id = str(self.get_parameter("frame_id").value)

        static_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.obstacle_pub = self.create_publisher(
            MarkerArray, "/viz/scenario_obstacles", static_qos
        )

        obstacles: tuple[ScenarioObstacle, ...] = ()
        markings: tuple[FloorMarking, ...] = ()
        if scenario_name.strip().lower() != "track_map":
            share_directory = Path(get_package_share_directory("simp_planner_tools"))
            scenario = load_scenario_definition(share_directory, scenario_name)
            obstacles, markings = scenario.obstacles, scenario.markings
        markers = scenario_obstacle_markers(obstacles, frame_id)
        markers.markers.extend(floor_marking_markers(markings, frame_id))
        self.obstacle_pub.publish(markers)
        self.get_logger().info(
            f"published {len(obstacles)} scenario obstacles as pallet stacks/walls "
            f"({len(markers.markers)} markers, scenario={scenario_name})"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ScenarioObstacleVisualizerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
