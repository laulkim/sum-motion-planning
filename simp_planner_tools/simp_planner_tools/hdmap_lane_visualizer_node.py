"""FMTC HD map의 차선(B2_SURFACELINEMARK) 원본을 RViz용 MarkerArray로 publish한다.

hdmap_* 시나리오(hdmap_lap_switch, hdmap_crab1_switch, fmtc_demo)는 HDMap/build_*_scenario_maps.py
가 큰트랙 UTM 좌표에서 원점 하나를 빼서 로컬 "odom" 프레임을 만든다. 이 노드는
그 원점(HDMap/output/{scenario}_origin.txt)을 그대로 다시 읽어, 원본 차선
shapefile 좌표에서 같은 원점을 빼서 publish한다 -- 그래야 차선이 시나리오
경로/차량 위치와 같은 자리에 겹쳐 보인다. 다른(HD map 기반이 아닌) 시나리오를
쓸 때는 이 원점 자체가 의미 없으므로, origin 파일이나 shapefile이 없으면
경고만 남기고 조용히 아무것도 publish하지 않는다.

TRANSIENT_LOCAL QoS로 한 번만 publish한다 (RViz가 나중에 붙어도 받아본다) --
차선은 정적 데이터라 계속 다시 보낼 필요가 없다.

REFERENCE_PATH_GUIDE.md 10절 기준 Type: 111=황색실선, 211=백색실선,
212=백색점선, 311=청색실선. (211/212는 흰 배경에서 잘 안 보이니 회색으로,
점선 종류는 알파를 낮춰 구분만 해준다 -- Marker에는 실제 dash 스타일이 없다.)
"""
from __future__ import annotations

import os

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point
from osgeo import ogr
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray

ogr.UseExceptions()

# Type -> (r, g, b, alpha)  -- 진하게: 채도/알파 올리고 아래 scale.x도 굵게
LANE_STYLE = {
    "111": (0.95, 0.72, 0.0, 1.0),    # 황색 실선 (중앙선)
    "211": (0.25, 0.25, 0.25, 1.0),   # 백색 실선 -> 짙은 회색(흰 배경에서 안 보여서)
    "212": (0.25, 0.25, 0.25, 0.75),  # 백색 점선 -> 짙은 회색, 알파만 낮춰서 구분
    "311": (0.0, 0.35, 0.9, 1.0),     # 청색 실선
}
LANE_DEFAULT_STYLE = (0.5, 0.5, 0.5, 0.85)
LANE_LINE_WIDTH_M = 0.20


def read_origin(path: str) -> tuple[float, float]:
    with open(path, encoding="utf-8") as f:
        easting_str, northing_str = f.read().split()
    return float(easting_str), float(northing_str)


def load_lane_markers(
    shapefile_path: str, origin_easting: float, origin_northing: float, frame_id: str
) -> MarkerArray:
    dataset = ogr.Open(shapefile_path, 0)
    layer = dataset.GetLayer(0)
    array = MarkerArray()
    for marker_id, feature in enumerate(layer):
        geometry = feature.GetGeometryRef()
        lane_type = str(feature["Type"])
        r, g, b, alpha = LANE_STYLE.get(lane_type, LANE_DEFAULT_STYLE)

        marker = Marker()
        marker.header.frame_id = frame_id
        marker.ns = "hdmap_lanes"
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = LANE_LINE_WIDTH_M
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = r, g, b, alpha
        marker.points = [
            Point(
                x=geometry.GetPoint(i)[0] - origin_easting,
                y=geometry.GetPoint(i)[1] - origin_northing,
                z=0.0,
            )
            for i in range(geometry.GetPointCount())
        ]
        array.markers.append(marker)
    return array


class HDMapLaneVisualizerNode(Node):
    def __init__(self) -> None:
        super().__init__("hdmap_lane_visualizer_node")
        hdmap_dir = os.path.join(
            get_package_share_directory("simp_planner_tools"), "HDMap"
        )
        self.declare_parameter(
            "shapefile_path",
            os.path.join(hdmap_dir, "HDMAP", "B2_SURFACELINEMARK.shp"),
        )
        self.declare_parameter(
            "origin_file",
            os.path.join(hdmap_dir, "output", "hdmap_lap_switch_origin.txt"),
        )
        self.declare_parameter("frame_id", "odom")

        shapefile_path = str(self.get_parameter("shapefile_path").value)
        origin_file = str(self.get_parameter("origin_file").value)
        frame_id = str(self.get_parameter("frame_id").value)

        static_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.lane_pub = self.create_publisher(MarkerArray, "/viz/hdmap_lanes", static_qos)

        if not os.path.isfile(shapefile_path) or not os.path.isfile(origin_file):
            self.get_logger().warn(
                "HD map shapefile or origin file missing "
                f"(shapefile_path={shapefile_path}, origin_file={origin_file}); "
                "not an hdmap_* scenario, or paths need adjusting -- lane "
                "visualization disabled."
            )
            return

        origin_easting, origin_northing = read_origin(origin_file)
        markers = load_lane_markers(shapefile_path, origin_easting, origin_northing, frame_id)
        self.lane_pub.publish(markers)
        self.get_logger().info(
            f"published {len(markers.markers)} HD map lane markers "
            f"(origin=({origin_easting:.3f},{origin_northing:.3f}))"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HDMapLaneVisualizerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
