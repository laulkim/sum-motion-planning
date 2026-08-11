from __future__ import annotations

import math
from pathlib import Path
from typing import Optional

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path as PathMessage
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from simp_planner_msgs.msg import ReferencePath as ReferencePathMessage
from std_msgs.msg import Float64, UInt8

from .local_reference import (
    build_local_reference_slice,
    reference_index_from_projection,
    should_publish_local_reference,
)
from .path_geometry import PathProjection, project_closed_path
from .track_map import TrackMap


def yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]:
    half = 0.5 * yaw
    return 0.0, 0.0, math.sin(half), math.cos(half)


class TrackMapProviderNode(Node):
    """Publish a distance-decimated local path with authored yaw and curvature."""

    def __init__(self) -> None:
        super().__init__("track_map_provider_node")

        default_map = (
            Path(get_package_share_directory("simp_planner_tools"))
            / "maps"
            / "stadium_track.csv"
        )

        self.declare_parameter("map_file", str(default_map))
        self.declare_parameter("frame_id", "odom")
        self.declare_parameter("path_back_length", 5.0)
        self.declare_parameter("path_ahead_length", 45.0)
        self.declare_parameter("path_update_distance", 1.0)
        self.declare_parameter("target_speed", 2.0)
        self.declare_parameter("costmap_resolution", 0.5)
        self.declare_parameter("costmap_margin", 10.0)
        self.declare_parameter("projection_search_back", 20)
        self.declare_parameter("projection_search_forward", 80)
        self.declare_parameter("projection_fallback_distance", 3.0)

        self.frame_id = str(self.get_parameter("frame_id").value)
        self.path_back_length = float(self.get_parameter("path_back_length").value)
        self.path_ahead_length = float(self.get_parameter("path_ahead_length").value)
        self.path_update_distance = float(
            self.get_parameter("path_update_distance").value
        )
        self.target_speed = float(self.get_parameter("target_speed").value)
        self.costmap_resolution = float(self.get_parameter("costmap_resolution").value)
        self.costmap_margin = float(self.get_parameter("costmap_margin").value)
        self.projection_search_back = int(
            self.get_parameter("projection_search_back").value
        )
        self.projection_search_forward = int(
            self.get_parameter("projection_search_forward").value
        )
        self.projection_fallback_distance = float(
            self.get_parameter("projection_fallback_distance").value
        )

        if self.path_back_length < 0.0:
            raise ValueError("path_back_length must be non-negative")
        if self.path_ahead_length <= 0.0:
            raise ValueError("path_ahead_length must be positive")
        if self.path_update_distance <= 0.0:
            raise ValueError("path_update_distance must be positive")
        if self.target_speed < 0.0:
            raise ValueError("target_speed must be non-negative")
        if self.costmap_resolution <= 0.0:
            raise ValueError("costmap_resolution must be positive")
        if self.costmap_margin <= 0.0:
            raise ValueError("costmap_margin must be positive")
        if self.projection_search_back < 0 or self.projection_search_forward < 0:
            raise ValueError("projection search counts must be non-negative")
        if self.projection_fallback_distance <= 0.0:
            raise ValueError("projection_fallback_distance must be positive")

        map_file = Path(str(self.get_parameter("map_file").value)).expanduser()
        self.track = TrackMap.load_csv(map_file)

        static_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.path_data_pub = self.create_publisher(
            ReferencePathMessage, "/reference_path_data", static_qos
        )
        self.path_pub = self.create_publisher(
            PathMessage, "/reference_path", static_qos
        )
        self.mode_pub = self.create_publisher(UInt8, "/requested_drive_mode", static_qos)
        self.speed_pub = self.create_publisher(Float64, "/target_speed", static_qos)
        self.costmap_pub = self.create_publisher(
            OccupancyGrid, "/costmap", static_qos
        )
        self.create_subscription(Odometry, "/odom", self.odom_callback, 20)

        self.last_projection: Optional[PathProjection] = None
        self.last_published_s: Optional[float] = None
        self.last_reference_index: Optional[int] = None
        self.last_mode: Optional[int] = None
        self.path_publish_count = 0

        self.heartbeat_timer = self.create_timer(1.0, self.publish_heartbeat)
        self.publish_target_speed()
        self.publish_free_costmap()

        self.get_logger().info(
            f"Loaded clothoid track: {map_file}, points={len(self.track.x)}, "
            f"length={self.track.total_length:.2f} m, "
            f"target_speed={self.target_speed:.2f} m/s, "
            f"path_update_distance={self.path_update_distance:.2f} m"
        )

    def project_vehicle(self, x: float, y: float) -> PathProjection:
        return project_closed_path(
            x,
            y,
            x=self.track.x,
            y=self.track.y,
            yaw=self.track.yaw,
            kappa=self.track.kappa,
            s=self.track.s,
            segment_length=self.track.segment_length,
            total_length=self.track.total_length,
            previous_segment=(
                None
                if self.last_projection is None
                else self.last_projection.segment_index
            ),
            search_back=self.projection_search_back,
            search_forward=self.projection_search_forward,
            fallback_distance=self.projection_fallback_distance,
        )

    def publish_target_speed(self) -> None:
        message = Float64()
        message.data = self.target_speed
        self.speed_pub.publish(message)

    def publish_heartbeat(self) -> None:
        self.publish_target_speed()
        if self.last_mode is not None:
            mode_message = UInt8()
            mode_message.data = int(self.last_mode)
            self.mode_pub.publish(mode_message)

    def publish_free_costmap(self) -> None:
        minimum_x = float(np.min(self.track.x) - self.costmap_margin)
        maximum_x = float(np.max(self.track.x) + self.costmap_margin)
        minimum_y = float(np.min(self.track.y) - self.costmap_margin)
        maximum_y = float(np.max(self.track.y) + self.costmap_margin)

        width = int(math.ceil((maximum_x - minimum_x) / self.costmap_resolution))
        height = int(math.ceil((maximum_y - minimum_y) / self.costmap_resolution))

        message = OccupancyGrid()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.frame_id
        message.info.resolution = self.costmap_resolution
        message.info.width = width
        message.info.height = height
        message.info.origin.position.x = minimum_x
        message.info.origin.position.y = minimum_y
        message.info.origin.orientation.w = 1.0
        message.data = [0] * (width * height)
        self.costmap_pub.publish(message)

    def publish_local_path(
        self,
        reference_index: int,
        projection_s: float,
    ) -> None:
        local = build_local_reference_slice(
            self.track,
            reference_index,
            self.path_back_length,
            self.path_ahead_length,
        )
        now = self.get_clock().now().to_msg()

        data_message = ReferencePathMessage()
        data_message.header.stamp = now
        data_message.header.frame_id = self.frame_id
        data_message.x = local.x.tolist()
        data_message.y = local.y.tolist()
        data_message.yaw = local.yaw.tolist()
        data_message.curvature = local.kappa.tolist()
        data_message.mode = local.mode.tolist()
        data_message.closed_loop = False
        self.path_data_pub.publish(data_message)

        path_message = PathMessage()
        path_message.header = data_message.header
        for x, y, yaw in zip(data_message.x, data_message.y, data_message.yaw):
            pose = PoseStamped()
            pose.header = data_message.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            qx, qy, qz, qw = yaw_to_quaternion(float(yaw))
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw
            path_message.poses.append(pose)
        self.path_pub.publish(path_message)

        current_mode = int(self.track.mode[reference_index])
        if self.last_mode != current_mode:
            mode_message = UInt8()
            mode_message.data = current_mode
            self.mode_pub.publish(mode_message)
            self.last_mode = current_mode

        self.last_reference_index = int(reference_index)
        self.last_published_s = float(projection_s)
        self.path_publish_count += 1

    def odom_callback(self, message: Odometry) -> None:
        if message.header.frame_id and message.header.frame_id != self.frame_id:
            self.get_logger().error(
                f"Expected odom frame '{self.frame_id}', "
                f"received '{message.header.frame_id}'"
            )
            return

        x = float(message.pose.pose.position.x)
        y = float(message.pose.pose.position.y)
        if not all(math.isfinite(value) for value in (x, y)):
            self.get_logger().error("Odometry position contains NaN or Inf")
            return

        projection = self.project_vehicle(x, y)
        self.last_projection = projection
        reference_index = reference_index_from_projection(
            projection,
            len(self.track.x),
        )
        current_mode = int(self.track.mode[reference_index])
        mode_changed = self.last_mode is not None and current_mode != self.last_mode
        reverse = current_mode == 1

        if should_publish_local_reference(
            self.last_published_s,
            projection.s,
            self.track.total_length,
            self.path_update_distance,
            reverse=reverse,
            mode_changed=mode_changed,
        ):
            self.publish_local_path(reference_index, projection.s)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TrackMapProviderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
