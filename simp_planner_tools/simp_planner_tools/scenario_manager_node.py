from __future__ import annotations

import json
import math
from enum import Enum, auto
from pathlib import Path
from typing import Optional

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path as PathMessage
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from simp_planner_msgs.msg import DriveModeState
from simp_planner_msgs.msg import ReferencePath as ReferencePathMessage
from std_msgs.msg import Float64, String, UInt8

from .path_geometry import PathProjection
from .scenario_definition import (
    ScenarioDefinition,
    global_display_segments,
    load_scenario_definition,
    rasterize_scenario_costmap,
)
from .scenario_path import ScenarioPath




def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]:
    half = 0.5 * float(yaw)
    return 0.0, 0.0, math.sin(half), math.cos(half)


class ScenarioState(Enum):
    RUNNING = auto()
    STOPPING = auto()
    WAITING_MODE_CONFIRMATION = auto()
    COMPLETE = auto()


class ScenarioManagerNode(Node):
    """Publish scenario-dependent path, mode, speed, and occupancy costmap."""

    def __init__(self) -> None:
        super().__init__("scenario_manager_node")

        self.declare_parameter("scenario", "stadium")
        self.declare_parameter("target_speed", -1.0)
        self.declare_parameter("frame_id", "odom")
        self.declare_parameter("path_back_length", 5.0)
        self.declare_parameter("path_ahead_length", 45.0)
        self.declare_parameter("path_update_distance", 1.0)
        self.declare_parameter("costmap_resolution", -1.0)
        self.declare_parameter("costmap_margin", 10.0)
        self.declare_parameter("stop_speed_threshold", 0.03)
        self.declare_parameter("terminal_capture_distance", 0.20)
        self.declare_parameter("projection_search_back", 20)
        self.declare_parameter("projection_search_forward", 80)
        self.declare_parameter("projection_fallback_distance", 3.0)

        self.scenario_name = str(self.get_parameter("scenario").value).strip().lower()
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.path_back_length = float(self.get_parameter("path_back_length").value)
        self.path_ahead_length = float(self.get_parameter("path_ahead_length").value)
        self.path_update_distance = float(
            self.get_parameter("path_update_distance").value
        )
        requested_costmap_resolution = float(
            self.get_parameter("costmap_resolution").value
        )
        self.costmap_margin = float(self.get_parameter("costmap_margin").value)
        self.stop_speed_threshold = float(
            self.get_parameter("stop_speed_threshold").value
        )
        self.terminal_capture_distance = float(
            self.get_parameter("terminal_capture_distance").value
        )
        self.projection_search_back = int(
            self.get_parameter("projection_search_back").value
        )
        self.projection_search_forward = int(
            self.get_parameter("projection_search_forward").value
        )
        self.projection_fallback_distance = float(
            self.get_parameter("projection_fallback_distance").value
        )

        share_directory = Path(get_package_share_directory("simp_planner_tools"))
        self.scenario: ScenarioDefinition = load_scenario_definition(
            share_directory,
            self.scenario_name,
        )
        self.costmap_resolution = (
            float(self.scenario.costmap_resolution)
            if requested_costmap_resolution <= 0.0
            else requested_costmap_resolution
        )

        target_override = float(self.get_parameter("target_speed").value)
        self.target_override = None if target_override < 0.0 else target_override
        if self.path_back_length < 0.0 or self.path_ahead_length <= 0.0:
            raise ValueError("Invalid local reference lengths")
        if self.path_update_distance <= 0.0:
            raise ValueError("path_update_distance must be positive")
        if self.costmap_resolution <= 0.0 or self.costmap_margin <= 0.0:
            raise ValueError("Costmap resolution and margin must be positive")
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
        self.global_path_pub = self.create_publisher(
            PathMessage, "/scenario/global_path", static_qos
        )
        self.mode_pub = self.create_publisher(UInt8, "/requested_drive_mode", static_qos)
        self.speed_pub = self.create_publisher(Float64, "/target_speed", static_qos)
        self.costmap_pub = self.create_publisher(
            OccupancyGrid, "/costmap", static_qos
        )
        self.status_pub = self.create_publisher(
            String, "/scenario/status", static_qos
        )
        self.create_subscription(Odometry, "/odom", self.odom_callback, 50)
        self.create_subscription(
            DriveModeState,
            "/vehicle/drive_mode_state",
            self.vehicle_mode_callback,
            static_qos,
        )

        self.phase_paths = [phase.path for phase in self.scenario.phases]
        self.phase_index = 0
        for index, path in enumerate(tuple(self.phase_paths)):
            self.phase_paths[index] = self.execution_path_for_phase(index, path)
        self.state = ScenarioState.RUNNING
        self.mode_wait_start_time: Optional[float] = None
        self.last_projection: Optional[PathProjection] = None
        self.last_published_s: Optional[float] = None
        self.last_mode: Optional[int] = None
        self.last_target_speed: Optional[float] = None
        self.path_publish_count = 0
        self.phase_switch_count = 0
        # These values are reported by publish_status() during construction,
        # before the first odometry callback.  Initialize both explicitly so
        # the scenario manager cannot terminate with AttributeError at startup.
        self.current_remaining = math.nan
        self.current_stop_error = math.nan
        self.current_measured_speed = 0.0
        self.received_odom = False
        self.startup_republish_count = 0
        self.current_map_yaw = math.nan
        self.current_motion_yaw = math.nan
        self.vehicle_current_mode: Optional[int] = None
        self.vehicle_requested_mode: Optional[int] = None
        self.vehicle_transition_in_progress = False
        self.vehicle_transition_complete = False

        self.heartbeat_timer = self.create_timer(0.5, self.publish_heartbeat)
        self.publish_costmap()
        self.publish_global_path()
        self.publish_active_reference(0.0, force=True)
        self.publish_command(force=True)
        self.publish_status()

        self.get_logger().info(
            f"Scenario '{self.scenario.name}' ready: phases={len(self.scenario.phases)}, "
            f"obstacles={len(self.scenario.obstacles)}, "
            f"default_speed={self.active_cruise_speed():.2f} m/s"
        )

    @property
    def active_phase(self):
        return self.scenario.phases[self.phase_index]

    @property
    def active_path(self) -> ScenarioPath:
        return self.phase_paths[self.phase_index]

    def phase_stop_s(self, phase_index: int, path: ScenarioPath) -> float | None:
        if self.scenario.repeat:
            return None
        phase = self.scenario.phases[phase_index]
        if phase_index + 1 < len(self.scenario.phases) and phase.switch_s is not None:
            return float(np.clip(phase.switch_s, 0.0, path.total_length))
        return max(0.0, path.total_length - self.scenario.terminal_margin)

    def execution_path_for_phase(
        self, phase_index: int, path: ScenarioPath
    ) -> ScenarioPath:
        stop_s = self.phase_stop_s(phase_index, path)
        return path if stop_s is None else path.clipped(stop_s)

    def active_cruise_speed(self) -> float:
        if self.target_override is not None:
            return float(self.target_override)
        return float(self.active_phase.cruise_speed)

    def current_target_speed(self) -> float:
        if self.state in (
            ScenarioState.WAITING_MODE_CONFIRMATION,
            ScenarioState.COMPLETE,
        ):
            return 0.0
        return self.active_cruise_speed()

    def current_mode(self) -> int:
        return int(self.active_phase.mode)

    def elapsed_seconds(self) -> float:
        return self.get_clock().now().nanoseconds * 1.0e-9

    def publish_command(self, *, force: bool = False) -> None:
        target_speed = self.current_target_speed()
        mode = self.current_mode()
        if force or self.last_target_speed is None or abs(target_speed - self.last_target_speed) > 1.0e-12:
            speed_message = Float64()
            speed_message.data = target_speed
            self.speed_pub.publish(speed_message)
            self.last_target_speed = target_speed
        if force or self.last_mode != mode:
            mode_message = UInt8()
            mode_message.data = mode
            self.mode_pub.publish(mode_message)
            self.last_mode = mode

    def publish_heartbeat(self) -> None:
        speed_message = Float64()
        speed_message.data = self.current_target_speed()
        self.speed_pub.publish(speed_message)
        mode_message = UInt8()
        mode_message.data = self.current_mode()
        self.mode_pub.publish(mode_message)

        # Transient-local QoS should deliver the initial static inputs to late
        # subscribers.  Re-publish them for the first few heartbeats as an
        # additional startup safeguard and to make launch ordering irrelevant.
        if not self.received_odom and self.startup_republish_count < 10:
            self.publish_costmap()
            self.publish_global_path()
            self.publish_active_reference(0.0, force=True)
            self.startup_republish_count += 1
        self.publish_status()

    def publish_costmap(self) -> None:
        grid, origin_x, origin_y = rasterize_scenario_costmap(
            self.scenario,
            resolution=self.costmap_resolution,
            margin=self.costmap_margin,
        )
        message = OccupancyGrid()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.frame_id
        message.info.resolution = self.costmap_resolution
        message.info.width = int(grid.shape[1])
        message.info.height = int(grid.shape[0])
        message.info.origin.position.x = origin_x
        message.info.origin.position.y = origin_y
        message.info.origin.orientation.w = 1.0
        message.data = grid.reshape(-1).astype(int).tolist()
        self.costmap_pub.publish(message)

    def publish_global_path(self) -> None:
        message = PathMessage()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.frame_id
        for path in self.phase_paths:
            for x, y, yaw in zip(path.x, path.y, path.map_yaw):
                pose = PoseStamped()
                pose.header = message.header
                pose.pose.position.x = float(x)
                pose.pose.position.y = float(y)
                qx, qy, qz, qw = yaw_to_quaternion(float(yaw))
                pose.pose.orientation.x = qx
                pose.pose.orientation.y = qy
                pose.pose.orientation.z = qz
                pose.pose.orientation.w = qw
                message.poses.append(pose)
        self.global_path_pub.publish(message)

    def publish_active_reference(
        self,
        projection_s: float,
        *,
        force: bool = False,
    ) -> None:
        if not force and self.last_published_s is not None:
            if self.active_path.closed_loop:
                progress = (projection_s - self.last_published_s) % self.active_path.total_length
                if progress > 0.5 * self.active_path.total_length:
                    progress = 0.0
            else:
                progress = max(0.0, projection_s - self.last_published_s)
            if progress < self.path_update_distance:
                return

        local = self.active_path.local_slice(
            projection_s,
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
        for x, y, yaw in zip(local.x, local.y, local.yaw):
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
        self.last_published_s = float(projection_s)
        self.path_publish_count += 1

    def switch_to_next_phase(
        self,
        stop_x: float,
        stop_y: float,
        body_yaw: float,
    ) -> None:
        if self.phase_index + 1 >= len(self.scenario.phases):
            self.state = ScenarioState.COMPLETE
            self.publish_command(force=True)
            return
        next_index = self.phase_index + 1
        if self.scenario.name == "crab_switch" and next_index == 1:
            nominal = self.scenario.phases[next_index].path
            distance = nominal.total_length
            count = max(4, int(math.ceil(distance / 0.2)) + 1)
            progress = np.linspace(0.0, distance, count)
            motion_yaw = float(body_yaw + 0.5 * math.pi)
            generated = ScenarioPath.from_arrays(
                stop_x + progress * math.cos(motion_yaw),
                stop_y + progress * math.sin(motion_yaw),
                np.full(count, motion_yaw),
                np.zeros(count),
                np.full(count, 2, dtype=np.uint8),
                closed_loop=False,
            )
            self.phase_paths[next_index] = self.execution_path_for_phase(
                next_index, generated
            )
        elif self.scenario.name == "reverse_switch" and next_index == 1:
            # Keep the authored map heading unchanged.  Only translate the
            # pre-authored reverse phase so that it begins at the actual stop
            # position.  Motion direction is map_yaw + pi because mode=REVERSE.
            nominal = self.scenario.phases[next_index].path
            translated = nominal.translated(
                stop_x - float(nominal.x[0]),
                stop_y - float(nominal.y[0]),
            )
            self.phase_paths[next_index] = self.execution_path_for_phase(
                next_index, translated
            )
        self.phase_index = next_index
        self.phase_switch_count += 1
        self.state = ScenarioState.WAITING_MODE_CONFIRMATION
        self.mode_wait_start_time = self.elapsed_seconds()
        self.last_projection = None
        self.last_published_s = None
        self.publish_active_reference(0.0, force=True)
        self.publish_global_path()
        self.publish_command(force=True)
        self.get_logger().info(
            f"Stopped and requested phase '{self.active_phase.name}', "
            f"requested_mode={self.current_mode()}; waiting for vehicle confirmation"
        )

    def update_scenario_state(
        self,
        projection: PathProjection,
        speed: float,
        x: float,
        y: float,
        body_yaw: float,
    ) -> None:
        if self.scenario.repeat:
            return

        has_next_phase = self.phase_index + 1 < len(self.scenario.phases)
        stop_s = self.active_path.total_length
        self.current_stop_error = stop_s - float(projection.s)
        self.current_remaining = max(0.0, self.current_stop_error)

        if self.state in (
            ScenarioState.WAITING_MODE_CONFIRMATION,
            ScenarioState.COMPLETE,
        ):
            return

        # The active reference is clipped exactly at the phase stop target.
        # Keep the requested cruise speed unchanged so the planner's terminal
        # controller generates position-aware braking.  STOPPING is only a
        # supervisory state; it no longer sends a late zero-speed command.
        braking_monitor_distance = max(
            self.scenario.stop_request_distance,
            speed * speed / (2.0 * 0.85) + speed / 1.5 + 0.5,
        )
        if (
            self.state == ScenarioState.RUNNING
            and self.current_stop_error <= braking_monitor_distance
        ):
            self.state = ScenarioState.STOPPING
            self.get_logger().info(
                f"Terminal approach active: remaining={self.current_stop_error:.2f} m, "
                f"speed={speed:.2f} m/s"
            )

        captured = (
            abs(self.current_stop_error) <= self.terminal_capture_distance
            and speed <= self.stop_speed_threshold
        )
        if captured:
            self.switch_to_next_phase(x, y, body_yaw)

    def vehicle_mode_callback(self, message: DriveModeState) -> None:
        self.vehicle_current_mode = int(message.current_mode)
        self.vehicle_requested_mode = int(message.requested_mode)
        self.vehicle_transition_in_progress = bool(message.transition_in_progress)
        self.vehicle_transition_complete = bool(message.transition_complete)
        if (
            self.state == ScenarioState.WAITING_MODE_CONFIRMATION
            and self.vehicle_transition_complete
            and not self.vehicle_transition_in_progress
            and self.vehicle_current_mode == self.current_mode()
        ):
            self.state = ScenarioState.RUNNING
            self.mode_wait_start_time = None
            self.last_published_s = None
            self.publish_active_reference(0.0, force=True)
            self.publish_command(force=True)
            self.get_logger().info(
                f"Vehicle confirmed mode={self.vehicle_current_mode}. "
                f"Starting phase '{self.active_phase.name}'."
            )
        self.publish_status()

    def publish_status(self) -> None:
        payload = {
            "scenario": self.scenario.name,
            "state": self.state.name,
            "phase_index": self.phase_index,
            "phase_count": len(self.scenario.phases),
            "phase_name": self.active_phase.name,
            "requested_drive_mode": self.current_mode(),
            "vehicle_current_mode": self.vehicle_current_mode,
            "vehicle_requested_mode": self.vehicle_requested_mode,
            "vehicle_transition_in_progress": self.vehicle_transition_in_progress,
            "vehicle_transition_complete": self.vehicle_transition_complete,
            "target_speed": self.current_target_speed(),
            "measured_speed": self.current_measured_speed,
            "remaining_to_terminal": self.current_remaining,
            "signed_stop_error": self.current_stop_error,
            "switch_s": self.active_phase.switch_s,
            "stop_request_distance": self.scenario.stop_request_distance,
            "mode_confirmation_wait_sec": (0.0 if self.mode_wait_start_time is None else max(0.0, self.elapsed_seconds() - self.mode_wait_start_time)),
            "phase_switch_count": self.phase_switch_count,
            "path_publish_count": self.path_publish_count,
            "obstacle_count": len(self.scenario.obstacles),
            "heading_semantics": self.active_path.heading_semantics,
            "reference_map_yaw": self.current_map_yaw,
            "reference_motion_yaw": self.current_motion_yaw,
        }
        message = String()
        message.data = json.dumps(payload, allow_nan=True, separators=(",", ":"))
        self.status_pub.publish(message)

    def odom_callback(self, message: Odometry) -> None:
        if message.header.frame_id and message.header.frame_id != self.frame_id:
            self.get_logger().error(
                f"Expected odom frame '{self.frame_id}', received '{message.header.frame_id}'"
            )
            return
        x = float(message.pose.pose.position.x)
        y = float(message.pose.pose.position.y)
        orientation = message.pose.pose.orientation
        body_yaw = quaternion_to_yaw(
            orientation.x, orientation.y, orientation.z, orientation.w
        )
        vx = float(message.twist.twist.linear.x)
        vy = float(message.twist.twist.linear.y)
        if not all(math.isfinite(value) for value in (x, y, body_yaw, vx, vy)):
            return
        speed = math.hypot(vx, vy)
        self.current_measured_speed = speed
        self.received_odom = True

        projection = self.active_path.project(
            x,
            y,
            previous_segment=(
                None if self.last_projection is None else self.last_projection.segment_index
            ),
            search_back=self.projection_search_back,
            search_forward=self.projection_search_forward,
            fallback_distance=self.projection_fallback_distance,
        )
        self.last_projection = projection
        self.current_map_yaw = self.active_path.map_yaw_at(projection)
        self.current_motion_yaw = float(projection.yaw)
        self.publish_active_reference(projection.s)
        self.update_scenario_state(projection, speed, x, y, body_yaw)
        self.publish_command()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ScenarioManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
