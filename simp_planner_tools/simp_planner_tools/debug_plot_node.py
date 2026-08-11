from __future__ import annotations

import copy
import csv
import json
import math
import multiprocessing
from concurrent.futures import Future, ProcessPoolExecutor
from datetime import datetime
from pathlib import Path
from typing import Optional

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry, Path as PathMessage
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from simp_planner_msgs.msg import DriveModeState, ExecutedCommand
from simp_planner_msgs.msg import ReferencePath as ReferencePathMessage
from std_msgs.msg import Float64, String, UInt8

from .debug_allocation_metrics import compute_allocation_debug_sample
from .debug_plot_renderer import render_debug_snapshot
from .debug_signal_history import SourceTimeAligner
from .diagnostic_metrics import OpenPathGeometry, tracking_error_to_executed_segment
from .path_geometry import PathProjection, project_open_path, wrap_angle


MODE_NAMES = {0: "FORWARD", 1: "REVERSE", 2: "LEFT", 3: "RIGHT"}


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def motion_heading_from_body_yaw(body_yaw: float, drive_mode: int) -> float:
    offsets = {0: 0.0, 1: math.pi, 2: 0.5 * math.pi, 3: -0.5 * math.pi}
    return float(wrap_angle(body_yaw + offsets.get(int(drive_mode), 0.0)))


class DebugPlotNode(Node):
    """Record planning, execution tracking, continuity, and real-time diagnostics."""

    def __init__(self) -> None:
        super().__init__("debug_plot_node")
        self.declare_parameter("scenario", "stadium")
        self.declare_parameter(
            "output_dir", "/home/sum/Desktop/simp_planner/simp_planner_debug"
        )
        self.declare_parameter("save_period", 10.0)
        self.declare_parameter("frame_id", "odom")
        self.declare_parameter("vehicle_length", 3.0)
        self.declare_parameter("vehicle_width", 2.0)
        self.declare_parameter("movement_speed_threshold", 0.03)
        self.declare_parameter("global_lateral_limit", 1.0)
        self.declare_parameter("global_motion_limit_deg", 45.0)
        self.declare_parameter("tracking_lateral_limit", 0.20)
        self.declare_parameter("tracking_motion_limit_deg", 3.0)
        self.declare_parameter("planning_deadline_ms", 100.0)
        self.declare_parameter("dynamic_topic_timeout", 2.0)

        self.scenario_name = str(self.get_parameter("scenario").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.save_period = float(self.get_parameter("save_period").value)
        self.vehicle_length = float(self.get_parameter("vehicle_length").value)
        self.vehicle_width = float(self.get_parameter("vehicle_width").value)
        self.movement_speed_threshold = float(
            self.get_parameter("movement_speed_threshold").value
        )
        self.global_lateral_limit = float(
            self.get_parameter("global_lateral_limit").value
        )
        self.global_motion_limit = math.radians(
            float(self.get_parameter("global_motion_limit_deg").value)
        )
        self.tracking_lateral_limit = float(
            self.get_parameter("tracking_lateral_limit").value
        )
        self.tracking_motion_limit = math.radians(
            float(self.get_parameter("tracking_motion_limit_deg").value)
        )
        self.planning_deadline_ms = float(
            self.get_parameter("planning_deadline_ms").value
        )
        self.dynamic_topic_timeout = float(
            self.get_parameter("dynamic_topic_timeout").value
        )

        base = Path(str(self.get_parameter("output_dir").value)).expanduser()
        self.session_dir = (
            base / self.scenario_name / datetime.now().strftime("%Y%m%d_%H%M%S")
        )
        self.session_dir.mkdir(parents=True, exist_ok=False)

        self.csv_file = (self.session_dir / "odom_history.csv").open(
            "w", newline="", encoding="utf-8", buffering=1
        )
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(
            [
                "time", "receive_time", "callback_delay",
                "scenario", "scenario_state", "phase_name",
                "x", "y", "body_yaw", "motion_direction", "beta", "speed",
                "requested_target_speed", "applied_target_speed",
                "speed_cap_active", "terminal_hold_latched",
                "emergency_braking_active", "mode", "global_projected_s",
                "global_lateral_deviation", "global_direction_deviation",
                "tracking_lateral_error", "tracking_motion_error",
                "reference_curvature", "executed_curvature",
                "latest_cmd_publish_time", "latest_cmd_age",
                "latest_cmd_vx", "latest_cmd_vy", "latest_cmd_yaw_rate", "plan_id",
                "selected_candidate_id", "selected_n_target",
                "curvature_switch_jump", "total_compute_time_ms",
                "coarse_min_clearance", "precise_min_clearance",
                "allocation_min_clearance", "coarse_collision_free",
                "precise_collision_free", "allocation_collision_free",
                "planner_state", "planner_block_reason", "diagnosis",
            ]
        )
        self.command_csv_file = (self.session_dir / "command_history.csv").open(
            "w", newline="", encoding="utf-8", buffering=1
        )
        self.command_csv_writer = csv.writer(self.command_csv_file)
        self.command_csv_writer.writerow(
            [
                "publish_time", "receive_time", "callback_delay",
                "trajectory_time", "interval_index",
                "vx", "vy", "yaw_rate", "planned_speed",
                "planned_acceleration", "planned_jerk",
                "motion_curvature", "motion_heading_rate",
                "motion_heading_acceleration", "beta", "beta_center",
                "beta_deviation", "beta_rate", "beta_acceleration",
                "yaw_acceleration", "rate_split_residual",
                "speed_reconstruction_error", "mode", "plan_id",
            ]
        )

        self.plan_csv_file = (self.session_dir / "plan_history.csv").open(
            "w", newline="", encoding="utf-8", buffering=1
        )
        self.plan_csv_writer = csv.writer(self.plan_csv_file)
        self.plan_csv_writer.writerow(
            [
                "time", "plan_id", "candidate_id", "n_target",
                "lateral_selection_basis", "reference_center_lock_active",
                "candidate_switched", "terminal_offset_switched",
                "n_target_jump", "curvature_switch_jump",
                "heading_rate_switch_jump", "planner_core_time_ms",
                "total_compute_time_ms", "deadline_ms", "deadline_missed",
                "safe_paths", "requested_target_speed", "applied_target_speed",
                "obstacle_speed_cap_active", "speed_search_attempts",
                "spatial_preview_safe", "spatial_preview_rejected",
                "full_trajectory_rollouts", "terminal_hold_latched",
                "emergency_braking_plan", "emergency_collision_override",
                "coarse_min_clearance", "precise_min_clearance",
                "allocation_min_clearance", "coarse_collision_free",
                "precise_collision_free", "allocation_collision_free",
                "spatial_candidates", "terminal_feedback_rollouts",
                "terminal_spatial_screen_time_ms",
                "terminal_feedback_rollout_time_ms",
                "terminal_mode_active", "terminal_constraint_active",
                "precise_footprint", "allocation_footprint",
            ]
        )

        static_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(Odometry, "/odom", self.odom_callback, 50)
        self.create_subscription(
            ReferencePathMessage,
            "/reference_path_data",
            self.reference_callback,
            static_qos,
        )
        self.create_subscription(
            ReferencePathMessage,
            "/planner/selected_trajectory_data",
            self.selected_data_callback,
            static_qos,
        )
        self.create_subscription(
            PathMessage, "/reference_path", self.path_callback, static_qos
        )
        self.create_subscription(
            PathMessage, "/scenario/global_path", self.global_path_callback, static_qos
        )
        self.create_subscription(
            OccupancyGrid, "/costmap", self.costmap_callback, static_qos
        )
        self.create_subscription(
            Float64, "/target_speed", self.target_speed_callback, static_qos
        )
        self.create_subscription(
            UInt8, "/requested_drive_mode", self.requested_mode_callback, static_qos
        )
        self.create_subscription(
            DriveModeState,
            "/vehicle/drive_mode_state",
            self.vehicle_mode_callback,
            static_qos,
        )
        self.create_subscription(
            ExecutedCommand, "/planner/executed_command",
            self.executed_command_callback, 500
        )
        self.create_subscription(
            String, "/planner/status", self.planner_status_callback, static_qos
        )
        self.create_subscription(
            String, "/planner/execution_state",
            self.execution_state_callback, static_qos
        )
        self.create_subscription(
            String, "/scenario/status", self.scenario_status_callback, static_qos
        )
        self.create_subscription(
            PathMessage,
            "/planner/selected_trajectory",
            self.selected_path_callback,
            static_qos,
        )

        self.start_time = self.get_clock().now()
        self.last_seen: dict[str, Optional[float]] = {
            name: None
            for name in (
                "odom", "reference", "selected_trajectory", "global_path",
                "costmap", "target_speed", "requested_drive_mode",
                "vehicle_mode", "cmd_vel",
                "planner_status", "execution_state", "scenario_status",
            )
        }

        self.global_x = np.empty(0)
        self.global_y = np.empty(0)
        self.reference_x = np.empty(0)
        self.reference_y = np.empty(0)
        self.reference_yaw = np.empty(0)
        self.reference_kappa = np.empty(0)
        self.reference_s = np.empty(0)
        self.reference_segment_length = np.empty(0)
        self.selected_x = np.empty(0)
        self.selected_y = np.empty(0)
        self.selected_geometry: Optional[OpenPathGeometry] = None
        self.current_projection: Optional[PathProjection] = None
        self.selected_projection: Optional[PathProjection] = None
        self.current_state: Optional[dict[str, float]] = None
        self.last_motion_direction: Optional[float] = None

        self.costmap_data: Optional[np.ndarray] = None
        self.costmap_extent: Optional[tuple[float, float, float, float]] = None
        self.target_speed: Optional[float] = None
        self.mode: Optional[int] = None
        self.requested_mode: Optional[int] = None
        self.vehicle_transition_in_progress = False
        self.vehicle_transition_complete = False
        self.execution_state = "STARTUP"
        self.cmd_vx = 0.0
        self.cmd_vy = 0.0
        self.cmd_yaw_rate = 0.0
        self.latest_cmd_time = math.nan
        self.latest_cmd_plan_id = 0
        self.latest_command_segment: Optional[tuple[float, float, float, float, float, float]] = None
        self.odom_time_aligner = SourceTimeAligner()
        self.cmd_time_aligner = SourceTimeAligner()
        self.planner_status: dict[str, object] = {}
        self.scenario_status: dict[str, object] = {}
        self.last_logged_plan_id = 0

        self.odom_time_history: list[float] = []
        self.odom_receive_time_history: list[float] = []
        self.odom_callback_delay_history: list[float] = []
        self.x_history: list[float] = []
        self.y_history: list[float] = []
        self.speed_history: list[float] = []
        self.target_history: list[float] = []
        self.applied_target_history: list[float] = []
        self.cmd_time_history: list[float] = []
        self.cmd_receive_time_history: list[float] = []
        self.cmd_callback_delay_history: list[float] = []
        self.cmd_vx_history: list[float] = []
        self.cmd_vy_history: list[float] = []
        self.cmd_yaw_rate_history: list[float] = []
        self.command_speed_history: list[float] = []
        self.command_acceleration_history: list[float] = []
        self.command_jerk_history: list[float] = []
        # Direct allocator outputs.  These are copied from ExecutedCommand or
        # formed from exact allocator identities; no signal is differentiated.
        self.command_motion_heading_rate_history: list[float] = []
        self.command_motion_heading_acceleration_history: list[float] = []
        self.command_beta_history: list[float] = []
        self.command_beta_center_history: list[float] = []
        self.command_beta_deviation_history: list[float] = []
        self.command_beta_rate_history: list[float] = []
        self.command_beta_acceleration_history: list[float] = []
        self.command_yaw_acceleration_history: list[float] = []
        self.allocation_rate_split_residual_history: list[float] = []
        self.allocation_speed_reconstruction_error_history: list[float] = []
        self.speed_cap_history: list[bool] = []
        self.terminal_hold_history: list[bool] = []
        self.emergency_history: list[bool] = []
        self.global_lateral_history: list[float] = []
        self.global_motion_history: list[float] = []
        self.tracking_lateral_history: list[float] = []
        self.tracking_motion_history: list[float] = []
        self.beta_history: list[float] = []
        self.reference_kappa_history: list[float] = []
        self.executed_kappa_history: list[float] = []

        self.plan_time_history: list[float] = []
        self.plan_id_history: list[int] = []
        self.candidate_history: list[int] = []
        self.n_target_history: list[float] = []
        self.curvature_jump_history: list[float] = []
        self.plan_compute_history: list[float] = []
        self.plan_deadline_miss_history: list[int] = []
        self.plan_candidate_switch_history: list[int] = []
        self.plan_offset_switch_history: list[int] = []

        self.render_executor = ProcessPoolExecutor(
            max_workers=1,
            mp_context=multiprocessing.get_context("spawn"),
        )
        self.render_future: Optional[Future[str]] = None
        self.render_skip_count = 0
        self.save_timer = self.create_timer(self.save_period, self.save_output)
        self.get_logger().info(f"Debug output: {self.session_dir}")

    def elapsed(self) -> float:
        return (self.get_clock().now() - self.start_time).nanoseconds * 1.0e-9

    def mark(self, name: str) -> None:
        self.last_seen[name] = self.elapsed()

    def age(self, name: str) -> float:
        value = self.last_seen[name]
        return math.inf if value is None else self.elapsed() - value

    def reference_callback(self, message: ReferencePathMessage) -> None:
        self.mark("reference")
        self.reference_x = np.asarray(message.x, dtype=float)
        self.reference_y = np.asarray(message.y, dtype=float)
        self.reference_yaw = np.unwrap(np.asarray(message.yaw, dtype=float))
        self.reference_kappa = np.asarray(message.curvature, dtype=float)
        if len(self.reference_x) >= 2:
            self.reference_segment_length = np.hypot(
                np.diff(self.reference_x), np.diff(self.reference_y)
            )
            self.reference_s = np.r_[0.0, np.cumsum(self.reference_segment_length)]
        else:
            self.reference_segment_length = np.empty(0)
            self.reference_s = np.empty(0)
        self.current_projection = None

    def selected_data_callback(self, message: ReferencePathMessage) -> None:
        self.mark("selected_trajectory")
        try:
            self.selected_geometry = OpenPathGeometry.from_arrays(
                message.x, message.y, message.yaw, message.curvature
            )
            self.selected_x = self.selected_geometry.x
            self.selected_y = self.selected_geometry.y
            self.selected_projection = None
        except ValueError as exc:
            self.selected_geometry = None
            self.get_logger().warning(f"Rejected selected trajectory data: {exc}")

    def selected_path_callback(self, message: PathMessage) -> None:
        if self.selected_geometry is None:
            self.selected_x = np.asarray(
                [pose.pose.position.x for pose in message.poses], dtype=float
            )
            self.selected_y = np.asarray(
                [pose.pose.position.y for pose in message.poses], dtype=float
            )

    def path_callback(self, message: PathMessage) -> None:
        if self.reference_x.size == 0:
            self.reference_x = np.asarray(
                [pose.pose.position.x for pose in message.poses], dtype=float
            )
            self.reference_y = np.asarray(
                [pose.pose.position.y for pose in message.poses], dtype=float
            )

    def global_path_callback(self, message: PathMessage) -> None:
        self.mark("global_path")
        self.global_x = np.asarray(
            [pose.pose.position.x for pose in message.poses], dtype=float
        )
        self.global_y = np.asarray(
            [pose.pose.position.y for pose in message.poses], dtype=float
        )

    def costmap_callback(self, message: OccupancyGrid) -> None:
        self.mark("costmap")
        width = int(message.info.width)
        height = int(message.info.height)
        self.costmap_data = np.asarray(message.data, dtype=np.int16).reshape(height, width)
        x0 = float(message.info.origin.position.x)
        y0 = float(message.info.origin.position.y)
        resolution = float(message.info.resolution)
        self.costmap_extent = (
            x0, x0 + width * resolution, y0, y0 + height * resolution
        )

    def target_speed_callback(self, message: Float64) -> None:
        self.mark("target_speed")
        self.target_speed = float(message.data)

    def requested_mode_callback(self, message: UInt8) -> None:
        self.mark("requested_drive_mode")
        self.requested_mode = int(message.data)

    def vehicle_mode_callback(self, message: DriveModeState) -> None:
        self.mark("vehicle_mode")
        self.mode = int(message.current_mode)
        self.vehicle_transition_in_progress = bool(message.transition_in_progress)
        self.vehicle_transition_complete = bool(message.transition_complete)

    def executed_command_callback(self, message: ExecutedCommand) -> None:
        self.mark("cmd_vel")
        receive_time = self.elapsed()
        publish_time = self.cmd_time_aligner.align(
            message.header.stamp.sec,
            message.header.stamp.nanosec,
            receive_time,
        )
        vx = float(message.vx)
        vy = float(message.vy)
        yaw_rate = float(message.yaw_rate)
        planned_speed = float(message.planned_speed)
        planned_acceleration = float(message.planned_acceleration)
        planned_jerk = float(message.planned_jerk)
        allocation_debug = compute_allocation_debug_sample(
            vx=vx,
            vy=vy,
            yaw_rate=yaw_rate,
            planned_speed=planned_speed,
            motion_heading_rate=float(message.motion_heading_rate),
            motion_heading_acceleration=float(message.motion_heading_acceleration),
            beta=float(message.beta),
            beta_rate=float(message.beta_rate),
            yaw_acceleration=float(message.yaw_acceleration),
            mode=self.mode,
        )
        motion_heading_rate_deg = allocation_debug.motion_heading_rate_degps
        motion_heading_acceleration_deg = (
            allocation_debug.motion_heading_acceleration_degps2
        )
        beta_deg = allocation_debug.beta_deg
        beta_center_deg = allocation_debug.beta_center_deg
        beta_deviation_deg = allocation_debug.beta_deviation_deg
        beta_rate_deg = allocation_debug.beta_rate_degps
        beta_acceleration_deg = allocation_debug.beta_acceleration_degps2
        yaw_acceleration_deg = allocation_debug.yaw_acceleration_degps2
        rate_split_residual_deg = allocation_debug.rate_split_residual_degps
        speed_reconstruction_error = (
            allocation_debug.speed_reconstruction_error_mps
        )
        self.cmd_vx = vx
        self.cmd_vy = vy
        self.cmd_yaw_rate = yaw_rate
        self.latest_cmd_time = publish_time
        self.latest_cmd_plan_id = int(message.plan_id)
        segment = (
            float(message.segment_start_x),
            float(message.segment_start_y),
            float(message.segment_start_heading),
            float(message.segment_end_x),
            float(message.segment_end_y),
            float(message.segment_end_heading),
        )
        self.latest_command_segment = segment if all(math.isfinite(v) for v in segment) else None
        self.cmd_time_history.append(publish_time)
        self.cmd_receive_time_history.append(receive_time)
        self.cmd_callback_delay_history.append(max(0.0, receive_time - publish_time))
        self.cmd_vx_history.append(vx)
        self.cmd_vy_history.append(vy)
        self.cmd_yaw_rate_history.append(yaw_rate)
        self.command_speed_history.append(planned_speed)
        self.command_acceleration_history.append(planned_acceleration)
        self.command_jerk_history.append(planned_jerk)
        self.command_motion_heading_rate_history.append(motion_heading_rate_deg)
        self.command_motion_heading_acceleration_history.append(
            motion_heading_acceleration_deg
        )
        self.command_beta_history.append(beta_deg)
        self.command_beta_center_history.append(beta_center_deg)
        self.command_beta_deviation_history.append(beta_deviation_deg)
        self.command_beta_rate_history.append(beta_rate_deg)
        self.command_beta_acceleration_history.append(beta_acceleration_deg)
        self.command_yaw_acceleration_history.append(yaw_acceleration_deg)
        self.allocation_rate_split_residual_history.append(
            rate_split_residual_deg
        )
        self.allocation_speed_reconstruction_error_history.append(
            speed_reconstruction_error
        )
        self.command_csv_writer.writerow(
            [
                f"{publish_time:.6f}", f"{receive_time:.6f}",
                f"{max(0.0, receive_time - publish_time):.6f}",
                f"{float(message.trajectory_time):.6f}",
                int(message.interval_index),
                f"{vx:.9f}", f"{vy:.9f}", f"{yaw_rate:.9f}",
                f"{planned_speed:.9f}", f"{planned_acceleration:.9f}",
                f"{planned_jerk:.9f}", f"{float(message.motion_curvature):.9f}",
                f"{motion_heading_rate_deg:.9f}",
                f"{motion_heading_acceleration_deg:.9f}",
                f"{beta_deg:.9f}", f"{beta_center_deg:.9f}",
                f"{beta_deviation_deg:.9f}", f"{beta_rate_deg:.9f}",
                f"{beta_acceleration_deg:.9f}",
                f"{yaw_acceleration_deg:.9f}",
                f"{rate_split_residual_deg:.12f}",
                f"{speed_reconstruction_error:.12f}",
                "" if self.mode is None else self.mode, int(message.plan_id),
            ]
        )

    def execution_state_callback(self, message: String) -> None:
        self.mark("execution_state")
        self.execution_state = str(message.data).strip() or "UNKNOWN"

    def planner_status_callback(self, message: String) -> None:
        self.mark("planner_status")
        try:
            value = json.loads(message.data)
            self.planner_status = value if isinstance(value, dict) else {}
        except json.JSONDecodeError:
            self.planner_status = {"state": "INVALID_JSON"}
            return

        execution = self.planner_section("execution")
        plan = self.planner_section("plan")
        timing = self.planner_section("timing")
        plan_id = int(execution.get("current_plan_id", 0) or 0)
        if plan_id <= 0 or plan_id == self.last_logged_plan_id:
            return
        self.last_logged_plan_id = plan_id
        now = self.elapsed()
        candidate = int(plan.get("selected_candidate_id", -1) or -1)
        n_target = float(plan.get("selected_n_target", math.nan))
        curvature_jump = float(plan.get("curvature_switch_jump", math.nan))
        total_compute = float(plan.get("total_compute_time_ms", math.nan))
        deadline = float(timing.get("deadline_ms", self.planning_deadline_ms))
        deadline_missed = int(math.isfinite(total_compute) and total_compute > deadline)

        self.plan_time_history.append(now)
        self.plan_id_history.append(plan_id)
        self.candidate_history.append(candidate)
        self.n_target_history.append(n_target)
        self.curvature_jump_history.append(curvature_jump)
        self.plan_compute_history.append(total_compute)
        candidate_switched = int(bool(plan.get("candidate_switched", False)))
        offset_switched = int(bool(plan.get("terminal_offset_switched", False)))
        self.plan_deadline_miss_history.append(deadline_missed)
        self.plan_candidate_switch_history.append(candidate_switched)
        self.plan_offset_switch_history.append(offset_switched)
        self.plan_csv_writer.writerow(
            [
                f"{now:.6f}", plan_id, candidate, f"{n_target:.9f}",
                str(plan.get("lateral_selection_basis", "UNKNOWN")),
                bool(plan.get("reference_center_lock_active", False)),
                bool(candidate_switched),
                bool(offset_switched),
                f"{float(plan.get('n_target_jump', math.nan)):.9f}",
                f"{curvature_jump:.9f}",
                f"{float(plan.get('heading_rate_switch_jump', math.nan)):.9f}",
                f"{float(plan.get('compute_time_ms', math.nan)):.6f}",
                f"{total_compute:.6f}", f"{deadline:.6f}", deadline_missed,
                int(plan.get("num_safe_paths", 0) or 0),
                f"{float(plan.get('requested_target_speed', math.nan)):.6f}",
                f"{float(plan.get('applied_target_speed', math.nan)):.6f}",
                bool(plan.get("obstacle_speed_cap_active", False)),
                int(plan.get("speed_search_attempts", 0) or 0),
                int(plan.get("num_spatial_preview_safe", 0) or 0),
                int(plan.get("num_spatial_preview_rejected", 0) or 0),
                int(plan.get("num_full_trajectory_rollouts", 0) or 0),
                bool(self.planner_status.get("terminal_hold", {}).get("latched", False)),
                bool(plan.get("emergency_braking_plan", False)),
                bool(plan.get("emergency_collision_override", False)),
                f"{float(plan.get('coarse_min_clearance', math.nan)):.9f}",
                f"{float(plan.get('precise_min_clearance', math.nan)):.9f}",
                f"{float(plan.get('allocation_min_clearance', math.nan)):.9f}",
                bool(plan.get("coarse_collision_free", False)),
                bool(plan.get("precise_collision_free", False)),
                bool(plan.get("allocation_collision_free", False)),
                int(plan.get("num_spatial_candidates", 0) or 0),
                int(plan.get("num_terminal_feedback_rollouts", 0) or 0),
                f"{float(plan.get('terminal_spatial_screen_time_ms', 0.0)):.6f}",
                f"{float(plan.get('terminal_feedback_rollout_time_ms', 0.0)):.6f}",
                bool(plan.get("terminal_mode_active", False)),
                bool(plan.get("terminal_constraint_active", False)),
                str(plan.get("precise_footprint", "UNKNOWN")),
                str(plan.get("allocation_footprint", "UNKNOWN")),
            ]
        )

    def scenario_status_callback(self, message: String) -> None:
        self.mark("scenario_status")
        try:
            value = json.loads(message.data)
            self.scenario_status = value if isinstance(value, dict) else {}
        except json.JSONDecodeError:
            self.scenario_status = {"state": "INVALID_JSON"}

    def planner_section(self, name: str) -> dict[str, object]:
        value = self.planner_status.get(name, {})
        return value if isinstance(value, dict) else {}

    def project_reference(self, x: float, y: float) -> Optional[PathProjection]:
        if len(self.reference_x) < 2 or len(self.reference_segment_length) < 1:
            return None
        return project_open_path(
            x, y,
            x=self.reference_x,
            y=self.reference_y,
            yaw=self.reference_yaw,
            kappa=self.reference_kappa,
            s=self.reference_s,
            segment_length=self.reference_segment_length,
            previous_segment=(
                None if self.current_projection is None
                else self.current_projection.segment_index
            ),
            search_back=20,
            search_forward=80,
            fallback_distance=3.0,
        )

    def odom_callback(self, message: Odometry) -> None:
        self.mark("odom")
        receive_elapsed = self.elapsed()
        elapsed = self.odom_time_aligner.align(
            message.header.stamp.sec,
            message.header.stamp.nanosec,
            receive_elapsed,
        )
        pose = message.pose.pose
        twist = message.twist.twist
        x = float(pose.position.x)
        y = float(pose.position.y)
        body_yaw = quaternion_to_yaw(
            pose.orientation.x, pose.orientation.y,
            pose.orientation.z, pose.orientation.w,
        )
        vx = float(twist.linear.x)
        vy = float(twist.linear.y)
        speed = math.hypot(vx, vy)
        projection = self.project_reference(x, y)
        if projection is None:
            return
        self.current_projection = projection

        if speed > self.movement_speed_threshold:
            motion_direction = float(wrap_angle(body_yaw + math.atan2(vy, vx)))
        elif self.mode is not None:
            # At standstill, reconstruct the undefined motion direction from
            # the vehicle-confirmed mode. Do not retain the previous phase's
            # direction across a Forward -> Crab/Reverse transition.
            motion_direction = motion_heading_from_body_yaw(body_yaw, self.mode)
        elif self.last_motion_direction is not None:
            motion_direction = self.last_motion_direction
        else:
            motion_direction = projection.yaw
        self.last_motion_direction = motion_direction
        beta = float(wrap_angle(motion_direction - body_yaw))
        global_motion_error = float(wrap_angle(motion_direction - projection.yaw))

        tracking_lateral = math.nan
        tracking_motion = math.nan
        tracking_source = "N/A"
        command = self.planner_section("command")
        terminal_hold = bool(
            self.planner_section("terminal_hold").get("latched", False)
        )
        segment_values = self.latest_command_segment
        command_age = (
            math.inf if not math.isfinite(self.latest_cmd_time)
            else max(0.0, elapsed - self.latest_cmd_time)
        )
        segment_valid = (
            segment_values is not None
            and command_age <= 0.05
        )
        if terminal_hold:
            tracking_lateral = 0.0
            tracking_motion = 0.0
            tracking_source = "TERMINAL_HOLD"
        elif segment_valid:
            result = tracking_error_to_executed_segment(
                x, y, motion_direction,
                start_x=segment_values[0], start_y=segment_values[1],
                start_heading=segment_values[2],
                end_x=segment_values[3], end_y=segment_values[4],
                end_heading=segment_values[5],
            )
            tracking_lateral = result.lateral_error
            tracking_motion = result.motion_direction_error
            tracking_source = "EXECUTED_COMMAND_SEGMENT"
        elif (self.execution_state in {"ACTIVE_PLAN", "STARTUP", "UNKNOWN"}
              and self.selected_geometry is not None):
            # Startup fallback only. During normal execution the command segment
            # above is used so plan replacement cannot create artificial spikes.
            try:
                result = self.selected_geometry.tracking_error(
                    x, y, motion_direction,
                    previous_segment=(
                        None if self.selected_projection is None
                        else self.selected_projection.segment_index
                    ),
                )
                self.selected_projection = result.projection
                tracking_lateral = result.lateral_error
                tracking_motion = result.motion_direction_error
                tracking_source = "SELECTED_PATH_FALLBACK"
            except ValueError:
                self.selected_projection = None
        elif self.execution_state in {"SAFETY_STOP", "MODE_STOP", "MODE_WAIT"}:
            tracking_source = self.execution_state

        plan = self.planner_section("plan")
        execution = self.planner_section("execution")
        executed_kappa = float(command.get("motion_kappa", math.nan))
        plan_id = int(execution.get("current_plan_id", 0) or 0)


        self.current_state = {
            "x": x, "y": y, "body_yaw": body_yaw,
            "motion_direction": motion_direction, "beta": beta, "speed": speed,
            "global_lateral": float(projection.lateral_error),
            "global_motion_error": global_motion_error,
            "tracking_lateral": tracking_lateral,
            "tracking_motion_error": tracking_motion,
            "tracking_source": tracking_source,
            "reference_kappa": float(projection.kappa),
            "executed_kappa": executed_kappa,
            "plan_id": float(plan_id),
        }

        self.odom_time_history.append(elapsed)
        self.odom_receive_time_history.append(receive_elapsed)
        self.odom_callback_delay_history.append(max(0.0, receive_elapsed - elapsed))
        self.x_history.append(x)
        self.y_history.append(y)
        self.speed_history.append(speed)
        self.target_history.append(
            math.nan if self.target_speed is None else self.target_speed
        )
        applied_target = float(plan.get("applied_target_speed", math.nan))
        self.applied_target_history.append(applied_target)
        self.speed_cap_history.append(
            bool(plan.get("obstacle_speed_cap_active", False))
        )
        self.terminal_hold_history.append(
            bool(self.planner_status.get("terminal_hold", {}).get("latched", False))
        )
        self.emergency_history.append(
            bool(plan.get("emergency_braking_plan", False))
        )
        self.global_lateral_history.append(float(projection.lateral_error))
        self.global_motion_history.append(math.degrees(global_motion_error))
        self.tracking_lateral_history.append(tracking_lateral)
        self.tracking_motion_history.append(math.degrees(tracking_motion))
        self.beta_history.append(math.degrees(beta))
        self.reference_kappa_history.append(float(projection.kappa))
        self.executed_kappa_history.append(executed_kappa)

        diagnosis, _ = self.diagnose()
        self.csv_writer.writerow(
            [
                f"{elapsed:.6f}", f"{receive_elapsed:.6f}",
                f"{max(0.0, receive_elapsed - elapsed):.6f}",
                self.scenario_status.get("scenario", self.scenario_name),
                self.scenario_status.get("state", "UNKNOWN"),
                self.scenario_status.get("phase_name", "UNKNOWN"),
                f"{x:.9f}", f"{y:.9f}", f"{body_yaw:.9f}",
                f"{motion_direction:.9f}", f"{beta:.9f}", f"{speed:.9f}",
                "" if self.target_speed is None else f"{self.target_speed:.9f}",
                f"{applied_target:.9f}",
                bool(plan.get("obstacle_speed_cap_active", False)),
                bool(self.planner_status.get("terminal_hold", {}).get("latched", False)),
                bool(plan.get("emergency_braking_plan", False)),
                "" if self.mode is None else self.mode,
                f"{projection.s:.9f}", f"{projection.lateral_error:.9f}",
                f"{global_motion_error:.9f}", f"{tracking_lateral:.9f}",
                f"{tracking_motion:.9f}", f"{projection.kappa:.9f}",
                f"{executed_kappa:.9f}", f"{self.latest_cmd_time:.6f}",
                f"{(elapsed - self.latest_cmd_time) if math.isfinite(self.latest_cmd_time) else math.nan:.6f}",
                f"{self.cmd_vx:.9f}", f"{self.cmd_vy:.9f}",
                f"{self.cmd_yaw_rate:.9f}", plan_id,
                int(plan.get("selected_candidate_id", -1) or -1),
                f"{float(plan.get('selected_n_target', math.nan)):.9f}",
                f"{float(plan.get('curvature_switch_jump', math.nan)):.9f}",
                f"{float(plan.get('total_compute_time_ms', math.nan)):.6f}",
                f"{float(plan.get('coarse_min_clearance', math.nan)):.9f}",
                f"{float(plan.get('precise_min_clearance', math.nan)):.9f}",
                f"{float(plan.get('allocation_min_clearance', math.nan)):.9f}",
                bool(plan.get("coarse_collision_free", False)),
                bool(plan.get("precise_collision_free", False)),
                bool(plan.get("allocation_collision_free", False)),
                self.planner_status.get("state", "UNKNOWN"),
                self.planner_status.get("block_reason", "UNKNOWN"), diagnosis,
            ]
        )

    def diagnose(self) -> tuple[str, str]:
        if self.last_seen["odom"] is None:
            return "NO_ODOM", "No odometry received."
        if self.age("odom") > self.dynamic_topic_timeout:
            return "ODOM_STALE", f"odom age={self.age('odom'):.2f} s"
        for topic in (
            "reference", "costmap", "target_speed",
            "requested_drive_mode", "vehicle_mode",
        ):
            if self.last_seen[topic] is None:
                return f"NO_{topic.upper()}", f"No {topic} received."
        if self.last_seen["planner_status"] is None:
            return "NO_PLANNER_STATUS", "No planner status received."
        planner_state = str(self.planner_status.get("state", "UNKNOWN"))
        if planner_state == "BLOCKED":
            return "PLANNER_BLOCKED", str(
                self.planner_status.get("block_reason", "UNKNOWN")
            )

        plan = self.planner_section("plan")
        selected_target = float(plan.get("selected_n_target", math.nan))
        if (
            bool(plan.get("reference_center_lock_active", False))
            and math.isfinite(selected_target)
            and abs(selected_target) > 1.0e-6
        ):
            return (
                "LATERAL_SELECTION_INCONSISTENCY",
                f"center lock active but n_target={selected_target:.3f} m",
            )
        if (
            bool(plan.get("terminal_mode_active", False))
            and str(plan.get("lateral_selection_basis", "UNKNOWN"))
            != "SPATIAL_ONLY"
        ):
            return (
                "TERMINAL_LATERAL_COUPLING",
                "terminal plan is not using spatial-only lateral selection",
            )

        timing = self.planner_section("timing")
        miss_ratio = float(timing.get("deadline_miss_ratio", 0.0) or 0.0)
        p95 = float(timing.get("p95_ms", math.nan))
        if miss_ratio > 0.0:
            return (
                "PLANNING_DEADLINE_MISS",
                f"p95={p95:.1f} ms, miss ratio={100.0 * miss_ratio:.1f}%",
            )

        if self.vehicle_transition_in_progress:
            return (
                "MODE_TRANSITION_IN_PROGRESS",
                f"vehicle mode {self.mode} -> requested {self.requested_mode}",
            )
        if (
            self.requested_mode is not None
            and self.mode is not None
            and self.requested_mode != self.mode
        ):
            return (
                "MODE_CONFIRMATION_MISMATCH",
                f"requested mode={self.requested_mode}, vehicle mode={self.mode}",
            )

        if self.current_state is not None:
            tracking_lateral = abs(float(self.current_state["tracking_lateral"]))
            tracking_motion = abs(float(self.current_state["tracking_motion_error"]))
            if math.isfinite(tracking_lateral) and tracking_lateral > self.tracking_lateral_limit:
                return (
                    "EXECUTED_TRAJECTORY_TRACKING_ERROR",
                    f"tracking lateral error={tracking_lateral:.3f} m",
                )
            if math.isfinite(tracking_motion) and tracking_motion > self.tracking_motion_limit:
                return (
                    "EXECUTED_TRAJECTORY_DIRECTION_ERROR",
                    f"tracking direction error={math.degrees(tracking_motion):.2f} deg",
                )

            active_scenario = str(
                self.scenario_status.get("scenario", self.scenario_name)
            )
            obstacle_scenario = active_scenario in (
                "obstacle_avoidance",
                "s_curve_obstacles",
                "alternating_gate_corridor",
                "curved_gate_maze",
                "winding_obstacle_course",
                "narrow_22m_stop_corridor",
                "narrow_28m_corridor",
                "narrow_offset_corridor",
            )
            global_lateral_limit = 7.0 if obstacle_scenario else self.global_lateral_limit
            global_motion_limit = math.radians(70.0) if obstacle_scenario else self.global_motion_limit
            global_lateral = abs(float(self.current_state["global_lateral"]))
            global_motion = abs(float(self.current_state["global_motion_error"]))
            scenario_state = str(self.scenario_status.get("state", "RUNNING"))
            measured_speed = abs(float(self.current_state.get("speed", 0.0)))
            if scenario_state == "RUNNING" and global_lateral > global_lateral_limit:
                return "EXCESSIVE_GLOBAL_DEVIATION", f"global deviation={global_lateral:.3f} m"
            if (
                scenario_state == "RUNNING"
                and measured_speed > self.movement_speed_threshold
                and global_motion > global_motion_limit
            ):
                return (
                    "EXCESSIVE_GLOBAL_DIRECTION_DEVIATION",
                    f"global direction deviation={math.degrees(global_motion):.2f} deg",
                )

        command_speed = math.hypot(self.cmd_vx, self.cmd_vy)
        if command_speed > 1.0e-3 and self.mode is not None:
            if self.mode == 0 and self.cmd_vx < -1.0e-3:
                return "MODE_COMMAND_SIGN_MISMATCH", "FORWARD mode requires non-negative vx."
            if self.mode == 1 and self.cmd_vx > 1.0e-3:
                return "MODE_COMMAND_SIGN_MISMATCH", "REVERSE mode requires non-positive vx."
            if self.mode == 2 and self.cmd_vy < -1.0e-3:
                return "MODE_COMMAND_SIGN_MISMATCH", "LEFT mode requires non-negative vy."
            if self.mode == 3 and self.cmd_vy > 1.0e-3:
                return "MODE_COMMAND_SIGN_MISMATCH", "RIGHT mode requires non-positive vy."
        return "OK", "Scenario, selected-trajectory tracking, continuity, and timing are consistent."

    def vehicle_polygon(self, x: float, y: float, yaw: float) -> np.ndarray:
        corners = np.asarray(
            [
                [0.5 * self.vehicle_length, 0.5 * self.vehicle_width],
                [0.5 * self.vehicle_length, -0.5 * self.vehicle_width],
                [-0.5 * self.vehicle_length, -0.5 * self.vehicle_width],
                [-0.5 * self.vehicle_length, 0.5 * self.vehicle_width],
            ]
        )
        c = math.cos(yaw)
        s = math.sin(yaw)
        return corners @ np.asarray([[c, -s], [s, c]]).T + np.asarray([x, y])

    @staticmethod
    def projection_payload(value: Optional[PathProjection]) -> dict[str, float]:
        if value is None:
            return {}
        return {"x": float(value.x), "y": float(value.y)}

    def build_plot_snapshot(self, diagnosis: str, detail: str) -> dict[str, object]:
        plan = copy.deepcopy(self.planner_section("plan"))
        timing = copy.deepcopy(self.planner_section("timing"))
        execution = copy.deepcopy(self.planner_section("execution"))
        return {
            "scenario_name": self.scenario_name,
            "vehicle_length": self.vehicle_length,
            "vehicle_width": self.vehicle_width,
            "latest_cmd_vx": self.cmd_vx,
            "latest_cmd_vy": self.cmd_vy,
            "latest_cmd_yaw_rate": self.cmd_yaw_rate,
            "requested_drive_mode": self.requested_mode,
            "vehicle_drive_mode": self.mode,
            "vehicle_transition_in_progress": self.vehicle_transition_in_progress,
            "vehicle_transition_complete": self.vehicle_transition_complete,
            "planning_deadline_ms": self.planning_deadline_ms,
            "diagnosis": diagnosis,
            "detail": detail,
            "global_x": self.global_x.copy(), "global_y": self.global_y.copy(),
            "reference_x": self.reference_x.copy(), "reference_y": self.reference_y.copy(),
            "selected_x": self.selected_x.copy(), "selected_y": self.selected_y.copy(),
            "costmap_data": None if self.costmap_data is None else self.costmap_data.copy(),
            "costmap_extent": self.costmap_extent,
            "current_state": copy.deepcopy(self.current_state),
            "current_projection": self.projection_payload(self.current_projection),
            "selected_projection": self.projection_payload(self.selected_projection),
            "scenario_status": copy.deepcopy(self.scenario_status),
            "planner_state": self.planner_status.get("state", "N/A"),
            "hold_latched": bool(self.planner_status.get("terminal_hold", {}).get("latched", False)),
            "plan": plan, "timing": timing, "execution": execution,
            "odom_time_history": list(self.odom_time_history),
            "odom_receive_time_history": list(self.odom_receive_time_history),
            "odom_callback_delay_history": list(self.odom_callback_delay_history),
            "x_history": list(self.x_history), "y_history": list(self.y_history),
            "speed_history": list(self.speed_history),
            "target_history": list(self.target_history),
            "applied_target_history": list(self.applied_target_history),
            "cmd_time_history": list(self.cmd_time_history),
            "cmd_receive_time_history": list(self.cmd_receive_time_history),
            "cmd_callback_delay_history": list(self.cmd_callback_delay_history),
            "cmd_vx_history": list(self.cmd_vx_history),
            "cmd_vy_history": list(self.cmd_vy_history),
            "cmd_yaw_rate_history": list(self.cmd_yaw_rate_history),
            "command_speed_history": list(self.command_speed_history),
            "command_acceleration_history": list(self.command_acceleration_history),
            "command_jerk_history": list(self.command_jerk_history),
            "command_motion_heading_rate_history": list(
                self.command_motion_heading_rate_history
            ),
            "command_motion_heading_acceleration_history": list(
                self.command_motion_heading_acceleration_history
            ),
            "command_beta_history": list(self.command_beta_history),
            "command_beta_center_history": list(
                self.command_beta_center_history
            ),
            "command_beta_deviation_history": list(
                self.command_beta_deviation_history
            ),
            "command_beta_rate_history": list(self.command_beta_rate_history),
            "command_beta_acceleration_history": list(
                self.command_beta_acceleration_history
            ),
            "command_yaw_acceleration_history": list(
                self.command_yaw_acceleration_history
            ),
            "allocation_rate_split_residual_history": list(
                self.allocation_rate_split_residual_history
            ),
            "allocation_speed_reconstruction_error_history": list(
                self.allocation_speed_reconstruction_error_history
            ),
            "terminal_hold_history": list(self.terminal_hold_history),
            "global_lateral_history": list(self.global_lateral_history),
            "global_motion_history": list(self.global_motion_history),
            "tracking_lateral_history": list(self.tracking_lateral_history),
            "tracking_motion_history": list(self.tracking_motion_history),
            "beta_history": list(self.beta_history),
            "reference_kappa_history": list(self.reference_kappa_history),
            "executed_kappa_history": list(self.executed_kappa_history),
            "plan_time_history": list(self.plan_time_history),
            "n_target_history": list(self.n_target_history),
            "curvature_jump_history": list(self.curvature_jump_history),
            "plan_compute_history": list(self.plan_compute_history),
            "plan_deadline_miss_history": list(self.plan_deadline_miss_history),
        }

    def check_render_future(self) -> None:
        if self.render_future is None or not self.render_future.done():
            return
        try:
            saved_path = self.render_future.result()
            self.get_logger().info(f"Saved debug output: {saved_path}")
        except Exception as exc:  # pragma: no cover - ROS runtime path
            self.get_logger().error(f"Debug rendering failed: {exc}")
        finally:
            self.render_future = None

    def save_output(self) -> None:
        self.check_render_future()
        diagnosis, detail = self.diagnose()
        payload = {
            "diagnosis": diagnosis,
            "detail": detail,
            "scenario_status": self.scenario_status,
            "planner_status": self.planner_status,
            "current_state": self.current_state,
            "timestamp_logging": {
                "odom_samples": len(self.odom_time_history),
                "command_samples": len(self.cmd_time_history),
                "render_skip_count": self.render_skip_count,
            },
        }
        with (self.session_dir / "status_latest.json").open("w", encoding="utf-8") as file:
            json.dump(payload, file, ensure_ascii=False, indent=2, allow_nan=True)
        for file in (self.csv_file, self.command_csv_file, self.plan_csv_file):
            file.flush()

        if self.render_future is not None:
            self.render_skip_count += 1
            self.get_logger().warning("Skipped debug render because the worker is still busy.")
            return
        snapshot = self.build_plot_snapshot(diagnosis, detail)
        elapsed_int = int(round(self.elapsed()))
        self.render_future = self.render_executor.submit(
            render_debug_snapshot, snapshot, str(self.session_dir), elapsed_int
        )

    def destroy_node(self) -> bool:
        self.check_render_future()
        if self.render_future is not None:
            try:
                self.render_future.result(timeout=30.0)
            except Exception as exc:  # pragma: no cover - ROS runtime path
                self.get_logger().error(f"Final debug rendering failed: {exc}")
        self.render_executor.shutdown(wait=True, cancel_futures=False)
        for file in (self.csv_file, self.command_csv_file, self.plan_csv_file):
            if not file.closed:
                file.flush()
                file.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DebugPlotNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
