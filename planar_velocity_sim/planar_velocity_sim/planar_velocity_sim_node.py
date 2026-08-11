#!/usr/bin/env python3

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from simp_planner_msgs.msg import DriveModeState
from std_msgs.msg import UInt8

from .kinematics import integrate_body_velocity
from .mode_transition import DriveModeTransitionModel, VALID_DRIVE_MODES


class PlanarVelocitySimNode(Node):
    """Integrate body-frame commands and report vehicle-confirmed drive mode."""

    def __init__(self) -> None:
        super().__init__("planar_velocity_sim")

        self.declare_parameter("update_rate_hz", 100.0)
        self.declare_parameter("mode_state_rate_hz", 20.0)
        self.declare_parameter("initial_x", 0.0)
        self.declare_parameter("initial_y", 0.0)
        self.declare_parameter("initial_yaw", 0.0)
        self.declare_parameter("initial_drive_mode", 0)
        self.declare_parameter("mode_transition_duration_sec", 2.0)
        self.declare_parameter("mode_change_speed_threshold", 0.03)
        self.declare_parameter("cmd_topic", "/cmd_vel")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("mode_command_topic", "/vehicle/drive_mode_command")
        self.declare_parameter("mode_state_topic", "/vehicle/drive_mode_state")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")

        update_rate_hz = float(self.get_parameter("update_rate_hz").value)
        mode_state_rate_hz = float(self.get_parameter("mode_state_rate_hz").value)
        if update_rate_hz <= 0.0 or mode_state_rate_hz <= 0.0:
            raise ValueError("update and mode-state rates must be positive")

        self.x = float(self.get_parameter("initial_x").value)
        self.y = float(self.get_parameter("initial_y").value)
        self.yaw = float(self.get_parameter("initial_yaw").value)
        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)

        self.command_vx = 0.0
        self.command_vy = 0.0
        self.command_yaw_rate = 0.0
        self.applied_vx = 0.0
        self.applied_vy = 0.0
        self.applied_yaw_rate = 0.0

        initial_mode = int(self.get_parameter("initial_drive_mode").value)
        if initial_mode not in VALID_DRIVE_MODES:
            raise ValueError("initial_drive_mode must be in [0, 3]")
        self.mode_model = DriveModeTransitionModel(
            initial_mode=initial_mode,
            transition_duration_sec=float(
                self.get_parameter("mode_transition_duration_sec").value
            ),
            stop_speed_threshold=float(
                self.get_parameter("mode_change_speed_threshold").value
            ),
        )

        static_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.cmd_sub = self.create_subscription(
            Twist,
            str(self.get_parameter("cmd_topic").value),
            self.cmd_vel_callback,
            20,
        )
        self.mode_command_sub = self.create_subscription(
            UInt8,
            str(self.get_parameter("mode_command_topic").value),
            self.mode_command_callback,
            static_qos,
        )
        self.odom_pub = self.create_publisher(
            Odometry, str(self.get_parameter("odom_topic").value), 20
        )
        self.mode_state_pub = self.create_publisher(
            DriveModeState,
            str(self.get_parameter("mode_state_topic").value),
            static_qos,
        )

        self.last_update_time = self.get_clock().now()
        self.update_timer = self.create_timer(1.0 / update_rate_hz, self.update)
        self.mode_timer = self.create_timer(1.0 / mode_state_rate_hz, self.publish_mode_state)
        self.publish_mode_state()

    def cmd_vel_callback(self, message: Twist) -> None:
        self.command_vx = float(message.linear.x)
        self.command_vy = float(message.linear.y)
        self.command_yaw_rate = float(message.angular.z)

    def mode_command_callback(self, message: UInt8) -> None:
        requested = int(message.data)
        if requested not in VALID_DRIVE_MODES:
            self.get_logger().error(f"Unsupported drive-mode command: {requested}")
            return
        speed = math.hypot(self.applied_vx, self.applied_vy)
        accepted = self.mode_model.command(
            requested, measured_speed=speed, now_sec=self.now_seconds()
        )
        if not accepted:
            self.get_logger().warning(
                f"Rejected mode command {requested}: vehicle speed={speed:.3f} m/s"
            )
            return
        if self.mode_model.transition_in_progress:
            self.command_vx = 0.0
            self.command_vy = 0.0
            self.command_yaw_rate = 0.0
            self.get_logger().info(
                f"Mode transition started: {self.mode_model.current_mode} -> {requested}"
            )
        self.publish_mode_state()

    def now_seconds(self) -> float:
        return self.get_clock().now().nanoseconds * 1.0e-9

    def publish_mode_state(self) -> None:
        feedback = self.mode_model.feedback()
        message = DriveModeState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.base_frame
        message.current_mode = feedback.current_mode
        message.requested_mode = feedback.requested_mode
        message.transition_in_progress = feedback.transition_in_progress
        message.transition_complete = feedback.transition_complete
        self.mode_state_pub.publish(message)

    def update(self) -> None:
        now = self.get_clock().now()
        dt = (now - self.last_update_time).nanoseconds * 1.0e-9
        self.last_update_time = now
        if dt <= 0.0:
            return

        completed = self.mode_model.update(now.nanoseconds * 1.0e-9)
        if completed:
            self.get_logger().info(
                f"Mode transition complete: current_mode={self.mode_model.current_mode}"
            )
            self.publish_mode_state()

        self.applied_vx, self.applied_vy, self.applied_yaw_rate = (
            self.mode_model.applied_velocity(
                self.command_vx, self.command_vy, self.command_yaw_rate
            )
        )
        self.x, self.y, self.yaw = integrate_body_velocity(
            self.x,
            self.y,
            self.yaw,
            self.applied_vx,
            self.applied_vy,
            self.applied_yaw_rate,
            dt,
        )

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(0.5 * self.yaw)
        odom.pose.pose.orientation.w = math.cos(0.5 * self.yaw)
        odom.twist.twist.linear.x = self.applied_vx
        odom.twist.twist.linear.y = self.applied_vy
        odom.twist.twist.angular.z = self.applied_yaw_rate
        self.odom_pub.publish(odom)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PlanarVelocitySimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
