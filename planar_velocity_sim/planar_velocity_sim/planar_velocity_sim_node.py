#!/usr/bin/env python3

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from simp_planner_msgs.msg import DriveModeState
from std_msgs.msg import UInt8

from .kinematics import integrate_body_velocity
from .mode_transition import DriveModeTransitionModel, VALID_DRIVE_MODES
from .state_estimator import PlanarState, StateEstimatorConfig, StateEstimatorModel
from .vehicle_dynamics import (
    BodyVelocity,
    CommandChannel,
    CommandChannelConfig,
    PlanarVehicleDynamics,
    VehicleDynamicsConfig,
)


class PlanarVelocitySimNode(Node):
    """Integrate body-frame commands and report vehicle-confirmed drive mode."""

    def __init__(self) -> None:
        super().__init__("planar_velocity_sim")

        self.declare_parameter("update_rate_hz", 100.0)
        self.declare_parameter("estimator_rate_hz", 100.0)
        self.declare_parameter("mode_state_rate_hz", 20.0)
        self.declare_parameter("max_internal_step_sec", 0.01)
        self.declare_parameter("initial_x", 0.0)
        self.declare_parameter("initial_y", 0.0)
        self.declare_parameter("initial_yaw", 0.0)
        self.declare_parameter("initial_drive_mode", 0)
        self.declare_parameter("mode_transition_duration_sec", 2.0)
        self.declare_parameter("mode_change_speed_threshold", 0.03)
        self.declare_parameter("cmd_topic", "/cmd_vel")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("ground_truth_odom_topic", "/sim/ground_truth/odom")
        self.declare_parameter("applied_twist_topic", "/sim/applied_twist")
        self.declare_parameter("mode_command_topic", "/vehicle/drive_mode_command")
        self.declare_parameter("mode_state_topic", "/vehicle/drive_mode_state")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")

        self.declare_parameter("command.delay_sec", 0.0)
        self.declare_parameter("command.timeout_sec", 0.0)

        self.declare_parameter("dynamics.enabled", False)
        self.declare_parameter("dynamics.vx_time_constant_sec", 0.20)
        self.declare_parameter("dynamics.vy_time_constant_sec", 0.30)
        self.declare_parameter("dynamics.yaw_rate_time_constant_sec", 0.20)
        self.declare_parameter("dynamics.vx_command_gain", 1.0)
        self.declare_parameter("dynamics.vy_command_gain", 1.0)
        self.declare_parameter("dynamics.yaw_rate_command_gain", 1.0)
        self.declare_parameter("dynamics.max_vx_mps", 6.0)
        self.declare_parameter("dynamics.max_vy_mps", 6.0)
        self.declare_parameter("dynamics.max_yaw_rate_radps", 1.0)
        self.declare_parameter("dynamics.max_vx_acceleration_mps2", 1.5)
        self.declare_parameter("dynamics.max_vy_acceleration_mps2", 1.2)
        self.declare_parameter("dynamics.max_yaw_acceleration_radps2", 1.5)
        self.declare_parameter("dynamics.max_vx_jerk_mps3", 4.0)
        self.declare_parameter("dynamics.max_vy_jerk_mps3", 3.0)
        self.declare_parameter("dynamics.max_yaw_jerk_radps3", 5.0)
        self.declare_parameter(
            "dynamics.rolling_resistance_acceleration_mps2", 0.0
        )
        self.declare_parameter("dynamics.command_deadband_mps", 0.0)
        self.declare_parameter("dynamics.yaw_rate_deadband_radps", 0.0)
        self.declare_parameter("dynamics.stop_speed_threshold_mps", 1.0e-3)

        self.declare_parameter("estimator.enabled", False)
        self.declare_parameter("estimator.delay_sec", 0.0)
        self.declare_parameter("estimator.dropout_probability", 0.0)
        self.declare_parameter("estimator.position_noise_std_m", 0.0)
        self.declare_parameter("estimator.yaw_noise_std_rad", 0.0)
        self.declare_parameter("estimator.velocity_noise_std_mps", 0.0)
        self.declare_parameter("estimator.yaw_rate_noise_std_radps", 0.0)
        self.declare_parameter("estimator.position_bias_std_m", 0.0)
        self.declare_parameter("estimator.yaw_bias_std_rad", 0.0)
        self.declare_parameter("estimator.velocity_bias_std_mps", 0.0)
        self.declare_parameter("estimator.yaw_rate_bias_std_radps", 0.0)
        self.declare_parameter("estimator.bias_correlation_time_sec", 30.0)
        self.declare_parameter("estimator.position_resolution_m", 0.0)
        self.declare_parameter("estimator.yaw_resolution_rad", 0.0)
        self.declare_parameter("estimator.velocity_resolution_mps", 0.0)
        self.declare_parameter("estimator.yaw_rate_resolution_radps", 0.0)
        self.declare_parameter("estimator.random_seed", 42)

        update_rate_hz = float(self.get_parameter("update_rate_hz").value)
        estimator_rate_hz = float(self.get_parameter("estimator_rate_hz").value)
        mode_state_rate_hz = float(self.get_parameter("mode_state_rate_hz").value)
        self.max_internal_step_sec = float(
            self.get_parameter("max_internal_step_sec").value
        )
        if (
            update_rate_hz <= 0.0
            or estimator_rate_hz <= 0.0
            or mode_state_rate_hz <= 0.0
            or self.max_internal_step_sec <= 0.0
        ):
            raise ValueError("update, estimator, mode-state, and internal rates must be positive")

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

        self.command_channel = CommandChannel(
            CommandChannelConfig(
                delay_sec=float(self.get_parameter("command.delay_sec").value),
                timeout_sec=float(self.get_parameter("command.timeout_sec").value),
            )
        )
        self.vehicle_dynamics = PlanarVehicleDynamics(
            VehicleDynamicsConfig(
                enabled=bool(self.get_parameter("dynamics.enabled").value),
                vx_time_constant_sec=float(
                    self.get_parameter("dynamics.vx_time_constant_sec").value
                ),
                vy_time_constant_sec=float(
                    self.get_parameter("dynamics.vy_time_constant_sec").value
                ),
                yaw_rate_time_constant_sec=float(
                    self.get_parameter("dynamics.yaw_rate_time_constant_sec").value
                ),
                vx_command_gain=float(
                    self.get_parameter("dynamics.vx_command_gain").value
                ),
                vy_command_gain=float(
                    self.get_parameter("dynamics.vy_command_gain").value
                ),
                yaw_rate_command_gain=float(
                    self.get_parameter("dynamics.yaw_rate_command_gain").value
                ),
                max_vx_mps=float(self.get_parameter("dynamics.max_vx_mps").value),
                max_vy_mps=float(self.get_parameter("dynamics.max_vy_mps").value),
                max_yaw_rate_radps=float(
                    self.get_parameter("dynamics.max_yaw_rate_radps").value
                ),
                max_vx_acceleration_mps2=float(
                    self.get_parameter("dynamics.max_vx_acceleration_mps2").value
                ),
                max_vy_acceleration_mps2=float(
                    self.get_parameter("dynamics.max_vy_acceleration_mps2").value
                ),
                max_yaw_acceleration_radps2=float(
                    self.get_parameter("dynamics.max_yaw_acceleration_radps2").value
                ),
                max_vx_jerk_mps3=float(
                    self.get_parameter("dynamics.max_vx_jerk_mps3").value
                ),
                max_vy_jerk_mps3=float(
                    self.get_parameter("dynamics.max_vy_jerk_mps3").value
                ),
                max_yaw_jerk_radps3=float(
                    self.get_parameter("dynamics.max_yaw_jerk_radps3").value
                ),
                rolling_resistance_acceleration_mps2=float(
                    self.get_parameter(
                        "dynamics.rolling_resistance_acceleration_mps2"
                    ).value
                ),
                command_deadband_mps=float(
                    self.get_parameter("dynamics.command_deadband_mps").value
                ),
                yaw_rate_deadband_radps=float(
                    self.get_parameter("dynamics.yaw_rate_deadband_radps").value
                ),
                stop_speed_threshold_mps=float(
                    self.get_parameter("dynamics.stop_speed_threshold_mps").value
                ),
            )
        )
        self.state_estimator = StateEstimatorModel(
            StateEstimatorConfig(
                enabled=bool(self.get_parameter("estimator.enabled").value),
                delay_sec=float(self.get_parameter("estimator.delay_sec").value),
                dropout_probability=float(
                    self.get_parameter("estimator.dropout_probability").value
                ),
                position_noise_std_m=float(
                    self.get_parameter("estimator.position_noise_std_m").value
                ),
                yaw_noise_std_rad=float(
                    self.get_parameter("estimator.yaw_noise_std_rad").value
                ),
                velocity_noise_std_mps=float(
                    self.get_parameter("estimator.velocity_noise_std_mps").value
                ),
                yaw_rate_noise_std_radps=float(
                    self.get_parameter("estimator.yaw_rate_noise_std_radps").value
                ),
                position_bias_std_m=float(
                    self.get_parameter("estimator.position_bias_std_m").value
                ),
                yaw_bias_std_rad=float(
                    self.get_parameter("estimator.yaw_bias_std_rad").value
                ),
                velocity_bias_std_mps=float(
                    self.get_parameter("estimator.velocity_bias_std_mps").value
                ),
                yaw_rate_bias_std_radps=float(
                    self.get_parameter("estimator.yaw_rate_bias_std_radps").value
                ),
                bias_correlation_time_sec=float(
                    self.get_parameter("estimator.bias_correlation_time_sec").value
                ),
                position_resolution_m=float(
                    self.get_parameter("estimator.position_resolution_m").value
                ),
                yaw_resolution_rad=float(
                    self.get_parameter("estimator.yaw_resolution_rad").value
                ),
                velocity_resolution_mps=float(
                    self.get_parameter("estimator.velocity_resolution_mps").value
                ),
                yaw_rate_resolution_radps=float(
                    self.get_parameter("estimator.yaw_rate_resolution_radps").value
                ),
                random_seed=int(self.get_parameter("estimator.random_seed").value),
            )
        )

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
        self.ground_truth_odom_pub = self.create_publisher(
            Odometry,
            str(self.get_parameter("ground_truth_odom_topic").value),
            20,
        )
        self.applied_twist_pub = self.create_publisher(
            TwistStamped,
            str(self.get_parameter("applied_twist_topic").value),
            20,
        )
        self.mode_state_pub = self.create_publisher(
            DriveModeState,
            str(self.get_parameter("mode_state_topic").value),
            static_qos,
        )

        self.last_update_time = self.get_clock().now()
        initial_sec = self.last_update_time.nanoseconds * 1.0e-9
        self.command_channel.reset(BodyVelocity(), initial_sec)
        self.state_estimator.record_truth(self.true_state(initial_sec))
        self.update_timer = self.create_timer(1.0 / update_rate_hz, self.update)
        self.estimator_timer = self.create_timer(
            1.0 / estimator_rate_hz, self.publish_estimated_odometry
        )
        self.mode_timer = self.create_timer(1.0 / mode_state_rate_hz, self.publish_mode_state)
        self.publish_mode_state()

    def cmd_vel_callback(self, message: Twist) -> None:
        self.command_vx = float(message.linear.x)
        self.command_vy = float(message.linear.y)
        self.command_yaw_rate = float(message.angular.z)
        self.command_channel.push(
            BodyVelocity(
                self.command_vx, self.command_vy, self.command_yaw_rate
            ),
            self.now_seconds(),
        )

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
            self.command_channel.reset(BodyVelocity(), self.now_seconds())
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

    def true_state(self, stamp_sec: float) -> PlanarState:
        return PlanarState(
            stamp_sec=float(stamp_sec),
            x=self.x,
            y=self.y,
            yaw=self.yaw,
            vx=self.applied_vx,
            vy=self.applied_vy,
            yaw_rate=self.applied_yaw_rate,
        )

    def make_odometry(
        self,
        state: PlanarState,
        pose_covariance: tuple[float, ...] | None = None,
        twist_covariance: tuple[float, ...] | None = None,
    ) -> Odometry:
        odom = Odometry()
        odom.header.stamp = Time(
            nanoseconds=int(round(state.stamp_sec * 1.0e9))
        ).to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = state.x
        odom.pose.pose.position.y = state.y
        odom.pose.pose.orientation.z = math.sin(0.5 * state.yaw)
        odom.pose.pose.orientation.w = math.cos(0.5 * state.yaw)
        odom.twist.twist.linear.x = state.vx
        odom.twist.twist.linear.y = state.vy
        odom.twist.twist.angular.z = state.yaw_rate
        if pose_covariance is not None:
            odom.pose.covariance = list(pose_covariance)
        if twist_covariance is not None:
            odom.twist.covariance = list(twist_covariance)
        return odom

    def publish_estimated_odometry(self) -> None:
        estimate = self.state_estimator.estimate(self.now_seconds())
        if estimate is None:
            return
        self.odom_pub.publish(
            self.make_odometry(
                estimate.state,
                estimate.pose_covariance,
                estimate.twist_covariance,
            )
        )

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

        now_sec = now.nanoseconds * 1.0e-9
        delayed_command = self.command_channel.sample(now_sec)
        target_vx, target_vy, target_yaw_rate = self.mode_model.applied_velocity(
            delayed_command.vx,
            delayed_command.vy,
            delayed_command.yaw_rate,
        )
        target = BodyVelocity(target_vx, target_vy, target_yaw_rate)
        substep_count = max(1, int(math.ceil(dt / self.max_internal_step_sec)))
        substep_dt = dt / substep_count
        for _ in range(substep_count):
            actual = self.vehicle_dynamics.step(target, substep_dt)
            self.x, self.y, self.yaw = integrate_body_velocity(
                self.x,
                self.y,
                self.yaw,
                actual.vx,
                actual.vy,
                actual.yaw_rate,
                substep_dt,
            )
        self.applied_vx = actual.vx
        self.applied_vy = actual.vy
        self.applied_yaw_rate = actual.yaw_rate

        truth = self.true_state(now_sec)
        self.state_estimator.record_truth(truth)
        self.ground_truth_odom_pub.publish(self.make_odometry(truth))

        applied = TwistStamped()
        applied.header.stamp = now.to_msg()
        applied.header.frame_id = self.base_frame
        applied.twist.linear.x = self.applied_vx
        applied.twist.linear.y = self.applied_vy
        applied.twist.angular.z = self.applied_yaw_rate
        self.applied_twist_pub.publish(applied)


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
