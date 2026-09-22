"""Reference executor for the planner's published execution trajectory.

Samples whatever ExecutionTrajectory the planner (simp_planner_cpp) most
recently published on /planner/trajectory and forwards it to /cmd_vel as-is.
No position feedback controller -- see simp_tracker.execution_reference for
the sampling/braking/turn math, a Python port of simp_planner_cpp's
execution.hpp/execution.cpp shared with the planner-side producer.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time

from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import String
from simp_planner_msgs.msg import (
    DriveModeState, ExecutedCommand, ExecutionStatus, Trajectory, TrajectoryPoint,
)

from simp_tracker.execution_reference import (
    BodyCommand, ExecutionBuffer, ExecutionTrajectory, ReferenceSample,
    make_stop_samples, sample_execution, validate_execution_trajectory,
)

_COMMAND_FIELDS = (
    "vx", "vy", "yaw_rate", "planned_speed", "planned_acceleration", "planned_jerk",
    "motion_heading", "motion_curvature", "motion_heading_rate", "beta", "beta_rate",
    "yaw_acceleration", "trajectory_time", "segment_start_x", "segment_start_y",
    "segment_start_heading", "segment_end_x", "segment_end_y", "segment_end_heading",
)


def _command_message(c: BodyCommand) -> ExecutedCommand:
    m = ExecutedCommand()
    for name in _COMMAND_FIELDS:
        setattr(m, name, float(getattr(c, name)))
    m.motion_heading_acceleration = c.planned_heading_acceleration
    m.interval_index = int(c.action_index)
    return m


def _command_from_message(m: ExecutedCommand) -> BodyCommand:
    c = BodyCommand()
    for name in _COMMAND_FIELDS:
        setattr(c, name, float(getattr(m, name)))
    c.planned_heading_acceleration = m.motion_heading_acceleration
    c.action_index = m.interval_index
    return c


def _trajectory_from_message(m: Trajectory) -> ExecutionTrajectory:
    tr = ExecutionTrajectory(
        session_id=m.session_id, plan_id=m.plan_id, execution_epoch=m.execution_epoch,
        base_plan_id=m.base_plan_id, start_ns=Time.from_msg(m.start_time).nanoseconds,
        frame_id=m.header.frame_id, type=m.trajectory_type, reason=m.stop_reason,
        drive_mode=m.drive_mode, deceleration=m.fallback_deceleration, jerk=m.fallback_jerk,
    )
    tr.points = [
        ReferenceSample(time=p.time, x=p.x, y=p.y, body_yaw=p.body_yaw,
                        command=_command_from_message(p.command))
        for p in m.points
    ]
    validate_execution_trajectory(tr)
    if not math.isfinite(m.duration) or abs(m.duration - tr.points[-1].time) > 1.0e-9:
        raise ValueError("duration does not match last sample")
    return tr


class TrackerNode(Node):
    def __init__(self):
        super().__init__("simp_tracker")
        hz = float(self.declare_parameter("command_frequency_hz", 100.0).value)
        if not math.isfinite(hz) or hz <= 0:
            raise ValueError("invalid command frequency")
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.command_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.stamped_pub = self.create_publisher(TwistStamped, "/planner/cmd_vel_stamped", 200)
        self.executed_pub = self.create_publisher(ExecutedCommand, "/planner/executed_command", 200)
        self.state_pub = self.create_publisher(String, "/planner/execution_state", latched)
        self.status_pub = self.create_publisher(ExecutionStatus, "/tracker/execution_status", 10)

        self.create_subscription(Trajectory, "/planner/trajectory", self._on_trajectory,
                                 rclpy.qos.QoSProfile(depth=10,
                                     reliability=rclpy.qos.ReliabilityPolicy.RELIABLE))
        self.create_subscription(DriveModeState, "/vehicle/drive_mode_state", self._on_mode, latched)

        self.buffer = ExecutionBuffer()
        self.fault: ExecutionTrajectory | None = None
        self.fault_reason = ""
        self.last_sample: ReferenceSample | None = None
        self.mode: DriveModeState | None = None
        self.execution_started = False
        self.acceptance = ""
        self.state = "IDLE"
        self.published_state = ""
        self.received_id = 0
        self.last_status_ns = 0
        self.last_tick_ns = None  # for manual clock-jump detection; rclpy has no
                                  # direct equivalent of rclcpp's create_jump_callback.

        self.timer = self.create_timer(1.0 / hz, self._tick)
        self.get_logger().info(
            f"Reference executor ready at {hz:.1f} Hz (position feedback controller not enabled)")

    def _on_trajectory(self, msg: Trajectory) -> None:
        self.received_id = msg.plan_id
        if self.fault:
            self.acceptance = "FAULT_LATCHED"
            self._publish_status()
            return
        try:
            tr = _trajectory_from_message(msg)
            self.acceptance = self.buffer.receive(tr, self.get_clock().now().nanoseconds)
            if self.acceptance == "ACCEPTED":
                self.execution_started = True
        except (ValueError, ArithmeticError) as error:
            self.acceptance = str(error)
        if self.acceptance not in ("ACCEPTED", "DUPLICATE", "OLD_PLAN", "OLD_EPOCH"):
            self._engage_fault("PLAN_REJECTED")
        self._publish_status()

    def _on_mode(self, msg: DriveModeState) -> None:
        self.mode = msg

    def _engage_fault(self, reason: str) -> None:
        if self.fault:
            return
        stamp = self.get_clock().now().nanoseconds
        initial = self.last_sample if self.last_sample is not None else self.buffer.sample(stamp)
        fault = ExecutionTrajectory(start_ns=stamp)
        fault.points = make_stop_samples(initial, 4.0, 0.01, 1.0, 0.8)
        self.fault = fault
        self.fault_reason = reason
        self.get_logger().error(
            f"{reason}: local braking latched; restart planner/tracker together after inspection")

    def _tick(self) -> None:
        stamp = self.get_clock().now().nanoseconds
        # rclpy has no direct equivalent of rclcpp's create_jump_callback, so
        # this only catches backward steps (unambiguous: ROS time never goes
        # backwards except on an actual reset). A forward-gap heuristic was
        # tried and dropped -- ordinary executor/DDS scheduling jitter at
        # startup or under load can legitimately exceed any fixed threshold,
        # and that produced false CLOCK_RESET faults that never recovered.
        if self.last_tick_ns is not None and self.execution_started:
            if stamp < self.last_tick_ns:
                self._engage_fault("CLOCK_RESET")
        self.last_tick_ns = stamp

        self.buffer.advance(stamp)
        active = self.buffer.active
        p = sample_execution(self.fault, stamp) if self.fault else self.buffer.sample(stamp)

        moving = math.hypot(p.command.vx, p.command.vy) > 0.03 or abs(p.command.yaw_rate) > 0.02
        mode_ok = (self.mode is not None and self.mode.status == DriveModeState.STATUS_READY and
                   active is not None and self.mode.current_mode == active.drive_mode)
        if not self.fault and active and moving and not mode_ok:
            self._engage_fault("VEHICLE_MODE_MISMATCH")
            p = sample_execution(self.fault, stamp)

        if self.fault:
            self.state = "LOCAL_STOP_" + self.fault_reason
        elif self.buffer.expired(stamp):
            self.state = "SAFETY_STOP_EXPIRED"
        else:
            self.state = active.type if active else "IDLE"
        self.last_sample = p

        twist = Twist()
        twist.linear.x = p.command.vx
        twist.linear.y = p.command.vy
        twist.angular.z = p.command.yaw_rate
        self.command_pub.publish(twist)

        stamped = TwistStamped()
        stamped.header.stamp = Time(nanoseconds=stamp).to_msg()
        stamped.header.frame_id = "base_link"
        stamped.twist = twist
        self.stamped_pub.publish(stamped)

        executed = _command_message(p.command)
        executed.header = stamped.header
        executed.plan_id = active.plan_id if active else 0
        self.executed_pub.publish(executed)

        if self.state != self.published_state:
            self.state_pub.publish(String(data=self.state))
            self.published_state = self.state

        if stamp - self.last_status_ns >= 50_000_000 or stamp < self.last_status_ns:
            self._publish_status()
            self.last_status_ns = stamp

    def _publish_status(self) -> None:
        msg = ExecutionStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.session_id = self.buffer.session
        msg.execution_epoch = self.buffer.epoch
        msg.active_plan_id = self.buffer.active.plan_id if self.buffer.active else 0
        msg.pending_plan_id = self.buffer.pending.plan_id if self.buffer.pending else 0
        msg.received_plan_id = self.received_id
        msg.acceptance = self.acceptance
        msg.execution_state = ("LOCAL_STOP_" + self.fault_reason) if self.fault else self.state
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TrackerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
