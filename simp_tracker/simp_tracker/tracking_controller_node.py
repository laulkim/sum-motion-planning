"""Ao et al., Fractal Fract. 2023, 7, 121, equations (10) and (14).

Periodic body-velocity controller; no wheel allocation or event-triggered control.
"""

from bisect import bisect_right
import math

import rclpy
from geometry_msgs.msg import PoseStamped, Twist, Vector3Stamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from simp_planner_msgs.msg import DriveModeState, ExecutedCommand, Trajectory, TrajectoryPoint


ZERO = (0.0, 0.0, 0.0)


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def validate_trajectory(message):
    if not message.header.frame_id or message.plan_id == 0 or len(message.points) < 2:
        raise ValueError("trajectory needs a frame, plan_id and at least two points")
    if stamp_ns(message.start_time) < 0 or message.points[0].time_from_start != 0.0:
        raise ValueError("invalid trajectory time origin")
    previous_time = -1.0
    for point in message.points:
        if not all(math.isfinite(value) for value in (
            point.x, point.y, point.yaw, point.vx, point.vy,
            point.yaw_rate, point.time_from_start,
        )):
            raise ValueError("trajectory contains a non-finite value")
        if point.time_from_start <= previous_time or point.mode not in range(5):
            raise ValueError("trajectory times must increase and mode must be valid")
        previous_time = point.time_from_start
    if any(point.mode != message.points[0].mode for point in message.points):
        raise ValueError("a mode transition requires a separate plan")


def reference_at(message, elapsed):
    """Linear interpolation of a validated horizon; no extrapolation or time reset."""
    points = message.points
    if not math.isfinite(elapsed) or elapsed < 0.0 or elapsed > points[-1].time_from_start:
        return None
    if elapsed == points[-1].time_from_start:
        return points[-1]
    index = bisect_right(points, elapsed, key=lambda point: point.time_from_start) - 1
    left, right = points[index:index + 2]
    weight = (elapsed - left.time_from_start) / (right.time_from_start - left.time_from_start)
    reference = TrajectoryPoint()
    for field in ("x", "y", "vx", "vy", "yaw_rate"):
        a, b = getattr(left, field), getattr(right, field)
        setattr(reference, field, a + weight * (b - a))
    reference.yaw = wrap_angle(left.yaw + weight * wrap_angle(right.yaw - left.yaw))
    reference.time_from_start = elapsed
    reference.mode = left.mode
    return reference


def tracking_command(reference, pose, gains):
    """Body-frame error (10) and periodic velocity law (14), before saturation."""
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    dx, dy = reference.x - x, reference.y - y
    ex, ey = c * dx + s * dy, -s * dx + c * dy
    e_yaw = wrap_angle(reference.yaw - yaw)
    ce, se = math.cos(e_yaw), math.sin(e_yaw)
    kx, ky, k_yaw = gains
    command = (
        reference.vx * ce - reference.vy * se + kx * ex,
        reference.vy * ce + reference.vx * se + ky * ey,
        reference.yaw_rate + k_yaw * e_yaw,
    )
    return command, (ex, ey, e_yaw)


def limit_command(command, max_speed, max_yaw_rate):
    vx, vy, yaw_rate = command
    scale = min(1.0, max_speed / max(math.hypot(vx, vy), 1.0e-12))
    return vx * scale, vy * scale, max(-max_yaw_rate, min(max_yaw_rate, yaw_rate))


class TrackingController(Node):
    def __init__(self):
        super().__init__("tracking_controller")
        defaults = {
            "control_frequency_hz": 50.0,
            "kx": 3.0, "ky": 4.0, "k_yaw": 2.0,
            "max_linear_speed": 5.0, "max_yaw_rate": 1.0,
            "odom_timeout_sec": 0.25, "command_timeout_sec": 0.25,
            "mode_timeout_sec": 0.5,
        }
        values = {}
        for name, default in defaults.items():
            self.declare_parameter(name, default)
            value = float(self.get_parameter(name).value)
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive")
            values[name] = value
        self.declare_parameter("base_frame", "base_link")
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.gains = (values["kx"], values["ky"], values["k_yaw"])
        self.max_speed = values["max_linear_speed"]
        self.max_yaw_rate = values["max_yaw_rate"]
        self.odom_timeout = values["odom_timeout_sec"]
        self.command_timeout = values["command_timeout_sec"]
        self.mode_timeout = values["mode_timeout_sec"]
        self.plans = {}
        self.odom = self.nominal = self.mode = None
        self.reference = self.error = None
        self.last_tick_ns = None
        self.status = ""
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Trajectory, "/planner/trajectory", self.on_trajectory, latched)
        self.create_subscription(Odometry, "/odom", self.on_odom, 1)
        self.create_subscription(ExecutedCommand, "/planner/executed_command", self.on_command, 1)
        self.create_subscription(DriveModeState, "/vehicle/drive_mode_state", self.on_mode, latched)
        self.command_pub = self.create_publisher(Twist, "/cmd_vel", 1)
        self.status_pub = self.create_publisher(String, "/tracker/status", latched)
        self.reference_pub = self.create_publisher(PoseStamped, "/tracker/reference", 1)
        self.error_pub = self.create_publisher(Vector3Stamped, "/tracker/error", 1)
        self.create_timer(1.0 / values["control_frequency_hz"], self.tick)

    def on_trajectory(self, message):
        try:
            validate_trajectory(message)
        except ValueError as error:
            self.get_logger().warning(str(error))
            return
        if self.plans and message.plan_id <= max(self.plans):
            return
        if self.plans and stamp_ns(message.start_time) < max(
            stamp_ns(plan.start_time) for plan in self.plans.values()
        ):
            return
        self.plans[message.plan_id] = message
        # Only the active plan and its next replacement are needed.
        if len(self.plans) > 2:
            del self.plans[min(self.plans)]

    def on_odom(self, message):
        if self.odom is None or stamp_ns(message.header.stamp) >= stamp_ns(self.odom.header.stamp):
            self.odom = message

    def on_command(self, message):
        if self.nominal is None or stamp_ns(message.header.stamp) >= stamp_ns(self.nominal.header.stamp):
            self.nominal = message

    def on_mode(self, message):
        if self.mode is None or stamp_ns(message.header.stamp) >= stamp_ns(self.mode.header.stamp):
            self.mode = message

    @staticmethod
    def fresh(message, now_ns, timeout):
        return message is not None and 0 <= now_ns - stamp_ns(message.header.stamp) <= timeout * 1e9

    def calculate(self, now_ns):
        self.reference = self.error = None
        if self.last_tick_ns is not None and now_ns < self.last_tick_ns:
            self.plans.clear()
            self.odom = self.nominal = self.mode = None
            self.last_tick_ns = now_ns
            return ZERO, "CLOCK_RESET"
        self.last_tick_ns = now_ns
        if not self.fresh(self.nominal, now_ns, self.command_timeout):
            return ZERO, "WAIT_COMMAND"
        if not self.fresh(self.odom, now_ns, self.odom_timeout):
            return ZERO, "WAIT_ODOM"
        if not self.fresh(self.mode, now_ns, self.mode_timeout):
            return ZERO, "WAIT_MODE"
        nominal = self.nominal
        if nominal.header.frame_id != self.base_frame:
            return ZERO, "COMMAND_FRAME_MISMATCH"
        if not all(math.isfinite(value) for value in (nominal.vx, nominal.vy, nominal.yaw_rate)):
            return ZERO, "INVALID_COMMAND"
        if self.mode.status != DriveModeState.STATUS_READY:
            return ZERO, "MODE_ALIGNING"
        if self.mode.current_mode not in range(5) or self.mode.current_mode != self.mode.requested_mode:
            return ZERO, "MODE_MISMATCH"
        state = nominal.execution_state
        if state in {"SAFETY_STOP", "MODE_STOP", "SPOT_TURN_ROTATING"}:
            if state == "SPOT_TURN_ROTATING" and self.mode.current_mode != DriveModeState.SPOT_TURN:
                return ZERO, "MODE_MISMATCH"
            return limit_command((nominal.vx, nominal.vy, nominal.yaw_rate),
                                 self.max_speed, self.max_yaw_rate), state
        if state != "ACTIVE_PLAN":
            return ZERO, state or "WAIT_EXECUTION_STATE"
        plan = self.plans.get(nominal.plan_id)
        if plan is None:
            return ZERO, "WAIT_TRAJECTORY"
        if self.odom.header.frame_id != plan.header.frame_id or self.odom.child_frame_id != self.base_frame:
            return ZERO, "ODOM_FRAME_MISMATCH"
        elapsed = (now_ns - stamp_ns(plan.start_time)) * 1e-9
        reference = reference_at(plan, elapsed)
        if reference is None:
            return ZERO, "TRAJECTORY_NOT_STARTED" if elapsed < 0.0 else "TRAJECTORY_EXPIRED"
        if reference.mode != self.mode.current_mode:
            return ZERO, "MODE_MISMATCH"
        pose = self.odom.pose.pose
        q = pose.orientation
        if not all(math.isfinite(v) for v in (pose.position.x, pose.position.y, q.x, q.y, q.z, q.w)):
            return ZERO, "INVALID_ODOM"
        norm = math.hypot(q.z, q.w)
        if norm < 1e-12 or abs(q.x) > 1e-6 or abs(q.y) > 1e-6:
            return ZERO, "INVALID_ODOM"
        yaw = 2.0 * math.atan2(q.z / norm, q.w / norm)
        command, error = tracking_command(reference, (pose.position.x, pose.position.y, yaw), self.gains)
        if not all(math.isfinite(value) for value in command):
            return ZERO, "INVALID_CONTROL"
        self.reference, self.error = reference, error
        return limit_command(command, self.max_speed, self.max_yaw_rate), "TRACKING"

    def tick(self):
        now = self.get_clock().now()
        command, status = self.calculate(now.nanoseconds)
        message = Twist()
        message.linear.x, message.linear.y, message.angular.z = command
        self.command_pub.publish(message)
        if status != self.status:
            self.status = status
            self.status_pub.publish(String(data=status))
            self.get_logger().info(status)
        if self.reference is not None:
            reference = PoseStamped()
            reference.header.stamp = now.to_msg()
            reference.header.frame_id = self.odom.header.frame_id
            reference.pose.position.x = self.reference.x
            reference.pose.position.y = self.reference.y
            reference.pose.orientation.z = math.sin(self.reference.yaw / 2.0)
            reference.pose.orientation.w = math.cos(self.reference.yaw / 2.0)
            self.reference_pub.publish(reference)
            error = Vector3Stamped()
            error.header.stamp = now.to_msg()
            error.header.frame_id = self.base_frame
            error.vector.x, error.vector.y, error.vector.z = self.error
            self.error_pub.publish(error)


def main(args=None):
    rclpy.init(args=args)
    node = TrackingController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.command_pub.publish(Twist())
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
