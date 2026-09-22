"""Pure-Python port of simp_planner_cpp's execution.hpp/execution.cpp.

No ROS imports here on purpose: this module is the shared math between the
(C++) planner producer and this package's Python tracker_node consumer, and
it is unit-testable without rclpy. Field names and thresholds mirror the C++
source (simp_planner/simp_planner_cpp/{include/simp_planner,src}/execution.*)
line for line so the two implementations can be diffed against each other.
"""
from __future__ import annotations

from dataclasses import dataclass, field, replace
from bisect import bisect_right
import math

KPI = math.pi


def wrap_angle(angle: float) -> float:
    wrapped = math.fmod(angle + KPI, 2.0 * KPI)
    if wrapped < 0.0:
        wrapped += 2.0 * KPI
    return wrapped - KPI


def _angle_lerp(a: float, b: float, t: float) -> float:
    return wrap_angle(a + t * wrap_angle(b - a))


@dataclass
class BodyCommand:
    vx: float = 0.0
    vy: float = 0.0
    yaw_rate: float = 0.0
    planned_speed: float = 0.0
    planned_acceleration: float = 0.0
    planned_jerk: float = 0.0
    planned_heading_acceleration: float = 0.0
    motion_heading: float = 0.0
    motion_curvature: float = 0.0
    motion_heading_rate: float = 0.0
    beta: float = 0.0
    beta_rate: float = 0.0
    yaw_acceleration: float = 0.0
    segment_start_x: float = 0.0
    segment_start_y: float = 0.0
    segment_start_heading: float = 0.0
    segment_end_x: float = 0.0
    segment_end_y: float = 0.0
    segment_end_heading: float = 0.0
    action_index: int = 0
    trajectory_time: float = 0.0


@dataclass
class ReferenceSample:
    time: float = 0.0
    x: float = 0.0
    y: float = 0.0
    body_yaw: float = 0.0
    command: BodyCommand = field(default_factory=BodyCommand)


@dataclass
class ExecutionTrajectory:
    session_id: str = ""
    plan_id: int = 0
    execution_epoch: int = 0
    base_plan_id: int = 0
    start_ns: int = 0
    frame_id: str = "map"
    type: str = "ACTIVE_PLAN"
    reason: str = ""
    drive_mode: int = 0  # DriveMode.Forward
    deceleration: float = 1.0
    jerk: float = 0.8
    points: list = field(default_factory=list)


_VALID_TYPES = {
    "ACTIVE_PLAN", "MODE_STOP", "SAFETY_STOP", "HARD_STOP", "MODE_WAIT",
    "TERMINAL_HOLD", "SPOT_TURN_ROTATING", "SPOT_TURN_WAITING_CLEARANCE", "IDLE",
}


def validate_execution_trajectory(tr: ExecutionTrajectory) -> None:
    if (not tr.session_id or not tr.frame_id or tr.plan_id == 0 or tr.start_ns < 0 or
            len(tr.points) < 2 or len(tr.points) > 10001 or
            not math.isfinite(tr.deceleration) or tr.deceleration <= 0 or
            not math.isfinite(tr.jerk) or tr.jerk <= 0 or
            not (0 <= tr.drive_mode <= 4)):
        raise ValueError("invalid execution metadata")
    if tr.type not in _VALID_TYPES:
        raise ValueError("unknown execution type")
    previous = -1.0
    for p in tr.points:
        c = p.command
        values = (p.time, p.x, p.y, p.body_yaw, c.vx, c.vy, c.yaw_rate,
                  c.planned_speed, c.planned_acceleration, c.planned_jerk,
                  c.planned_heading_acceleration, c.motion_heading, c.motion_curvature,
                  c.motion_heading_rate, c.beta, c.beta_rate, c.yaw_acceleration)
        if not all(math.isfinite(v) for v in values):
            raise ValueError("non-finite execution sample")
        if p.time <= previous or p.time < 0 or p.time > 30.0 or c.planned_speed < -1.0e-8:
            raise ValueError("invalid execution time/speed")
        previous = p.time
    if abs(tr.points[0].time) > 1.0e-9:
        raise ValueError("execution must start at relative time zero")


class JerkLimitedSafetyStop:
    def __init__(self, speed: float, acceleration: float, beta: float,
                 motion_heading: float, curvature: float,
                 deceleration_limit: float, jerk_limit: float, elapsed: float = 0.0):
        self.speed = max(speed, 0.0)
        self.acceleration = acceleration
        self.beta = beta
        self.motion_heading = motion_heading
        self.curvature = curvature
        self.deceleration_limit = max(deceleration_limit, 1.0e-6)
        self.jerk_limit = max(jerk_limit, 1.0e-6)
        self.elapsed = elapsed

    @staticmethod
    def from_command(command: BodyCommand, deceleration_limit: float,
                      jerk_limit: float) -> "JerkLimitedSafetyStop":
        return JerkLimitedSafetyStop(
            command.planned_speed, command.planned_acceleration, command.beta,
            command.motion_heading, command.motion_curvature,
            deceleration_limit, jerk_limit, command.trajectory_time)

    def stopped(self) -> bool:
        return self.speed <= 1.0e-6 and abs(self.acceleration) <= 1.0e-6

    def _current_jerk(self, dt: float) -> float:
        if self.stopped():
            return 0.0
        braking_acceleration = min(self.acceleration, 0.0)
        release_speed = braking_acceleration * braking_acceleration / (2.0 * self.jerk_limit)
        release_margin = max(0.5 * abs(self.acceleration) * dt, 1.0e-5)
        if self.acceleration < -1.0e-9 and self.speed <= release_speed + release_margin:
            return self.jerk_limit
        if self.acceleration > -self.deceleration_limit + 1.0e-9:
            return -self.jerk_limit
        return 0.0

    def sample(self) -> BodyCommand:
        is_stopped = self.stopped()
        jerk = 0.0 if is_stopped else self._current_jerk(0.01)
        speed = 0.0 if is_stopped else max(self.speed, 0.0)
        acceleration = 0.0 if is_stopped else self.acceleration
        heading_rate = speed * self.curvature
        vx = speed * math.cos(self.beta)
        vy = speed * math.sin(self.beta)
        heading_accel = acceleration * self.curvature
        nan = math.nan
        return BodyCommand(
            vx=vx, vy=vy, yaw_rate=heading_rate, planned_speed=speed,
            planned_acceleration=acceleration, planned_jerk=jerk,
            planned_heading_acceleration=heading_accel, motion_heading=self.motion_heading,
            motion_curvature=self.curvature, motion_heading_rate=heading_rate,
            beta=self.beta, beta_rate=0.0, yaw_acceleration=heading_accel,
            segment_start_x=nan, segment_start_y=nan,
            segment_start_heading=self.motion_heading,
            segment_end_x=nan, segment_end_y=nan, segment_end_heading=self.motion_heading,
            action_index=0, trajectory_time=self.elapsed)

    def advance(self, dt: float) -> None:
        if not (dt > 0.0) or not math.isfinite(dt):
            raise ValueError("invalid safety stop dt")
        if self.stopped():
            return
        jerk = self._current_jerk(dt)
        next_acceleration = self.acceleration + jerk * dt
        upper = self.acceleration if self.acceleration > 0.0 else 0.0
        next_acceleration = max(-self.deceleration_limit, min(upper, next_acceleration))
        next_speed = self.speed + self.acceleration * dt + 0.5 * jerk * dt * dt
        if next_speed <= 0.0:
            self.speed = 0.0
            self.acceleration = 0.0
        else:
            self.speed = next_speed
            self.acceleration = next_acceleration
        self.elapsed += dt

    def sample_and_advance(self, dt: float) -> BodyCommand:
        result = self.sample()
        self.advance(dt)
        return result


def _integrate(p: ReferenceSample, c: BodyCommand, dt: float) -> None:
    yaw = p.body_yaw + 0.5 * c.yaw_rate * dt
    p.x += (math.cos(yaw) * c.vx - math.sin(yaw) * c.vy) * dt
    p.y += (math.sin(yaw) * c.vx + math.cos(yaw) * c.vy) * dt
    p.body_yaw = wrap_angle(p.body_yaw + c.yaw_rate * dt)


def _geometry(p: ReferenceSample) -> None:
    c = p.command
    c.segment_start_x = c.segment_end_x = p.x
    c.segment_start_y = c.segment_end_y = p.y
    c.motion_heading = wrap_angle(p.body_yaw + c.beta)
    c.segment_start_heading = c.segment_end_heading = c.motion_heading
    c.trajectory_time = p.time


def _braking(initial: ReferenceSample, horizon: float, dt: float,
             deceleration: float, jerk: float) -> list:
    stop = JerkLimitedSafetyStop.from_command(initial.command, deceleration, jerk)
    beta0 = initial.command.beta
    rate0 = initial.command.beta_rate
    state = {"t": 0.0}

    def command_at(elapsed: float) -> BodyCommand:
        while state["t"] < elapsed - 1.0e-10:
            step = min(0.01, elapsed - state["t"])
            stop.advance(step)
            state["t"] += step
        c = stop.sample()
        u = min(elapsed, 0.5)
        c.beta = beta0 + rate0 * (u - u * u)
        c.beta_rate = rate0 * max(0.0, 1.0 - 2.0 * elapsed)
        c.vx = c.planned_speed * math.cos(c.beta)
        c.vy = c.planned_speed * math.sin(c.beta)
        c.yaw_rate = c.motion_heading_rate - c.beta_rate
        if elapsed < 0.5:
            c.yaw_acceleration += 2.0 * rate0
        return c

    return make_reference_samples(initial, horizon, dt, command_at)


def make_reference_samples(p: ReferenceSample, horizon: float, dt: float, command_at) -> list:
    if not math.isfinite(horizon) or horizon <= 0 or horizon > 120 or \
            not math.isfinite(dt) or dt <= 0:
        raise ValueError("invalid sampling interval")
    p = replace(p, command=replace(p.command))
    result = []
    p.time = 0.0
    p.command = command_at(0.0)
    _geometry(p)
    result.append(replace(p, command=replace(p.command)))
    while p.time < horizon - 1.0e-10:
        step = min(dt, horizon - p.time)
        nxt = command_at(p.time + step)
        mid = replace(p.command)
        mid.vx = 0.5 * (mid.vx + nxt.vx)
        mid.vy = 0.5 * (mid.vy + nxt.vy)
        mid.yaw_rate = 0.5 * (mid.yaw_rate + nxt.yaw_rate)
        _integrate(p, mid, step)
        p.time += step
        p.command = nxt
        _geometry(p)
        result[-1].command.segment_end_x = p.x
        result[-1].command.segment_end_y = p.y
        result[-1].command.segment_end_heading = p.command.motion_heading
        result.append(replace(p, command=replace(p.command)))
    return result


def make_stop_samples(p: ReferenceSample, horizon: float, dt: float,
                       deceleration: float, jerk: float) -> list:
    return _braking(p, horizon, dt, deceleration, jerk)


def sample_braking(initial: ReferenceSample, elapsed: float,
                    deceleration: float, jerk: float) -> ReferenceSample:
    if elapsed <= 0:
        return initial
    points = _braking(initial, min(elapsed, 120.0), 0.01, deceleration, jerk)
    return points[-1]


def make_turn_samples(p: ReferenceSample, target: float, horizon: float, dt: float,
                       yaw_rate_max: float, yaw_rate_accel_max: float,
                       yaw_tolerance_rad: float) -> list:
    state = {"yaw": p.body_yaw, "rate": p.command.yaw_rate, "previous_t": 0.0}

    def command_at(t: float) -> BodyCommand:
        c = BodyCommand()
        step = t - state["previous_t"]
        error = wrap_angle(target - state["yaw"])
        if abs(error) <= yaw_tolerance_rad:
            desired = 0.0
        else:
            desired = math.copysign(
                min(yaw_rate_max, math.sqrt(2.0 * yaw_rate_accel_max * abs(error))), error)
        old = state["rate"]
        delta = max(-yaw_rate_accel_max * step, min(yaw_rate_accel_max * step, desired - old))
        state["rate"] = old + delta
        state["yaw"] = wrap_angle(state["yaw"] + 0.5 * (old + state["rate"]) * step)
        state["previous_t"] = t
        c.yaw_rate = state["rate"]
        c.yaw_acceleration = (state["rate"] - old) / step if step > 0 else p.command.yaw_acceleration
        c.beta_rate = -state["rate"]
        c.beta = wrap_angle(p.body_yaw - state["yaw"])
        return c

    return make_reference_samples(p, horizon, dt, command_at)


def sample_execution(tr: ExecutionTrajectory, stamp_ns: int) -> ReferenceSample:
    t = 1.0e-9 * (stamp_ns - tr.start_ns)
    if t <= 0:
        return tr.points[0]
    if t > tr.points[-1].time:
        p = sample_braking(tr.points[-1], t - tr.points[-1].time, tr.deceleration, tr.jerk)
        p = replace(p, time=t, command=replace(p.command))
        _geometry(p)
        return p
    times = [pt.time for pt in tr.points]
    index = bisect_right(times, t)
    if index >= len(tr.points):
        return tr.points[-1]
    a = tr.points[index - 1]
    b = tr.points[index]
    alpha = (t - a.time) / (b.time - a.time)

    def lerp(x, y):
        return x + alpha * (y - x)

    p = replace(a, command=replace(a.command))
    p.time = t
    p.x = lerp(a.x, b.x)
    p.y = lerp(a.y, b.y)
    p.body_yaw = _angle_lerp(a.body_yaw, b.body_yaw, alpha)
    for field_name in ("vx", "vy", "yaw_rate", "planned_speed", "planned_acceleration",
                       "planned_jerk", "planned_heading_acceleration", "motion_curvature",
                       "motion_heading_rate", "beta_rate", "yaw_acceleration"):
        setattr(p.command, field_name,
                lerp(getattr(a.command, field_name), getattr(b.command, field_name)))
    p.command.beta = _angle_lerp(a.command.beta, b.command.beta, alpha)
    _geometry(p)
    p.command.segment_end_x = b.x
    p.command.segment_end_y = b.y
    p.command.segment_end_heading = b.command.motion_heading
    return p


class ExecutionBuffer:
    """No locks: the owning node must serialize receive/advance/sample calls
    (a single-threaded rclpy executor tick already does this)."""

    def __init__(self):
        self.session = ""
        self.epoch = 0
        self.latest_id = 0
        self.active = None
        self.pending = None

    def receive(self, tr: ExecutionTrajectory, now_ns: int) -> str:
        try:
            validate_execution_trajectory(tr)
        except ValueError as error:
            return str(error)
        self.advance(now_ns)
        if self.session and tr.session_id != self.session:
            return "SESSION_MISMATCH"
        if tr.execution_epoch < self.epoch:
            return "OLD_EPOCH"
        if tr.execution_epoch == self.epoch and tr.plan_id == self.latest_id:
            return "DUPLICATE"
        if tr.plan_id <= self.latest_id:
            return "OLD_PLAN"
        if tr.start_ns <= now_ns:
            return "LATE_PLAN"
        if self.active and tr.frame_id != self.active.frame_id:
            return "FRAME_MISMATCH"
        base = self.active.plan_id if self.active else 0
        if tr.base_plan_id != base:
            return "BASE_MISMATCH"
        if self.pending and tr.execution_epoch == self.epoch:
            return "PENDING_EXISTS"
        if self.active:
            expected = sample_execution(self.active, tr.start_ns)
            first = tr.points[0]
            if (math.hypot(expected.x - first.x, expected.y - first.y) > 0.03 or
                    abs(wrap_angle(expected.body_yaw - first.body_yaw)) > 0.03 or
                    math.hypot(expected.command.vx - first.command.vx,
                              expected.command.vy - first.command.vy) > 0.05 or
                    abs(expected.command.yaw_rate - first.command.yaw_rate) > 0.05 or
                    abs(expected.command.planned_acceleration -
                        first.command.planned_acceleration) > 0.1):
                return "DISCONTINUOUS"
        self.session = tr.session_id
        self.epoch = tr.execution_epoch
        self.latest_id = tr.plan_id
        self.pending = tr
        return "ACCEPTED"

    def advance(self, now_ns: int) -> None:
        if self.pending and now_ns >= self.pending.start_ns:
            self.active = self.pending
            self.pending = None

    def sample(self, now_ns: int) -> ReferenceSample:
        if self.pending and now_ns >= self.pending.start_ns:
            return sample_execution(self.pending, now_ns)
        return sample_execution(self.active, now_ns) if self.active else ReferenceSample()

    def expired(self, now_ns: int) -> bool:
        return bool(self.active and not self.pending and
                    now_ns > self.active.start_ns + int(round(self.active.points[-1].time * 1.0e9)))

    def reset(self) -> None:
        self.__init__()
