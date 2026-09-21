"""Tracker 실행 여부와 무관하게 같은 ROS 시각의 목표/실제 위치를 비교한다."""

from collections import deque
import math

from simp_tracker.tracking_controller_node import reference_at, stamp_ns, validate_trajectory


class PositionComparison:
    def __init__(self):
        self.plans = {}
        self.commands = deque(maxlen=500)
        self.last_odom_ns = None

    def on_trajectory(self, message):
        validate_trajectory(message)
        self.plans[message.plan_id] = message
        # 수신 지연이 있는 odometry도 직전 계획과 비교할 수 있도록 보관한다.
        if len(self.plans) > 8:
            del self.plans[min(self.plans)]

    def on_command(self, message):
        self.commands.append(message)

    def reference_xy(self, odom):
        now_ns = stamp_ns(odom.header.stamp)
        if self.last_odom_ns is not None and now_ns < self.last_odom_ns:
            self.plans.clear()
            self.commands.clear()
        self.last_odom_ns = now_ns
        # 수신 시각 대신 odometry 측정 시각 이하의 최신 실행 명령을 선택한다.
        command = max(
            (cmd for cmd in self.commands if stamp_ns(cmd.header.stamp) <= now_ns),
            key=lambda cmd: stamp_ns(cmd.header.stamp), default=None,
        )
        missing = (math.nan, math.nan)
        if command is None or command.execution_state != "ACTIVE_PLAN":
            return missing
        if now_ns - stamp_ns(command.header.stamp) > 250_000_000:
            return missing
        plan = self.plans.get(command.plan_id)
        if plan is None or plan.header.frame_id != odom.header.frame_id:
            return missing
        elapsed = (now_ns - stamp_ns(plan.start_time)) * 1e-9
        reference = reference_at(plan, elapsed)
        if reference is None:
            return missing
        return reference.x, reference.y
