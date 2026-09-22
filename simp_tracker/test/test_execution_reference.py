"""Mirrors simp_planner_cpp/test/test_execution.cpp so the Python port of
execution.hpp/execution.cpp can be checked against the same expectations."""
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from simp_tracker.execution_reference import (
    BodyCommand, ExecutionBuffer, ExecutionTrajectory, ReferenceSample,
    make_stop_samples, make_turn_samples, sample_execution, wrap_angle,
)


def trajectory(plan_id, base, epoch, start, initial=None):
    tr = ExecutionTrajectory(session_id="test", plan_id=plan_id, base_plan_id=base,
                             execution_epoch=epoch, start_ns=start)
    tr.points = make_stop_samples(initial or ReferenceSample(), 4, 0.01, 1, 0.8)
    return tr


def require(condition, message):
    if not condition:
        raise AssertionError(message)


def main():
    b = ExecutionBuffer()
    initial = ReferenceSample()
    initial.command.vx = initial.command.planned_speed = 5
    a = trajectory(1, 0, 1, 1_000_000_000, initial)
    require(b.receive(a, 0) == "ACCEPTED", "initial receive")
    require(not b.active, "received trajectory activated early")
    b.advance(1_000_000_000)
    p = b.sample(1_090_000_000)
    require(p.command.vx > 4.9, "90ms without a message interrupted execution")
    require(b.receive(a, 1_100_000_000) == "DUPLICATE", "duplicate reset execution")
    nxt = trajectory(2, 1, 1, 1_500_000_000, b.sample(1_500_000_000))
    require(b.receive(nxt, 1_200_000_000) == "ACCEPTED", "normal handover rejected")
    stop = trajectory(3, 1, 2, 1_400_000_000, sample_execution(a, 1_400_000_000))
    require(b.receive(stop, 1_250_000_000) == "ACCEPTED", "stop supersession rejected")
    require(b.receive(nxt, 1_300_000_000) == "OLD_EPOCH", "cancelled plan revived")
    b.advance(1_400_000_000)
    require(b.active.plan_id == 3, "stop not activated")
    late = trajectory(4, 3, 2, 1_400_000_000)
    require(b.receive(late, 1_500_000_000) == "LATE_PLAN", "late plan accepted")
    bad = trajectory(4, 3, 2, 2_000_000_000)
    bad.points[0].x = 99
    require(b.receive(bad, 1_600_000_000) == "DISCONTINUOUS", "discontinuous handover accepted")
    require(a.points[-1].command.planned_speed > 0, "4s stop incorrectly forced zero")
    tail = sample_execution(a, 11_000_000_000)
    require(tail.command.planned_speed == 0, "expired trajectory did not finish braking")
    require(tail.x > a.points[-1].x, "braking extension position frozen")
    stopped = sample_execution(a, 12_000_000_000)
    require(abs(tail.x - stopped.x) < 1.0e-8, "stationary tail moved")
    nan_tr = trajectory(4, 3, 2, 2_000_000_000)
    nan_tr.points[0].body_yaw = math.nan
    require(b.receive(nan_tr, 1_600_000_000) != "ACCEPTED", "NaN accepted")
    turn = ReferenceSample(x=2, y=3)
    turns = make_turn_samples(turn, 0.6, 4, 0.01, 0.3, 0.3, 0.02)
    require(abs(wrap_angle(turns[-1].body_yaw - 0.6)) < 0.03, "turn did not reach heading")
    require(turns[-1].x == 2 and turns[-1].y == 3, "spot turn translated")
    crab = ReferenceSample()
    crab.command.planned_speed = 2
    crab.command.beta = 0.4
    crab.command.beta_rate = 0.2
    crab.command.motion_curvature = 0.1
    crab.command.yaw_rate = 0
    crab.command.vx = 2 * math.cos(0.4)
    crab.command.vy = 2 * math.sin(0.4)
    brake = make_stop_samples(crab, 4, 0.01, 1, 0.8)
    require(abs(brake[0].command.yaw_rate - crab.command.yaw_rate) < 1.0e-8,
            "beta-rate braking discontinuity")
    print("Execution reference tests passed")


if __name__ == "__main__":
    main()
