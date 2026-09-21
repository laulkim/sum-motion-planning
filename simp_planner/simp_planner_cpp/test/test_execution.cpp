#include "simp_planner/execution.hpp"
#include <iostream>
#include <cmath>
#include <stdexcept>
using namespace simp_planner;
void require(bool value, const char* message) { if (!value) throw std::runtime_error(message); }
std::shared_ptr<ExecutionTrajectory> trajectory(std::uint64_t id, std::uint64_t base,
    std::uint64_t epoch, std::int64_t start, ReferenceSample initial = {}) {
  auto tr = std::make_shared<ExecutionTrajectory>();
  tr->session_id = "test"; tr->plan_id = id; tr->base_plan_id = base;
  tr->execution_epoch = epoch; tr->start_ns = start;
  tr->points = make_stop_samples(initial, 4, .01, 1, .8);
  return tr;
}
int main() {
  ExecutionBuffer b;
  ReferenceSample initial; initial.command.vx = initial.command.planned_speed = 5;
  auto a = trajectory(1, 0, 1, 1000000000LL, initial);
  require(b.receive(a, 0) == "ACCEPTED", "initial receive");
  require(!b.active(), "received trajectory activated early");
  b.advance(1000000000LL);
  auto p = b.sample(1090000000LL);
  require(p.command.vx > 4.9, "90ms without a message interrupted execution");
  require(b.receive(a, 1100000000LL) == "DUPLICATE", "duplicate reset execution");
  auto next = trajectory(2, 1, 1, 1500000000LL, b.sample(1500000000LL));
  require(b.receive(next, 1200000000LL) == "ACCEPTED", "normal handover rejected");
  auto stop = trajectory(3, 1, 2, 1400000000LL, sample_execution(*a, 1400000000LL));
  require(b.receive(stop, 1250000000LL) == "ACCEPTED", "stop supersession rejected");
  require(b.receive(next, 1300000000LL) == "OLD_EPOCH", "cancelled plan revived");
  b.advance(1400000000LL);
  require(b.active()->plan_id == 3, "stop not activated");
  auto late = trajectory(4, 3, 2, 1400000000LL);
  require(b.receive(late, 1500000000LL) == "LATE_PLAN", "late plan accepted");
  auto bad = trajectory(4, 3, 2, 2000000000LL); bad->points.front().x = 99;
  require(b.receive(bad, 1600000000LL) == "DISCONTINUOUS", "discontinuous handover accepted");
  require(a->points.back().command.planned_speed > 0, "4s stop incorrectly forced zero");
  auto tail = sample_execution(*a, 11000000000LL);
  require(tail.command.planned_speed == 0, "expired trajectory did not finish braking");
  require(tail.x > a->points.back().x, "braking extension position frozen");
  auto stopped = sample_execution(*a, 12000000000LL);
  require(std::abs(tail.x - stopped.x) < 1.e-8, "stationary tail moved");
  auto nan = trajectory(4, 3, 2, 2000000000LL); nan->points.front().body_yaw = NAN;
  require(b.receive(nan, 1600000000LL) != "ACCEPTED", "NaN accepted");
  ReferenceSample turn; turn.x = 2; turn.y = 3;
  auto turns = make_turn_samples(turn, 0.6, 4, .01, {});
  require(std::abs(wrap_angle(turns.back().body_yaw - 0.6)) < .03, "turn did not reach heading");
  require(turns.back().x == 2 && turns.back().y == 3, "spot turn translated");
  ReferenceSample crab; crab.command.planned_speed = 2; crab.command.beta = .4;
  crab.command.beta_rate = .2; crab.command.motion_curvature = .1;
  crab.command.yaw_rate = 0; crab.command.vx = 2 * std::cos(.4); crab.command.vy = 2 * std::sin(.4);
  auto brake = make_stop_samples(crab, 4, .01, 1, .8);
  require(std::abs(brake.front().command.yaw_rate - crab.command.yaw_rate) < 1.e-8,
          "beta-rate braking discontinuity");
  std::cout << "Execution tests passed\n";
}
