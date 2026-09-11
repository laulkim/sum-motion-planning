#include "simp_planner/runtime.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>

using namespace simp_planner;

namespace {
void require(bool ok, const char* message) {
  if (!ok) throw std::runtime_error(message);
}

struct Input {
  std::vector<double> x, y, yaw;
  void leg(double px, double py, double heading, double length) {
    const int steps = std::max(2, static_cast<int>(std::round(length / 0.2)));
    for (int i = 0; i <= steps; ++i) {
      x.push_back(px + length * i / steps * std::cos(heading));
      y.push_back(py + length * i / steps * std::sin(heading));
      yaw.push_back(heading);
    }
  }
  void pad() {
    x.push_back(x.back() + 0.2 * std::cos(yaw.back()));
    y.push_back(y.back() + 0.2 * std::sin(yaw.back()));
    yaw.push_back(yaw.back());
  }
};

void test_split_arrival_and_replanning() {
  Input input;
  input.leg(0, 0, 0, 8);
  input.leg(8, 0, 40 * kPi / 180, 8);
  input.pad();
  SpotTurnReferenceBuffer buffer;
  auto path = buffer.update(input.x, input.y, input.yaw, DriveMode::Forward, 1000);
  require(buffer.pending(), "missing turn");
  require(path->x().back() == 8 && path->kappa().back() == 0, "seam curvature leaked into head");
  require(!buffer.arrived(PlannerState{}, 0.2), "triggered at origin");
  PlannerState near_end{};
  near_end.x = 7.95;
  require(buffer.arrived(near_end, 0.2), "failed to capture corner");
  near_end.y = 3;
  require(!buffer.arrived(near_end, 0.2), "projection alone incorrectly captured remote vehicle");

  EnvConfig config;
  Costmap2D map(std::vector<std::int8_t>(300 * 300, 0), 300, 300, 0.2, -20, -20);
  PathVelocityPlanner planner(config, *path, map);
  auto result = planner.plan({}, {}, {1.5, DriveMode::Forward});
  require(result.selected_path && result.trajectory.safe(), "split approach has no driving plan");
  require(result.trajectory.states.back().speed > 1.0, "split approach cannot accelerate");
  auto tail = buffer.complete_turn();
  require(!buffer.pending(), "completed corner still pending");
  require(std::hypot(tail->x().front() - 8, tail->y().front()) < 1.0e-9, "turn introduces position gap");
  PlannerState after{};
  after.x = 8;
  after.chi = 40 * kPi / 180;
  PathVelocityPlanner next(config, *tail, map);
  auto resumed = next.plan(after, {}, {1.5, DriveMode::Forward});
  require(resumed.selected_path && resumed.trajectory.states.back().speed > 1.0,
          "cannot accelerate after restoring remainder");

  // Republishing a rolling window resets s, but still contains the old seam.
  input.x.erase(input.x.begin(), input.x.begin() + 20);
  input.y.erase(input.y.begin(), input.y.begin() + 20);
  input.yaw.erase(input.yaw.begin(), input.yaw.begin() + 20);
  auto rolling = buffer.update(input.x, input.y, input.yaw, DriveMode::Forward, 1000);
  require(!buffer.pending() && rolling->x().front() == 8, "completed seam replayed from rolling window");
  Input later;
  later.leg(20, 0, 0, 0.6);
  later.leg(20.6, 0, -kPi / 3, 8);
  later.pad();
  buffer.update(later.x, later.y, later.yaw, DriveMode::Forward, 1000);
  require(buffer.pending(), "future corner with smaller local s was skipped");
}

void test_multiple_turns_and_modes() {
  Input input;
  input.leg(0, 0, 0, 8);
  input.leg(8, 0, kPi / 2, 8);
  input.leg(8, 8, 0, 8);
  input.pad();
  for (auto mode : {DriveMode::Forward, DriveMode::Reverse, DriveMode::Left, DriveMode::Right}) {
    SpotTurnReferenceBuffer buffer;
    buffer.update(input.x, input.y, input.yaw, mode, 1000);
    require(std::abs(wrap_angle(buffer.target_body_yaw() + drive_mode_heading_offset(mode) - kPi / 2)) < 1.0e-9,
            "target body yaw does not account for mode");
    auto second = buffer.complete_turn();
    require(buffer.pending() && second->kappa().back() == 0, "second split failed");
    PlannerState old_corner{};
    old_corner.x = 8;
    old_corner.chi = kPi / 2;
    require(!buffer.arrived(old_corner, 0.2), "second rotation triggered at first corner");
    auto third = buffer.complete_turn();
    require(!buffer.pending() && third->y().front() == 8, "third leg not restored");
  }
}

void test_invalid_split_is_atomic() {
  SpotTurnReferenceBuffer buffer;
  Input straight;
  straight.leg(0, 0, 0, 8);
  straight.pad();
  buffer.update(straight.x, straight.y, straight.yaw, DriveMode::Forward, 1000);
  bool rejected = false;
  try {
    buffer.update({0, 1, 1, 1, 1, 1}, {0, 0, 0, 1, 2, 3},
                  {0, 0, kPi / 2, kPi / 2, kPi / 2, kPi / 2}, DriveMode::Forward, 1000);
  } catch (const std::invalid_argument&) { rejected = true; }
  require(rejected && !buffer.pending(), "invalid short head left pending state");
  require(static_cast<bool>(buffer.update(straight.x, straight.y, straight.yaw, DriveMode::Forward, 1000)),
          "valid update blocked after malformed input");
}

void test_rotation_uses_feedback_and_accepts_new_return_mode() {
  SpotTurnConfig config;
  SpotTurnManeuver maneuver(config);
  DriveModeSupervisor supervisor;
  supervisor.set_requested_mode(DriveMode::Forward);
  supervisor.update_vehicle_feedback(DriveMode::Forward, DriveMode::Forward, VehicleModeStatus::Ready);
  maneuver.trigger(40 * kPi / 180, DriveMode::Forward);
  supervisor.set_requested_mode(DriveMode::SpotTurn);
  supervisor.update_vehicle_feedback(DriveMode::SpotTurn, DriveMode::SpotTurn, VehicleModeStatus::Ready);
  maneuver.on_mode_ready(supervisor);
  double yaw = 0, measured_rate = 0, previous_rate = 0;
  for (int i = 0; i < 2500 && maneuver.state() == SpotTurnManeuverState::Rotating; ++i) {
    auto cmd = maneuver.sample(0.01, yaw, measured_rate, supervisor);
    require(cmd && cmd->vx == 0 && cmd->vy == 0, "rotation moved vehicle laterally");
    require(std::abs(cmd->yaw_rate - previous_rate) <= config.yaw_rate_accel_max * 0.01 + 1.0e-9,
            "yaw acceleration limit exceeded");
    previous_rate = cmd->yaw_rate;
    // Hold actual yaw for the first five seconds, then apply only 75% of
    // the command. An internally integrated profile would finish too soon.
    measured_rate = i < 500 ? 0.0 : 0.75 * cmd->yaw_rate;
    yaw = wrap_angle(yaw + measured_rate * 0.01);
    if (i == 499) require(maneuver.state() == SpotTurnManeuverState::Rotating, "completed without actual rotation");
  }
  require(maneuver.state() == SpotTurnManeuverState::AligningWheels, "rotation did not converge");
  require(std::abs(wrap_angle(40 * kPi / 180 - yaw)) <= config.yaw_tolerance_rad, "actual heading off target");
  require(maneuver.set_external_requested_mode(DriveMode::Left, supervisor), "return request not retargeted");
  supervisor.update_vehicle_feedback(DriveMode::Left, DriveMode::Left, VehicleModeStatus::Ready);
  require(maneuver.on_mode_ready(supervisor), "return alignment stuck after new request");
}

void test_clearance_gate() {
  auto data = std::vector<std::int8_t>(100 * 100, 0);
  data[50 * 100 + 55] = 100;
  Costmap2D blocked(data, 100, 100, 0.1, -5, -5);
  Costmap2D clear(std::vector<std::int8_t>(100 * 100, 0), 100, 100, 0.1, -5, -5);
  require(!spot_turn_feasible(blocked, {}, VehicleConfig{}, 0), "unsafe rotation accepted");
  require(spot_turn_feasible(clear, {}, VehicleConfig{}, 0), "cleared obstacle did not release gate");
}
}  // namespace

int main() {
  try {
    test_split_arrival_and_replanning();
    test_multiple_turns_and_modes();
    test_invalid_split_is_atomic();
    test_rotation_uses_feedback_and_accepts_new_return_mode();
    test_clearance_gate();
    std::cout << "all spot-turn integration tests passed\n";
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
