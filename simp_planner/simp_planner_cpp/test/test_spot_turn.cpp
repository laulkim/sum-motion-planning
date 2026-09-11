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
  // A smoothly curving continuation (constant curvature) appended after
  // whatever is already in the array, so per-point curvature stays far
  // below any corner threshold even though the total heading change can be
  // large. Unlike leg(), this does not push its own starting point again --
  // it continues directly from x.back()/y.back(), so it never creates the
  // zero-distance duplicate that marks an in-place corner.
  void arc(double curvature, double length) {
    const int steps = std::max(2, static_cast<int>(std::round(length / 0.2)));
    const double ds = length / steps;
    double heading = yaw.back();
    double cx = x.back(), cy = y.back();
    for (int i = 1; i <= steps; ++i) {
      heading += curvature * ds;
      cx += ds * std::cos(heading);
      cy += ds * std::sin(heading);
      x.push_back(cx);
      y.push_back(cy);
      yaw.push_back(heading);
    }
  }
};

void test_build_reference_path_drops_padding_and_matches_curvature() {
  Input input;
  input.leg(0, 0, 0, 8);
  input.pad();
  auto path = build_reference_path(input.x, input.y, input.yaw);
  require(path->size() == input.x.size() - 1, "padding point was not dropped");
  require(std::abs(path->x().back() - 8.0) < 1.0e-9, "kept the padding point's own coordinate");
  require(path->kappa().back() == 0.0, "straight leg reports nonzero curvature");

  EnvConfig config;
  Costmap2D map(std::vector<std::int8_t>(300 * 300, 0), 300, 300, 0.2, -20, -20);
  PathVelocityPlanner planner(config, *path, map);
  auto result = planner.plan({}, {}, {1.5, DriveMode::Forward});
  require(result.selected_path && result.trajectory.safe(), "leg has no driving plan");
  require(result.trajectory.states.back().speed > 1.0, "leg cannot accelerate");
}

void test_heading_jump_between_legs_is_detected_per_mode() {
  Input before;
  before.leg(0, 0, 0, 8);
  before.pad();
  Input after;
  after.leg(8, 0, 40 * kPi / 180, 8);
  after.pad();
  auto before_path = build_reference_path(before.x, before.y, before.yaw);
  auto after_path = build_reference_path(after.x, after.y, after.yaw);
  for (auto mode : {DriveMode::Forward, DriveMode::Reverse, DriveMode::Left, DriveMode::Right}) {
    const double target = spot_turn_target_body_yaw(after_path->psi().front(), mode);
    require(std::abs(wrap_angle(target + drive_mode_heading_offset(mode) - 40 * kPi / 180)) < 1.0e-9,
            "target body yaw does not account for mode");
    const double current_body_yaw =
        spot_turn_target_body_yaw(before_path->psi().back(), mode);
    require(std::abs(wrap_angle(target - current_body_yaw)) > 0.349066,
            "40 degree leg boundary was not flagged as a spot turn");
  }
  // A small heading step (a gentle curve continuing into the next leg)
  // must not be mistaken for a corner.
  Input gentle;
  gentle.leg(8, 0, 5 * kPi / 180, 8);
  gentle.pad();
  auto gentle_path = build_reference_path(gentle.x, gentle.y, gentle.yaw);
  const double gentle_target = spot_turn_target_body_yaw(gentle_path->psi().front(), DriveMode::Forward);
  const double straight_body_yaw =
      spot_turn_target_body_yaw(before_path->psi().back(), DriveMode::Forward);
  require(std::abs(wrap_angle(gentle_target - straight_body_yaw)) <= 0.349066,
          "a 5 degree step was incorrectly flagged as a spot turn");
}

void test_split_reference_path_at_interior_corner() {
  EnvConfig config;
  Input combined;
  combined.leg(0, 0, 0, 36);
  // In-place corner: the next leg continues from the exact same point on a
  // new heading, same as a real authored course encodes a sharp turn.
  combined.leg(combined.x.back(), combined.y.back(), 40 * kPi / 180, 36);
  combined.pad();
  auto split = split_reference_path_at_corner(combined.x, combined.y, combined.yaw,
                                               config.constraints.curvature_max);
  require(static_cast<bool>(split.before) && static_cast<bool>(split.after),
          "interior corner was not detected");
  require(std::abs(split.before->x().back() - 36.0) < 1.0e-6,
          "before segment does not end at the corner");
  require(split.before->kappa().back() == 0.0, "corner spike leaked into the before segment");
  require(std::abs(split.before->psi().back()) < 1.0e-9, "before segment kept the wrong heading");
  require(std::abs(split.after->x().front() - 36.0) < 1.0e-6,
          "after segment does not start at the corner");
  require(std::abs(wrap_angle(split.after->psi().front() - 40 * kPi / 180)) < 1.0e-9,
          "after segment did not pick up the new heading");

  Input straight;
  straight.leg(0, 0, 0, 8);
  straight.pad();
  auto no_corner = split_reference_path_at_corner(straight.x, straight.y, straight.yaw,
                                                   config.constraints.curvature_max);
  require(static_cast<bool>(no_corner.before) && !no_corner.after,
          "a straight leg was incorrectly split");

  // A smooth curve the vehicle can actually steer through (per-point
  // curvature well under curvature_max) must not be mistaken for a corner,
  // even though it adds up to a large total heading change.
  Input gentle;
  gentle.leg(0, 0, 0, 8);
  gentle.arc(0.05, 18.0);
  gentle.pad();
  auto gentle_split = split_reference_path_at_corner(gentle.x, gentle.y, gentle.yaw,
                                                      config.constraints.curvature_max);
  require(!gentle_split.after, "a smooth curve under the curvature limit was incorrectly flagged");
}

void test_invalid_reference_arrays_are_rejected() {
  bool rejected = false;
  try {
    build_reference_path({0, 1, 1, 1, 1, 1}, {0, 0, 0, 1, 2, 3},
                         {0, 0, kPi / 2, kPi / 2, kPi / 2, kPi / 2});
  } catch (const std::invalid_argument&) { rejected = true; }
  require(rejected, "duplicate point without a heading change was accepted");

  Input straight;
  straight.leg(0, 0, 0, 8);
  straight.pad();
  require(static_cast<bool>(build_reference_path(straight.x, straight.y, straight.yaw)),
          "a valid reference array was rejected");
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
    test_build_reference_path_drops_padding_and_matches_curvature();
    test_heading_jump_between_legs_is_detected_per_mode();
    test_split_reference_path_at_interior_corner();
    test_invalid_reference_arrays_are_rejected();
    test_rotation_uses_feedback_and_accepts_new_return_mode();
    test_clearance_gate();
    std::cout << "all spot-turn integration tests passed\n";
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
