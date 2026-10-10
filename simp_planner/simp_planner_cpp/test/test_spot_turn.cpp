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

void test_rotation_follows_previous_trajectory_point_and_accepts_new_return_mode() {
  SpotTurnConfig config;
  SpotTurnManeuver maneuver(config);
  DriveModeSupervisor supervisor;
  supervisor.set_requested_mode(DriveMode::Forward);
  supervisor.update_vehicle_feedback(DriveMode::Forward, DriveMode::Forward, VehicleModeStatus::Ready);
  // 170 deg -> -150 deg: the short way is +40 deg across +-pi.
  const double start = 170 * kPi / 180;
  const double target = -150 * kPi / 180;
  maneuver.trigger(target, DriveMode::Forward);
  supervisor.set_requested_mode(DriveMode::SpotTurn);
  supervisor.update_vehicle_feedback(DriveMode::SpotTurn, DriveMode::SpotTurn, VehicleModeStatus::Ready);
  maneuver.on_mode_ready(supervisor);
  require(maneuver.state() == SpotTurnManeuverState::Rotating, "ready SpotTurn mode did not start rotating");
  const auto commands = maneuver.rotation_commands(start, 0.01);
  double yaw = start, previous_rate = 0;
  for (const auto& cmd : commands) {
    require(cmd.vx == 0 && cmd.vy == 0, "rotation moved vehicle laterally");
    require(std::abs(cmd.yaw_rate - previous_rate) <= config.yaw_rate_accel_max * 0.01 + 1.0e-9,
            "yaw acceleration limit exceeded");
    require(cmd.yaw_rate >= 0.0, "rotation reversed or took the long way around");
    // Every point must continue from the previous trajectory point, and its
    // end must be exactly where holding yaw_rate for dt puts the vehicle.
    require(std::abs(wrap_angle(cmd.segment_start_heading - yaw)) < 1.0e-12,
            "trajectory point did not start from the previous point");
    require(std::abs(wrap_angle(cmd.segment_end_heading - (yaw + cmd.yaw_rate * 0.01))) < 1.0e-12,
            "segment end heading does not match the held yaw rate");
    previous_rate = cmd.yaw_rate;
    yaw = cmd.segment_end_heading;
  }
  require(commands.size() < 1000, "40 degree rotation took implausibly long");
  require(previous_rate == 0.0, "trajectory did not come to rest");
  require(std::abs(wrap_angle(target - yaw)) <= config.yaw_tolerance_rad, "trajectory heading off target");
  maneuver.finish_rotation(supervisor);
  require(maneuver.state() == SpotTurnManeuverState::AligningWheels, "finished rotation did not return");
  require(supervisor.requested_mode() == DriveMode::Forward, "external mode not requested after rotation");
  require(maneuver.set_external_requested_mode(DriveMode::Left, supervisor), "return request not retargeted");
  supervisor.update_vehicle_feedback(DriveMode::Left, DriveMode::Left, VehicleModeStatus::Ready);
  require(maneuver.on_mode_ready(supervisor), "return alignment stuck after new request");
}

TrackingPoint at_rest_point(double x, double y, double yaw, DriveMode mode) {
  TrackingPoint point;
  point.x = x;
  point.y = y;
  point.body_yaw = yaw;
  point.mode = mode;
  return point;
}

void test_tracking_index_is_floor_and_holds_last_point() {
  TrackingTrajectoryBuilder builder(1'000'000'000, 10'000'000);
  builder.hold_prefix(at_rest_point(0, 0, 0, DriveMode::Forward), 1'050'000'000);
  auto trajectory = builder.take();
  require(trajectory.points.size() == 5, "hold prefix did not fill up to its end time");
  require(trajectory.index_at(999'000'000) == 0, "time before the stamp did not use the first point");
  require(trajectory.index_at(1'019'999'999) == 1, "index was not the floor");
  require(trajectory.index_at(1'020'000'000) == 2, "exact grid time used the previous point");
  require(trajectory.index_at(5'000'000'000) == 4, "time past the end did not hold the last point");
}

void test_stop_tail_integrates_pose_and_ends_at_rest() {
  BodyCommand moving{};
  moving.vx = moving.planned_speed = 3.0;
  moving.motion_heading = 0.0;
  TrackingTrajectoryBuilder builder(0, 10'000'000);
  builder.hold_prefix(at_rest_point(0, 0, 0, DriveMode::Forward), 10'000'000);
  builder.append_stop(JerkLimitedSafetyStop::from_command(moving, 1.0, 0.8),
                      DriveMode::Forward, ExecutionState::SafetyStop, 7);
  auto trajectory = builder.take();
  const auto& last = trajectory.points.back();
  require(last.command.vx == 0.0 && last.command.planned_speed == 0.0, "stop tail did not end at rest");
  require(last.segment == TrajectorySegment::Stop && last.plan_id == 7, "stop tail lost its labels");
  // v / A + A / J = 3 / 1 + 1 / 0.8 = 4.25 s, distance v * T / 2 = 6.375 m.
  const double duration = 0.01 * static_cast<double>(trajectory.points.size() - 2);
  require(std::abs(duration - 4.25) < 0.05, "stop tail duration does not match the jerk-limited profile");
  require(std::abs(last.x - 6.375) < 0.05, "stop tail pose was not integrated from its commands");
  for (std::size_t i = 1; i < trajectory.points.size(); ++i) {
    require(trajectory.points[i].x >= trajectory.points[i - 1].x, "stop tail moved backwards");
  }
}

void test_copy_prefix_repeats_previous_points_and_continues_pose() {
  BodyCommand cruise{};
  cruise.vx = cruise.planned_speed = 2.0;
  cruise.yaw_rate = 0.1;
  TrackingTrajectoryBuilder first(0, 10'000'000);
  first.hold_prefix(at_rest_point(1, 2, 0.3, DriveMode::Forward), 10'000'000);
  for (int i = 0; i < 100; ++i) {
    first.append(cruise, DriveMode::Forward, TrajectorySegment::Drive, ExecutionState::ActivePlan, 1);
  }
  const auto previous = first.take();

  // Next cycle: stamp 0.1 s later, new content from t0 = 0.3 s.
  TrackingTrajectoryBuilder next(100'000'000, 10'000'000);
  next.copy_prefix(previous, 300'000'000);
  next.append(cruise, DriveMode::Forward, TrajectorySegment::Drive, ExecutionState::ActivePlan, 2);
  const auto stitched = next.take();
  for (std::int64_t t = 100'000'000; t < 300'000'000; t += 10'000'000) {
    const auto& a = previous.at(t);
    const auto& b = stitched.at(t);
    require(a.x == b.x && a.y == b.y && a.body_yaw == b.body_yaw && a.plan_id == b.plan_id,
            "prefix before t0 differs from the trajectory already sent");
  }
  const auto& old_t0 = previous.at(300'000'000);
  const auto& new_t0 = stitched.at(300'000'000);
  require(std::abs(old_t0.x - new_t0.x) < 1.0e-12 && std::abs(old_t0.y - new_t0.y) < 1.0e-12 &&
          std::abs(old_t0.body_yaw - new_t0.body_yaw) < 1.0e-12,
          "new content does not start at the previous trajectory's pose at t0");
  require(new_t0.plan_id == 2, "handover point is not labelled with the new plan");
}

void test_hold_in_new_mode_uses_that_modes_slip_angle() {
  TrackingTrajectoryBuilder builder(0, 10'000'000);
  builder.hold_prefix(at_rest_point(0, 0, 0.5, DriveMode::Forward), 10'000'000);
  builder.append_hold(DriveMode::Left, ExecutionState::ModeWait, 3);
  const auto trajectory = builder.take();
  const auto& hold = trajectory.points.back();
  require(hold.mode == DriveMode::Left, "hold did not carry the requested mode");
  require(std::abs(hold.command.beta - 0.5 * kPi) < 1.0e-12, "hold kept the old mode's slip angle");
  const auto handover = handover_from_point(hold, hold);
  require(std::abs(wrap_angle(handover.state.chi - (0.5 + 0.5 * kPi))) < 1.0e-12,
          "handover motion direction does not follow the held mode");
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
    test_rotation_follows_previous_trajectory_point_and_accepts_new_return_mode();
    test_tracking_index_is_floor_and_holds_last_point();
    test_stop_tail_integrates_pose_and_ends_at_rest();
    test_copy_prefix_repeats_previous_points_and_continues_pose();
    test_hold_in_new_mode_uses_that_modes_slip_angle();
    test_clearance_gate();
    std::cout << "all spot-turn integration tests passed\n";
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
