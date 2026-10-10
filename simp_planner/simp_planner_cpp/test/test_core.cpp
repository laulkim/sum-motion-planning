#include "simp_planner/core.hpp"
#include "simp_planner/runtime.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <vector>

namespace {

void require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}

simp_planner::ReferencePath straight_path(double length = 90.0, double ds = 0.1) {
  const int count = static_cast<int>(std::round(length / ds)) + 1;
  std::vector<double> s(count), x(count), y(count, 0.0), psi(count, 0.0), kappa(count, 0.0);
  for (int i = 0; i < count; ++i) {
    s[static_cast<std::size_t>(i)] = i * ds;
    x[static_cast<std::size_t>(i)] = i * ds;
  }
  return {std::move(s), std::move(x), std::move(y), std::move(psi), std::move(kappa)};
}

simp_planner::Costmap2D empty_costmap() {
  constexpr int width = 600;
  constexpr int height = 100;
  return {std::vector<std::int8_t>(width * height, 0), width, height,
          0.2, -10.0, -10.0};
}

simp_planner::Costmap2D terminal_center_obstacle_costmap() {
  constexpr double resolution = 0.10;
  constexpr double origin_x = -10.0;
  constexpr double origin_y = -10.0;
  constexpr int width = 1200;
  constexpr int height = 200;
  std::vector<std::int8_t> data(width * height, 0);
  for (int iy = 0; iy < height; ++iy) {
    const double y = origin_y + (static_cast<double>(iy) + 0.5) * resolution;
    for (int ix = 0; ix < width; ++ix) {
      const double x = origin_x + (static_cast<double>(ix) + 0.5) * resolution;
      if (x >= 87.9 && x <= 90.5 && std::abs(y) <= 1.25) {
        data[static_cast<std::size_t>(iy * width + ix)] = 100;
      }
    }
  }
  return {std::move(data), width, height, resolution, origin_x, origin_y};
}


simp_planner::Costmap2D blocking_wall_costmap(double wall_x = 18.0) {
  constexpr double resolution = 0.10;
  constexpr double origin_x = -10.0;
  constexpr double origin_y = -12.0;
  constexpr int width = 1200;
  constexpr int height = 240;
  std::vector<std::int8_t> data(width * height, 0);
  for (int iy = 0; iy < height; ++iy) {
    for (int ix = 0; ix < width; ++ix) {
      const double x = origin_x + (static_cast<double>(ix) + 0.5) * resolution;
      if (x >= wall_x && x <= wall_x + 0.6) {
        data[static_cast<std::size_t>(iy * width + ix)] = 100;
      }
    }
  }
  return {std::move(data), width, height, resolution, origin_x, origin_y};
}

void test_math_and_projection() {
  require(std::abs(simp_planner::wrap_angle(3.5) + 2.7831853071795862) < 1e-12,
          "wrap_angle mismatch");
  auto path = straight_path();
  const auto projection = path.project(4.25, 0.40, 0.05);
  require(std::abs(projection.s - 4.25) < 1e-12, "continuous projection s mismatch");
  require(std::abs(projection.n - 0.40) < 1e-12, "continuous projection n mismatch");
}

void test_rotated_costmap_registration() {
  constexpr int width = 40;
  constexpr int height = 30;
  constexpr double resolution = 0.5;
  constexpr double origin_x = 10.0;
  constexpr double origin_y = -4.0;
  constexpr double origin_yaw = 0.5 * simp_planner::kPi;
  constexpr int occupied_x = 12;
  constexpr int occupied_y = 8;
  std::vector<std::int8_t> data(width * height, 0);
  data[static_cast<std::size_t>(occupied_y * width + occupied_x)] = 100;
  simp_planner::Costmap2D costmap(
      std::move(data), width, height, resolution,
      origin_x, origin_y, origin_yaw);

  const double occupied_local_x =
      (static_cast<double>(occupied_x) + 0.5) * resolution;
  const double occupied_local_y =
      (static_cast<double>(occupied_y) + 0.5) * resolution;
  const double occupied_world_x = origin_x - occupied_local_y;
  const double occupied_world_y = origin_y + occupied_local_x;
  require(costmap.distance_at_world(occupied_world_x, occupied_world_y) < 1.0e-12,
          "rotated costmap did not register occupied cell in world frame");

  const double free_local_x = 4.25;
  const double free_local_y = 6.25;
  const double free_world_x = origin_x - free_local_y;
  const double free_world_y = origin_y + free_local_x;
  require(costmap.distance_at_world(free_world_x, free_world_y) > 0.5,
          "rotated costmap mapped a free world point to the obstacle");

  simp_planner::AllocationResult allocation;
  allocation.trajectory.x = {occupied_world_x};
  allocation.trajectory.y = {occupied_world_y};
  allocation.trajectory.speed = {0.0};
  allocation.psi = {origin_yaw};
  const auto collision = simp_planner::check_oriented_allocation_collision(
      allocation, costmap, {}, {});
  require(!collision.collision_free,
          "oriented collision gate ignored rotated costmap registration");
}

void test_nominal_planning_and_allocation() {
  simp_planner::EnvConfig config;
  require(config.lateral.n_targets.size() == 65, "lateral exploration count mismatch");
  require(std::abs(config.lateral.n_targets.front() + 8.0) < 1e-12, "lateral minimum mismatch");
  require(std::abs(config.lateral.n_targets.back() - 8.0) < 1e-12, "lateral maximum mismatch");
  for (std::size_t i = 1; i < config.lateral.n_targets.size(); ++i) {
    require(std::abs(config.lateral.n_targets[i] - config.lateral.n_targets[i - 1] - 0.25) < 1e-12,
            "lateral target spacing mismatch");
  }
  simp_planner::PathVelocityPlanner planner(config, straight_path(), empty_costmap());
  const simp_planner::PlannerState initial{};
  const simp_planner::PlannerAction previous{};
  const simp_planner::PlanningCommand command{2.0, simp_planner::DriveMode::Forward};
  const auto result = planner.plan(initial, previous, command);

  require(result.diagnostics.status == "FEASIBLE_OPEN_LOOP", "nominal planner status");
  require(result.trajectory.states.size() == 41, "state horizon mismatch");
  require(result.trajectory.actions.size() == 40, "action horizon mismatch");
  require(result.diagnostics.selected_candidate_id >= 0, "candidate selection failed");
  require(std::abs(result.diagnostics.selected_n_target) < 1e-12, "center target mismatch");
  require(result.trajectory.valid_dynamic, "nominal trajectory dynamic invalid");
  require(result.trajectory.collision_free, "nominal trajectory collision");

  double max_curvature = 0.0;
  for (double value : result.motion.kappa) max_curvature = std::max(max_curvature, std::abs(value));
  require(max_curvature <= config.constraints.curvature_max + 1e-12, "curvature limit exceeded");

  const auto allocation = simp_planner::allocate_trajectory(result.motion);
  require(allocation.fallback_used.size() == result.motion.t.size(), "allocator horizon mismatch");
  require(std::none_of(allocation.fallback_used.begin(), allocation.fallback_used.end(),
                       [](std::uint8_t value) { return value != 0; }),
          "allocator fallback occurred");
  const auto max_speed_error = *std::max_element(
      allocation.speed_reconstruction_error.begin(), allocation.speed_reconstruction_error.end());
  const auto max_heading_error = *std::max_element(
      allocation.heading_relation_error.begin(), allocation.heading_relation_error.end());
  require(max_speed_error <= 1e-12, "allocator speed identity mismatch");
  require(max_heading_error <= 1e-12, "allocator heading identity mismatch");
}

// Trajectory values come from the path function at the travelled distance, not
// from linear interpolation of the 0.5 m grid: across the old grid nodes the
// curvature rate must stay the derivative of curvature, the heading must turn
// at the curvature, and each position step must match the distance travelled.
void test_arc_length_path_values_are_consistent() {
  simp_planner::EnvConfig config;
  config.longitudinal.dt = 0.01;
  config.longitudinal.execution_dt = 0.01;
  config.longitudinal.horizon = 2.0;
  simp_planner::PathVelocityPlanner planner(config, straight_path(), empty_costmap());
  simp_planner::PlannerState state;
  state.y = 1.0;
  state.speed = 2.0;
  const auto result = planner.plan(state, {}, {2.0, simp_planner::DriveMode::Forward});
  require(result.selected_path.has_value() && result.selected_path->function,
          "arc-length test has no path function");
  require(result.trajectory.safe(), "arc-length test trajectory unsafe");
  const auto& motion = result.motion;
  const auto& progress = result.trajectory.progress;
  require(progress.back() > 4.0 * config.lateral.spatial_ds,
          "arc-length test does not cross several grid nodes");

  double max_kappa = 0.0;
  for (std::size_t i = 0; i + 1 < motion.t.size(); ++i) {
    max_kappa = std::max(max_kappa, std::abs(motion.kappa[i]));
    const double dl = progress[i + 1] - progress[i];
    if (dl < 1.0e-4) continue;
    const double kappa_slope = (motion.kappa[i + 1] - motion.kappa[i]) / dl;
    const double kappa_l_mean = 0.5 * (motion.kappa_s[i] + motion.kappa_s[i + 1]);
    require(std::abs(kappa_slope - kappa_l_mean) <= 1.0e-4 + 1.0e-3 * std::abs(kappa_l_mean),
            "curvature rate is not the derivative of curvature");
    const double heading_rate = (motion.chi[i + 1] - motion.chi[i]) / dl;
    const double kappa_mean = 0.5 * (motion.kappa[i] + motion.kappa[i + 1]);
    require(std::abs(heading_rate - kappa_mean) <= 1.0e-5,
            "heading does not turn at the path curvature");
    const double step = std::hypot(motion.x[i + 1] - motion.x[i], motion.y[i + 1] - motion.y[i]);
    require(std::abs(step - dl) <= 1.0e-6, "position step does not match travelled distance");
  }
  require(max_kappa > 1.0e-3, "arc-length test path has no curvature to check");
}

// At low speed the path's front curvature blends the measured curvature with
// the reference, so it differs from the handover's heading rate / speed. The
// allocator trajectory must still carry heading rate = v * path curvature at
// every state, including the first: the runtime command uses v * kappa, and a
// different first value turns the gap into a yaw-rate zigzag at every replan.
void test_first_heading_rate_follows_path_curvature() {
  simp_planner::EnvConfig config;
  config.longitudinal.dt = 0.01;
  config.longitudinal.execution_dt = 0.01;
  config.longitudinal.horizon = 2.0;
  simp_planner::PathVelocityPlanner planner(config, straight_path(), empty_costmap());
  simp_planner::PlannerState state;
  state.y = 1.0;
  state.speed = 0.3;
  state.motion_heading_rate = 0.03;
  const auto result = planner.plan(state, {}, {2.0, simp_planner::DriveMode::Forward});
  require(result.selected_path.has_value(), "heading-rate test has no selected path");
  const auto& motion = result.motion;
  require(std::abs(motion.kappa.front() - state.motion_heading_rate / state.speed) > 1.0e-3,
          "heading-rate test path starts at the measured curvature; nothing to check");
  for (std::size_t i = 0; i < motion.t.size(); ++i) {
    require(std::abs(motion.motion_heading_rate[i] - motion.speed[i] * motion.kappa[i]) <= 1.0e-9,
            "allocator heading rate is not speed times path curvature");
  }
}

void test_stationary_hold() {
  simp_planner::EnvConfig config;
  simp_planner::PathVelocityPlanner planner(config, straight_path(), empty_costmap());
  const auto result = planner.plan({}, {}, {0.0, simp_planner::DriveMode::Left});
  require(result.diagnostics.status == "STATIONARY_COMMAND_HOLD", "stationary status mismatch");
  require(std::all_of(result.motion.speed.begin(), result.motion.speed.end(),
                      [](double value) { return std::abs(value) <= 1e-15; }),
          "stationary hold has nonzero speed");
  require(std::all_of(result.motion.drive_mode.begin(), result.motion.drive_mode.end(),
                      [](simp_planner::DriveMode value) { return value == simp_planner::DriveMode::Left; }),
          "stationary mode bypass mismatch");
}


void test_runtime_execution_and_handover() {
  simp_planner::EnvConfig config;
  simp_planner::PathVelocityPlanner planner(config, straight_path(), empty_costmap());
  const auto plan = planner.plan({}, {}, {2.0, simp_planner::DriveMode::Forward});
  const auto allocation = simp_planner::allocate_trajectory(plan.motion);
  const auto command = simp_planner::sample_body_command(
      allocation, plan.trajectory.actions, 0.155, 0.01);
  require(command.planned_speed >= 0.0, "runtime sample negative speed");
  require(std::abs(command.vx - command.planned_speed * std::cos(command.beta)) < 1e-12,
          "runtime vx identity mismatch");
  require(std::abs(command.vy - command.planned_speed * std::sin(command.beta)) < 1e-12,
          "runtime vy identity mismatch");

  const auto predicted = simp_planner::predict_handover_state(
      {}, 0.0, 0, 150000000, std::int64_t{0}, &allocation,
      &plan.trajectory.actions, 0.01, 1.0, 0.8);
  require(predicted.expected_command.has_value(), "handover expected command missing");
  require(predicted.allocator_state.has_value(), "handover allocator state missing");
  require(std::abs(predicted.state.speed - predicted.expected_command->planned_speed) < 1e-12,
          "handover speed mismatch");
  require(simp_planner::align_time_ns(151000001, 0.01) == 160000000,
          "time alignment mismatch");
}

// Replanning the same motion from a handover point must continue the
// allocation unchanged. If the handover passes the rates of the interval that
// starts there, the new plan recomputes that interval and resets the yaw
// acceleration toward zero, which shows up as a yaw jerk spike.
void test_handover_replan_continues_allocation() {
  constexpr int count = 401;
  constexpr double dt = 0.01;
  constexpr double speed = 3.0;
  simp_planner::PlannerMotionTrajectory trajectory;
  std::vector<simp_planner::PlannerAction> actions;
  double chi = 0.0;
  for (int i = 0; i < count; ++i) {
    const double t = static_cast<double>(i) * dt;
    const double heading_rate = 0.4 * std::sin(2.0 * t);
    const double heading_acceleration = 0.8 * std::cos(2.0 * t);
    trajectory.t.push_back(t);
    trajectory.x.push_back(0.0);
    trajectory.y.push_back(0.0);
    trajectory.chi.push_back(chi);
    trajectory.kappa.push_back(heading_rate / speed);
    trajectory.kappa_s.push_back(0.0);
    trajectory.speed.push_back(speed);
    trajectory.acceleration.push_back(0.0);
    trajectory.longitudinal_jerk.push_back(0.0);
    trajectory.motion_heading_rate.push_back(heading_rate);
    trajectory.motion_heading_acceleration.push_back(heading_acceleration);
    trajectory.drive_mode.push_back(simp_planner::DriveMode::Forward);
    actions.push_back({0.0, heading_acceleration});
    chi += heading_rate * dt;
  }
  const auto original = simp_planner::allocate_trajectory(trajectory);

  // Hand over where the yaw acceleration is too large for the jerk limit to
  // reset in one step, so a wrong handover cannot pass by accident.
  std::size_t handover = 50;
  for (std::size_t i = 50; i < 250; ++i) {
    if (std::abs(original.yaw_acceleration[i]) > std::abs(original.yaw_acceleration[handover]))
      handover = i;
  }
  require(std::abs(original.yaw_acceleration[handover]) > 0.1,
          "handover test motion has too little yaw acceleration");

  const auto point_at = [&](std::size_t i) {
    simp_planner::TrackingPoint point;
    point.command = simp_planner::sample_body_command(original, actions, trajectory.t[i], dt);
    return point;
  };
  const auto handover_state =
      simp_planner::handover_from_point(point_at(handover), point_at(handover - 1));
  require(handover_state.allocator_state.has_value(), "handover allocator state missing");

  simp_planner::PlannerMotionTrajectory remaining;
  const auto tail = [handover](const auto& values) {
    return std::decay_t<decltype(values)>(values.begin() + static_cast<std::ptrdiff_t>(handover),
                                          values.end());
  };
  for (std::size_t i = handover; i < trajectory.t.size(); ++i)
    remaining.t.push_back(trajectory.t[i] - trajectory.t[handover]);
  remaining.x = tail(trajectory.x);
  remaining.y = tail(trajectory.y);
  remaining.chi = tail(trajectory.chi);
  remaining.kappa = tail(trajectory.kappa);
  remaining.kappa_s = tail(trajectory.kappa_s);
  remaining.speed = tail(trajectory.speed);
  remaining.acceleration = tail(trajectory.acceleration);
  remaining.longitudinal_jerk = tail(trajectory.longitudinal_jerk);
  remaining.motion_heading_rate = tail(trajectory.motion_heading_rate);
  remaining.motion_heading_acceleration = tail(trajectory.motion_heading_acceleration);
  remaining.drive_mode = tail(trajectory.drive_mode);

  const auto replanned = simp_planner::allocate_trajectory(
      remaining, simp_planner::AllocationLimits{}, handover_state.allocator_state);
  for (std::size_t j = 0; j + 1 < remaining.t.size(); ++j) {
    require(std::abs(replanned.yaw_rate[j] - original.yaw_rate[handover + j]) < 1.0e-9,
            "replan from the handover point does not continue the yaw rate");
    require(std::abs(replanned.beta[j] - original.beta[handover + j]) < 1.0e-9,
            "replan from the handover point does not continue beta");
  }
}


void test_lateral_priority_allocation() {
  simp_planner::PlannerMotionTrajectory trajectory;
  constexpr int count = 41;
  constexpr double dt = 0.10;
  constexpr double speed = 3.0;
  constexpr double motion_heading_rate = 0.25;
  for (int i = 0; i < count; ++i) {
    const double t = static_cast<double>(i) * dt;
    trajectory.t.push_back(t);
    trajectory.x.push_back(0.0);
    trajectory.y.push_back(0.0);
    trajectory.chi.push_back(motion_heading_rate * t);
    trajectory.kappa.push_back(motion_heading_rate / speed);
    trajectory.kappa_s.push_back(0.0);
    trajectory.speed.push_back(speed);
    trajectory.acceleration.push_back(0.0);
    trajectory.longitudinal_jerk.push_back(0.0);
    trajectory.motion_heading_rate.push_back(motion_heading_rate);
    trajectory.motion_heading_acceleration.push_back(0.0);
    trajectory.drive_mode.push_back(simp_planner::DriveMode::Forward);
  }

  const auto allocation = simp_planner::allocate_trajectory(trajectory);
  double maximum_abs_vy = 0.0;
  double maximum_abs_yaw_rate = 0.0;
  double maximum_abs_beta = 0.0;
  for (std::size_t i = 0; i < allocation.vy.size(); ++i) {
    maximum_abs_vy = std::max(maximum_abs_vy, std::abs(allocation.vy[i]));
    maximum_abs_yaw_rate = std::max(maximum_abs_yaw_rate,
                                    std::abs(allocation.yaw_rate[i]));
    maximum_abs_beta = std::max(maximum_abs_beta, std::abs(allocation.beta[i]));
  }
  require(maximum_abs_vy >= 1.10, "lateral-priority vy utilization too low");
  require(maximum_abs_yaw_rate <= 0.22, "lateral-priority yaw rate too high");
  require(maximum_abs_beta >= 20.0 * simp_planner::kPi / 180.0,
          "lateral-priority beta utilization too low");
  require(std::none_of(allocation.fallback_used.begin(), allocation.fallback_used.end(),
                       [](std::uint8_t value) { return value != 0; }),
          "lateral-priority allocation fallback occurred");
}



void test_drive_mode_feedback_supervisor() {
  using simp_planner::DriveMode;
  require(std::abs(simp_planner::motion_heading_from_body_yaw(0.2, DriveMode::Forward) - 0.2) < 1e-12,
          "forward stationary motion heading mismatch");
  require(std::abs(simp_planner::wrap_angle(
              simp_planner::motion_heading_from_body_yaw(0.2, DriveMode::Reverse) - (0.2 + simp_planner::kPi))) < 1e-12,
          "reverse stationary motion heading mismatch");
  require(std::abs(simp_planner::motion_heading_from_body_yaw(0.0, DriveMode::Left) - 0.5 * simp_planner::kPi) < 1e-12,
          "left-crab stationary motion heading mismatch");
  require(std::abs(simp_planner::motion_heading_from_body_yaw(0.0, DriveMode::Right) + 0.5 * simp_planner::kPi) < 1e-12,
          "right-crab stationary motion heading mismatch");
  using simp_planner::DriveModeControlState;
  using simp_planner::VehicleModeStatus;
  simp_planner::DriveModeSupervisor supervisor;
  require(supervisor.state(0.0, 0.03) ==
              DriveModeControlState::WaitingForRequest,
          "mode supervisor should wait for request");
  require(supervisor.set_requested_mode(DriveMode::Left),
          "initial mode request was not detected");
  require(supervisor.state(0.0, 0.03) ==
              DriveModeControlState::WaitingForFeedback,
          "mode supervisor should wait for vehicle feedback");
  require(!supervisor.update_vehicle_feedback(
              DriveMode::Forward, DriveMode::Forward,
              VehicleModeStatus::Ready),
          "unconfirmed mode must not increment generation");
  require(supervisor.state(0.5, 0.03) ==
              DriveModeControlState::StoppingForChange,
          "mode change must require standstill");
  require(!supervisor.should_publish_command(0.5, 0.03),
          "mode command must not be sent while moving");
  require(supervisor.should_publish_command(0.0, 0.03),
          "mode command should be sent at standstill");
  require(!supervisor.update_vehicle_feedback(
              DriveMode::Forward, DriveMode::Left,
              VehicleModeStatus::Aligning),
          "in-progress mode must not be confirmed");
  require(!supervisor.ready(), "in-progress mode reported ready");
  require(!supervisor.should_publish_command(0.0, 0.03),
          "matching in-progress command should not be resent");
  require(supervisor.update_vehicle_feedback(
              DriveMode::Left, DriveMode::Left, VehicleModeStatus::Ready),
          "completed vehicle mode was not confirmed");
  require(supervisor.ready(), "completed vehicle mode not ready");
  require(supervisor.current_mode() == DriveMode::Left,
          "confirmed vehicle mode mismatch");
  require(supervisor.confirmed_generation() == 1,
          "confirmed mode generation mismatch");
  require(!supervisor.update_vehicle_feedback(
              DriveMode::Left, DriveMode::Left, VehicleModeStatus::Ready),
          "repeated feedback must not create a new generation");
}


void test_terminal_monotonic_braking_with_positive_jerk() {
  simp_planner::EnvConfig config;
  simp_planner::PathVelocityPlanner planner(config, straight_path(), empty_costmap());

  simp_planner::PlannerState state;
  state.x = 88.0;
  state.y = 0.0;
  state.chi = 0.0;
  state.speed = 1.0;
  state.acceleration = 0.0;

  const auto result = planner.plan(
      state, {}, {1.0, simp_planner::DriveMode::Forward});
  require(result.trajectory.terminal_braking_active,
          "terminal braking was not activated");
  require(result.trajectory.terminal_stop_reached,
          "terminal trajectory did not reach stop");

  bool positive_jerk_observed = false;
  for (std::size_t i = 1; i < result.trajectory.states.size(); ++i) {
    require(result.trajectory.states[i].speed <=
                result.trajectory.states[i - 1].speed + 1.0e-10,
            "terminal speed increased during braking");
    require(result.trajectory.states[i].acceleration <= 1.0e-10,
            "terminal controller generated positive acceleration");
  }
  for (const auto& action : result.trajectory.actions) {
    if (action.longitudinal_jerk > 1.0e-6) {
      positive_jerk_observed = true;
    }
  }
  require(positive_jerk_observed,
          "positive jerk was not available to release braking acceleration");
}


void test_soft_input_revision_policy() {
  using simp_planner::PlanningRevisionState;
  const PlanningRevisionState planned{10, 3, 4, 5, 6};

  auto current = planned;
  current.request_revision += 100;
  require(simp_planner::plan_registration_is_current(planned, current),
          "odometry/request revision incorrectly invalidated plan");

  current = planned;
  current.path_revision += 1;
  require(simp_planner::plan_registration_is_current(planned, current),
          "rolling local-reference update incorrectly invalidated plan");
  require(simp_planner::planner_rebuild_required(planned, current),
          "new local reference did not request planner rebuild");

  current = planned;
  current.structural_revision += 1;
  require(simp_planner::plan_registration_is_current(planned, current),
          "rolling costmap update incorrectly invalidated plan handover");
  require(simp_planner::planner_rebuild_required(planned, current),
          "rolling costmap update did not request planner rebuild");

  current = planned;
  current.request_revision += 1000;
  current.structural_revision += 1000;
  require(simp_planner::plan_registration_is_current(planned, current),
          "costmap burst incorrectly invalidated plan handover");
  require(simp_planner::planner_rebuild_required(planned, current),
          "costmap burst did not preserve latest-map planner rebuild");

  current = planned;
  current.command_revision += 1;
  require(!simp_planner::plan_registration_is_current(planned, current),
          "target-speed or requested-mode change failed to invalidate plan");

  current = planned;
  current.mode_revision += 1;
  require(!simp_planner::plan_registration_is_current(planned, current),
          "confirmed vehicle-mode change failed to invalidate plan");
}

void test_costmap_refresh_collision_gate() {
  constexpr int width = 20;
  constexpr int height = 20;
  constexpr double resolution = 0.5;
  constexpr double origin = -5.0;
  std::vector<std::int8_t> free_data(width * height, 0);
  auto blocked_data = free_data;
  const int center_x = static_cast<int>(std::floor(-origin / resolution));
  const int center_y = static_cast<int>(std::floor(-origin / resolution));
  blocked_data[static_cast<std::size_t>(center_y * width + center_x)] = 100;

  const simp_planner::Costmap2D free_map(
      std::move(free_data), width, height, resolution, origin, origin);
  const simp_planner::Costmap2D blocked_map(
      std::move(blocked_data), width, height, resolution, origin, origin);
  simp_planner::AllocationResult pending;
  pending.trajectory.x = {0.0};
  pending.trajectory.y = {0.0};
  pending.trajectory.speed = {0.0};
  pending.psi = {0.0};
  const simp_planner::OrientedFootprintConfig footprint{
      3, 0.20, 2.0 * simp_planner::kPi / 180.0};

  require(simp_planner::check_oriented_allocation_collision(
              pending, free_map, {}, {}, footprint).collision_free,
          "safe costmap refresh incorrectly invalidated pending handover");
  require(!simp_planner::check_oriented_allocation_collision(
               pending, blocked_map, {}, {}, footprint).collision_free,
          "colliding costmap refresh failed to invalidate pending handover");
}



void test_oriented_footprint_directionality_and_profiles() {
  constexpr int width = 200;
  constexpr int height = 200;
  constexpr double resolution = 0.05;
  constexpr double origin = -5.0;
  std::vector<std::int8_t> data(width * height, 0);
  const double obstacle_x = 1.375;
  const double obstacle_y = 0.025;
  const int ix = static_cast<int>(std::floor((obstacle_x - origin) / resolution));
  const int iy = static_cast<int>(std::floor((obstacle_y - origin) / resolution));
  data[static_cast<std::size_t>(iy * width + ix)] = 100;
  simp_planner::Costmap2D costmap(
      std::move(data), width, height, resolution, origin, origin);

  simp_planner::AllocationResult longitudinal;
  longitudinal.trajectory.x = {0.0};
  longitudinal.trajectory.y = {0.0};
  longitudinal.trajectory.speed = {0.0};
  longitudinal.psi = {0.0};
  simp_planner::AllocationResult lateral = longitudinal;
  lateral.psi = {0.5 * simp_planner::kPi};
  const simp_planner::VehicleConfig vehicle{};
  simp_planner::CostConfig cost{};
  cost.hard_clearance_margin = 0.15;
  cost.hard_clearance_margin_low_speed = 0.15;
  const simp_planner::OrientedFootprintConfig footprint{3, 0.20, 2.0 * simp_planner::kPi / 180.0};
  const auto longitudinal_check = simp_planner::check_oriented_allocation_collision(
      longitudinal, costmap, vehicle, cost, footprint);
  const auto lateral_check = simp_planner::check_oriented_allocation_collision(
      lateral, costmap, vehicle, cost, footprint);
  require(!longitudinal_check.collision_free,
          "oriented footprint failed to detect longitudinal collision");
  require(lateral_check.collision_free,
          "oriented footprint incorrectly rejected rotated body");

  simp_planner::PlannerMotionTrajectory trajectory;
  constexpr int count = 41;
  constexpr double dt = 0.10;
  constexpr double speed = 3.0;
  constexpr double heading_rate = 0.25;
  for (int i = 0; i < count; ++i) {
    const double t = i * dt;
    trajectory.t.push_back(t); trajectory.x.push_back(0.0); trajectory.y.push_back(0.0);
    trajectory.chi.push_back(heading_rate * t);
    trajectory.kappa.push_back(heading_rate / speed); trajectory.kappa_s.push_back(0.0);
    trajectory.speed.push_back(speed); trajectory.acceleration.push_back(0.0);
    trajectory.longitudinal_jerk.push_back(0.0);
    trajectory.motion_heading_rate.push_back(heading_rate);
    trajectory.motion_heading_acceleration.push_back(0.0);
    trajectory.drive_mode.push_back(simp_planner::DriveMode::Forward);
  }
  const auto lateral_priority = simp_planner::allocate_trajectory(
      trajectory, simp_planner::allocation_limits_for_profile(
          simp_planner::AllocationProfile::LateralPriority));
  const auto minimum_vy = simp_planner::allocate_trajectory(
      trajectory, simp_planner::allocation_limits_for_profile(
          simp_planner::AllocationProfile::MinimumVy));
  double lateral_vy = 0.0;
  double minimum_vy_sum = 0.0;
  for (double value : lateral_priority.vy) lateral_vy += std::abs(value);
  for (double value : minimum_vy.vy) minimum_vy_sum += std::abs(value);
  require(minimum_vy_sum < lateral_vy,
          "minimum-vy profile did not reduce body-frame lateral velocity");
}



simp_planner::Costmap2D two_bottleneck_costmap() {
  constexpr int width = 600;
  constexpr int height = 100;
  constexpr double resolution = 0.20;
  constexpr double origin_x = -10.0;
  constexpr double origin_y = -10.0;
  std::vector<std::int8_t> data(width * height, 0);
  const auto add_point = [&](double x, double y) {
    const int ix = static_cast<int>(std::floor((x - origin_x) / resolution));
    const int iy = static_cast<int>(std::floor((y - origin_y) / resolution));
    data[static_cast<std::size_t>(iy * width + ix)] = 100;
  };
  add_point(15.0, 0.0);
  add_point(45.0, 0.0);
  return {std::move(data), width, height, resolution, origin_x, origin_y};
}

void test_bottleneck_limiter_and_low_speed_maneuver_latch() {
  simp_planner::EnvConfig config;
  // This fixture needs its original 32 m horizon to include both bottlenecks.
  config.adaptive_replan.minimum_spatial_preview = 32.0;
  simp_planner::PathVelocityPlanner planner(
      config, straight_path(), two_bottleneck_costmap());
  simp_planner::PlannerState state;
  state.speed = 0.20;

  const auto first = planner.plan(
      state, {}, {0.20, simp_planner::DriveMode::Forward});
  require(first.diagnostics.status == "FEASIBLE_OPEN_LOOP",
          "low-speed bottleneck plan was infeasible");
  require(first.diagnostics.bottleneck_limited,
          "first-bottleneck decision horizon was not activated");
  require(first.diagnostics.decision_horizon_length <
              config.adaptive_replan.minimum_spatial_preview - 1.0,
          "decision horizon still included the next bottleneck");
  require(std::abs(first.diagnostics.selected_n_target) >= 0.5,
          "bottleneck did not initiate a lateral maneuver");

  const auto continuity_before = planner.export_continuity_state(state);
  require(continuity_before.maneuver_remaining_length.has_value(),
          "active maneuver was not latched");
  const double remaining_before = *continuity_before.maneuver_remaining_length;

  require(first.selected_path.has_value(),
          "low-speed plan did not expose the selected spatial path");
  const auto& first_path = *first.selected_path;
  const auto sample_index = [&](const simp_planner::SpatialPathCandidate& path,
                                double q) {
    auto it = std::lower_bound(path.q_ref.begin(), path.q_ref.end(), q);
    return std::min(static_cast<std::size_t>(it - path.q_ref.begin()),
                    path.q_ref.size() - 1);
  };
  const auto executed_index = sample_index(first_path, 0.50);
  state.x = first_path.x[executed_index];
  state.y = first_path.y[executed_index];
  state.chi = first_path.psi[executed_index];
  state.motion_heading_rate = state.speed * first_path.kappa[executed_index];
  const auto second = planner.plan(
      state, {}, {0.20, simp_planner::DriveMode::Forward});
  const auto continuity_after = planner.export_continuity_state(state);
  require(second.diagnostics.selected_path_mode == "latched_transition",
          "low-speed replanning restarted the lateral profile");
  require(continuity_after.maneuver_remaining_length.has_value(),
          "latched maneuver was unexpectedly released");
  require(*continuity_after.maneuver_remaining_length < remaining_before - 0.015,
          "latched maneuver endpoint did not remain fixed in reference coordinates");
  require(second.selected_path.has_value(),
          "latched replan did not expose the continued path");
  const auto& second_path = *second.selected_path;
  const double global_q = first_path.q_ref[executed_index];
  require(std::abs(second_path.lateral_offset.front()
                   - first_path.lateral_offset[executed_index]) < 2.0e-3,
          "latched maneuver did not preserve the original polynomial phase");
  const auto first_ahead = sample_index(first_path, global_q + 1.0);
  const auto second_ahead = sample_index(second_path, 1.0);
  require(std::abs(second_path.lateral_offset[second_ahead]
                   - first_path.lateral_offset[first_ahead]) < 3.0e-2,
          "latched maneuver regenerated a new seventh-order start segment");

  simp_planner::PlannerState return_state;
  return_state.y = -3.0;
  return_state.speed = 0.20;
  simp_planner::PathVelocityPlanner return_planner(
      config, straight_path(), std::nullopt);
  const auto return_plan = return_planner.plan(
      return_state, {}, {0.20, simp_planner::DriveMode::Forward});
  require(std::abs(return_plan.diagnostics.selected_n_target) < 1.0e-9,
          "clear centerline did not request a return maneuver");
  const auto return_continuity = return_planner.export_continuity_state(return_state);
  require(return_continuity.maneuver_profile.has_value()
              && std::abs(return_continuity.maneuver_profile->target) < 1.0e-9,
          "return-to-center maneuver was not latched");
}

void test_minimum_vy_collision_retry() {
  simp_planner::PlannerMotionTrajectory trajectory;
  constexpr int count = 41;
  constexpr double dt = 0.10;
  constexpr double speed = 3.0;
  constexpr double heading_rate = 0.25;
  const double radius = speed / heading_rate;
  for (int i = 0; i < count; ++i) {
    const double t = static_cast<double>(i) * dt;
    const double chi = heading_rate * t;
    trajectory.t.push_back(t);
    trajectory.x.push_back(radius * std::sin(chi));
    trajectory.y.push_back(radius * (1.0 - std::cos(chi)));
    trajectory.chi.push_back(chi);
    trajectory.kappa.push_back(heading_rate / speed);
    trajectory.kappa_s.push_back(0.0);
    trajectory.speed.push_back(speed);
    trajectory.acceleration.push_back(0.0);
    trajectory.longitudinal_jerk.push_back(0.0);
    trajectory.motion_heading_rate.push_back(heading_rate);
    trajectory.motion_heading_acceleration.push_back(0.0);
    trajectory.drive_mode.push_back(simp_planner::DriveMode::Forward);
  }

  constexpr int width = 700;
  constexpr int height = 500;
  constexpr double resolution = 0.025;
  constexpr double origin_x = -2.0;
  constexpr double origin_y = -2.0;
  std::vector<std::int8_t> data(width * height, 0);
  const double obstacle_x = 1.40;
  const double obstacle_y = 1.45;
  const int ix = static_cast<int>(std::floor(
      (obstacle_x - origin_x) / resolution));
  const int iy = static_cast<int>(std::floor(
      (obstacle_y - origin_y) / resolution));
  data[static_cast<std::size_t>(iy * width + ix)] = 100;
  simp_planner::Costmap2D costmap(
      std::move(data), width, height, resolution, origin_x, origin_y);

  const simp_planner::VehicleConfig vehicle{};
  simp_planner::CostConfig cost{};
  cost.hard_clearance_margin = 0.15;
  cost.hard_clearance_margin_low_speed = 0.15;
  const auto nominal = simp_planner::allocate_trajectory(
      trajectory, simp_planner::allocation_limits_for_profile(
          simp_planner::AllocationProfile::LateralPriority));
  const auto minimum_vy = simp_planner::allocate_trajectory(
      trajectory, simp_planner::allocation_limits_for_profile(
          simp_planner::AllocationProfile::MinimumVy));
  const auto nominal_collision =
      simp_planner::check_oriented_allocation_collision(
          nominal, costmap, vehicle, cost);
  const auto minimum_vy_collision =
      simp_planner::check_oriented_allocation_collision(
          minimum_vy, costmap, vehicle, cost);
  require(!nominal_collision.collision_free,
          "targeted nominal allocation did not collide");
  require(minimum_vy_collision.collision_free,
          "targeted minimum-vy allocation was not collision-free");

  const auto selection =
      simp_planner::allocate_with_oriented_collision_search(
          trajectory, costmap, vehicle, cost);
  require(selection.candidates_evaluated == 2,
          "minimum-vy retry was not evaluated exactly once");
  require(selection.profile == simp_planner::AllocationProfile::MinimumVy,
          "minimum-vy retry was not selected");
  require(selection.collision.collision_free,
          "minimum-vy retry did not recover a safe allocation");
}

void test_one_shot_candidate_exclusion() {
  simp_planner::EnvConfig config;
  simp_planner::PathVelocityPlanner planner(
      config, straight_path(), two_bottleneck_costmap());
  simp_planner::PlannerState state;
  state.speed = 0.20;
  const auto nominal = planner.plan(
      state, {}, {0.20, simp_planner::DriveMode::Forward});
  require(nominal.selected_path.has_value(), "nominal candidate missing");
  const int failed_candidate = nominal.selected_path->candidate_id;

  planner.set_one_shot_excluded_candidate(
      failed_candidate, nominal.selected_path->n_target);
  const auto replanned = planner.plan(
      state, {}, {0.20, simp_planner::DriveMode::Forward});
  require(replanned.selected_path.has_value(), "excluded-path replan failed");
  require(replanned.selected_path->candidate_id != failed_candidate,
          "one-shot exclusion repeated the failed path");
  require(std::abs(replanned.selected_path->n_target
                   - nominal.selected_path->n_target) > 1.0e-9,
          "one-shot exclusion repeated the failed lateral target");
  require(replanned.diagnostics.excluded_candidate_id == failed_candidate,
          "excluded candidate was not reported in diagnostics");

  const auto next = planner.plan(
      state, {}, {0.20, simp_planner::DriveMode::Forward});
  require(next.diagnostics.excluded_candidate_id < 0,
          "candidate exclusion was not one-shot");
}

void test_short_path_stop_fallback() {
  simp_planner::EnvConfig config;
  config.lateral.n_targets = {-1.0, -0.5, 0.0, 0.5, 1.0};
  config.lateral.short_path_max_length = 20.0;
  simp_planner::PathVelocityPlanner planner(
      config, straight_path(), blocking_wall_costmap());
  simp_planner::PlannerState state;
  state.speed = 1.0;
  const auto result = planner.plan(
      state, {}, {1.0, simp_planner::DriveMode::Forward});
  require(result.diagnostics.short_path_fallback_active,
          "short-path fallback was not activated");
  require(result.diagnostics.status.find("FEASIBLE_SHORT_PATH_STOP") != std::string::npos,
          "short-path fallback did not produce a controlled stop");
  require(result.selected_path.has_value(), "short-path candidate missing");
  require(result.selected_path->real_end_l < 18.0,
          "short path was not truncated before the wall");
  require(result.trajectory.valid_dynamic,
          "short-path stop violated dynamic constraints");
  require(result.trajectory.terminal_stop_valid,
          "short-path stop was not longitudinally feasible");
}

void test_terminal_safe_region_prefers_minimum_safe_offset() {
  simp_planner::EnvConfig config;
  config.terminal.minimum_activation_distance = 20.0;
  config.lateral.n_targets = {
      -4.0, -3.5, -3.0, -2.5, -2.0, -1.5, -1.0, -0.5,
       0.0,  0.5,  1.0,  1.5,  2.0,  2.5,  3.0,  3.5, 4.0};
  simp_planner::PathVelocityPlanner planner(
      config, straight_path(), terminal_center_obstacle_costmap());
  simp_planner::PlannerState state;
  state.x = 78.0;
  state.speed = 1.0;

  const auto result = planner.plan(
      state, {}, {1.0, simp_planner::DriveMode::Forward});
  require(result.trajectory.safe(),
          "terminal safe-region plan was not feasible");
  require(result.diagnostics.terminal_safe_region_active,
          "terminal safe-region mode was not activated");
  require(result.diagnostics.terminal_goal_region_safe,
          "selected terminal goal region was not collision-free");
  require(result.diagnostics.lateral_selection_basis ==
              "TERMINAL_SAFE_REGION_MIN_OFFSET",
          "terminal lateral selection policy mismatch");
  require(std::abs(std::abs(result.diagnostics.selected_n_target) - 2.5) < 1.0e-9,
          "planner did not choose the minimum safe terminal offset");
  require(result.trajectory.terminal_lateral_error
              > config.terminal.lateral_tolerance,
          "terminal plan was incorrectly forced back to the reference center");
  require(result.trajectory.predicted_stop_error
              <= config.terminal.longitudinal_tolerance + 1.0e-9,
          "terminal safe-region plan lost longitudinal stop accuracy");
}

void test_terminal_safe_region_keeps_center_when_clear() {
  simp_planner::EnvConfig config;
  config.terminal.minimum_activation_distance = 20.0;
  config.lateral.n_targets = {-1.0, -0.5, 0.0, 0.5, 1.0};
  simp_planner::PathVelocityPlanner planner(
      config, straight_path(), empty_costmap());
  simp_planner::PlannerState state;
  state.x = 78.0;
  state.speed = 1.0;

  const auto result = planner.plan(
      state, {}, {1.0, simp_planner::DriveMode::Forward});
  require(result.trajectory.safe(),
          "clear terminal center plan was not feasible");
  require(!result.diagnostics.terminal_safe_region_active,
          "terminal safe-region mode activated for a clear reference endpoint");
  require(std::abs(result.diagnostics.selected_n_target) < 1.0e-12,
          "clear terminal region did not preserve the reference center");
  require(result.diagnostics.terminal_goal_offset < 1.0e-12,
          "clear terminal region reported a nonzero goal offset");
}

void test_terminal_braking_fallback_recovers_from_early_stop() {
  simp_planner::EnvConfig config;
  const std::vector<double> accelerations{-0.10, 0.0, 0.05, 0.10};
  for (const double acceleration : accelerations) {
    simp_planner::PathVelocityPlanner planner(
        config, straight_path(77.0), empty_costmap());
    simp_planner::PlannerState state;
    state.x = 76.472522333;
    state.speed = 0.032538205;
    state.acceleration = acceleration;

    const auto result = planner.plan(
        state, {}, {0.20, simp_planner::DriveMode::Forward});
    require(result.trajectory.safe(),
            "quasi-stationary terminal recovery was rejected");
    require(result.diagnostics.status.rfind("FEASIBLE", 0) == 0,
            "terminal recovery did not produce a validated feasible plan");
    require(result.trajectory.terminal_stop_feasible,
            "terminal feedback could not recover an early braking stop");
    require(result.trajectory.predicted_stop_error
                <= config.terminal.longitudinal_tolerance + 1.0e-9,
            "terminal recovery exceeded the longitudinal tolerance");
  }
}

void test_terminal_virtual_extension_within_tolerance() {
  simp_planner::EnvConfig config;
  simp_planner::PathVelocityPlanner planner(
      config, straight_path(77.0), empty_costmap());
  simp_planner::PlannerState state;
  state.x = 76.99518;
  state.speed = 0.086816786;
  state.acceleration = -0.773340087;

  const auto result = planner.plan(
      state, {}, {1.50, simp_planner::DriveMode::Forward});
  require(result.trajectory.safe(),
          "terminal virtual extension inside tolerance was rejected");
  require(result.diagnostics.status.rfind("FEASIBLE", 0) == 0,
          "terminal virtual extension did not produce a feasible plan");
  require(result.trajectory.terminal_stop_feasible,
          "terminal virtual extension did not preserve stop feasibility");
  require(!result.trajectory.endpoint_overshoot_attempt,
          "terminal virtual extension was misclassified as overshoot");
  require(result.trajectory.predicted_stop_error
              <= config.terminal.longitudinal_tolerance + 1.0e-9,
          "terminal virtual extension exceeded longitudinal tolerance");
}

void test_scheduler_and_safety_tail() {
  simp_planner::LatestOnlyPlanningScheduler scheduler(10.0, 0.5, 0.005);
  scheduler.request(0, 1, 1, "INITIAL", true);
  auto first = scheduler.begin_if_due(0, false, false);
  require(first.has_value(), "urgent initial request not consumed");
  for (std::uint64_t revision = 2; revision <= 10; ++revision) {
    scheduler.request(
        static_cast<std::int64_t>(revision) * 1000000,
        revision, revision, "COSTMAP_CHANGED", true);
  }
  require(!scheduler.begin_if_due(50000000, true, false).has_value(),
          "frequency cap violated");
  const auto coalesced = scheduler.begin_if_due(100000000, true, false);
  require(coalesced.has_value(), "coalesced request not consumed");
  require(coalesced->input_revision == 10 &&
              coalesced->structural_revision == 10,
          "costmap burst did not coalesce to the latest revision");

  simp_planner::JerkLimitedSafetyStop safety(1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.8);
  for (int i = 0; i < 1000 && !safety.stopped(); ++i) safety.advance(0.01);
  require(safety.stopped(), "safety tail did not stop");

  simp_planner::AdaptiveHandoverTiming timing;
  const double lead = timing.record(0.04, false);
  require(lead >= 0.12 && lead <= 0.60, "adaptive lead out of bounds");
}

}  // namespace

int main() {
  try {
    test_math_and_projection();
    test_rotated_costmap_registration();
    test_nominal_planning_and_allocation();
    test_arc_length_path_values_are_consistent();
    test_first_heading_rate_follows_path_curvature();
    test_stationary_hold();
    test_runtime_execution_and_handover();
    test_handover_replan_continues_allocation();
    test_lateral_priority_allocation();
    test_drive_mode_feedback_supervisor();
    test_terminal_monotonic_braking_with_positive_jerk();
    test_soft_input_revision_policy();
    test_costmap_refresh_collision_gate();
    test_oriented_footprint_directionality_and_profiles();
    test_bottleneck_limiter_and_low_speed_maneuver_latch();
    test_minimum_vy_collision_retry();
    test_one_shot_candidate_exclusion();
    test_short_path_stop_fallback();
    test_terminal_safe_region_prefers_minimum_safe_offset();
    test_terminal_safe_region_keeps_center_when_clear();
    test_terminal_braking_fallback_recovers_from_early_stop();
    test_terminal_virtual_extension_within_tolerance();
    test_scheduler_and_safety_tail();
    std::cout << "all standalone C++ core tests passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "test failure: " << error.what() << '\n';
    return 1;
  }
}
