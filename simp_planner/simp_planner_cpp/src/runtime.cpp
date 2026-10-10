#include "simp_planner/runtime.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace simp_planner {

bool plan_registration_is_current(const PlanningRevisionState& planned,
                                  const PlanningRevisionState& current) {
  // Rolling costmaps are a latest-only planning input.  A newer map must
  // rebuild the next planner cycle, but it must not make an already computed
  // plan impossible to hand over when maps arrive faster than the handover
  // lead.  Command and confirmed-mode changes still invalidate immediately.
  return planned.command_revision == current.command_revision &&
         planned.mode_revision == current.mode_revision;
}

bool planner_rebuild_required(const PlanningRevisionState& built,
                              const PlanningRevisionState& current) {
  return built.structural_revision != current.structural_revision ||
         built.path_revision != current.path_revision ||
         built.mode_revision != current.mode_revision;
}


bool DriveModeSupervisor::set_requested_mode(DriveMode mode) {
  const bool changed = !requested_mode_ || *requested_mode_ != mode;
  requested_mode_ = mode;
  return changed;
}

bool DriveModeSupervisor::update_vehicle_feedback(
    DriveMode current_mode, DriveMode vehicle_requested_mode,
    VehicleModeStatus vehicle_status) {
  current_mode_ = current_mode;
  vehicle_requested_mode_ = vehicle_requested_mode;
  vehicle_status_ = vehicle_status;
  if (!ready()) return false;
  const bool changed = !last_confirmed_mode_ || *last_confirmed_mode_ != current_mode;
  if (changed) {
    last_confirmed_mode_ = current_mode;
    ++confirmed_generation_;
  }
  return changed;
}

bool DriveModeSupervisor::ready() const {
  return requested_mode_ && current_mode_ &&
         *requested_mode_ == *current_mode_ &&
         vehicle_status_ == VehicleModeStatus::Ready;
}

bool DriveModeSupervisor::should_publish_command(
    double measured_speed, double stop_speed_threshold) const {
  if (!requested_mode_ || !current_mode_) return false;
  if (ready()) return false;
  if (measured_speed > stop_speed_threshold) return false;
  if (vehicle_status_ == VehicleModeStatus::Aligning && vehicle_requested_mode_ &&
      *vehicle_requested_mode_ == *requested_mode_) {
    return false;
  }
  return true;
}

DriveModeControlState DriveModeSupervisor::state(
    double measured_speed, double stop_speed_threshold) const {
  if (!requested_mode_) return DriveModeControlState::WaitingForRequest;
  if (!current_mode_) return DriveModeControlState::WaitingForFeedback;
  if (ready()) return DriveModeControlState::Ready;
  if (measured_speed > stop_speed_threshold) {
    return DriveModeControlState::StoppingForChange;
  }
  return DriveModeControlState::WaitingForCompletion;
}

const char* drive_mode_control_state_name(DriveModeControlState state) {
  switch (state) {
    case DriveModeControlState::WaitingForRequest: return "WAITING_FOR_MODE_REQUEST";
    case DriveModeControlState::WaitingForFeedback: return "WAITING_FOR_VEHICLE_MODE";
    case DriveModeControlState::Ready: return "MODE_READY";
    case DriveModeControlState::StoppingForChange: return "STOPPING_FOR_MODE_CHANGE";
    case DriveModeControlState::WaitingForCompletion: return "WAITING_FOR_MODE_COMPLETION";
  }
  return "UNKNOWN_MODE_STATE";
}
namespace {

double wrap_interpolate(double angle0, double angle1, double alpha) {
  return wrap_angle(angle0 + alpha * wrap_angle(angle1 - angle0));
}

std::optional<double> positive_velocity_root(double speed, double acceleration,
                                             double jerk, double dt) {
  std::vector<double> roots;
  if (std::abs(jerk) < 1.0e-12) {
    if (acceleration < -1.0e-12) roots.push_back(-speed / acceleration);
  } else {
    const double discriminant = acceleration * acceleration - 2.0 * jerk * speed;
    if (discriminant >= 0.0) {
      const double root = std::sqrt(discriminant);
      roots.push_back((-acceleration + root) / jerk);
      roots.push_back((-acceleration - root) / jerk);
    }
  }
  std::optional<double> result;
  for (double value : roots) {
    if (value >= 0.0 && value <= dt + 1.0e-12 && (!result || value < *result)) result = value;
  }
  return result;
}

void validate_allocation(const AllocationResult& allocation,
                         const std::vector<PlannerAction>& actions) {
  const auto n = allocation.trajectory.t.size();
  if (n < 2 || allocation.vx.size() != n || allocation.vy.size() != n ||
      allocation.yaw_rate.size() != n || actions.empty()) {
    throw std::invalid_argument("invalid allocated trajectory");
  }
}

}  // namespace

BodyCommand sample_body_command(const AllocationResult& allocation,
                                const std::vector<PlannerAction>& planned_actions,
                                double trajectory_time, double execution_dt) {
  validate_allocation(allocation, planned_actions);
  if (!std::isfinite(trajectory_time)) throw std::invalid_argument("non-finite trajectory time");
  const auto& tr = allocation.trajectory;
  const auto& t = tr.t;
  const double elapsed = std::clamp(trajectory_time, t.front(), t.back());
  std::size_t interval = 0;
  double tau = 0.0;
  if (elapsed >= t.back()) {
    interval = t.size() - 2;
    tau = t.back() - t[interval];
  } else {
    auto upper = std::upper_bound(t.begin(), t.end(), elapsed);
    interval = upper == t.begin() ? 0 : static_cast<std::size_t>(upper - t.begin() - 1);
    interval = std::min(interval, t.size() - 2);
    tau = elapsed - t[interval];
  }
  const double dt = t[interval + 1] - t[interval];
  const double alpha = dt <= 0.0 ? 0.0 : std::clamp(tau / dt, 0.0, 1.0);
  const std::size_t action_index = std::min(interval, planned_actions.size() - 1);
  double jerk = planned_actions[action_index].longitudinal_jerk;
  double heading_acceleration = planned_actions[action_index].motion_heading_acceleration;
  const auto stop_time = positive_velocity_root(tr.speed[interval], tr.acceleration[interval], jerk, dt);
  const bool interval_ends_stopped = tr.speed[interval + 1] <= 1.0e-12;
  const bool stopped = interval_ends_stopped &&
      ((tr.speed[interval] <= 1.0e-12 && tr.acceleration[interval] <= 0.0) ||
       (stop_time && tau >= *stop_time - 1.0e-12));
  double speed;
  double acceleration;
  if (stopped) {
    speed = 0.0;
    acceleration = 0.0;
    jerk = 0.0;
    heading_acceleration = 0.0;
  } else {
    speed = tr.speed[interval] + tr.acceleration[interval] * tau + 0.5 * jerk * tau * tau;
    acceleration = tr.acceleration[interval] + jerk * tau;
  }
  const double beta_rate = stopped ? 0.0 : allocation.beta_rate[interval];
  const double beta = allocation.beta[interval] + beta_rate * tau;
  const double kappa = (1.0 - alpha) * tr.kappa[interval] + alpha * tr.kappa[interval + 1];
  const double motion_heading = wrap_interpolate(tr.chi[interval], tr.chi[interval + 1], alpha);
  const double motion_heading_rate = speed * kappa;
  const double yaw_rate = motion_heading_rate - beta_rate;
  const double yaw_acceleration = stopped ? 0.0 :
      (1.0 - alpha) * allocation.yaw_acceleration[interval] +
      alpha * allocation.yaw_acceleration[interval + 1];
  const double vx = speed * std::cos(beta);
  const double vy = speed * std::sin(beta);
  const double start_x = (1.0 - alpha) * tr.x[interval] + alpha * tr.x[interval + 1];
  const double start_y = (1.0 - alpha) * tr.y[interval] + alpha * tr.y[interval + 1];
  const double next_elapsed = std::min(elapsed + execution_dt, t.back());
  const double next_alpha = dt <= 0.0 ? 0.0 :
      std::clamp((next_elapsed - t[interval]) / dt, 0.0, 1.0);
  const double end_x = (1.0 - next_alpha) * tr.x[interval] + next_alpha * tr.x[interval + 1];
  const double end_y = (1.0 - next_alpha) * tr.y[interval] + next_alpha * tr.y[interval + 1];
  const double end_heading = wrap_interpolate(tr.chi[interval], tr.chi[interval + 1], next_alpha);
  return {vx, vy, yaw_rate, speed, acceleration, jerk, heading_acceleration,
          motion_heading, kappa, motion_heading_rate, beta, beta_rate,
          yaw_acceleration, start_x, start_y, motion_heading, end_x, end_y,
          end_heading, interval, interval, action_index, elapsed};
}

std::int64_t align_time_ns(std::int64_t time_ns, double period_sec) {
  if (!(period_sec > 0.0) || !std::isfinite(period_sec))
    throw std::invalid_argument("period must be finite and positive");
  const auto period_ns = std::max<std::int64_t>(1, static_cast<std::int64_t>(std::llround(period_sec * 1.0e9)));
  return ((time_ns + period_ns - 1) / period_ns) * period_ns;
}


PredictedHandoverState predict_handover_state(
    const PlannerState& current_state, double current_body_yaw,
    std::int64_t current_state_time_ns, std::int64_t handover_time_ns,
    std::optional<std::int64_t> active_plan_start_ns,
    const AllocationResult* allocation,
    const std::vector<PlannerAction>* planned_actions,
    double integration_dt, double safety_deceleration_limit,
    double safety_jerk_limit) {
  if (!std::isfinite(current_body_yaw) || !(integration_dt > 0.0) ||
      !std::isfinite(integration_dt) || handover_time_ns < current_state_time_ns) {
    throw std::invalid_argument("invalid handover prediction input");
  }
  if (allocation == nullptr || planned_actions == nullptr || !active_plan_start_ns) {
    return {current_state, current_body_yaw, {}, std::nullopt, std::nullopt};
  }
  validate_allocation(*allocation, *planned_actions);
  const double start_elapsed = std::max(0.0, 1.0e-9 *
      static_cast<double>(current_state_time_ns - *active_plan_start_ns));
  const double end_elapsed = std::max(0.0, 1.0e-9 *
      static_cast<double>(handover_time_ns - *active_plan_start_ns));
  const double trajectory_end = allocation->trajectory.t.back();
  double x = current_state.x;
  double y = current_state.y;
  double body_yaw = current_body_yaw;
  auto integrate = [&](const BodyCommand& command, double dt) {
    const double yaw_mid = body_yaw + 0.5 * command.yaw_rate * dt;
    x += (std::cos(yaw_mid) * command.vx - std::sin(yaw_mid) * command.vy) * dt;
    y += (std::sin(yaw_mid) * command.vx + std::cos(yaw_mid) * command.vy) * dt;
    body_yaw = wrap_angle(body_yaw + command.yaw_rate * dt);
  };
  double active_elapsed = std::min(start_elapsed, trajectory_end);
  const double active_target = std::min(end_elapsed, trajectory_end);
  while (active_elapsed < active_target - 1.0e-12) {
    const double dt = std::min(integration_dt, active_target - active_elapsed);
    integrate(sample_body_command(*allocation, *planned_actions,
                                  active_elapsed + 0.5 * dt, integration_dt), dt);
    active_elapsed += dt;
  }
  BodyCommand expected;
  if (end_elapsed <= trajectory_end + 1.0e-12) {
    expected = sample_body_command(*allocation, *planned_actions, end_elapsed, integration_dt);
  } else {
    if (!(safety_deceleration_limit > 0.0) || !(safety_jerk_limit > 0.0))
      throw std::invalid_argument("invalid safety braking limits");
    const auto final_command = sample_body_command(
        *allocation, *planned_actions, trajectory_end, integration_dt);
    JerkLimitedSafetyStop safety = start_elapsed >= trajectory_end
        ? JerkLimitedSafetyStop(current_state.speed, current_state.acceleration,
              final_command.beta, current_state.chi, final_command.motion_curvature,
              safety_deceleration_limit, safety_jerk_limit, start_elapsed)
        : JerkLimitedSafetyStop::from_command(
              final_command, safety_deceleration_limit, safety_jerk_limit);
    double safety_elapsed = start_elapsed >= trajectory_end ? start_elapsed : trajectory_end;
    while (safety_elapsed < end_elapsed - 1.0e-12) {
      const double dt = std::min(integration_dt, end_elapsed - safety_elapsed);
      const auto command = safety.sample();
      integrate(command, dt);
      safety.advance(dt);
      safety_elapsed += dt;
    }
    expected = safety.sample();
  }
  PlannerState predicted;
  predicted.x = x;
  predicted.y = y;
  predicted.chi = wrap_angle(body_yaw + expected.beta);
  predicted.speed = expected.planned_speed;
  predicted.acceleration = expected.planned_acceleration;
  predicted.motion_heading_rate = expected.motion_heading_rate;
  PlannerAction last_action{expected.planned_jerk,
                            expected.planned_heading_acceleration};
  AllocatorInitialState allocator_state{expected.beta, expected.beta_rate,
                                         expected.yaw_rate,
                                         expected.yaw_acceleration};
  return {predicted, body_yaw, last_action, allocator_state, expected};
}

JerkLimitedSafetyStop::JerkLimitedSafetyStop(
    double speed, double acceleration, double beta, double motion_heading,
    double curvature, double deceleration_limit, double jerk_limit, double elapsed)
    : speed_(std::max(speed, 0.0)), acceleration_(acceleration), beta_(beta),
      motion_heading_(motion_heading), curvature_(curvature),
      deceleration_limit_(std::max(deceleration_limit, 1.0e-6)),
      jerk_limit_(std::max(jerk_limit, 1.0e-6)), elapsed_(elapsed) {}

JerkLimitedSafetyStop JerkLimitedSafetyStop::from_command(
    const BodyCommand& command, double deceleration_limit, double jerk_limit) {
  return {command.planned_speed, command.planned_acceleration, command.beta,
          command.motion_heading, command.motion_curvature,
          deceleration_limit, jerk_limit, command.trajectory_time};
}

bool JerkLimitedSafetyStop::stopped() const {
  return speed_ <= 1.0e-6 && std::abs(acceleration_) <= 1.0e-6;
}

double JerkLimitedSafetyStop::current_jerk(double dt) const {
  if (stopped()) return 0.0;
  const double braking_acceleration = std::min(acceleration_, 0.0);
  const double release_speed = braking_acceleration * braking_acceleration /
      (2.0 * jerk_limit_);
  const double release_margin = std::max(0.5 * std::abs(acceleration_) * dt, 1.0e-5);
  if (acceleration_ < -1.0e-9 && speed_ <= release_speed + release_margin) return jerk_limit_;
  if (acceleration_ > -deceleration_limit_ + 1.0e-9) return -jerk_limit_;
  return 0.0;
}

BodyCommand JerkLimitedSafetyStop::sample() const {
  const bool is_stopped = stopped();
  const double jerk = is_stopped ? 0.0 : current_jerk(0.01);
  const double speed = is_stopped ? 0.0 : std::max(speed_, 0.0);
  const double acceleration = is_stopped ? 0.0 : acceleration_;
  const double heading_rate = speed * curvature_;
  const double vx = speed * std::cos(beta_);
  const double vy = speed * std::sin(beta_);
  const double heading_accel = acceleration * curvature_;
  const double nan = std::numeric_limits<double>::quiet_NaN();
  return {vx, vy, heading_rate, speed, acceleration, jerk, heading_accel,
          motion_heading_, curvature_, heading_rate, beta_, 0.0, heading_accel,
          nan, nan, motion_heading_, nan, nan, motion_heading_, 0, 0, 0, elapsed_};
}

void JerkLimitedSafetyStop::advance(double dt) {
  if (!(dt > 0.0) || !std::isfinite(dt)) throw std::invalid_argument("invalid safety stop dt");
  if (stopped()) return;
  const double jerk = current_jerk(dt);
  double next_acceleration = acceleration_ + jerk * dt;
  next_acceleration = std::clamp(next_acceleration, -deceleration_limit_,
      acceleration_ > 0.0 ? acceleration_ : 0.0);
  const double next_speed = speed_ + acceleration_ * dt + 0.5 * jerk * dt * dt;
  if (next_speed <= 0.0) {
    speed_ = 0.0;
    acceleration_ = 0.0;
  } else {
    speed_ = next_speed;
    acceleration_ = next_acceleration;
  }
  elapsed_ += dt;
}

BodyCommand JerkLimitedSafetyStop::sample_and_advance(double dt) {
  auto result = sample();
  advance(dt);
  return result;
}

namespace {

std::vector<double> cumulative_arc_length(const std::vector<double>& x,
                                         const std::vector<double>& y) {
  std::vector<double> s(x.size(), 0.0);
  for (std::size_t i = 1; i < x.size(); ++i) {
    s[i] = s[i - 1] + std::hypot(x[i] - x[i - 1], y[i] - y[i - 1]);
  }
  return s;
}

// Same forward difference every reference path uses to report its own
// per-point curvature.
std::vector<double> estimate_curvature_from_yaw(const std::vector<double>& psi,
                                               const std::vector<double>& s) {
  std::vector<double> kappa(psi.size() - 1, 0.0);
  for (std::size_t i = 0; i < kappa.size(); ++i) {
    kappa[i] = (psi[i + 1] - psi[i]) / std::max(s[i + 1] - s[i], 1.0e-15);
  }
  return kappa;
}

}  // namespace

std::shared_ptr<ReferencePath> build_reference_path(
    const std::vector<double>& x, const std::vector<double>& y,
    const std::vector<double>& yaw) {
  if (x.size() < 4 || x.size() != y.size() || x.size() != yaw.size()) {
    throw std::invalid_argument("reference arrays require at least three points plus padding");
  }
  for (std::size_t i = 0; i < x.size(); ++i) {
    if (!std::isfinite(x[i]) || !std::isfinite(y[i]) || !std::isfinite(yaw[i])) {
      throw std::invalid_argument("reference path contains non-finite values");
    }
  }
  const auto s = cumulative_arc_length(x, y);
  const auto psi = unwrap_angles(yaw);
  const auto kappa = estimate_curvature_from_yaw(psi, s);
  const auto used = kappa.size();  // Drop the trailing curvature-sampling point.
  return std::make_shared<ReferencePath>(
      std::vector<double>(s.begin(), s.begin() + static_cast<std::ptrdiff_t>(used)),
      std::vector<double>(x.begin(), x.begin() + static_cast<std::ptrdiff_t>(used)),
      std::vector<double>(y.begin(), y.begin() + static_cast<std::ptrdiff_t>(used)),
      std::vector<double>(psi.begin(), psi.begin() + static_cast<std::ptrdiff_t>(used)), kappa);
}

ReferencePathSplit split_reference_path_at_corner(
    const std::vector<double>& x, const std::vector<double>& y,
    const std::vector<double>& yaw, double curvature_max) {
  if (x.size() < 4 || x.size() != y.size() || x.size() != yaw.size()) {
    throw std::invalid_argument("reference arrays require at least three points plus padding");
  }
  const auto s = cumulative_arc_length(x, y);
  const auto psi = unwrap_angles(yaw);
  const auto kappa = estimate_curvature_from_yaw(psi, s);
  // kappa.back() belongs to the publisher's own padding point (see
  // build_reference_path) and is never a real interior corner.
  for (std::size_t i = 0; i + 1 < kappa.size(); ++i) {
    if (std::abs(kappa[i]) <= curvature_max) continue;
    std::vector<double> before_x(x.begin(), x.begin() + static_cast<std::ptrdiff_t>(i) + 1);
    std::vector<double> before_y(y.begin(), y.begin() + static_cast<std::ptrdiff_t>(i) + 1);
    std::vector<double> before_yaw(yaw.begin(), yaw.begin() + static_cast<std::ptrdiff_t>(i) + 1);
    // build_reference_path always drops its own last point as a
    // curvature-sampling pad; duplicate the corner point itself so the
    // segment ending there gets a real (zero) curvature, not the spike.
    before_x.push_back(x[i]);
    before_y.push_back(y[i]);
    before_yaw.push_back(yaw[i]);
    std::vector<double> after_x(x.begin() + static_cast<std::ptrdiff_t>(i) + 1, x.end());
    std::vector<double> after_y(y.begin() + static_cast<std::ptrdiff_t>(i) + 1, y.end());
    std::vector<double> after_yaw(yaw.begin() + static_cast<std::ptrdiff_t>(i) + 1, yaw.end());
    return {build_reference_path(before_x, before_y, before_yaw),
            build_reference_path(after_x, after_y, after_yaw)};
  }
  return {build_reference_path(x, y, yaw), nullptr};
}

double spot_turn_target_body_yaw(double path_start_psi, DriveMode mode) {
  return wrap_angle(path_start_psi - drive_mode_heading_offset(mode));
}

bool spot_turn_feasible(const Costmap2D& costmap, const PlannerState& state,
                        const VehicleConfig& vehicle, double safety_margin) {
  const double radius = circumscribed_radius(vehicle.length, vehicle.width,
                                              vehicle.footprint_margin) + safety_margin;
  return costmap.clearance_single_circle(state.x, state.y, radius) > 0.0;
}

YawRotationProfile::YawRotationProfile(const SpotTurnConfig& config) : config_(config) {
  if (!std::isfinite(config.yaw_rate_max) || config.yaw_rate_max <= 0.0 ||
      !std::isfinite(config.yaw_rate_accel_max) || config.yaw_rate_accel_max <= 0.0 ||
      !std::isfinite(config.yaw_tolerance_rad) || config.yaw_tolerance_rad <= 0.0) {
    throw std::invalid_argument("spot turn rotation limits must be finite and positive");
  }
}

void YawRotationProfile::engage(double start_yaw, double target_yaw) {
  if (!std::isfinite(start_yaw) || !std::isfinite(target_yaw))
    throw std::invalid_argument("invalid rotation start or target yaw");
  target_yaw_ = wrap_angle(target_yaw);
  yaw_ = wrap_angle(start_yaw);
  rate_ = 0.0;
  done_ = false;
}

BodyCommand YawRotationProfile::sample(double dt) {
  if (!std::isfinite(dt) || dt <= 0.0) throw std::invalid_argument("invalid rotation dt");
  // Feedback is the previous trajectory point (yaw_, rate_), not odometry.
  const double start_yaw = yaw_;
  const double error = wrap_angle(target_yaw_ - start_yaw);
  const double old_rate = rate_;
  const double feasible_rate = std::sqrt(2.0 * config_.yaw_rate_accel_max * std::abs(error));
  const double desired = std::abs(error) <= config_.yaw_tolerance_rad ? 0.0 :
      std::copysign(std::min(config_.yaw_rate_max, feasible_rate), error);
  const double delta = config_.yaw_rate_accel_max * dt;
  rate_ += std::clamp(desired - rate_, -delta, delta);
  // The command is held for dt, matching how the vehicle executes yaw_rate.
  yaw_ = wrap_angle(start_yaw + rate_ * dt);
  done_ = std::abs(error) <= config_.yaw_tolerance_rad && std::abs(rate_) <= 1.0e-9;
  BodyCommand command{};
  command.yaw_rate = rate_;
  command.yaw_acceleration = (rate_ - old_rate) / dt;
  command.planned_heading_acceleration = command.yaw_acceleration;
  command.motion_heading = start_yaw;
  command.motion_heading_rate = rate_;
  command.segment_start_heading = start_yaw;
  command.segment_end_heading = yaw_;
  return command;
}

SpotTurnManeuver::SpotTurnManeuver(const SpotTurnConfig& config) : rotation_(config) {}

void SpotTurnManeuver::trigger(double target_body_yaw, DriveMode external_mode) {
  if (state_ != SpotTurnManeuverState::Inactive) throw std::logic_error("spot turn already active");
  target_yaw_ = target_body_yaw;
  external_mode_ = external_mode;
  returning_ = false;
  state_ = SpotTurnManeuverState::AligningWheels;
}

bool SpotTurnManeuver::on_mode_ready(const DriveModeSupervisor& supervisor) {
  if (!supervisor.ready() || state_ != SpotTurnManeuverState::AligningWheels) return false;
  if (!returning_ && supervisor.requested_mode() == DriveMode::SpotTurn) {
    state_ = SpotTurnManeuverState::Rotating;
  } else if (returning_ && supervisor.requested_mode() == external_mode_) {
    state_ = SpotTurnManeuverState::Inactive;
    return true;
  }
  return false;
}

std::vector<BodyCommand> SpotTurnManeuver::rotation_commands(double start_yaw, double dt) {
  if (state_ != SpotTurnManeuverState::Rotating) throw std::logic_error("spot turn is not rotating");
  if (!std::isfinite(dt) || dt <= 0.0) throw std::invalid_argument("invalid rotation dt");
  rotation_.engage(start_yaw, target_yaw_);
  // Guard only: a half turn at the default limits settles in about 12 s.
  const auto limit = static_cast<std::size_t>(std::ceil(600.0 / dt));
  std::vector<BodyCommand> commands;
  do {
    commands.push_back(rotation_.sample(dt));
  } while (!rotation_.done() && commands.size() < limit);
  if (!rotation_.done()) throw std::runtime_error("spot turn rotation did not settle");
  return commands;
}

void SpotTurnManeuver::finish_rotation(DriveModeSupervisor& supervisor) {
  if (state_ != SpotTurnManeuverState::Rotating) throw std::logic_error("spot turn is not rotating");
  supervisor.set_requested_mode(external_mode_);
  returning_ = true;
  state_ = SpotTurnManeuverState::AligningWheels;
}

bool SpotTurnManeuver::set_external_requested_mode(DriveMode mode, DriveModeSupervisor& supervisor) {
  external_mode_ = mode;
  // A request arriving during RETURN alignment must retarget the actual
  // supervisor as well, otherwise its ready edge would never finish us.
  return returning_ && supervisor.set_requested_mode(mode);
}

const char* SpotTurnManeuver::state_name() const {
  if (state_ == SpotTurnManeuverState::Inactive) return "INACTIVE";
  if (state_ == SpotTurnManeuverState::Rotating) return "SPOT_TURN_ROTATING";
  return returning_ ? "SPOT_TURN_ALIGNING_REGULAR" : "SPOT_TURN_ALIGNING_WHEELS";
}

const char* execution_state_name(ExecutionState state) {
  switch (state) {
    case ExecutionState::Idle: return "IDLE";
    case ExecutionState::ActivePlan: return "ACTIVE_PLAN";
    case ExecutionState::SafetyStop: return "SAFETY_STOP";
    case ExecutionState::TerminalHold: return "TERMINAL_HOLD";
    case ExecutionState::ModeStop: return "MODE_STOP";
    case ExecutionState::ModeWait: return "MODE_WAIT";
    case ExecutionState::SpotTurnRotating: return "SPOT_TURN_ROTATING";
    case ExecutionState::SpotTurnWaitingClearance: return "SPOT_TURN_WAITING_CLEARANCE";
  }
  return "UNKNOWN";
}

std::size_t TrackingTrajectory::index_at(std::int64_t time_ns) const {
  if (points.empty()) throw std::logic_error("empty tracking trajectory");
  if (time_ns <= stamp_ns) return 0;
  const auto index = static_cast<std::size_t>((time_ns - stamp_ns) / period_ns);
  return std::min(index, points.size() - 1);
}

const TrackingPoint& TrackingTrajectory::at(std::int64_t time_ns) const {
  return points[index_at(time_ns)];
}

BodyCommand hold_command(const BodyCommand& previous, DriveMode previous_mode,
                         DriveMode mode, double body_yaw) {
  BodyCommand hold{};
  hold.beta = mode == previous_mode || mode == DriveMode::SpotTurn
      ? previous.beta : drive_mode_heading_offset(mode);
  hold.motion_heading = wrap_angle(body_yaw + hold.beta);
  hold.segment_start_heading = hold.segment_end_heading = hold.motion_heading;
  hold.trajectory_time = previous.trajectory_time;
  return hold;
}

PredictedHandoverState handover_from_point(const TrackingPoint& point,
                                           const TrackingPoint& previous) {
  const auto& command = point.command;
  const auto& rates = previous.command;
  PlannerState state;
  state.x = point.x;
  state.y = point.y;
  state.chi = wrap_angle(point.body_yaw + command.beta);
  state.speed = command.planned_speed;
  state.acceleration = command.planned_acceleration;
  state.motion_heading_rate = command.motion_heading_rate;
  PlannerAction last_action{command.planned_jerk, command.planned_heading_acceleration};
  AllocatorInitialState allocator_state{command.beta, rates.beta_rate,
                                         rates.yaw_rate, rates.yaw_acceleration};
  return {state, point.body_yaw, last_action, allocator_state, command};
}

TrackingTrajectoryBuilder::TrackingTrajectoryBuilder(std::int64_t stamp_ns,
                                                     std::int64_t period_ns)
    : dt_(1.0e-9 * static_cast<double>(period_ns)) {
  if (period_ns <= 0) throw std::invalid_argument("tracking period must be positive");
  trajectory_.stamp_ns = stamp_ns;
  trajectory_.period_ns = period_ns;
}

std::int64_t TrackingTrajectoryBuilder::next_time_ns() const {
  return trajectory_.stamp_ns +
         static_cast<std::int64_t>(trajectory_.points.size()) * trajectory_.period_ns;
}

void TrackingTrajectoryBuilder::push(TrackingPoint point) {
  // Same midpoint-yaw body-velocity integration as the vehicle simulator.
  const auto& c = point.command;
  const double yaw_mid = point.body_yaw + 0.5 * c.yaw_rate * dt_;
  x_ = point.x + (std::cos(yaw_mid) * c.vx - std::sin(yaw_mid) * c.vy) * dt_;
  y_ = point.y + (std::sin(yaw_mid) * c.vx + std::cos(yaw_mid) * c.vy) * dt_;
  body_yaw_ = wrap_angle(point.body_yaw + c.yaw_rate * dt_);
  trajectory_.points.push_back(std::move(point));
}

void TrackingTrajectoryBuilder::copy_prefix(const TrackingTrajectory& previous,
                                            std::int64_t until_ns) {
  if (previous.period_ns != trajectory_.period_ns)
    throw std::invalid_argument("tracking period changed between trajectories");
  while (next_time_ns() < until_ns) push(previous.at(next_time_ns()));
}

void TrackingTrajectoryBuilder::hold_prefix(const TrackingPoint& point, std::int64_t until_ns) {
  TrackingPoint hold = point;
  hold.command = hold_command(point.command, point.mode, point.mode, point.body_yaw);
  hold.segment = TrajectorySegment::Hold;
  while (next_time_ns() < until_ns) push(hold);
}

void TrackingTrajectoryBuilder::append(const BodyCommand& command, DriveMode mode,
                                       TrajectorySegment segment, ExecutionState execution,
                                       std::uint64_t plan_id) {
  if (trajectory_.points.empty()) throw std::logic_error("tracking trajectory has no start pose");
  push({x_, y_, body_yaw_, command, mode, segment, execution, plan_id});
}

void TrackingTrajectoryBuilder::append_stop(JerkLimitedSafetyStop stop, DriveMode mode,
                                            ExecutionState execution, std::uint64_t plan_id) {
  while (!stop.stopped()) {
    append(stop.sample_and_advance(dt_), mode, TrajectorySegment::Stop, execution, plan_id);
  }
  append(stop.sample(), mode, TrajectorySegment::Stop, execution, plan_id);
}

void TrackingTrajectoryBuilder::append_hold(DriveMode mode, ExecutionState execution,
                                            std::uint64_t plan_id) {
  if (trajectory_.points.empty()) throw std::logic_error("tracking trajectory has no start pose");
  const auto& last = trajectory_.points.back();
  append(hold_command(last.command, last.mode, mode, body_yaw_), mode,
         TrajectorySegment::Hold, execution, plan_id);
}

LatestOnlyPlanningScheduler::LatestOnlyPlanningScheduler(
    double max_planning_frequency_hz, double maximum_plan_age_sec,
    double input_coalescing_sec)
    : minimum_start_interval_ns_(static_cast<std::int64_t>(std::llround(1.0e9 / max_planning_frequency_hz))),
      maximum_plan_age_ns_(static_cast<std::int64_t>(std::llround(1.0e9 * maximum_plan_age_sec))),
      input_coalescing_ns_(static_cast<std::int64_t>(std::llround(1.0e9 * input_coalescing_sec))) {
  if (!(max_planning_frequency_hz > 0.0) || !(maximum_plan_age_sec > 0.0) || input_coalescing_sec < 0.0)
    throw std::invalid_argument("invalid scheduler configuration");
}

void LatestOnlyPlanningScheduler::request(
    std::int64_t now_ns, std::uint64_t input_revision,
    std::uint64_t structural_revision, std::string reason, bool urgent) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!pending_) first_request_ns_ = now_ns;
  pending_ = true;
  urgent_ = urgent_ || urgent;
  latest_request_ns_ = now_ns;
  latest_input_revision_ = input_revision;
  latest_structural_revision_ = structural_revision;
  reasons_.insert(std::move(reason));
  ++request_id_;
}

void LatestOnlyPlanningScheduler::mark_plan_activated(std::int64_t now_ns) {
  std::lock_guard<std::mutex> lock(mutex_);
  last_activated_ns_ = now_ns;
}

bool LatestOnlyPlanningScheduler::request_pending() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return pending_;
}

std::optional<PlanningRequestToken> LatestOnlyPlanningScheduler::begin_if_due(
    std::int64_t now_ns, bool has_active_plan, bool worker_busy) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (worker_busy) return std::nullopt;
  const bool watchdog_due = !has_active_plan ||
      (last_activated_ns_ && now_ns - *last_activated_ns_ >= maximum_plan_age_ns_);
  const bool request_due = pending_ && (urgent_ || now_ns - first_request_ns_ >= input_coalescing_ns_);
  if (!request_due && !watchdog_due) return std::nullopt;
  if (last_started_ns_ && now_ns - *last_started_ns_ < minimum_start_interval_ns_) return std::nullopt;
  PlanningRequestToken token;
  token.request_id = request_id_;
  token.input_revision = latest_input_revision_;
  token.structural_revision = latest_structural_revision_;
  token.reasons.assign(reasons_.begin(), reasons_.end());
  if (token.reasons.empty()) token.reasons.push_back("PLAN_AGE_WATCHDOG");
  token.urgent = urgent_;
  token.first_request_ns = pending_ ? first_request_ns_ : now_ns;
  token.latest_request_ns = pending_ ? latest_request_ns_ : now_ns;
  token.started_ns = now_ns;
  token.watchdog_triggered = watchdog_due;
  pending_ = false;
  urgent_ = false;
  first_request_ns_ = latest_request_ns_ = 0;
  reasons_.clear();
  last_started_ns_ = now_ns;
  return token;
}

AdaptiveHandoverTiming::AdaptiveHandoverTiming(
    double minimum_lead_sec, double initial_lead_sec, double maximum_lead_sec,
    double margin_sec, double quantile, double scale, std::size_t history_size)
    : minimum_lead_sec_(minimum_lead_sec), maximum_lead_sec_(maximum_lead_sec),
      margin_sec_(margin_sec), quantile_(quantile), scale_(scale),
      history_size_(history_size), current_lead_sec_(initial_lead_sec) {
  if (!(minimum_lead_sec > 0.0) || maximum_lead_sec < minimum_lead_sec ||
      initial_lead_sec < minimum_lead_sec || initial_lead_sec > maximum_lead_sec ||
      margin_sec < 0.0 || !(quantile > 0.0 && quantile <= 1.0) || scale < 1.0 || history_size < 4)
    throw std::invalid_argument("invalid adaptive handover configuration");
}

double AdaptiveHandoverTiming::percentile(double quantile) const {
  if (samples_.empty()) return std::numeric_limits<double>::quiet_NaN();
  std::vector<double> values(samples_.begin(), samples_.end());
  std::sort(values.begin(), values.end());
  const double position = std::clamp(quantile, 0.0, 1.0) * (values.size() - 1);
  const auto lower = static_cast<std::size_t>(std::floor(position));
  const auto upper = static_cast<std::size_t>(std::ceil(position));
  const double alpha = position - lower;
  return (1.0 - alpha) * values[lower] + alpha * values[upper];
}

double AdaptiveHandoverTiming::record(double compute_time_sec, bool late) {
  if (compute_time_sec < 0.0 || !std::isfinite(compute_time_sec))
    throw std::invalid_argument("invalid compute time");
  samples_.push_back(compute_time_sec);
  if (samples_.size() > history_size_) samples_.pop_front();
  const double p = percentile(quantile_);
  const auto begin = samples_.end() - static_cast<std::ptrdiff_t>(std::min<std::size_t>(8, samples_.size()));
  const double recent_peak = *std::max_element(begin, samples_.end());
  double target = std::max({minimum_lead_sec_, p * scale_ + margin_sec_, recent_peak + margin_sec_});
  if (late) target = std::max(target, compute_time_sec * 1.35 + margin_sec_);
  target = std::min(target, maximum_lead_sec_);
  current_lead_sec_ = target >= current_lead_sec_ ? target : std::max(target, current_lead_sec_ - 0.01);
  return current_lead_sec_;
}

TerminalHoldLatch::TerminalHoldLatch(
    double capture_longitudinal_m, double capture_speed_mps,
    double capture_acceleration_mps2, double goal_release_distance_m)
    : capture_longitudinal_m_(capture_longitudinal_m),
      capture_speed_mps_(capture_speed_mps),
      capture_acceleration_mps2_(capture_acceleration_mps2),
      goal_release_distance_m_(goal_release_distance_m) {}

bool TerminalHoldLatch::set_goal(double goal_s, DriveMode drive_mode) {
  const bool changed = !goal_s_ || !drive_mode_ ||
      std::abs(goal_s - *goal_s_) > goal_release_distance_m_ || drive_mode != *drive_mode_;
  goal_s_ = goal_s;
  drive_mode_ = drive_mode;
  if (changed) active_ = false;
  return changed;
}

void TerminalHoldLatch::reset() { active_ = false; }

bool TerminalHoldLatch::update(double longitudinal_error_m, double speed_mps,
                               double acceleration_mps2) {
  if (!active_ && std::abs(longitudinal_error_m) <= capture_longitudinal_m_ &&
      std::abs(speed_mps) <= capture_speed_mps_ &&
      std::abs(acceleration_mps2) <= capture_acceleration_mps2_) active_ = true;
  return active_;
}

}  // namespace simp_planner
