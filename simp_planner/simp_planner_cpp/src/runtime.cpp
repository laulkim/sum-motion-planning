#include "simp_planner/runtime.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace simp_planner {

bool plan_registration_is_current(const PlanningRevisionState& planned,
                                  const PlanningRevisionState& current) {
  return planned.structural_revision == current.structural_revision &&
         planned.command_revision == current.command_revision &&
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
    bool transition_in_progress, bool transition_complete) {
  current_mode_ = current_mode;
  vehicle_requested_mode_ = vehicle_requested_mode;
  transition_in_progress_ = transition_in_progress;
  transition_complete_ = transition_complete;
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
         !transition_in_progress_ && transition_complete_;
}

bool DriveModeSupervisor::should_publish_command(
    double measured_speed, double stop_speed_threshold) const {
  if (!requested_mode_ || !current_mode_) return false;
  if (ready()) return false;
  if (measured_speed > stop_speed_threshold) return false;
  if (transition_in_progress_ && vehicle_requested_mode_ &&
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
