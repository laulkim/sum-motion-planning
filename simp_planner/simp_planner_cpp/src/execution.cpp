#include "simp_planner/execution.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace simp_planner {
namespace {
double angle(double a, double b, double t) { return wrap_angle(a + t * wrap_angle(b - a)); }
void integrate(ReferenceSample& p, const BodyCommand& c, double dt) {
  const double yaw = p.body_yaw + 0.5 * c.yaw_rate * dt;
  p.x += (std::cos(yaw) * c.vx - std::sin(yaw) * c.vy) * dt;
  p.y += (std::sin(yaw) * c.vx + std::cos(yaw) * c.vy) * dt;
  p.body_yaw = wrap_angle(p.body_yaw + c.yaw_rate * dt);
}
void geometry(ReferenceSample& p) {
  p.command.segment_start_x = p.command.segment_end_x = p.x;
  p.command.segment_start_y = p.command.segment_end_y = p.y;
  p.command.motion_heading = wrap_angle(p.body_yaw + p.command.beta);
  p.command.segment_start_heading = p.command.segment_end_heading = p.command.motion_heading;
  p.command.trajectory_time = p.time;
}
// Same finite-step braking law for producer, tracker expiry and handover.
// The beta-rate tail avoids a yaw-rate jump at the entrance to braking.
std::vector<ReferenceSample> braking(ReferenceSample initial, double horizon, double dt,
                                    double deceleration, double jerk) {
  auto stop = JerkLimitedSafetyStop::from_command(initial.command, deceleration, jerk);
  const double beta0 = initial.command.beta;
  const double rate0 = initial.command.beta_rate;
  double t = 0.0;
  auto command_at = [&](double elapsed) {
    while (t < elapsed - 1.e-10) {
      const double step = std::min(0.01, elapsed - t);
      stop.advance(step);
      t += step;
    }
    auto c = stop.sample();
    const double u = std::min(elapsed, 0.5);
    c.beta = beta0 + rate0 * (u - u * u);
    c.beta_rate = rate0 * std::max(0.0, 1.0 - 2.0 * elapsed);
    c.vx = c.planned_speed * std::cos(c.beta);
    c.vy = c.planned_speed * std::sin(c.beta);
    c.yaw_rate = c.motion_heading_rate - c.beta_rate;
    if (elapsed < 0.5) c.yaw_acceleration += 2.0 * rate0;
    return c;
  };
  return make_reference_samples(initial, horizon, dt, command_at);
}
}  // namespace

void validate_execution_trajectory(const ExecutionTrajectory& tr) {
  if (tr.session_id.empty() || tr.frame_id.empty() || tr.plan_id == 0 ||
      tr.start_ns < 0 || tr.points.size() < 2 || tr.points.size() > 10001 ||
      !std::isfinite(tr.deceleration) || tr.deceleration <= 0 ||
      !std::isfinite(tr.jerk) || tr.jerk <= 0 ||
      static_cast<unsigned>(tr.drive_mode) > static_cast<unsigned>(DriveMode::SpotTurn))
    throw std::invalid_argument("invalid execution metadata");
  if (tr.type != "ACTIVE_PLAN" && tr.type != "MODE_STOP" && tr.type != "SAFETY_STOP" &&
      tr.type != "HARD_STOP" && tr.type != "MODE_WAIT" && tr.type != "TERMINAL_HOLD" &&
      tr.type != "SPOT_TURN_ROTATING" && tr.type != "SPOT_TURN_WAITING_CLEARANCE" && tr.type != "IDLE")
    throw std::invalid_argument("unknown execution type");
  double previous = -1.0;
  for (const auto& p : tr.points) {
    const auto& c = p.command;
    for (double v : {p.time, p.x, p.y, p.body_yaw, c.vx, c.vy, c.yaw_rate,
                    c.planned_speed, c.planned_acceleration, c.planned_jerk,
                    c.planned_heading_acceleration, c.motion_heading, c.motion_curvature,
                    c.motion_heading_rate, c.beta, c.beta_rate, c.yaw_acceleration})
      if (!std::isfinite(v)) throw std::invalid_argument("non-finite execution sample");
    if (p.time <= previous || p.time < 0 || p.time > 30.0 || c.planned_speed < -1.e-8)
      throw std::invalid_argument("invalid execution time/speed");
    previous = p.time;
  }
  if (std::abs(tr.points.front().time) > 1.e-9)
    throw std::invalid_argument("execution must start at relative time zero");
}

std::vector<ReferenceSample> make_reference_samples(
    ReferenceSample p, double horizon, double dt,
    const std::function<BodyCommand(double)>& command_at) {
  if (!std::isfinite(horizon) || horizon <= 0 || horizon > 120 ||
      !std::isfinite(dt) || dt <= 0) throw std::invalid_argument("invalid sampling interval");
  std::vector<ReferenceSample> result;
  p.time = 0;
  p.command = command_at(0);
  geometry(p);
  result.push_back(p);
  // Trapezoidal command integration; sample time is explicit, never tick count.
  while (p.time < horizon - 1.e-10) {
    const double step = std::min(dt, horizon - p.time);
    auto next = command_at(p.time + step);
    auto mid = p.command;
    mid.vx = 0.5 * (mid.vx + next.vx);
    mid.vy = 0.5 * (mid.vy + next.vy);
    mid.yaw_rate = 0.5 * (mid.yaw_rate + next.yaw_rate);
    integrate(p, mid, step);
    p.time += step;
    p.command = next;
    geometry(p);
    result.back().command.segment_end_x = p.x;
    result.back().command.segment_end_y = p.y;
    result.back().command.segment_end_heading = p.command.motion_heading;
    result.push_back(p);
  }
  return result;
}

std::vector<ReferenceSample> make_stop_samples(
    ReferenceSample p, double horizon, double dt, double deceleration, double jerk) {
  return braking(p, horizon, dt, deceleration, jerk);
}

ReferenceSample sample_braking(const ReferenceSample& initial, double elapsed,
                               double deceleration, double jerk) {
  if (elapsed <= 0) return initial;
  // An absurd clock jump is handled by the nodes; bound work here as well.
  auto points = braking(initial, std::min(elapsed, 120.0), 0.01, deceleration, jerk);
  return points.back();
}

std::vector<ReferenceSample> make_turn_samples(
    ReferenceSample p, double target, double horizon, double dt, const SpotTurnConfig& config) {
  double yaw = p.body_yaw;
  double rate = p.command.yaw_rate;
  double previous_t = 0;
  auto command_at = [&](double t) {
    BodyCommand c;
    const double step = t - previous_t;
    const double error = wrap_angle(target - yaw);
    const double desired = std::abs(error) <= config.yaw_tolerance_rad ? 0.0 :
        std::copysign(std::min(config.yaw_rate_max,
            std::sqrt(2.0 * config.yaw_rate_accel_max * std::abs(error))), error);
    const double old = rate;
    rate += std::clamp(desired - rate, -config.yaw_rate_accel_max * step,
                                          config.yaw_rate_accel_max * step);
    yaw = wrap_angle(yaw + 0.5 * (old + rate) * step);
    previous_t = t;
    c.yaw_rate = rate;
    c.yaw_acceleration = step > 0 ? (rate - old) / step : p.command.yaw_acceleration;
    c.beta_rate = -rate;
    c.beta = wrap_angle(p.body_yaw - yaw); // fixed motion heading at zero translation
    return c;
  };
  return make_reference_samples(p, horizon, dt, command_at);
}

ReferenceSample sample_execution(const ExecutionTrajectory& tr, std::int64_t stamp_ns) {
  const double t = 1.e-9 * static_cast<double>(stamp_ns - tr.start_ns);
  if (t <= 0) return tr.points.front();
  if (t > tr.points.back().time) {
    auto p = sample_braking(tr.points.back(), t - tr.points.back().time, tr.deceleration, tr.jerk);
    p.time = t;
    geometry(p);
    return p;
  }
  const auto upper = std::upper_bound(tr.points.begin(), tr.points.end(), t,
      [](double v, const ReferenceSample& p) { return v < p.time; });
  if (upper == tr.points.end()) return tr.points.back();
  const auto& a = *(upper - 1);
  const auto& b = *upper;
  const double alpha = (t - a.time) / (b.time - a.time);
  ReferenceSample p = a;
  auto lerp = [alpha](double x, double y) { return x + alpha * (y - x); };
  p.time = t;
  p.x = lerp(a.x, b.x); p.y = lerp(a.y, b.y);
  p.body_yaw = angle(a.body_yaw, b.body_yaw, alpha);
#define INTERPOLATE(field) p.command.field = lerp(a.command.field, b.command.field)
  INTERPOLATE(vx); INTERPOLATE(vy); INTERPOLATE(yaw_rate);
  INTERPOLATE(planned_speed); INTERPOLATE(planned_acceleration); INTERPOLATE(planned_jerk);
  INTERPOLATE(planned_heading_acceleration); INTERPOLATE(motion_curvature);
  INTERPOLATE(motion_heading_rate); INTERPOLATE(beta_rate); INTERPOLATE(yaw_acceleration);
#undef INTERPOLATE
  p.command.beta = angle(a.command.beta, b.command.beta, alpha);
  geometry(p);
  p.command.segment_end_x = b.x; p.command.segment_end_y = b.y;
  p.command.segment_end_heading = b.command.motion_heading;
  return p;
}

PredictedHandoverState reference_handover(const ReferenceSample& p) {
  const auto& c = p.command;
  return {{p.x, p.y, wrap_angle(p.body_yaw + c.beta), c.planned_speed,
           c.planned_acceleration, c.motion_heading_rate}, p.body_yaw,
          {c.planned_jerk, c.planned_heading_acceleration},
          AllocatorInitialState{c.beta, c.beta_rate, c.yaw_rate, c.yaw_acceleration}, c};
}

std::string ExecutionBuffer::receive(std::shared_ptr<const ExecutionTrajectory> tr, std::int64_t now) {
  try { validate_execution_trajectory(*tr); }
  catch (const std::exception& e) { return e.what(); }
  advance(now);
  if (!session_.empty() && tr->session_id != session_) return "SESSION_MISMATCH";
  if (tr->execution_epoch < epoch_) return "OLD_EPOCH";
  if (tr->execution_epoch == epoch_ && tr->plan_id == latest_id_) return "DUPLICATE";
  if (tr->plan_id <= latest_id_) return "OLD_PLAN";
  if (tr->start_ns <= now) return "LATE_PLAN";
  if (active_ && tr->frame_id != active_->frame_id) return "FRAME_MISMATCH";
  const auto base = active_ ? active_->plan_id : 0;
  if (tr->base_plan_id != base) return "BASE_MISMATCH";
  if (pending_ && tr->execution_epoch == epoch_) return "PENDING_EXISTS";
  if (active_) {
    const auto expected = sample_execution(*active_, tr->start_ns);
    const auto& first = tr->points.front();
    if (std::hypot(expected.x - first.x, expected.y - first.y) > 0.03 ||
        std::abs(wrap_angle(expected.body_yaw - first.body_yaw)) > 0.03 ||
        std::hypot(expected.command.vx - first.command.vx, expected.command.vy - first.command.vy) > 0.05 ||
        std::abs(expected.command.yaw_rate - first.command.yaw_rate) > 0.05 ||
        std::abs(expected.command.planned_acceleration - first.command.planned_acceleration) > 0.1)
      return "DISCONTINUOUS";
  }
  session_ = tr->session_id;
  epoch_ = tr->execution_epoch;
  latest_id_ = tr->plan_id;
  pending_ = std::move(tr);
  return "ACCEPTED";
}
void ExecutionBuffer::advance(std::int64_t now) {
  if (pending_ && now >= pending_->start_ns) { active_ = pending_; pending_.reset(); }
}
ReferenceSample ExecutionBuffer::sample(std::int64_t now) const {
  if (pending_ && now >= pending_->start_ns) return sample_execution(*pending_, now);
  return active_ ? sample_execution(*active_, now) : ReferenceSample{};
}
bool ExecutionBuffer::expired(std::int64_t now) const {
  return active_ && !pending_ && now > active_->start_ns +
      static_cast<std::int64_t>(std::llround(active_->points.back().time * 1.e9));
}
void ExecutionBuffer::reset() { *this = ExecutionBuffer{}; }
} // namespace simp_planner
