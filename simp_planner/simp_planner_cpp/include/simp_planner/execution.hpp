#pragma once

#include "simp_planner/runtime.hpp"
#include <functional>
#include <memory>

namespace simp_planner {

// This reference is open loop: neither producer nor consumer writes odometry
// corrections into it. Both processes use this exact sampler.
struct ReferenceSample {
  double time{0.0};
  double x{0.0}, y{0.0}, body_yaw{0.0};
  BodyCommand command;
};

struct ExecutionTrajectory {
  std::string session_id;
  std::uint64_t plan_id{0}, execution_epoch{0}, base_plan_id{0};
  std::int64_t start_ns{0};
  std::string frame_id{"map"};
  std::string type{"ACTIVE_PLAN"};
  std::string reason;
  DriveMode drive_mode{DriveMode::Forward};
  double deceleration{1.0}, jerk{0.8};
  std::vector<ReferenceSample> points;
};

void validate_execution_trajectory(const ExecutionTrajectory& trajectory);
ReferenceSample sample_execution(const ExecutionTrajectory& trajectory, std::int64_t stamp_ns);
ReferenceSample sample_braking(const ReferenceSample& initial, double elapsed,
                               double deceleration, double jerk);
std::vector<ReferenceSample> make_reference_samples(
    ReferenceSample initial, double horizon, double dt,
    const std::function<BodyCommand(double)>& command_at);
std::vector<ReferenceSample> make_stop_samples(
    ReferenceSample initial, double horizon, double dt, double deceleration, double jerk);
std::vector<ReferenceSample> make_turn_samples(
    ReferenceSample initial, double target_yaw, double horizon, double dt,
    const SpotTurnConfig& config);
PredictedHandoverState reference_handover(const ReferenceSample& sample);

// No locks inside: the owner serializes receive/advance/sample. A message is
// never applied on receipt; absolute ROS start time controls activation.
class ExecutionBuffer {
 public:
  std::string receive(std::shared_ptr<const ExecutionTrajectory> trajectory, std::int64_t now_ns);
  void advance(std::int64_t now_ns);
  ReferenceSample sample(std::int64_t stamp_ns) const;
  void reset();
  std::shared_ptr<const ExecutionTrajectory> active() const { return active_; }
  std::shared_ptr<const ExecutionTrajectory> pending() const { return pending_; }
  std::uint64_t epoch() const { return epoch_; }
  std::uint64_t latest_id() const { return latest_id_; }
  const std::string& session() const { return session_; }
  bool expired(std::int64_t stamp_ns) const;

 private:
  std::string session_;
  std::uint64_t epoch_{0}, latest_id_{0};
  std::shared_ptr<const ExecutionTrajectory> active_, pending_;
};
}  // namespace simp_planner
