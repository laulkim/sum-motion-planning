#pragma once

#include "simp_planner/core.hpp"

#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <tuple>
#include <vector>

namespace simp_planner {




struct PlanningRevisionState {
  std::uint64_t request_revision{0};
  std::uint64_t path_revision{0};
  std::uint64_t structural_revision{0};
  std::uint64_t command_revision{0};
  std::uint64_t mode_revision{0};
};

bool plan_registration_is_current(const PlanningRevisionState& planned,
                                  const PlanningRevisionState& current);

bool planner_rebuild_required(const PlanningRevisionState& built,
                              const PlanningRevisionState& current);

enum class DriveModeControlState : std::uint8_t {
  WaitingForRequest = 0,
  WaitingForFeedback = 1,
  Ready = 2,
  StoppingForChange = 3,
  WaitingForCompletion = 4,
};

class DriveModeSupervisor {
 public:
  bool set_requested_mode(DriveMode mode);
  bool update_vehicle_feedback(DriveMode current_mode,
                               DriveMode vehicle_requested_mode,
                               bool transition_in_progress,
                               bool transition_complete);
  bool ready() const;
  bool should_publish_command(double measured_speed,
                              double stop_speed_threshold) const;
  DriveModeControlState state(double measured_speed,
                              double stop_speed_threshold) const;
  std::optional<DriveMode> requested_mode() const { return requested_mode_; }
  std::optional<DriveMode> current_mode() const { return current_mode_; }
  std::optional<DriveMode> vehicle_requested_mode() const {
    return vehicle_requested_mode_;
  }
  bool transition_in_progress() const { return transition_in_progress_; }
  bool transition_complete() const { return transition_complete_; }
  std::uint64_t confirmed_generation() const { return confirmed_generation_; }

 private:
  std::optional<DriveMode> requested_mode_;
  std::optional<DriveMode> current_mode_;
  std::optional<DriveMode> vehicle_requested_mode_;
  bool transition_in_progress_{false};
  bool transition_complete_{false};
  std::optional<DriveMode> last_confirmed_mode_;
  std::uint64_t confirmed_generation_{0};
};

const char* drive_mode_control_state_name(DriveModeControlState state);

struct BodyCommand {
  double vx{0.0};
  double vy{0.0};
  double yaw_rate{0.0};
  double planned_speed{0.0};
  double planned_acceleration{0.0};
  double planned_jerk{0.0};
  double planned_heading_acceleration{0.0};
  double motion_heading{0.0};
  double motion_curvature{0.0};
  double motion_heading_rate{0.0};
  double beta{0.0};
  double beta_rate{0.0};
  double yaw_acceleration{0.0};
  double segment_start_x{0.0};
  double segment_start_y{0.0};
  double segment_start_heading{0.0};
  double segment_end_x{0.0};
  double segment_end_y{0.0};
  double segment_end_heading{0.0};
  std::size_t state_index{0};
  std::size_t yaw_index{0};
  std::size_t action_index{0};
  double trajectory_time{0.0};
};

BodyCommand sample_body_command(const AllocationResult& allocation,
                                const std::vector<PlannerAction>& planned_actions,
                                double trajectory_time,
                                double execution_dt = 0.01);

std::int64_t align_time_ns(std::int64_t time_ns, double period_sec);

struct PredictedHandoverState {
  PlannerState state;
  double body_yaw{0.0};
  PlannerAction last_action;
  std::optional<AllocatorInitialState> allocator_state;
  std::optional<BodyCommand> expected_command;
};

PredictedHandoverState predict_handover_state(
    const PlannerState& current_state,
    double current_body_yaw,
    std::int64_t current_state_time_ns,
    std::int64_t handover_time_ns,
    std::optional<std::int64_t> active_plan_start_ns,
    const AllocationResult* allocation,
    const std::vector<PlannerAction>* planned_actions,
    double integration_dt,
    double safety_deceleration_limit = 1.0,
    double safety_jerk_limit = 0.8);

class JerkLimitedSafetyStop {
 public:
  JerkLimitedSafetyStop(double speed, double acceleration, double beta,
                        double motion_heading, double curvature,
                        double deceleration_limit, double jerk_limit,
                        double elapsed = 0.0);
  static JerkLimitedSafetyStop from_command(const BodyCommand& command,
                                             double deceleration_limit,
                                             double jerk_limit);
  bool stopped() const;
  BodyCommand sample() const;
  void advance(double dt);
  BodyCommand sample_and_advance(double dt);

 private:
  double current_jerk(double dt) const;
  double speed_{0.0};
  double acceleration_{0.0};
  double beta_{0.0};
  double motion_heading_{0.0};
  double curvature_{0.0};
  double deceleration_limit_{1.0};
  double jerk_limit_{0.8};
  double elapsed_{0.0};
};

struct PlanningRequestToken {
  std::uint64_t request_id{0};
  std::uint64_t input_revision{0};
  std::uint64_t structural_revision{0};
  std::vector<std::string> reasons;
  bool urgent{false};
  std::int64_t first_request_ns{0};
  std::int64_t latest_request_ns{0};
  std::int64_t started_ns{0};
  bool watchdog_triggered{false};
};

class LatestOnlyPlanningScheduler {
 public:
  LatestOnlyPlanningScheduler(double max_planning_frequency_hz,
                              double maximum_plan_age_sec,
                              double input_coalescing_sec);
  void request(std::int64_t now_ns, std::uint64_t input_revision,
               std::uint64_t structural_revision, std::string reason,
               bool urgent = false);
  void mark_plan_activated(std::int64_t now_ns);
  bool request_pending() const;
  std::optional<PlanningRequestToken> begin_if_due(std::int64_t now_ns,
                                                    bool has_active_plan,
                                                    bool worker_busy);

 private:
  const std::int64_t minimum_start_interval_ns_;
  const std::int64_t maximum_plan_age_ns_;
  const std::int64_t input_coalescing_ns_;
  mutable std::mutex mutex_;
  bool pending_{false};
  bool urgent_{false};
  std::int64_t first_request_ns_{0};
  std::int64_t latest_request_ns_{0};
  std::uint64_t latest_input_revision_{0};
  std::uint64_t latest_structural_revision_{0};
  std::set<std::string> reasons_;
  std::uint64_t request_id_{0};
  std::optional<std::int64_t> last_started_ns_;
  std::optional<std::int64_t> last_activated_ns_;
};

class AdaptiveHandoverTiming {
 public:
  AdaptiveHandoverTiming(double minimum_lead_sec = 0.12,
                         double initial_lead_sec = 0.15,
                         double maximum_lead_sec = 0.60,
                         double margin_sec = 0.03,
                         double quantile = 0.95,
                         double scale = 1.20,
                         std::size_t history_size = 64);
  double recommended_lead_sec() const { return current_lead_sec_; }
  double record(double compute_time_sec, bool late = false);

 private:
  double percentile(double quantile) const;
  double minimum_lead_sec_;
  double maximum_lead_sec_;
  double margin_sec_;
  double quantile_;
  double scale_;
  std::size_t history_size_;
  std::deque<double> samples_;
  double current_lead_sec_;
};

class TerminalHoldLatch {
 public:
  TerminalHoldLatch(double capture_longitudinal_m = 0.20,
                    double capture_speed_mps = 0.08,
                    double capture_acceleration_mps2 = 1.00,
                    double goal_release_distance_m = 0.30);
  bool set_goal(double goal_s, DriveMode drive_mode);
  void reset();
  bool update(double longitudinal_error_m, double speed_mps,
              double acceleration_mps2 = 0.0);
  bool active() const { return active_; }

 private:
  double capture_longitudinal_m_;
  double capture_speed_mps_;
  double capture_acceleration_mps2_;
  double goal_release_distance_m_;
  bool active_{false};
  std::optional<double> goal_s_;
  std::optional<DriveMode> drive_mode_;
};

}  // namespace simp_planner
