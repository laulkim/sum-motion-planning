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

// 차량이 보고하는 바퀴 configuration 상태. DriveModeState.msg의
// STATUS_ALIGNING(0)/STATUS_READY(1)와 1:1로 대응한다 -- 예전에는
// transition_in_progress/transition_complete 두 bool로 나뉘어 있었지만,
// 실제로 나오는 조합이 항상 서로의 부정(complete == !in_progress)이라
// 독립적인 정보가 아니었다. 필드 하나로 합친다.
enum class VehicleModeStatus : std::uint8_t {
  Aligning = 0,
  Ready = 1,
};

class DriveModeSupervisor {
 public:
  bool set_requested_mode(DriveMode mode);
  bool update_vehicle_feedback(DriveMode current_mode,
                               DriveMode vehicle_requested_mode,
                               VehicleModeStatus vehicle_status);
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
  VehicleModeStatus vehicle_status() const { return vehicle_status_; }
  std::uint64_t confirmed_generation() const { return confirmed_generation_; }

 private:
  std::optional<DriveMode> requested_mode_;
  std::optional<DriveMode> current_mode_;
  std::optional<DriveMode> vehicle_requested_mode_;
  VehicleModeStatus vehicle_status_{VehicleModeStatus::Aligning};
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

struct SpotTurnConfig {
  double heading_jump_threshold_rad{0.349066};  // 20 deg
  double safety_margin{0.0};
  double yaw_rate_max{0.3};
  double yaw_rate_accel_max{0.3};
  double yaw_tolerance_rad{0.02};
};

// Input includes the publisher's extra curvature-sampling point: the last
// point is dropped after its forward-difference curvature estimates the
// true last usable point.
std::shared_ptr<ReferencePath> build_reference_path(
    const std::vector<double>& x, const std::vector<double>& y,
    const std::vector<double>& yaw);

struct ReferencePathSplit {
  std::shared_ptr<ReferencePath> before;
  std::shared_ptr<ReferencePath> after;  // null when the array has no interior corner.
};

// Scans the interior of the array for a point whose forward-difference
// curvature exceeds `curvature_max` -- a heading change sharper than
// steering can ever track, so no comparison against the vehicle's actual
// heading is needed: it unconditionally needs a spot turn. When found, the
// path is split there: `before` ends at the corner (its own last point);
// `after` restarts on the far side with its own s = 0. Returns `after ==
// nullptr` when no interior corner exists, with `before` covering the whole
// array (same result as build_reference_path()).
ReferencePathSplit split_reference_path_at_corner(
    const std::vector<double>& x, const std::vector<double>& y,
    const std::vector<double>& yaw, double curvature_max);

// The body yaw a vehicle in `mode` must hold to drive a reference path whose
// first point's motion heading is `path_start_psi`.
double spot_turn_target_body_yaw(double path_start_psi, DriveMode mode);

bool spot_turn_feasible(const Costmap2D& costmap, const PlannerState& state,
                        const VehicleConfig& vehicle, double safety_margin);

// Acceleration-limited rotation generated as a yaw trajectory. Each sample
// is fed back from the previous trajectory point (yaw, yaw rate) rather than
// measured odometry, so consecutive points are continuous and tracking the
// trajectory is left to the controller. Completion means the trajectory
// itself has reached the target heading and come to rest.
class YawRotationProfile {
 public:
  explicit YawRotationProfile(const SpotTurnConfig& config = {});
  void engage(double start_yaw, double target_yaw);
  BodyCommand sample(double dt);
  bool done() const { return done_; }

 private:
  SpotTurnConfig config_;
  double target_yaw_{0.0};
  double yaw_{0.0};
  double rate_{0.0};
  bool done_{false};
};

enum class SpotTurnManeuverState { Inactive, AligningWheels, Rotating };

class SpotTurnManeuver {
 public:
  explicit SpotTurnManeuver(const SpotTurnConfig& config = {});
  void trigger(double target_body_yaw, DriveMode external_mode);
  bool on_mode_ready(const DriveModeSupervisor& supervisor);
  // Rotating only: the whole rotation as one command per dt, starting from
  // the trajectory point whose body yaw is `start_yaw` and ending at rest.
  std::vector<BodyCommand> rotation_commands(double start_yaw, double dt);
  // Rotating -> return alignment: request the external mode again.
  void finish_rotation(DriveModeSupervisor& supervisor);
  bool set_external_requested_mode(DriveMode mode, DriveModeSupervisor& supervisor);
  SpotTurnManeuverState state() const { return state_; }
  DriveMode external_mode() const { return external_mode_; }
  const char* state_name() const;

 private:
  SpotTurnManeuverState state_{SpotTurnManeuverState::Inactive};
  bool returning_{false};
  double target_yaw_{0.0};
  DriveMode external_mode_{DriveMode::Forward};
  YawRotationProfile rotation_;
};

// Tracking trajectory: the time-indexed array the planner hands to the
// controller. points[i] is the vehicle state at stamp + i * period and the
// command held from that instant until the next point. Past the last point
// the last point is held, so every trajectory must end at rest.
enum class TrajectorySegment : std::uint8_t { Drive = 0, Stop = 1, Turn = 2, Hold = 3 };

enum class ExecutionState : std::uint8_t {
  Idle,
  ActivePlan,
  SafetyStop,
  TerminalHold,
  ModeStop,
  ModeWait,
  SpotTurnRotating,
  SpotTurnWaitingClearance,
};

const char* execution_state_name(ExecutionState state);

struct TrackingPoint {
  double x{0.0};
  double y{0.0};
  double body_yaw{0.0};
  BodyCommand command;
  DriveMode mode{DriveMode::Forward};
  TrajectorySegment segment{TrajectorySegment::Hold};
  ExecutionState execution{ExecutionState::Idle};
  std::uint64_t plan_id{0};
};

struct TrackingTrajectory {
  std::int64_t stamp_ns{0};
  std::int64_t period_ns{10000000};
  std::vector<TrackingPoint> points;

  // Floor index of the point governing time_ns, clamped to the array.
  std::size_t index_at(std::int64_t time_ns) const;
  const TrackingPoint& at(std::int64_t time_ns) const;
};

// Zero-motion command that keeps the previous command's slip angle, or the
// mode's nominal one when the hold is in a different mode.
BodyCommand hold_command(const BodyCommand& previous, DriveMode previous_mode,
                         DriveMode mode, double body_yaw);

// The handover state a new plan starts from when it begins at `point`.
// `previous` is the point commanded one period earlier: beta is the state at
// `point`, while the allocator's rates are those of the interval ending there
// (as AllocationResult::state_at), since the new plan recomputes `point`'s own.
PredictedHandoverState handover_from_point(const TrackingPoint& point,
                                           const TrackingPoint& previous);

// Builds a TrackingTrajectory point by point. The pose of each new point is
// integrated from the previous one with the same body-velocity kinematics the
// vehicle executes, so the array is what the vehicle does under ideal tracking.
class TrackingTrajectoryBuilder {
 public:
  TrackingTrajectoryBuilder(std::int64_t stamp_ns, std::int64_t period_ns);
  // Copies `previous` unchanged for every grid time in [stamp, until_ns).
  void copy_prefix(const TrackingTrajectory& previous, std::int64_t until_ns);
  // Repeats `point` as a hold for every grid time in [next time, until_ns).
  void hold_prefix(const TrackingPoint& point, std::int64_t until_ns);
  void append(const BodyCommand& command, DriveMode mode, TrajectorySegment segment,
              ExecutionState execution, std::uint64_t plan_id);
  // Appends the stop's samples until it is stopped, including the final
  // stationary sample.
  void append_stop(JerkLimitedSafetyStop stop, DriveMode mode,
                   ExecutionState execution, std::uint64_t plan_id);
  void append_hold(DriveMode mode, ExecutionState execution, std::uint64_t plan_id);
  std::int64_t next_time_ns() const;
  bool empty() const { return trajectory_.points.empty(); }
  const TrackingPoint& back() const { return trajectory_.points.back(); }
  TrackingTrajectory take() { return std::move(trajectory_); }

 private:
  void push(TrackingPoint point);

  TrackingTrajectory trajectory_;
  double dt_{0.01};
  double x_{0.0};
  double y_{0.0};
  double body_yaw_{0.0};
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
