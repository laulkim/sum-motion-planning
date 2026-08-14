#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <limits>
#include <optional>
#include <string>
#include <vector>

namespace simp_planner {

constexpr double kPi = 3.141592653589793238462643383279502884;

enum class DriveMode : std::uint8_t { Forward = 0, Reverse = 1, Left = 2, Right = 3 };

double drive_mode_heading_offset(DriveMode mode);
double motion_heading_from_body_yaw(double body_yaw, DriveMode mode);

struct PlanningCommand {
  double target_speed{0.0};
  DriveMode drive_mode{DriveMode::Forward};
};

// Per-cycle call counters for the three core planning stages, used for
// runtime profiling/visualization (not part of the planning logic itself).
struct PlanningCallCounts {
  int spatial_path_generation{0};
  int trajectory_planning{0};
  int allocation{0};
  // Raw number of generate_spatial_path_candidate() invocations, including
  // every curvature-violation retry (up to 8x recursive re-attempts with an
  // extended length) and the short-path fallback pass. Unlike
  // spatial_path_generation above (one per generation-phase entry), this
  // counts every individual candidate-generation attempt.
  int spatial_candidate_generation_attempts{0};
};

// Per-cycle accumulated wall-clock time (milliseconds) for the same three
// stages. Spatial path generation and trajectory generation each split into
// a "normal" and a "terminal/stop" bucket, since the two take different
// code paths with different cost; allocation (which also performs the
// trajectory-level collision check) has a single bucket.
struct PlanningBlockTimings {
  double spatial_normal_ms{0.0};
  double spatial_terminal_ms{0.0};
  double trajectory_normal_ms{0.0};
  double trajectory_terminal_ms{0.0};
  double allocation_ms{0.0};
};

void reset_planning_call_counts();
PlanningCallCounts planning_call_counts();
PlanningBlockTimings planning_block_timings();

struct PlannerState {
  double x{0.0};
  double y{0.0};
  double chi{0.0};
  double speed{0.0};
  double acceleration{0.0};
  double motion_heading_rate{0.0};
};

struct PlannerAction {
  double longitudinal_jerk{0.0};
  double motion_heading_acceleration{0.0};
};

struct AllocatorInitialState {
  double beta{0.0};
  double beta_rate{0.0};
  double yaw_rate{0.0};
  double yaw_acceleration{0.0};
};

struct VehicleConfig {
  double length{3.0};
  double width{2.0};
  double footprint_margin{0.0};
};

struct ConstraintConfig {
  double v_max{5.8};
  double v_min{0.0};
  double a_max{1.2};
  double a_min{-2.0};
  double jerk_max{3.0};
  double heading_rate_max{50.0 * kPi / 180.0};
  double heading_accel_max{90.0 * kPi / 180.0};
  double a_lat_max{2.8};
  double lateral_jerk_max{3.0};
  double curvature_max{0.20};
  double v_eff_min{0.20};
};

struct LateralPathConfig {
  std::vector<double> n_targets{
      -8.0, -7.75, -7.5, -7.25, -7.0, -6.75, -6.5, -6.25,
      -6.0, -5.75, -5.5, -5.25, -5.0, -4.75, -4.5, -4.25,
      -4.0, -3.75, -3.5, -3.25, -3.0, -2.75, -2.5, -2.25,
      -2.0, -1.75, -1.5, -1.25, -1.0, -0.75, -0.5, -0.25,
      0.0, 0.25, 0.5, 0.75, 1.0, 1.25, 1.5, 1.75,
      2.0, 2.25, 2.5, 2.75, 3.0, 3.25, 3.5, 3.75,
      4.0, 4.25, 4.5, 4.75, 5.0, 5.25, 5.5, 5.75,
      6.0, 6.25, 6.5, 6.75, 7.0, 7.25, 7.5, 7.75,
      8.0};
  double min_length{3.0};
  double max_length{50.0};
  double spatial_ds{0.25};
  double dynamic_margin{1.08};
  double preview_extra{2.0};
  int normal_shortlist_size{9};
  int terminal_shortlist_size{3};
  int normal_fallback_batch_size{4};
  double target_continuity_weight{3.0};
  bool reference_center_lock_enabled{true};
  double reference_center_lock_obstacle_cost_tolerance{1.0e-12};
  bool short_path_fallback_enabled{true};
  double short_path_collision_buffer{1.0};
  double short_path_stop_reserve{0.75};
  double short_path_max_length{20.0};
  int allocation_path_replan_max_attempts{16};
};

struct LongitudinalConfig {
  double horizon{4.0};
  double dt{0.1};
  double execution_dt{0.01};
  double speed_time_constant{1.6};
  double acceleration_response_time{0.9};
  double cruise_acceleration{0.8};
  double service_deceleration{1.0};
  double comfort_jerk{0.8};
  double terminal_acceleration_response_time{1.00};
  double terminal_braking_deceleration{1.60};
  double terminal_braking_curve_deceleration{0.85};
  double terminal_jerk_limit{1.50};
  double terminal_initial_jerk_continuity_weight{2.0};
  double terminal_braking_distance_buffer{0.0};
  double terminal_capture_speed{0.03};
  double curvature_lookahead_time{2.0};
  double curvature_lookahead_min{8.0};
  double lateral_jerk_speed_margin{0.30};
  double stop_target_offset{0.06};
  double stop_position_tolerance{0.05};
  double stop_speed_threshold{0.03};
  double obstacle_speed_backoff_factor{0.50};
  double obstacle_speed_recovery_step{0.25};
  double obstacle_speed_minimum{0.50};
  int obstacle_speed_max_attempts{3};
};

struct CostConfig {
  double w_speed{5.0};
  double w_low_speed{10.0};
  double w_progress{7.0};
  double w_accel{0.35};
  double w_jerk{0.50};
  double w_heading_accel{0.05};
  double w_lateral_jerk{0.10};
  double w_lateral_offset{0.08};
  double w_terminal_offset{0.12};
  double w_offset_overshoot{2.0};
  double w_curvature{8.0};
  double w_curvature_rate{1.2};
  double w_real_end_offset{90.0};
  double w_real_end_heading{45.0};
  double w_obstacle{110.0};
  double w_min_clearance{380.0};
  double w_collision{2.0e5};
  double obstacle_soft_margin{0.10};
  double obstacle_time_headway{0.20};
  double obstacle_soft_margin_max{1.25};
  double hard_clearance_margin_low_speed{0.10};
  double hard_clearance_margin{0.10};
  double hard_clearance_full_speed{2.0};
};

struct TerminalConstraintConfig {
  double activation_margin{3.0};
  double minimum_activation_distance{1.0};
  double safe_region_planning_buffer{12.0};
  double safe_region_settle_distance{4.0};
  double longitudinal_tolerance{0.20};
  double speed_tolerance{0.03};
  double acceleration_tolerance{0.10};
  double feedback_feasibility_horizon{15.0};
  double lateral_tolerance{0.20};
  double activation_distance{0.0};
  double position_profile_blend{0.0};
  bool hold_return_enabled{false};
  double hold_clearance_margin{0.0};
};

struct SimulationConfig {
  double path_end_margin{0.0};
  double virtual_extension_min{12.0};
  double virtual_extension_max{55.0};
  double virtual_extension_blend_length{8.0};
  double local_projection_back{2.0};
  double local_projection_forward{15.0};
};

struct AdaptiveReplanConfig {
  bool enabled{true};
  // Preserve long-range obstacle awareness at low speed.  The first active
  // bottleneck limiter below prevents a single-target path from being judged
  // against multiple alternating gates at once.
  double minimum_spatial_preview{32.0};
  double bottleneck_trigger_extra{1.50};
  double bottleneck_release_extra{2.00};
  double bottleneck_post_buffer{8.0};
  double bottleneck_minimum_decision_length{8.0};
  int bottleneck_release_samples{3};

  // Allocation-failure-driven soft-cost scheduling.  Hard collision margins
  // remain unchanged.
  double failure_soft_clearance_extra{0.20};
  double failure_sigmoid_slope{5.0};
  double speed_reference{1.0};
  double failure_memory_decay{0.75};
  double clearance_weight_gain{2.50};
  double memory_weight_gain{1.50};
  double clearance_speed_interaction_gain{1.00};
  double continuity_decay_gain{2.00};
  double reference_decay_gain{1.20};
  double minimum_continuity_scale{0.20};
  double minimum_reference_scale{0.35};
  double failed_target_weight{80.0};
  double failed_target_sigma{0.75};

  // Keep an active lateral maneuver tied to a fixed reference-path end
  // position so frequent replanning does not repeatedly restart the first
  // infinitesimal portion of a seventh-order profile.
  double maneuver_target_tolerance{0.30};
  double maneuver_release_progress_margin{0.50};
};

struct EnvConfig {
  VehicleConfig vehicle{};
  ConstraintConfig constraints{};
  LateralPathConfig lateral{};
  LongitudinalConfig longitudinal{};
  CostConfig cost{};
  TerminalConstraintConfig terminal{};
  SimulationConfig simulation{};
  AdaptiveReplanConfig adaptive_replan{};
};

struct FrenetProjection {
  double s{0.0};
  double n{0.0};
  double heading_error{0.0};
  double kappa{0.0};
  double kappa_s{0.0};
  double reference_heading{0.0};
  std::size_t segment_index{0};
};

class ReferencePath {
 public:
  ReferencePath() = default;
  ReferencePath(std::vector<double> s, std::vector<double> x,
                std::vector<double> y, std::vector<double> psi,
                std::vector<double> kappa);

  double s_min() const;
  double s_max() const;
  std::size_t size() const;
  void evaluate(double s_query, double& x, double& y, double& psi, double& kappa) const;
  FrenetProjection project(double px, double py, double psi) const;
  FrenetProjection project_local(double px, double py, double psi, double s_hint,
                                  double search_back = 2.0,
                                  double search_forward = 15.0) const;

  const std::vector<double>& s() const { return s_; }
  const std::vector<double>& x() const { return x_; }
  const std::vector<double>& y() const { return y_; }
  const std::vector<double>& psi() const { return psi_; }
  const std::vector<double>& kappa() const { return kappa_; }
  const std::vector<double>& kappa_s() const { return kappa_s_; }

 private:
  std::vector<double> s_;
  std::vector<double> x_;
  std::vector<double> y_;
  std::vector<double> psi_;
  std::vector<double> kappa_;
  std::vector<double> kappa_s_;
};

class Costmap2D {
 public:
  Costmap2D() = default;
  Costmap2D(std::vector<std::int8_t> data, int width, int height,
            double resolution, double origin_x, double origin_y,
            int occupied_threshold = 50, int unknown_value = -1,
            bool unknown_is_occupied = true,
            bool conservative_cell_correction = true);

  double distance_at_world(double x, double y) const;
  double clearance_single_circle(double x, double y, double radius) const;
  int width() const { return width_; }
  int height() const { return height_; }
  double resolution() const { return resolution_; }
  bool empty() const { return distance_field_.empty(); }

 private:
  int width_{0};
  int height_{0};
  double resolution_{0.1};
  double origin_x_{0.0};
  double origin_y_{0.0};
  std::vector<std::int8_t> data_;
  std::vector<double> distance_field_;
};

struct SpatialPathCandidate {
  int candidate_id{-1};
  double n_target{0.0};
  double lateral_length{0.0};
  double start_delay{0.0};
  bool terminal_position_only{false};
  std::vector<double> q_ref;
  std::vector<double> arc_length;
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> psi;
  std::vector<double> kappa;
  std::vector<double> kappa_l;
  std::vector<double> lateral_offset;
  std::vector<double> reference_x;
  std::vector<double> reference_y;
  std::vector<std::uint8_t> virtual_mask;
  double path_cost{0.0};
  double reference_offset_cost{0.0};
  double real_end_q{0.0};
  double real_end_l{0.0};
  double real_end_n{0.0};
  double real_end_heading_error{0.0};
  double virtual_extension_length{0.0};
};

struct TimeTrajectory {
  std::vector<PlannerState> states;
  std::vector<PlannerAction> actions;
  std::vector<double> progress;
  std::vector<double> speed_reference;
  std::vector<double> lateral_acceleration;
  std::vector<double> lateral_jerk;
  std::vector<double> clearance;
  bool valid_dynamic{false};
  bool collision_free{false};
  bool endpoint_overshoot_attempt{false};
  double trajectory_cost{0.0};
  bool terminal_constraint_active{false};
  bool terminal_spatial_valid{true};
  bool terminal_stop_reached{false};
  bool terminal_stop_feasible{false};
  bool terminal_braking_active{false};
  bool terminal_stop_valid{false};
  double terminal_longitudinal_error{0.0};
  double terminal_lateral_error{0.0};
  double terminal_speed_error{0.0};
  double predicted_stop_progress{0.0};
  double predicted_stop_error{0.0};
  int terminal_feedback_rollout_steps{0};
  double terminal_feedback_rollout_time{0.0};
  double coarse_min_clearance{0.0};
  double precise_min_clearance{0.0};

  bool terminal_valid() const { return terminal_spatial_valid && terminal_stop_valid; }
  bool safe() const { return valid_dynamic && collision_free && terminal_valid(); }
};

struct PlannerMotionTrajectory {
  std::string name;
  std::string description;
  std::vector<double> t;
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> chi;
  std::vector<double> kappa;
  std::vector<double> kappa_s;
  std::vector<double> speed;
  std::vector<double> acceleration;
  std::vector<double> longitudinal_jerk;
  std::vector<double> motion_heading_rate;
  std::vector<double> motion_heading_acceleration;
  std::vector<DriveMode> drive_mode;
};

struct PlanDiagnostics {
  std::string status{"UNKNOWN"};
  double compute_time_ms{0.0};
  int number_of_lateral_paths{0};
  int number_of_safe_paths{0};
  int selected_candidate_id{-1};
  std::string selected_path_mode{"none"};
  double selected_n_target{std::numeric_limits<double>::quiet_NaN()};
  double selected_lateral_length{0.0};
  double selected_start_delay{0.0};
  double selected_min_clearance{0.0};
  double selected_total_cost{0.0};
  double requested_target_speed{0.0};
  double applied_target_speed{0.0};
  bool obstacle_speed_cap_active{false};
  int speed_search_attempts{0};
  bool terminal_mode_active{false};
  bool terminal_constraint_active{false};
  bool terminal_safe_region_active{false};
  bool terminal_goal_region_safe{true};
  double terminal_goal_offset{std::numeric_limits<double>::quiet_NaN()};
  double terminal_goal_clearance{std::numeric_limits<double>::infinity()};
  bool reference_center_lock_active{false};
  bool bottleneck_limited{false};
  double decision_horizon_length{0.0};
  bool adaptive_replan_active{false};
  double allocation_failure_severity{0.0};
  double allocation_failure_memory{0.0};
  std::string lateral_selection_basis{"SPATIAL_ONLY"};
  bool short_path_fallback_active{false};
  double short_path_length{0.0};
  int excluded_candidate_id{-1};
};

struct PlanResult {
  TimeTrajectory trajectory;
  std::optional<SpatialPathCandidate> selected_path;
  PlannerMotionTrajectory motion;
  PlanDiagnostics diagnostics;
};

struct ManeuverProfileState {
  double target{0.0};
  double elapsed_length{0.0};
  double start_delay{0.0};
  double lateral_length{0.0};
  double n0{0.0};
  double n1{0.0};
  double n2{0.0};
  double n3{0.0};
};

struct PlannerContinuityState {
  std::optional<double> lateral_target_hint;
  std::optional<double> maneuver_target;
  std::optional<double> maneuver_remaining_length;
  std::optional<ManeuverProfileState> maneuver_profile;
  bool terminal_safe_region_active_last{false};
  double allocation_failure_severity{0.0};
  double allocation_failure_memory{0.0};
  std::optional<double> failed_lateral_target;
};



enum class AllocationProfile : std::uint8_t {
  LateralPriority = 0,
  Balanced = 1,
  YawPriority = 2,
  MinimumVy = 3,
};

const char* allocation_profile_name(AllocationProfile profile);

struct OrientedFootprintConfig {
  int circle_count{3};
  double maximum_translation_step{0.20};
  double maximum_yaw_step{2.0 * kPi / 180.0};
};

struct OrientedCollisionResult {
  bool collision_free{true};
  double minimum_clearance{std::numeric_limits<double>::infinity()};
  int checked_poses{0};
  int checked_circles{0};
};

struct AllocationLimits {
  // Lateral-priority allocation: preserve motion direction while allowing the
  // body heading to lag, so beta and body-frame vy carry more of the maneuver.
  std::array<double, 4> beta_max_deviation{{45.0 * kPi / 180.0,
                                            40.0 * kPi / 180.0,
                                            25.0 * kPi / 180.0,
                                            25.0 * kPi / 180.0}};
  std::array<double, 4> body_yaw_fraction{{0.20, 0.25, 0.15, 0.15}};
  std::array<double, 4> beta_return_gain{{0.35, 0.40, 0.45, 0.45}};
  double beta_rate_max{40.0 * kPi / 180.0};
  double beta_accel_max{180.0 * kPi / 180.0};
  double beta_filter_tau{0.80};
  double yaw_rate_max{50.0 * kPi / 180.0};
  double yaw_accel_max{90.0 * kPi / 180.0};
  double yaw_jerk_max{400.0 * kPi / 180.0};
  double vx_accel_max{3.2};
  double vy_accel_max{3.2};
  double vx_jerk_max{9.0};
  double vy_jerk_max{9.0};
  double stop_speed_threshold{0.03};
  double allocation_active_speed{0.30};
  double path_replay_tolerance{0.30};
  double psi_replay_tolerance{1.0 * kPi / 180.0};
};

struct AllocationResult {
  PlannerMotionTrajectory trajectory;
  std::vector<double> beta_center;
  std::vector<double> beta_desired;
  std::vector<double> beta;
  std::vector<double> beta_rate;
  std::vector<double> beta_acceleration;
  std::vector<double> psi;
  std::vector<double> vx;
  std::vector<double> vy;
  std::vector<double> yaw_rate;
  std::vector<double> yaw_acceleration;
  std::vector<double> yaw_jerk;
  std::vector<double> vx_acceleration;
  std::vector<double> vy_acceleration;
  std::vector<double> vx_jerk;
  std::vector<double> vy_jerk;
  std::vector<std::uint8_t> fallback_used;
  std::vector<std::uint8_t> mode_change_while_moving;
  std::vector<double> speed_reconstruction_error;
  std::vector<double> heading_relation_error;
  std::vector<double> world_velocity_identity_error;

  AllocatorInitialState state_at(std::size_t index) const;
};



struct AllocationSelectionResult {
  AllocationResult allocation;
  OrientedCollisionResult collision;
  AllocationProfile profile{AllocationProfile::LateralPriority};
  int candidates_evaluated{0};
};

AllocationLimits allocation_limits_for_profile(AllocationProfile profile);

OrientedCollisionResult check_oriented_allocation_collision(
    const AllocationResult& allocation,
    const Costmap2D& costmap,
    const VehicleConfig& vehicle,
    const CostConfig& cost,
    const OrientedFootprintConfig& footprint = OrientedFootprintConfig{});

AllocationSelectionResult allocate_with_oriented_collision_search(
    const PlannerMotionTrajectory& trajectory,
    const Costmap2D& costmap,
    const VehicleConfig& vehicle,
    const CostConfig& cost,
    std::optional<AllocatorInitialState> initial_state = std::nullopt,
    const OrientedFootprintConfig& footprint = OrientedFootprintConfig{});

class PathVelocityPlanner {
 public:
  PathVelocityPlanner(EnvConfig config, ReferencePath path,
                      std::optional<Costmap2D> costmap = std::nullopt);

  PlanResult plan(const PlannerState& state,
                  const PlannerAction& previous_action,
                  const PlanningCommand& command);

  void set_lateral_target_hint(std::optional<double> hint);
  std::optional<double> lateral_target_hint() const { return lateral_target_hint_; }
  void set_feasible_speed_cap_hint(std::optional<double> cap, int success_count = 0);
  std::optional<double> feasible_speed_cap() const { return last_feasible_speed_cap_; }
  int feasible_speed_cap_success_count() const { return speed_cap_success_count_; }
  void record_allocation_failure(double minimum_clearance, double speed,
                                 std::optional<double> failed_lateral_target);
  void record_allocation_success();
  void set_one_shot_excluded_candidate(
      std::optional<int> candidate_id,
      std::optional<double> lateral_target = std::nullopt);
  void set_one_shot_excluded_paths(
      std::vector<int> candidate_ids,
      std::vector<double> lateral_targets);
  double allocation_failure_memory() const { return allocation_failure_memory_; }
  PlannerContinuityState export_continuity_state(const PlannerState& state) const;
  void import_continuity_state(const PlannerContinuityState& continuity,
                               const PlannerState& state);

  const EnvConfig& config() const { return config_; }
  const ReferencePath& path() const { return path_; }
  const std::optional<Costmap2D>& costmap() const { return costmap_; }

 private:
  PlanResult plan_at_speed(const PlannerState& state,
                           const PlannerAction& previous_action,
                           double target_speed,
                           DriveMode drive_mode,
                           const std::vector<int>& excluded_candidate_ids,
                           const std::vector<double>& excluded_lateral_targets);
  TimeTrajectory emergency_stop(const PlannerState& state,
                                const PlannerAction& previous_action,
                                double target_speed,
                                bool terminal_stop_required);
  std::vector<double> speed_trials(double requested_speed) const;

  EnvConfig config_;
  ReferencePath path_;
  std::optional<Costmap2D> costmap_;
  std::optional<double> lateral_target_hint_;
  std::optional<double> last_feasible_speed_cap_;
  int speed_cap_success_count_{0};
  int speed_cap_probe_interval_{18};
  double allocation_failure_severity_{0.0};
  double allocation_failure_memory_{0.0};
  std::optional<double> failed_lateral_target_;
  std::vector<int> one_shot_excluded_candidate_ids_;
  std::vector<double> one_shot_excluded_lateral_targets_;
  std::optional<ManeuverProfileState> maneuver_profile_;
  std::optional<double> maneuver_start_s_;
  bool terminal_safe_region_active_last_{false};
};

AllocationResult allocate_trajectory(
    const PlannerMotionTrajectory& trajectory,
    const AllocationLimits& limits = AllocationLimits{},
    std::optional<AllocatorInitialState> initial_state = std::nullopt);

double wrap_angle(double angle);
std::vector<double> unwrap_angles(const std::vector<double>& angles);
double effective_hard_clearance_margin(double speed, const CostConfig& config);
double obstacle_soft_clearance(double speed, const CostConfig& config);
double circumscribed_radius(double length, double width, double margin = 0.0);

}  // namespace simp_planner
