#include "simp_planner/core.hpp"
#include "simp_planner/runtime.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <simp_planner_msgs/msg/drive_mode_state.hpp>
#include <simp_planner_msgs/msg/executed_command.hpp>
#include <simp_planner_msgs/msg/reference_path.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace simp_planner {
namespace {

using ReferencePathMsg = simp_planner_msgs::msg::ReferencePath;
using ExecutedCommandMsg = simp_planner_msgs::msg::ExecutedCommand;
using DriveModeStateMsg = simp_planner_msgs::msg::DriveModeState;

constexpr double kMotionThreshold = 0.03;

double quaternion_to_yaw(double x, double y, double z, double w) {
  const double siny_cosp = 2.0 * (w * z + x * y);
  const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
  return std::atan2(siny_cosp, cosy_cosp);
}

geometry_msgs::msg::Quaternion yaw_to_quaternion(double yaw) {
  geometry_msgs::msg::Quaternion q;
  q.z = std::sin(0.5 * yaw);
  q.w = std::cos(0.5 * yaw);
  return q;
}

std::vector<double> cumulative_arc_length(const std::vector<double>& x,
                                          const std::vector<double>& y) {
  std::vector<double> s(x.size(), 0.0);
  for (std::size_t i = 1; i < x.size(); ++i) {
    const double ds = std::hypot(x[i] - x[i - 1], y[i] - y[i - 1]);
    if (ds <= 1.0e-4) throw std::invalid_argument("reference path contains duplicate points");
    s[i] = s[i - 1] + ds;
  }
  return s;
}

struct ExecutablePlan {
  PlanResult result;
  AllocationResult allocation;
  std::int64_t start_ns{0};
  std::int64_t ready_ns{0};
  std::uint64_t plan_id{0};
  std::uint64_t structural_revision{0};
  std::string frame_id;
  double allocation_min_clearance{0.0};
  std::string allocation_profile{"LATERAL_PRIORITY"};
  int allocation_candidates_evaluated{1};
  int footprint_circle_count{8};
  int outer_speed_attempts{1};
  int path_replans{0};
  double compute_ms{0.0};
};

struct InputSnapshot {
  PlannerState state;
  double body_yaw{0.0};
  std::int64_t state_time_ns{0};
  std::shared_ptr<const ReferencePath> path;
  std::shared_ptr<const Costmap2D> costmap;
  double target_speed{0.0};
  DriveMode requested_mode{DriveMode::Forward};
  DriveMode drive_mode{DriveMode::Forward};
  DriveMode reference_mode{DriveMode::Forward};
  bool mode_ready{false};
  std::uint64_t mode_revision{0};
  std::uint64_t input_revision{0};
  std::uint64_t path_revision{0};
  std::uint64_t structural_revision{0};
  std::uint64_t command_revision{0};
  std::string frame_id{"map"};
};

struct ModeStatusSnapshot {
  int requested_mode{-1};
  int actual_mode{-1};
  int vehicle_requested_mode{-1};
  int reference_mode{-1};
  bool ready{false};
  bool transition_in_progress{false};
  bool transition_complete{false};
};

}  // namespace

class PlannerNodeCpp final : public rclcpp::Node {
 public:
  PlannerNodeCpp()
      : Node("planner_node_cpp"),
        scheduler_(declare_parameter<double>("max_planning_frequency_hz", 10.0),
                   declare_parameter<double>("maximum_plan_age_sec", 0.50),
                   declare_parameter<double>("input_coalescing_sec", 0.005)),
        handover_timing_(
            declare_parameter<double>("planning_handover_min_lead_sec", 0.12),
            declare_parameter<double>("planning_handover_initial_lead_sec", 0.15),
            declare_parameter<double>("planning_handover_max_lead_sec", 0.60),
            declare_parameter<double>("planning_handover_margin_sec", 0.03),
            0.95,
            declare_parameter<double>("planning_handover_scale", 1.20)) {
    trajectory_knot_dt_ = declare_parameter<double>("trajectory_knot_dt_sec", 0.10);
    command_frequency_hz_ = declare_parameter<double>("command_frequency_hz", 100.0);
    scheduler_frequency_hz_ = declare_parameter<double>("planning_scheduler_frequency_hz", 100.0);
    mode_change_stop_speed_ = declare_parameter<double>("mode_change_stop_speed_mps", 0.03);
    mode_command_period_sec_ = declare_parameter<double>("mode_command_period_sec", 0.25);
    if (!(trajectory_knot_dt_ > 0.0) || !(command_frequency_hz_ > 0.0) ||
        !(scheduler_frequency_hz_ > 0.0)) {
      throw std::invalid_argument("planner timing parameters must be positive");
    }
    command_dt_ = 1.0 / command_frequency_hz_;
    config_.longitudinal.dt = trajectory_knot_dt_;
    config_.longitudinal.execution_dt = command_dt_;
    footprint_circle_count_ = declare_parameter<int>("oriented_footprint_circle_count", 3);
    footprint_translation_step_m_ = declare_parameter<double>("oriented_footprint_translation_step_m", 0.20);
    footprint_yaw_step_deg_ = declare_parameter<double>("oriented_footprint_yaw_step_deg", 2.0);
    if (footprint_circle_count_ < 1 || footprint_translation_step_m_ <= 0.0 ||
        footprint_yaw_step_deg_ <= 0.0) {
      throw std::invalid_argument("invalid oriented footprint parameters");
    }

    input_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    planning_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    execution_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    auto static_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    cmd_stamped_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
        "/planner/cmd_vel_stamped", 200);
    executed_pub_ = create_publisher<ExecutedCommandMsg>("/planner/executed_command", 200);
    status_pub_ = create_publisher<std_msgs::msg::String>("/planner/status", static_qos);
    execution_state_pub_ = create_publisher<std_msgs::msg::String>(
        "/planner/execution_state", static_qos);
    trajectory_pub_ = create_publisher<nav_msgs::msg::Path>(
        "/planner/selected_trajectory", static_qos);
    trajectory_data_pub_ = create_publisher<ReferencePathMsg>(
        "/planner/selected_trajectory_data", static_qos);
    mode_command_pub_ = create_publisher<std_msgs::msg::UInt8>(
        "/vehicle/drive_mode_command", static_qos);

    rclcpp::SubscriptionOptions input_options;
    input_options.callback_group = input_group_;
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 20, std::bind(&PlannerNodeCpp::odom_callback, this, std::placeholders::_1),
        input_options);
    path_sub_ = create_subscription<ReferencePathMsg>(
        "/reference_path_data", static_qos,
        std::bind(&PlannerNodeCpp::path_callback, this, std::placeholders::_1), input_options);
    costmap_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/costmap", static_qos,
        std::bind(&PlannerNodeCpp::costmap_callback, this, std::placeholders::_1), input_options);
    target_speed_sub_ = create_subscription<std_msgs::msg::Float64>(
        "/target_speed", static_qos,
        std::bind(&PlannerNodeCpp::target_speed_callback, this, std::placeholders::_1),
        input_options);
    requested_mode_sub_ = create_subscription<std_msgs::msg::UInt8>(
        "/requested_drive_mode", static_qos,
        std::bind(&PlannerNodeCpp::requested_mode_callback, this, std::placeholders::_1),
        input_options);
    vehicle_mode_sub_ = create_subscription<DriveModeStateMsg>(
        "/vehicle/drive_mode_state", static_qos,
        std::bind(&PlannerNodeCpp::vehicle_mode_callback, this, std::placeholders::_1),
        input_options);

    planning_timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / scheduler_frequency_hz_),
        std::bind(&PlannerNodeCpp::planning_callback, this), planning_group_);
    command_timer_ = create_wall_timer(
        std::chrono::duration<double>(command_dt_),
        std::bind(&PlannerNodeCpp::command_callback, this), execution_group_);

    RCLCPP_INFO(get_logger(),
                "Native C++ planner ready: planning<=%.1f Hz, command=%.1f Hz, "
                "knot=%.3f s, allocation=LATERAL_PRIORITY",
                get_parameter("max_planning_frequency_hz").as_double(),
                command_frequency_hz_, trajectory_knot_dt_);
  }

 private:
  std::int64_t now_ns() const { return get_clock()->now().nanoseconds(); }

  void request_replan(const std::string& reason, bool urgent) {
    const auto stamp = now_ns();
    std::uint64_t input_revision;
    std::uint64_t structural_revision;
    {
      std::lock_guard<std::mutex> lock(input_mutex_);
      input_revision = ++input_revision_;
      structural_revision = structural_revision_;
    }
    scheduler_.request(stamp, input_revision, structural_revision, reason, urgent);
  }

  bool inputs_ready_locked() const {
    return received_odom_ && received_path_ && received_costmap_ &&
           received_target_speed_ && received_requested_mode_ &&
           received_vehicle_mode_;
  }

  std::optional<InputSnapshot> snapshot() const {
    std::lock_guard<std::mutex> lock(input_mutex_);
    if (!inputs_ready_locked()) return std::nullopt;
    const auto requested = mode_supervisor_.requested_mode();
    const auto current = mode_supervisor_.current_mode();
    if (!requested || !current) return std::nullopt;
    return InputSnapshot{current_state_, current_body_yaw_, current_state_time_ns_,
                         reference_path_, costmap_, target_speed_, *requested, *current,
                         reference_mode_, mode_supervisor_.ready(), mode_revision_,
                         input_revision_, path_revision_, structural_revision_,
                         command_revision_, path_frame_};
  }

  static PlanningRevisionState revision_state(const InputSnapshot& input) {
    return {input.input_revision, input.path_revision, input.structural_revision,
            input.command_revision, input.mode_revision};
  }

  PlanningRevisionState current_revision_state_locked() const {
    return {input_revision_, path_revision_, structural_revision_,
            command_revision_, mode_revision_};
  }

  bool snapshot_is_current(const InputSnapshot& input) const {
    std::lock_guard<std::mutex> lock(input_mutex_);
    // Odometry and rolling local-reference updates are soft inputs.
    return plan_registration_is_current(
        revision_state(input), current_revision_state_locked());
  }

  void invalidate_pending_plan() {
    std::lock_guard<std::mutex> lock(execution_mutex_);
    pending_plan_.reset();
  }

  void invalidate_motion_plan() {
    std::lock_guard<std::mutex> lock(execution_mutex_);
    pending_plan_.reset();
    active_plan_.reset();
    safety_stop_.reset();
    mode_stop_.reset();
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    const auto& q = msg->pose.pose.orientation;
    const double body_yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w);
    const double body_vx = msg->twist.twist.linear.x;
    const double body_vy = msg->twist.twist.linear.y;
    const double speed = std::hypot(body_vx, body_vy);
    std::optional<BodyCommand> last;
    {
      std::lock_guard<std::mutex> lock(execution_mutex_);
      last = last_command_;
    }
    std::optional<DriveMode> confirmed_mode;
    {
      std::lock_guard<std::mutex> lock(input_mutex_);
      confirmed_mode = mode_supervisor_.current_mode();
    }
    double chi;
    if (speed > kMotionThreshold) {
      chi = wrap_angle(body_yaw + std::atan2(body_vy, body_vx));
    } else if (confirmed_mode) {
      // At standstill the velocity direction is undefined. Reconstruct it from
      // the vehicle-confirmed mode instead of retaining the previous phase's
      // motion direction. This is essential for Forward -> Crab/Reverse.
      chi = motion_heading_from_body_yaw(body_yaw, *confirmed_mode);
    } else {
      chi = last_motion_chi_.value_or(body_yaw);
    }
    last_motion_chi_ = chi;
    const double acceleration = last ? last->planned_acceleration : 0.0;
    const double heading_rate = last ? last->motion_heading_rate : 0.0;
    {
      std::lock_guard<std::mutex> lock(input_mutex_);
      current_state_ = {msg->pose.pose.position.x, msg->pose.pose.position.y,
                        chi, speed, acceleration, heading_rate};
      current_body_yaw_ = body_yaw;
      current_state_time_ns_ = rclcpp::Time(msg->header.stamp).nanoseconds();
      if (current_state_time_ns_ <= 0) current_state_time_ns_ = now_ns();
      odom_frame_ = msg->header.frame_id;
      received_odom_ = true;
    }
    request_replan("LATEST_ODOMETRY", false);
  }

  static bool same_reference_path(const ReferencePath& lhs,
                                  const ReferencePath& rhs,
                                  double tolerance = 1.0e-9) {
    if (lhs.size() != rhs.size()) return false;
    const auto equal_vector = [tolerance](const std::vector<double>& a,
                                          const std::vector<double>& b) {
      return std::equal(a.begin(), a.end(), b.begin(),
                        [tolerance](double x, double y) {
                          return std::abs(x - y) <= tolerance;
                        });
    };
    return equal_vector(lhs.x(), rhs.x()) && equal_vector(lhs.y(), rhs.y()) &&
           equal_vector(lhs.psi(), rhs.psi()) &&
           equal_vector(lhs.kappa(), rhs.kappa());
  }

  bool is_soft_reference_continuation_locked(const ReferencePath& new_path,
                                              DriveMode new_mode) const {
    if (!reference_path_ || new_mode != reference_mode_ || !received_odom_) return false;
    try {
      const auto old_projection = reference_path_->project(
          current_state_.x, current_state_.y, current_state_.chi);
      const auto new_projection = new_path.project(
          current_state_.x, current_state_.y, current_state_.chi);
      double old_x = 0.0, old_y = 0.0, old_heading = 0.0, old_kappa = 0.0;
      double new_x = 0.0, new_y = 0.0, new_heading = 0.0, new_kappa = 0.0;
      reference_path_->evaluate(old_projection.s, old_x, old_y, old_heading, old_kappa);
      new_path.evaluate(new_projection.s, new_x, new_y, new_heading, new_kappa);
      const double reference_position_difference = std::hypot(old_x - new_x, old_y - new_y);
      const double heading_difference = std::abs(wrap_angle(old_heading - new_heading));
      const double curvature_difference = std::abs(old_kappa - new_kappa);
      // Compare the two reference geometries, not the vehicle's lateral offset.
      // During intentional obstacle avoidance the vehicle may be several metres
      // from both windows while the windows still represent the same route.
      return reference_position_difference <= 0.50 &&
             heading_difference <= 15.0 * kPi / 180.0 &&
             curvature_difference <= 0.05;
    } catch (const std::exception&) {
      return false;
    }
  }

  static std::uint64_t costmap_fingerprint(
      const nav_msgs::msg::OccupancyGrid& message) {
    std::uint64_t hash = 1469598103934665603ULL;
    const auto mix = [&hash](std::uint64_t value) {
      hash ^= value;
      hash *= 1099511628211ULL;
    };
    mix(message.info.width);
    mix(message.info.height);
    mix(static_cast<std::uint64_t>(std::llround(message.info.resolution * 1.0e9)));
    mix(static_cast<std::uint64_t>(std::llround(
        message.info.origin.position.x * 1.0e6)));
    mix(static_cast<std::uint64_t>(std::llround(
        message.info.origin.position.y * 1.0e6)));
    for (const auto value : message.data) {
      mix(static_cast<std::uint8_t>(value));
    }
    return hash;
  }

  void path_callback(const ReferencePathMsg::SharedPtr msg) {
    try {
      if (msg->x.size() < 4 || msg->x.size() != msg->y.size() ||
          msg->x.size() != msg->yaw.size() || msg->x.size() != msg->curvature.size() ||
          msg->x.size() != msg->mode.size()) {
        throw std::invalid_argument("reference path arrays must have equal length >= 4");
      }
      if (msg->mode.front() > static_cast<std::uint8_t>(DriveMode::Right) ||
          !std::all_of(msg->mode.begin(), msg->mode.end(),
                       [&](std::uint8_t value) { return value == msg->mode.front(); })) {
        throw std::invalid_argument("each local reference must contain one valid drive mode");
      }
      const auto reference_mode = static_cast<DriveMode>(msg->mode.front());
      auto path = std::make_shared<ReferencePath>(
          cumulative_arc_length(msg->x, msg->y), msg->x, msg->y, msg->yaw, msg->curvature);
      bool identical = false;
      bool hard_change = false;
      {
        std::lock_guard<std::mutex> lock(input_mutex_);
        identical = reference_path_ && reference_mode == reference_mode_ &&
                    same_reference_path(*reference_path_, *path);
        if (!identical) {
          hard_change = !is_soft_reference_continuation_locked(*path, reference_mode);
          reference_path_ = std::move(path);
          reference_mode_ = reference_mode;
          path_frame_ = msg->header.frame_id.empty() ? "map" : msg->header.frame_id;
          received_path_ = true;
          ++path_revision_;
          if (hard_change) ++command_revision_;
        }
      }
      if (identical) return;
      // A rolling local window is a soft update: keep the already validated
      // pending/active plan.  A route or mode discontinuity remains hard.
      if (hard_change) invalidate_motion_plan();
      request_replan(hard_change ? "REFERENCE_PATH_HARD" : "REFERENCE_PATH_SOFT",
                     hard_change);
    } catch (const std::exception& error) {
      RCLCPP_ERROR(get_logger(), "Reference path rejected: %s", error.what());
    }
  }

  void costmap_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    try {
      const int width = static_cast<int>(msg->info.width);
      const int height = static_cast<int>(msg->info.height);
      if (width < 2 || height < 2 || static_cast<int>(msg->data.size()) != width * height)
        throw std::invalid_argument("invalid OccupancyGrid dimensions");
      const auto& q = msg->info.origin.orientation;
      if (std::abs(quaternion_to_yaw(q.x, q.y, q.z, q.w)) > 1.0e-6)
        throw std::invalid_argument("rotated OccupancyGrid origins are unsupported");
      const auto fingerprint = costmap_fingerprint(*msg);
      {
        std::lock_guard<std::mutex> lock(input_mutex_);
        if (received_costmap_ && fingerprint == costmap_fingerprint_) return;
      }
      std::vector<std::int8_t> data(msg->data.begin(), msg->data.end());
      auto map = std::make_shared<Costmap2D>(
          std::move(data), width, height, msg->info.resolution,
          msg->info.origin.position.x, msg->info.origin.position.y);
      {
        std::lock_guard<std::mutex> lock(input_mutex_);
        costmap_ = std::move(map);
        costmap_frame_ = msg->header.frame_id;
        costmap_fingerprint_ = fingerprint;
        received_costmap_ = true;
        ++structural_revision_;
      }
      invalidate_pending_plan();
      request_replan("COSTMAP_CHANGED", true);
    } catch (const std::exception& error) {
      RCLCPP_ERROR(get_logger(), "Costmap rejected: %s", error.what());
    }
  }

  void target_speed_callback(const std_msgs::msg::Float64::SharedPtr msg) {
    if (!std::isfinite(msg->data) || msg->data < 0.0) {
      RCLCPP_ERROR(get_logger(), "target_speed must be finite and non-negative");
      return;
    }
    bool changed = false;
    {
      std::lock_guard<std::mutex> lock(input_mutex_);
      changed = !received_target_speed_ || std::abs(target_speed_ - msg->data) > 1.0e-9;
      target_speed_ = msg->data;
      received_target_speed_ = true;
      if (changed) ++command_revision_;
    }
    if (!changed) return;
    invalidate_pending_plan();
    request_replan("TARGET_SPEED_CHANGED", true);
  }

  void requested_mode_callback(const std_msgs::msg::UInt8::SharedPtr msg) {
    if (msg->data > static_cast<std::uint8_t>(DriveMode::Right)) {
      RCLCPP_ERROR(get_logger(), "unsupported requested drive mode: %u", msg->data);
      return;
    }
    bool changed;
    {
      std::lock_guard<std::mutex> lock(input_mutex_);
      changed = mode_supervisor_.set_requested_mode(
          static_cast<DriveMode>(msg->data));
      received_requested_mode_ = true;
      if (changed) ++command_revision_;
    }
    if (!changed) return;
    invalidate_motion_plan();
    request_replan("MODE_REQUEST_CHANGED", true);
  }

  void vehicle_mode_callback(const DriveModeStateMsg::SharedPtr msg) {
    if (msg->current_mode > static_cast<std::uint8_t>(DriveMode::Right) ||
        msg->requested_mode > static_cast<std::uint8_t>(DriveMode::Right)) {
      RCLCPP_ERROR(get_logger(), "invalid vehicle drive-mode feedback");
      return;
    }
    bool confirmed_changed;
    bool became_ready;
    {
      std::lock_guard<std::mutex> lock(input_mutex_);
      const bool was_ready = mode_supervisor_.ready();
      confirmed_changed = mode_supervisor_.update_vehicle_feedback(
          static_cast<DriveMode>(msg->current_mode),
          static_cast<DriveMode>(msg->requested_mode),
          msg->transition_in_progress, msg->transition_complete);
      received_vehicle_mode_ = true;
      became_ready = !was_ready && mode_supervisor_.ready();
      if (confirmed_changed) {
        ++mode_revision_;
        if (current_state_.speed <= mode_change_stop_speed_) {
          current_state_.chi = motion_heading_from_body_yaw(
              current_body_yaw_, static_cast<DriveMode>(msg->current_mode));
          current_state_.acceleration = 0.0;
          current_state_.motion_heading_rate = 0.0;
          last_motion_chi_ = current_state_.chi;
        }
      }
    }
    if (confirmed_changed) {
      invalidate_motion_plan();
      {
        std::lock_guard<std::mutex> lock(terminal_hold_mutex_);
        terminal_hold_.reset();
      }
    }
    if (confirmed_changed || became_ready) {
      request_replan("VEHICLE_MODE_CONFIRMED", true);
    }
  }

  void ensure_planner(const InputSnapshot& input) {
    const PlanningRevisionState built{0, planner_path_revision_,
                                      planner_structural_revision_, 0,
                                      planner_mode_revision_};
    if (planner_ && !planner_rebuild_required(built, revision_state(input))) return;
    const bool mode_changed = planner_ && planner_mode_revision_ != input.mode_revision;
    const auto old_cap = (!mode_changed && planner_)
        ? planner_->feasible_speed_cap() : std::nullopt;
    const int old_count = (!mode_changed && planner_)
        ? planner_->feasible_speed_cap_success_count() : 0;
    const auto old_hint = (!mode_changed && planner_)
        ? planner_->lateral_target_hint() : std::nullopt;
    const auto continuity = (!mode_changed && planner_)
        ? std::optional<PlannerContinuityState>(
              planner_->export_continuity_state(input.state))
        : std::nullopt;
    planner_ = std::make_unique<PathVelocityPlanner>(
        config_, *input.path, *input.costmap);
    planner_->set_feasible_speed_cap_hint(old_cap, old_count);
    if (continuity) {
      planner_->import_continuity_state(*continuity, input.state);
    } else {
      planner_->set_lateral_target_hint(old_hint);
    }
    planner_structural_revision_ = input.structural_revision;
    planner_path_revision_ = input.path_revision;
    planner_mode_revision_ = input.mode_revision;
  }

  double allocation_min_clearance(const AllocationResult& allocation,
                                  const Costmap2D& costmap) const {
    const double radius = circumscribed_radius(
        config_.vehicle.length, config_.vehicle.width,
        config_.vehicle.footprint_margin);
    double minimum = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < allocation.trajectory.x.size(); ++i) {
      minimum = std::min(
          minimum,
          costmap.clearance_single_circle(
              allocation.trajectory.x[i], allocation.trajectory.y[i], radius));
    }
    return minimum;
  }

  void planning_callback() {
    const auto input = snapshot();
    if (!input) {
      publish_status("WAITING_FOR_INPUTS", 0.0, -1);
      return;
    }
    if (!input->mode_ready) {
      DriveModeControlState mode_state;
      {
        std::lock_guard<std::mutex> lock(input_mutex_);
        mode_state = mode_supervisor_.state(current_state_.speed, mode_change_stop_speed_);
      }
      publish_status(drive_mode_control_state_name(mode_state), 0.0, -1);
      return;
    }
    if (input->requested_mode != input->drive_mode ||
        input->reference_mode != input->drive_mode) {
      publish_status("REFERENCE_OR_REQUEST_MODE_MISMATCH", 0.0, -1);
      return;
    }
    bool has_plan;
    {
      std::lock_guard<std::mutex> lock(execution_mutex_);
      has_plan = static_cast<bool>(active_plan_) || static_cast<bool>(pending_plan_);
      if (pending_plan_ && now_ns() < pending_plan_->start_ns) return;
    }
    const auto token = scheduler_.begin_if_due(now_ns(), has_plan, false);
    if (!token) return;

    try {
      ensure_planner(*input);
      const auto start_wall = std::chrono::steady_clock::now();
      const auto scheduled_start = align_time_ns(
          now_ns() + static_cast<std::int64_t>(
              std::llround(handover_timing_.recommended_lead_sec() * 1.0e9)),
          command_dt_);
      std::shared_ptr<const ExecutablePlan> active;
      {
        std::lock_guard<std::mutex> lock(execution_mutex_);
        active = active_plan_;
      }
      const auto handover = predict_handover_state(
          input->state, input->body_yaw, input->state_time_ns, scheduled_start,
          active ? std::optional<std::int64_t>(active->start_ns) : std::nullopt,
          active ? &active->allocation : nullptr,
          active ? &active->result.trajectory.actions : nullptr,
          command_dt_, std::min(std::abs(config_.constraints.a_min),
                                config_.longitudinal.service_deceleration),
          std::min(config_.constraints.jerk_max, config_.longitudinal.comfort_jerk));

      if (active) {
        const auto handover_projection = planner_->path().project(
            handover.state.x, handover.state.y, handover.state.chi);
        const double terminal_goal_s = std::max(
            planner_->path().s_max() - config_.simulation.path_end_margin
                - config_.longitudinal.stop_target_offset,
            0.0);
        const double remaining = terminal_goal_s - handover_projection.s;
        if (std::abs(remaining) <= 0.20 && handover.state.speed <= 0.25) {
          publish_status("TERMINAL_ACTIVE_PLAN_FINISHING", 0.0,
                         static_cast<std::int64_t>(active->plan_id));
          return;
        }
      }

      std::optional<PlanResult> selected_result;
      std::optional<AllocationSelectionResult> selected_allocation;
      int outer_speed_attempts = 1;
      int path_replans = 0;
      const OrientedFootprintConfig footprint_config{
          footprint_circle_count_, footprint_translation_step_m_,
          footprint_yaw_step_deg_ * kPi / 180.0};

      auto attempt_result = planner_->plan(
          handover.state, handover.last_action,
          PlanningCommand{input->target_speed, input->drive_mode});
      const bool infeasible =
          attempt_result.diagnostics.status.find("EMERGENCY") != std::string::npos ||
          attempt_result.diagnostics.status.find("INFEASIBLE") != std::string::npos;
      if (!infeasible) {
        auto allocation_selection = allocate_with_oriented_collision_search(
            attempt_result.motion, *input->costmap, config_.vehicle, config_.cost,
            handover.allocator_state, footprint_config);
        std::vector<int> excluded_candidate_ids;
        std::vector<double> excluded_lateral_targets;
        const int maximum_path_replans = std::max(
            0, config_.lateral.allocation_path_replan_max_attempts);
        for (int attempt = 0;
             !allocation_selection.collision.collision_free
                 && attempt_result.selected_path
                 && attempt < maximum_path_replans;
             ++attempt) {
          excluded_candidate_ids.push_back(attempt_result.selected_path->candidate_id);
          excluded_lateral_targets.push_back(attempt_result.selected_path->n_target);
          planner_->set_one_shot_excluded_paths(
              excluded_candidate_ids, excluded_lateral_targets);
          ++path_replans;
          auto replanned_result = planner_->plan(
              handover.state, handover.last_action,
              PlanningCommand{input->target_speed, input->drive_mode});
          const bool replanned_infeasible =
              replanned_result.diagnostics.status.find("EMERGENCY") != std::string::npos ||
              replanned_result.diagnostics.status.find("INFEASIBLE") != std::string::npos;
          if (replanned_infeasible) break;
          auto replanned_allocation = allocate_with_oriented_collision_search(
              replanned_result.motion, *input->costmap,
              config_.vehicle, config_.cost,
              handover.allocator_state, footprint_config);
          attempt_result = std::move(replanned_result);
          allocation_selection = std::move(replanned_allocation);
        }
        if (allocation_selection.collision.collision_free
            && path_replans > 0) {
          attempt_result.diagnostics.status += "_EXCLUDED_PATH_REPLAN";
        }
        if (allocation_selection.collision.collision_free) {
          planner_->record_allocation_success();
          attempt_result.diagnostics.requested_target_speed = input->target_speed;
          selected_result = std::move(attempt_result);
          selected_allocation = std::move(allocation_selection);
        }
      }

      if (!selected_result || !selected_allocation) {
        engage_forced_safety_stop();
        scheduler_.request(now_ns(), input->input_revision,
                           input->structural_revision,
                           "NO_SAFE_PLAN_RETRY", false);
        publish_status("NO_SAFE_PLAN_SAFETY_STOP", 0.0,
                       active ? static_cast<std::int64_t>(active->plan_id) : -1);
        return;
      }
      auto result = std::move(*selected_result);
      auto allocation_selection = std::move(*selected_allocation);
      auto allocation = std::move(allocation_selection.allocation);
      const double allocation_clearance =
          allocation_selection.collision.minimum_clearance;
      const auto end_wall = std::chrono::steady_clock::now();
      const double elapsed_sec = std::chrono::duration<double>(end_wall - start_wall).count();
      const auto ready_ns = now_ns();
      if (!snapshot_is_current(*input)) {
        // Every input callback already queued the latest revision. Do not write
        // the stale revision back into the latest-only scheduler.
        publish_status("STALE_PLAN_DISCARDED", elapsed_sec * 1000.0, -1);
        return;
      }
      const bool late = ready_ns >= scheduled_start;
      handover_timing_.record(elapsed_sec, late);
      if (late) {
        scheduler_.request(ready_ns, input->input_revision, input->structural_revision,
                           "LATE_PLAN_RETRY", false);
        publish_status("LATE_PLAN_DISCARDED", elapsed_sec * 1000.0, -1);
        return;
      }
      auto executable = std::make_shared<ExecutablePlan>();
      executable->result = std::move(result);
      executable->allocation = std::move(allocation);
      executable->start_ns = scheduled_start;
      executable->ready_ns = ready_ns;
      executable->plan_id = ++plan_id_counter_;
      executable->structural_revision = input->structural_revision;
      executable->frame_id = input->frame_id;
      executable->allocation_min_clearance = allocation_clearance;
      executable->allocation_profile = allocation_profile_name(allocation_selection.profile);
      executable->allocation_candidates_evaluated = allocation_selection.candidates_evaluated;
      executable->footprint_circle_count = footprint_config.circle_count;
      executable->outer_speed_attempts = outer_speed_attempts;
      executable->path_replans = path_replans;
      executable->compute_ms = elapsed_sec * 1000.0;
      bool registration_stale = false;
      {
        // Register the plan atomically with respect to input revision updates
        // and pending-plan invalidation callbacks.
        std::scoped_lock lock(input_mutex_, execution_mutex_);
        registration_stale = !plan_registration_is_current(
            revision_state(*input), current_revision_state_locked());
        if (!registration_stale) pending_plan_ = executable;
      }
      if (registration_stale) {
        publish_status("STALE_PLAN_DISCARDED", elapsed_sec * 1000.0, -1);
        return;
      }
      publish_status("PENDING_PLAN_READY", elapsed_sec * 1000.0,
                     executable->result.diagnostics.selected_candidate_id);
    } catch (const std::exception& error) {
      RCLCPP_ERROR(get_logger(), "Planning failed: %s", error.what());
      request_replan("PLANNING_RETRY", false);
      publish_status(std::string("PLANNING_FAILED: ") + error.what(), 0.0, -1);
    }
  }

  void engage_forced_safety_stop() {
    std::lock_guard<std::mutex> lock(execution_mutex_);
    if (!safety_stop_) {
      const BodyCommand initial = last_command_.value_or(BodyCommand{});
      safety_stop_ = JerkLimitedSafetyStop::from_command(
          initial,
          std::min(std::abs(config_.constraints.a_min),
                   config_.longitudinal.service_deceleration),
          std::min(config_.constraints.jerk_max,
                   config_.longitudinal.comfort_jerk));
    }
    forced_safety_stop_ = true;
    pending_plan_.reset();
  }

  std::shared_ptr<const ExecutablePlan> activate_pending_locked(
      std::int64_t stamp_ns) {
    if (!pending_plan_ || stamp_ns < pending_plan_->start_ns) return nullptr;
    active_plan_ = pending_plan_;
    pending_plan_.reset();
    safety_stop_.reset();
    forced_safety_stop_ = false;
    scheduler_.mark_plan_activated(stamp_ns);
    return active_plan_;
  }

  void publish_mode_command_if_due(std::int64_t stamp_ns) {
    std::optional<DriveMode> requested;
    bool should_publish = false;
    {
      std::lock_guard<std::mutex> lock(input_mutex_);
      requested = mode_supervisor_.requested_mode();
      should_publish = mode_supervisor_.should_publish_command(
          current_state_.speed, mode_change_stop_speed_);
    }
    const auto period_ns = static_cast<std::int64_t>(
        std::llround(mode_command_period_sec_ * 1.0e9));
    if (!requested || !should_publish ||
        (last_mode_command_ns_ > 0 && stamp_ns - last_mode_command_ns_ < period_ns)) {
      return;
    }
    std_msgs::msg::UInt8 message;
    message.data = static_cast<std::uint8_t>(*requested);
    mode_command_pub_->publish(message);
    last_mode_command_ns_ = stamp_ns;
  }

  void command_callback() {
    const auto stamp_ns = now_ns();
    DriveModeControlState mode_state;
    bool mode_ready;
    double measured_speed;
    {
      std::lock_guard<std::mutex> lock(input_mutex_);
      measured_speed = current_state_.speed;
      mode_ready = mode_supervisor_.ready();
      mode_state = mode_supervisor_.state(measured_speed, mode_change_stop_speed_);
    }

    if (!mode_ready) {
      publish_mode_command_if_due(stamp_ns);
      std::optional<BodyCommand> command;
      {
        std::lock_guard<std::mutex> lock(execution_mutex_);
        pending_plan_.reset();
        active_plan_.reset();
        safety_stop_.reset();
        forced_safety_stop_ = false;
        if (mode_state == DriveModeControlState::StoppingForChange && last_command_) {
          if (!mode_stop_) {
            mode_stop_ = JerkLimitedSafetyStop::from_command(
                *last_command_,
                std::min(std::abs(config_.constraints.a_min),
                         config_.longitudinal.service_deceleration),
                std::min(config_.constraints.jerk_max,
                         config_.longitudinal.comfort_jerk));
          }
          command = mode_stop_->sample_and_advance(command_dt_);
        } else {
          mode_stop_.reset();
          command = zero_command();
        }
        last_command_ = command;
      }
      publish_execution_state_if_changed(
          mode_state == DriveModeControlState::StoppingForChange
              ? "MODE_STOP" : "MODE_WAIT");
      publish_command(*command, 0, stamp_ns);
      return;
    }

    std::optional<BodyCommand> command;
    std::shared_ptr<const ExecutablePlan> active;
    std::shared_ptr<const ExecutablePlan> activated;
    bool using_safety_stop = false;
    {
      std::lock_guard<std::mutex> lock(execution_mutex_);
      mode_stop_.reset();
      activated = activate_pending_locked(stamp_ns);
      active = active_plan_;
      if (forced_safety_stop_) {
        if (!safety_stop_) {
          safety_stop_ = JerkLimitedSafetyStop::from_command(
              last_command_.value_or(BodyCommand{}),
              std::min(std::abs(config_.constraints.a_min),
                       config_.longitudinal.service_deceleration),
              std::min(config_.constraints.jerk_max,
                       config_.longitudinal.comfort_jerk));
        }
        command = safety_stop_->sample_and_advance(command_dt_);
        using_safety_stop = true;
      } else if (active) {
        const double elapsed = std::max(0.0, 1.0e-9 * (stamp_ns - active->start_ns));
        if (elapsed <= active->allocation.trajectory.t.back() + 1.0e-12) {
          command = sample_body_command(active->allocation,
                                        active->result.trajectory.actions,
                                        elapsed, command_dt_);
        } else {
          if (!safety_stop_) {
            const auto final_command = sample_body_command(
                active->allocation, active->result.trajectory.actions,
                active->allocation.trajectory.t.back(), command_dt_);
            safety_stop_ = JerkLimitedSafetyStop::from_command(
                final_command,
                std::min(std::abs(config_.constraints.a_min),
                         config_.longitudinal.service_deceleration),
                std::min(config_.constraints.jerk_max,
                         config_.longitudinal.comfort_jerk));
          }
          command = safety_stop_->sample_and_advance(command_dt_);
          using_safety_stop = true;
        }
      }
      if (command) last_command_ = command;
    }

    if (activated) {
      publish_trajectory(*activated);
      publish_plan_status(*activated, activated->compute_ms);
    }
    const bool terminal_hold = terminal_hold_active();
    if (terminal_hold) command = zero_command();
    if (!command) command = zero_command();
    const std::string execution_state = terminal_hold ? "TERMINAL_HOLD"
        : (using_safety_stop ? "SAFETY_STOP"
                            : (active ? "ACTIVE_PLAN" : "IDLE"));
    publish_execution_state_if_changed(execution_state);
    publish_command(*command, active ? active->plan_id : 0, stamp_ns);
  }

  void publish_execution_state_if_changed(const std::string& state) {
    {
      std::lock_guard<std::mutex> lock(execution_state_mutex_);
      if (state == execution_state_) return;
      execution_state_ = state;
    }
    std_msgs::msg::String message;
    message.data = state;
    execution_state_pub_->publish(message);
  }

  std::string current_execution_state() const {
    std::lock_guard<std::mutex> lock(execution_state_mutex_);
    return execution_state_;
  }

  bool terminal_hold_active() {
    const auto input = snapshot();
    if (!input) return false;
    const auto projection = input->path->project(
        input->state.x, input->state.y, input->state.chi);
    const double goal = std::max(
        input->path->s_max() - config_.simulation.path_end_margin,
        input->path->s_min());
    std::lock_guard<std::mutex> lock(terminal_hold_mutex_);
    terminal_hold_.set_goal(goal, input->drive_mode);
    return terminal_hold_.update(
        goal - projection.s, input->state.speed, input->state.acceleration);
  }

  bool terminal_hold_latched() const {
    std::lock_guard<std::mutex> lock(terminal_hold_mutex_);
    return terminal_hold_.active();
  }

  std::uint64_t current_plan_id() const {
    std::lock_guard<std::mutex> lock(execution_mutex_);
    return active_plan_ ? active_plan_->plan_id : 0;
  }

  static BodyCommand zero_command() {
    return BodyCommand{};
  }

  void publish_command(const BodyCommand& command, std::uint64_t plan_id,
                       std::int64_t stamp_ns) {
    geometry_msgs::msg::Twist twist;
    twist.linear.x = command.vx;
    twist.linear.y = command.vy;
    twist.angular.z = command.yaw_rate;
    cmd_pub_->publish(twist);

    geometry_msgs::msg::TwistStamped stamped;
    stamped.header.stamp = rclcpp::Time(stamp_ns);
    stamped.header.frame_id = "base_link";
    stamped.twist = twist;
    cmd_stamped_pub_->publish(stamped);

    ExecutedCommandMsg executed;
    executed.header = stamped.header;
    executed.plan_id = plan_id;
    executed.interval_index = static_cast<std::uint32_t>(command.action_index);
    executed.trajectory_time = command.trajectory_time;
    executed.vx = command.vx;
    executed.vy = command.vy;
    executed.yaw_rate = command.yaw_rate;
    executed.planned_speed = command.planned_speed;
    executed.planned_acceleration = command.planned_acceleration;
    executed.planned_jerk = command.planned_jerk;
    executed.motion_heading = command.motion_heading;
    executed.motion_curvature = command.motion_curvature;
    executed.motion_heading_rate = command.motion_heading_rate;
    executed.motion_heading_acceleration = command.planned_heading_acceleration;
    executed.beta = command.beta;
    executed.beta_rate = command.beta_rate;
    executed.yaw_acceleration = command.yaw_acceleration;
    executed.segment_start_x = command.segment_start_x;
    executed.segment_start_y = command.segment_start_y;
    executed.segment_start_heading = command.segment_start_heading;
    executed.segment_end_x = command.segment_end_x;
    executed.segment_end_y = command.segment_end_y;
    executed.segment_end_heading = command.segment_end_heading;
    executed_pub_->publish(executed);
  }

  void publish_trajectory(const ExecutablePlan& plan) {
    const auto stamp = get_clock()->now();
    nav_msgs::msg::Path path_message;
    path_message.header.stamp = stamp;
    path_message.header.frame_id = plan.frame_id;
    ReferencePathMsg data_message;
    data_message.header = path_message.header;
    data_message.closed_loop = false;
    const auto& motion = plan.result.motion;
    path_message.poses.reserve(motion.x.size());
    data_message.x.reserve(motion.x.size());
    data_message.y.reserve(motion.y.size());
    data_message.yaw.reserve(motion.chi.size());
    data_message.curvature.reserve(motion.kappa.size());
    data_message.mode.reserve(motion.drive_mode.size());
    for (std::size_t i = 0; i < motion.x.size(); ++i) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path_message.header;
      pose.pose.position.x = motion.x[i];
      pose.pose.position.y = motion.y[i];
      pose.pose.orientation = yaw_to_quaternion(motion.chi[i]);
      path_message.poses.push_back(pose);
      data_message.x.push_back(motion.x[i]);
      data_message.y.push_back(motion.y[i]);
      data_message.yaw.push_back(motion.chi[i]);
      data_message.curvature.push_back(motion.kappa[i]);
      data_message.mode.push_back(static_cast<std::uint8_t>(motion.drive_mode[i]));
    }
    trajectory_pub_->publish(path_message);
    trajectory_data_pub_->publish(data_message);
  }

  ModeStatusSnapshot mode_status_snapshot() const {
    std::lock_guard<std::mutex> lock(input_mutex_);
    ModeStatusSnapshot status;
    if (const auto requested = mode_supervisor_.requested_mode()) {
      status.requested_mode = static_cast<int>(*requested);
    }
    if (const auto actual = mode_supervisor_.current_mode()) {
      status.actual_mode = static_cast<int>(*actual);
    }
    if (const auto vehicle_requested = mode_supervisor_.vehicle_requested_mode()) {
      status.vehicle_requested_mode = static_cast<int>(*vehicle_requested);
    }
    status.reference_mode = received_path_ ? static_cast<int>(reference_mode_) : -1;
    status.ready = mode_supervisor_.ready();
    status.transition_in_progress = mode_supervisor_.transition_in_progress();
    status.transition_complete = mode_supervisor_.transition_complete();
    return status;
  }

  void publish_plan_status(const ExecutablePlan& plan, double compute_ms) {
    const auto& diagnostics = plan.result.diagnostics;
    const auto& trajectory = plan.result.trajectory;
    const auto mode_status = mode_status_snapshot();
    std_msgs::msg::String message;
    std::ostringstream stream;
    const double selected_n_target =
        std::isfinite(diagnostics.selected_n_target)
            ? diagnostics.selected_n_target
            : 0.0;
    const double coarse_clearance =
        std::isfinite(trajectory.coarse_min_clearance)
            ? trajectory.coarse_min_clearance
            : 0.0;
    const double precise_clearance =
        std::isfinite(trajectory.precise_min_clearance)
            ? trajectory.precise_min_clearance
            : 0.0;
    const double allocation_clearance =
        std::isfinite(plan.allocation_min_clearance)
            ? plan.allocation_min_clearance
            : 0.0;
    const double terminal_goal_offset =
        std::isfinite(diagnostics.terminal_goal_offset)
            ? diagnostics.terminal_goal_offset
            : 0.0;
    const double terminal_goal_clearance =
        std::isfinite(diagnostics.terminal_goal_clearance)
            ? diagnostics.terminal_goal_clearance
            : 0.0;
    stream << std::fixed << std::setprecision(6)
           << "{\"backend\":\"CPP_NATIVE\""
           << ",\"allocation_profile\":\"" << plan.allocation_profile << "\""
           << ",\"state\":\"RUNNING\""
           << ",\"block_reason\":\"NONE\""
           << ",\"mode_control\":{"
           << "\"requested_mode\":" << mode_status.requested_mode
           << ",\"actual_mode\":" << mode_status.actual_mode
           << ",\"vehicle_requested_mode\":"
           << mode_status.vehicle_requested_mode
           << ",\"reference_mode\":" << mode_status.reference_mode
           << ",\"ready\":" << (mode_status.ready ? "true" : "false")
           << ",\"transition_in_progress\":"
           << (mode_status.transition_in_progress ? "true" : "false")
           << ",\"transition_complete\":"
           << (mode_status.transition_complete ? "true" : "false") << "}"
           << ",\"execution\":{"
           << "\"current_plan_id\":" << plan.plan_id
           << ",\"state\":\"" << current_execution_state() << "\"}"
           << ",\"plan\":{"
           << "\"status\":\"" << diagnostics.status << "\""
           << ",\"selected_candidate_id\":"
           << diagnostics.selected_candidate_id
           << ",\"selected_n_target\":" << selected_n_target
           << ",\"lateral_selection_basis\":\""
           << diagnostics.lateral_selection_basis << "\""
           << ",\"reference_center_lock_active\":"
           << (diagnostics.reference_center_lock_active ? "true" : "false")
           << ",\"bottleneck_limited\":"
           << (diagnostics.bottleneck_limited ? "true" : "false")
           << ",\"decision_horizon_length\":"
           << diagnostics.decision_horizon_length
           << ",\"compute_time_ms\":" << compute_ms
           << ",\"total_compute_time_ms\":" << compute_ms
           << ",\"num_safe_paths\":" << diagnostics.number_of_safe_paths
           << ",\"num_spatial_candidates\":"
           << diagnostics.number_of_lateral_paths
           << ",\"requested_target_speed\":"
           << diagnostics.requested_target_speed
           << ",\"applied_target_speed\":"
           << diagnostics.applied_target_speed
           << ",\"obstacle_speed_cap_active\":"
           << (diagnostics.obstacle_speed_cap_active ? "true" : "false")
           << ",\"speed_search_attempts\":"
           << diagnostics.speed_search_attempts
           << ",\"terminal_mode_active\":"
           << (diagnostics.terminal_mode_active ? "true" : "false")
           << ",\"terminal_constraint_active\":"
           << (diagnostics.terminal_constraint_active ? "true" : "false")
           << ",\"terminal_safe_region_active\":"
           << (diagnostics.terminal_safe_region_active ? "true" : "false")
           << ",\"terminal_goal_region_safe\":"
           << (diagnostics.terminal_goal_region_safe ? "true" : "false")
           << ",\"terminal_goal_offset\":"
           << terminal_goal_offset
           << ",\"terminal_goal_clearance\":"
           << terminal_goal_clearance
           << ",\"coarse_min_clearance\":"
           << coarse_clearance
           << ",\"precise_min_clearance\":"
           << precise_clearance
           << ",\"allocation_min_clearance\":"
           << allocation_clearance
           << ",\"spatial_collision_screen_passed\":true"
           << ",\"temporal_static_collision_check_performed\":false"
           << ",\"allocation_collision_free\":true"
           << ",\"short_path_fallback_active\":"
           << (diagnostics.short_path_fallback_active ? "true" : "false")
           << ",\"short_path_length\":" << diagnostics.short_path_length
           << ",\"excluded_candidate_id\":" << diagnostics.excluded_candidate_id
           << ",\"allocation_candidates_evaluated\":"
           << plan.allocation_candidates_evaluated
           << ",\"footprint_circle_count\":" << plan.footprint_circle_count
           << ",\"outer_speed_attempts\":" << plan.outer_speed_attempts
           << ",\"path_replans\":" << plan.path_replans << "}"
           << ",\"timing\":{"
           << "\"deadline_ms\":100.0"
           << ",\"handover_lead_sec\":"
           << handover_timing_.recommended_lead_sec() << "}"
           << ",\"terminal_hold\":{"
           << "\"latched\":" << (terminal_hold_latched() ? "true" : "false")
           << "}}";
    message.data = stream.str();
    status_pub_->publish(message);
  }

  void publish_status(const std::string& status, double compute_ms, int candidate_id) {
    const auto mode_status = mode_status_snapshot();
    std_msgs::msg::String message;
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(3)
           << "{\"backend\":\"CPP_NATIVE\""
           << ",\"allocation_profile\":\"LATERAL_PRIORITY\""
           << ",\"state\":\"" << status << "\""
           << ",\"block_reason\":\"" << status << "\""
           << ",\"mode_control\":{"
           << "\"requested_mode\":" << mode_status.requested_mode
           << ",\"actual_mode\":" << mode_status.actual_mode
           << ",\"vehicle_requested_mode\":"
           << mode_status.vehicle_requested_mode
           << ",\"reference_mode\":" << mode_status.reference_mode
           << ",\"ready\":" << (mode_status.ready ? "true" : "false")
           << ",\"transition_in_progress\":"
           << (mode_status.transition_in_progress ? "true" : "false")
           << ",\"transition_complete\":"
           << (mode_status.transition_complete ? "true" : "false") << "}"
           << ",\"execution\":{\"current_plan_id\":"
           << current_plan_id()
           << ",\"state\":\"" << current_execution_state() << "\"}"
           << ",\"plan\":{\"selected_candidate_id\":"
           << candidate_id
           << ",\"total_compute_time_ms\":" << compute_ms << "}"
           << ",\"timing\":{\"deadline_ms\":100.0"
           << ",\"handover_lead_sec\":"
           << handover_timing_.recommended_lead_sec() << "}"
           << ",\"terminal_hold\":{\"latched\":"
           << (terminal_hold_latched() ? "true" : "false") << "}}";
    message.data = stream.str();
    status_pub_->publish(message);
  }

  mutable std::mutex input_mutex_;
  mutable std::mutex execution_mutex_;
  mutable std::mutex terminal_hold_mutex_;
  mutable std::mutex execution_state_mutex_;
  rclcpp::CallbackGroup::SharedPtr input_group_;
  rclcpp::CallbackGroup::SharedPtr planning_group_;
  rclcpp::CallbackGroup::SharedPtr execution_group_;

  LatestOnlyPlanningScheduler scheduler_;
  AdaptiveHandoverTiming handover_timing_;
  TerminalHoldLatch terminal_hold_;
  EnvConfig config_;
  std::unique_ptr<PathVelocityPlanner> planner_;
  std::uint64_t planner_structural_revision_{std::numeric_limits<std::uint64_t>::max()};
  std::uint64_t planner_path_revision_{std::numeric_limits<std::uint64_t>::max()};
  std::uint64_t planner_mode_revision_{std::numeric_limits<std::uint64_t>::max()};
  DriveModeSupervisor mode_supervisor_;

  std::shared_ptr<const ReferencePath> reference_path_;
  std::shared_ptr<const Costmap2D> costmap_;
  PlannerState current_state_;
  double current_body_yaw_{0.0};
  std::int64_t current_state_time_ns_{0};
  double target_speed_{0.0};
  DriveMode reference_mode_{DriveMode::Forward};
  std::optional<double> last_motion_chi_;
  std::string path_frame_{"map"};
  std::string costmap_frame_;
  std::string odom_frame_;
  bool received_odom_{false};
  bool received_path_{false};
  bool received_costmap_{false};
  bool received_target_speed_{false};
  bool received_requested_mode_{false};
  bool received_vehicle_mode_{false};
  std::uint64_t input_revision_{0};
  std::uint64_t path_revision_{0};
  std::uint64_t structural_revision_{0};
  std::uint64_t command_revision_{0};
  std::uint64_t mode_revision_{0};
  std::uint64_t costmap_fingerprint_{0};

  std::shared_ptr<const ExecutablePlan> active_plan_;
  std::shared_ptr<const ExecutablePlan> pending_plan_;
  std::optional<BodyCommand> last_command_;
  std::optional<JerkLimitedSafetyStop> safety_stop_;
  std::optional<JerkLimitedSafetyStop> mode_stop_;
  bool forced_safety_stop_{false};
  std::uint64_t plan_id_counter_{0};
  std::string execution_state_{"STARTUP"};

  double trajectory_knot_dt_{0.10};
  double command_frequency_hz_{100.0};
  double scheduler_frequency_hz_{100.0};
  double command_dt_{0.01};
  double mode_change_stop_speed_{0.03};
  double mode_command_period_sec_{0.25};
  int footprint_circle_count_{3};
  double footprint_translation_step_m_{0.20};
  double footprint_yaw_step_deg_{2.0};
  std::int64_t last_mode_command_ns_{0};

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_stamped_pub_;
  rclcpp::Publisher<ExecutedCommandMsg>::SharedPtr executed_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr execution_state_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<ReferencePathMsg>::SharedPtr trajectory_data_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr mode_command_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<ReferencePathMsg>::SharedPtr path_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr target_speed_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr requested_mode_sub_;
  rclcpp::Subscription<DriveModeStateMsg>::SharedPtr vehicle_mode_sub_;
  rclcpp::TimerBase::SharedPtr planning_timer_;
  rclcpp::TimerBase::SharedPtr command_timer_;
};

}  // namespace simp_planner

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<simp_planner::PlannerNodeCpp>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
