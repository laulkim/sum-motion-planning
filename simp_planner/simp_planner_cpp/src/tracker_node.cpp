#include "simp_planner/execution_ros.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <simp_planner_msgs/msg/execution_status.hpp>
#include <simp_planner_msgs/msg/drive_mode_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <rclcpp/create_timer.hpp>
#include <atomic>

namespace simp_planner {
class TrackerNode final : public rclcpp::Node {
 public:
  TrackerNode() : Node("simp_tracker") {
    const double hz = declare_parameter<double>("command_frequency_hz", 100.0);
    if (!std::isfinite(hz) || hz <= 0) throw std::invalid_argument("invalid command frequency");
    auto qos = rclcpp::QoS(1).reliable().transient_local();
    command_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    stamped_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("/planner/cmd_vel_stamped", 200);
    executed_pub_ = create_publisher<CommandMsg>("/planner/executed_command", 200);
    state_pub_ = create_publisher<std_msgs::msg::String>("/planner/execution_state", qos);
    status_pub_ = create_publisher<simp_planner_msgs::msg::ExecutionStatus>("/tracker/execution_status", 10);
    trajectory_sub_ = create_subscription<TrajectoryMsg>("/planner/trajectory", rclcpp::QoS(10).reliable(),
        [this](TrajectoryMsg::ConstSharedPtr msg) {
          received_id_ = msg->plan_id;
          if (fault_) { acceptance_ = "FAULT_LATCHED"; publish_status(); return; }
          try {
            acceptance_ = buffer_.receive(trajectory_from_message(*msg), now().nanoseconds());
            if (acceptance_ == "ACCEPTED") execution_started_.store(true);
          } catch (const std::exception& e) { acceptance_ = e.what(); }
          if (acceptance_ != "ACCEPTED" && acceptance_ != "DUPLICATE" &&
              acceptance_ != "OLD_PLAN" && acceptance_ != "OLD_EPOCH") engage_fault("PLAN_REJECTED");
          publish_status();
        });
    mode_sub_ = create_subscription<simp_planner_msgs::msg::DriveModeState>(
        "/vehicle/drive_mode_state", qos,
        [this](simp_planner_msgs::msg::DriveModeState::ConstSharedPtr msg) { mode_ = *msg; });
    rcl_jump_threshold_t threshold{};
    threshold.on_clock_change = true;
    threshold.min_backward.nanoseconds = -1;
    threshold.min_forward.nanoseconds = 2000000000LL;
    jump_ = get_clock()->create_jump_callback(nullptr,
        [this](const rcl_time_jump_t&) { if (execution_started_.load()) clock_reset_.store(true); }, threshold);
    timer_ = rclcpp::create_timer(this, get_clock(), rclcpp::Duration::from_seconds(1.0 / hz),
                                std::bind(&TrackerNode::tick, this));
    RCLCPP_INFO(get_logger(), "Reference executor ready at %.1f Hz (position feedback controller not enabled)", hz);
  }
 private:
  void engage_fault(const std::string& reason) {
    if (fault_) return;
    const auto stamp = now().nanoseconds();
    auto initial = last_sample_.value_or(buffer_.sample(stamp));
    fault_ = std::make_shared<ExecutionTrajectory>();
    fault_->start_ns = stamp;
    fault_->points = make_stop_samples(initial, 4.0, 0.01, 1.0, 0.8);
    fault_reason_ = reason;
    RCLCPP_ERROR(get_logger(), "%s: local braking latched; restart planner/tracker together after inspection", reason.c_str());
  }
  void tick() {
    const auto stamp = now().nanoseconds();
    if (clock_reset_.exchange(false)) engage_fault("CLOCK_RESET");
    buffer_.advance(stamp);
    const auto active = buffer_.active();
    auto p = fault_ ? sample_execution(*fault_, stamp) : buffer_.sample(stamp);
    // Mode alignment must never receive a position-correcting or moving reference.
    if (!fault_ && active && (std::hypot(p.command.vx, p.command.vy) > 0.03 ||
        std::abs(p.command.yaw_rate) > 0.02) &&
        (!mode_ || mode_->status != mode_->STATUS_READY ||
         mode_->current_mode != static_cast<std::uint8_t>(active->drive_mode))) {
      engage_fault("VEHICLE_MODE_MISMATCH");
      p = sample_execution(*fault_, stamp);
    }
    state_ = fault_ ? "LOCAL_STOP_" + fault_reason_ :
        (buffer_.expired(stamp) ? "SAFETY_STOP_EXPIRED" : (active ? active->type : "IDLE"));
    last_sample_ = p;
    geometry_msgs::msg::Twist twist;
    twist.linear.x = p.command.vx; twist.linear.y = p.command.vy;
    twist.angular.z = p.command.yaw_rate;
    command_pub_->publish(twist);
    geometry_msgs::msg::TwistStamped stamped;
    stamped.header.stamp = rclcpp::Time(stamp); stamped.header.frame_id = "base_link";
    stamped.twist = twist; stamped_pub_->publish(stamped);
    auto executed = command_message(p.command);
    executed.header = stamped.header; executed.plan_id = active ? active->plan_id : 0;
    executed_pub_->publish(executed);
    if (state_ != published_state_) {
      std_msgs::msg::String msg; msg.data = state_; state_pub_->publish(msg); published_state_ = state_;
    }
    if (stamp - last_status_ns_ >= 50000000LL || stamp < last_status_ns_) {
      publish_status(); last_status_ns_ = stamp;
    }
  }
  void publish_status() {
    simp_planner_msgs::msg::ExecutionStatus msg;
    msg.header.stamp = now(); msg.session_id = buffer_.session();
    msg.execution_epoch = buffer_.epoch();
    msg.active_plan_id = buffer_.active() ? buffer_.active()->plan_id : 0;
    msg.pending_plan_id = buffer_.pending() ? buffer_.pending()->plan_id : 0;
    msg.received_plan_id = received_id_; msg.acceptance = acceptance_;
    msg.execution_state = fault_ ? "LOCAL_STOP_" + fault_reason_ : state_;
    status_pub_->publish(msg);
  }
  ExecutionBuffer buffer_;
  std::shared_ptr<ExecutionTrajectory> fault_;
  std::optional<ReferenceSample> last_sample_;
  std::optional<simp_planner_msgs::msg::DriveModeState> mode_;
  std::atomic<bool> clock_reset_{false}, execution_started_{false};
  std::string acceptance_, state_{"IDLE"}, published_state_, fault_reason_;
  std::uint64_t received_id_{0};
  std::int64_t last_status_ns_{0};
  rclcpp::JumpHandler::SharedPtr jump_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr stamped_pub_;
  rclcpp::Publisher<CommandMsg>::SharedPtr executed_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<simp_planner_msgs::msg::ExecutionStatus>::SharedPtr status_pub_;
  rclcpp::Subscription<TrajectoryMsg>::SharedPtr trajectory_sub_;
  rclcpp::Subscription<simp_planner_msgs::msg::DriveModeState>::SharedPtr mode_sub_;
};
}
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<simp_planner::TrackerNode>());
  rclcpp::shutdown();
}
