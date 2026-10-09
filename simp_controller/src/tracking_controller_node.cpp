#include "simp_controller/control.hpp"

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <simp_planner_msgs/msg/tracking_control.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <stdexcept>

namespace simp_controller {

// Tracks the planner's tracking trajectory with odometry feedback. Every tick
// it predicts the measured pose to the next trajectory point's time, feeds the
// error to the control law, and forwards that point's requested mode. It makes
// no decisions of its own: the planner puts stops, mode changes and spot turns
// in the trajectory ahead of time.
class TrackingControllerNode final : public rclcpp::Node {
 public:
  TrackingControllerNode() : Node("tracking_controller_node") {
    // Must match the planner's command_frequency_hz: the trajectory carries no
    // period of its own, its points are exactly one command period apart.
    const double command_frequency_hz =
        declare_parameter<double>("command_frequency_hz", 100.0);
    if (!(command_frequency_hz > 0.0) || !std::isfinite(command_frequency_hz)) {
      throw std::invalid_argument("command_frequency_hz must be finite and positive");
    }
    period_ns_ = static_cast<std::int64_t>(std::llround(1.0e9 / command_frequency_hz));
    declare_parameter("kx", 1.0);
    declare_parameter("ky", 2.0);
    declare_parameter("ktheta", 1.0);
    declare_parameter("base_frame", "base_link");

    auto static_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    cmd_stamped_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
        "/planner/cmd_vel_stamped", 200);
    tracking_control_pub_ = create_publisher<simp_planner_msgs::msg::TrackingControl>(
        "/tracker/control", rclcpp::QoS(10).best_effort());
    mode_command_pub_ = create_publisher<std_msgs::msg::UInt8>(
        "/vehicle/drive_mode_command", static_qos);
    trajectory_sub_ = create_subscription<Trajectory>(
        "/planner/tracking_trajectory", static_qos,
        [this](Trajectory::ConstSharedPtr message) {
          std::lock_guard<std::mutex> lock(input_mutex_);
          // Out-of-order delivery: an older trajectory never replaces a newer one.
          if (latest_.trajectory) {
            const auto& incoming = message->header.stamp;
            const auto& stored = latest_.trajectory->header.stamp;
            if (incoming.sec < stored.sec ||
                (incoming.sec == stored.sec && incoming.nanosec <= stored.nanosec)) return;
          }
          latest_.trajectory = std::move(message);
        });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/odom", rclcpp::QoS(1).best_effort(),
        [this](nav_msgs::msg::Odometry::ConstSharedPtr message) {
          const auto& incoming = message->header.stamp;
          if (incoming.sec < 0 || incoming.nanosec >= 1000000000 ||
              rclcpp::Time(incoming).nanoseconds() > get_clock()->now().nanoseconds()) return;
          std::lock_guard<std::mutex> lock(input_mutex_);
          if (latest_.odom) {
            const auto& stored = latest_.odom->header.stamp;
            if (incoming.sec < stored.sec ||
                (incoming.sec == stored.sec && incoming.nanosec <= stored.nanosec)) return;
          }
          latest_.odom = std::move(message);
        });
    control_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    timer_ = create_wall_timer(std::chrono::nanoseconds(period_ns_), [this] { control(); },
                               control_group_);
    RCLCPP_INFO(get_logger(), "Tracking controller ready: %.1f Hz feedback tracking",
                command_frequency_hz);
  }

 private:
  void control() {
    Inputs snapshot;
    {
      std::lock_guard<std::mutex> lock(input_mutex_);
      snapshot = latest_;
    }
    // 잠금 해제 후에는 이번 cycle의 snapshot만 사용한다.
    const auto now_ns = get_clock()->now().nanoseconds();
    const auto parameters = get_parameters({"kx", "ky", "ktheta", "base_frame"});
    const Gains gains{parameters[0].as_double(), parameters[1].as_double(),
                      parameters[2].as_double()};
    const auto output = control_cycle(snapshot, now_ns, period_ns_, gains,
                                      parameters[3].as_string());
    if (output.requested_mode) {
      std_msgs::msg::UInt8 mode;
      mode.data = *output.requested_mode;
      mode_command_pub_->publish(mode);
    }
    cmd_pub_->publish(output.velocity);
    geometry_msgs::msg::TwistStamped stamped;
    stamped.header.stamp = rclcpp::Time(now_ns, get_clock()->get_clock_type());
    stamped.header.frame_id = parameters[3].as_string();
    stamped.twist = output.velocity;
    cmd_stamped_pub_->publish(stamped);
    // Publish the exact snapshot used above; the plotter never reselects a point.
    simp_planner_msgs::msg::TrackingControl diagnostic;
    diagnostic.header.stamp = stamped.header.stamp;
    diagnostic.command = output.velocity;
    diagnostic.valid = output.tracking.valid;
    if (diagnostic.valid) {
      const auto& tracking = output.tracking;
      diagnostic.header.frame_id = snapshot.trajectory->header.frame_id;
      diagnostic.odom_stamp = snapshot.odom->header.stamp;
      diagnostic.reference_stamp = rclcpp::Time(tracking.reference_ns, get_clock()->get_clock_type());
      diagnostic.reference = tracking.reference;
      diagnostic.predicted_x = tracking.predicted.x;
      diagnostic.predicted_y = tracking.predicted.y;
      diagnostic.predicted_yaw = tracking.predicted.yaw;
      diagnostic.ex = tracking.error.ex;
      diagnostic.ey = tracking.error.ey;
      diagnostic.etheta = tracking.error.etheta;
      diagnostic.measured_velocity = snapshot.odom->twist.twist;
    }
    tracking_control_pub_->publish(diagnostic);
  }

  std::int64_t period_ns_{10000000};
  std::mutex input_mutex_;
  Inputs latest_;
  rclcpp::Subscription<Trajectory>::SharedPtr trajectory_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_stamped_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr mode_command_pub_;
  rclcpp::Publisher<simp_planner_msgs::msg::TrackingControl>::SharedPtr tracking_control_pub_;
  rclcpp::CallbackGroup::SharedPtr control_group_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace simp_controller

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<simp_controller::TrackingControllerNode>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
