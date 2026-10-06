#include <chrono>
#include <memory>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include "simp_tracker/control.hpp"

class TrackerNode final : public rclcpp::Node {
 public:
  TrackerNode() : Node("simp_tracker") {
    declare_parameter("kx", 0.3);
    declare_parameter("ky", 0.2);
    declare_parameter("ktheta", 0.2);
    declare_parameter("max_prediction_sec", 0.05);
    declare_parameter("base_frame", "base_link");
    velocity_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);
    diagnostic_pub_ = create_publisher<simp_planner_msgs::msg::TrackingControl>(
        "/tracker/control", rclcpp::QoS(10).best_effort());
    mode_pub_ = create_publisher<std_msgs::msg::UInt8>(
        "/vehicle/drive_mode_command", rclcpp::QoS(1).reliable().transient_local());
    trajectory_sub_ = create_subscription<simp_tracker::Trajectory>(
        "/planner/tracking_trajectory", rclcpp::QoS(1).reliable(),
        [this](simp_tracker::Trajectory::ConstSharedPtr message) {
          std::lock_guard<std::mutex> lock(input_mutex_);
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
    timer_ = create_wall_timer(std::chrono::milliseconds(10), [this] { control(); }, control_group_);
  }

 private:
  void control() {
    simp_tracker::Inputs snapshot;
    {
      std::lock_guard<std::mutex> lock(input_mutex_);
      snapshot = latest_;
    }
    // 잠금 해제 후에는 이번 cycle의 snapshot만 사용한다.
    const auto now_ns = get_clock()->now().nanoseconds();
    const auto parameters = get_parameters({"kx", "ky", "ktheta", "max_prediction_sec", "base_frame"});
    const simp_tracker::Gains gains{parameters[0].as_double(), parameters[1].as_double(),
                                     parameters[2].as_double()};
    auto output = simp_tracker::control_cycle(
        snapshot, now_ns, gains, parameters[3].as_double(), parameters[4].as_string());
    if (output.requested_mode) {
      std_msgs::msg::UInt8 mode;
      mode.data = *output.requested_mode;
      mode_pub_->publish(mode);
    }
    velocity_pub_->publish(output.velocity);
    output.diagnostic.header.stamp = rclcpp::Time(now_ns);
    output.diagnostic.command = output.velocity;
    diagnostic_pub_->publish(output.diagnostic);
  }

  std::mutex input_mutex_;
  simp_tracker::Inputs latest_;
  rclcpp::Subscription<simp_tracker::Trajectory>::SharedPtr trajectory_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_pub_;
  rclcpp::Publisher<simp_planner_msgs::msg::TrackingControl>::SharedPtr diagnostic_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr mode_pub_;
  rclcpp::CallbackGroup::SharedPtr control_group_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TrackerNode>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
