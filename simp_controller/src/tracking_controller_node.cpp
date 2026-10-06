#include "simp_controller/tracking_index.hpp"

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <simp_planner_msgs/msg/tracking_trajectory.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>

namespace simp_controller {

using TrackingTrajectoryMsg = simp_planner_msgs::msg::TrackingTrajectory;

// Plays back the planner's tracking trajectory. Every tick it sends the
// velocity and drive mode of the point that governs the current time; it
// makes no decisions of its own. The planner decides stops, mode changes and
// spot turns, and puts them in the trajectory ahead of time.
class TrackingControllerNode final : public rclcpp::Node {
 public:
  TrackingControllerNode() : Node("tracking_controller_node") {
    // Must match the planner's command_frequency_hz: the trajectory carries no
    // period of its own, its points are exactly one command period apart.
    const double command_frequency_hz =
        declare_parameter<double>("command_frequency_hz", 100.0);
    const double mode_command_period_sec =
        declare_parameter<double>("mode_command_period_sec", 0.25);
    if (!(command_frequency_hz > 0.0) || !std::isfinite(command_frequency_hz) ||
        !(mode_command_period_sec > 0.0) || !std::isfinite(mode_command_period_sec)) {
      throw std::invalid_argument("controller timing parameters must be finite and positive");
    }
    period_ns_ = static_cast<std::int64_t>(std::llround(1.0e9 / command_frequency_hz));
    mode_command_period_ns_ =
        static_cast<std::int64_t>(std::llround(mode_command_period_sec * 1.0e9));

    auto static_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    trajectory_sub_ = create_subscription<TrackingTrajectoryMsg>(
        "/planner/tracking_trajectory", static_qos,
        [this](TrackingTrajectoryMsg::SharedPtr message) { on_trajectory(std::move(message)); });
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    cmd_stamped_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
        "/planner/cmd_vel_stamped", 200);
    mode_command_pub_ = create_publisher<std_msgs::msg::UInt8>(
        "/vehicle/drive_mode_command", static_qos);
    timer_ = create_wall_timer(std::chrono::nanoseconds(period_ns_), [this] { tick(); });
    RCLCPP_INFO(get_logger(), "Tracking controller ready: %.1f Hz playback", command_frequency_hz);
  }

 private:
  std::int64_t now_ns() const { return get_clock()->now().nanoseconds(); }

  void on_trajectory(TrackingTrajectoryMsg::SharedPtr message) {
    if (message->points.empty()) {
      RCLCPP_WARN(get_logger(), "Ignoring empty tracking trajectory");
      return;
    }
    const auto stamp_ns = rclcpp::Time(message->header.stamp).nanoseconds();
    std::lock_guard<std::mutex> lock(mutex_);
    // Out-of-order delivery: an older trajectory never replaces a newer one.
    if (trajectory_ && stamp_ns < stamp_ns_) return;
    trajectory_ = std::move(message);
    stamp_ns_ = stamp_ns;
  }

  void tick() {
    const auto now = now_ns();
    TrackingTrajectoryMsg::SharedPtr trajectory;
    std::int64_t stamp_ns = 0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      trajectory = trajectory_;
      stamp_ns = stamp_ns_;
    }
    geometry_msgs::msg::Twist twist;  // Zero until the first trajectory arrives.
    std::optional<std::uint8_t> mode;
    bool at_rest = false;
    if (trajectory) {
      if (now < stamp_ns) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                             "Tracking trajectory stamped %.3f s in the future; "
                             "planner and controller clocks disagree",
                             1.0e-9 * static_cast<double>(stamp_ns - now));
      }
      const auto& point = trajectory->points[tracking_index(
          now, stamp_ns, period_ns_, trajectory->points.size())];
      twist.linear.x = point.vx;
      twist.linear.y = point.vy;
      twist.angular.z = point.yaw_rate;
      mode = point.mode;
      at_rest = std::abs(point.vx) <= 1.0e-9 && std::abs(point.vy) <= 1.0e-9 &&
                std::abs(point.yaw_rate) <= 1.0e-9;
    }
    cmd_pub_->publish(twist);
    geometry_msgs::msg::TwistStamped stamped;
    stamped.header.stamp = rclcpp::Time(now, get_clock()->get_clock_type());
    stamped.header.frame_id = "base_link";
    stamped.twist = twist;
    cmd_stamped_pub_->publish(stamped);

    // The vehicle only accepts a mode command at standstill and rejects any
    // other, and the planner only changes the mode at rest points. So send
    // only while the commanded point is at rest: on a change, and repeated in
    // case one was missed. While moving there is nothing the vehicle would accept.
    // Repeat only while a change is pending -- the mode differs from the one
    // the vehicle last moved in -- so an ordinary stop sends nothing.
    if (mode && !at_rest) last_moving_mode_ = mode;
    const bool change_pending = !last_moving_mode_ || mode != last_moving_mode_;
    if (mode && at_rest &&
        (!last_mode_ || *mode != *last_mode_ ||
         (change_pending && now - last_mode_command_ns_ >= mode_command_period_ns_))) {
      std_msgs::msg::UInt8 message;
      message.data = *mode;
      mode_command_pub_->publish(message);
      last_mode_ = mode;
      last_mode_command_ns_ = now;
    }
  }

  std::int64_t period_ns_{10000000};
  std::int64_t mode_command_period_ns_{250000000};
  std::mutex mutex_;
  TrackingTrajectoryMsg::SharedPtr trajectory_;
  std::int64_t stamp_ns_{0};
  std::optional<std::uint8_t> last_mode_;
  std::optional<std::uint8_t> last_moving_mode_;
  std::int64_t last_mode_command_ns_{0};
  rclcpp::Subscription<TrackingTrajectoryMsg>::SharedPtr trajectory_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_stamped_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr mode_command_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace simp_controller

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<simp_controller::TrackingControllerNode>());
  rclcpp::shutdown();
  return 0;
}
