#include <iostream>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <type_traits>
#include "simp_tracker/control.hpp"

using namespace simp_tracker;

void require(bool value, const char* message) {
  if (!value) throw std::runtime_error(message);
}
void near(double actual, double expected) {
  require(std::isfinite(actual) && std::abs(actual - expected) < 1e-10, "numeric mismatch");
}
void zero(const Control& output) {
  near(output.velocity.linear.x, 0.0);
  near(output.velocity.linear.y, 0.0);
  near(output.velocity.angular.z, 0.0);
}

int main() {
  try {
    Point reference;
    reference.x = 4.0;
    reference.y = 6.0;
    reference.body_yaw = kPi / 2 + 0.3;
    const auto error = tracking_error(reference, Pose{1.0, 2.0, kPi / 2});
    near(error.ex, 4.0);
    near(error.ey, -3.0);
    near(error.etheta, 0.3);

    reference.vx = 1.2;
    reference.vy = -0.4;
    reference.yaw_rate = 0.3;
    const auto feedforward = lyapunov_control(reference, {0.0, 0.0, 0.0}, Gains{});
    near(feedforward.linear.x, 1.2);
    near(feedforward.linear.y, -0.4);
    near(feedforward.angular.z, 0.3);
    const auto rotated = lyapunov_control(reference, {1.0, -2.0, kPi / 2}, Gains{});
    near(rotated.linear.x, 3.4);
    near(rotated.linear.y, -6.8);
    near(rotated.angular.z, 0.3 + kPi);
    reference.body_yaw = -kPi + 0.1;
    near(tracking_error(reference, {4.0, 6.0, kPi - 0.1}).etheta, 0.2);
    near(wrap_angle(3.0 * kPi), -kPi);
    near(wrap_angle(-3.0 * kPi), kPi);

    auto a = std::make_shared<Trajectory>();
    a->header.stamp.sec = 1;
    a->sample_period.nanosec = 10000000;
    a->points.resize(31);
    for (std::size_t i = 0; i < a->points.size(); ++i) {
      a->points[i].vx = static_cast<double>(i);
      a->points[i].requested_mode = static_cast<std::uint8_t>(i % 5);
    }
    require(!reference_index(*a, 999999999), "future trajectory accepted");
    require(reference_index(*a, 1000000000) == 0, "start index");
    require(reference_index(*a, 1000000001) == 1, "ceil 1 ns after start");
    require(reference_index(*a, 1009999999) == 1, "ceil just before knot");
    require(reference_index(*a, 1010000000) == 1, "exact knot");
    require(reference_index(*a, 1010000001) == 2, "ceil just after knot");
    require(reference_index(*a, 1300000000) == 30, "last point must be usable");
    require(!reference_index(*a, 1300000001), "expired trajectory accepted");

    Inputs latest{a, Pose{0.0, 0.0, 0.0}};
    std::mutex mutex;
    Inputs snapshot;
    {
      std::lock_guard<std::mutex> lock(mutex);
      snapshot = latest;
    }
    static_assert(std::is_const_v<Trajectory::ConstSharedPtr::element_type>);
    auto b = std::make_shared<Trajectory>(*a);
    b->points[1].vx = 99.0;
    b->points[1].requested_mode = 3;
    {
      // A를 확보한 cycle 도중 callback이 B로 교체한 순서를 재현한다.
      std::lock_guard<std::mutex> lock(mutex);
      latest.trajectory = b;
    }
    const auto old_cycle = control_cycle(snapshot, 1000000001, Gains{});
    near(old_cycle.velocity.linear.x, 1.0);
    require(old_cycle.requested_mode == 1, "mode must come from A's selected point");
    const auto next_cycle = control_cycle(latest, 1000000001, Gains{});
    near(next_cycle.velocity.linear.x, 99.0);
    require(next_cycle.requested_mode == 3, "next cycle must use B");
    zero(control_cycle(latest, 1300000001, Gains{}));
    zero(control_cycle(latest, 999999999, Gains{}));
    zero(control_cycle(Inputs{}, 1000000000, Gains{}));
    zero(control_cycle(Inputs{a, std::nullopt}, 1010000000, Gains{}));

    auto invalid = std::make_shared<Trajectory>(*a);
    latest.trajectory = invalid;
    invalid->sample_period.nanosec = 0;
    zero(control_cycle(latest, 1000000000, Gains{}));
    invalid->sample_period.sec = -1;
    zero(control_cycle(latest, 1000000000, Gains{}));
    invalid->sample_period.sec = 0;
    invalid->sample_period.nanosec = 1000000000;
    zero(control_cycle(latest, 1000000000, Gains{}));
    invalid->sample_period.nanosec = 10000000;
    invalid->points[1].vx = std::numeric_limits<double>::quiet_NaN();
    zero(control_cycle(latest, 1000000001, Gains{}));
    invalid->points[1].vx = std::numeric_limits<double>::infinity();
    zero(control_cycle(latest, 1000000001, Gains{}));
    invalid->points.clear();
    zero(control_cycle(latest, 1000000000, Gains{}));
    latest.trajectory = a;
    latest.pose = Pose{std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0};
    zero(control_cycle(latest, 1010000000, Gains{}));
    latest.pose = Pose{-std::numeric_limits<double>::max(), 0.0, 0.0};
    zero(control_cycle(latest, 1010000000, Gains{})); // 유한 입력의 계산 overflow

    nav_msgs::msg::Odometry odom;
    odom.pose.pose.orientation.w = 0.0;
    require(!odom_pose(odom), "zero quaternion accepted");
    odom.pose.pose.orientation.z = std::sin(kPi / 4);
    odom.pose.pose.orientation.w = std::cos(kPi / 4);
    near(odom_pose(odom)->yaw, kPi / 2);
    odom.pose.pose.orientation.w = std::numeric_limits<double>::infinity();
    require(!odom_pose(odom), "nonfinite quaternion accepted");
    std::cout << "tracker control: equations, yaw wrap, ceil, snapshot and safety passed\n";
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
