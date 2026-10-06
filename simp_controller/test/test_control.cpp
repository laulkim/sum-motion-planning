#include <iostream>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <type_traits>
#include "simp_controller/control.hpp"

using namespace simp_controller;

namespace {
constexpr std::int64_t kPeriod = 10000000;

void require(bool value, const char* message) {
  if (!value) throw std::runtime_error(message);
}
void near(double actual, double expected) {
  require(std::isfinite(actual) && std::abs(actual - expected) < 1e-10, "numeric mismatch");
}
void zero(const Control& output) {
  require(!output.tracking.valid, "invalid control must not report a tracking error");
  near(output.velocity.linear.x, 0.0);
  near(output.velocity.linear.y, 0.0);
  near(output.velocity.angular.z, 0.0);
}
Control cycle(const Inputs& inputs, std::int64_t now_ns,
              const std::string& base_frame = "base_link") {
  return control_cycle(inputs, now_ns, kPeriod, Gains{}, base_frame);
}

nav_msgs::msg::Odometry::SharedPtr make_odom() {
  auto odom = std::make_shared<nav_msgs::msg::Odometry>();
  odom->header.stamp.sec = 1;
  odom->header.frame_id = "odom";
  odom->child_frame_id = "base_link";
  odom->pose.pose.orientation.w = 1.0;
  return odom;
}
}  // namespace

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
    a->header.frame_id = "odom";
    a->header.stamp.sec = 1;
    a->points.resize(31);
    for (std::size_t i = 0; i < a->points.size(); ++i) {
      a->points[i].vx = static_cast<double>(i);
      a->points[i].requested_mode = static_cast<std::uint8_t>(i % 5);
    }
    require(!reference_index(*a, 999999999, kPeriod), "future trajectory accepted");
    require(reference_index(*a, 1000000000, kPeriod) == 0, "start index");
    require(reference_index(*a, 1000000001, kPeriod) == 1, "ceil 1 ns after start");
    require(reference_index(*a, 1009999999, kPeriod) == 1, "ceil just before knot");
    require(reference_index(*a, 1010000000, kPeriod) == 1, "exact knot");
    require(reference_index(*a, 1010000001, kPeriod) == 2, "ceil just after knot");
    require(reference_index(*a, 1300000000, kPeriod) == 30, "last point must be usable");
    require(!reference_index(*a, 1300000001, kPeriod), "expired trajectory accepted");
    require(!reference_index(*a, 1000000000, 0), "zero period accepted");
    require(!reference_index(*a, 1000000000, -kPeriod), "negative period accepted");

    Inputs latest{a, make_odom()};
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
    const auto old_cycle = cycle(snapshot, 1000000001);
    near(old_cycle.velocity.linear.x, 1.0);
    require(old_cycle.requested_mode == 1, "mode must come from A's selected point");
    const auto next_cycle = cycle(latest, 1000000001);
    near(next_cycle.velocity.linear.x, 99.0);
    require(next_cycle.requested_mode == 3, "next cycle must use B");
    auto wrong_frame = latest;
    auto frame_odom = make_odom();
    wrong_frame.odom = frame_odom;
    frame_odom->header.frame_id = "map";
    const auto mismatch = cycle(wrong_frame, 1000000001);
    zero(mismatch);
    require(mismatch.requested_mode == 3, "frame check must not change mode relay");
    frame_odom->header.frame_id.clear();
    zero(cycle(wrong_frame, 1000000001));
    auto empty_frame = std::make_shared<Trajectory>(*b);
    empty_frame->header.frame_id.clear();
    wrong_frame.trajectory = empty_frame;
    zero(cycle(wrong_frame, 1000000001));  // 둘 다 빈 frame
    frame_odom->header.frame_id = "odom";
    zero(cycle(wrong_frame, 1000000001));  // trajectory만 빈 frame
    wrong_frame.trajectory = b;
    frame_odom->child_frame_id = "sensor";
    zero(cycle(wrong_frame, 1000000001));
    near(cycle(wrong_frame, 1000000001, "sensor").velocity.linear.x, 99.0);
    zero(cycle(latest, 1300000001));
    require(!cycle(latest, 1300000001).requested_mode, "expired trajectory relayed a mode");
    zero(cycle(latest, 999999999));
    zero(cycle(Inputs{}, 1000000000));
    zero(cycle(Inputs{a, nullptr}, 1010000000));

    auto invalid = std::make_shared<Trajectory>(*a);
    latest.trajectory = invalid;
    zero(control_cycle(latest, 1000000000, 0, Gains{}));
    invalid->points[1].vx = std::numeric_limits<double>::quiet_NaN();
    zero(cycle(latest, 1000000001));
    invalid->points[1].vx = std::numeric_limits<double>::infinity();
    zero(cycle(latest, 1000000001));
    invalid->points.clear();
    zero(cycle(latest, 1000000000));
    latest.trajectory = a;
    auto bad_odom = make_odom();
    latest.odom = bad_odom;
    bad_odom->pose.pose.position.x = std::numeric_limits<double>::quiet_NaN();
    zero(cycle(latest, 1010000000));
    bad_odom->pose.pose.position.x = -std::numeric_limits<double>::max();
    zero(cycle(latest, 1010000000));  // 유한 입력의 계산 overflow

    // now=1.014여도 선택한 점은 1.020: 전진/후진/횡이동의 가짜 오차가 없어야 한다.
    auto moving = make_odom();
    auto aligned = std::make_shared<Trajectory>(*a);
    Inputs prediction{aligned, moving};
    auto& target = aligned->points[2];
    for (const double vx : {-2.0, 0.0, 2.0}) {
      moving->twist.twist.linear.x = target.vx = vx;
      moving->twist.twist.linear.y = target.vy = 0.5;
      target.x = vx * 0.02;
      target.y = 0.01;
      const auto output = cycle(prediction, 1014000000);
      require(output.tracking.valid, "missing control snapshot");
      require(output.tracking.reference_ns == 1020000000, "reference timestamp");
      require(output.tracking.reference == target, "diagnostic must keep the selected reference");
      near(output.tracking.predicted.x, target.x);
      near(output.tracking.predicted.y, target.y);
      near(output.tracking.error.ex, 0.0);
      near(output.tracking.error.ey, 0.0);
      near(output.tracking.error.etheta, 0.0);
      near(output.velocity.linear.x, vx);
      near(output.velocity.linear.y, 0.5);
      near(output.velocity.angular.z, 0.0);
    }
    // 중간 방향이 pi/2인 회전+횡이동, 그리고 pi 경계를 지나는 회전.
    moving->twist.twist.angular.z = target.yaw_rate = 1.0;
    for (const double mid_yaw : {kPi / 2, kPi}) {
      moving->pose.pose.orientation.z = std::sin((mid_yaw - 0.01) / 2);
      moving->pose.pose.orientation.w = std::cos((mid_yaw - 0.01) / 2);
      target.x = mid_yaw == kPi ? -0.04 : -0.01;
      target.y = mid_yaw == kPi ? -0.01 : 0.04;
      target.body_yaw = wrap_angle(mid_yaw + 0.01);
      for (int repeat = 0; repeat < 2; ++repeat) {
        const auto output = cycle(prediction, 1014000000);
        near(output.tracking.predicted.yaw, target.body_yaw);
        near(output.tracking.error.ex, 0.0);
        near(output.tracking.error.ey, 0.0);
        near(output.tracking.error.etheta, 0.0);
        near(output.velocity.linear.x, 2.0);
        near(output.velocity.linear.y, 0.5);
        near(output.velocity.angular.z, 1.0);
      }
      near(moving->pose.pose.position.x, 0.0);  // 예측을 odom 원본에 누적하지 않는다.
    }
    // 오래된 측위도 시간 상한으로 정지하지 않고 선택한 점까지 예측한다.
    moving->pose.pose.orientation.z = 0.0;
    moving->pose.pose.orientation.w = 1.0;
    moving->twist.twist.angular.z = 0.0;
    for (std::size_t i = 0; i < aligned->points.size(); ++i) {
      auto& p = aligned->points[i];
      p.x = 2.0 * i * 0.01;
      p.y = 0.5 * i * 0.01;
      p.body_yaw = p.yaw_rate = 0.0;
      p.vx = 2.0;
      p.vy = 0.5;
    }
    for (const auto now : {1060000000LL, 1294000000LL}) {
      const auto output = cycle(prediction, now);
      require(output.tracking.valid, "old odom must not trigger a time-limit stop");
      near(output.velocity.linear.x, 2.0);
      near(output.velocity.linear.y, 0.5);
      near(output.tracking.error.ex, 0.0);
      near(output.tracking.error.ey, 0.0);
    }
    zero(cycle(prediction, 1300000001));  // 궤적 만료는 여전히 정지한다.
    moving->header.stamp.nanosec = 14000001;
    zero(cycle(prediction, 1014000000));  // future odom, before reference
    moving->header.stamp.nanosec = 1000000000;
    zero(cycle(prediction, 1014000000));
    moving->header.stamp.nanosec = 0;
    moving->header.stamp.sec = -1;
    zero(cycle(prediction, 1014000000));
    moving->header.stamp.sec = 1;
    for (double* component : {&moving->twist.twist.linear.x, &moving->twist.twist.linear.y,
                              &moving->twist.twist.angular.z}) {
      const double saved = *component;
      *component = std::numeric_limits<double>::quiet_NaN();
      zero(cycle(prediction, 1014000000));
      *component = saved;
    }

    nav_msgs::msg::Odometry odom;
    odom.pose.pose.orientation.w = 0.0;
    require(!odom_pose(odom), "zero quaternion accepted");
    odom.pose.pose.orientation.z = std::sin(kPi / 4);
    odom.pose.pose.orientation.w = std::cos(kPi / 4);
    near(odom_pose(odom)->yaw, kPi / 2);
    odom.pose.pose.orientation.w = std::numeric_limits<double>::infinity();
    require(!odom_pose(odom), "nonfinite quaternion accepted");
    std::cout << "controller tracking: equations, midpoint prediction, ceil, snapshot and safety passed\n";
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
