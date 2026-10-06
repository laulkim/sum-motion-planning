#pragma once

#include <cmath>
#include <cstdint>
#include <optional>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <simp_planner_msgs/msg/tracking_trajectory.hpp>
#include <simp_planner_msgs/msg/tracking_control.hpp>

namespace simp_tracker {
using Trajectory = simp_planner_msgs::msg::TrackingTrajectory;
using Point = simp_planner_msgs::msg::TrackingTrajectoryPoint;
constexpr double kPi = 3.14159265358979323846;

struct Pose { double x, y, yaw; };
struct Gains { double kx{3.0}, ky{4.0}, ktheta{2.0}; };
struct Error { double ex, ey, etheta; };
struct Inputs {
  Trajectory::ConstSharedPtr trajectory;
  nav_msgs::msg::Odometry::ConstSharedPtr odom;
};
struct Control {
  geometry_msgs::msg::Twist velocity;
  std::optional<std::uint8_t> requested_mode;
  simp_planner_msgs::msg::TrackingControl diagnostic;
};

inline double wrap_angle(double angle) {
  return std::remainder(angle, 2.0 * kPi);
}

inline std::optional<Pose> odom_pose(const nav_msgs::msg::Odometry& odom) {
  const auto& p = odom.pose.pose.position;
  const auto& q = odom.pose.pose.orientation;
  const double norm = std::hypot(std::hypot(q.x, q.y), std::hypot(q.z, q.w));
  if (!std::isfinite(p.x) || !std::isfinite(p.y) ||
      !std::isfinite(norm) || norm == 0.0) return std::nullopt;
  const double x = q.x / norm, y = q.y / norm, z = q.z / norm, w = q.w / norm;
  return Pose{p.x, p.y, std::atan2(2.0 * (w * z + x * y),
                                 1.0 - 2.0 * (y * y + z * z))};
}

inline std::optional<std::size_t> reference_index(const Trajectory& trajectory,
                                                std::int64_t now_ns) {
  const auto& period = trajectory.sample_period;
  const auto& stamp = trajectory.header.stamp;
  if (trajectory.points.empty() || period.sec < 0 || period.nanosec >= 1000000000 ||
      stamp.nanosec >= 1000000000) return std::nullopt;
  const std::uint64_t dt = std::uint64_t(period.sec) * 1000000000 + period.nanosec;
  if (dt == 0) return std::nullopt;
  const std::int64_t start = std::int64_t(stamp.sec) * 1000000000 + stamp.nanosec;
  if (now_ns < start) return std::nullopt;
  // 정수 나노초로 ceil을 구한다. 종점 시각의 곱셈 overflow도 피한다.
  const std::uint64_t elapsed = std::uint64_t(now_ns) - std::uint64_t(start);
  const auto index = elapsed / dt + (elapsed % dt != 0);
  if (index >= trajectory.points.size()) return std::nullopt;
  return static_cast<std::size_t>(index);
}

// Fractal Fract. 2023, 7, 121, Section 4.1, 식 (10).
inline Error tracking_error(const Point& reference, const Pose& actual) {
  const double dx = reference.x - actual.x, dy = reference.y - actual.y;
  const double c = std::cos(actual.yaw), s = std::sin(actual.yaw);
  return {c * dx + s * dy, -s * dx + c * dy,
          wrap_angle(reference.body_yaw - actual.yaw)};
}

// 식 (14). ETC/ETM, 적분항, 속도 제한 또는 모드 판단을 추가하지 않는다.
inline geometry_msgs::msg::Twist lyapunov_control(const Point& reference,
                                                 const Error& error, const Gains& gains) {
  const double c = std::cos(error.etheta), s = std::sin(error.etheta);
  geometry_msgs::msg::Twist command;
  command.linear.x = reference.vx * c - reference.vy * s + gains.kx * error.ex;
  command.linear.y = reference.vy * c + reference.vx * s + gains.ky * error.ey;
  command.angular.z = reference.yaw_rate + gains.ktheta * error.etheta;
  return command;
}

inline Control control_cycle(const Inputs& snapshot, std::int64_t now_ns,
                             const Gains& gains,
                             const std::string& base_frame = "base_link") {
  Control output;  // 매 cycle의 기본 출력은 zero command다.
  if (!snapshot.trajectory) return output;
  const auto index = reference_index(*snapshot.trajectory, now_ns);
  if (!index) return output;
  for (const auto& p : snapshot.trajectory->points) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.body_yaw) ||
        !std::isfinite(p.vx) || !std::isfinite(p.vy) || !std::isfinite(p.yaw_rate)) return output;
  }
  const auto& reference = snapshot.trajectory->points[*index];
  output.requested_mode = reference.requested_mode;
  if (!snapshot.odom) return output;
  const auto& odom = *snapshot.odom;
  // 비어 있는 frame도 허용하지 않는다. 모드 전달은 기존 정책을 유지한다.
  if (odom.header.frame_id.empty() ||
      snapshot.trajectory->header.frame_id != odom.header.frame_id ||
      base_frame.empty() || odom.child_frame_id != base_frame) return output;
  auto pose = odom_pose(odom);
  const auto& velocity = odom.twist.twist;
  if (!pose || !std::isfinite(velocity.linear.x) || !std::isfinite(velocity.linear.y) ||
      !std::isfinite(velocity.angular.z) ||
      !std::isfinite(gains.kx) || !std::isfinite(gains.ky) || !std::isfinite(gains.ktheta) ||
      gains.kx <= 0.0 || gains.ky <= 0.0 || gains.ktheta <= 0.0) return output;
  const auto& stamp = odom.header.stamp;
  if (stamp.sec < 0 || stamp.nanosec >= 1000000000) return output;
  const auto odom_ns = std::int64_t(stamp.sec) * 1000000000 + stamp.nanosec;
  if (odom_ns > now_ns) return output;
  const auto& trajectory = *snapshot.trajectory;
  const auto start_ns = std::int64_t(trajectory.header.stamp.sec) * 1000000000 +
                        trajectory.header.stamp.nanosec;
  const auto period_ns = std::uint64_t(trajectory.sample_period.sec) * 1000000000 +
                         trajectory.sample_period.nanosec;
  // ceil로 선택한 점까지 남은 시간. index * period의 overflow 없이 계산한다.
  const auto elapsed_ns = std::uint64_t(now_ns) - std::uint64_t(start_ns);
  const auto ahead_ns = (period_ns - elapsed_ns % period_ns) % period_ns;
  const auto prediction_ns = std::uint64_t(now_ns - odom_ns) + ahead_ns;
  const double dt = prediction_ns * 1.0e-9;
  // ponytail: 실측 body 속도를 유지해 외삽한다. 가속/긴 지연의 오차가 크면 모델을 보강한다.
  const double yaw_mid = pose->yaw + 0.5 * velocity.angular.z * dt;
  const double c = std::cos(yaw_mid), s = std::sin(yaw_mid);
  pose->x += (c * velocity.linear.x - s * velocity.linear.y) * dt;
  pose->y += (s * velocity.linear.x + c * velocity.linear.y) * dt;
  pose->yaw = wrap_angle(pose->yaw + velocity.angular.z * dt);
  const auto error = tracking_error(reference, *pose);
  const auto command = lyapunov_control(reference, error, gains);
  if (std::isfinite(command.linear.x) && std::isfinite(command.linear.y) &&
      std::isfinite(command.angular.z)) {
    output.velocity = command;
    auto& diagnostic = output.diagnostic;
    diagnostic.valid = true;
    diagnostic.header.frame_id = trajectory.header.frame_id;
    diagnostic.odom_stamp = odom.header.stamp;
    const auto reference_ns = std::uint64_t(odom_ns) + prediction_ns;
    diagnostic.reference_stamp.sec = reference_ns / 1000000000;
    diagnostic.reference_stamp.nanosec = reference_ns % 1000000000;
    diagnostic.reference = reference;
    diagnostic.predicted_x = pose->x;
    diagnostic.predicted_y = pose->y;
    diagnostic.predicted_yaw = pose->yaw;
    diagnostic.ex = error.ex;
    diagnostic.ey = error.ey;
    diagnostic.etheta = error.etheta;
    diagnostic.measured_velocity = velocity;
  }
  return output;
}
}  // namespace simp_tracker
