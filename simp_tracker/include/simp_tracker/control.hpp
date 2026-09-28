#pragma once

#include <cmath>
#include <cstdint>
#include <optional>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <simp_planner_msgs/msg/tracking_trajectory.hpp>

namespace simp_tracker {
using Trajectory = simp_planner_msgs::msg::TrackingTrajectory;
using Point = simp_planner_msgs::msg::TrackingTrajectoryPoint;
constexpr double kPi = 3.14159265358979323846;

struct Pose { double x, y, yaw; };
struct Gains { double kx{3.0}, ky{4.0}, ktheta{2.0}; };
struct Error { double ex, ey, etheta; };
struct Inputs {
  Trajectory::ConstSharedPtr trajectory;
  std::optional<Pose> pose;
  std::string odom_frame;
};
struct Control {
  geometry_msgs::msg::Twist velocity;
  std::optional<std::uint8_t> requested_mode;
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
                             const Gains& gains) {
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
  if (!snapshot.pose) return output;
  // 비어 있는 frame도 허용하지 않는다. 모드 전달은 기존 정책을 유지한다.
  if (snapshot.odom_frame.empty() ||
      snapshot.trajectory->header.frame_id != snapshot.odom_frame) return output;
  const auto& pose = *snapshot.pose;
  if (!std::isfinite(pose.x) || !std::isfinite(pose.y) || !std::isfinite(pose.yaw) ||
      !std::isfinite(gains.kx) || !std::isfinite(gains.ky) || !std::isfinite(gains.ktheta) ||
      gains.kx <= 0.0 || gains.ky <= 0.0 || gains.ktheta <= 0.0) return output;
  const auto command = lyapunov_control(reference, tracking_error(reference, pose), gains);
  if (std::isfinite(command.linear.x) && std::isfinite(command.linear.y) &&
      std::isfinite(command.angular.z)) output.velocity = command;
  return output;
}
}  // namespace simp_tracker
