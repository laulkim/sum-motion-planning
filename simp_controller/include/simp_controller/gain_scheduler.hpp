#pragma once

#include <cmath>
#include <simp_planner_msgs/msg/tracking_trajectory_point.hpp>

namespace simp_controller {
using Point = simp_planner_msgs::msg::TrackingTrajectoryPoint;
struct Gains { double kx{3.0}, ky{4.0}, ktheta{2.0}; };

// Initial tuning values: speed bounds in m/s, yaw-rate threshold in rad/s.
// Uses only the selected Planner reference, independent of odometry or mode.
inline double translational_gain_scale(const Point& reference) {
  constexpr double kYawRateThreshold = 0.1;
  if (std::abs(reference.yaw_rate) <= kYawRateThreshold) return 1.0;
  constexpr struct { double speed_upper_bound, scale; } kLookup[] = {
      {0.02, 0.0}, {0.05, 0.3}, {0.10, 0.7}};
  const double speed = std::hypot(reference.vx, reference.vy);
  for (const auto& entry : kLookup) {
    if (speed < entry.speed_upper_bound) return entry.scale;
  }
  return 1.0;
}

inline Gains scheduled_gains(const Point& reference, const Gains& nominal_gains) {
  const double alpha = translational_gain_scale(reference);
  return {alpha * nominal_gains.kx, alpha * nominal_gains.ky, nominal_gains.ktheta};
}
}  // namespace simp_controller
