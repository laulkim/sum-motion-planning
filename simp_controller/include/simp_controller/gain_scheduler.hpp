#pragma once

#include <simp_planner_msgs/msg/tracking_trajectory_point.hpp>

namespace simp_controller {
using Point = simp_planner_msgs::msg::TrackingTrajectoryPoint;
struct Gains { double kx{3.0}, ky{4.0}, ktheta{2.0}; };

// Planner explicitly supplies zero translational references for a spot turn.
// Compare those values directly; no motion thresholds, odometry or mode checks.
inline double translational_gain_scale(const Point& reference) {
  return reference.vx == 0.0 && reference.vy == 0.0 && reference.yaw_rate != 0.0
             ? 0.0 : 1.0;
}

inline Gains scheduled_gains(const Point& reference, const Gains& nominal_gains) {
  const double alpha = translational_gain_scale(reference);
  return {alpha * nominal_gains.kx, alpha * nominal_gains.ky, nominal_gains.ktheta};
}
}  // namespace simp_controller
