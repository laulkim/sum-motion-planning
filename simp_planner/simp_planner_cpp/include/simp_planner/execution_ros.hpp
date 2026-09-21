#pragma once
#include "simp_planner/execution.hpp"
#include <rclcpp/rclcpp.hpp>
#include <simp_planner_msgs/msg/trajectory.hpp>

namespace simp_planner {
using TrajectoryMsg = simp_planner_msgs::msg::Trajectory;
using CommandMsg = simp_planner_msgs::msg::ExecutedCommand;
inline CommandMsg command_message(const BodyCommand& c) {
  CommandMsg m;
#define COPY(name) m.name = c.name
  COPY(vx); COPY(vy); COPY(yaw_rate); COPY(planned_speed); COPY(planned_acceleration);
  COPY(planned_jerk); COPY(motion_heading); COPY(motion_curvature); COPY(motion_heading_rate);
  COPY(beta); COPY(beta_rate); COPY(yaw_acceleration); COPY(trajectory_time);
  COPY(segment_start_x); COPY(segment_start_y); COPY(segment_start_heading);
  COPY(segment_end_x); COPY(segment_end_y); COPY(segment_end_heading);
#undef COPY
  m.motion_heading_acceleration = c.planned_heading_acceleration;
  m.interval_index = static_cast<std::uint32_t>(c.action_index);
  return m;
}
inline BodyCommand command_from_message(const CommandMsg& m) {
  BodyCommand c;
#define COPY(name) c.name = m.name
  COPY(vx); COPY(vy); COPY(yaw_rate); COPY(planned_speed); COPY(planned_acceleration);
  COPY(planned_jerk); COPY(motion_heading); COPY(motion_curvature); COPY(motion_heading_rate);
  COPY(beta); COPY(beta_rate); COPY(yaw_acceleration); COPY(trajectory_time);
  COPY(segment_start_x); COPY(segment_start_y); COPY(segment_start_heading);
  COPY(segment_end_x); COPY(segment_end_y); COPY(segment_end_heading);
#undef COPY
  c.planned_heading_acceleration = m.motion_heading_acceleration;
  c.action_index = m.interval_index;
  return c;
}
inline TrajectoryMsg trajectory_message(const ExecutionTrajectory& tr, std::int64_t now) {
  TrajectoryMsg m;
  m.header.stamp = rclcpp::Time(now); m.header.frame_id = tr.frame_id;
  m.session_id = tr.session_id; m.plan_id = tr.plan_id;
  m.execution_epoch = tr.execution_epoch; m.base_plan_id = tr.base_plan_id;
  m.start_time = rclcpp::Time(tr.start_ns); m.duration = tr.points.back().time;
  m.trajectory_type = tr.type; m.stop_reason = tr.reason;
  m.drive_mode = static_cast<std::uint8_t>(tr.drive_mode);
  m.fallback_deceleration = tr.deceleration; m.fallback_jerk = tr.jerk;
  m.points.reserve(tr.points.size());
  for (const auto& p : tr.points) {
    simp_planner_msgs::msg::TrajectoryPoint point;
    point.time = p.time; point.x = p.x; point.y = p.y; point.body_yaw = p.body_yaw;
    point.command = command_message(p.command);
    m.points.push_back(point);
  }
  return m;
}
inline std::shared_ptr<ExecutionTrajectory> trajectory_from_message(const TrajectoryMsg& m) {
  auto tr = std::make_shared<ExecutionTrajectory>();
  tr->session_id = m.session_id; tr->plan_id = m.plan_id;
  tr->execution_epoch = m.execution_epoch; tr->base_plan_id = m.base_plan_id;
  tr->start_ns = rclcpp::Time(m.start_time).nanoseconds(); tr->frame_id = m.header.frame_id;
  tr->type = m.trajectory_type; tr->reason = m.stop_reason;
  tr->drive_mode = static_cast<DriveMode>(m.drive_mode);
  tr->deceleration = m.fallback_deceleration; tr->jerk = m.fallback_jerk;
  for (const auto& p : m.points)
    tr->points.push_back({p.time, p.x, p.y, p.body_yaw, command_from_message(p.command)});
  validate_execution_trajectory(*tr);
  if (!std::isfinite(m.duration) || std::abs(m.duration - tr->points.back().time) > 1.e-9)
    throw std::invalid_argument("duration does not match last sample");
  return tr;
}
} // namespace simp_planner
