std_msgs/Header header
uint64 plan_id
uint32 interval_index
float64 trajectory_time
float64 vx
float64 vy
float64 yaw_rate
float64 planned_speed
float64 planned_acceleration
float64 planned_jerk
float64 motion_heading
float64 motion_curvature
float64 motion_heading_rate
float64 motion_heading_acceleration
float64 beta
float64 beta_rate
float64 yaw_acceleration
float64 segment_start_x
float64 segment_start_y
float64 segment_start_heading
float64 segment_end_x
float64 segment_end_y
float64 segment_end_heading
# Position errors are reference minus measured pose in the reference motion frame.
# Heading error is wrap(chi_ref - (measured_body_yaw + allocated_beta)),
# equivalently reference body yaw minus measured body yaw.
bool tracking_active
float64 tracking_longitudinal_error
float64 tracking_lateral_error
float64 tracking_heading_error
float64 tracking_speed_correction
float64 tracking_heading_rate_correction
