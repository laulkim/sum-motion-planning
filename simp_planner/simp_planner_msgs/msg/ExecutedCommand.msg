std_msgs/Header header
uint64 plan_id
# Planner execution state paired atomically with this nominal command/plan_id.
# ACTIVE_PLAN, SAFETY_STOP, MODE_STOP, MODE_WAIT, TERMINAL_HOLD, IDLE,
# SPOT_TURN_ROTATING or SPOT_TURN_WAITING_CLEARANCE.
string execution_state
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
