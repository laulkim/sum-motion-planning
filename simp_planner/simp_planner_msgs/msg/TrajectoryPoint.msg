# Relative ROS seconds from Trajectory.start_time; strictly increasing from zero.
float64 time
float64 x
float64 y
float64 body_yaw
# Reference command, NOT measured vehicle feedback. Header/plan_id are unused here.
ExecutedCommand command
