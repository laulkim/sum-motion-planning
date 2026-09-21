# Publication time and fixed coordinate frame for point positions/body yaw.
std_msgs/Header header
uint64 plan_id

# ROS clock time at which points[0] (time_from_start = 0) is scheduled.
# This is the execution time origin, not the publication or receipt time.
builtin_interfaces/Time start_time
TrajectoryPoint[] points
