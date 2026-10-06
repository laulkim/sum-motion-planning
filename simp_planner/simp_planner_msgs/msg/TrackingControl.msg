# One snapshot per tracker control cycle. header.stamp is the control time.
std_msgs/Header header
bool valid
# The remaining state/reference/error fields are meaningful only when valid.
builtin_interfaces/Time odom_stamp
builtin_interfaces/Time reference_stamp
TrackingTrajectoryPoint reference
# Predicted odometry at reference_stamp, in header.frame_id; yaw in radians.
float64 predicted_x
float64 predicted_y
float64 predicted_yaw
# Error used by the controller, expressed in the predicted body frame.
float64 ex
float64 ey
float64 etheta
# Measured body velocity at odom_stamp, held constant during prediction.
geometry_msgs/Twist measured_velocity
# Actual output of this cycle, including zero commands when invalid.
geometry_msgs/Twist command
