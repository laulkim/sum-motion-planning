# Position [m] and body yaw [rad] in Trajectory.header.frame_id.
# yaw is body orientation, not the direction of motion.
float64 x
float64 y
float64 yaw

# Velocity in the reference body's axes [m/s, m/s, rad/s].
float64 vx
float64 vy
float64 yaw_rate

# Seconds since Trajectory.start_time; strictly increasing across points.
float64 time_from_start

# Drive mode; uses the mode values defined in DriveModeState.
uint8 mode
