# 동일 인덱스의 TrackingTrajectoryPoint를 생성할 때 사용한 nominal 진단값이다.
# 차량의 실제 실행 여부를 뜻하지 않으며, 제어에는 사용하지 않는다.
uint64 plan_id
uint8 requested_mode
bool allocation_valid
uint32 interval_index
float64 trajectory_time
float64 vx
float64 vy
float64 yaw_rate
float64 planned_speed
float64 planned_acceleration
float64 planned_jerk
float64 motion_curvature
float64 motion_heading_rate
float64 motion_heading_acceleration
float64 beta
float64 beta_rate
float64 yaw_acceleration
