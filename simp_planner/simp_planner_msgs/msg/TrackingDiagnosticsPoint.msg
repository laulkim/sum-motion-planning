# 같은 인덱스의 TrackingTrajectoryPoint를 만들 때 쓴 계획 값이다.
# 제어에는 쓰지 않는다 (debug plot / 로그용).
uint64 plan_id              # 이 점을 만든 계획 번호. 바뀌는 지점 = handover 지점

uint8 DRIVE=0
uint8 STOP=1
uint8 TURN=2
uint8 HOLD=3
uint8 segment               # 주행 / 정지 꼬리 / 회전 / 정지 유지

uint32 interval_index
float64 trajectory_time
float64 planned_speed
float64 planned_acceleration
float64 planned_jerk
float64 motion_curvature
float64 motion_heading_rate
float64 motion_heading_acceleration
float64 beta
float64 beta_rate
float64 yaw_acceleration
