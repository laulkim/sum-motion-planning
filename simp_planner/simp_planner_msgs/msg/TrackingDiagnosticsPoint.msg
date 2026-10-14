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
# 차량 projection(|gamma|/delta_max + |r|/(kappa_max V) <= 1) 관련 값.
# allocator가 이 명령에서 projection 때문에 바꾼 yaw rate 양 [rad/s]
# (projection 없이 같은 step에서 골랐을 yaw rate - 실제 명령 yaw rate).
float64 projection_yaw_rate_delta
# 실제 발행 명령 (vx, vy, yaw_rate)의 projection 점수. 1 이하면 차량이 자르지 않는다.
float64 projection_score
# 이 명령의 gamma에서 차량이 허용하는 최대 |yaw rate| [rad/s]: kappa_max V (1 - |gamma|/delta_max).
# 정지(V <= 1e-6)면 0, projection을 하지 않는 제자리턴 모드면 +inf.
float64 projection_yaw_rate_limit
