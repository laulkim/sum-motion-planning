# TrackingTrajectory.header.frame_id 좌표계 기준 위치와 차체 방향이다.
float64 x
float64 y
float64 body_yaw

# 목표 차체 좌표계 기준 속도이다.
float64 vx
float64 vy
float64 yaw_rate

# 모드 값은 DriveModeState에 정의된 상수를 사용한다.
# 플래너가 마지막으로 확인한 차량의 현재 모드이다.
uint8 current_mode
# 이 점에 대해 플래너가 요청하는 목표 모드이다.
uint8 requested_mode
