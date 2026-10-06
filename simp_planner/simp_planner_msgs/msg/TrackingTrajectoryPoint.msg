# 위치·자세 (TrackingTrajectory.header.frame_id 좌표계 기준)
float64 x
float64 y
float64 body_yaw

# 차체 좌표계 속도 명령. 이 점의 시각부터 다음 점의 시각까지 유지한다.
float64 vx
float64 vy
float64 yaw_rate

# 차량에 그대로 보낼 모드 (DriveModeState 상수: FORWARD=0 … SPOT_TURN=4)
uint8 mode
