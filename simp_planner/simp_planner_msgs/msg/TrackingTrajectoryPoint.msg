# 위치·자세 (TrackingTrajectory.header.frame_id 좌표계 기준)
float64 x
float64 y
float64 body_yaw

# 차체 좌표계 속도 명령. 이 점의 시각부터 다음 점의 시각까지 유지한다.
float64 vx
float64 vy
float64 yaw_rate

# 모드 값은 DriveModeState 상수를 쓴다 (FORWARD=0 … SPOT_TURN=4).
# 배열을 발행할 때 플래너가 마지막으로 확인한 차량의 현재 모드.
uint8 current_mode
# 이 점에서 플래너가 요청하는 모드. 컨트롤러가 그대로 차량에 보낸다.
uint8 requested_mode
