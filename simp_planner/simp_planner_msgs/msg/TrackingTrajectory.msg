# stamp는 points[0]의 실행 시각이다.
# frame_id는 각 점의 위치와 차체 방향을 표현하는 기준 좌표계이다.
std_msgs/Header header

# 모든 인접한 점 사이의 고정 시간 간격이다.
builtin_interfaces/Duration sample_period

simp_planner_msgs/TrackingTrajectoryPoint[] points
