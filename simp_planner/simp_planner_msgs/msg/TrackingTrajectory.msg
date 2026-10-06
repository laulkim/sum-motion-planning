# stamp = points[0]의 시각 (0.1 s 경계), frame_id = 좌표계
# points[i]의 시각 = stamp + i × 0.01 s (command_frequency_hz와 같은 간격)
# 배열 끝을 지나면 마지막 점(정지)을 유지한다.
std_msgs/Header header
TrackingTrajectoryPoint[] points
