# SIMP Planner C++ v8

## 핵심 구조

- 횡방향 종단 offset `-8.0 ~ +8.0 m`, 0.25 m 간격
- 7차 공간 다항식 기반 횡경로 생성
- 곡률·횡가속도·횡저크·종방향 jerk 기반 속도계획
- 정상 길이 Path 후보 충돌 제거 후 대체 후보 탐색
- 모든 정상 후보 실패 시 정지거리 조건을 만족하는 짧은 Path fallback
- 시간 궤적 단계에서는 동적·종단 제약만 검사
- `LATERAL_PRIORITY` 충돌 시 동일 Motion trajectory의 `MINIMUM_VY` 재평가
- 두 Allocation 실패 시 해당 Path/횡목표를 제외하고 다음 후보 탐색
- 차체 yaw 기반 multi-circle swept-footprint 최종 충돌검사
- 최종 실패 시 jerk-limited 안전정지

## 빌드

```bash
cd ~/ros2_ws/src
unzip ~/Downloads/simp_planner_v8.zip
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select \
  planar_velocity_sim simp_planner_msgs simp_planner_cpp simp_tracker simp_planner_tools
source install/setup.bash
```

현재 적용 파라미터와 사용 여부는 최상위 `PARAMETERS_KR.md`를 참고하십시오.

## 시간 궤적 메시지

`/planner/trajectory` (`simp_planner_msgs/msg/Trajectory`)는 계획이 활성화될 때
최종 allocation의 전체 horizon을 한 번 발행합니다. QoS는 reliable,
transient-local, depth 1로 기존 선택 궤적 토픽과 같습니다.

- `header.stamp`: 발행 시각. `header.frame_id`: 위치와 차체 자세의 고정 좌표계.
- `plan_id`: 현재 플래너 실행 내 계획 식별 번호.
- `start_time`: 플래너가 정한 실행 시작 ROS 시각. 발행/수신이 늦어져도 바뀌지 않습니다.
- `points`: 시작점부터 마지막 점까지의 `TrajectoryPoint` 배열.
  - `x`, `y` [m], `yaw` [rad]: 고정 좌표계 위치와 **차체 방향**. 이동 방향 `chi`와 구분합니다.
  - `vx`, `vy` [m/s], `yaw_rate` [rad/s]: 해당 reference 차체 축 기준 속도.
  - `time_from_start` [초]: `start_time`부터의 상대 시간. 첫 점은 0이며 이후 증가합니다.
  - `mode`: `DriveModeState`와 동일한 주행 모드 값.

향후 제어기는 동일한 ROS clock에서 `now - start_time`으로 참조할 구간을
찾습니다. 메시지에는 원래 샘플 간격을 보존하며 `simp_tracker`가 시간 보간과 추종 제어를 수행합니다.
기존 실행기는 jerk 등의 내부 정보를 사용하므로, 향후 단순 선형 보간 결과가
기존 `/cmd_vel`과 완전히 같다고 가정해서는 안 됩니다.

플래너의 선택 경로 및 실행 진단 토픽은 유지합니다. 통합 launch에서 기본적으로
플래너 `/cmd_vel`은 `/planner/cmd_vel`로 remap하고 `simp_tracker`가 최종 `/cmd_vel`을 발행합니다.
새 토픽은 활성화된 계획의 horizon이며, 별도 실행 경로의 안전정지·모드 전환·
제자리 회전 명령이나 계획 취소 알림은 포함하지 않습니다. 마지막 메시지가
보관되므로 수신 자체를 실행 허가로 해석해서는 안 됩니다. 제어기 연결 단계에서
`ExecutedCommand.execution_state`와 plan_id를 함께 사용해 제어기가 계획 실행을
판단합니다. 정상 계획 외 정지/회전 명령은 해당 nominal 메시지에서 전달합니다.

메시지 생성은 `planner_node.cpp`의 `publish_trajectory()`에서 직접 수행합니다.
실제 발행 확인은 workspace 루트에서 아래 스크립트로 실행할 수 있습니다.
기존 시뮬레이션 패키지도 빌드되어 있어야 하며, 사용하지 않는 ROS domain을 선택합니다.

```bash
source install/setup.bash
ROS_DOMAIN_ID=88 ROS_LOG_DIR=/tmp/trajectory_topic_ros_logs \
  python3 src/sum-motion-planning/simp_planner/simp_planner_cpp/test/check_trajectory_topic.py
```
