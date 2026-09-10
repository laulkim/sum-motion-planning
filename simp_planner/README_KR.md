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
- odometry 기반 종방향·횡방향·진행 헤딩 P 피드백으로 실행 궤적 추종

## 위치·헤딩 P 피드백

실행 단계에서 선택 궤적과 odometry의 위치·자세 오차를 계산하고,
기존 speed와 chi_dot 명령에 아래 P 항만 더한다.

```text
dx = x_ref - x_measured
dy = y_ref - y_measured
e_long =  cos(chi_ref) * dx + sin(chi_ref) * dy
e_lat  = -sin(chi_ref) * dx + cos(chi_ref) * dy
e_heading = wrap((chi_ref - beta_ref) - body_yaw_measured)

speed_cmd   = speed_ff   + K_long * e_long
chi_dot_cmd = chi_dot_ff + K_lat * e_lat + K_heading * e_heading
vx_cmd      = speed_cmd * cos(beta_ref)
vy_cmd      = speed_cmd * sin(beta_ref)
yaw_rate_cmd = yaw_rate_ff + K_lat * e_lat + K_heading * e_heading
```

위치 오차의 기준은 장애물 회피를 포함한 선택 궤적의 진행 방향이다.
헤딩 오차는 계획된 차체 yaw와 odometry pose yaw의 차이이다.
기존 odometry 시각 정렬을 사용하며, 다음 plan의 handover 예측은 원래
feedforward 방식이다. 기존 정지 명령·안전정지·모드 전환 처리는 유지한다.

P 보정에는 별도의 상한, 가속도·jerk 제한기, 저속 완화를 넣지 않는다.
원래 Planner/Allocation의 제약 계산은 유지되며, 그 계산 이후 P 항을 더한다.
따라서 보정 후 명령에 원래 계획의 제약 충족이 자동으로 보장되는 것은 아니다.
P 오차/이득이 0이면 원래 명령을 그대로 유지한다.

노드 시작 시 읽는 파라미터는 네 가지다.

| 파라미터 | 노드 기본값 | 의미 |
|---|---:|---|
| `tracking_enabled` | `true` | P 보정 적용 여부 |
| `tracking_longitudinal_kp` | `0.8` | 종방향 위치 오차 → speed, 1/s |
| `tracking_lateral_kp` | `0.6` | 횡방향 위치 오차 → chi_dot, rad/(m·s) |
| `tracking_heading_kp` | `1.5` | 헤딩 오차 → chi_dot, 1/s |

launch 파일에 지정한 이득이 노드 기본값보다 우선한다.
`simulation.launch.py`, `track_map.launch.py`, `planner.launch.xml`에서
이 파라미터들을 지정할 수 있다. 사용자가 조정한 launch 이득은 유지했다.

`/planner/executed_command`의 `tracking_longitudinal_error`,
`tracking_lateral_error`, `tracking_heading_error`는 계산된 오차이고,
`tracking_speed_correction`, `tracking_heading_rate_correction`은 적용한 P 항이다.
`tracking_enabled=false`여도 실행 중 오차 계산은 수행하고 보정만 끈다.

`planned_speed`, `vx/vy`, `motion_heading_rate`, `yaw_rate`에는 보정을 반영한다.
가속도·jerk·곡률·beta 및 기준 위치/헤딩은 원래 계획 값을 유지하며,
P 항을 시간 차분하여 다음 계획에 주입하지 않는다. 따라서 가속도·jerk 필드는
P 보정 후 명령을 미분한 값이 아니라 계획의 feedforward 값이다.

## 빌드

```bash
cd ~/ros2_ws/src
unzip ~/Downloads/simp_planner_v8.zip
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select \
  planar_velocity_sim simp_planner_msgs simp_planner_cpp simp_planner_tools
source install/setup.bash
```

현재 적용 파라미터와 사용 여부는 최상위 `PARAMETERS_KR.md`를 참고하십시오.
