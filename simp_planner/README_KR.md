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

`runtime.cpp::apply_tracking_feedback()`가 실행 주기(기본 100 Hz)마다 선택된
시간 궤적의 현재 목표 위치와 `/odom`의 위치를 비교한다. 기준 방향은 차체 yaw가
아닌 궤적의 진행 방향 `chi_ref`이며, 후진·좌우 이동에도 같은 오차 부호를 쓴다.

```text
dx = x_ref - x_measured
dy = y_ref - y_measured
e_long =  cos(chi_ref) * dx + sin(chi_ref) * dy
e_lat  = -sin(chi_ref) * dx + cos(chi_ref) * dy
e_chi  = wrap(chi_ref - (body_yaw_measured + beta_ref))

speed_target   = speed_ff + K_long * e_long
chi_dot_target = speed_cmd * kappa_ref + K_lat * e_lat + K_heading * e_chi
vx_cmd         = speed_cmd * cos(beta)
vy_cmd         = speed_cmd * sin(beta)
yaw_rate_cmd   = chi_dot_cmd - beta_rate
```

헤딩 피드백은 odometry pose의 yaw를 사용한다. `e_chi`는
`wrap((chi_ref - beta_ref) - body_yaw_measured)`, 즉 목표 차체 yaw와 측정 yaw의
차이와 같다. 계획 초기 상태의 진행 방향에는 속도 기반 `atan2(vy, vx)`를 계속 쓰지만,
헤딩 P 항에는 이 속도 방향 추정 잡음을 직접 넣지 않는다.
오차는 원래 reference path 중심이 아닌 **장애물 회피를 포함한 선택 궤적**을 기준으로
계산한다. odometry timestamp에서 명령 시각까지 속도·진행 각속도로 짧게 예측한
위치를 사용한다. 다음 plan의 handover 상태도 같은 피드백으로 예측한다.

P 목표에는 보정량 상한을 적용하고, 실제 명령에는 종가속도·jerk 및 yaw
가속도·jerk 여유를 사용한 변화율 제한을 적용한다. 속력은 음수가 되지 않으며,
최대 속력·진행 각속도·차체 yaw rate·횡가속도 상한도 적용한다. 정지에 가까워지면
종방향 보정 여유를 줄이고, 각속도 P 항은 정지 임계값부터
`AllocationLimits::allocation_active_speed`(0.30 m/s)까지 서서히 활성화한다.
기준 속력이 정지 임계값 이하이면 보정을 끈다.
정지/포화 상한은 변화율 제한보다 우선한다. 안전정지·모드 전환 정지·terminal hold는
추종 보정을 적용하지 않는다. odometry가 timeout을 넘으면 기존 feedforward를 사용한다.

다음 ROS 파라미터는 노드 시작 시 읽는다.

| 파라미터 | 기본값 | 의미 |
|---|---:|---|
| `tracking_enabled` | `true` | P 피드백 활성화 |
| `tracking_longitudinal_kp` | `0.8` | 종방향 위치 오차 → 속력 보정, 1/s |
| `tracking_lateral_kp` | `0.6` | 횡방향 위치 오차 → 진행 각속도 보정, rad/(m·s) |
| `tracking_heading_kp` | `1.5` | 진행 헤딩 오차 → 진행 각속도 보정, 1/s |
| `tracking_max_speed_correction_mps` | `0.5` | 속력 보정 절댓값 상한 |
| `tracking_max_heading_rate_correction_radps` | `0.35` | 횡·헤딩 P 항 합의 절댓값 상한 |
| `tracking_stop_speed_threshold_mps` | `0.03` | 기준 속력이 이 값 이하이면 보정 중지 |
| `tracking_odom_timeout_sec` | `0.25` | 피드백에 사용할 odometry 최대 경과 시간 |

```bash
ros2 launch simp_planner_tools simulation.launch.py \
  kinematics_model:=noisy \
  tracking_longitudinal_kp:=0.8 \
  tracking_lateral_kp:=0.6 \
  tracking_heading_kp:=1.5
```

`simulation.launch.py`, `track_map.launch.py`, `planner.launch.xml`에서 활성화 여부와
세 P 이득을 지정할 수 있다. `tracking_enabled:=false`로 feedforward 실행과 비교한다.
나머지 파라미터는 노드를 직접 실행할 때 `--ros-args -p 이름:=값`으로 지정한다.

`/planner/executed_command`의 `tracking_active`, `tracking_longitudinal_error`,
`tracking_lateral_error`, `tracking_heading_error`, `tracking_speed_correction`,
`tracking_heading_rate_correction`으로 오차와 적용된 보정량을 확인한다.
마지막 두 필드는 포화·변화율 제한을 거친 **최종 명령 − feedforward**이다.
속력 변경에 따른 곡률 feedforward 변화도 각속도 보정량에 포함된다.
`planned_speed/acceleration/jerk`, `motion_heading_rate/acceleration`, `yaw_acceleration`은
보정된 명령 및 보정량의 시간 차분을 반영한다. `motion_heading`과 segment 위치는
오차 계산에 사용한 기준 궤적을 유지한다.

Allocation의 전체 궤적 충돌검사는 feedforward 궤적에 대한 검사다. 실행 중 P 보정으로
실제 이동 궤적이 달라질 수 있으므로, 이득·보정 상한은 차량 응답과 장애물 여유에 맞춰
조정해야 한다. 추가 테스트는 네 drive mode에서 위치·헤딩 수렴 및 1차 응답 지연을
검증하며, 실제 차량이나 모든 noisy 시나리오에 대한 안정성 보장을 뜻하지 않는다.

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
