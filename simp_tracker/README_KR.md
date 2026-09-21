# SIMP Tracker — Python prototype

플래너의 시간 궤적과 실제 odometry를 받아 차체 속도 명령을 만드는 독립 ROS 2 패키지입니다.
제어 코드는 `simp_tracker/tracking_controller_node.py` 한 파일에 있습니다.

## 논문에서 적용한 부분

Ao et al., *Structure Design and Event-Triggered Control of a Modular Omnidirectional
Mobile Chassis of Life Support Robotics*, Fractal Fract. 2023, 7, 121.
DOI: https://doi.org/10.3390/fractalfract7020121

사용자 제공 `fractalfract-07-00121.pdf`의 4.1절 식 (10), (14), (16), (18)을
참고했습니다. 차륜 구조·차륜별 명령 분배·ETM/STM은 구현하지 않습니다.

현재 차체 yaw를 θ, reference 차체 yaw를 θr로 두면:

```text
ex =  cos(θ) (xr - x) + sin(θ) (yr - y)
ey = -sin(θ) (xr - x) + cos(θ) (yr - y)
eθ = wrap(θr - θ)

vx_cmd = vxr cos(eθ) - vyr sin(eθ) + kx ex
vy_cmd = vyr cos(eθ) + vxr sin(eθ) + ky ey
ω_cmd  = ωr + kθ eθ

V    = 0.5 (ex² + ey² + eθ²)
Vdot = -kx ex² - ky ey² - kθ eθ²
```

기본 gain은 논문 5절의 `kx=3`, `ky=4`, `k_yaw=2`입니다. 양수여야 합니다.
위 감소식은 연속시간의 이상적 속도 추종과 일관된 reference를 전제로 합니다.
이 프로토타입은 샘플링·선형 보간·각도 wrap·출력 포화를 사용하므로 실제 구현에
논문의 연속시간 안정성 증명을 그대로 적용했다고 주장하지 않습니다.

## 연결 및 시간 계약

```text
Planner ─ /planner/trajectory ──────────→ Tracker ─ /cmd_vel → Simulator
        └ /planner/executed_command ────→    ↑                    │
                                           └──── /odom ────────┘
Vehicle ─ /vehicle/drive_mode_state ─────→ Tracker
```

- `Trajectory.start_time`과 동일한 ROS clock의 현재 시각 차로 reference를 선택합니다.
  위치·속도는 선형 보간, yaw는 ±π 경계를 고려한 최단각 보간을 사용합니다.
- 목표 `vx/vy`는 목표 차체 축 기준입니다. 식 (14)에서 현재 차체 축으로 회전합니다.
- 좌표계는 trajectory와 odom이 같아야 하고 odom child frame은 `base_link`여야 합니다.
  TF 변환을 추측하지 않습니다. `base_frame` 파라미터로 다른 차체 프레임을 지정할 수 있습니다.
- 계획 ID가 증가하는 새 궤적만 수용하며 활성 계획과 다음 계획, 최대 2개를 보관합니다.
  실행 heartbeat의 plan_id와 일치하는 계획만 사용합니다. 미래 계획은 시작 전에 실행하지
  않으며 horizon 이후로 extrapolate하지 않습니다. ROS 시간이 역행하면 캐시를 초기화합니다.
- 플래너만 재시작하여 plan_id가 초기화되면 Tracker도 함께 재시작해야 합니다.

`ExecutedCommand.execution_state`를 추가해 실행 상태·plan_id·명령을 한 메시지로
전달합니다. 기존 별도 String 상태 토픽과 명령 토픽의 수신 순서에 의존하지 않습니다.
이 메시지는 여전히 **플래너의 nominal 명령**이며 실제 제어기 출력은 `/cmd_vel`입니다.

| 실행 상태 | Tracker 출력 |
|---|---|
| ACTIVE_PLAN | 논문 식 (14)의 추종 제어 |
| SAFETY_STOP / MODE_STOP | 플래너의 정지 명령 전달 |
| SPOT_TURN_ROTATING | SPOT_TURN 모드 확인 후 플래너 회전 명령 전달 |
| TERMINAL_HOLD / MODE_WAIT / IDLE / 나머지 | 0 속도 |

모드 정렬 중, 오래된 heartbeat/odom/모드 피드백, 계획 불일치·만료 시 0을 출력합니다.
플래너/제어기 중단 시 이전 속도가 유지되지 않도록 Simulator에도 0.5초 명령 timeout을
추가했습니다. 이때의 정지는 이상적 시뮬레이터의 즉시 정지이며 물리 차량의 제동 모델이 아닙니다.

## 실행

workspace 루트에서:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select \
  simp_planner_msgs simp_planner_cpp planar_velocity_sim simp_tracker simp_planner_tools
source install/setup.bash
ros2 launch simp_planner_tools simulation.launch.py scenario:=s_curve \
  use_tracking_controller:=true command_frequency_hz:=100.0 control_frequency_hz:=50.0
```

통합 launch에서는 Tracker가 기본 활성화되며 `/cmd_vel`의 유일한 발행자입니다.
플래너 원래 Twist는 `/planner/cmd_vel`로 remap됩니다.
`use_tracking_controller:=false`로 기존 직접 연결을 실행할 수 있습니다.
launch gain 인자는 `tracking_kx`, `tracking_ky`, `tracking_k_yaw`입니다.

Tracker만 실행할 때는 다음 명령을 사용합니다. 이 경우에도 Planner의 `/cmd_vel`을
반드시 `/planner/cmd_vel`로 remap해야 합니다.

```bash
ros2 run simp_tracker tracking_controller_node --ros-args \
  -p control_frequency_hz:=50.0 -p kx:=3.0 -p ky:=4.0 -p k_yaw:=2.0
```

| Tracker 파라미터 | 기본값 |
|---|---:|
| control_frequency_hz | 50 Hz |
| max_linear_speed | 5 m/s, 평면 속도 크기 제한 |
| max_yaw_rate | 1 rad/s |
| odom_timeout_sec / command_timeout_sec | 각각 0.25초 |
| mode_timeout_sec | 0.5초 |

`/tracker/reference`는 현재 보간한 목표 pose, `/tracker/error`는 현재 차체 축의
`ex[m], ey[m], eθ[rad]`, `/tracker/status`는 추종/대기/정지 이유입니다.

## 검증과 범위

```bash
colcon test --packages-select simp_tracker --event-handlers console_direct+
ROS_DOMAIN_ID=90 ROS_LOG_DIR=/tmp/simp_tracker_ros_logs \
  python3 src/sum-motion-planning/simp_tracker/test/check_tracking_ros.py
```

테스트는 식 (18)의 감소식, 논문 식 (28)의 figure-eight 궤적 오차 수렴,
20/50/100 Hz 계산 주기, 후진·횡이동, yaw 경계, 지연 수신, 계획 교체·만료,
상태 timeout, 모드 전환 및 기존 정지/회전 명령 전달을 검사합니다.
ROS 통합 스크립트는 초기 위치·방향 오차가 있는 횡이동에서 실제 오차 감소,
단일 command 발행, 모드 전환/제자리 회전, heartbeat 중단 및 Tracker 종료 시
Simulator timeout을 확인합니다. 사용하지 않는 ROS domain에서 실행합니다.

플래너는 기존처럼 최초 odometry 이후 계획 예측 상태를 재계획 기준으로 사용합니다.
Tracker는 매 주기 최신 odometry로 오차를 보정합니다. 플래너의 실측 상태 기반
재계획은 이번 구현에서 바꾸지 않았습니다. 큰 오차의 복귀 경로는 장애물 검증되지
않으며, 제어 보정에는 가속도·저크·차륜 조향 한계가 포함되지 않습니다.
이는 현재 차체 속도 적분 시뮬레이터용 프로토타입입니다.
