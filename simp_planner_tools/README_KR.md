# SIMP Planner Tools

입력 시나리오, reference path/costmap 생성, debug plot 및 통합 launch를 제공합니다.

## 지원 시나리오

```text
stadium
crab_switch
reverse_switch
s_curve
obstacle_avoidance
s_curve_obstacles
alternating_gate_corridor
curved_gate_maze
winding_obstacle_course
narrow_28m_corridor
narrow_22m_stop_corridor
narrow_offset_corridor
terminal_safe_region
```

## ideal / noisy P OFF / noisy P ON 비교

```bash
ros2 run simp_planner_tools run_simulation_comparison \
  --scenario stadium --target-speed 4.0 --duration 100 --show
```

기존 ideal/noisy 플롯 2개를 유지하고, 각 실행의 **자기 목표 − odometry 상태**를
비교하는 창 4개를 추가한다.

- `tracking_errors.png`: 종방향 위치 오차(m), 횡방향 위치 오차(m), 차체 헤딩 오차(deg)의
  시간 그래프. 각 축에 ideal(P ON), noisy P OFF, noisy P ON을 겹쳐 표시한다.
- `tracking_error_metrics.png`: 같은 오차의 RMSE와 최대 절댓값을 비교한다.
- `ideal_target_feedback.png`, `noisy_target_feedback.png`: 각 P ON 실행의
  목표/odom X, Y, 차체 yaw를 왼쪽에 겹쳐 표시하고, 오른쪽에는 종·횡·헤딩 오차를 표시한다.
  위치 오차는 지도 X/Y 차이가 아니라 **목표 이동 방향 기준**으로 분해한다.
  헤딩은 ±180° 경계에서 가짜 360° 차이가 생기지 않도록 같은 각도 가지로 표시한다.
  이 두 그림은 각 실행의 전체 기록 구간을 표시한다.

기존 플롯의 noisy는 P ON 실행이다. ideal(P ON), noisy P ON, noisy P OFF를 순서대로
실행하므로 `--duration 100`은 **각 100초, 총 300초 + 시작/종료 시간**이다.
게인은 현재 `simulation.launch.py` 설정을 그대로 사용한다. 두 noisy 실행은 같은
시나리오·목표 속도와 모델의 기본 seed(42)를 사용하고, P 적용 여부만 다르게 지정한다.
독립적인 ROS 실행이므로 콜백 타이밍까지 완전히 같지는 않다.

새 평가는 명령 메시지의 목표 위치/차체 yaw와 odometry pose를 원본 header timestamp로
맞춘다. 계획 변경 시에도 각 명령에 해당하는 목표를 사용하며, 목표가 없는 안전정지·대기
구간은 0 오차로 채우지 않고 그래프에서 제외한다. 세 실행의 공통 경과 시간 구간으로
통계를 계산하고 유효 샘플 수를 `tracking_metrics.json`에 저장한다.
현재 noisy 모델의 odometry pose를 평가하며, 제어기가 내부적으로 예측한 pose 오차와는
구분된다. 새 timestamp/목표 열이 필요하므로 새 그래프에는 수정 후 기록한 로그를 사용한다.

## 2.80 m 정규 협소 통로

```bash
ros2 launch simp_planner_tools simulation.launch.py \
  scenario:=narrow_28m_corridor \
  target_speed:=3.0
```

고정 0.20 m Costmap과 3-circle footprint를 사용하며, 물리 통로 폭은 2.80 m입니다. reference path는 통로 중심에서 약 0.25 m 편향되고 작은 heading mismatch를 포함합니다.

## 2.20 m 안전정지 전용 통로

```bash
ros2 launch simp_planner_tools simulation.launch.py \
  scenario:=narrow_22m_stop_corridor \
  target_speed:=1.0
```

2.20 m 통로는 현재 안전 형상에서 통과 불가능한 한계 조건입니다. 평가 목적은 통로 통과가 아니라 진입 전 충돌 없는 안전정지입니다.

## 2.40 m 장거리 통로 + 편향 reference path

```bash
ros2 launch simp_planner_tools simulation.launch.py \
  scenario:=narrow_offset_corridor \
  target_speed:=1.0
```

물리 통로는 global `y=0`을 중심으로 70 m 직선 구간이며, reference path는 통로 내부에서 약 `+0.25 m` 편향됩니다. 이 시나리오는 0.05 m Costmap과 16-circle footprint를 사용합니다.
