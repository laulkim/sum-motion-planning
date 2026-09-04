# SIMP Planner Tools

입력 시나리오, reference path/costmap 생성, debug plot 및 통합 launch를 제공합니다.

## 현실화 시뮬레이션 디버그 출력

`simulation.launch.py`는 같은 저장 주기마다 상태추정값과 ground truth를
각각 렌더링합니다.

```text
snapshot_estimate_000010.png
snapshot_ground_truth_000010.png
latest_estimate.png
latest_ground_truth.png
```

두 이미지는 같은 `scenario/session_id` 디렉터리에 저장됩니다. Estimate
이미지는 Planner 입력인 `/odom`, ground-truth 이미지는
`/sim/ground_truth/odom`을 사용합니다. 경로, 선택 궤적과 Planner 명령은
두 이미지에서 동일하고 차량 위치, 속도 및 이들로 계산한 추종 오차가
서로 다릅니다.

### 종료된 실행의 GT-추정 오차 분석

각 세션에 저장된 두 odometry CSV를 시간 정렬하고 확대 가능한 그래프로
보려면 다음 명령을 사용합니다.

```bash
python3 ~/ros2_ws/src/sum-motion-planning/\
simp_planner_tools/simp_planner_tools/estimation_error_plot.py \
  ~/Desktop/simp_planner/simp_planner_debug/winding_obstacle_course/20260904_153233
```

마우스 휠로 포인터 주변을 확대하고 Matplotlib 도구 모음으로 영역 확대,
이동 및 원래 화면 복귀가 가능합니다. 시간 그래프 위에서 마우스를 움직이면
해당 시점의 위치, yaw, 속도 및 global lateral 오차가 하단에 표시됩니다.

기본값은 같은 수신 시각의 값을 비교하므로 estimator delay가 포함됩니다.
`--alignment source`를 지정하면 각 logger가 정렬한 message header 시간축을
사용합니다. 실행할 때 다음 파일도 세션 폴더에 생성됩니다.

```text
estimation_error_comparison.csv  # 시간 정렬된 상세 수치
estimation_error_overview.png
```

PlotJuggler 없이 Python과 Matplotlib만 사용합니다. GUI 없이 파일만 만들려면
`--no-show`를 추가합니다. 패키지를 빌드한 뒤에는 아래 명령도 같습니다.

```bash
ros2 run simp_planner_tools estimation_error_plot <세션 폴더>
```

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
