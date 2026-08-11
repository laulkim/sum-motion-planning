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
