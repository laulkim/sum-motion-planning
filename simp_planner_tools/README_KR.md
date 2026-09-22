# SIMP Planner Tools

입력 시나리오, reference path/costmap 생성, debug plot 및 통합 launch를 제공합니다.

통합 launch는 기본적으로 Python `simp_tracker`를 포함합니다. 플래너의 nominal
속도 토픽은 `/planner/cmd_vel`, 제어기 최종 출력은 `/cmd_vel`입니다.
`use_tracking_controller:=false`로 기존 직접 연결을 사용할 수 있습니다.
제어기는 기본 50 Hz, 플래너 명령은 100 Hz이며 `control_frequency_hz`와
`command_frequency_hz`로 조정합니다. 제어식과 파라미터는
[`simp_tracker/README_KR.md`](../simp_tracker/README_KR.md)를 참고하십시오.

## 차량 기준 local costmap

Scenario Manager는 odom을 받은 뒤 캡처 시점 차량 Body 축에 정렬된
`60 m × 60 m` local costmap을 5 Hz로 발행합니다. 메시지 frame은 `odom`이고
timestamp는 grid 생성에 사용한 odom timestamp와 같습니다.
`OccupancyGrid.info.origin`은 차량 위치가 아니라 회전된 grid의 좌하단 corner이며,
position과 orientation을 함께 적용하면 셀 좌표가 캡처 시점의 odom 좌표로 복원됩니다.
Debug map에는 local costmap 셀과 별도로 시나리오의 모든 장애물을 붉은 폴리곤으로
항상 표시하고, local costmap 외곽은 청록색 점선과 `60 × 60 m (body ±30 m)`
라벨로 표시합니다.

30 m 반경과 겹치지 않도록 Planner의 기본 spatial preview는 `28 m`로 설정합니다.

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
## 목표 궤적과 실제 위치 비교 (Tracker ON/OFF 공통)

`debug_plot_node`는 `/planner/trajectory`와 `/planner/executed_command`를 받아,
`/odom.header.stamp`와 같은 시각의 활성 계획 목표 위치를 보간합니다.
Tracker 전용 진단 토픽을 사용하지 않으므로 Tracker를 꺼도 같은 기준으로 비교합니다.
실행 명령의 `plan_id`에 해당하는 계획을 사용하며, 새 계획 수신만으로 목표를 바꾸지 않습니다.

워크스페이스에서 필요한 패키지를 빌드한 뒤 실행합니다.

```bash
cd /home/sum/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select \
  simp_planner_msgs simp_planner_cpp planar_velocity_sim simp_tracker simp_planner_tools
source install/setup.bash

# ON 실행을 종료한 뒤 false로 바꾸어 OFF 결과를 별도로 기록합니다.
ros2 launch simp_planner_tools simulation.launch.py \
  scenario:=s_curve use_tracking_controller:=true \
  save_period:=5.0 debug_output_dir:=/tmp/simp_position_plots
```

결과는 `/tmp/simp_position_plots/s_curve/<실행시각>/`에 저장됩니다.

Ctrl+C로 종료하면 마지막으로 수신한 odometry까지 포함한 Matplotlib 창 두 개가 뜹니다.

- 목표/실제 비교 창: x, y, yaw, yaw rate (계획 목표와 시뮬레이터 `/odom`).
- 오차 창: x, y, yaw, yaw rate 오차와 위치 오차 크기. 각도 오차는 ±180° 경계를 보정합니다.
- `position_comparison.csv`: 종료 시점까지의 전체 비교 데이터. 각도는 rad, 각속도는 rad/s입니다.

비교 그래프는 주기적 PNG에 저장하지 않습니다. 기존 진단 PNG 저장은 유지합니다.
창은 런치와 독립적으로 실행되므로 런치 종료 후에도 확대·이동·저장할 수 있습니다.
데스크톱 디스플레이와 `python3-tk`가 필요하며, 창 실행 오류는 `tracking_viewer.log`에서 확인합니다.
CSV는 다음 명령으로 다시 열 수 있습니다:
`python3 -m simp_planner_tools.tracking_result_viewer <position_comparison.csv 경로>`.

위치는 궤적의 전역 좌표계 기준이며 단위는 m입니다.
오차는 `dx = 실제 x - 목표 x`, `dy = 실제 y - 목표 y`,
거리 오차는 `sqrt(dx² + dy²)`입니다. 차체 축 기준의 `/tracker/error`와는 다릅니다.
정지·모드 전환·계획 시작 전·만료 후·계획 미수신·프레임 불일치 구간이나,
측정 시각 기준 실행 명령이 0.25초보다 오래된 구간은 목표/오차를 NaN으로 기록합니다.
그래프에서도 해당 구간은 끊어 표시하며 실제 위치는 계속 기록합니다.
이미지는 저장 주기마다 갱신되는 파일이며 실시간 GUI 창을 열지는 않습니다.
