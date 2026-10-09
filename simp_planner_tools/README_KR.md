# Ctrl+C 후 추종 그래프

`simulation.launch.py`는 `tracking_shutdown_plot` 기록 노드도 실행합니다.
시뮬레이션을 실행한 뒤 Ctrl+C를 누르면 기존 추종 그래프 3개와 spot-turn 요약창,
회전 구간별 확대창을 표시합니다. 기록과 PNG도 함께 저장합니다.

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select simp_controller simp_planner_tools
source install/setup.bash
ros2 launch simp_planner_tools simulation.launch.py \
  scenario:=spot_turn_course use_rviz:=true target_speed:=4.2 \
  costmap_resolution:=0.1 debug_output_dir:=/home/a/ros2_ws/simp_planner_debug \
  sensor_delay_sec:=0.0
```

저장 위치는 `debug_output_dir/tracking_YYYYMMDD_HHMMSS_microseconds/`입니다.
`debug_output_dir`의 기본값은 `/home/sum/Desktop/simp_planner/simp_planner_debug`입니다.

| 파일 | 내용 |
| --- | --- |
| `tracking.csv` | 제어 주기별 기준점, 예측 위치, 오차, 실측 속도, 출력 명령, 원본 timestamp, 유효 여부 |
| `tracking_errors.png` | 차체 좌표계 종방향 `ex`, 횡방향 `ey`, yaw 오차 |
| `tracking_states_inputs.png` | 기준/예측 x·y·yaw 및 기준/실측/명령 vx·vy·omega |
| `tracking_world_errors.png` | world frame의 reference − predicted 위치/yaw 오차 |
| `odometry.csv` | `/odom` 취득 timestamp, 측정 x·y·yaw와 body velocity |
| `drive_mode.csv` | `/vehicle/drive_mode_state` timestamp, current/requested mode와 status |
| `spot_turn_summary.csv` | 회전별 명령·이동·yaw·회전 후 지표, 측정 표본 수와 시간 범위 |
| `spot_turn_summary.png` | 회전별 지표 요약표; 회전이 없으면 안내 표시 |
| `spot_turn_01.png` 등 | 회전별 4×2개 확대 패널 |
| `viewer.log` | 종료 후 뷰어의 출력과 오류 |

## Spot-turn 분석 화면과 지표

선택된 Planner reference가 정확히 `vx=0, vy=0, yaw_rate!=0`인 유효 제어 구간을 찾습니다.
mode 기록이 있으면 제어 시각 이하의 timestamp를 가진 최신 상태가 SPOT_TURN/READY인 구간만 분석합니다.
움직이면서 회전하는 일반 곡선과 yaw-rate가 zero인 hold는 제외합니다.
각 구간의 시작을 시간 zero로 맞춰 두 브랜치의 창과 수치를 비교할 수 있습니다.

회전별 확대창은 다음 순서의 4×2 패널입니다.

| 왼쪽 | 오른쪽 |
| --- | --- |
| vx reference/command/측정값 | vy reference/command/측정값 |
| yaw-rate reference/command/측정값 | controller yaw 오차 |
| 첫 odom 표본 대비 XY 중심 이동 | 중심 거리와 누적 이동 거리 |
| 회전 후 위치 오차 norm | 회전 후 reference/command/측정 병진 속도 norm |

요약창의 command integral은 `∫hypot(vx_cmd,vy_cmd) dt`이며,
명령이 다음 제어 시각까지 유지된다는 zero-order hold로 계산합니다.
중심 이동은 predicted pose로 추정하지 않고 raw odom의 **취득 timestamp**로 구간을 맞춥니다.
최대 중심 변위와 최종 변위를 함께 표시하고, odom 위치 표본 사이 이동 거리를 누적합니다.
이동 후 시작점으로 돌아오면 최종 변위만 zero가 되고 최대 변위/누적 거리는 남습니다.
시작 기준은 해당 구간에서 확보한 첫 odom 표본입니다. 표본 시간 범위/수도 CSV에 기록하므로
센서 지연이나 메시지 누락으로 측정 범위가 짧은 경우 확인할 수 있습니다.
`sensor_delay_sec=0`인 현재 simulator에서는 raw odom이 그 표본 시각의 ground truth입니다.
측정 곡선은 odom 시각, command/reference와 controller error는 제어 주기 시각에 표시됩니다.

회전 후 분석은 최대 3초이며 다음 회전이 시작되면 잘라냅니다.
ALIGNING과 다음 정상 주행이 포함될 수 있으므로 command peak를 settling time이나 overshoot로
자동 판정하지 않습니다. reference speed를 함께 보면서 보정 출력과 정상 주행을 구분합니다.
창과 CSV에서 위치 오차는 계속 reference − predicted 기준이고 측정 중심 이동과 별개입니다.
추가 지표는 yaw 최대 오차/RMSE, 회전 중 vx/vy command peak/RMS,
회전 후 position RMSE와 병진 command peak입니다.

기록 시작/끝이 회전 중이면 `PARTIAL`/`*`로 표시합니다. 회전이 끝난 뒤 Ctrl+C를 누르면
회전 후 3초 구간도 확인할 수 있습니다. 예전 CSV에 mode 기록이 없으면 `ref only`로 표시하며,
raw odom이 없거나 표본이 2개 미만이면 이동 지표는 `--`입니다.
이 분석은 Planner/controller/simulator의 동작을 변경하지 않고 기존 topic만 구독합니다.

양 브랜치 전환 후 위 build/source 명령을 실행해야 C++ controller와 Python 도구가 해당 브랜치와 맞습니다.

브랜치 전환 뒤 `TrackingControl` import 오류가 나면 메시지 생성물 캐시를 갱신한 뒤 다시 빌드합니다.

```bash
colcon build --symlink-install --packages-select simp_planner_msgs --cmake-clean-first --cmake-force-configure
colcon build --symlink-install --packages-up-to simp_planner_tools
source install/setup.bash
```

화면이 없는 환경에서는 창을 띄우지 않고 PNG만 저장합니다. 기록이 없으면 뷰어를 실행하지 않습니다.
Tk 창에는 `python3-tk`가 필요하며 패키지 실행 의존성에 선언되어 있습니다.
기록한 CSV를 다시 열 수도 있습니다.

```bash
python3 -m simp_planner_tools.tracking_shutdown_plot --show /저장경로/tracking.csv
```

## 기존 브랜치의 구현

`refactor/tracking-before-diagnostics`에서는 `simp_tracker`가 제어 주기마다
`TrackingControl`을 `/tracker/control`로 발행했습니다. 메시지 하나에 실제 선택한 기준점,
그 기준점 시각으로 예측한 odometry, 제어에 사용한 오차, 실측 차체 속도와 출력 명령을 담았습니다.
`tracking_shutdown_plot`은 이 메시지를 CSV에 기록하고 Ctrl+C 후 파일을 닫았습니다.
이후 `start_new_session=True`로 별도 Python 뷰어를 실행해 launch의 종료 신호와 창을 분리했습니다.
뷰어는 오차 3개 축과 상태·입출력 3×2개 축을 그려 저장하고 사용자가 창을 닫을 때까지 유지했습니다.

## develop에 적용한 구현

기존 브랜치의 `TrackingControl` 메시지와 기록/뷰어 코드를 재사용했습니다.
`develop`의 `simp_controller`는 이미 기준점 시각으로 위치를 예측하고 추종 오차를 계산하므로,
그 제어 주기의 `Tracking` 결과에 선택한 기준점을 보관하고 실제 출력과 함께 `/tracker/control`로 발행합니다.
기록 노드는 기준점을 다시 선택하거나 오차를 재계산하지 않습니다.
제어식, 기준점 선택, 예측 방식, 제어 주파수는 그대로 사용합니다.

그래프의 위치는 선택된 기준점 시각의 **예측 위치**이고, 실측 속도는 odometry 시각의 값입니다.
오차는 `reference - predicted`를 예측 차체 좌표계로 회전한 `ex`, `ey`와 wrap한 yaw 차이입니다.
CSV 시간축은 최초 제어 메시지 시각부터의 경과 시간이며 각 원본 timestamp도 별도 열에 보존합니다.
제어 입력이 무효인 주기는 `valid=0`으로 기록하고 상태/오차를 NaN으로 남기되 실제 zero command는 보존합니다.
각도는 CSV에서는 rad, 그래프에서는 deg를 사용합니다.

기존 주기적 디버그 PNG와 별도로 기록합니다. 디버그 렌더링 워커는 SIGINT를 무시하고,
부모 노드는 종료 중 중복 SIGINT와 이미 종료된 ROS context를 처리해 저장을 마무리합니다.
`track_map.launch.py`에는 현재 컨트롤러가 없으므로 자동 연결은 `simulation.launch.py`에 적용했습니다.
다른 실행 구성에서는 컨트롤러와 함께 `ros2 run simp_planner_tools tracking_shutdown_plot`을 실행하면 됩니다.

## 검증

- C++ 제어 테스트: 선택한 기준점과 예측/오차 결과가 동일 제어 주기의 것인지 확인.
- Python 그래프 테스트: 오차 원본 보존, 무효 주기의 NaN/zero command, 단일 샘플/기존 CSV 표시.
- Spot-turn 테스트: 취득 timestamp 구간 분리, 미래 mode 배제, zero-order hold 적분,
  이동 후 복귀하는 경우, partial/측정 누락, raw odom 패널 확인.
- ROS 통합 테스트: 실제 controller·odom·mode → CSV 기록 → SIGINT 종료 → 기존/추가 PNG와 요약 CSV 생성.

기존 전체 `spot_turn_course` 검증과 별도로, 추가 분석의 SIGINT 경로는 ROS 프로세스와
headless 렌더링으로 검증합니다. GUI 창은 headless 검증에서 직접 확인하지 않습니다.
기존 HDMap의 `osgeo` 누락과 다른 노드의 중복 ROS shutdown traceback은 별도 문제로 남아 있습니다.
