# Ctrl+C 후 추종 그래프

`simulation.launch.py`는 `tracking_shutdown_plot` 기록 노드도 실행합니다.
시뮬레이션을 실행한 뒤 Ctrl+C를 누르면 두 Matplotlib 창을 표시하고 PNG를 저장합니다.

```bash
cd /home/sum/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --cmake-force-configure --packages-up-to simp_planner_tools
source install/setup.bash
ros2 launch simp_planner_tools simulation.launch.py scenario:=spot_turn_course
```

저장 위치는 `debug_output_dir/tracking_YYYYMMDD_HHMMSS_microseconds/`입니다.
`debug_output_dir`의 기본값은 `~/Desktop/simp_planner/simp_planner_debug`입니다.

| 파일 | 내용 |
| --- | --- |
| `tracking.csv` | 제어 주기별 기준점, 예측 위치, 오차, 실측 속도, 출력 명령, 원본 timestamp, 유효 여부 |
| `tracking_errors.png` | 차체 좌표계 종방향 `ex`, 횡방향 `ey`, yaw 오차 |
| `tracking_states_inputs.png` | 기준/예측 x·y·yaw 및 기준/실측/명령 vx·vy·omega |
| `viewer.log` | 종료 후 뷰어의 출력과 오류 |

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
- Python 그래프 테스트: 오차 원본 보존, 무효 주기의 NaN/zero command, 단일 샘플 PNG 저장 확인.
- ROS 통합 테스트: 실제 컨트롤러 → CSV 기록 → SIGINT 종료 → PNG 두 장 생성 확인.

전체 `spot_turn_course` launch에서도 SIGINT 후 두 추종 PNG와 기존 디버그 PNG가 생성되고
두 플롯 노드가 정상 종료하는 것을 확인했습니다. GUI 창은 headless 검증에서 직접 확인하지 않았습니다.
기존 HDMap의 `osgeo` 누락과 다른 노드의 중복 ROS shutdown traceback은 별도 문제로 남아 있습니다.
