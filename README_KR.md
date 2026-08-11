# SIMP Local Trajectory Planner v8

본 배포본은 정적 장애물 기반 Path–Velocity Decomposed Planner, Motion–Body Allocation 및 방향성 swept-footprint 충돌검사를 결합한 ROS2 Jazzy용 구성입니다.

## 구성

- `simp_planner/simp_planner_cpp`: C++17 Planner, Allocator, ROS2 runtime
- `simp_planner/simp_planner_msgs`: 사용자 메시지
- `simp_planner_tools`: 시나리오, Costmap, debug plot 및 launch
- `planar_velocity_sim`: `v_x`, `v_y`, yaw-rate 기반 이상적 운동학 simulator
- `validation`: 독립 C++ 폐루프 검증기와 재현 스크립트

## 충돌·재계산 파이프라인

```text
정상 길이 Path 후보 전체 생성
→ 공간 충돌 후보 제거 후 안전 후보 탐색
→ 모두 실패하면 정지 가능한 짧은 Path 생성
→ 시간 궤적에서 동적·종단 제약 검사
→ LATERAL_PRIORITY Allocation + 최종 3-circle swept check
→ 충돌 시 MINIMUM_VY Allocation
→ 그래도 충돌하면 실패 Path/횡목표 제외 후 다음 Path 탐색
→ 남은 후보가 없으면 jerk-limited 안전정지
```

정적 장애물 충돌검사는 Path 공간 단계와 Allocation 이후 최종 차량 형상 단계의 두 단계로 구성됩니다. 시간 궤적 단계에서는 동일 Costmap의 충돌검사를 반복하지 않습니다.

## v8 시나리오 정리

- `narrow_28m_corridor`: 고정 0.20 m Costmap과 3-circle footprint를 사용하는 **정규 협소 통로 통과 시나리오**
- `narrow_22m_stop_corridor`: 통과 불가능한 2.20 m 통로에 대해 **통로 진입 전 안전정지만 확인하는 stop-only 시나리오**
- 기존 이름 `narrow_28m_coarse_corridor`, `narrow_10pct_corridor`는 호환용 alias로만 유지됩니다.

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

## 시뮬레이션 실행 예시

```bash
ros2 launch simp_planner_tools simulation.launch.py \
  scenario:=narrow_28m_corridor \
  target_speed:=3.0
```

## 검증 재현

선택 회귀 26개:

```bash
cd validation
python3 run_validation.py
```

13개 시나리오 정의 전체 35개 사례:

```bash
cd validation
python3 run_all_scenarios.py
```

상세 변경사항은 `CHANGELOG_KR.md`, 검증 결과는 `VALIDATION_KR.md`를 참고하십시오.
