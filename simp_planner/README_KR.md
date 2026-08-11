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

검증 결과는 최상위 `VALIDATION_KR.md`를 참고하십시오.
