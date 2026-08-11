# SIMP Planner v8 검증 보고서

## 1. 변경 검증 목적

v8은 Planner 알고리즘 자체를 변경하지 않고 협소 통로 검증 목적을 명확하게 분리했습니다.

1. 2.80 m 통로를 정규 통과 시나리오로 지정
2. 2.20 m 통로는 통과 성능이 아니라 진입 전 안전정지만 판정
3. 최대 계획시간이 발생한 시점·경로 진행거리·동일 주기 재탐색 횟수를 검증 출력에 추가

## 2. 단위·빌드 검증

| 항목 | 결과 |
|---|---:|
| Python 단위시험 | 28/28 PASS |
| C++17 core/runtime 독립시험 | PASS |
| 검증기 `-Wall -Wextra -Wpedantic` 빌드 | PASS |

현재 실행환경에는 ROS2 Jazzy/DDS가 없어 실제 `colcon build`, executor scheduling, callback jitter 및 노드 통신지연은 검증하지 못했습니다.

## 3. 협소 통로 검증

### 3.1 2.80 m 정규 통과 시나리오

| 시나리오 | 속도 [m/s] | 결과 | 게이트 통과 | 실행 충돌 | 최소 clearance [m] |
|---|---:|---:|---:|---:|---:|
| `narrow_28m_corridor` | 1 | PASS | 1/1 | 0 | 0.1189 |
| `narrow_28m_corridor` | 3 | PASS | 1/1 | 0 | 0.1164 |
| `narrow_28m_corridor` | 5 | PASS | 1/1 | 0 | 0.1170 |

2.80 m 시나리오는 0.20 m Costmap과 3-circle footprint 설정을 유지한 상태에서 1·3·5 m/s 모두 통과했습니다.

### 3.2 2.20 m stop-only 시나리오

2.20 m 통로의 진입부는 경로 진행거리 약 15 m입니다. 판정 조건은 다음과 같습니다.

- swept-footprint 충돌 0회
- 게이트 통과 0회
- 최종 속도 0.05 m/s 이하
- 최종 진행거리 14 m 이하

| 속도 [m/s] | 검증 시작점 [m] | 최종 진행거리 [m] | 최종 속도 [m/s] | 결과 |
|---:|---:|---:|---:|---:|
| 0.2 | 8.0 | 8.049 | 0.000 | 진입 전 안전정지 PASS |
| 1 | 0.0 | 9.654 | 0.000 | 진입 전 안전정지 PASS |
| 3 | 0.0 | 5.456 | 0.000 | 진입 전 안전정지 PASS |

0.2 m/s 사례는 출발점에서 즉시 정지하는 결과를 피하고 실제 통로 전방 정지를 확인하기 위해 경로 진행거리 8 m에서 시작했습니다.

## 4. 전체 회귀검증

| 분류 | 결과 |
|---|---:|
| 시나리오 정의 | 13/13 검증 |
| 정상 주행 사례 | 32/32 PASS |
| 2.20 m 통로 안전정지 사례 | 3/3 PASS |
| 전체 기대동작 | 35/35 PASS |
| 실행 swept-footprint 충돌 | 0회 |
| 선택 회귀검증 | 26/26 PASS |

## 5. 311.12 ms 발생 원인

v7에서 보고된 311.12 ms는 다음 조건에서 발생했습니다.

- 시나리오: `s_curve_obstacles`
- 목표속도: 5 m/s
- 시뮬레이션 시간: 16.5 s
- 경로 진행거리: 약 42.4 m
- 위치 특성: 두 번째 장애물 부근
- 동일 계획 주기 Path 재탐색: 12회

기본 Allocation과 `MINIMUM_VY`가 연속 충돌하면서 실패 Path 후보를 순차 제외하고 12개 후보를 재평가한 주기입니다. 따라서 해당 시간은 일반 Planner 1회가 아니라 **하나의 snapshot에서 다수 Path–Allocation 후보를 연속 재계산한 전체 wall-clock 시간**입니다.

동일 조건을 5회 독립 반복한 결과는 다음과 같습니다.

| 최소 [ms] | 중앙값 [ms] | 최대 [ms] |
|---:|---:|---:|
| 286.14 | 299.83 | 325.92 |

즉, 311.12 ms는 재현 가능한 알고리즘성 worst-case 범위에 포함됩니다.

전체 회귀 실행 중 `narrow_28m_corridor @ 3 m/s`에서 499.12 ms의 단발성 raw 값이 관측되었으나, 해당 주기는 Path 재탐색 0회였고 독립 5회 반복 결과가 25.15~37.31 ms였으므로 OS scheduling 또는 실행환경 부하에 의한 outlier로 판단했습니다. 원시 결과는 숨기지 않고 `all_scenarios_results.csv`에 유지했습니다.

## 6. 실시간성 판단

기능 검증은 통과했지만 10 Hz hard real-time deadline은 아직 보장하지 않습니다. 실시간성 병목은 일반 경로 생성보다 Allocation 충돌 후 여러 Path 후보를 같은 snapshot에서 순차 재검증하는 경우입니다.

검증 출력에는 다음 값이 추가되었습니다.

- `maximum_plan_time_s`
- `maximum_plan_progress_m`
- `maximum_plan_replans`

## 7. 결과 파일

- `validation/selected_regression_results.csv`
- `validation/selected_regression_summary.json`
- `validation/all_scenarios_results.csv`
- `validation/all_scenarios_summary.json`
- `validation/performance_repeat_results.csv`
- `validation/run_validation.py`
- `validation/run_all_scenarios.py`
