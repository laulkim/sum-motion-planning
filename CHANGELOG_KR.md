# SIMP Planner v8 변경사항

## 1. 협소 통로 시나리오 재정의

- `narrow_28m_corridor`를 정규 협소 통로 통과 시나리오로 지정했습니다.
- 2.80 m 통로를 1·3·5 m/s에서 검증하도록 확대했습니다.
- `narrow_22m_stop_corridor`는 통과 성능 평가에서 제외하고 통로 진입 전 안전정지 전용 시나리오로 분리했습니다.
- 2.20 m stop-only 판정에 최종 속도, 게이트 미통과 및 정지 위치 범위를 명시했습니다.
- 저속 0.2 m/s 시험은 경로 진행거리 8 m에서 시작하여 실제 통로 전방 정지를 확인합니다.

## 2. 시나리오 명칭 정리

- `narrow_28m_coarse_corridor` → `narrow_28m_corridor`
- `narrow_10pct_corridor` → `narrow_22m_stop_corridor`

기존 명칭은 외부 설정 호환을 위해 alias로 유지됩니다.

## 3. 계획시간 진단 강화

검증기 CSV에 최대 계획시간 발생 조건을 추가했습니다.

- 최대 계획시간이 발생한 시뮬레이션 시간
- 경로 진행거리
- 해당 계획 주기의 Path 재탐색 횟수

v7의 311.12 ms는 `s_curve_obstacles @ 5 m/s`, 시뮬레이션 16.5 s, 진행거리 42.4 m에서 Path 후보를 12회 재탐색할 때 발생한 것으로 확인했습니다.

## 4. 검증 요약

- Python 단위시험: 28/28 PASS
- 독립 C++17 core/runtime 시험: PASS
- 2.80 m 통로: 1·3·5 m/s 3/3 PASS
- 2.20 m 통로 진입 전 안전정지: 0.2·1·3 m/s 3/3 PASS
- 선택 회귀검증: 26/26 PASS
- 전체 13개 시나리오, 35개 사례: 35/35 기대동작 PASS
- 실행 swept-footprint 충돌: 0회

Planner 및 Allocation의 충돌 재계산 알고리즘은 v7과 동일합니다.
