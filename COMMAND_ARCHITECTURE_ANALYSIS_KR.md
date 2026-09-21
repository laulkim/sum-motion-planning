# Command/Planning 아키텍처 분석 — pending↔active 전환, 상태 freeze, 노드 분리 설계

`odom-once-handover-state` 브랜치(`simp_planner/simp_planner_cpp/src/planner_node.cpp`)의
`planning_callback()`/`command_callback()` 구조를 분석하고, 트래킹 컨트롤러 대신 "레퍼런스를
그대로 전달하는" 별도 노드를 만들 때 고려해야 할 사항을 정리합니다.

## 0. 배경 — odom-once 구조

오도메트리를 **최초 1회만** `current_state_` 시딩에 쓰고, 그 이후로는 `predict_handover_state()`가
현재 활성 계획(`active_plan_`)의 궤적을 계속 적분(dead-reckoning)해서 다음 계획의 시작 상태를
계산합니다 ([planner_node.cpp:348-402](simp_planner/simp_planner_cpp/src/planner_node.cpp#L348-L402),
[runtime.cpp:198-273](simp_planner/simp_planner_cpp/src/runtime.cpp#L198-L273)). `received_odom_`는
한번 true가 된 뒤 다시 false로 되돌리는 코드가 없어서, 실제 오도메트리는 노드 수명 전체에서
단 한 번만 읽힙니다.

## 1. pending_plan_ ↔ active_plan_ 전환 문제

`planning_timer_`(계획 판단+계산)와 `command_timer_`(activate+명령 발행)는 서로 다른 콜백
그룹에서 `MultiThreadedExecutor(4)`로 독립 구동됩니다
([267-272](simp_planner/simp_planner_cpp/src/planner_node.cpp#L267-L272)).

```cpp
// planning_callback, 1026줄
if (pending_plan_ && now_ns() < pending_plan_->start_ns) return;
```

이 가드는 "pending이 있고 아직 start_ns 전"일 때만 막습니다. `pending_plan_`의 start_ns가
이미 지났는데 `command_callback`이 아직 activate를 처리하지 못한 좁은 순간에 urgent 재계획이
겹치면, `planning_callback`은 **옛 `active_plan_`을 기준**으로 새 candidate를 계산합니다.

최종 등록 체크(`plan_registration_is_current`, [runtime.cpp](simp_planner/simp_planner_cpp/src/runtime.cpp))는
`command_revision_`/`mode_revision_`만 비교하고 `activate_pending_locked()`로 인한
`active_plan_` 교체는 감지하지 못하므로, 이 잘못 계산된 candidate가 그대로 `pending_plan_`에
등록될 수 있습니다.

- **평소 안 터지는 이유**: `max_planning_frequency_hz`(기본 10Hz→100ms 최소 간격)와
  `AdaptiveHandoverTiming`의 lead time(120~600ms)이 `command_timer_`의 activate 지연(최대
  1 command 주기)보다 넉넉해서 보통은 문제없음 — 단, 이건 **락으로 보장된 게 아니라 순전히
  두 타이머 주파수 차이에 기대는 암묵적 가정**.
- **실제로 자주 노출되는 이유**: `costmap_callback`의 fingerprint 비교가 grid의 `data` 전체를
  바이트 단위로 해싱해서([456-458](simp_planner/simp_planner_cpp/src/planner_node.cpp#L456-L458)),
  센서 노이즈로 셀 값이 미세하게만 달라져도 무조건 `request_replan(..., urgent=true)`가
  호출됩니다. urgent 요청이 사실상 costmap 발행 주기(기본 `costmap_update_period_sec_`=0.2초)
  마다 거의 매번 발생하므로, 이 레이스가 열리는 창이 "희귀 엣지케이스"가 아니라
  **구조적으로 자주 노출되는 지점**입니다.

**발표용 표현**: "새 계획을 만들 때 예상한 차량 위치와, 실제 새 계획을 실행할 때의 차량
위치가 다를 수 있는데, 현재는 이를 확인하지 않고 계획을 교체한다."

## 2. command_callback — "순수 보간 함수"가 아니라 7분기 상태머신

| # | 상황 | 실제 동작 | 즉시 0? |
|---|---|---|---|
| 1 | 경로 hard change | `active_plan_`만 삭제, 감속곡선 없이 `zero_command()` | ⚠️ 즉시 0 (설계 근거 불명확, 잠재적 gap) |
| 2~3 | 모드 요청/확정 변경 | `mode_stop_`(JerkLimitedSafetyStop) 부드러운 감속 | ✅ 감속 |
| 4 | 스팟턴 종료 복귀 | 원래 저속 상태라 0이어도 무해 | 무해 |
| 5 | 스팟턴 대기(waiting_clearance) | 원래 저속 전제조건이라 무해 | 무해 |
| 6 | `!mode_ready` 일반 | 속도>threshold면 감속, 아니면 0 | ✅/무해 |
| 7 | 목표속도 변경 | `active_plan_` 안 지워짐, 계속 감 | 영향없음 |
| 8 | 안전한 계획 못 찾음(`forced_safety_stop_`) | `safety_stop_` 부드러운 감속 | ✅ 감속 |
| 9~10 | pending만 버려짐(costmap 충돌, 낡음/늦음) | `active_plan_` 그대로 유지 | 영향없음 |

유일한 순수 보간 지점은 `sample_body_command()`(elapsed만 입력받는 순수함수,
[runtime.cpp:125-188](simp_planner/simp_planner_cpp/src/runtime.cpp#L125-L188)) — 4번(정상
ACTIVE_PLAN 추종) 분기에서만 쓰입니다. 시간 매칭은 인라인 한 줄입니다:

```cpp
// command_callback, 1476줄
const double elapsed = std::max(0.0, 1.0e-9 * (stamp_ns - active->start_ns));
command = sample_body_command(active->allocation, active->result.trajectory.actions, elapsed, command_dt_);
```

### active_plan_을 지우는 경우 vs 명령만 가로채는 경우

- **mode_stop_ (2,3,6번)**: `pending_plan_`/`active_plan_`을 **같은 블록에서 즉시 리셋**
  ([1428-1429](simp_planner/simp_planner_cpp/src/planner_node.cpp#L1428-L1429)) — 모드가
  바뀌면 옛 `active_plan_`(옛 모드 기준 궤적)은 의미가 없어지므로 완전 폐기.
- **safety_stop_ (8번)**: `active_plan_`은 **안 지움**. `engage_forced_safety_stop()`은
  `pending_plan_`만 리셋하고, `forced_safety_stop_` 플래그가 `else if (active)`보다 먼저
  체크되어 **명령값만 가로채는** 구조. 나중에 새 plan이 activate되면(`activate_pending_locked`)
  자연스럽게 덮어써지며 `forced_safety_stop_=false`로 복귀.

### 왜 mode_stop_/safety_stop_이 "정상 planning"과 별개로 존재하는가

- **목적지 도착 정지**: 정상 `planning_callback`이 `config_.longitudinal`/`terminal_hold_`로
  이미 처리 — 별도 로직 불필요.
- **8번(안전한 계획 못 찾음)**: `planner_->plan()`이 **이미 실패한 뒤**에만 발동. 정상
  파이프라인(충돌검사·후보생성)에 다시 의존하면 같은 이유로 또 실패할 위험 — 그래서
  costmap/경로/최적화가 전혀 필요 없는, 실패할 수 없는 최후의 수단(jerk-limited 직선 감속)이 필요.
- **2,3,6번(모드전환)**: `planning_callback`이 `!input->mode_ready`면 맨 앞에서 즉시
  return — 계획 계산 자체를 시도하지 않음. 이유는 회전 반경·footprint 등 모든 계산이
  **하나의 확정된 드라이브 모드**를 전제로 하는데, 바퀴가 재배치되는 전환 구간의 운동학은
  어느 모드 모델로도 정확히 표현할 수 없기 때문.

## 3. hard_change ↔ 제자리턴(스팟턴) 관계

3중 게이트로 서로 배타적입니다 (`path_callback`):

```
maneuver_.state() != Inactive  → 새 경로는 deferred_reference_로 보류, hard_change 평가 안 함
needs_boundary_spot_turn(헤딩차 > 20°, heading_jump_threshold_rad) → 스팟턴 예약, hard_change 안 건드림
위 둘 다 아닐 때만 → is_soft_reference_continuation_locked()로 hard_change 계산
```

`is_soft_reference_continuation_locked()`([420-446](simp_planner/simp_planner_cpp/src/planner_node.cpp#L420-L446))는
"헤딩차>15°" 외에도 **위치차>0.5m**, **곡률차>0.05**, **드라이브 모드 불일치**를 봅니다 —
`needs_boundary_spot_turn`(경로 시작점 헤딩 vs 실제 차체 헤딩, 20° 기준)과는 비교 대상도
기준값도 다릅니다. 두 "곡률" 체크도 다른 것을 잽니다: 내부 스팟턴은 "이 구간 자체가
조향 한계(`curvature_max`)를 넘는가"(절대기준), hard_change는 "신구 경로의 곡률
**차이**가 큰가"(상대비교).

**제자리턴이 못 잡는(=hard_change만 남는) 케이스**: 방향은 맞는데(20° 이내) 경로 자체
(위치/곡률/모드)가 실질적으로 다른 상황 — 실무적으로는 **미션 페이즈 전환**(다음 랩,
파킹 램프 루프 등 phase 전환)이 가장 현실적인 발생 지점. 곡률 큰 코너나 큰 방향전환은
스팟턴이 처리하므로, "곡률/헤딩이 크게 바뀌면 다 스팟턴이 잡는다"는 가정만으로 hard_change를
죽은 코드로 볼 수는 없음. **실제 시나리오 로그에서 `REFERENCE_PATH_HARD`가 찍히는지 확인 후
제거 여부 결정 필요** (미확인 상태).

## 4. mutex 구조

4개 뮤텍스: `input_mutex_`(odom/path/costmap/mode/`current_state_`), `execution_mutex_`
(active/pending plan, last_command_, safety_stop_), `terminal_hold_mutex_`,
`execution_state_mutex_`(독립적).

- 교차 락은 `std::scoped_lock(input_mutex_, execution_mutex_)` 2곳뿐
  ([819](simp_planner/simp_planner_cpp/src/planner_node.cpp#L819),
  [1252](simp_planner/simp_planner_cpp/src/planner_node.cpp#L1252)) — 안전.
- **암묵적 중첩 1건**: `command_callback`이 `input_mutex_`를 잡은 채 `terminal_hold_latched()`
  호출 → 내부에서 `terminal_hold_mutex_` 추가 획득 ([1374](simp_planner/simp_planner_cpp/src/planner_node.cpp#L1374)).
  역순 호출은 현재 없어 데드락은 없지만, 문서화 안 된 순서 규칙이라 향후 리팩터링 시 위험.
- `command_callback`의 `input_mutex_` 보유 구간이 `spot_turn_feasible()`(costmap 검사)까지
  포함해 상대적으로 길어, 고빈도 콜백들과의 락 경합 여지 있음.

## 5. 타이머 / 재계획 판단 구조

| 타이머 | 콜백 | 역할 |
|---|---|---|
| `planning_timer_` | `planning_callback()` | 100Hz 틱, "짤 때 됐나" 판단 + 통과 시 그 자리에서 동기적으로 계획 계산까지 완료 |
| `command_timer_` | `command_callback()` | 100Hz, pending→active 전환 + `/cmd_vel` 등 발행 |

재계획 판단 3단계: ① 선행조건(모드ready/스팟턴아님/모드일치/pending 미도래) → ② 스케줄러
판단(`request_due` 또는 `watchdog_due`, 최소 100ms 간격) → ③ 실제 트리거 이벤트
(`LATEST_ODOMETRY`, `REFERENCE_PATH_HARD/SOFT`, `COSTMAP_CHANGED`(가장 빈번), `TARGET_SPEED_CHANGED`,
`MODE_REQUEST_CHANGED`, `VEHICLE_MODE_CONFIRMED`, `LATEST_COSTMAP_COLLISION`, `PLANNING_RETRY`,
`SPOT_TURN_COMPLETED`, `NO_SAFE_PLAN_RETRY`/`LATE_PLAN_RETRY`, watchdog의 `PLAN_AGE_WATCHDOG`).

기본값: `max_planning_frequency_hz`=10Hz(100ms 상한), `maximum_plan_age_sec`=0.5s(워치독 하한),
`costmap_update_period_sec_`=0.2s(실질적 재계획 주기 지배).

## 6. 타이밍 불일치로 이어지는 실패 모드 6가지

1. `LATE_PLAN_DISCARDED` — 계산이 예약시각(`scheduled_start`)을 넘김
2. `STALE_PLAN_DISCARDED` — 계산 도중 `command_revision_`/`mode_revision_` 변경
3. `predict_handover_state` 예외(`handover_time_ns < current_state_time_ns`) — 미래시각이
   현재시각보다 과거로 계산됨
4. pending/active 레이스 (1절)
5. costmap 메시지 순서 뒤바뀜 → 조용히 드롭 ([736-738](simp_planner/simp_planner_cpp/src/planner_node.cpp#L736-L738))
6. mode_stop_/스팟턴 구간에서 `current_state_` freeze — `invalidate_motion_plan()`이 발동하는
   5개 지점(경로hard change, 모드요청/확정변경, 스팟턴복귀/대기) 이후 실제 차는 감속·회전
   중인데 `current_state_`는 안 갱신되어, 재개 시 stale한 위치/속도로 다음 계획을 시작함 —
   3번 예외의 실제 원인이 될 수 있음

## 7. 새 노드(레퍼런스 포워딩, 피드백 제어기 없이) 설계

기존 `simp_tracker/simp_tracker/tracking_controller_node.py`(현재 존재, 별도 팀원 작업)는
PID형 피드백 추종(`tracking_command()`)을 하는데, 이번 논의는 그 피드백 보정을 빼고
**레퍼런스를 그대로 `/cmd_vel`로 내보내는** 별도 노드를 만드는 설계입니다.

### 시간 매칭 (이미 검증된 패턴)

```python
elapsed = (now_ns - stamp_ns(plan.start_time)) * 1e-9   # 자기 시계 - 메시지에 실린 절대시작시각
reference = reference_at(plan, elapsed)                  # bisect_right로 구간 찾아 선형보간(yaw는 wrap_angle)
```

전제조건: 두 노드가 **같은 ROS 시각 소스**(`use_sim_time` 일치)를 봐야 함. 이게 해결하는 건
"두 노드가 같은 시각에 대해 얘기하는가"뿐이고, 전송지연/타이머 위상 불일치/메시지 유실은
별개로 남음 — 시계 동기화는 freshness 체크가 의미를 갖기 위한 **전제조건**이지 그 자체로
지연/유실 문제의 해결책은 아님.

### 스코프 결정: "역할 분담" 방식 채택

특수상황(안전정지/모드전환/스팟턴/터미널홀드) 판단은 planner가 계속 하고, 그 결과를
`/planner/executed_command`의 `execution_state`+`vx/vy/yaw_rate`로 전달 — 새 노드는
`execution_state != "ACTIVE_PLAN"`이면 그 값을 그대로 릴레이, `"ACTIVE_PLAN"`일 때만
`/planner/trajectory` 배열을 직접 보간. 이게 "프로세스 경계를 못 넘는 공유상태" 문제의
실질적 해법.

**미해결/결정 필요 사항**:
- `/cmd_vel` 이중 발행(planner도 발행, 새 노드도 발행) 정리 필요.
- mode_stop_/safety_stop_ 감속곡선을 궤적 배열로 통일 발행하는 안 (아래 8절).

### 버려지는/못 쓰는 경우 종합 (16가지)

**planner 내부(active_plan_/pending_plan_ 리셋)**: 경로hard change, 모드요청변경,
모드확정변경, 스팟턴종료복귀, 스팟턴대기, `!mode_ready`매tick (이상 6개는 active_plan_까지
삭제) / 목표속도변경, 안전계획못찾음, costmap충돌로pending무효화, late/stale/collision
discard (이상 4개는 pending만 또는 신규 candidate만 버려짐, active_plan_ 안전).

**새 노드/tracker 쪽(캐시·신선도)**: plan 캐시(2개) 초과로 사용중인 plan_id 밀려남,
서로 다른 토픽 간 도착순서 미보장, ROS시각 역행 시 전체 캐시 초기화(`CLOCK_RESET`),
odom/command/mode freshness timeout, frame 불일치, 궤적 범위 밖(`TRAJECTORY_NOT_STARTED`/
`_EXPIRED`). **Python tracker 쪽은 이 모든 예외상황에서 예외 없이 즉시 `ZERO`** — C++
planner처럼 jerk-limited 부드러운 감속 개념이 없음. 특히 캐시 밀림/토픽 순서 뒤바뀜처럼
"plan 자체는 멀쩡한데 일시적 타이밍 문제"인 경우까지 급정지시키는 건 재고 필요.

## 8. 정지곡선(mode_stop_/safety_stop_)의 궤적화 — 실현 가능

`JerkLimitedSafetyStop`은 초기조건(speed, acceleration, beta, motion_heading, curvature)과
jerk/deceleration 한계가 주어지면 **결정론적으로 유한 시간 안에 반드시 정지**하는 닫힌 형태
함수입니다 (`stopped()`가 유한 스텝 안에 true가 됨이 수학적으로 보장). 따라서:

- `sample()`→`advance(dt)`를 반복해 정지할 때까지 전체 곡선을 미리 이산화(x,y는
  `predict_handover_state`의 `integrate()`와 같은 방식으로 vx/vy/yaw_rate를 적분해 채움)
- 기존 `Trajectory`/`TrajectoryPoint` 포맷 그대로 `plan_id`를 붙여 `/planner/trajectory`로 발행
- 새 노드는 ACTIVE_PLAN과 완전히 같은 코드 경로로 처리 — SAFETY_STOP/MODE_STOP 전용 예외
  분기가 불필요해짐

**시작점 연속성 문제와 해법**: 처음엔 "새 노드가 실제로 낸 마지막 값을 planner에 다시
피드백해야 한다"고 판단했으나, 이는 틀렸음. 정상 plan-to-plan 전환도 `last_command_`가
아니라 **`predict_handover_state()`가 이전 궤적 데이터를 계속 적분**해서 연속성을
확보하므로, 정지곡선도 "지금 활성화된 궤적(정상 plan이든 이전 정지곡선이든)을 계속
적분해서 시작 상태를 구하는" 동일한 방식으로 통합하면 됩니다. 조건은 단 하나 — **planner가
정상 plan과 정지곡선을 항상 하나의 "현재 활성 궤적" 개념으로 통합 관리**하는 것. 이건
planner 프로세스 내부 리팩터링만으로 가능하고, 새 노드로부터의 피드백 채널은 불필요합니다.

남는 구현 이슈: `mode_stop_`(2,3,6번)은 `planning_callback` 경로를 안 타므로
(`!input->mode_ready`면 맨 앞에서 return), 이 궤적 발행 로직을 `command_callback` 쪽에
`pending_plan_`/`activate_pending_locked` 파이프라인과는 별개의 독립 경로로 추가해야 함.
knot 간격은 기존 `trajectory_knot_dt_sec`(0.1s)을 그대로 쓸지, jerk 민감도를 고려해
`command_dt_`(0.01s) 수준으로 더 촘촘히 할지 정밀도/메시지 크기 트레이드오프 결정 필요.
