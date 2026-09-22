# command_callback의 정지 로직과 simp_tracker 도입 시 처리 방향

기준 커밋: `e7120b3` (제자리턴 중 yaw 갱신 fix까지만 적용된, `simp_tracker` 도입 이전의 원본
`command_callback`). 이 문서는 (1) 지금 `safety_stop_`/`mode_stop_`이 정확히 뭐고 언제 쓰이는지,
(2) `simp_tracker`가 상태를 안 갖는 "순수 재생" 노드가 될 때 이걸 어떻게 다뤄야 하는지를 정리합니다.

## 1. 지금 command_callback은 7가지 상황을 판정하는 상태머신

매 tick(`command_dt_` 주기) 아래를 순서대로 판정해서 하나의 출구로만 나갑니다
([planner_node.cpp:1338-1512](simp_planner/simp_planner_cpp/src/planner_node.cpp#L1338-L1512)):

| # | 상황 | 무엇을 계산해서 내보내는가 |
|---|---|---|
| 1 | 스팟턴 회전 중 (`rotating_command`) | `maneuver_.sample()`이 만든 순수 회전 명령 (vx=vy=0, yaw_rate만) |
| 2 | 스팟턴 공간 대기 중 (`waiting_clearance`) | `zero_command()` |
| 3 | 모드 전환 대기, 아직 감속 안 끝남 (`mode_state==StoppingForChange`) | **`mode_stop_`** |
| 4 | 모드 전환 대기, 이미 저속 (`WaitingForCompletion`) | `zero_command()` |
| 5 | 안전한 계획을 못 찾음 (`forced_safety_stop_`) | **`safety_stop_`** |
| 6 | 정상 추종, 궤적 구간 안 | `sample_body_command(active_plan_ 궤적, elapsed)` |
| 7 | 정상 추종, 궤적 구간 다 씀 | 마지막 지점에서 `safety_stop_`로 전환해서 이어감 |

## 2. `mode_stop_`이 뭔가

**트리거**: `!mode_supervisor_.ready()`(요청 모드 ≠ 확정 모드, 또는 차량이 아직 정렬 중) **이면서**
`measured_speed > mode_change_stop_speed_`([1444](simp_planner/simp_planner_cpp/src/planner_node.cpp#L1444)).

**왜 필요한가**: 드라이브 모드 전환(예: Forward→Crab)은 바퀴 배치 자체가 바뀌는 거라, 차량이
어느 정도 저속으로 내려간 뒤에야 안전하게 요청할 수 있습니다. `planning_callback`은 이 구간에서
**아예 계획 계산을 시도하지 않습니다** — 특정 모드 하나를 전제로 한 운동학 모델(회전반경,
footprint)이 전환 중엔 어느 쪽으로도 맞지 않기 때문입니다. 그래서 "정상 planning으로 감속
계획 짜기"가 원천적으로 불가능한 구간이고, 대신 `mode_stop_`이 그 자리를 메웁니다.

**어떻게 계산하는가**:
```cpp
if (!mode_stop_) {
  mode_stop_ = JerkLimitedSafetyStop::from_command(*last_command_, a_max, jerk_max);
}
command = mode_stop_->sample_and_advance(command_dt_);
```
경로/costmap/최적화 전혀 없이, **직전에 실제로 나간 명령(`last_command_`)**에서 시작해서
jerk 한계 내로 감속하는 순수 analytic 곡선입니다. 매 tick `sample_and_advance()`로 내부
상태(속도, 가속도, 경과시간)를 스스로 전진시킵니다.

**언제 사라지는가**: 모드가 확정되어 `mode_ready`가 다시 true가 되는 순간, `mode_stop_.reset()`
([1473](simp_planner/simp_planner_cpp/src/planner_node.cpp#L1473))으로 버려집니다.

## 3. `safety_stop_`이 뭔가

**트리거**: `planning_callback`이 충돌 없는 후보를 **단 하나도** 못 찾았을 때
`engage_forced_safety_stop()`이 호출되어 `forced_safety_stop_ = true`가 됨. **또는** 정상
추종 중이던 `active_plan_`의 궤적 구간을 다 써버렸는데 다음 계획이 아직 준비 안 됐을 때도
같은 메커니즘으로 전환됩니다(위 표의 7번).

**왜 필요한가**: "안전한 계획을 못 찾았다"는 건 이미 정상 planning 파이프라인이 실패한
뒤이므로, 그 파이프라인에 다시 의존해서 정지 계획을 짜려고 하면 같은 이유로 또 실패할 수
있습니다. `safety_stop_`는 costmap/경로/최적화가 전혀 필요 없는, **항상 계산 가능한 최후의
수단**입니다. 계산 방식은 `mode_stop_`와 완전히 동일한 `JerkLimitedSafetyStop` — 다만
호출 이유가 다를 뿐입니다.

**공통점**: `mode_stop_`과 `safety_stop_` 둘 다 "지금 activeplan_이 잘못됐다"는 뜻이 아니라
"지금 이 순간 정상적인 계획을 낼 수 없는 상황"이라는 신호이고, 둘 다 `last_command_`(또는
궤적의 마지막 지점)에서 물리적으로 이어지는 jerk-limited 감속 하나로 처리됩니다.

**중요한 차이 하나**: `mode_stop_`은 진입 시 `active_plan_`/`pending_plan_`을 즉시
리셋합니다([1440-1441](simp_planner/simp_planner_cpp/src/planner_node.cpp#L1440-L1441)) —
모드가 바뀌면 옛 계획 자체가 의미 없어지니까요. `safety_stop_`는 `active_plan_`을 안
지웁니다 — `engage_forced_safety_stop()`은 `pending_plan_`만 리셋하고, 나중에 새 계획이
성공하면 `activate_pending_locked()`가 자연스럽게 덮어씁니다.

## 4. simp_tracker 도입 시: 이 둘을 어떻게 다룰 것인가

이번에 정한 원칙 — **simp_tracker는 상태를 갖지 않고, 그때그때 받은 궤적을 시간 보간해서
100Hz로 재생만 한다.** 스팟턴/모드전환/안전정지 같은 "지금 뭘 해야 하는가" 판단은 전부
`planner_node.cpp`(=지금의 `command_callback`이 하던 역할)에 그대로 남습니다.

**바뀌는 것은 "판단 결과를 어떻게 전달하는가"뿐입니다:**

| 항목 | 지금(e7120b3) | simp_tracker 도입 후 |
|---|---|---|
| **판단 주체** | `command_callback` (같은 프로세스) | 그대로 planner (다른 프로세스로 안 옮김) |
| **판단 결과 전달 방식** | `last_command_`를 직접 `/cmd_vel`로 발행 | **궤적(포인트 배열)을 발행**, tracker가 보간해서 발행 |
| **mode_stop_ 계산 위치** | command_callback 안에서 매 tick `sample_and_advance()` | planner가 트리거되는 그 순간 **한 번**, `JerkLimitedSafetyStop`을 정지할 때까지 미리 다 돌려서 포인트 배열로 이산화 → 발행 |
| **safety_stop_ 계산 위치** | 위와 동일 | 위와 동일한 방식으로 미리 이산화해서 궤적으로 발행 |
| **tracker가 이 둘을 구분해서 처리하는가** | 해당 없음 | **아니오.** tracker 입장에선 "정상 계획 궤적"이든 "정지 궤적"이든 그냥 포인트 배열 + 시작시각일 뿐, 완전히 동일한 코드 경로(시간 보간)로 처리 |

**즉 `mode_stop_`/`safety_stop_`이라는 이름의 "특수 케이스"가 tracker 쪽에는 아예 존재하지
않게 됩니다.** planner가 "지금은 정지시켜야 한다"고 판단하면, 그 결과물로 **정지 궤적**(예:
`JerkLimitedSafetyStop`을 `stopped()`가 될 때까지 미리 샘플링해서 만든 포인트 배열)을 하나
만들어서 평소 계획 궤적과 똑같은 형태로 발행하면 됩니다. tracker는 그게 정지 궤적인지 정상
계획인지 몰라도 되고, 그냥 시간에 맞춰 보간해서 내보내면 끝입니다.

**planner 쪽에서 남는 결정 사항** (이번 대화에서 뭘 궤적으로 바꿀지 하나씩 정하기로 한 부분):
- `mode_stop_`/`safety_stop_`을 몇 초 길이로 미리 이산화할지 (정지까지 걸리는 시간은 매번
  다름 — 속도에 따라 가변적)
- 스팟턴 회전(`rotating_command`)도 같은 방식으로 궤적화할지, 아니면 별도로 다룰지
- 궤적이 다 끝났는데 tracker가 다음 궤적을 아직 못 받은 경우 tracker 자체적으로 뭘 할지
  (이것도 "상태 관리 없음" 원칙과 부딪히는 지점이라 논의 필요)
