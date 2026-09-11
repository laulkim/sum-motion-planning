# 제자리턴 시나리오/기능 설계

시나리오(`spot_turn_course`, Python)와 C++ 플래너/차량 시뮬레이터를 아래 설계에 따라 구현했습니다.
목표속도 4.2 m/s의 실제 ROS 실행에서 제자리턴 3회와 크랩 전환을 포함한 7개 phase 완료를
확인했습니다. 구현 중 보완한 경계 조건과 남은 검증은 [8절](#8-남은-작업)에 기록합니다.

## 0. 배경 — 역할 분담

`ScenarioPath.from_arrays()`의 정상 주행 곡률/헤딩 검사는 유지합니다. 이전에 전역 제거했던
아래 두 조건을 정상 구간에는 다시 적용하고, 제자리턴 이음매의 연속성 검사만 예외 처리합니다
([scenario_path.py](simp_planner_tools/simp_planner_tools/scenario_path.py)):

```python
if np.max(np.abs(kappa)) > curvature_limit + 1.0e-9:          # 정상 주행 구간에 적용
    raise ValueError("Scenario path exceeds the curvature limit")
if np.max(np.abs(yaw_step)) > math.radians(yaw_step_limit_deg):  # 정상 주행 구간에 적용
    raise ValueError("Scenario path contains a heading discontinuity")
```

그래서 **급코너를 하나의 `ScenarioPath`(= 하나의 phase) 안에 그대로 담을 수 있습니다** —
`ScenarioPath.join_with_turns()`(`scenario_path.py`)로 두 개의 정상 구간을 이어붙이면, 이음매의
두 점은 **완전히 같은 위치(구간 길이 0)**, 그 사이에서 헤딩만 크게 꺾입니다. `join_with_turns()`는
이 이음매 인덱스만 곡률/헤딩-스텝/접선-일치 검사에서 제외하고, 나머지 구간(실제 주행 구간)에는
검사를 그대로 적용합니다 — 정상 구간 저작 실수는 여전히 걸러집니다.

역할 분담:
- **시나리오 매니저(Python)**: phase 분할은 **드라이브 모드가 바뀔 때만** 합니다. 급코너
  때문에 phase를 끊지 않습니다.
- **플래너(C++)**: 받은 레퍼런스 경로 배열 **안에서** 급코너를 스스로 찾고 (`path_callback()`이
  이미 계산하는 `estimate_curvature_from_yaw()` 결과를 그대로 씀, 1-2절),
  그 지점 앞까지만 잘라서 씁니다 — 기존 `switch_s`가 `path.clipped(stop_s)`
  ([scenario_manager_node.py](simp_planner_tools/simp_planner_tools/scenario_manager_node.py))로
  하던 것과 같은 방식이지만, 이제는 플래너가 직접 합니다. 잘려나간 나머지 구간은
  플래너가 들고 있다가 회전이 끝나면 이어서 씁니다 (1-2절).

---

## 1. 급코너 지점 — 시나리오가 아니라 플래너가 스스로 찾는다

### 1-1. 왜 다른 모드와 다르게 다뤄야 하는가

Forward/Reverse/Left/Right(0~3)는 **항상 외부에서 요청이 들어오는 값**입니다: 시나리오가
`/requested_drive_mode`로 "이 모드로 바꿔라"라고 명시적으로 보내면, 플래너는 그 요청값과
차량의 현재 모드를 비교만 합니다(`DriveModeSupervisor::ready()`). "언제, 무엇으로 바꿀지"는
전부 시나리오가 결정하고, 플래너는 그 결정을 실행/감시만 합니다.

**제자리턴(4)은 이 패턴을 따르지 않습니다.** 시나리오는 그냥 평소처럼 다음 phase의 레퍼런스
경로만 보낼 뿐이고(제자리턴이라는 개념 자체를 모릅니다), **플래너가 그 경로를 받아서 스스로**
"이 경로를 조향만으로 따라갈 수 있나, 아니면 제자리에서 먼저 돌아야 하나"를 판단합니다. 이건
플래너가 매 `path_callback()`마다 `estimate_curvature_from_yaw()`로 곡률을 스스로 계산하고
있는 것과 정확히 같은 성격의 작업입니다 — 곡률을 검사하듯이, 헤딩 불연속도 플래너가 직접
검사합니다.

### 1-2. 플래너 내부 탐지 + 절단 — 새로 계산하지 않고, 이미 계산되는 곡률을 그대로 쓴다

`path_callback()`이 사용하는 입력 처리 코드는 `estimate_curvature_from_yaw()`로 곡률을 계산하고
있습니다 (`s`, `psi`로부터 전진차분 `kappa[i] = (psi[i+1]-psi[i]) / max(s[i+1]-s[i], 1e-15)`).
`join_with_turns()`로 이어붙인 이음매는 `s[i+1]-s[i] ≈ 0`인 채로 헤딩만 크게 꺾이므로, 이
함수가 그 지점에서 **자연히 비정상적으로 큰 값**(정상 주행 곡률과는 몇 자릿수 차이 나는 값)을
내놓습니다. 별도의 헤딩-점프 스캔 함수를 새로 만들 필요가 없습니다 — **이미 계산되는 이
곡률값 자체가 급코너 판별 기준**입니다:

아래는 분할 기준을 설명하는 개념 코드입니다. 실제 저장 객체는 1-2절 하단의
`SpotTurnReferenceBuffer`입니다.

```cpp
// 입력 처리: arc_length/kappa를 계산한 직후 — ReferencePath를 만들기 전
const auto arc_length = cumulative_arc_length(msg->x, msg->y);
const auto kappa = estimate_curvature_from_yaw(unwrap_angles(msg->yaw), arc_length);
const auto used = kappa.size();

std::size_t split = used;
for (std::size_t i = 0; i < used; ++i) {
  if (std::abs(kappa[i]) > spot_turn_kappa_threshold) { split = i; break; }
}
```

`split < used`면 포인트 `[0, split]`까지만 `reference_path_`로 쓰고 — 차량이 거기서 종단 정지
로직 그대로 자연히 멈춤 — **뒤쪽 나머지(`[split+1, ...]`)는** `pending_post_turn_path_`에 남겨
둡니다. 목표 차체각은 회전 후 이동 방향에서 `drive_mode_heading_offset(reference_mode)`를
뺀 값입니다. 아래 도식의 `pending_post_turn_path_`와 `pending_spot_turn_target_yaw_`는
설계상 역할을 나타내는 이름이며, 실제로는 `reference_buffer_`가 구간 큐와 다음 목표각을
함께 관리합니다. 어떤 메시지
인터페이스로도 외부로 나가지 않습니다.

절단은 `ReferencePath`를 만들기 **전** 단계이므로, 그 뒤에 이어지는
`is_soft_reference_continuation_locked()`(soft/hard 판정, 0절 이후 순서 변경 없음)는 이미
잘린 `path`를 대상으로 그대로 수행됩니다. 다만 그 판정 결과와 무관하게, **코너를 발견해서
절단한 이 메시지는 항상 `hard_change`로 강제**합니다:

```cpp
if (split < used) hard_change = true;   // is_soft_reference_continuation_locked() 결과와 별개로 강제
```

이 강제가 필요한 이유: 코너 절단은 **같은 phase 안(모드 불변)**에서 일어나므로
`is_soft_reference_continuation_locked()`의 첫 줄 가드(`new_mode != reference_mode_`)에
걸리지 않습니다(일반 모드 전환은 mode 자체가 바뀌어서 이 가드에서 바로 hard 처리됨 — 5절).
그리고 지오메트리 비교(위치/헤딩/곡률)도 **차량의 현재 위치 근처만** 보므로, 코너가 차량보다
한참 앞에 있으면 옛 경로와 새(잘린) 경로가 그 지점에서는 완전히 같아 보여 soft로 오판될 수
있습니다 — 코너를 그냥 조향으로 통과하려던 낡은 계획이 무효화되지 않고 남는 걸 막기 위해
명시적으로 강제합니다.

`spot_turn_kappa_threshold`는 정상 주행에서 나올 수 있는 최대 곡률(시나리오 저작 시 관례상
쓰던 `curvature_limit` 0.2 rad/m 수준)보다 몇 자릿수 위, 이음매의 사실상-무한대 값보다는
훨씬 아래인 값이면 됩니다 — 자릿수 차이가 워낙 크므로 정밀한 튜닝이 필요 없습니다.

`msg->x`/`msg->y`는 odom 프레임(고정 원점) 절대좌표입니다 —
`scenario_manager_node.py`의 `frame_id` 파라미터(기본값 `"odom"`)로 발행되고,
`odom_callback()`이 그 프레임과 다른 오도메트리는 거부합니다. 그래서 `pending_post_turn_path_`는
**그 메시지 하나에서 받은 절대좌표 스냅샷**이고, 이후 새로 들어오는 다른 메시지와 좌표를
맞추거나 비교할 필요가 전혀 없습니다 — 재설정되는 건 `s`(메시지마다 자기 첫 점을 0으로 잡는
누적 아크렝스)뿐이고, `x`/`y`는 메시지가 바뀌어도 같은 원점을 계속 씁니다.

회전이 끝나면(`command_callback()`, `SpotTurnManeuver::on_mode_ready()`가 완료를 보고할
때) 이 스냅샷(`pending_post_turn_path_`)을 그대로 새 `reference_path_`로 설치합니다 — 새
메시지를 기다리거나 조회하지 않습니다. 이때도 그 구간 안에 코너가 또 있을 수 있어서 같은
곡률 스캔을 한 번 더 적용합니다. 한 phase 안에 코너가 여러 개 있어도 이 과정이 반복되면서
순서대로 처리됩니다.

실제 구현은 입력 처리와 단위 검증에서 같은 코드를 사용하도록 `runtime.cpp`의
`SpotTurnReferenceBuffer::update()`에 기존 누적거리/전진차분 계산을 옮겼습니다. 별도
헤딩 점프 탐지기를 추가하지 않고 위 곡률 스캔을 사용합니다. 이음매 위치 차이는 최대
1e-6 m, 곡률 임계값 기본값은 1000 m⁻¹입니다. 동일 위치인데 헤딩 변화가 없는 중복점은
거부합니다.

**절단 후 끝점 곡률은 반드시 0으로 바꿉니다.** `kappa[split]`은 삭제한 이음매를 가로질러
계산한 값이므로, 이를 주행 경로에 복사하면 직선 후보도 곡률 검사에 탈락합니다.
`ReferencePath`는 같은 s를 가진 두 점을 담을 수 없으므로 잔여 스냅샷 역시 이음매에서
나눈 정상 `ReferencePath`들의 큐로 보관합니다. 각 구간은 최소 3점이어야 하고, 전체
구간 생성이 성공한 뒤에만 기존 상태를 교체합니다.

롤링 창이 다시 이전 코너를 포함할 수 있으므로 완료한 코너는 `s`가 아닌 고정 좌표
`(x, y, 회전 전 이동 방향, 회전 후 이동 방향)`로 식별합니다. 완료 코너가 새 창에 있으면
그 코너 뒤부터 사용합니다. 모드/프레임이 바뀌면 기록을 초기화합니다.

### 1-3. 트리거 — 매 틱 재시도하는 `command_callback()`에서

`path_callback()`은 급코너를 발견하면 목표각만 `pending_spot_turn_target_yaw_`에 기억해 두고,
**실제 트리거는 `command_callback()`이 매 틱 검사**합니다 — 3절의 "장애물이 있으면 매 틱
재시도" 요건을 만족하려면 새 레퍼런스가 들어올 때만 도는 `path_callback()`이 아니라 매 틱
도는 쪽에서 재시도해야 하기 때문입니다.

트리거 조건은 "정지"와 "그 지점에 도착"을 **둘 다** 봅니다 — 시나리오 매니저의
`update_scenario_state()`(`abs(current_stop_error) <= terminal_capture_distance and speed <=
stop_speed_threshold`, 5절)와 같은 원리이고, 플래너 안에도 이미 같은 패턴이 있습니다
(`terminal_hold_active()`, 차량 위치를 `reference_path_` 자신에게 투영해서 그 경로 자신의
`s_max()`와 비교) — 그 패턴을 그대로 재사용합니다:

```cpp
// command_callback() 맨 앞, mode_ready/mode_state를 읽는 바로 그 자리
bool spot_turn_arrived() {
  const auto projection = reference_path_->project(current_state_.x, current_state_.y, current_state_.chi);
  return reference_path_->s_max() - projection.s <= spot_turn_arrival_tolerance_m &&
         std::hypot(current_state_.x - reference_path_->x().back(),
                    current_state_.y - reference_path_->y().back()) <= spot_turn_arrival_tolerance_m;
}

if (mode_ready && maneuver_.state() == Inactive && pending_spot_turn_target_yaw_ &&
    measured_speed <= mode_change_stop_speed_ && spot_turn_arrived() &&
    spot_turn_feasible(*costmap_, x, y, config_.vehicle, config_.spot_turn.safety_margin)) {
  maneuver_.trigger(*pending_spot_turn_target_yaw_,
                    mode_supervisor_.requested_mode().value_or(reference_mode_));
  mode_supervisor_.set_requested_mode(DriveMode::SpotTurn);  // target_yaw는 여기 안 들어감
  pending_spot_turn_target_yaw_.reset();
  // mode_ready/mode_state를 이 자리에서 즉시 다시 읽어서 같은 틱에 StoppingForChange로 넘어간다.
}
```

`spot_turn_arrived()`가 필요한 이유: `measured_speed <= mode_change_stop_speed_`만으로는 "코너에
도착해서 멈춘 상태"와 "출발 전이라 아직 속도가 0인 상태"를 구분하지 못합니다. `reference_path_`는
이미 코너 앞에서 끝나도록 절단된 경로(1-2절)이므로, 그 경로 자신의 `s_max()`에 대한 투영 진행도와 실제 종점까지의 거리를
함께 봅니다. 투영은 경로 끝에서 clamp되므로 진행도만으로는 멀리 떨어진 차량도
도착한 것으로 오인할 수 있습니다. 기본 도착 허용 거리는 0.20 m입니다 — 다른 메시지와 비교하지 않는, `terminal_hold_active()`와 동일한
자기완결적 검사입니다.

`DriveModeSupervisor::set_requested_mode()`는 `DriveMode` 하나만 받습니다 — "지금 어떤 모드가
요청됐는가"만 알면 되고 그 모드로 뭘 할지는 몰라도 됩니다 (2절). 이 호출로 마치 외부에서 모드
4를 요청받은 것처럼 되고, 이후 "바퀴가 실제로 SpotTurn configuration으로 바뀌었는가"는
`DriveModeSupervisor`가 기존 로직 그대로 처리합니다 (2절).

### 1-4. 완료 후 원래 요청 모드로 복귀 (중요한 함정)

`requested_mode_`는 지금까지 "외부에서 마지막으로 요청받은 값" 하나만 담는 변수였는데,
제자리턴은 플래너가 **일시적으로 그 값을 가로채서** 4로 덮어쓰는 셈입니다. 그래서 "외부에서
실제로 요청한 모드"(예: `FORWARD`)를 별도 변수(`external_requested_mode_`)로 따로 기억해 둡니다.

**주의(2절과 연결되는 부분)**: 되돌리는 시점은 `mode_supervisor_.ready()`가 되는 순간이
**아닙니다.** `ready()==true`는 "바퀴가 SpotTurn 자세로 정렬 완료"일 뿐, 회전이 끝났다는
뜻이 아닙니다. 실제로 되돌리는 시점은 **회전(ROTATING)이 끝난 순간**입니다 — 자세히는 2-3절.

제자리턴이 진행 중인 동안(`AligningWheels` 또는 `Rotating`) 시나리오가 `/requested_drive_mode`로
또 다른 모드를 새로 요청하면, `mode_supervisor_`를 직접 바꾸지 않고
`maneuver_.set_external_requested_mode()`로 `external_requested_mode_`만 최신 요청값으로
덮어씁니다 — 회전이 끝나면 그 최신 요청 쪽으로 복귀합니다. 다만 **복귀 정렬을 이미
시작한 뒤** 새 요청이 오면 supervisor의 복귀 요청도 함께 갱신합니다. 실제 구현은
`set_external_requested_mode(mode, supervisor)`로 이 경우를 처리합니다. 회전 도중 받은
다른 모드의 경로는 보류했다가 복귀 후 설치합니다.

### 1-5. Phase 분할은 이제 드라이브 모드 전환에서만 일어남

`ScenarioPath.join_with_turns()`로 이음매의 연속성 검사만 예외 처리하므로(0절), 40도/60도/70도급 급코너를 여러 개 가진 웨이포인트도 **하나의 연속
`ScenarioPath`(= 하나의 phase)** 로 저작할 수 있습니다. `ScenarioPhase`는 원래 형태 그대로
유지합니다 (신규 필드 없음):

```python
@dataclass(frozen=True)
class ScenarioPhase:
    name: str
    path: ScenarioPath
    cruise_speed: float
    switch_s: float | None = None
    # 제자리턴 관련 필드 없음 -- 그건 전부 플래너 내부 판단 (1-2, 1-3절)
```

`switch_s`/phase 분할은 **드라이브 모드가 바뀔 때**(예: FORWARD로 다 돌고 나서 LEFT 크랩으로
마무리)만 씁니다. 급코너는 phase 경계와 무관하게 그 phase의 경로 배열 안에 그냥 들어 있고,
플래너가 알아서 찾아서 처리합니다 (1-2절).

### 1-6. 테스트 시나리오

`spot_turn_course`(5절)가 서로 다른 각도(40도/-60도/70도)의 코너와 곡선 구간을 섞어서, 코너마다
회전이 잘 되고 다음 구간으로 이어지는지 검증합니다. 회전 여부/목표각 판단은 전부 플래너가
스스로 계산합니다 — 시나리오는 코너가 있는 경로만 던져주면 됩니다.

---

## 2. 상태 전이 설계 — "바퀴 정렬"과 "실제 회전"은 서로 다른 레이어

`DriveModeSupervisor`가 담당하는 **바퀴 정렬(configuration 전환)** 레이어와, 그 위에 새로 얹는
**실제 차체 회전** 레이어를 분리합니다. `spot_turn_target_yaw` 같은 필드나 별도
`DriveModeCommand.msg`는 두지 않습니다 — 아래 이유 참고.

### 2-1. 왜 분리해야 하는가

0~3번 모드에서는 "바퀴 configuration을 요청값으로 바꾼다"와 "그 상태를 이용해 뭘 한다"가
**항상 같은 순간**이었습니다 — Forward/Reverse/Left/Right 전환은 정지 상태에서 바퀴만 돌리면
끝나는 조향 기하학 변경이라, 정렬이 끝나는 순간이 곧 전환이 끝나는 순간입니다. 그래서
`DriveModeSupervisor::ready()` 하나로 "전환 완료"를 판정해도 문제가 없었습니다.

**제자리턴은 이 전제가 깨지는 첫 사례입니다.** SpotTurn(4)으로 바퀴가 정렬된 다음에도, 그
정렬된 상태를 이용해 **차체를 실제로 돌리는 별도의 동작**이 남아있습니다. 즉:

- `DriveMode::SpotTurn` = "차량이 제자리 회전을 수행할 수 있도록 바퀴가 구성된 상태" (바퀴
  configuration일 뿐)
- 회전 자체는 그 configuration 위에서 플래너가 별도로 관리하는 **상위 동작**

이 둘을 하나의 상태머신(`DriveModeSupervisor`)에 다 넣으면 "지금 바퀴를 정렬하고 있는 건지,
실제 회전 중인지, 회전은 끝났는데 Regular로 복귀 정렬 중인지"가 구분되지 않습니다. 그래서
`DriveModeSupervisor`는 **바퀴 정렬 여부만** 계속 담당하고, 그 위에 작은 **매뉴버 상태**를
하나 새로 얹습니다.

### 2-2. 바퀴 정렬 레이어 — 사실 이미 다 있음 (신규 코드 거의 0)

DriveMode enum에 5번째 값만 추가합니다:

```cpp
// core.hpp
enum class DriveMode : std::uint8_t {
  Forward = 0,
  Reverse = 1,
  Left = 2,
  Right = 3,
  SpotTurn = 4,   // ← 추가
};
```

그리고 `mode_supervisor_.set_requested_mode(DriveMode::SpotTurn)`을 호출하는 순간, **이미 있는**
`DriveModeControlState`([runtime.hpp:34](simp_planner/simp_planner_cpp/include/simp_planner/runtime.hpp#L34))
상태머신이 지금과 완전히 동일하게 돌아갑니다 — 이게 바로 "STOPPING → 바퀴 정렬"
구간의 실체입니다. 신규 코드가 필요 없습니다:

| 매뉴버 단계 | 기존 `DriveModeControlState` 값 | 진입 조건 ([runtime.cpp:68](simp_planner/simp_planner_cpp/src/runtime.cpp#L68) `state()`) |
|---|---|---|
| STOPPING | `StoppingForChange` | `requested_mode_ != current_mode_`, 아직 속도 있음(`measured_speed > stop_speed_threshold`) |
| ALIGN_FOR_SPOT_TURN / ALIGN_FOR_REGULAR | `WaitingForCompletion` | 정지 완료, 차량이 바퀴 재구성 중(차량 피드백의 `status == STATUS_ALIGNING`) |
| (정렬 완료) | `Ready` (`ready()==true`) | `current_mode_ == requested_mode_` 확정 |

`command_callback()`도 이 구간에서 지금과 똑같이 동작합니다 — 아직 속도가 있으면
`mode_stop_`(`JerkLimitedSafetyStop`, [runtime.hpp:128](simp_planner/simp_planner_cpp/include/simp_planner/runtime.hpp#L128))으로
감속하고, 정지 후엔 `zero_command()`를 유지하며 `ready()`를 기다립니다
([planner_node.cpp:1241](simp_planner/simp_planner_cpp/src/planner_node.cpp#L1241) `command_callback`,
`!mode_ready` 분기). **ALIGN_FOR_SPOT_TURN과 ALIGN_FOR_REGULAR도 코드상으로는 완전히 같은
경로**입니다 — `requested_mode_`가 무엇이냐(SpotTurn이냐 `external_requested_mode_`냐)만 다를 뿐,
`DriveModeSupervisor`도 `DriveModeState.msg`도 방향을 구분할 필요가 전혀 없습니다.

즉 `DriveModeSupervisor`, `DriveModeState.msg`, `/vehicle/drive_mode_command`(`UInt8`) —
**기존 데이터 필드를 유지합니다.** 메시지에는 `SPOT_TURN=4` 상수만 추가합니다. `set_requested_mode()`도 시그니처
변경 없이 기존 그대로(`DriveMode` 단일 인자)입니다. **`target_yaw`는 이 인터페이스 어디에도
들어가지 않습니다** — 아래 2-3절의 상위 객체 안에만 존재합니다.

### 2-3. 실제 회전 레이어 — 진짜 새로 만드는 부분

바퀴 정렬(`ready()==true`)이 끝난 뒤에 실제로 차체를 돌리는 부분만 신규입니다. 상태 흐름은
다음과 같이 구성합니다. 실제 선언/구현은 `runtime.hpp`/`runtime.cpp`에 있으며
`sample()`에서 실측 odom 피드백을 받습니다:

```cpp
enum class SpotTurnManeuverState { Inactive, AligningWheels, Rotating };

// command_callback(): Ready edge에만 의존하지 않고 매 tick 확인한다.
if (maneuver_.on_mode_ready(mode_supervisor_)) {
  reference_path_ = reference_buffer_.complete_turn();
  // path/command revision 갱신, 이전 실행 계획과 terminal hold 초기화, 재계획 요청
}

// 실제 회전 중에만 BodyCommand를 반환한다.
auto rotating_command = maneuver_.sample(
    command_dt_, current_body_yaw_, current_body_yaw_rate_, mode_supervisor_);
```

`SpotTurnManeuver`는 내부 `returning_` 값으로 SpotTurn 바퀴 정렬과 원래 모드 복귀 정렬을
구분합니다. 회전 완료 시 supervisor에 최신 외부 모드를 요청하고, 그 모드의 Ready를
확인한 뒤 `Inactive`로 돌아갑니다. `on_mode_ready()`는 이 복귀 완료에서만 true를 반환합니다.

`YawRotationProfile`은 `JerkLimitedSafetyStop`처럼 가속도 제한 패턴을 쓰지만, P-게인이 아니라
"지금 각속도에서 가속도 한계로 감속했을 때 정확히 목표에 도달하는 최대 각속도"를 매 틱
다시 계산하는 물리식입니다 — `JerkLimitedSafetyStop`의 `release_speed` 계산과 같은 방식입니다:

$$e_\psi = \operatorname{wrap}(\psi_{target} - \psi), \qquad
r_{feasible} = \sqrt{2 \cdot a_{max} \cdot |e_\psi|}, \qquad
r_{desired} = \operatorname{sign}(e_\psi) \cdot \min(r_{max},\, r_{feasible})$$

매 틱 $r_{cmd}$를 $r_{desired}$ 방향으로 가속도 한계($a_{max}$)만큼만 움직입니다.
각도 오차가 허용 범위에 들어오면 목표 각속도를 0으로 낮추며, 실측 각도 오차가
$\varepsilon_\psi$ 이내이고 명령 각속도가 0으로 내려간 뒤 실측 각속도도
$\varepsilon_r$ 이내가 되어야 `done()`이 true를 반환합니다.

실제 `sample(dt, measured_yaw, measured_yaw_rate)`는 매 틱 odom의 차체 yaw와 각속도를
사용합니다. 내부 예상 yaw를 적분해서 완료했다고 판정하지 않습니다. 목표 이동 방향에서
드라이브 모드 오프셋을 빼 차체 목표각으로 변환하므로 FORWARD/REVERSE/LEFT/RIGHT를
동일한 절차로 처리할 수 있습니다. 이 프로파일은 **각가속도 제한**이며 각저크 제한은
구현하지 않았습니다.

수치: $r_{max}$(`yaw_rate_max`) 0.3 rad/s, $a_{max}$(`yaw_rate_accel_max`) 0.3 rad/s²,
$\varepsilon_\psi$(`yaw_tolerance_rad`) 0.02 rad, $\varepsilon_r$(`yaw_rate_tolerance`) 0.02 rad/s.

`command_callback()`에서 `publish_command()`([planner_node.cpp:1388](simp_planner/simp_planner_cpp/src/planner_node.cpp#L1388))로
내보내는 채널은 평소 경로 추종과 동일합니다 — `maneuver_.sample(dt, measured_yaw, measured_yaw_rate, mode_supervisor_)`가
값을 반환하면 그걸 보내고, `std::nullopt`면 기존처럼 `active_plan_`에서 샘플링합니다 (2-4절).
새 메시지 타입은 필요 없습니다 — `geometry_msgs/Twist`가 이미 `angular.z`로 yaw_rate를 나릅니다.

`reference_buffer_.target_body_yaw()`로 구한 목표각은 `command_callback()`에서
`maneuver_.trigger()`에 전달합니다. 이 목표각은 플래너 내부에서만 사용하고 차량에는
각속도 명령을 전송합니다.

### 2-4. `command_callback()`이 뭘 publish할지 결정하는 기준

```cpp
if (!mode_supervisor_.ready()) {
  // 기존 그대로: StoppingForChange면 mode_stop_ 감속 샘플, 아니면 zero_command (무변경)
} else if (auto cmd = maneuver_.sample(command_dt_, current_body_yaw_, current_body_yaw_rate_, mode_supervisor_)) {
  publish_command(*cmd, active_plan_id, stamp_ns);           // 신규: 회전 프로파일 스트리밍
} else {
  // 기존 그대로: active_plan_에서 sample_body_command (무변경)
}
```

실제로 추가되는 분기는 가운데 `else if` 하나뿐입니다.

---

## 3. 회전 가능성 사전 검사

제자리턴을 시작하기 전에, 그 자리에서 회전해도 충돌이 없는지 확인하는 **1단계 게이트**를
추가합니다. **타이밍**: 1-3절의 트리거 시점, 즉 `set_requested_mode(SpotTurn)`을 호출하기
**직전**에 검사합니다 — 실패하면 `AligningWheels`로 아예 진입하지 않습니다.

이미 있는 유틸리티를 그대로 재사용하면 됩니다 — 새로 만들 게 거의 없습니다:

- `circumscribed_radius(length, width, margin)` ([core.hpp:733](simp_planner/simp_planner_cpp/include/simp_planner/core.hpp#L733)) — 차량을 감싸는 원의 반지름. `hypot(0.5*length, 0.5*width) + margin`, 즉 전장/전폭 절반의 피타고라스(대각선)
- `Costmap2D::clearance_single_circle(x, y, radius)` ([core.hpp:391](simp_planner/simp_planner_cpp/include/simp_planner/core.hpp#L391)) — 그 원이 장애물과 얼마나 떨어져 있는지

`allocation_min_clearance()`([planner_node.cpp:557-570](simp_planner/simp_planner_cpp/src/planner_node.cpp#L557-L570))가
이미 정확히 이 조합으로 궤적 전체의 최소 clearance를 구하고 있어서, 그 자리에서 한 점(차량
현재 위치)에 대해서만 호출하면 됩니다:

```cpp
bool spot_turn_feasible(const Costmap2D& costmap, double x, double y,
                         const VehicleConfig& vehicle, double safety_margin) {
  const double radius = circumscribed_radius(
      vehicle.length, vehicle.width, vehicle.footprint_margin) + safety_margin;
  return costmap.clearance_single_circle(x, y, radius) > 0.0;
}
```

**결정된 수치**: `vehicle.length`/`vehicle.width`/`footprint_margin`은
`VehicleConfig`([core.hpp:159](simp_planner/simp_planner_cpp/include/simp_planner/core.hpp#L159))의
코드 기준값을 그대로 읽어서 씁니다 — 전장 3.0 m, 전폭 2.0 m, `footprint_margin` 0.0 m.

$$r_{base} = \operatorname{hypot}(0.5 \times 3.0,\ 0.5 \times 2.0) = \operatorname{hypot}(1.5,\ 1.0) \approx 1.803\ \text{m (소수 3째 자리 반올림)}$$

여기에 `spot_turn_safety_margin`만 더해서 최종 검사 반경이 됩니다. `spot_turn_safety_margin`은
0.0 m입니다 — 최종 검사 반경은 1.803 m입니다.

**충돌 시 동작**: `spot_turn_feasible()`이 false인 동안은 `AligningWheels`로 진입하지 않고
그 자리에서 정지(0속도) 유지 — 다음 틱에 다시 검사합니다. 장애물이 사라지는 순간 바로
트리거되어 정렬→회전이 진행됩니다. 타임아웃으로 끊거나 에러 상태로 빠지는 절차는 두지
않습니다 — 무한 재시도가 기본 동작입니다.

---

## 4. 차량 시뮬레이터 — 실제 회전 거동

### 4-1. 기존 모델의 한계

지금 `DriveModeTransitionModel`([mode_transition.py:17](planar_velocity_sim/planar_velocity_sim/mode_transition.py#L17))은
**실제로 차량을 회전시키지 않습니다** — 그냥 `transition_duration_sec`(기본 2초) 동안
Vx=Vy=YawRate=0을 유지하다가, 시간이 다 되면 `current_mode` 값만 순간적으로 바뀝니다 (몸체
yaw는 안 변함). 이건 크랩 조향이라 모드 전환 자체가 조향 기하학만 바꾸고 물리적 회전이 필요
없기 때문에 지금까지 문제가 안 됐던 겁니다.

제자리턴은 몸체 yaw 자체가 실제로 돌아야 하지만, 별도의 회전 제어 클래스를 새로 만들 필요는
없습니다 — 2절의 레이어 분리 덕분입니다. 이유는 아래에서 확인합니다.

### 4-2. 왜 별도 모델이 필요 없는가 — 코드로 확인됨

`planar_velocity_sim_node.py`의 매 틱 루프([planar_velocity_sim_node.py:158](planar_velocity_sim/planar_velocity_sim/planar_velocity_sim_node.py#L158)):

```python
self.applied_vx, self.applied_vy, self.applied_yaw_rate = (
    self.mode_model.applied_velocity(
        self.command_vx, self.command_vy, self.command_yaw_rate
    )
)
```

그리고 `applied_velocity()`([mode_transition.py](planar_velocity_sim/planar_velocity_sim/mode_transition.py))는:

```python
def applied_velocity(self, vx, vy, yaw_rate):
    if self.status == VehicleModeStatus.ALIGNING:
        return 0.0, 0.0, 0.0
    return float(vx), float(vy), float(yaw_rate)
```

(`transition_in_progress`/`transition_complete` 두 bool이 `status`
(`STATUS_ALIGNING`/`STATUS_READY`) 필드 하나로 합쳐졌습니다 — 제자리턴과 별개로 이미
구현/커밋됐습니다.)

ROTATING 구간은 정의상 바퀴 정렬이 **이미 끝난**(`ready()==true`, 즉 `status ==
STATUS_READY`) 상태입니다. 그러면 이 함수는 `/cmd_vel`로 들어온 값을 **그대로 통과**시킵니다 — 즉
2-3절에서 플래너가 스트리밍하는 `(vx=0, vy=0, yaw_rate=r_cmd)`가 **아무 수정 없이 이미
올바르게** 몸체에 적용되고, `integrate_body_velocity`가 yaw를 정상적으로 적분합니다.

**시뮬레이터는 모드 4 수용을 구현했습니다.** `VALID_DRIVE_MODES`
([mode_transition.py:6](planar_velocity_sim/planar_velocity_sim/mode_transition.py#L6))가 이전에는
`(0, 1, 2, 3)`이라 모드 4(SpotTurn)를 요청하면
`mode_command_callback()`([planar_velocity_sim_node.py:107-111](planar_velocity_sim/planar_velocity_sim/planar_velocity_sim_node.py#L107-L111))에서
거부됐습니다. `(0, 1, 2, 3, 4)`로 확장하고, `initial_drive_mode`/`command()`의 검증
메시지도 함께 변경했습니다.

회전 완료를 누가 판정하는지도 이걸로 명확해집니다. ROTATING 구간에서는 `DriveModeState`의
`status` 필드가 애초에 움직이지 않습니다(이미 `STATUS_READY`로 고정된 채 유지) — 기다릴
차량 피드백 자체가 없으므로, 완료 판정은 처음부터 끝까지 플래너 내부 $e_\psi$/$r$ 기준(2-3절)만으로
이뤄지는 게 맞습니다.

---

## 5. 시나리오 phase 관리 — 크랩 검증 phase

`spot_turn_course` 시나리오(`build_spot_turn_crab_course_phases()`,
[scenario_definition.py](simp_planner_tools/simp_planner_tools/scenario_definition.py))는 FORWARD와
LEFT(크랩)를 번갈아 7개 phase로 구성합니다. phase 분할은 모드가 바뀌는 곳(1-5절)에서만 하므로,
각 급코너는 자신이 속한 FORWARD phase의 경로 배열 안에 들어갑니다 — 크랩 phase는 급코너와
무관하게 순수 횡이동만 검증합니다:

```
forward_1_with_turn  FORWARD  40도 코너 + 곡선 구간 포함 (72.0 m)
crab_1                LEFT     24.0 m
forward_2_with_turn  FORWARD  -60도 코너 포함 (36.6 m)
crab_2                LEFT     24.0 m
forward_3_with_turn  FORWARD  70도 코너 포함 (36.6 m)
crab_3                LEFT     24.0 m
forward_4_plain       FORWARD  곡선 구간, 코너 없음 (36.0 m)
```

크랩 phase는 몸체 헤딩을 바꾸지 않으므로(옆으로만 이동), 크랩 바로 다음 FORWARD phase의
제자리턴은 항상 그 크랩에 진입하기 직전의 헤딩에서 시작합니다.

---

## 6. 전체 흐름 요약 (한 코너 기준)

```
[시나리오 매니저] phase(모드 하나) 경로 발행 -- 급코너가 여러 개 있어도 한 phase 전체 (1-5절)
      │
      ▼
[플래너] path_callback(): 이미 계산하는 kappa(estimate_curvature_from_yaw)를 스캔 (1-2절)
      │  급코너 없음 ↓                     급코너 있음(|kappa|>threshold) → reference_path_ = 그 지점 앞까지만
      ▼                                          │       pending_post_turn_path_ = 나머지(절대좌표 스냅샷)
정상 주행 (전체 경로 추종)                        │       hard_change 강제 (soft/hard 판정과 무관, 1-2절)
                                                   ▼
                                     [플래너] 잘린 경로를 정상 추종 → 끝에서 자연히 정지
                                                   ▼        (종단 정지 로직 그대로, 신규 코드 없음)
                              [command_callback, 매 틱] spot_turn_arrived() + spot_turn_feasible() 검사 (1-3/3절)
                                             │  둘 다 만족 ↓      아니면 → 정지 유지, 매 틱 재검사
                                             ▼
      maneuver_.trigger(target_yaw, mode_supervisor_.requested_mode())  (1-3절)
      mode_supervisor_.set_requested_mode(SpotTurn)  -- target_yaw는 인자로 안 넘어감
      → maneuver_.state() == AligningWheels
                                             ▼
┌─────────────────────────── STOPPING + ALIGN_FOR_SPOT_TURN ───────────────────────────┐
│ [DriveModeSupervisor] 기존 로직 그대로 (2-2절, 무변경)                                  │
│   StoppingForChange: JerkLimitedSafetyStop으로 감속, BodyCommand 스트리밍               │
│   → 정지 완료 → WaitingForCompletion: 차량이 바퀴를 SpotTurn 자세로 재구성              │
│   [차량] DriveModeState{current_mode=SpotTurn, status=STATUS_READY}로 확정              │
└──────────────────────────────────────┬─────────────────────────────────────────────────┘
                                        │ mode_supervisor_.ready() == true
                                        ▼
                    maneuver_.on_mode_ready(mode_supervisor_)
                    → rotation_.engage(target_yaw_), state() == Rotating
                                        ▼
┌────────────────────────────────── ROTATING (신규) ───────────────────────────────────┐
│ [플래너] 매 틱 maneuver_.sample(dt, mode_supervisor_) → BodyCommand{vx=0,vy=0,yaw_rate=r_cmd} │
│          (기존 publish_command() 채널 그대로, 2-3/2-4절)                                │
│ [차량 시뮬레이터] status==STATUS_READY라 들어온 값을 그대로 통과 (4절, 무수정)          │
│ rotation_.done()이 |e_ψ|<ε_ψ && |r|<ε_r 되는 순간 sample() 내부에서 아래 전이 실행       │
└──────────────────────────────────────┬─────────────────────────────────────────────────┘
                                        │ rotation_.done() == true
                                        ▼
      mode_supervisor_.set_requested_mode(external_requested_mode_)
      → maneuver_.state() == AligningWheels (역방향)
                                        ▼
┌─────────────────────────── ALIGN_FOR_REGULAR ────────────────────────────────────────┐
│ [DriveModeSupervisor] 위와 동일한 경로, 방향만 반대 (2-2절)                             │
│ (이미 정지 상태이므로 대개 StoppingForChange 없이 바로 WaitingForCompletion)             │
└──────────────────────────────────────┬─────────────────────────────────────────────────┘
                                        │ mode_supervisor_.ready() == true
                                        ▼
                    maneuver_.on_mode_ready(...) → state() == Inactive
                    reference_path_ = pending_post_turn_path_ 설치 (다시 스캔, 1-2절)
                    [플래너] 정상 재계획 재개, 나머지 구간 추종 시작
```

---

## 7. 재사용 vs 신규 컴포넌트 정리

| 구분 | 컴포넌트 | 재사용/신규 |
|---|---|---|
| 급코너 탐지 + 경로 절단 (플래너 내부, C++) | `path_callback()`의 `estimate_curvature_from_yaw()` 결과 스캔 | 절단 로직만 신규 — 곡률 계산 자체는 기존 것 그대로 재사용, 별도 탐지 함수 안 만듦 (1-2절) |
| Phase 분할 (시나리오) | `ScenarioPhase` | 구조체 변경 없음 — 단, 분할 기준이 모드 전환으로 바뀜(급코너는 더 이상 분할 이유 아님, 1-5절) |
| 경로 저작 검증 | `ScenarioPath.from_arrays()` / `join_with_turns()` | 검사 자체는 유지 — 이음매 인덱스만 `skip_continuity_at`로 예외 처리 (0절) |
| 계획 무효화(hard_change) | `is_soft_reference_continuation_locked()` | **판정 로직은 무변경** — 코너 절단 메시지는 그 판정과 무관하게 `hard_change` 강제 (1-2절) |
| 도착 확인 | `reference_path_->project()` + `s_max()` 비교 | 신규이지만 `terminal_hold_active()`와 동일한 패턴 재사용 (1-3절) |
| 회전 트리거 | `command_callback()` 매 틱 재시도 | 신규 — 시나리오 관여 없음 (1-3절) |
| 회전 가능성 검사 | `circumscribed_radius` + `clearance_single_circle` | 완전 재사용 — `VehicleConfig` 코드 기준값(전장 3.0m/전폭 2.0m/margin 0.0) 그대로, `spot_turn_safety_margin`만 신규(0.0) (3절) |
| 모드 enum | `DriveMode::SpotTurn = 4` | 필드값 추가만 (기존 enum 재사용) |
| **바퀴 정렬 상태머신** | `DriveModeSupervisor` / `DriveModeControlState` | **완전 재사용, 무변경** — STOPPING/ALIGN이 전부 기존 상태 그대로 (2-2절) |
| 피드백 메시지 | `DriveModeState.msg` | `status`(`STATUS_ALIGNING`/`STATUS_READY`) 필드 병합은 제자리턴과 무관하게 이미 완료 — `SPOT_TURN=4` 상수만 추가하며 직렬화되는 데이터 필드는 유지 |
| 명령 메시지 | `/vehicle/drive_mode_command` (`UInt8`) | **완전 재사용, 변경 없음** — target_yaw가 와이어를 안 탐 (2절) |
| **매뉴버 상위 상태** | `SpotTurnManeuverState` (신규, 플래너 내부 전용) | 신규이지만 값 3개짜리 작은 enum (2-3절) |
| 회전 프로파일 생성기 | `YawRotationProfile` (`JerkLimitedSafetyStop` 자매 클래스) | 신규 — 유일하게 진짜 새로 만드는 제어 로직 (2-3절) |
| 속도 명령 채널 | `publish_command()` / `geometry_msgs/Twist` | **완전 재사용** — 회전도 이 채널로 yaw_rate만 실어 보냄 |
| 차량 시뮬레이터 | `DriveModeTransitionModel` / `applied_velocity()` | **거의 완전 재사용** — `VALID_DRIVE_MODES`에 4 추가하는 것 외 무수정 (4절) |
| 시각화 | `vehicle_visualizer_node`/`debug_plot_node`의 `MODE_NAMES` | SPOT_TURN 모드명과 planner status의 접근/정렬/회전/복귀/공간 대기 상태 표시 구현 |

---

## 8. 남은 작업

### 8-1. 구현 중 발견한 문제와 처리 상태

- [x] **동일 위치 두 점이 Python 경로 처리에서 유실/거부됨**: 이음매 인덱스를
  `ScenarioPath`에 보관하고 `local_slice()`/`clipped()`/`translated()`로 전달합니다.
  위치 투영에서는 길이 0인 이음매를 제외하고 앞뒤 실제 주행 구간에 투영합니다.
- [x] **이음매가 수신 창 끝에 걸리면 잔여 경로가 1~2점뿐임**: Python 발행 창을 필요한
  만큼 늘려 이음매 뒤 최소 3개 사용점과 곡률 추정용 추가점을 확보합니다. C++에서는
  모든 분할 구간을 먼저 검증하여 예외 발생 시 대기 상태가 반쯤 저장되지 않게 합니다.
- [x] **급코너 곡률이 절단 종점에 잔류함**: 주행 구간 마지막 곡률을 0으로 처리합니다.
  정상 주행의 `curvature_max=0.20` 제한과 soft/hard 연속성 기준은 완화하지 않습니다.
- [x] **출발점 또는 멀리 떨어진 정지 위치에서 회전함**: 정지 속도와 투영 잔여 거리,
  실제 종점까지 거리(0.20 m)를 모두 검사합니다. 코너 절단은 hard 변경으로 처리합니다.
- [x] **메시지마다 s가 초기화되어 완료 코너를 다시 처리하거나 후속 코너를 건너뜀**:
  완료한 코너의 고정 위치와 전후 방향을 기억하고, 새 창에 남은 완료 코너 앞부분을
  제거합니다. 모드/프레임 변경 시 기록을 초기화합니다.
- [x] **복귀 정렬 중 새 모드 요청으로 상태가 고착됨**: 실제 supervisor 복귀 요청도
  최신 모드로 바꾸며 Ready 상태를 매 command tick 확인합니다.
- [x] **이동 방향과 차체 방향 혼동/실제 회전 오차 방치**: 모드 오프셋을 제거한 목표
  차체각을 사용하고, 완료는 실측 yaw 및 각속도로 판정합니다.
- [x] **costmap crop에서 실제 차량이 제외되거나 경로 변경 후 crop이 그대로임**:
  차량 회전 원을 crop에 포함하고 원본 지도뿐 아니라 경로 revision도 비교합니다.
- [x] **안전정지인데 진단은 OK / 회전 상태가 모드 불일치로 표시됨**:
  planner status에 제자리턴 접근/정렬/회전/복귀/공간 대기를 표시합니다. debug 진단은
  `NO_SAFE_PLAN_SAFETY_STOP`을 실패로 표시하고 RViz 텍스트에도 maneuver를 노출합니다.
- [x] C++ 코어/제자리턴 회귀 테스트 2개 실행 파일 통과. 관련 Python 테스트 52개 통과.
- [x] 실제 ROS, 목표속도 4.2 m/s: 전진→회전→재출발→크랩 전환, 7개 phase 완료.
- [ ] 기본 시나리오 속도(전진 1.5 / 크랩 1.0 m/s)의 전체 ROS 주행 확인. 추가 실행의
  로컬 소켓 권한 요청이 거절되어 실행하지 않았습니다. C++에서는 목표 1.5 m/s로
  코너 전후 가속을 검증했습니다.
- [ ] 기존 그래프 리포트 테스트 2건 정비. `test_planning_call_count_report.py`는 현재
  리포트의 세부 timing 필드와 활성 계획 수집 조건을 반영하지 못해 실패합니다.
  해당 코드와 테스트는 이번 변경 전 HEAD와 동일하며 이번 작업에서 수정하지 않았습니다.

### 8-2. 동작 범위와 후속 확인이 필요한 사항

- **같은 모드의 진짜 경로 교체/같은 코너 재방문**: 현재 인터페이스에는 route/phase ID가
  없습니다. 코너 처리 중에는 수신한 절대좌표 스냅샷을 마무리하므로 같은 모드의 새
  경로를 즉시 취소·교체하지 않습니다. 같은 위치와 전후 방향을 가진 코너를 같은 모드에서
  다시 방문하는 경로에는 별도 세대 식별 정책이 필요합니다.
- **회전 도중 새 장애물**: 이 설계의 충돌 검사는 회전 시작 전 게이트입니다. 회전 중
  새 장애물이 진입했을 때 각속도를 감속하고 재개하는 동작은 별도 설계/검증이 필요합니다.
- **센서 피드백 지연/중단 및 실제 차량**: 실측 피드백을 사용하지만 통신 두절 감시,
  휠 슬립, 실제 조향/구동 응답과 각저크 제한은 이번 이상적 시뮬레이터 검증 범위 밖입니다.
- **코너/phase 종점 오차**: 기존 종단 제어의 정지 오프셋과 허용 오차 때문에 정확한
  동일 좌표에서 정지하지 않을 수 있습니다. 0.6 m stub의 접근/재출발은 4.2 m/s 설정으로
  확인했으며 기본 속도 설정의 전체 주행은 남아 있습니다. 입력 코너 자체에는 이전
  0.15 m 연결 구간을 두지 않습니다.
- **동시 실행 간섭**: `/cmd_vel`, `/odom`, 경로/모드 토픽을 공유하는 launch를 같은 ROS
  domain에서 동시에 실행하면 명령이 섞입니다. 검증 실행은 별도 ROS_DOMAIN_ID로
  분리합니다. namespace 기반 다중 차량 지원은 별도 작업입니다.

- **목표속도 오버슈트**: 목표 4.2 m/s 실행에서 최고 4.321 m/s가 관측됐습니다.
  이번에 변경하지 않은 종방향 속도 제어의 정착 특성과 허용 범위는 별도 확인이 필요합니다.

### 8-3. 검증 기록과 재현 방법

| 항목 | 확인 결과 |
|---|---|
| 빌드 | `simp_planner_msgs`, `simp_planner_cpp`, `planar_velocity_sim`, `simp_planner_tools` 성공 |
| C++ | 기존 코어 테스트 + 제자리턴 입력/접근/재출발/피드백/복귀/충돌 게이트 테스트 통과 |
| Python | 기존 리포트 테스트 파일을 제외한 52개 통과 |
| 실제 ROS 목표속도 | 4.2 m/s, 장애물 없는 작성 코스 |
| 코스 완료 | 7개 phase, 모드 전환 6회, 제자리턴 3회, 약 151.4초 |
| 회전 시작점과 작성 코너 사이 거리 | 0.067 / 0.116 / 0.115 m |
| 최종 상태 | `COMPLETE`, 속도 0, 종점 잔여 거리 0.077 m |

결과: [ROS 실행 요약](validation/spot_turn_ros_4p2_summary.json).
C++ 테스트의 회전 완료 검사는 실제 피드백에 일부러 지연/구동 오차를 넣어 확인합니다.
ROS 검증 스크립트에도 복귀 시 실측 yaw 오차 검사를 추가했으나, 이 추가 검사 및 기본 속도
실행은 후속 ROS 재실행 때 확인해야 합니다.

```bash
source install/setup.bash
ctest --test-dir build/simp_planner_cpp --output-on-failure
python3 -m pytest -q planar_velocity_sim/test simp_planner_tools/test \
  --ignore=simp_planner_tools/test/test_planning_call_count_report.py

# 로컬 ROS 소켓 통신이 가능한 환경에서 실행. GUI 없이 별도 domain을 사용한다.
python3 validation/run_spot_turn_ros.py --speed -1 --timeout 420 \
  --output /tmp/simp_spot_turn_ros_default
```
