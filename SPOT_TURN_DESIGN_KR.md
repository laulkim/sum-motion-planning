# 제자리턴 시나리오/기능 설계 초안

> "확정 스펙"이 아니라 구현 착수 전 합의를 위한 초안이며, 맨 아래
> [8. 열린 질문](#8-열린-질문-구현-착수-전-결정-필요) 항목들은 아직 결정되지 않았습니다.

## 0. 배경 — 왜 지금 구조로는 안 되는가

현재 `ScenarioPath.from_arrays()`는 경로 생성 시점에 아래 두 조건을 이미 강제로 검사합니다
([scenario_path.py:96-98](simp_planner_tools/simp_planner_tools/scenario_path.py#L96-L98)):

```python
if np.max(np.abs(kappa)) > curvature_limit + 1.0e-9:          # 기본 0.2
    raise ValueError("Scenario path exceeds the curvature limit")
...
if np.max(np.abs(yaw_step)) > math.radians(yaw_step_limit_deg):  # 기본 15도
    raise ValueError("Scenario path contains a heading discontinuity")
```

즉 **40도/60도/70도급 급코너는 애초에 "경로 점들의 배열"로 표현이 불가능**합니다 — 하나의
연속된 `ScenarioPath` 안에 그런 지점이 있으면 시나리오 로딩 단계에서 바로 예외가 납니다.

이게 이번 설계의 핵심 전제입니다: **제자리턴 지점은 "경로 위의 한 점"이 아니라, 기존
`switch_s`([scenario_definition.py:34](simp_planner_tools/simp_planner_tools/scenario_definition.py#L34))처럼
"phase와 phase 사이의 경계 이벤트"로 다뤄야** 합니다. 다행히 이 패턴(정지 지점 도달 감지 →
다음 phase로 전환 → 새 경로 발행)은 크랩/리버스 모드 전환용으로 이미 완성돼서 동작 중입니다
([scenario_manager_node.py:453-498](simp_planner_tools/simp_planner_tools/scenario_manager_node.py#L453-L498),
`switch_to_next_phase`). 제자리턴은 이 인프라를 재사용하는 **새로운 종류의 phase 전환 액션**으로
설계합니다.

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

### 1-2. 플래너 내부 탐지 함수 (C++, 신규)

`scenario_path.py`(Python, 시나리오 저작 쪽)가 아니라 **`simp_planner_cpp` 안에** 둡니다
(`core.cpp`, `estimate_curvature_from_yaw` 옆에 나란히):

```cpp
// core.hpp / core.cpp
struct SpotTurnPoint {
  double s;            // 레퍼런스 경로 위 arc-length
  double target_yaw;   // 꺾인 후 목표 헤딩 (map/odom 프레임, 라디안)
};

std::optional<SpotTurnPoint> detect_spot_turn_point(
    const std::vector<double>& x, const std::vector<double>& y,
    const std::vector<double>& yaw, double jump_threshold_rad);
```

`path_callback()`이 새 레퍼런스를 받을 때마다(기존 곡률 계산과 같은 자리) 이 함수도 같이
돌려서, 지금 들고 있는 레퍼런스 경로 안에 제자리턴이 필요한 지점이 있는지 매번 다시 확인하고
저장해 둡니다. `target_yaw`는 이 시점부터 끝까지 **플래너 프로세스 내부에만** 머무릅니다 —
2절에서 보듯 어떤 메시지 인터페이스로도 외부로 나가지 않습니다.

### 1-3. 트리거 — "요청 vs 현재 비교"가 아니라 "그 지점 도달"

다른 모드는 `requested_mode_ != current_mode_`가 되는 순간(외부에서 새 요청이 들어온 순간)
트리거됩니다. 제자리턴은 다릅니다 — **차량이 `detect_spot_turn_point()`가 찾아낸 지점(s)에
도달하는 순간**이 트리거입니다. `path_callback()`은 그 지점에서 다음을 실행합니다:

```cpp
if (spot_turn_feasible(costmap, x, y, vehicle)) {              // 3절
  maneuver_.trigger(spot_turn_point->target_yaw,
                     mode_supervisor_.requested_mode());       // target_yaw는 maneuver_ 멤버로 저장 (2-3절)
  mode_supervisor_.set_requested_mode(DriveMode::SpotTurn);    // target_yaw는 여기 안 들어감
}
```

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

### 1-5. Phase 분할 자체는 시나리오에 여전히 필요함 — 단, 제자리턴 의미는 없음

`ScenarioPath.from_arrays()`가 여전히 `yaw_step_limit_deg`(15도)/`curvature_limit`을 강제
검사하기 때문에([scenario_path.py:96-98](simp_planner_tools/simp_planner_tools/scenario_path.py#L96-L98)),
40도/60도/70도급 급코너를 가진 원본 웨이포인트는 시나리오 단계에서 애초에 하나의 연속
`ScenarioPath`로 만들 수 없습니다. 그래서 **phase 분할 자체는 여전히 필요**합니다 — 다만 그
분할점에는 **제자리턴 관련 필드가 전혀 붙지 않습니다**. `ScenarioPhase`는 원래
형태 그대로 유지합니다 (신규 필드 없음):

```python
@dataclass(frozen=True)
class ScenarioPhase:
    name: str
    path: ScenarioPath
    cruise_speed: float
    switch_s: float | None = None
    # 제자리턴 관련 필드 없음 -- 그건 전부 플래너 내부 판단 (1-2, 1-3절)
```

phase 경계에서 시나리오가 하는 일은 지난번에 정리한 대로 그대로입니다: 정지 지점 도달 감지 →
다음 phase 레퍼런스 발행, 그게 전부입니다. "여기서 얼마나 돌아야 하는지"는 시나리오가 전혀
모릅니다 — 플래너가 새로 받은 레퍼런스의 헤딩을 보고 알아서 계산합니다.

### 1-6. 테스트 시나리오: 다양한 각도의 코너

요청하신 대로 40도/60도/70도 등 서로 다른 각도의 코너를 포함한 새 시나리오(예:
`spot_turn_course`)를 하나 만들어서, 코너 각도별로 회전이 잘 되는지 검증합니다. phase는
여전히 코너 개수만큼 나뉘지만(1-5절, 구조적 제약 때문), 회전 여부/목표각 판단은 전부
플래너가 매 코너에서 스스로 계산합니다 — 시나리오는 코너가 있는 경로만 던져주면 됩니다.

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
상태머신이 지금과 완전히 동일하게 돌아갑니다 — 이게 바로 요청하신 "STOPPING → 바퀴 정렬"
구간의 실체입니다. 신규 코드가 필요 없습니다:

| 요청하신 단계 | 기존 `DriveModeControlState` 값 | 진입 조건 ([runtime.cpp:68](simp_planner/simp_planner_cpp/src/runtime.cpp#L68) `state()`) |
|---|---|---|
| STOPPING | `StoppingForChange` | `requested_mode_ != current_mode_`, 아직 속도 있음(`measured_speed > stop_speed_threshold`) |
| ALIGN_FOR_SPOT_TURN / ALIGN_FOR_REGULAR | `WaitingForCompletion` | 정지 완료, 차량이 바퀴 재구성 중(차량 피드백의 `transition_in_progress`) |
| (정렬 완료) | `Ready` (`ready()==true`) | `current_mode_ == requested_mode_` 확정 |

`command_callback()`도 이 구간에서 지금과 똑같이 동작합니다 — 아직 속도가 있으면
`mode_stop_`(`JerkLimitedSafetyStop`, [runtime.hpp:128](simp_planner/simp_planner_cpp/include/simp_planner/runtime.hpp#L128))으로
감속하고, 정지 후엔 `zero_command()`를 유지하며 `ready()`를 기다립니다
([planner_node.cpp:1241](simp_planner/simp_planner_cpp/src/planner_node.cpp#L1241) `command_callback`,
`!mode_ready` 분기). **ALIGN_FOR_SPOT_TURN과 ALIGN_FOR_REGULAR도 코드상으로는 완전히 같은
경로**입니다 — `requested_mode_`가 무엇이냐(SpotTurn이냐 `external_requested_mode_`냐)만 다를 뿐,
`DriveModeSupervisor`도 `DriveModeState.msg`도 방향을 구분할 필요가 전혀 없습니다.

즉 `DriveModeSupervisor`, `DriveModeState.msg`, `/vehicle/drive_mode_command`(`UInt8`) —
**전부 완전히 그대로, 필드 1개도 추가하지 않습니다.** `set_requested_mode()`도 시그니처
변경 없이 기존 그대로(`DriveMode` 단일 인자)입니다. **`target_yaw`는 이 인터페이스 어디에도
들어가지 않습니다** — 아래 2-3절의 상위 객체 안에만 존재합니다.

### 2-3. 실제 회전 레이어 — 진짜 새로 만드는 부분

바퀴 정렬(`ready()==true`)이 끝난 뒤에 실제로 차체를 돌리는 부분만 신규입니다. 알고리즘은
다음 클래스 하나로 정리됩니다 (`DriveModeSupervisor`보다 상위 레이어, `planner_node.cpp` 신규):

```cpp
enum class SpotTurnManeuverState { Inactive, AligningWheels, Rotating };

class SpotTurnManeuver {
 public:
  // 1-3절 트리거 시점에 1회 호출
  void trigger(double target_yaw, DriveMode external_mode) {
    target_yaw_ = target_yaw;
    external_requested_mode_ = external_mode;
    state_ = SpotTurnManeuverState::AligningWheels;
  }

  // mode_supervisor_.ready()가 새로 true가 될 때마다 호출
  void on_mode_ready(double current_yaw, const DriveModeSupervisor& sup) {
    if (state_ == SpotTurnManeuverState::AligningWheels &&
        sup.requested_mode() == DriveMode::SpotTurn) {
      rotation_.engage(target_yaw_, current_yaw);   // 저크 제한 yaw 프로파일 기동
      state_ = SpotTurnManeuverState::Rotating;
    } else if (state_ == SpotTurnManeuverState::AligningWheels &&
               sup.requested_mode() == external_requested_mode_) {
      state_ = SpotTurnManeuverState::Inactive;      // 복귀 정렬까지 끝남
    }
  }

  // command_callback()이 mode_ready()==true일 때 매 틱 호출
  std::optional<BodyCommand> sample(double dt, DriveModeSupervisor& sup) {
    if (state_ != SpotTurnManeuverState::Rotating) return std::nullopt;
    const BodyCommand cmd = rotation_.sample_and_advance(dt);  // vx=vy=0, yaw_rate=r_cmd
    if (rotation_.done()) {                                     // |e_ψ|<ε_ψ && |r|<ε_r (8절)
      sup.set_requested_mode(external_requested_mode_);
      state_ = SpotTurnManeuverState::AligningWheels;
    }
    return cmd;
  }

  SpotTurnManeuverState state() const { return state_; }

 private:
  SpotTurnManeuverState state_ = SpotTurnManeuverState::Inactive;
  double target_yaw_ = 0.0;
  DriveMode external_requested_mode_ = DriveMode::Forward;
  YawRotationProfile rotation_;  // JerkLimitedSafetyStop 자매 클래스, 2-3절 하단
};
```

`YawRotationProfile`(이름 미정)은 `JerkLimitedSafetyStop`과 같은 저크 제한 패턴을 각속도
축에 적용합니다:

$$e_\psi = \operatorname{wrap}(\psi_{target} - \psi), \qquad r_{cmd} = \operatorname{clamp}(K_\psi e_\psi,\, -r_{max},\, r_{max})$$

목표 근처에서는 이 $r_{cmd}$를 저크 제한으로 0까지 부드럽게 줄이고,
$|e_\psi| < \varepsilon_\psi \,\&\&\, |r| < \varepsilon_r$이 되면 `done()`이 true를 반환합니다.

`command_callback()`에서 `publish_command()`([planner_node.cpp:1388](simp_planner/simp_planner_cpp/src/planner_node.cpp#L1388))로
내보내는 채널은 평소 경로 추종과 동일합니다 — `maneuver_.sample(dt, mode_supervisor_)`가
값을 반환하면 그걸 보내고, `std::nullopt`면 기존처럼 `active_plan_`에서 샘플링합니다 (2-4절).
새 메시지 타입은 필요 없습니다 — `geometry_msgs/Twist`가 이미 `angular.z`로 yaw_rate를 나릅니다.

`target_yaw_`를 아는 쪽(`trigger()`를 호출하는 `path_callback()`)과 그 값을 소비하는 쪽
(`on_mode_ready()`)이 둘 다 이 클래스 하나 안에 있기 때문에, `target_yaw`는 `SpotTurnManeuver`
객체 밖으로 나갈 일이 없습니다.

### 2-4. `command_callback()`이 뭘 publish할지 결정하는 기준

```cpp
if (!mode_supervisor_.ready()) {
  // 기존 그대로: StoppingForChange면 mode_stop_ 감속 샘플, 아니면 zero_command (무변경)
} else if (auto cmd = maneuver_.sample(command_dt_, mode_supervisor_)) {
  publish_command(*cmd, active_plan_id, stamp_ns);           // 신규: 회전 프로파일 스트리밍
} else {
  // 기존 그대로: active_plan_에서 sample_body_command (무변경)
}
```

실제로 추가되는 분기는 가운데 `else if` 하나뿐입니다.

---

## 3. 회전 가능성 사전 검사 (요청 3번)

제자리턴을 시작하기 전에, 그 자리에서 회전해도 충돌이 없는지 확인하는 **1단계 게이트**를
추가합니다. **타이밍**: 1-3절의 트리거 시점, 즉 `set_requested_mode(SpotTurn)`을 호출하기
**직전**에 검사합니다 — 실패하면 `AligningWheels`로 아예 진입하지 않고, 다음 틱에 재시도하거나
[8절 Q2](#8-열린-질문-구현-착수-전-결정-필요)에 따라 처리합니다.

이미 있는 유틸리티를 그대로 재사용하면 됩니다 — 새로 만들 게 거의 없습니다:

- `circumscribed_radius(length, width, margin)` ([core.hpp:733](simp_planner/simp_planner_cpp/include/simp_planner/core.hpp#L733)) — 차량을 감싸는 원의 반지름
- `Costmap2D::clearance_single_circle(x, y, radius)` ([core.hpp:391](simp_planner/simp_planner_cpp/include/simp_planner/core.hpp#L391)) — 그 원이 장애물과 얼마나 떨어져 있는지

`allocation_min_clearance()`([planner_node.cpp:557-570](simp_planner/simp_planner_cpp/src/planner_node.cpp#L557-L570))가
이미 정확히 이 조합으로 궤적 전체의 최소 clearance를 구하고 있어서, 그 자리에서 한 점(차량
현재 위치)에 대해서만 호출하면 됩니다:

```cpp
bool spot_turn_feasible(const Costmap2D& costmap, double x, double y,
                          const VehicleConfig& vehicle) {
  const double radius = circumscribed_radius(
      vehicle.length, vehicle.width, vehicle.footprint_margin) + spot_turn_safety_margin;
  return costmap.clearance_single_circle(x, y, radius) > 0.0;
}
```

"원 크기는 넓게 잡아서"라고 하신 부분은 `spot_turn_safety_margin`(추가 여유값, 예: 0.3~0.5m)으로
반영합니다 — 이 값 자체는 8절 열린 질문입니다. 불가능하면(장애물 있으면) 어떻게 할지도
[8절](#8-열린-질문-구현-착수-전-결정-필요)에 정리했습니다.

---

## 4. 차량 시뮬레이터 — 실제 회전 거동 (요청 8번)

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

그리고 `applied_velocity()`([mode_transition.py:88](planar_velocity_sim/planar_velocity_sim/mode_transition.py#L88))는:

```python
def applied_velocity(self, vx, vy, yaw_rate):
    if self.transition_in_progress:
        return 0.0, 0.0, 0.0
    return float(vx), float(vy), float(yaw_rate)
```

ROTATING 구간은 정의상 바퀴 정렬이 **이미 끝난**(`ready()==true`, 즉 `transition_in_progress
== False`) 상태입니다. 그러면 이 함수는 `/cmd_vel`로 들어온 값을 **그대로 통과**시킵니다 — 즉
2-3절에서 플래너가 스트리밍하는 `(vx=0, vy=0, yaw_rate=r_cmd)`가 **아무 수정 없이 이미
올바르게** 몸체에 적용되고, `integrate_body_velocity`가 yaw를 정상적으로 적분합니다.

**실제로 필요한 변경은 딱 한 줄입니다** — `VALID_DRIVE_MODES`
([mode_transition.py:6](planar_velocity_sim/planar_velocity_sim/mode_transition.py#L6))가 지금
`(0, 1, 2, 3)`이라 모드 4(SpotTurn)를 요청하면
`mode_command_callback()`([planar_velocity_sim_node.py:107-111](planar_velocity_sim/planar_velocity_sim/planar_velocity_sim_node.py#L107-L111))에서
그냥 거부됩니다. `(0, 1, 2, 3, 4)`로 확장하고, `initial_drive_mode`/`command()`의 관련 검증
메시지("[0, 3]")만 같이 고치면 됩니다.

회전 완료를 누가 판정하는지도 이걸로 명확해집니다. ROTATING 구간에서는 `DriveModeState`의
`transition_in_progress`/`transition_complete`가 애초에 움직이지 않습니다(이미 `Ready`로
고정된 채 유지) — 기다릴 차량 피드백 자체가 없으므로, 완료 판정은 처음부터 끝까지 플래너
내부 $e_\psi$/$r$ 기준(2-3절)만으로 이뤄지는 게 맞습니다.

---

## 5. 시나리오 phase 관리 — 크랩 검증 phase (요청 5번)

새로 만들 `spot_turn_course` 시나리오의 **마지막 phase 하나를 크랩(Left/Right) 모드로
정의**해서, 같은 시나리오 안에서 "급코너 = 제자리턴"과 "측면 이동 = 크랩"이라는 서로 다른 두
기동 방식이 나란히 검증되도록 합니다. 예:

```
phase 0: FORWARD  (일반 주행, 40도 코너에서 종료 → 제자리턴)
phase 1: FORWARD  (제자리턴 후 재개, 60도 코너에서 종료 → 제자리턴)
phase 2: FORWARD  (제자리턴 후 재개, 70도 코너에서 종료 → 제자리턴)
phase 3: FORWARD  (제자리턴 후 재개, 목적지 근처까지)
phase 4: LEFT     (크랩 모드로 옆 주차 등 마무리 — 기존 crab_switch 패턴 재사용)
```

---

## 6. 전체 흐름 요약 (한 코너 기준)

```
[phase N 주행 중] (maneuver_.state() == Inactive)
      │ (odom 기준 정지 지점 도달: capture 조건과 동일 — s 도달 + 속도 임계값 이하)
      ▼
[시나리오 매니저] phase N+1 경로 발행 (기존과 동일, 대기 없음)
      │
      ▼
[플래너] detect_spot_turn_point()로 새 레퍼런스를 스스로 스캔 (시나리오는 관여 안 함, 1-2절)
      │  급코너 없음 ↓                     급코너 있음 → 그 지점(s)까지는 정상 주행
      ▼                                          │
정상 주행 재개                              [플래너] 차량이 그 지점(s) 도달
                                                   ▼
                                     [플래너] spot_turn_feasible() 검사 (3절)
                                             │  가능 ↓            불가능 → (8절 Q2)
                                             ▼
      maneuver_.trigger(target_yaw, mode_supervisor_.requested_mode())  (1-3절)
      mode_supervisor_.set_requested_mode(SpotTurn)  -- target_yaw는 인자로 안 넘어감
      → maneuver_.state() == AligningWheels
                                             ▼
┌─────────────────────────── STOPPING + ALIGN_FOR_SPOT_TURN ───────────────────────────┐
│ [DriveModeSupervisor] 기존 로직 그대로 (2-2절, 무변경)                                  │
│   StoppingForChange: JerkLimitedSafetyStop으로 감속, BodyCommand 스트리밍               │
│   → 정지 완료 → WaitingForCompletion: 차량이 바퀴를 SpotTurn 자세로 재구성              │
│   [차량] DriveModeState{current_mode=SpotTurn, transition_in_progress→false}로 확정     │
└──────────────────────────────────────┬─────────────────────────────────────────────────┘
                                        │ mode_supervisor_.ready() == true
                                        ▼
                    maneuver_.on_mode_ready(current_yaw, mode_supervisor_)
                    → rotation_.engage(target_yaw_, current_yaw), state() == Rotating
                                        ▼
┌────────────────────────────────── ROTATING (신규) ───────────────────────────────────┐
│ [플래너] 매 틱 maneuver_.sample(dt, mode_supervisor_) → BodyCommand{vx=0,vy=0,yaw_rate=r_cmd} │
│          (기존 publish_command() 채널 그대로, 2-3/2-4절)                                │
│ [차량 시뮬레이터] transition_in_progress==false라 들어온 값을 그대로 통과 (4절, 무수정)  │
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
                    [플래너] 정상 재계획 재개, phase N+1 레퍼런스 추종 시작
```

---

## 7. 재사용 vs 신규 컴포넌트 정리

| 구분 | 컴포넌트 | 재사용/신규 |
|---|---|---|
| 급코너 탐지 (플래너 내부, C++) | `detect_spot_turn_point()` | 신규 (기존 곡률 추정 스타일 재사용, 1-2절) |
| Phase 분할 (시나리오, 구조적 제약) | `ScenarioPhase` | **변경 없음** — 제자리턴 관련 필드 추가 안 함 (1-5절) |
| 회전 트리거 | `path_callback()` 내부 로직 | 신규 — "지점 도달" 감지, 시나리오 관여 없음 (1-3절) |
| 회전 가능성 검사 | `circumscribed_radius` + `clearance_single_circle` | 완전 재사용 |
| 모드 enum | `DriveMode::SpotTurn = 4` | 필드값 추가만 (기존 enum 재사용) |
| **바퀴 정렬 상태머신** | `DriveModeSupervisor` / `DriveModeControlState` | **완전 재사용, 무변경** — STOPPING/ALIGN이 전부 기존 상태 그대로 (2-2절) |
| 피드백 메시지 | `DriveModeState.msg` | **완전 재사용, 변경 없음** |
| 명령 메시지 | `/vehicle/drive_mode_command` (`UInt8`) | **완전 재사용, 변경 없음** — target_yaw가 와이어를 안 탐 (2절) |
| **매뉴버 상위 상태** | `SpotTurnManeuverState` (신규, 플래너 내부 전용) | 신규이지만 값 3개짜리 작은 enum (2-3절) |
| 회전 프로파일 생성기 | (이름 미정, `JerkLimitedSafetyStop` 자매 클래스) | 신규 — 유일하게 진짜 새로 만드는 제어 로직 (2-3절) |
| 속도 명령 채널 | `publish_command()` / `geometry_msgs/Twist` | **완전 재사용** — 회전도 이 채널로 yaw_rate만 실어 보냄 |
| 차량 시뮬레이터 | `DriveModeTransitionModel` / `applied_velocity()` | **거의 완전 재사용** — `VALID_DRIVE_MODES`에 4 추가하는 것 외 무수정 (4절) |
| 시각화 | 기존 `vehicle_visualizer_node`의 상태 텍스트 마커 | 확장 (제자리턴 상태 줄 추가, `maneuver_.state()` 표시) |

---

## 8. 열린 질문 (구현 착수 전 결정 필요)

1. **급코너 자동 탐지 임계값**: `jump_threshold_deg`를 몇 도로 할지 (30도 제안, 확정 아님).
2. **회전 불가능(충돌) 시 동작**: 그냥 그 자리에서 계속 대기(재시도)할지, 에러 상태로 빠질지,
   장애물이 사라질 때까지 무한 대기할지.
3. **회전 프로파일 파라미터**: 각속도/저크 한계(`spot_turn_yaw_rate_max`,
   `spot_turn_yaw_jerk_max`)와, 회전 완료 판정 허용오차($\varepsilon_\psi$, $\varepsilon_r$,
   2-3절)의 구체적 수치.
4. **회전 안전 여유 반지름**: 3절의 `spot_turn_safety_margin` 구체적 수치("넓게"의 정량화).
5. **`DriveModeState.msg`의 `transition_in_progress`/`transition_complete`를 필드 1개로
   합칠지**: 이 메시지는 손대지 않기로 했지만(2-2절), 두 필드가 실질적으로 2가지 조합만
   나온다는 사실 자체는 여전히 유효함. 합치면 `DriveModeSupervisor::ready()`,
   `DriveModeTransitionModel`, `debug_plot_node.py`의 자체 구독,
   `vehicle_visualizer_node.py`의 상태 텍스트까지 전부 같이 고쳐야 해서 마이그레이션 범위가
   있음 — 이번 작업과 묶을지, 별도 작업으로 미룰지.
6. **`external_requested_mode_` 복귀 시점의 예외 상황**: 제자리턴 도중에(`AligningWheels`
   또는 `Rotating` 어느 쪽이든) 시나리오가 `/requested_drive_mode`로 또 다른 모드를 새로
   요청하면(예: phase가 더 진행돼서 Left로 바뀜) 어떻게 할지 — 저장해둔
   `external_requested_mode_`를 그 새 값으로 덮어써서 회전 완료 후 그쪽으로 복귀할지, 아니면
   제자리턴을 먼저 끝내는 동안은(`maneuver_.state() != Inactive`) 새 요청을 무시할지.
