# 제자리턴 시나리오/기능 설계

C++ 플래너/차량 시뮬레이터 구현이 완료됐고 단위 테스트가 통과합니다. 남은 작업은
[8절](#8-남은-작업)의 시나리오 작성과 시각화뿐입니다.

## 0. 배경 — 역할 분담

`ScenarioPath.from_arrays()`가 경로 생성 시점에 강제하던 두 조건은 제거했습니다
([scenario_path.py](simp_planner_tools/simp_planner_tools/scenario_path.py)):

```python
if np.max(np.abs(kappa)) > curvature_limit + 1.0e-9:          # 제거됨
    raise ValueError("Scenario path exceeds the curvature limit")
if np.max(np.abs(yaw_step)) > math.radians(yaw_step_limit_deg):  # 제거됨
    raise ValueError("Scenario path contains a heading discontinuity")
```

그래서 **급코너를 하나의 `ScenarioPath`(= 하나의 phase) 안에 그대로 담을 수 있습니다** —
코너를 "같은 위치, 짧은 구간(5~20cm), 그 사이에서 헤딩만 크게 꺾이는" 두 점으로 표현하면
됩니다. (나머지 검사 — zero-length segment, x-y와 헤딩의 tangent 일치성 — 는 그대로 둡니다.
짧지만 0은 아닌 구간이라 자연히 통과합니다.)

역할 분담:
- **시나리오 매니저(Python)**: phase 분할은 **드라이브 모드가 바뀔 때만** 합니다. 급코너
  때문에 phase를 끊지 않습니다.
- **플래너(C++)**: 받은 레퍼런스 경로 배열 **안에서** 급코너를 스스로 찾고
  ([`find_spot_turn_split_index()`](simp_planner/simp_planner_cpp/include/simp_planner/core.hpp)),
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

### 1-2. 플래너 내부 탐지 + 절단 (C++, 구현됨)

`path_callback()`이 레퍼런스 경로 배열을 받으면, 연속된 두 점 사이에서 헤딩이
`jump_threshold_rad`(30도) 넘게 꺾이는 첫 지점을 직접 스캔합니다 (`core.hpp`/`core.cpp`):

```cpp
std::optional<SpotTurnPoint> detect_spot_turn_point(
    double old_heading, double new_heading, double jump_threshold_rad);

// 배열 s/yaw 안에서 스캔. resolved_max_s보다 앞쪽(이미 처리한 코너)은 건너뛴다 --
// 롤링 윈도우로 같은 구간이 매 틱 재발행돼도 지나간 코너를 또 찾지 않기 위함.
std::optional<std::size_t> find_spot_turn_split_index(
    const std::vector<double>& s, const std::vector<double>& yaw,
    double resolved_max_s, double jump_threshold_rad);
```

지점을 찾으면(`apply_spot_turn_truncation_locked()`), 그 지점 **앞까지만** 쓰는 경로를
`reference_path_`로 쓰고 — 차량이 거기서 종단 정지 로직 그대로 자연히 멈춤 — **뒤쪽 나머지는**
`pending_post_turn_path_`에 남겨 둡니다. 목표각(`yaw[i+1]`)은 `pending_spot_turn_target_yaw_`에
저장하고, 어떤 메시지 인터페이스로도 외부로 나가지 않습니다.

회전이 끝나면(`vehicle_mode_callback()`, `SpotTurnManeuver::on_mode_ready()`가 완료를 보고할
때) `pending_post_turn_path_`를 새 `reference_path_`로 설치합니다 — 이때도 그 구간 안에 코너가
또 있을 수 있어서 같은 함수로 한 번 더 스캔합니다. 한 phase 안에 코너가 여러 개 있어도 이
과정이 반복되면서 순서대로 처리됩니다.

### 1-3. 트리거 — 매 틱 재시도하는 `command_callback()`에서

`path_callback()`은 급코너를 발견하면 목표각만 `pending_spot_turn_target_yaw_`에 기억해 두고,
**실제 트리거는 `command_callback()`이 매 틱 검사**합니다 — 3절의 "장애물이 있으면 매 틱
재시도" 요건을 만족하려면 새 레퍼런스가 들어올 때만 도는 `path_callback()`이 아니라 매 틱
도는 쪽에서 재시도해야 하기 때문입니다:

```cpp
// command_callback() 맨 앞, mode_ready/mode_state를 읽는 바로 그 자리
if (mode_ready && maneuver_.state() == Inactive && pending_spot_turn_target_yaw_ &&
    measured_speed <= mode_change_stop_speed_ &&
    spot_turn_feasible(*costmap_, x, y, config_.vehicle, config_.spot_turn.safety_margin)) {
  maneuver_.trigger(*pending_spot_turn_target_yaw_,
                    mode_supervisor_.requested_mode().value_or(reference_mode_));
  mode_supervisor_.set_requested_mode(DriveMode::SpotTurn);  // target_yaw는 여기 안 들어감
  pending_spot_turn_target_yaw_.reset();
  // mode_ready/mode_state를 이 자리에서 즉시 다시 읽어서 같은 틱에 StoppingForChange로 넘어간다.
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

제자리턴이 진행 중인 동안(`AligningWheels` 또는 `Rotating`) 시나리오가 `/requested_drive_mode`로
또 다른 모드를 새로 요청하면, `mode_supervisor_`를 직접 바꾸지 않고
`maneuver_.set_external_requested_mode()`로 `external_requested_mode_`만 최신 요청값으로
덮어씁니다 — 회전이 끝나면 그 최신 요청 쪽으로 복귀합니다.

### 1-5. Phase 분할은 이제 드라이브 모드 전환에서만 일어남

`ScenarioPath.from_arrays()`의 `yaw_step_limit_deg`/`curvature_limit` 강제 검사를
제거했으므로(0절), 40도/60도/70도급 급코너를 여러 개 가진 웨이포인트도 **하나의 연속
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

### 1-6. 테스트 시나리오: 다양한 각도의 코너

40도/60도/70도 등 서로 다른 각도의 코너를 **한 phase 안에 순서대로** 포함한 새 시나리오(예:
`spot_turn_course`)를 하나 만들어서, 코너마다 회전이 잘 되고 다음 코너로 이어지는지
검증합니다. 회전 여부/목표각 판단은 전부 플래너가 스스로 계산합니다 — 시나리오는 코너가
있는 경로만 던져주면 됩니다.

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
    if (rotation_.done()) {                                     // |e_ψ|<ε_ψ && |r|<ε_r (2-3절)
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

`YawRotationProfile`은 `JerkLimitedSafetyStop`처럼 가속도 제한 패턴을 쓰지만, P-게인이 아니라
"지금 각속도에서 가속도 한계로 감속했을 때 정확히 목표에 도달하는 최대 각속도"를 매 틱
다시 계산하는 물리식입니다 — `JerkLimitedSafetyStop`의 `release_speed` 계산과 같은 방식입니다:

$$e_\psi = \operatorname{wrap}(\psi_{target} - \psi), \qquad
r_{feasible} = \sqrt{2 \cdot a_{max} \cdot |e_\psi|}, \qquad
r_{desired} = \operatorname{sign}(e_\psi) \cdot \min(r_{max},\, r_{feasible})$$

매 틱 $r_{cmd}$를 $r_{desired}$ 방향으로 가속도 한계($a_{max}$)만큼만 움직이고,
$|e_\psi| < \varepsilon_\psi \,\&\&\, |r_{cmd}| < \varepsilon_r$이 되면 `done()`이 true를 반환합니다.

수치: $r_{max}$(`yaw_rate_max`) 0.3 rad/s, $a_{max}$(`yaw_rate_accel_max`) 0.3 rad/s²,
$\varepsilon_\psi$(`yaw_tolerance_rad`) 0.02 rad, $\varepsilon_r$(`yaw_rate_tolerance`) 0.02 rad/s.

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

**실제로 필요한 변경은 딱 한 줄입니다** — `VALID_DRIVE_MODES`
([mode_transition.py:6](planar_velocity_sim/planar_velocity_sim/mode_transition.py#L6))가 지금
`(0, 1, 2, 3)`이라 모드 4(SpotTurn)를 요청하면
`mode_command_callback()`([planar_velocity_sim_node.py:107-111](planar_velocity_sim/planar_velocity_sim/planar_velocity_sim_node.py#L107-L111))에서
그냥 거부됩니다. `(0, 1, 2, 3, 4)`로 확장하고, `initial_drive_mode`/`command()`의 관련 검증
메시지("[0, 3]")만 같이 고치면 됩니다.

회전 완료를 누가 판정하는지도 이걸로 명확해집니다. ROTATING 구간에서는 `DriveModeState`의
`status` 필드가 애초에 움직이지 않습니다(이미 `STATUS_READY`로 고정된 채 유지) — 기다릴
차량 피드백 자체가 없으므로, 완료 판정은 처음부터 끝까지 플래너 내부 $e_\psi$/$r$ 기준(2-3절)만으로
이뤄지는 게 맞습니다.

---

## 5. 시나리오 phase 관리 — 크랩 검증 phase (요청 5번)

새로 만들 `spot_turn_course` 시나리오의 **마지막 phase 하나를 크랩(Left/Right) 모드로
정의**해서, 같은 시나리오 안에서 "급코너 = 제자리턴"과 "측면 이동 = 크랩"이라는 서로 다른 두
기동 방식이 나란히 검증되도록 합니다. phase 분할은 모드가 바뀌는 곳(1-5절)에서만 하므로,
급코너 3개는 전부 phase 0 하나의 경로 배열 안에 순서대로 들어갑니다:

```
phase 0: FORWARD  (40도 코너 → 제자리턴 → 60도 코너 → 제자리턴 → 70도 코너 → 제자리턴 → 목적지 근처까지, 전부 한 phase)
phase 1: LEFT      (크랩 모드로 옆 주차 등 마무리 — 기존 crab_switch 패턴 재사용, 모드가 바뀌므로 여기서만 phase 분할)
```

---

## 6. 전체 흐름 요약 (한 코너 기준)

```
[시나리오 매니저] phase(모드 하나) 경로 발행 -- 급코너가 여러 개 있어도 한 phase 전체 (1-5절)
      │
      ▼
[플래너] path_callback(): find_spot_turn_split_index()로 배열 안을 스캔 (1-2절)
      │  급코너 없음 ↓                     급코너 있음 → reference_path_ = 그 지점 앞까지만
      ▼                                          │       pending_post_turn_path_ = 나머지
정상 주행 (전체 경로 추종)                  [플래너] 잘린 경로를 정상 추종 → 끝에서 자연히 정지
                                                   ▼        (종단 정지 로직 그대로, 신규 코드 없음)
                                     [command_callback, 매 틱] spot_turn_feasible() 검사 (3절)
                                             │  가능 ↓            불가능 → 정지 유지, 매 틱 재검사 (3절)
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
                    maneuver_.on_mode_ready(current_yaw, mode_supervisor_)
                    → rotation_.engage(target_yaw_, current_yaw), state() == Rotating
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
| 급코너 탐지 + 경로 절단 (플래너 내부, C++) | `find_spot_turn_split_index()` / `apply_spot_turn_truncation_locked()` | 신규 — `path_callback()`이 배열 안을 직접 스캔 (1-2절) |
| Phase 분할 (시나리오) | `ScenarioPhase` | 구조체 변경 없음 — 단, 분할 기준이 모드 전환으로 바뀜(급코너는 더 이상 분할 이유 아님, 1-5절) |
| 경로 저작 검증 | `ScenarioPath.from_arrays()` | `curvature_limit`/`yaw_step_limit_deg` 검사 제거 (0절) — zero-length segment/tangent 검사는 유지 |
| 회전 트리거 | `command_callback()` 매 틱 재시도 | 신규 — 시나리오 관여 없음 (1-3절) |
| 회전 가능성 검사 | `circumscribed_radius` + `clearance_single_circle` | 완전 재사용 — `VehicleConfig` 코드 기준값(전장 3.0m/전폭 2.0m/margin 0.0) 그대로, `spot_turn_safety_margin`만 신규(0.0) (3절) |
| 모드 enum | `DriveMode::SpotTurn = 4` | 필드값 추가만 (기존 enum 재사용) |
| **바퀴 정렬 상태머신** | `DriveModeSupervisor` / `DriveModeControlState` | **완전 재사용, 무변경** — STOPPING/ALIGN이 전부 기존 상태 그대로 (2-2절) |
| 피드백 메시지 | `DriveModeState.msg` | `status`(`STATUS_ALIGNING`/`STATUS_READY`) 필드 병합은 제자리턴과 무관하게 이미 완료 — 제자리턴 자체는 이 메시지에 추가 변경 없음 |
| 명령 메시지 | `/vehicle/drive_mode_command` (`UInt8`) | **완전 재사용, 변경 없음** — target_yaw가 와이어를 안 탐 (2절) |
| **매뉴버 상위 상태** | `SpotTurnManeuverState` (신규, 플래너 내부 전용) | 신규이지만 값 3개짜리 작은 enum (2-3절) |
| 회전 프로파일 생성기 | `YawRotationProfile` (`JerkLimitedSafetyStop` 자매 클래스) | 신규 — 유일하게 진짜 새로 만드는 제어 로직 (2-3절) |
| 속도 명령 채널 | `publish_command()` / `geometry_msgs/Twist` | **완전 재사용** — 회전도 이 채널로 yaw_rate만 실어 보냄 |
| 차량 시뮬레이터 | `DriveModeTransitionModel` / `applied_velocity()` | **거의 완전 재사용** — `VALID_DRIVE_MODES`에 4 추가하는 것 외 무수정 (4절) |
| 시각화 | `vehicle_visualizer_node`/`debug_plot_node`의 `MODE_NAMES` | SPOT_TURN 모드명 인식만 추가됨 — `maneuver_.state()`(정렬 중/회전 중 구분) 표시는 미구현 |

---

## 8. 남은 작업

- `spot_turn_course` 시나리오(40도/60도/70도 코너 웨이포인트, 1-6절) 미작성.
- 시각화에 `maneuver_.state()`(`AligningWheels`/`Rotating` 구분) 표시 미구현 — 지금은 모드명만
  보임.
