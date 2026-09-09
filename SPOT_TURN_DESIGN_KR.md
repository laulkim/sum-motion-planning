# 제자리턴 시나리오/기능 설계 초안

> 1차 정리 문서입니다. "확정 스펙"이 아니라 구현 착수 전 합의를 위한 초안이며,
> 맨 아래 [8. 열린 질문](#8-열린-질문-구현-착수-전-결정-필요) 항목들은 아직 결정되지 않았습니다.

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

(사용자 요청 1, 4번 — **개정**: 제자리턴 판단/목표각 계산 주체를 시나리오에서 플래너 내부로
이동. 아래 지적을 반영함: *"다른 모드들은 phase마다 정의하는 게 맞는데, 4번(제자리턴)은
플래너 내부에서 하라는 것. 경로도 알아서 잘라야 하고, 이 경로가 제자리턴이 필요한지 확인하는
것도 내부 함수에서. 다른 모드는 요청모드/현재모드로 정하지만, 여기는 그 지점에 도달하면
플래너가 스스로 요청(4번, target_yaw)을 보내는 것"*)

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
저장해 둡니다.

### 1-3. 트리거 — "요청 vs 현재 비교"가 아니라 "그 지점 도달"

다른 모드는 `requested_mode_ != current_mode_`가 되는 순간(외부에서 새 요청이 들어온 순간)
트리거됩니다. 제자리턴은 다릅니다 — **차량이 `detect_spot_turn_point()`가 찾아낸 지점(s)에
도달하는 순간**이 트리거입니다. 그 순간 플래너가 (외부 요청을 기다리지 않고) 스스로:

```cpp
mode_supervisor_.set_requested_mode(DriveMode::SpotTurn, spot_turn_point->target_yaw);
```

를 호출해서, 마치 외부에서 모드 4를 요청받은 것처럼 만듭니다. 이후 진행/완료 판정은
`DriveModeSupervisor`가 기존 로직 그대로 처리합니다 (2절).

### 1-4. 완료 후 원래 요청 모드로 복귀 (중요한 함정)

`requested_mode_`는 지금까지 "외부에서 마지막으로 요청받은 값" 하나만 담는 변수였는데,
제자리턴은 플래너가 **일시적으로 그 값을 가로채서** 4로 덮어쓰는 셈입니다. 그래서 "외부에서
실제로 요청한 모드"(예: `FORWARD`)를 별도 변수(`external_requested_mode_`)로 따로 기억해 두고,
제자리턴이 `ready()`가 되는 순간 플래너가 다시 그 값으로 `set_requested_mode()`를 호출해서
되돌려놔야 합니다 — 안 그러면 회전만 끝나고 원래 모드로 못 돌아옵니다. (8절 열린 질문에 반영)

### 1-5. Phase 분할 자체는 시나리오에 여전히 필요함 — 단, 제자리턴 의미는 없음

`ScenarioPath.from_arrays()`가 여전히 `yaw_step_limit_deg`(15도)/`curvature_limit`을 강제
검사하기 때문에([scenario_path.py:96-98](simp_planner_tools/simp_planner_tools/scenario_path.py#L96-L98)),
40도/60도/70도급 급코너를 가진 원본 웨이포인트는 시나리오 단계에서 애초에 하나의 연속
`ScenarioPath`로 만들 수 없습니다. 그래서 **phase 분할 자체는 여전히 필요**합니다 — 다만 이번
개정으로 그 분할점에는 **제자리턴 관련 필드가 전혀 붙지 않습니다**. `ScenarioPhase`는 원래
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

## 2. 상태 전이 설계 — 차량 ↔ 플래너

(사용자 요청 2, 6, 7번 — **개정**: 새 메시지 쌍을 따로 만들지 않고, 기존 `DriveMode`
enum/메시지를 최대한 그대로 재사용하는 방향으로 바꿈)

### 2-1. 방향 전환: 새 상태 머신을 병렬로 만들지 않는다

이전 초안에서는 `SpotTurnCommand`/`SpotTurnState`라는 완전히 새로운 메시지 쌍과
`SpotTurnSupervisor`라는 별도 클래스를 제안했는데, 기존 메시지를 최대한 유지하는 쪽으로
방향을 바꿉니다. 근거: 제자리턴도 "요청 → 차량이 정지 상태에서만 접수 → 진행 중 → 완료 확인
→ 재개"라는 흐름 자체는 기존 모드 전환(Forward/Reverse/Left/Right)과 완전히 동일합니다.
그렇다면 **제자리턴을 `DriveMode`의 5번째 값으로 추가**하고, 기존 `DriveModeSupervisor` /
`DriveModeState.msg` 상태 머신을 그대로 재사용하는 게 훨씬 적은 변경으로 같은 결과를 냅니다.

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

### 2-2. 무엇이 그대로 유지되고, 무엇 하나만 추가되는가

| 요소 | 상태 |
|---|---|
| `DriveModeState.msg` (차량→플래너 피드백: `current_mode`, `requested_mode`, `transition_in_progress`, `transition_complete`) | **완전히 그대로.** 필드가 전부 `uint8`/`bool`이라 값 4(SpotTurn)도 스키마 변경 없이 그냥 들어감 |
| `DriveModeSupervisor` (`ready()`, `state()`, `should_publish_command()`) | **완전히 그대로.** "모드 일치 && 전환 안 하는 중 && 완료" 판정 로직 자체는 모드가 뭐든 안 바뀜 |
| `/vehicle/drive_mode_command` 토픽 이름 | **그대로.** payload 타입만 바뀜 (아래) |
| `/vehicle/drive_mode_command`의 wire 타입 | `std_msgs/UInt8` → 새 최소 메시지 `DriveModeCommand.msg`로 교체 (필드 1개 추가 위해 불가피) |

즉 실제로 새로 만드는 건 **필드 하나(목표 yaw)와 그걸 담을 그릇(현재 UInt8이던 명령 메시지)
뿐**이고, 상태 판정 로직·피드백 메시지·상태 관리 클래스는 전부 기존 것을 그대로 씁니다.

### 2-3. 목표 yaw 필드 — 이름과 위치

`/vehicle/drive_mode_command`가 지금은 순수 모드 값 하나(`UInt8`)만 나르기 때문에, 목표 yaw를
같이 보내려면 이 토픽의 타입을 아주 작은 메시지로 바꿔야 합니다:

**`DriveModeCommand.msg`** (신규, `/vehicle/drive_mode_command`의 새 타입)
```
uint8 mode                      # 기존과 동일 (DriveMode enum 값, 0~4)
float64 spot_turn_target_yaw    # mode==SpotTurn(4)일 때만 유효. 그 외에는 NaN
```

필드 이름을 `target_yaw`가 아니라 **`spot_turn_target_yaw`**로 지은 이유가 요청하신 부분입니다
— 이 필드가 "제자리턴 전용" 값이고 나머지 4개 모드에서는 의미 없다는 걸 필드 이름만 보고도
알 수 있어야 하기 때문입니다.

### 2-4. 완료 시 초기화 — NaN을 값 없음 센티널로, 내부는 `std::optional`로 감싼다

"0으로 초기화 vs NaN으로 초기화" 중 **NaN을 권장**합니다. 이유: `0.0 rad`은 그 자체로 유효한
실제 목표 각도(정북 방향)라서, 0을 "값 없음"으로 쓰면 "목표가 0도로 설정됨"과 "목표 없음"을
구분할 수 없습니다. NaN은 절대 유효한 각도 값이 될 수 없으니 센티널로 안전합니다.

모듈화 방식은 `DriveModeSupervisor`가 이미 쓰고 있는 패턴을 그대로 따릅니다 — 클래스
내부에서는 `std::optional<double>`로 타입 안전하게 들고 있다가, **메시지 송수신 경계에서만**
NaN으로 변환합니다 (`requested_mode_`가 이미 `std::optional<DriveMode>`인 것과 동일한 패턴):

```cpp
class DriveModeSupervisor {
 public:
  // mode != SpotTurn일 땐 spot_turn_target_yaw를 생략(nullopt)해야 하고,
  // mode == SpotTurn일 땐 반드시 값을 줘야 한다 -- 이 불변조건을 여기 한 곳에서만 검증한다.
  bool set_requested_mode(DriveMode mode,
                           std::optional<double> spot_turn_target_yaw = std::nullopt);

  std::optional<double> spot_turn_target_yaw() const { return spot_turn_target_yaw_; }

 private:
  std::optional<DriveMode> requested_mode_;
  std::optional<double> spot_turn_target_yaw_;   // ← 추가된 상태 한 개
};

// 메시지 송신 시점에만 optional -> NaN 변환 (반대 방향은 수신 시점에 NaN -> optional)
double to_wire_yaw(std::optional<double> value) {
  return value ? *value : std::numeric_limits<double>::quiet_NaN();
}
```

**초기화(리셋) 시점**: 플래너가 제자리턴 완료 후 정상 모드(예: Forward)로 복귀하려고
`set_requested_mode(DriveMode::Forward)`를 다시 호출하는 그 순간, 두 번째 인자를 생략하면
기본값 `std::nullopt`가 그대로 `spot_turn_target_yaw_`에 덮어써집니다. 즉 **"완료되면 초기화"를
위한 별도 코드가 필요 없이, 다음 모드를 요청하는 동일한 함수 호출에 자연스럽게 포함**됩니다 —
이게 "모듈화"의 핵심입니다: 불변조건 검증과 리셋이 전부 `set_requested_mode()` 한 곳에만
있고, `planner_node.cpp` 쪽 호출부는 그냥 평소처럼 모드만 넘기면 됩니다.

### 2-5. Phase 전환 흐름에 어떻게 끼워 넣나

→ 1-1~1-4절로 옮겨서 정리했습니다: 트리거 주체는 **플래너 자신**이고(시나리오는 관여 안 함),
"그 지점 도달"이 트리거이며, 완료 후 원래 요청 모드로 복귀하는 부분까지 확정된 설계입니다.

---

## 3. 회전 가능성 사전 검사 (요청 3번)

제자리턴을 시작하기 전에, 그 자리에서 회전해도 충돌이 없는지 확인하는 **1단계 게이트**를
추가합니다. 이미 있는 유틸리티를 그대로 재사용하면 됩니다 — 새로 만들 게 거의 없습니다:

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

지금 `DriveModeTransitionModel`([mode_transition.py](planar_velocity_sim/planar_velocity_sim/mode_transition.py))은
**실제로 차량을 회전시키지 않습니다** — 그냥 `transition_duration_sec`(기본 2초) 동안
Vx=Vy=YawRate=0을 유지하다가, 시간이 다 되면 `current_mode` 값만 순간적으로 바뀝니다 (몸체
yaw는 안 변함). 이건 크랩 조향이라 모드 전환 자체가 조향 기하학만 바꾸고 물리적 회전이 필요
없기 때문에 지금까지 문제가 안 됐던 겁니다.

제자리턴은 이름 그대로 **몸체 yaw 자체가 실제로 돌아야** 하므로, 시간만 세는 방식이 아니라
진짜 각속도를 적분해서 yaw를 갱신하는 새 모델이 필요합니다.

### 4-2. 새 `SpotTurnRotationModel` (안)

```python
class SpotTurnRotationModel:
    """Vx=Vy=0, yaw_rate만으로 목표 yaw까지 실제로 회전시킨다."""

    def engage(self, target_yaw: float, current_yaw: float) -> None: ...
    def update(self, dt: float) -> None:
        # 목표까지 남은 각도 부호에 따라 yaw_rate 방향 결정
        # 목표 근처에서는 감속(저크 제한) -- JerkLimitedSafetyStop과 같은 패턴
        ...
    def applied_velocity(self) -> tuple[float, float, float]:
        return 0.0, 0.0, self.current_yaw_rate   # (vx, vy, yaw_rate)
    def rotating(self) -> bool: ...
    def current_yaw(self) -> float: ...
```

목표 근처에서 목표 각속도를 0으로 부드럽게 줄이는 부분은, 플래너 쪽에 이미 있는
`JerkLimitedSafetyStop`([runtime.hpp:128-152](simp_planner/simp_planner_cpp/include/simp_planner/runtime.hpp#L128-L152))의
"저크 제한 감속" 패턴을 각속도 축에 그대로 옮겨 쓰면 됩니다 (직선 감속 ↔ 각속도 감속, 구조가
동일함).

`planar_velocity_sim_node.py`는 (기존과 동일한) `/vehicle/drive_mode_command`를 계속 구독하되,
수신한 `DriveModeCommand.mode == SpotTurn`이면 이 모델을 갱신하고, `applied_velocity()`가
반환하는 `(0, 0, yaw_rate)`를 매 스텝 실제 body_yaw 적분에 반영합니다 (`integrate_body_velocity`,
[kinematics.py](planar_velocity_sim/planar_velocity_sim/kinematics.py) 재사용). 새 토픽은
필요 없습니다 — 명령이 이미 같은 메시지 안에 다 들어있습니다 (2-3절).

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
[phase N 주행 중]
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
                                     [플래너] 회전 가능성 검사 (원 충돌 체크, 3절)
                                             │  가능 ↓            불가능 → (8절 열린 질문)
                                             ▼
      external_requested_mode_ = 기존 요청값 저장 (1-4절)
      set_requested_mode(SpotTurn, target_yaw) 호출 (1-3절)
                                             ▼
[플래너 → 차량] DriveModeCommand{mode=SpotTurn, spot_turn_target_yaw=target}
      ▼
[차량] SpotTurnRotationModel로 실제 yaw_rate 적분 회전
       DriveModeState{current_mode=SpotTurn, transition_in_progress=true} 피드백
      ▼
[플래너] mode_supervisor_.ready() == true 될 때까지 대기 (정지 명령만 발행)
      ▼
[차량] 목표 yaw 도달 → transition_complete=true, transition_in_progress=false
      ▼
[플래너] set_requested_mode(external_requested_mode_) 로 원래 모드 복귀 (1-4절)
         (spot_turn_target_yaw_는 이 호출로 자동 초기화, 2-4절 참고)
      ▼
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
| 피드백 메시지 | `DriveModeState.msg` | **완전 재사용, 변경 없음** |
| 명령 메시지 | `DriveModeCommand.msg` (기존 `UInt8` 대체) | `spot_turn_target_yaw` 필드 1개만 신규 추가 |
| 플래너 상태 관리 | `DriveModeSupervisor` | **거의 재사용** — `spot_turn_target_yaw_`, `external_requested_mode_` 상태 2개 추가 (1-4, 2-4절) |
| 차량 회전 모델 | `SpotTurnRotationModel` | 신규 (JerkLimitedSafetyStop 패턴 재사용) — 유일하게 진짜 새로 만드는 로직 |
| 시각화 | 기존 `vehicle_visualizer_node`의 상태 텍스트 마커 | 확장 (제자리턴 상태 줄 추가) |

---

## 8. 열린 질문 (구현 착수 전 결정 필요)

1. **급코너 자동 탐지 임계값**: `jump_threshold_deg`를 몇 도로 할지 (30도 제안, 확정 아님).
2. ~~회전 트리거 주체: 플래너 자체 판단 vs 시나리오 명시 지시~~ → **이번 요청으로 결정됨**:
   플래너가 `detect_spot_turn_point()`로 스스로 판단, 시나리오는 관여 안 함 (1-1~1-3절).
3. **회전 불가능(충돌) 시 동작**: 그냥 그 자리에서 계속 대기(재시도)할지, 에러 상태로 빠질지,
   장애물이 사라질 때까지 무한 대기할지.
4. **회전 각속도/저크 한계값**: `spot_turn_yaw_rate_max`, `spot_turn_yaw_jerk_max` 같은 파라미터의
   구체적 수치.
5. **회전 안전 여유 반지름**: 3절의 `spot_turn_safety_margin` 구체적 수치("넓게"의 정량화).
6. **`DriveModeState.msg`의 `transition_in_progress`/`transition_complete`를 필드 1개로
   합칠지**: 이번 개정으로 이 메시지는 안 바꾸기로 했지만(2-2절), 두 필드가 실질적으로 2가지
   조합만 나온다는 사실 자체는 여전히 유효함. 합치면 `DriveModeSupervisor::ready()`,
   `DriveModeTransitionModel`, `debug_plot_node.py`의 자체 구독,
   `vehicle_visualizer_node.py`의 상태 텍스트까지 전부 같이 고쳐야 해서 마이그레이션 범위가
   있음 — 이번 작업과 묶을지, 별도 작업으로 미룰지.
7. ~~`SpotTurnCommand`를 새 메시지로 만들지, `DriveMode` enum 5번째 값으로 합칠지~~ →
   **이번 요청으로 결정됨**: `DriveMode::SpotTurn = 4`로 합침 (2-1절).
8. **`/vehicle/drive_mode_command`를 `UInt8` → `DriveModeCommand.msg`로 바꾸면서 생기는 파급
   범위**: 지금 이 토픽을 `UInt8`로 구독하는 쪽(`planar_velocity_sim_node.py`의
   `mode_command_sub_`, `vehicle_visualizer_node.py`의 새 상태 텍스트 마커 구독)을 전부 새
   메시지 타입으로 같이 고쳐야 함. 한 번에 다 바꿀지, 과도기적으로 `UInt8`는 유지하고 목표
   yaw만 별도 토픽(`/vehicle/spot_turn_target_yaw`, `Float64`)으로 따로 보낼지 — 후자는 메시지
   교체가 전혀 없는 대신 두 토픽 간 원자성이 깨짐(순간적으로 모드와 yaw가 서로 다른 시점 값일
   수 있음).
9. **`external_requested_mode_` 복귀 시점의 예외 상황**: 1-4절에서, 제자리턴 도중에 시나리오가
   `/requested_drive_mode`로 또 다른 모드를 새로 요청하면(예: phase가 더 진행돼서 Left로
   바뀜) 어떻게 할지 — 저장해둔 `external_requested_mode_`를 그 새 값으로 덮어써서 회전 완료
   후 그쪽으로 복귀할지, 아니면 제자리턴을 먼저 끝내는 동안은 새 요청을 무시할지.
