# ReferencePath 곡률 필드 제거 및 내부 계산 방향

## 1. 배경

- `ReferencePath.msg`에서 `curvature` 필드를 더 이상 정의하지 않기로 결정했다.
- 현재는 퍼블리셔가 곡률을 계산해 메시지에 실어 보내고, `planner_node.cpp`는 그 값을
  그대로 받아 쓰고 있다.
  - 퍼블리셔: `track_map_provider_node.py:202`, `scenario_manager_node.py:362`
    (`data_message.curvature = local.kappa.tolist()`)
  - 구독: `planner_node.cpp:566` `path_callback()` 이 `msg->curvature` 크기를
    검증하고(`:569`), `ReferencePath` 생성자에 그대로 전달한다(`:579-580`).
  - 디버그 툴도 같은 필드를 소비한다: `debug_plot_node.py:373, 388`
    (`message.curvature`).
- 필드가 사라지면 **메시지를 소비하는 쪽에서 `x, y, yaw`만으로 곡률을 계산**해야 한다.

## 2. 건드리지 않는 부분

- `ReferencePath` 클래스(`core.hpp`/`core.cpp`)는 생성자 시그니처를 그대로 둔다.
  이 클래스는 planner 내부에서 생성하는 spatial candidate처럼 **정확한 analytic
  곡률**을 넘기는 다른 호출부도 있기 때문에, 클래스 자체를 차분 계산으로 바꾸면
  정밀도가 필요한 경로까지 근사치로 바뀐다.
- `validation/src/closed_loop_validator.cpp:180-183` (`make_reference()`)도
  authored `phase.kappa`를 그대로 쓰므로 변경 대상이 아니다.
- 즉, 변경 범위는 "ROS 메시지를 경계로 삼는 지점"으로 한정한다: `planner_node.cpp`의
  메시지 수신부, 그리고 같은 메시지를 구독하는 `debug_plot_node.py`.

## 2.5. 설계 전제 — "레퍼런스 경로 = 정확히 전제된 값" 기준과의 관계

- `x, y, yaw`는 여전히 메시지로 전달되는 그대로, 즉 정확히 전제된(given) 값으로
  유지된다. 이 기준을 어기는 건 `kappa` 하나뿐이다 — 필드가 사라지는 순간
  kappa는 "given"이 아니라 "x,y,yaw로부터 유도한 근사값"으로 성격이 바뀐다.
  이건 메시지 스키마 결정 자체가 만든 트레이드오프이며, 계산 방식을 어떻게
  짜도 완전히는 피할 수 없다.
- 이 근사가 장식용이 아니라는 점이 중요하다. 레퍼런스 kappa는
  `initial_spatial_boundary()`(`core.cpp:447-476`)에서 lateral 7차 다항식의
  경계조건을 결정하고, `offset_path_curvature()`(`core.cpp:435-445`, 호출부
  `:499`, `:503`)를 거쳐 후보 경로의 실제 curvature로 변환되어
  `curvature_max` 하드 체크와 [CURVATURE_LOCAL_REGENERATION_FLOW_KR.md](CURVATURE_LOCAL_REGENERATION_FLOW_KR.md)의
  재생성 트리거까지 그대로 전파된다.
- 따라서 목표는 "완전히 exact"가 아니라, **코드베이스가 이미 신뢰하고 쓰는
  근사 정밀도(=`gradient()`의 3점 스텐실 수준)를 그대로 맞추는 것**이다. 아래
  3절의 "여분 waypoint 1개" 설계가 그 기준이다.

## 3. 계산 방법 — 필요 구간 + forward 여분 1점

전진차분 `kappa_i = (psi[i+1]-psi[i]) / (s[i+1]-s[i])`는 `i`의 다음 인덱스가
있어야 계산할 수 있다. 실사용 구간의 마지막 점(N)에서 이게 막히는데, 이걸
"직전 구간 값을 복사"로 때우면 그 지점의 kappa가 실제보다 부정확해진다.

대신 로컬 슬라이스를 만들 때 실사용 범위보다 forward 방향으로 **여분 1점을
더 확보**한 뒤, 계산이 끝나면 그 여분 점을 버린다.

```text
필요한 실사용 범위:      P0 ... P99 P100
곡률 계산용(여분 1점 포함): P0 ... P99 P100 P101
                                         ^^^^ 계산 후 버림
```

```text
psi_unwrapped = unwrap(yaw)              # 기존 ReferencePath 생성자와 동일하게 unwrap 먼저
s             = cumulative_arc_length(x, y)   # 여분 점까지 포함해서 계산

for i in 0 .. n-2:                       # n = 여분 점 포함 개수 (=실사용 개수 + 1)
    kappa[i] = (psi_unwrapped[i+1] - psi_unwrapped[i]) / max(s[i+1] - s[i], eps)

# 여기서 kappa 배열 전체(길이 n)에 대해 gradient(kappa, s, true)로 kappa_s까지
# 계산한 다음, 마지막 한 점(P101에 해당하는 kappa/kappa_s)만 버리고
# 나머지 n-1개(P0..P100)를 최종 ReferencePath에 사용한다.
```

- `eps`는 기존 코드 관례(`1.0e-15`)를 그대로 맞춘다.
- yaw를 먼저 unwrap하지 않으면 ±π 근방에서 차분값이 튄다 — `unwrap_angles()`가
  이미 `core.hpp:722`에 공개 선언되어 있으니 그대로 재사용한다.

### 3.1. kappa_s 끝점 처리 — 여분 점이 몇 개 더 필요한가

`gradient()`(`core.cpp:68-106`)는 `edge_order_two=true`일 때 배열 양 끝에서
**뒤(앞)쪽으로만 보는 3점 one-sided 공식**을 쓴다 (끝점은 `n-3,n-2,n-1` 세 점만
사용, 배열 밖 데이터를 요구하지 않는다).

여분 점을 1개만 확보해서 `kappa` 배열을 실사용 범위+1(즉 P0..P101, 길이
n+1)까지 채워두면, 우리가 실제로 쓰는 마지막 점 P100은 그 배열에서
"끝에서 두 번째" 위치가 된다. `gradient()`를 이 확장 배열 전체에 대해
돌리면, P100의 kappa_s는 `kappa[N-2], kappa[N-1], kappa[N]` 세 점을 쓰는
backward 3점 공식으로 계산되는데 — 이건 **기존 코드가 지금도 모든 경로
끝점에서 쓰고 있는 것과 정확히 같은 처리**다 (`kappa_s_ = gradient(kappa_,
s_, true)`가 항상 이렇게 동작해왔다). 즉 새로 추가한 근사 때문에 kappa_s
정밀도가 떨어지는 게 아니라, 딱 기존 표준만큼만 유지된다.

- **여분 1점으로 충분하다.** kappa와 kappa_s 모두 "기존과 동일한 근사
  수준"을 얻는 데 여분 점 2개는 필요 없다.
- 여분 2점(P101, P102)까지 확보하면 P100의 kappa_s를 중앙차분(central,
  `kappa[N-1],kappa[N],kappa[N+1]`)으로 업그레이드할 수 있지만, 이건 기존
  표준을 넘어서는 선택적 개선이지 지금 필요한 요구사항은 아니다. 우선순위는
  낮게 둔다.

### 3.2. 진짜 global reference 종점 예외

로컬 슬라이스가 실제 global reference의 마지막 점까지 도달한 경우엔 여분
점(P101) 자체가 존재하지 않는다. 이 마지막 점은 항상 해당 phase의 정지
목표점(`stop_s`, `scenario_manager_node.py:220-226`의 `phase_stop_s()` /
`execution_path_for_phase()`가 만드는 `ScenarioPath.clipped()` 종점)이고,
거기 도달하는 궤적은 terminal 조건상 속도가 0으로 수렴하도록 설계된다.
`heading_rate = speed * kappa`이므로 그 지점에서는 kappa 값 자체가 동역학에
영향을 주지 않는다 — **경로 지오메트리가 항상 직선으로 끝난다는 보장이
있어서가 아니라, 그 지점에서 speed가 0으로 가기 때문에 kappa 값이 무의미해
지는 것**이다. (실제로 `curved_gate_maze.csv`처럼 마지막 행도 kappa≈-0.023로
안 끝나는 authored 경로가 있어, "지오메트리가 항상 0"이라는 가정은 성립하지
않는다.)

따라서 후진차분을 계산하는 대신 고정값을 쓴다.

```text
kappa[N] = 0.0   # P_{N+1}이 없는 진짜 종점(=phase 정지 목표점)에서만 적용
```

kappa_s 쪽도 특별 처리가 필요 없다 — `gradient()`의 끝점 공식은 배열의 마지막
3점(`N-2,N-1,N`)만 보고 동작하므로, kappa[N]=0으로 두면 그 값을 그대로 반영해
자연스럽게 kappa_s를 계산한다.

주의할 점은 이게 "마지막 노드 하나"에만 해당한다는 것이다. 그 앞 노드들
(N-1 이하)은 로컬 슬라이스 안에 실제 다음 점이 있으므로 3절의 전진차분을
그대로 쓴다 — 정지 목표점에 접근하는 마지막 구간 전체를 곡률 0으로
뭉개는 게 아니다. 또한 `curvature_ok` 하드 체크(`core.cpp:1893`)는 speed와
무관하게 `|kappa| <= curvature_max`만 보므로, 이 근사는 딱 그 마지막 한
샘플에서만 실제 지오메트리와 다를 수 있다는 점은 남는다 (지금 확인한
map들에서는 어차피 `curvature_max=0.20`보다 한참 작은 값들이라 문제되지
않는다).

구분 기준: 로컬 슬라이스를 만드는 쪽이 "여분 1점을 실제로 확보했는지"를
안다 (인덱스가 배열 범위를 벗어나면 확보 실패 = 진짜 종점). 이 판단은
슬라이싱을 수행하는 코드 안에서 자연히 나오므로, 메시지에 별도 플래그를
추가할 필요는 없다.

**구현 시 정리된 방식 (완료):** 위 구분을 컨슈머(`planner_node.cpp`,
`debug_plot_node.py`)로 넘기려면 "여분 점을 실제로 받았는지"를 메시지만
보고 판단해야 하는데, 퍼블리셔와 컨슈머가 서로 다른 프로세스/언어라 이
신호를 메시지 스키마 변경 없이 넘길 방법이 마땅치 않았다. 그래서 실제
구현은 **퍼블리셔가 항상 여분 1점을 채워 보낸다**로 통일했다 — 실제 다음
점이 있으면 그 점을, 없으면(진짜 종점) 마지막 점의 heading을 따라 직선으로
한 걸음 연장한 synthetic 점을 만들어 넣는다. Synthetic 점은 yaw가 이전
점과 완전히 같으므로 전진차분값이 정확히 0이 되어, 위에서 도출한
`kappa[N]=0`과 수학적으로 동일한 결과를 자동으로 만든다. 결과적으로
컨슈머 쪽에는 분기 자체가 없다 — 받은 배열 전체에 3절의 전진차분을
그대로 적용하고, 마지막 1점만 잘라내면 끝이다. 메시지에 새 필드도
필요 없었다.
- `scenario_path.py`의 `local_slice()` open-loop 분기(`:281-316` 부근):
  실제 다음 점이 있으면(`stop < len(self.x)`) 그 점을 포함해서 반환하고,
  없으면 `math.cos/sin(yaw)`로 한 걸음 연장한 synthetic 점을 붙인다.
- `scenario_path.py`의 closed-loop 분기, `local_reference.py`의
  `local_reference_indices()`: 트랙 자체가 순환하므로 "진짜 종점"이 없다
  — 항상 다음 인덱스를 modulo로 1개 더 붙이면 끝이라 synthetic 처리가
  아예 필요 없다.
- `planner_node.cpp`의 `estimate_curvature_from_yaw()`: 입력 n개를 받아
  n-1개의 전진차분 kappa를 무조건 반환한다. 분기 없음.

## 4. 적용 위치

여분 1점을 "확보 → forward 차분 → 버림" 하는 책임은 **로컬 슬라이스를 최종
확정하는 지점**에 있다. 오늘 기준으로는 그게 퍼블리셔(`scenario_manager_node.py`,
`track_map_provider_node.py`)의 슬라이싱 함수이고, 회의록에서 논의된 "상위는
충분히/전체를 주고 쓰는 쪽에서 자른다" 방향으로 가면 이 책임이
`planner_node.cpp`의 수신부로 옮겨간다. 어느 쪽이든 로직은 동일하다: **슬라이스를
결정하는 코드가 인덱스 범위 안에서 다음 점이 있는지 이미 알고 있으므로**, 거기서
자연스럽게 여분 1점 확보 여부(=진짜 종점인지 아닌지)를 판단할 수 있다.

오늘 코드 구조 기준의 구체적인 적용 지점:

1. 퍼블리셔 쪽 슬라이싱을 ahead 방향으로 1점 더 넉넉하게 잡는다.
   - `scenario_path.py:218-276` `ScenarioPath.local_slice()`: open-loop 분기의
     `stop`(:257-259) 계산을 1점 더 포함하도록 조정하되, `stop`이 이미
     `len(self.x)`(진짜 종점)에 닿아 있으면 더 늘리지 않는다 — 이게 3.2절의
     "진짜 종점 판단"이 자연히 일어나는 지점이다.
   - `track_map_provider_node.py`의 `build_local_reference_slice(...)`
     호출부(`:188-193`)도 동일하게 ahead 방향 여유 1점을 반영한다.
2. `planner_node.cpp` 쪽:
   - 로컬 헬퍼 구역(현재 `cumulative_arc_length`가 있는 `:77` 부근)에
     `estimate_curvature_from_yaw(psi, s)` 같은 작은 함수를 추가한다. 3절의
     전진차분 공식을 그대로 구현하되, 마지막 인덱스는 "여분 점이 있으면
     계산 후 버림, 없으면(=진짜 종점=phase 정지 목표점) `kappa=0.0` 고정"을
     판단해서 처리한다.
   - `path_callback()`(`:566`) 수정:
     - 크기 검증(`:568-570`)에서 `msg->curvature` 관련 조건을 제거한다.
     - `const auto s = cumulative_arc_length(msg->x, msg->y);` 이후,
       `unwrap_angles(msg->yaw)`로 unwrap한 값과 `s`를 넘겨 `kappa`를 계산한다.
     - 여분 점을 실제로 받았다면(퍼블리셔가 +1을 채워 보낸 경우) `x, y, yaw, s,
       kappa`에서 마지막 한 점을 잘라낸 뒤, `ReferencePath` 생성자(`:579-580`)에
       `msg->curvature` 대신 이 잘라낸 `kappa`를 전달한다.
   - `ReferencePath` 클래스 내부는 그대로 둔다 — `kappa_s_ = gradient(kappa_,
     s_, true)`가 이미 3.1절에서 확인한 것과 동일한 끝점 처리를 자동으로
     해준다.

## 5. Python/tools 쪽 적용

- `track_map_provider_node.py`, `scenario_manager_node.py`: `data_message.curvature
  = local.kappa.tolist()` 줄을 삭제한다. `local.kappa` 자체(내부 트랙 계산용)는
  다른 용도로 쓰이면 그대로 둬도 무방하다. 대신 4절 1번의 ahead 여유 1점 확보는
  유지한다 — 메시지에는 여전히 x, y, yaw가 여분 1점만큼 더 실려서 나간다.
- `debug_plot_node.py`: `message.curvature`를 더 이상 받을 수 없으므로,
  `reference_callback`(`:368-380`)과 `selected_data_callback`(`:383-393`)에서
  `message.x, message.y, message.yaw`로부터 3절과 동일한 로직(여분 1점 처리
  포함, numpy 버전)으로 `reference_kappa` / `OpenPathGeometry.from_arrays` 입력을
  직접 만든다. C++ 쪽과 같은 공식·같은 끝점 규칙을 써야 플롯이 planner 내부
  값과 어긋나 보이지 않는다.

## 6. 체크리스트

- [x] `ReferencePath.msg`에서 `float64[] curvature` 삭제
- [x] `scenario_path.py` `local_slice()` / `local_reference.py`
      `local_reference_indices()`: ahead 방향 여분 1점을 항상 포함하도록
      수정 (진짜 종점에서는 synthetic 점으로 대체, closed-loop은 modulo로
      항상 real)
- [x] `planner_node.cpp`: 크기 검증에서 `curvature` 조건 제거 (최소 길이도
      4→5로 조정, 여분 1점만큼)
- [x] `planner_node.cpp`: `estimate_curvature_from_yaw` 헬퍼 추가 — 분기
      없이 입력 n개 → 전진차분 n-1개
- [x] `planner_node.cpp`: `path_callback`에서 여분 점을 잘라낸 `kappa`를
      `ReferencePath` 생성자에 전달
- [x] `track_map_provider_node.py` / `scenario_manager_node.py`: `curvature`
      대입 코드만 제거 (슬라이싱 여유는 `local_slice`/`local_reference`
      쪽에서 유지)
- [x] `debug_plot_node.py`: `reference_callback`(여분 1점 처리 포함),
      `selected_data_callback`(별도 padding 없는 dense trajectory이므로
      단순 전진차분 + 마지막 값 복제)에서 곡률을 직접 계산하도록 수정
- [x] `planner_node.cpp`의 outgoing `trajectory_data_pub_`(선택된 궤적
      디버그 토픽)도 같은 `ReferencePathMsg` 타입을 재사용하고 있어서
      `curvature` 필드 제거의 영향을 받는다는 걸 뒤늦게 발견 — 거기서
      `motion.kappa`를 싣던 라인도 같이 제거 (2절에서 "건드리지 않는다"고
      했던 analytic 곡률 소스이지만, 메시지 타입을 공유해서 어쩔 수 없이
      영향받음; 순수 디버그 시각화 용도라 손실 허용)
- [x] 빌드 및 `test_core.cpp` 등 기존 테스트 통과 확인 (colcon build
      simp_planner_msgs/simp_planner_cpp 성공, `test_simp_planner_core`
      통과, `simp_planner_tools` pytest 41 passed — 나머지 2개 실패는
      `test_planning_call_count_report.py`로 이번 변경과 무관한 기존
      build 아티팩트 문제임을 stash 비교로 확인)
- [ ] 근사 곡률이 기존 `msg->curvature` 기반 결과와 시뮬레이션상 큰 차이가
      없는지 비교 (특히 `curvature_max` 근처 후보의 재생성 트리거 여부) —
      실차/시뮬레이션 재생 필요, 코드 리뷰만으론 확인 불가

## 7. 한계

- `x, y, yaw`는 여전히 정확히 전제된 값이지만 `kappa`는 유도값으로 바뀐다 —
  2.5절에서 정리했듯 이건 메시지 스키마 결정 자체의 트레이드오프이며, 이
  문서의 설계(여분 1점)는 그 근사 정밀도를 "기존에 kappa_s가 이미 쓰던
  수준"까지 최대한 끌어올리는 것이 목표이지, 정밀도 손실 자체를 없애지는
  못한다.
- 전진차분은 포인트 간격이 불균일하거나 노이즈가 있으면 부정확할 수 있다.
  이건 단순 참고용 근사가 아니다 — 2.5절에서 확인했듯 `fr.kappa`,
  `fr.kappa_s`는 `initial_spatial_boundary()`(`core.cpp:447-476`)에서 7차
  lateral 다항식의 초기 경계조건(`n1`, `n2`, `desired_kappa_l`) 자체를
  결정하는 입력값이다. 즉 오차가 있으면 후보 경로의 **시작 형상 자체**가
  왜곡될 수 있고, `curvature_max` 체크나 cost는 그 다음 단계에서 별도로
  일어나는 것뿐이다. 그래서 3절/3.1절에서 근사 정밀도를 "대충 2점 차분"이
  아니라 `gradient()`가 이미 코드베이스 전체에서 신뢰하고 쓰는 3점 스텐실
  수준까지 맞추도록 설계한 것이고, 그 이상으로 정밀도를 낮출 이유는 없다.
- 진짜 global reference 종점(3.2절)에서만 `kappa=0.0`을 고정값으로 쓴다 —
  그 지점은 항상 phase 정지 목표점이라 speed가 0으로 수렴하므로
  `heading_rate = speed * kappa` 관점에서 영향이 없지만, `curvature_max`
  하드 체크처럼 speed와 무관하게 kappa 크기만 보는 로직에서는 그 한 샘플만
  실제 지오메트리와 다를 수 있다는 점은 남는다. 그 외 구간에서는 "직전 값
  복사" 같은 임의 근사가 없다.
- 계산 비용은 O(n)이며 매 replan 사이클(메시지 수신 시)마다 재계산되지만,
  기존에도 `cumulative_arc_length`를 매번 재계산하고 있어 추가 부담은 미미하다.
