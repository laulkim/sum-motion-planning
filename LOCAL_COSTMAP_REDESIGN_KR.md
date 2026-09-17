# Local Costmap 5Hz 전환 설계안

## 배경 / 목적

현재 `scenario_manager_node`는 시나리오 전체 경로를 커버하는 코스트맵을 시작 시점에 1회만(latched QoS) 발행한다. 이를 실제 로봇의 로컬 코스트맵처럼 **차량 기준 5Hz 주기 발행**으로 바꾸는 게 목표다.

이번 전환의 부차적인 목적은, 5Hz 코스트맵 캡처 시점과 매 planning 사이클의 조회 시점 사이에 발생하는 **좌표 정합(coordinate registration) 연산**을 명시적으로 만들어서, 그 연산 비용을 디버깅/계측할 수 있게 하는 것이다.

> 중간에 "회전 없는 축정렬 방식"으로 단순화했다가, 최종적으로 다시 **회전 포함(차량 Yaw 기준 좌표축)** 방식으로 확정했다. 아래는 최종 확정 버전이다.

## 요구사항 (확정)

1. `frame_id`는 코스트맵·레퍼런스 경로 모두 `"odom"`으로 유지한다.
2. `scenario_manager_node`는 전역 지오메트리를 내부에 1회만 만들어 갖고 있고, **차량 기준 좌표로 정합한 로컬 코스트맵을 5Hz로 발행**한다.
3. `planner_node`는 5Hz로 들어오는 이 좌표계를 odom(전역)처럼 조회할 수 있어야 하고, 이를 위해 **매 코스트맵 메시지(캡처 시점)마다 고유한 변환행렬**을 적용한다. 이 변환은 충돌검사를 포함한 모든 코스트맵 조회 지점에서 확인 가능해야 한다.

## 좌표 정합: AABB가 아니라 "쿼리 포인트를 그리드 프레임으로 역회전"

> 이전 버전 문서는 이 부분을 "정합 후 X/Y 오프셋으로 min/max를 만들어 AABB를 구성한다"고 잘못 서술하고 있었다. 실제 구현(그리고 최종 확정한 방식)은 AABB를 전혀 만들지 않는다.

- 그리드 셀 자체는 **캡처 시점 차량 바디 프레임**(회전된 상태)으로 저장된다. `frame_id`(=odom)와 `OccupancyGrid.info.origin`(=캡처 시점 차량 pose: position + yaw)의 조합이 전역 좌표와의 관계를 완전히 정의한다.
- 조회 시(`Costmap2D::distance_at_world`, [core.cpp:1275-1298](simp_planner/simp_planner_cpp/src/core.cpp#L1275-L1298)):
  1. odom 쿼리 좌표 `(x, y)`에서 `origin_x_/origin_y_`를 뺀다.
  2. 그 결과를 `-origin_yaw`만큼 **역회전**시켜 그리드가 저장된 로컬(=캡처 시점 차량) 프레임으로 가져온다.
  3. 역회전된 로컬 좌표를 바로 그리드 인덱스로 쓰고, 그 로컬 프레임 안에서 `0 <= local < width/height`인지만 확인한다.
- 즉 "박스를 만들어서 포함 여부를 검사"하는 게 아니라 **쿼리 포인트 쪽을 박스의 기준 축으로 되돌려서 검사**하는 것이므로, 결과적으로 회전된 직사각형 경계 그대로(OBB, oriented bounding box) 정확하게 체크된다.
- 만약 대신 odom 프레임에서 회전된 사각형을 감싸는 AABB를 썼다면, 45도 부근에서 실제 영역보다 최대 `√2`배 넓은 박스가 되어 그리드 밖인데도 "범위 안"으로 오판할 여지가 생긴다. 지금 방식은 그 문제가 구조적으로 없다.

## 범위(extent): 레퍼런스 경로 점의 AABB + 오프셋 → **planner가** 그 박스만 rasterize/EDT

핵심 원칙: **시나리오/센서 쪽은 이 개념을 몰라도 된다.** `scenario_manager_node.py`는 계속 차량 Yaw로 회전된 고정 크기 정사각형(실제 센서가 만드는 로컬 코스트맵의 시뮬레이션)을 그대로 보낸다 — 아래 "아키텍처" 절 참고. 레퍼런스 경로 슬라이스·candidate 최대 길이·차량 크기 같은 **planner 고유의 지식은 전부 planner 쪽에 있으므로, 크롭 자체도 planner가 한다**: 받은 정사각형 중 이번 사이클에 실제로 필요한 부분만 잘라내서 그 부분에 대해서만 EDT(Euclidean distance transform)를 돌린다. (이전 버전 문서는 이 크롭을 `scenario_manager_node.py`에 넣는 안을 검토했었는데, 책임 소재가 잘못됐다는 지적을 받고 여기로 옮겼다.)

winding 경로에서는 호길이(S)와 실제 x/y 퍼짐이 크게 어긋나므로(경로가 급커브면 짧은 S 구간에도 lateral로 크게 벌어짐), 범위는 **레퍼런스 경로 점 자체의 좌표(회전 후 min/max)** 로 정하고 그 위에 마진을 더하는 방식으로 잡는다.

구현: `crop_costmap_to_reference_path_window()` ([planner_node.cpp](simp_planner/simp_planner_cpp/src/planner_node.cpp), `costmap_callback()` 바로 앞에 정의, `costmap_callback()` 안에서 `Costmap2D` 생성 직전에 호출).

### 1단계 — 차량 위치의 Frenet 투영 + S 스윕 범위 결정
`reference_path_->project(state.x, state.y, state.chi)`로 현재 위치의 `s`를 구하고, 그 앞뒤로 스윕할 범위를 정한다:
- 전방: `ahead_length_s = max(adaptive_replan.minimum_spatial_preview, lateral.max_length)` ≈ 28~30m — 최대 candidate S 진행거리를 커버.
- 후방: `back_length_s` = 아래 종방향 마진과 같은 값 — 차량 몸체만 덮으면 되므로 별도 상수 없이 재사용.

### 2단계 — 그 S 범위를 스윕하며, "이 메시지 자신의" origin/yaw로 회전
`reference_path_->evaluate(s, x, y, ...)`로 1m 간격 점을 뽑고, **받은 코스트맵 메시지 자신의 origin_x/origin_y/origin_yaw**(현재 차량 pose가 아니라 — 캡처 시점과 조회 시점 사이에 시차가 있으므로, `distance_at_world`와 동일한 회전 규칙)로 그리드의 로컬 프레임으로 옮긴 뒤 `local_x`/`local_y`의 min/max를 구한다.

### 3단계 — 마진 추가
- 종방향(±): `0.5×vehicle.length + footprint_margin + v_max×costmap_update_period_sec` (마지막 항은 캡처 사이 이동거리 버퍼이며 simulation launch가 실제 publish 주기를 planner에 전달)
- 횡방향(±): `max|lateral.n_targets| + 0.5×vehicle.width + footprint_margin` ≈ 9.0m

### 4단계 — 셀 인덱스로 변환, 크롭, `Costmap2D` 생성
로컬 min/max를 그리드 해상도로 나눠 `[col_start,col_stop) × [row_start,row_stop)` 셀 범위로 만들고(수신 그리드 범위로 clamp), 그 부분 배열만 잘라내서 `Costmap2D`를 생성한다 — **EDT는 이 잘라낸 부분에 대해서만 실행**된다. 레퍼런스 경로/차량 상태가 아직 없거나(`received_path_`/`received_odom_` 둘 다 아직 false), 계산된 창이 원본보다 안 작으면 원본 그리드 그대로 사용 — 항상 안전한 폴백.

### 실측 확인 (winding_obstacle_course)
직접 시나리오를 돌려서 확인함 — 원본 60×60m(3600㎡) 정사각형이 매 tick 약 800~1300㎡(원본의 20~35%) 수준의 직사각형으로 잘려서 그 위에서만 EDT가 도는 걸 디버그 플롯("Local costmap ... (planner window)" 라벨, [debug_plot_renderer.py](simp_planner_tools/simp_planner_tools/debug_plot_renderer.py))으로 확인했다. 급커브 구간에서 좌우 폭이 비대칭으로 달라지는 것도 기대대로 동작.

## 아키텍처

### 좌표계
- `frame_id`는 계속 `"odom"`이지만, 발행되는 `OccupancyGrid.info.origin`에 **position + orientation(캡처 시점 차량 Yaw)**을 둘 다 채운다.
- 그리드 셀은 "캡처 시점 차량 바디 프레임" 기준으로 배치된다. 자세한 조회 방식은 위 "좌표 정합" 절 참고.

### scenario_manager_node.py (Python) — 변경 없음, 고정 회전 정사각형 그대로 유지

1. `/odom` 구독(`odom_callback`)에서 최신 차량 pose(`self.current_x/y/current_body_yaw`)를 저장.
2. 타이머(`costmap_publish_hz`, 노드 자체 기본값 10.0 — [scenario_manager_node.py:77](simp_planner_tools/simp_planner_tools/scenario_manager_node.py#L77) — 이지만 `simulation.launch.py`가 5.0으로 오버라이드 — [launch/simulation.launch.py:158](simp_planner_tools/launch/simulation.launch.py#L158))로 `publish_costmap()` 호출.
3. 매 tick, `rasterize_vehicle_costmap()` ([scenario_definition.py:706](simp_planner_tools/simp_planner_tools/scenario_definition.py#L706))이 차량 현재 pose를 중심으로 `size_m`×`size_m`(기본 60.0m) 정사각 그리드를 만들고, `scenario.obstacles`를 차량 바디 프레임으로 직접 좌표변환해서 그 안에 rasterize. 레퍼런스 경로는 참조하지 않음 — 실제 센서를 흉내낸 것이므로 의도한 그대로.
4. 발행되는 `OccupancyGrid.info.origin`은 position + orientation(캡처 시점 차량 Yaw의 quaternion) 둘 다 채움 ([scenario_manager_node.py:306-312](simp_planner_tools/simp_planner_tools/scenario_manager_node.py#L306-L312)).

### planner_node.cpp / core.cpp (C++)

1. `costmap_callback()`의 회전 origin 거부 로직 제거, quaternion에서 yaw를 추출해 `Costmap2D` 생성자에 전달.
2. `Costmap2D`에 `cos_origin_yaw_`/`sin_origin_yaw_` 캐시 추가(생성자에 `origin_yaw` 파라미터, 기본값 0.0 — 하위호환) — [core.hpp:368-401](simp_planner/simp_planner_cpp/include/simp_planner/core.hpp#L368-L401).
3. `distance_at_world(x, y)`에서 조회 좌표를 그리드 조회 전에 역회전 — [core.cpp:1275-1298](simp_planner/simp_planner_cpp/src/core.cpp#L1275-L1298) (위 "좌표 정합" 절 참고).
4. `costmap_fingerprint()`에도 origin의 yaw를 반영 — position이 같고 orientation만 바뀐 경우도 갱신으로 잡히도록 ([planner_node.cpp:407-428](simp_planner/simp_planner_cpp/src/planner_node.cpp#L407-L428)).
5. **(신규)** `crop_costmap_to_reference_path_window()` — 위 "범위(extent)" 절 1~4단계. `costmap_callback()`에서 `received_path_ && received_odom_`일 때만 호출하고, 아니면 원본 그리드로 폴백.
6. **(신규)** 크롭 결과(origin x/y/yaw, width/height, resolution)를 `costmap_crop_origin_x/y/yaw`, `costmap_crop_width/height/resolution` 필드로 status JSON의 `"plan"` 객체에 포함 — `CostmapBuildSnapshot`을 확장해서 `costmap_build_ms`/`costmap_rebuild_count`와 같은 락으로 스냅샷.

### 디버그 시각화 (debug_plot_node.py / debug_plot_renderer.py)

- `_costmap_geometry()`에 `prefix` 인자를 추가해서 `"costmap"`(수신 원본, 래스터 이미지용)과 `"costmap_crop"`(planner가 실제로 처리한 영역, 점선 박스용)을 같은 코드로 처리.
- `_draw_costmap_boundary()`가 `costmap_crop_*`을 우선 사용하고, 없으면(구버전 스냅샷 등) 원본 `costmap_*`으로 폴백.
- **중요 — sticky 캐싱이 필요했음:** `/planner/status`는 매 사이클 **두 종류**의 메시지를 낸다 — `publish_status("PENDING_PLAN_READY", ...)`(lean, crop 필드 없음)과 `publish_plan_status()`(rich, crop 필드 포함). 실측(6초간 62개 메시지)으로 lean 42개(68%) vs rich 18개(29%, 나머지는 STALE_PLAN_DISCARDED)를 확인했다 — `debug_plot_node.py`가 "가장 최근 메시지"만 본다면 스냅샷 저장 시점에 lean이 걸릴 확률이 훨씬 높아서 실제로 처음 두 번의 실측 스냅샷 모두 크롭 박스 없이 "as received"로 나왔다. `self.costmap_crop_*`을 rich 메시지가 올 때만 갱신되는 **sticky 필드**로 바꿔서(기존 `self.costmap_origin`이 `/costmap` 메시지에서만 갱신되는 것과 동일한 패턴) 해결 — lean 메시지가 그 사이에 껴도 마지막으로 알려진 크롭 박스를 계속 보여준다.
- 마커/주석도 "박스 중심 = 차량 위치" 가정을 버리고 실제 `current_state.x/y`를 로컬 좌표로 변환해서 "ahead/behind/left/right" 형식으로 표시 — 크롭 박스는 대개 비대칭이라 중심이 차량 위치와 다르다.

## 관련 파라미터

| 파라미터 | 값 | 위치 | 비고 |
|---|---|---|---|
| `costmap_size_m` | 60.0 (정사각) | `scenario_manager_node.py` | 시나리오 쪽은 이대로 유지 (변경 없음) |
| `costmap_publish_hz` | 노드 기본 10.0, launch에서 5.0으로 오버라이드 | `scenario_manager_node.py` / `simulation.launch.py` | |
| 전방 S 스윕 범위 | `max(adaptive_replan.minimum_spatial_preview, lateral.max_length)` ≈ 28~30m | `planner_node.cpp: crop_costmap_to_reference_path_window()` | |
| 종방향 마진(±) | `0.5×vehicle.length + footprint_margin + v_max×costmap_update_period_sec` | 〃 | launch에서 `costmap_update_period_sec=1/costmap_publish_hz` 자동 전달 |
| 횡방향 마진(±) | `max\|lateral.n_targets\| + 0.5×vehicle.width + footprint_margin` ≈ 9.0m | 〃 | |

### `path_ahead_length`와 `terminal_safe_region_active` (코스트맵 크롭과는 무관 — 참고용)

`path_ahead_length`(`scenario_manager_node.py`, 기본 35.0)는 **로컬 레퍼런스 경로 publish 범위**(`/reference_path` 용도) 파라미터다. 위 크롭이 planner의 C++ config(`adaptive_replan.minimum_spatial_preview`, `lateral.max_length`)로 옮겨가면서, 이 파라미터는 코스트맵 크기와는 이제 완전히 무관해졌다 — 다만 아래 별개의 이유로 여전히 32.0보다 커야 한다.

`terminal_safe_region_active`(spatial 버킷을 나누는 조건)는 `remaining_to_goal <= safe_region_activation`을 필요로 하는데, `safe_region_activation`은 `config_.adaptive_replan.minimum_spatial_preview`(=32.0m) 하한을 가진다. 로컬 레퍼런스 창의 전방 길이(`path_ahead_length`)가 이 하한보다 짧으면(30.0 < 32.0), `remaining_to_goal`(로컬 창 끝까지 남은 거리, 최대 `path_ahead_length`)이 **구조적으로 항상** `safe_region_activation` 이하가 되어버려서, `terminal_safe_region_active`가 실질적으로 `!terminal_center_region_safe`(장애물 유무) 하나로만 결정되는 부작용이 있었다. 실제 기본값 `35.0`은 이미 `32.0`보다 커서 이 버그 조건을 피하고 있다.

## 진행 상태

1. ✅ `path_ahead_length`(35.0)/`path_back_length`(5.0): 문서엔 45.0 복원으로 잘못 기록돼 있었으나, 실제 기본값 35.0으로 `terminal_safe_region_active` 버그 조건은 이미 회피됨. 코스트맵 크기 산정과는 이제 무관(크롭이 planner의 C++ config로 옮겨감) — 착오만 정정.
2. ✅ `scenario_manager_node.py`: 5Hz 회전 origin 정사각형 발행 — 원래 구현 그대로 유지(레퍼런스 경로 지식은 여기 두지 않음).
3. ✅ `planner_node.cpp`/`core.cpp`: 회전 origin 지원 재구현 (`Costmap2D` + `distance_at_world` + fingerprint에 yaw 반영). 쿼리 시점 역회전 방식(OBB) 확인 완료.
4. ✅ **(신규)** `planner_node.cpp`: `crop_costmap_to_reference_path_window()` 구현 — 수신 정사각형을 레퍼런스 경로 슬라이스+마진 기반 직사각형으로 크롭한 뒤 그 위에서만 `Costmap2D`(EDT) 생성. 경로/상태 미수신 또는 크롭 무의미 시 원본으로 안전 폴백.
5. ✅ **(신규)** status JSON에 `costmap_crop_origin_x/y/yaw`, `costmap_crop_width/height/resolution` 추가 (`publish_plan_status()`에서만 — lean `publish_status()`에는 없음).
6. ✅ **(신규)** 디버그 시각화가 planner의 실제 크롭 영역을 표시하도록 갱신 — sticky 캐싱으로 lean 상태 메시지가 끼어도 마지막 크롭 박스 유지. `winding_obstacle_course` 실행해서 확인: "Local costmap 31.4×35.8 m (planner window)" / "ahead 24.7 m, behind 6.7 m, left 7.9 m, right 27.9 m" — 비대칭 직사각형이 정상적으로 그려짐.
7. ✅ 빌드(`colcon build --packages-select simp_planner_cpp`)/유닛테스트(`simp_planner_tools`, 무관한 사전 실패 2건 제외 36개 전부) 통과.
8. ⬜ 실측: `costmap_build_ms`/`costmap_rebuild_count`가 크롭 전/후로 정량적으로 얼마나 달라지는지 `planning_call_count_report_node`의 기존 패널로 비교 — 아직 미실행(정성적으로는 디버그 플롯에서 60×60→~30×35 수준 축소를 확인함). `costmap_fingerprint()`가 position+yaw를 포함해 움직이는 차량에서는 5Hz 메시지마다 사실상 매번 리빌드가 발생하므로(planner는 최대 100Hz), 이 크롭이 그 반복 비용을 얼마나 낮추는지가 핵심 확인 대상.
