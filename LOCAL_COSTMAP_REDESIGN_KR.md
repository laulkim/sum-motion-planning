# Local Costmap 5Hz 전환 설계안

## 배경 / 목적

현재 `scenario_manager_node`는 시나리오 전체 경로를 커버하는 코스트맵을 시작 시점에 1회만(latched QoS) 발행한다. 이를 실제 로봇의 로컬 코스트맵처럼 **차량 기준 5Hz 주기 발행**으로 바꾸는 게 목표다.

이번 전환의 부차적인 목적은, 5Hz 코스트맵 캡처 시점과 매 planning 사이클의 조회 시점 사이에 발생하는 **좌표 정합(coordinate registration) 연산**을 명시적으로 만들어서, 그 연산 비용을 디버깅/계측할 수 있게 하는 것이다.

> 중간에 "회전 없는 축정렬 방식"으로 단순화했다가, 최종적으로 다시 **회전 포함(차량 Yaw 기준 좌표축)** 방식으로 확정했다. 아래는 최종 확정 버전이다.

## 요구사항 (확정)

1. `frame_id`는 코스트맵·레퍼런스 경로 모두 `"odom"`으로 유지한다.
2. `scenario_manager_node`는 전역 지오메트리를 내부에 1회만 만들어 갖고 있고, **차량 기준 좌표로 정합한 로컬 코스트맵을 5Hz로 발행**한다.
3. `planner_node`는 5Hz로 들어오는 이 좌표계를 odom(전역)처럼 조회할 수 있어야 하고, 이를 위해 **매 코스트맵 메시지(캡처 시점)마다 고유한 변환행렬**을 적용한다. 이 변환은 충돌검사를 포함한 모든 코스트맵 조회 지점에서 확인 가능해야 한다.

## 범위(extent)와 좌표축(basis)은 서로 다른 기준으로 정해진다

- **범위**: 투영 S 위치(`projection_s`) 기준 로컬 레퍼런스 경로 슬라이스(`path_back_length`=5m, `path_ahead_length`=45m)를 따라간다. 즉 "어디부터 어디까지 담을지"는 경로 형상을 그대로 따른다.
- **좌표축**: 차량의 현재 Yaw로 회전된 축을 쓴다. 즉 "그 담긴 영역을 어떤 기준 축으로 표현할지"는 차량 헤딩이 결정한다.
- 여기에 좌우(차량 기준 lateral) 방향으로 `costmap_lateral_margin`(10.0m) 여유를 추가로 둔다.

## 아키텍처

### 좌표계
- `frame_id`는 계속 `"odom"`이지만, 발행되는 `OccupancyGrid.info.origin`에 **position + orientation(캡처 시점 차량 Yaw)**을 둘 다 채운다.
- 그리드 셀은 "캡처 시점 차량 바디 프레임" 기준으로 배치되고, `frame_id`(=odom)와 `origin`(=캡처 시점 차량 pose) 조합으로 전역 좌표와의 관계가 완전히 정의된다.

### scenario_manager_node.py (Python)
1. 전역 그리드는 `rasterize_scenario_costmap()`으로 `__init__`에서 **1회만** 생성해서 `self.global_grid`/`self.global_origin_x/y`로 보관.
2. `/odom` 구독(`odom_callback`)에서 최신 차량 pose(`self.current_x`, `self.current_y`, `self.current_body_yaw`)를 저장.
3. 5Hz 타이머(`costmap_publish_hz`, 기본 5.0)로 `publish_costmap()` 호출.
4. 매 tick (`publish_costmap()`):
   - `self.active_path.local_slice(projection_s, path_back_length, path_ahead_length)`로 로컬 레퍼런스 경로 점들을 얻음(odom 좌표).
   - 이 점들을 차량 Yaw 기준 로컬 좌표(`local_path_x`=전방, `local_path_y`=좌측)로 변환 → 그 범위가 회전된 윈도우의 전후 크기를 결정.
   - 좌우(local_path_y)에는 `costmap_lateral_margin`(10.0m)을 추가로 더함.
   - 회전된 로컬 윈도우의 각 셀을 차량 pose로 월드좌표 변환한 뒤, 전역 그리드에서 **nearest-neighbor**로 값을 채움.
5. 전역 맵 범위 밖은 `-1`(unknown)로 채움 — 장애물도 자유공간도 아닌, 기존 unknown-is-occupied 관례 유지.
6. 발행되는 `OccupancyGrid.info.origin`은 position + orientation(캡처 시점 차량 Yaw의 quaternion) 둘 다 채움.

### planner_node.cpp / core.cpp (C++)
1. `costmap_callback()`의 회전 origin 거부 로직 제거, quaternion에서 yaw를 추출해 `Costmap2D` 생성자에 전달.
2. `Costmap2D`에 `cos_yaw_`/`sin_yaw_` 캐시 추가(생성자에 `origin_yaw` 파라미터, 기본값 0.0 — 하위호환).
3. `distance_at_world(x, y)`에서 조회 좌표를 그리드 조회 전에 회전 변환:
   ```
   [dx, dy] = [x - origin_x, y - origin_y]
   [local_x, local_y] = R(-origin_yaw) · [dx, dy]      ← 요구사항 3의 "변환행렬"
   (이후 local_x, local_y로 기존 격자 보간 로직 그대로 사용)
   ```
4. `costmap_fingerprint()`에도 origin의 yaw를 반영 — position이 같고 orientation만 바뀐 경우도 갱신으로 잡히도록.

## 관련 파라미터

| 파라미터 | 값 | 위치 | 비고 |
|---|---|---|---|
| `path_back_length` | 5.0 | scenario_manager_node.py | 로컬 레퍼런스/코스트맵 후방 |
| `path_ahead_length` | **45.0** (30.0에서 복원) | scenario_manager_node.py | 로컬 레퍼런스/코스트맵 전방 — `minimum_spatial_preview`(32.0)보다 커야 함 |
| `costmap_lateral_margin` | 10.0 | scenario_manager_node.py | 좌우 여유 |
| `costmap_publish_hz` | 5.0 | scenario_manager_node.py | 코스트맵 발행 주기 |

### `path_ahead_length`를 30 → 45로 되돌린 이유 (중요한 발견)
`terminal_safe_region_active`(spatial 버킷을 나누는 조건)는 `remaining_to_goal <= safe_region_activation`을 필요로 하는데, `safe_region_activation`은 `config_.adaptive_replan.minimum_spatial_preview`(=32.0m) 하한을 가진다. 로컬 레퍼런스 창의 전방 길이(`path_ahead_length`)가 이 하한보다 짧으면(30.0 < 32.0), `remaining_to_goal`(로컬 창 끝까지 남은 거리, 최대 `path_ahead_length`)이 **구조적으로 항상** `safe_region_activation` 이하가 되어버려서, `terminal_safe_region_active`가 실질적으로 `!terminal_center_region_safe`(장애물 유무) 하나로만 결정되는 부작용이 있었다. 실제로 `winding_obstacle_course` 로그에서 장애물 밀집 구간마다 "terminal/stop-region" 스파이크가 빈번히 발생하는 걸 확인했다. `path_ahead_length`를 45.0으로 되돌려 32.0m 하한보다 여유 있게 만들어 해소했다.

## 진행 상태

1. ✅ `path_ahead_length`: 45.0 → 30.0 → **45.0 복원**.
2. ✅ `scenario_manager_node.py`: 5Hz 타이머 + 레퍼런스 경로 범위 + 차량 Yaw 좌표축 + 좌우 마진 크롭.
3. ✅ `planner_node.cpp`/`core.cpp`: 회전 origin 지원 재구현 (`Costmap2D` + `distance_at_world` + fingerprint에 yaw 반영).
4. ✅ 빌드(colcon)/유닛테스트/validation g++ 빌드 전부 통과.
5. ⬜ 실측: 5Hz 재계산 비용, 좌표 정합 결과, `costmap_build_ms`/`costmap_rebuild_count` 패널로 확인 — 아직 미실행.
