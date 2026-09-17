# Planner 내부 Costmap ROI Crop 및 Distance Map 최적화 설계안

## 1. 목적

상위 모듈이 발행하는 차량 기준 정사각형 Costmap 인터페이스는 유지하면서,
Planner 내부에서 실제 Planning에 필요한 직사각형 영역만 추출하여 Distance Map을
생성한다.

이 최적화의 목적은 AABB 계산 시간을 줄이는 것이 아니라, Distance Transform이
처리하는 cell 수를 줄이는 것이다.

```text
Square Vehicle-frame Costmap 입력
                ↓
Planner 내부 Reference 기반 rectangular ROI 계산
                ↓
ROI occupancy crop
                ↓
ROI Distance Map 생성
                ↓
기존 collision/scoring 로직에서 사용
```

외부 Costmap의 다음 항목은 변경하지 않는다.

* message type과 topic
* resolution
* width와 height
* origin pose
* frame과 timestamp
* occupancy, unknown, free, occupied 의미
* 송신 측 Costmap crop 정책

이 문서는 구현 전 설계를 확정하기 위한 문서이며, 아직 기존 Full Costmap 경로를
삭제하거나 동작 의미를 변경하지 않는다.

---

## 2. 가장 중요한 정확성 조건

단순히 Reference Path AABB를 최대 lateral sampling 범위인 8 m만큼 확장한 뒤
occupancy를 자르고 Distance Transform을 수행하면 안 된다.

ROI 경계 안쪽의 query point와 매우 가까운 장애물이 ROI 바로 바깥에 있을 수 있다.
이 장애물을 crop 단계에서 제거하면 다음 관계가 성립한다.

$$
d_{crop}(q) \ge d_{full}(q)
$$

즉 crop된 Distance Map은 실제 Full Costmap보다 장애물 거리를 크게 계산할 수 있다.
이는 단순한 근사 오차가 아니라 충돌 또는 안전거리 위반을 놓칠 수 있는 비보수적
오류다.

따라서 원본 Costmap에서 가져오는 범위는 최소한 다음 조건을 만족해야 한다.

$$
\boxed{
P_{source}
\ge
N_{max}
+ R_{collision\ influence}
+ M_{safety}
+ M_{grid}
}
$$

현재 최대 lateral offset이 8 m이므로 이를 풀어 쓰면 다음과 같다.

$$
\boxed{
P_{source}
\ge
8\,\mathrm{m}
+ \text{collision footprint 영향거리}
+ \text{margin}
}
$$

단, collision 이외의 scoring 또는 bottleneck 판단에서 더 먼 Distance Map 값을
사용한다면 그 거리가 footprint 영향거리보다 우선한다. 최종 padding은
`distance_at_world()`의 모든 사용처가 요구하는 최대 의미 거리를 기준으로 정해야
한다.

---

## 3. 하나가 아닌 두 개의 ROI

경계 정확성을 명확하게 보장하기 위해 ROI를 다음 두 영역으로 구분한다.

| 영역 | 의미 | 용도 |
|---|---|---|
| `Query ROI` | Planner가 실제로 Distance Map을 조회할 수 있는 모든 query point의 내부 영역 | 유효한 Distance Map 조회 영역 |
| `DT Source ROI` | `Query ROI` 바깥의 장애물 영향거리까지 포함한 외부 영역 | occupancy crop 및 Distance Transform 입력 |

`DT Source ROI - Query ROI` 영역을 `halo`라고 한다.

```text
+---------------------------------------------------+
| DT Source ROI                                     |
|                                                   |
|     halo                                          |
|     +---------------------------------------+     |
|     | Query ROI                             |     |
|     | candidate, circle center, 보간 pose 등|     |
|     | 실제 collision/scoring query 영역     |     |
|     +---------------------------------------+     |
|                                                   |
+---------------------------------------------------+
```

Distance Transform은 `DT Source ROI` 전체에서 계산하지만, 그 결과를 Planning에
사용할 때는 `Query ROI`만 유효 영역으로 취급한다. halo는 query 범위를 넓히기 위한
공간이 아니라 경계 밖 장애물의 영향을 내부 거리값에 전달하기 위한 계산용 공간이다.

---

## 4. Reference AABB와 Query ROI

### 4.1 동일 snapshot과 좌표계 사용

ROI 계산에는 Costmap과 동일한 pose/timestamp snapshot을 사용한다. Reference Path가
Costmap grid와 다른 좌표 표현을 사용한다면 Costmap 갱신 또는 Reference Path 갱신
시 한 번 변환하고, candidate 생성 루프 안에서 반복 변환하지 않는다.

ROI와 Distance Map cache의 key에는 Costmap revision만 넣어서는 안 된다. 최소한
Reference Path revision, origin pose/yaw, ROI 관련 lateral 설정, footprint, hard/soft
margin 변경도 rebuild 조건에 포함하고, 한 번의 build에는 동일 revision 조합의
snapshot만 사용한다.

현재 Costmap처럼 `origin_yaw`를 갖는 회전 격자에서는 `frame_id` 문자열이 같더라도
world 좌표의 AABB를 그대로 grid index로 바꾸면 안 된다. 각 Reference point를 먼저
Costmap grid-local 좌표로 변환한다.

$$
\mathbf p_i^{grid}
=
R(-\theta_{map})
\left(
\mathbf p_i^{world}-\mathbf o_{map}^{world}
\right)
$$

여기서 $\mathbf o_{map}^{world}$는 Costmap origin position이고,
$\theta_{map}$은 origin yaw다.

### 4.2 Reference AABB 계산

grid-local Reference point를 $P_i=(x_i,y_i)$라고 할 때 한 번의 loop에서 extrema를
계산한다.

$$
x_{min}^{ref}=\min_i x_i,
\quad
x_{max}^{ref}=\max_i x_i
$$

$$
y_{min}^{ref}=\min_i y_i,
\quad
y_{max}^{ref}=\max_i y_i
$$

복잡도는 $O(N_{ref})$다. 곡선 경로를 안전하게 포함하기 위해 실제 Reference Path
좌표의 extrema를 사용하며, 차량의 현재 heading 방향으로 경로가 진행된다고
가정하지 않는다.

### 4.3 최대 candidate 기준점 범위 반영

최대 lateral sampling offset은 하드코딩된 숫자보다 실제 설정에서 계산한다.

$$
N_{max}=\max_j |n_j|
$$

현재 `n_targets` 기준으로 $N_{max}=8\,\mathrm{m}$다.

안전한 축 정렬 over-approximation으로 candidate 차량 기준점 영역을 다음과 같이
정의한다.

$$
B_{candidate}
=
[x_{min}^{ref}-P_{candidate},\ x_{max}^{ref}+P_{candidate}]
\times
[y_{min}^{ref}-P_{candidate},\ y_{max}^{ref}+P_{candidate}]
$$

$$
P_{candidate}=N_{max}+M_{trajectory}
$$

$M_{trajectory}$는 target 이외에 실제 candidate 차량 기준점이 더 벗어날 수 있는
범위다. 초기 Frenet offset, polynomial overshoot, terminal/virtual extension,
allocation/interpolation 과정이 모두 $|n|\le N_{max}$를 보장한다면 0으로 둘 수 있다.
그 invariant를 증명하지 못하면 실제 생성된 candidate AABB를 사용하거나 알려진 최대
초과량을 반영한다.

실제 lateral offset은 Reference 법선 방향이지만, 두 grid 축을 모두
$P_{candidate}$만큼 확장하면 U-turn, S-curve, 큰 heading 변화에서도 누락 없는 AABB를
얻을 수 있다.

### 4.4 실제 Query ROI

집합 $Q$는 Planner가 `distance_at_world()`를 호출할 수 있는 모든 위치를 뜻한다.
candidate 차량 기준점뿐 아니라 다음 위치가 포함된다.

* coarse/ranking/terminal query point
* 최종 oriented multi-circle의 각 circle center
* swept collision 검사의 translation/yaw 보간 pose
* stationary, fallback, bottleneck 검사에서 사용하는 query point

candidate 기준점에서 실제 query point까지의 최대 변위를
$\Delta_{query,max}$라고 하면 안전한 근사 Query ROI는 다음과 같다.

$$
B_{query}
=
B_{candidate}\oplus\Delta_{query,max}
$$

multi-circle만 고려하면 $\Delta_{query,max}=\max_i\|\mathbf c_i\|_2$가 한 구성 요소다.
가능하면 생성 가능한 모든 실제 query의 AABB를 직접 구하고, 그렇지 않으면 위와 같은
증명 가능한 상한을 사용한다. 이후 debug build에서는 모든 query가 `Query ROI` 안에
있는지 assertion으로 확인한다.

---

## 5. Halo 영향거리 산정

### 5.1 일반식

Distance Map 사용처 $u$마다 해당 query가 Planner 결정에 영향을 주는 최대 obstacle
distance $D_u$를 조사한다. query 위치 자체는 앞 절의 집합 $Q$와 `Query ROI`에 이미
포함되어 있어야 한다.

* $D_u$: 해당 query에서 Planner 결정에 영향을 주는 최대 obstacle distance

필요한 연속 공간 halo는 다음과 같다.

$$
H_{continuous}
=
\max_u D_u
+M_{safety}
$$

따라서 `DT Source ROI`는 다음과 같이 정의한다.

$$
B_{source}
=
B_{query}\oplus H_{continuous}\oplus M_{grid}
$$

$\oplus$는 여기서 AABB의 모든 축 방향 확장을 의미한다.

최종적으로 Reference AABB에서 원본 occupancy를 가져와야 하는 총 padding은 다음과
같다.

$$
P_{source}
=
N_{max}
+M_{trajectory}
+\Delta_{query,max}
+\max_u D_u
+M_{safety}
+M_{grid}
$$

### 5.2 Oriented multi-circle collision model

차량 기준 circle 중심을 $\mathbf c_i=(c_{x,i},c_{y,i})$, circle 반경을 $r_i$,
hard clearance margin을 $M_{hard}$라고 하면 collision 판정에 필요한 최소 영향거리는
다음과 같다.

$$
R_{collision\ influence}
=
\max_i
\left(
\|\mathbf c_i\|_2+r_i+M_{hard}
\right)
$$

이는 circle 반경만 사용하는 값이 아니다. 차량 기준점에서 떨어진 circle 중심까지의
거리도 반드시 포함한다. 차량 yaw가 어떤 방향이더라도 $\|\mathbf c_i\|_2$를 쓰면
축 정렬 AABB를 안전하게 확장할 수 있다.

현재 구현처럼 모든 circle의 반경이 같더라도 각 중심의 종방향 offset은 별도로
포함해야 한다. `footprint_margin`이 이미 $r_i$에 반영되어 있다면 같은 margin을 다시
더하지 않도록 각 항의 의미를 코드와 일치시킨다.

계산 방식은 다음 둘 중 하나로 통일한다.

1. `Query ROI`를 circle center까지 확장한 뒤 halo에 $r_i+M_{hard}$만 더한다.
2. candidate 기준점 영역에서 바로 $\|\mathbf c_i\|_2+r_i+M_{hard}$를 더한다.

두 방식을 섞어 circle center offset이나 radius를 두 번 더하지 않는다.

### 5.3 Collision 외 Distance Map 사용처

halo는 최종 multi-circle 충돌 검사만 보고 정하면 부족할 수 있다. 구현 전에 최소한
다음 사용처를 전부 inventory 한다.

| 사용처 | Query ROI에 넣을 위치 | 결정에 필요한 거리 $D_u$ 예시 |
|---|---|---:|
| coarse collision | candidate/sample point | coarse circle radius + hard margin |
| circumscribed-radius ranking | candidate/sample point | circumscribed radius + soft margin |
| oriented multi-circle collision | 각 circle center | circle radius + hard margin |
| terminal region 검사 | 실제 terminal sample | 사용 circle radius + hard/soft margin |
| bottleneck trigger/release | Reference query point | trigger/release threshold |
| clearance scoring | 해당 sample point | cost가 0이 되는 최대 clearance threshold |

예를 들어 scoring에서 $D_{score,max}$까지 거리 차이가 비용에 영향을 준다면 다음을
만족해야 한다.

$$
H_{continuous}
\ge
\max
\left(
D_{collision,max},
D_{score,max},
D_{bottleneck,max},
\ldots
\right)
+M_{safety}
$$

현재 값만 보고 상수 `8.0 + footprint_radius`를 박아 넣지 않고, 관련 Planner 설정에서
계산된 값으로 halo를 구성한다.

현재 구현을 기준으로 audit를 시작할 때는 특히 다음 관계를 확인한다.

* soft obstacle cost는 `clearance = distance - ranking_radius`와
  `max(0, soft_margin - clearance)`를 사용하므로
  `ranking_radius + obstacle_soft_margin_max`까지 의미가 있다.
* bottleneck은 trigger뿐 아니라 더 큰 release threshold까지 포함해야 한다.
* final oriented collision은 각 circle center에서 `circle radius + hard margin`까지
  의미가 있다.

위 값은 현재 코드의 출발점이며, 모든 호출부 inventory가 끝난 뒤 하나의
`D_use_max`로 확정한다.

---

## 6. Full Distance Map과의 동등성 범위

Full Costmap의 obstacle 집합을 $O_{full}$, crop된 obstacle 집합을 $O_{crop}$이라고
하면 다음과 같다.

$$
d_{full}(q)=\min_{o\in O_{full}}\|q-o\|
$$

$$
d_{crop}(q)=\min_{o\in O_{crop}}\|q-o\|
$$

$O_{crop}\subseteq O_{full}$이므로 obstacle을 crop에서 제외할수록 거리는 같거나
커진다. 유한한 halo로 모든 raw distance 값을 Full Costmap과 완전히 같게 만드는
것은 일반적으로 불가능하다. 매우 먼 obstacle이 raw minimum distance를 바꿀 수 있기
때문이다.

대신 Planner 결정에 필요한 최대 거리 $D_{use,max}$를 명시하고, 그보다 큰 거리는
동일한 의미로 포화시키면 다음 동등성을 보장할 수 있다.

$$
\min(d_{crop}(q),D_{use,max})
=
\min(d_{full}(q),D_{use,max})
$$

보장 조건은 모든 유효 query $q\in Q$에 대해 반경 $D_{use,max}$ 안의 원본 Costmap
obstacle이 `DT Source ROI`에 포함되는 것이다. circle center를 비롯한 실제 query
위치는 이미 $Q$와 `Query ROI`에 포함되어 있어야 한다.

따라서 구현의 목표는 다음 두 가지를 구분해야 한다.

1. collision/scoring 결과와 같은 **Planner semantics의 동등성**
2. threshold보다 먼 영역까지 포함한 **raw Distance Map 수치의 완전 동등성**

1번만 필요하다면 각 사용처가 threshold 이후의 거리를 실제로 동일하게 취급하는지
확인하거나 명시적으로 clamp한다. raw distance 자체가 제한 없이 비용에 들어간다면
유한 halo만으로는 동등성을 주장하지 말고, 해당 사용처를 변경하거나 Full Costmap
fallback을 사용한다.

---

## 7. Grid 이산화와 보간 margin

Costmap resolution을 $r$이라고 할 때 연속 공간의 halo를 cell 수로 올림한다.

$$
h_{base,cells}
=
\left\lceil
\frac{H_{continuous}}{r}
\right\rceil
$$

여기에 현재 `Costmap2D` 구현의 다음 요소를 보존할 guard cell이 필요하다.

* ROI min/max의 `floor()`와 `ceil()` 변환
* occupied cell 중심과 실제 cell 면적 차이에 대한 보수적 반대각 보정
* `distance_at_world()`의 bilinear interpolation이 참조하는 4-cell stencil
* floating-point 경계 오차

현재 구현은 EDT cell-center 거리에서 $0.5\sqrt{2}r$를 빼는 보수적 보정을 사용하고,
bilinear interpolation에서 인접 cell을 조회한다. 따라서 `M_grid`를 0으로 두지 않는다.
구현 시에는 연속 길이 margin과 interpolation guard cell을 분리해서 기록한다.

현재 동작에 대한 보수적인 초기식은 다음과 같다.

$$
h_{cells}
=
\left\lceil
\frac{D_{use,max}+0.5\sqrt{2}r+M_{safety}}{r}
\right\rceil
+g_{interp}
$$

현재 bilinear interpolation에는 $g_{interp}=1$ cell을 최소 출발점으로 사용한다.
더 엄밀하게 구현하려면 먼저 모든 query가 참조할 4-neighbor cell 집합
$I_Q$를 만들고, 그 cell 집합을 EDT 영향 cell 수만큼 확장한다. 이 방식은 미터 단위
AABB에서 floor/ceil과 보간 guard를 따로 추론하는 오류를 줄일 수 있다.

예시 형태는 다음과 같다.

```cpp
const int halo_cells =
    static_cast<int>(std::ceil(
        (continuous_halo_m + raster_guard_m) / resolution))
    + interpolation_guard_cells;
```

최종 guard 값은 cell center/edge/corner query를 포함한 경계 property test로 확정한다.

index convention은 코드 전체에서 `[min, max)`로 통일한다.

$$
i_{min}=\left\lfloor\frac{x_{min}}{r}\right\rfloor,
\qquad
i_{max}=\left\lceil\frac{x_{max}}{r}\right\rceil
$$

$$
j_{min}=\left\lfloor\frac{y_{min}}{r}\right\rfloor,
\qquad
j_{max}=\left\lceil\frac{y_{max}}{r}\right\rceil
$$

`i_max`, `j_max`는 포함되지 않는 exclusive bound다. clamp 전후에 빈 영역, overflow,
off-by-one을 검사한다.

---

## 8. 원본 Costmap과의 intersection

계산된 `DT Source ROI`는 원본 Costmap index 범위와 intersection 한다.

```cpp
source_i_min = std::max(requested_i_min, 0);
source_j_min = std::max(requested_j_min, 0);
source_i_max = std::min(requested_i_max, full_width);
source_j_max = std::min(requested_j_max, full_height);
```

필요 영역이 원본 Costmap 밖으로 나간 사실을 단순 clamp로 숨기지 않는다. 최소한
다음 상태를 구분한다.

| 상태 | 의미 | 기본 처리 방향 |
|---|---|---|
| `ROI_FULLY_CONTAINED` | Query ROI와 halo가 모두 원본 안에 있음 | ROI 경로 사용 |
| `SOURCE_HALO_CLIPPED` | Query ROI는 안에 있지만 halo 일부가 원본 밖임 | out-of-map 정책 적용 또는 Full 경로와 비교 |
| `QUERY_ROI_CLIPPED` | candidate query 가능 영역 자체가 원본 밖임 | 보수적 reject/fallback 검토 |
| `REFERENCE_OUTSIDE_COSTMAP` | Reference 영역이 원본과 유효하게 겹치지 않음 | Planning 불가 상태 반환 |

Full Costmap fallback도 센서 또는 원본 맵 바깥의 장애물 정보를 새로 만들 수는 없다.
따라서 `SOURCE_HALO_CLIPPED`의 정책은 기존 out-of-map/unknown 의미와 일치시켜야 한다.
현재처럼 맵 밖 query를 collision으로 취급하는 보수적 의미를 유지하거나, 잘린 halo
방향에 synthetic occupied boundary를 둘 수 있다. 어떤 정책이든 ROI 경로가 기존 Full
경로보다 낙관적으로 바뀌어서는 안 된다.

halo가 원본 경계에서 잘렸더라도 원본에 존재하는 cell을 모두 포함했다면 Full
Costmap baseline과의 capped-distance 동등성은 유지될 수 있다. 다만 이는 센서 바깥의
실제 환경이 안전하다는 뜻이 아니다. `SOURCE_HALO_CLIPPED`에서는 **기존 Full baseline
동등성**과 **원본 밖 환경의 unknown 안전 정책**을 별도 항목으로 판정한다. 누락 영역을
free cell로 synthetic padding해서는 안 된다.

---

## 9. Rectangular ROI Costmap 생성

원본 occupancy 배열에서 `DT Source ROI` index 범위만 row 단위로 복사한다.

ROI metadata는 다음을 갖는다.

* `source_width`, `source_height`
* 원본과 같은 resolution
* 원본과 같은 origin yaw
* crop offset이 반영된 새 origin position
* 원본과 같은 frame과 timestamp
* 원본과 동일한 occupancy/unknown 해석
* Query ROI의 source-local index 범위

원본 grid의 crop 시작 index를 $(i_0,j_0)$라고 하면 새 origin position은 다음과 같다.

$$
\mathbf o_{roi}^{world}
=
\mathbf o_{map}^{world}
+
R(\theta_{map})
\begin{bmatrix}
i_0r\\
j_0r
\end{bmatrix}
$$

origin yaw는 원본과 동일하게 유지한다. 내부 전용 구조에서 원본 origin과 index offset을
보관하는 방식도 가능하지만, 모든 world-to-grid 조회가 같은 결과를 내야 한다.

매 Costmap update마다 불필요한 allocation을 반복하지 않도록 다음을 우선 검토한다.

1. 기존 occupancy/Distance Map buffer의 capacity 재사용
2. 최대 관측 ROI 기준 pre-allocation
3. capacity를 유지하는 `resize()`
4. Distance Transform 구현이 허용할 때만 stride 기반 view

Distance Transform이 contiguous memory를 요구한다면 view를 위해 알고리즘을
복잡하게 만들지 않고 row copy 비용을 profiling한다.

---

## 10. 처리 흐름

```text
Square Vehicle-frame Costmap 수신
            ↓
동일 snapshot 기준 Reference Path를 grid-local 좌표로 변환
            ↓
Reference Xmin/Xmax/Ymin/Ymax 계산                    O(Nref)
            ↓
Query ROI = Reference AABB + Nmax(현재 8 m)
            ↓
모든 Distance Map 사용처에서 max(query offset + 의미 거리) 계산
            ↓
DT Source ROI = Query ROI + collision/scoring halo + grid guard
            ↓
원본 Square Costmap과 intersection + 상태 판정
            ↓
[min, max) grid index 변환
            ↓
Rectangular source occupancy 추출
            ↓
DT Source ROI에 대해서만 Distance Transform 생성
            ↓
Query ROI 안에서만 Distance Map 조회 허용
            ↓
기존 collision/scoring semantics와 Full 경로 결과 비교
```

---

## 11. Full Costmap fallback

Reference Path가 U-turn 또는 큰 S-curve를 가지면 AABB가 원본 Costmap 대부분을
차지할 수 있다. 이때 ROI copy 비용 때문에 오히려 느려질 수 있다.

$$
\rho
=
\frac{N_{source}}{N_{full}}
=
\frac{W_{source}H_{source}}{W_{full}H_{full}}
$$

`roi_use_threshold`를 parameter로 두고 다음처럼 선택할 수 있다.

```cpp
if (source_cell_count >= full_cell_count * roi_use_threshold) {
    build_full_distance_map();
} else {
    crop_and_build_roi_distance_map();
}
```

threshold는 임의로 확정하지 않고 실제 ROI copy와 Distance Transform 시간을 기준으로
결정한다. 정확성 검증이 끝나기 전까지 Full 경로는 A/B 비교 가능한 fallback으로
유지한다.

---

## 12. 정확성 검증

### 12.1 핵심 경계 회귀 테스트

반드시 다음 반례를 먼저 만든다.

1. query point를 `Query ROI` 오른쪽 경계 바로 안쪽에 둔다.
2. 장애물을 `Query ROI` 오른쪽 경계 바로 바깥이지만 collision 영향거리 안에 둔다.
3. 단순 `Reference AABB + 8 m` crop에서는 장애물이 사라짐을 확인한다.
4. halo가 포함된 `DT Source ROI`에서는 Full Costmap과 collision 결과가 같음을 확인한다.

같은 테스트를 좌, 우, 상, 하와 네 corner에 대해 수행한다.

### 12.2 Full/ROI 동등성 테스트

동일 occupancy 입력에서 Full Distance Map과 ROI Distance Map을 동시에 만든다. 모든
유효 query sample $q$에 대해 다음을 비교한다.

$$
\left|
\min(d_{roi}(q),D_{use,max})
-
\min(d_{full}(q),D_{use,max})
\right|
\le \epsilon
$$

수치뿐 아니라 다음 최종 결과도 같아야 한다.

* hard collision boolean
* coarse/ranking collision boolean
* oriented multi-circle collision boolean
* bottleneck trigger/release 상태
* obstacle/clearance cost
* selected candidate ID

### 12.3 필수 테스트 조합

* 직선, 급커브, S-curve, U-turn Reference Path
* $-8\,\mathrm{m}$와 $+8\,\mathrm{m}$ 끝단 candidate
* 여러 차량 yaw와 Costmap `origin_yaw`
* footprint의 앞/뒤 끝 circle이 ROI 경계에 가까운 경우
* obstacle이 Query ROI 밖, halo 안, halo 경계, halo 밖에 각각 있는 경우
* occupied, free, unknown cell 조합
* 원본 Costmap 네 변과 corner에서 halo가 clip되는 경우
* cell boundary와 정확히 일치하는 좌표
* bilinear interpolation의 4개 cell이 ROI 경계에 걸리는 좌표
* 현재 주요 해상도인 0.2 m와 추가 해상도
* 빈 obstacle map과 모든 cell이 occupied/unknown인 map

무작위 obstacle map과 query를 생성하는 property test도 추가한다. ROI 결과가 Full
결과보다 낙관적으로 바뀌는 사례가 하나라도 있으면 ROI 경로를 활성화하지 않는다.

---

## 13. 성능 계측

변경 전과 후를 동일한 Costmap snapshot과 Reference Path에서 비교한다.

측정 항목은 다음과 같다.

* Reference 좌표 변환 시간
* AABB 계산 시간
* halo와 ROI index 계산 시간
* ROI occupancy copy 시간
* Distance Map 생성 시간
* 전체 Costmap 처리 시간
* Full/Query/Source width와 height
* Full/Source cell count와 면적 비율
* buffer allocation/reallocation 횟수
* Full fallback 횟수와 사유

예시 로그는 다음과 같다.

```text
[Costmap ROI]
Full Map       : 300 x 300 = 90000 cells
Query ROI      : 180 x 80  = 14400 cells
DT Source ROI  : 200 x 100 = 20000 cells
Source Ratio   : 22.22 %
ROI Status     : ROI_FULLY_CONTAINED

Transform      : xx us
AABB           : xx us
ROI Copy       : xx us
Distance Map   : xx us
Total          : xx us
Fallback       : false
```

0.2 m 해상도에서는 면적 감소가 Distance Transform cell 수 감소로 직접 이어지므로,
`N_source / N_full`과 실제 `Distance Map` 시간을 함께 기록하면 효과를 명확하게
확인할 수 있다.

---

## 14. 구현 순서

1. `distance_at_world()`와 clearance 관련 모든 사용처를 inventory한다.
2. 각 사용처의 $\Delta_u$와 $D_u$를 표로 확정한다.
3. Query ROI와 DT Source ROI 계산을 독립 함수로 구현한다.
4. `[min, max)` index 및 회전 origin 변환 unit test를 먼저 추가한다.
5. 기존 Full Distance Map을 유지한 채 ROI Distance Map을 병렬 생성하는 검증 모드를
   추가한다.
6. 경계/곡선/unknown/property test에서 Full/ROI semantics 동등성을 확인한다.
7. buffer 재사용과 면적비 fallback을 적용한다.
8. profiling 결과로 `roi_use_threshold`를 확정한다.
9. 충분한 A/B 검증 후 ROI 경로를 기본값으로 전환한다.

---

## 15. 완료 조건

다음 조건을 모두 만족해야 이 최적화를 완료한 것으로 본다.

1. 외부 Square Costmap 인터페이스가 변경되지 않는다.
2. 곡선 Reference Path와 최대 $\pm8\,\mathrm{m}$ candidate 영역이 누락되지 않는다.
3. 원본 crop 범위가 최소 `8 m + collision footprint 영향거리 + margin`을 포함한다.
4. collision 외 모든 Distance Map 사용처의 최대 의미 거리까지 halo에 반영한다.
5. Query ROI 경계 밖의 가까운 장애물 때문에 거리가 과대평가되지 않는다.
6. Full/ROI의 truncated distance, collision, scoring, candidate 선택 결과가 허용 오차
   안에서 동일하다.
7. 원본 Costmap 경계 clip 상태가 명시적으로 보고되고 보수적으로 처리된다.
8. 동적 메모리 할당이 불필요하게 반복되지 않는다.
9. ROI가 작은 대표 시나리오에서 Distance Map cell 수와 생성 시간이 실제로 감소한다.
10. ROI가 큰 worst case에서는 profiling 기반 Full Costmap fallback이 동작한다.
