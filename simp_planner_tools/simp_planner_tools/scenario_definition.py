from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from .scenario_path import ScenarioPath


@dataclass(frozen=True)
class ScenarioObstacle:
    x: float
    y: float
    length: float
    width: float
    yaw: float = 0.0
    # RViz 표시용 종류일 뿐 costmap/플래너와는 무관하다: "pallet"(상자 쌓인 팔레트) | "wall"
    kind: str = "pallet"


@dataclass(frozen=True)
class ScenarioGate:
    s: float
    lateral_center: float
    gap_width: float
    barrier_extent: float
    obstacle_length: float


@dataclass(frozen=True)
class ScenarioPhase:
    name: str
    path: ScenarioPath
    cruise_speed: float
    switch_s: float | None = None
    # 경로 거리 구간별 목표 속도 ((s_start, s_end, speed), ...). 구간 밖은 cruise_speed.
    # 단계를 나누면 경계마다 정지하므로, 한 단계 안에서 속도만 바꿀 때 쓴다 (예: 램프 감속).
    speed_zones: tuple[tuple[float, float, float], ...] = ()

    def cruise_speed_at(self, s: float) -> float:
        for start, end, speed in self.speed_zones:
            if start <= s <= end:
                return float(speed)
        return float(self.cruise_speed)

    @property
    def mode(self) -> int:
        values = np.unique(self.path.mode.astype(int))
        if len(values) != 1:
            raise ValueError(f"Scenario phase '{self.name}' must use one drive mode")
        return int(values[0])


@dataclass(frozen=True)
class FloorMarking:
    """RViz 바닥 표현 전용(구역 칠, 차선, 화살표, 글자). costmap/플래너와 무관하다.

    kind: "area"   points = 볼록 다각형, 채워서 칠한다
          "line"   points = 폴리라인, width 폭의 띠로 칠한다 (넓으면 램프 노면 같은 면)
          "dashed" "line"과 같되 점선
          "arrow"  points = (꼬리, 머리)
          "label"  points = ((x, y),), text를 띄운다
    color: 시각화 노드의 FLOOR_COLORS 이름
    """
    kind: str
    points: tuple[tuple[float, float], ...]
    color: str = "white"
    width: float = 0.15
    text: str = ""


@dataclass(frozen=True)
class ScenarioDefinition:
    name: str
    phases: tuple[ScenarioPhase, ...]
    obstacles: tuple[ScenarioObstacle, ...] = tuple()
    gates: tuple[ScenarioGate, ...] = tuple()
    terminal_margin: float = 3.0
    stop_request_distance: float = 3.0
    repeat: bool = False
    costmap_resolution: float = 0.20
    footprint_circle_count: int = 3
    markings: tuple[FloorMarking, ...] = tuple()

    @property
    def default_speed(self) -> float:
        return float(self.phases[0].cruise_speed)


def _map_path(share_directory: Path, name: str) -> Path:
    return Path(share_directory) / "maps" / name


def obstacle_on_path(
    path: ScenarioPath,
    s_position: float,
    *,
    lateral_offset: float = 0.0,
    length: float = 3.2,
    width: float = 2.0,
    yaw_offset: float = 0.0,
    kind: str = "pallet",
) -> ScenarioObstacle:
    """Place a rectangular obstacle using the authored path tangent.

    ``yaw_offset`` rotates the obstacle relative to the local path tangent and
    is used by mixed-geometry validation scenarios.
    """
    s_value = float(np.clip(s_position, 0.0, path.total_length))
    x = float(np.interp(s_value, path.s, path.x))
    y = float(np.interp(s_value, path.s, path.y))
    yaw_unwrapped = np.unwrap(path.map_yaw)
    yaw = float(np.interp(s_value, path.s, yaw_unwrapped))
    x -= float(lateral_offset) * math.sin(yaw)
    y += float(lateral_offset) * math.cos(yaw)
    return ScenarioObstacle(
        x=x,
        y=y,
        length=float(length),
        width=float(width),
        yaw=float(yaw + yaw_offset),
        kind=kind,
    )



def gate_on_path(
    path: ScenarioPath,
    s_position: float,
    *,
    lateral_center: float,
    gap_width: float = 5.2,
    barrier_extent: float = 14.0,
    obstacle_length: float = 5.0,
) -> tuple[ScenarioGate, ScenarioObstacle, ScenarioObstacle]:
    """Create two long barriers with one finite traversable opening."""
    half_gap = 0.5 * float(gap_width)
    lower_edge = float(lateral_center) - half_gap
    upper_edge = float(lateral_center) + half_gap
    extent = float(barrier_extent)
    if not (-extent < lower_edge < upper_edge < extent):
        raise ValueError("Gate opening must remain inside barrier_extent")

    lower_width = lower_edge + extent
    upper_width = extent - upper_edge
    lower_center = 0.5 * (-extent + lower_edge)
    upper_center = 0.5 * (upper_edge + extent)
    gate = ScenarioGate(
        s=float(s_position),
        lateral_center=float(lateral_center),
        gap_width=float(gap_width),
        barrier_extent=extent,
        obstacle_length=float(obstacle_length),
    )
    lower = obstacle_on_path(
        path, s_position, lateral_offset=lower_center,
        length=obstacle_length, width=lower_width,
    )
    upper = obstacle_on_path(
        path, s_position, lateral_offset=upper_center,
        length=obstacle_length, width=upper_width,
    )
    return gate, lower, upper


def _ramp_loop_kappa(
    s: float,
    ramp_length: float,
    transition_length: float,
    kappa_circle: float,
    sweep_length: float,
) -> float:
    s1 = ramp_length
    s2 = s1 + transition_length
    s3 = s2 + sweep_length
    s4 = s3 + transition_length
    if s <= s1:
        return 0.0
    if s <= s2:
        return kappa_circle * (s - s1) / transition_length
    if s <= s3:
        return kappa_circle
    if s <= s4:
        return kappa_circle * (1.0 - (s - s3) / transition_length)
    return 0.0


def build_ramp_loop_path(
    *,
    ramp_length: float,
    transition_length: float,
    circle_radius: float,
    sweep_deg: float,
    ds: float = 0.2,
) -> ScenarioPath:
    """Straight entry ramp -> clothoid-in -> constant-curvature circular
    sweep -> clothoid-out -> straight exit ramp, the shape of a parking-
    garage ramp that loops around before continuing on.  Curvature is
    integrated with a trapezoidal rule at a fixed arc-length step, which
    keeps consecutive yaw steps well inside ScenarioPath.from_arrays'
    tolerances even through the tightest part of the sweep.
    """
    kappa_circle = 1.0 / float(circle_radius)
    sweep_length = math.radians(float(sweep_deg)) * float(circle_radius)
    total_length = 2.0 * float(ramp_length) + 2.0 * float(transition_length) + sweep_length
    count = int(math.ceil(total_length / ds)) + 1
    s_values = np.linspace(0.0, total_length, count)
    kappa = np.array(
        [
            _ramp_loop_kappa(
                float(s), ramp_length, transition_length, kappa_circle, sweep_length
            )
            for s in s_values
        ]
    )
    yaw = np.zeros(count)
    x = np.zeros(count)
    y = np.zeros(count)
    for i in range(1, count):
        d = s_values[i] - s_values[i - 1]
        yaw[i] = yaw[i - 1] + 0.5 * (kappa[i - 1] + kappa[i]) * d
        mean_yaw = 0.5 * (yaw[i - 1] + yaw[i])
        x[i] = x[i - 1] + d * math.cos(mean_yaw)
        y[i] = y[i - 1] + d * math.sin(mean_yaw)
    mode = np.zeros(count, dtype=np.uint8)
    return ScenarioPath.from_arrays(x, y, yaw, kappa, mode, closed_loop=False)


def _straight_segment(
    x0: float, y0: float, heading: float, length: float, mode: int, *, ds: float = 0.2
) -> ScenarioPath:
    count = max(4, int(math.ceil(length / ds)) + 1)
    s = np.linspace(0.0, length, count)
    x = x0 + s * math.cos(heading)
    y = y0 + s * math.sin(heading)
    yaw = np.full(count, heading)
    kappa = np.zeros(count)
    mode_array = np.full(count, mode, dtype=np.uint8)
    return ScenarioPath.from_arrays(x, y, yaw, kappa, mode_array)


def _curved_segment(
    x0: float, y0: float, heading0: float, curvature: float, length: float, mode: int,
    *, ds: float = 0.2,
) -> ScenarioPath:
    count = max(4, int(math.ceil(length / ds)) + 1)
    s_values = np.linspace(0.0, length, count)
    yaw = heading0 + curvature * s_values
    x = np.empty(count)
    y = np.empty(count)
    x[0], y[0] = x0, y0
    for i in range(1, count):
        d = s_values[i] - s_values[i - 1]
        mean_yaw = 0.5 * (yaw[i - 1] + yaw[i])
        x[i] = x[i - 1] + d * math.cos(mean_yaw)
        y[i] = y[i - 1] + d * math.sin(mean_yaw)
    kappa = np.full(count, curvature)
    mode_array = np.full(count, mode, dtype=np.uint8)
    return ScenarioPath.from_arrays(x, y, yaw, kappa, mode_array)


def _concat_segments(segments: tuple[ScenarioPath, ...]) -> ScenarioPath:
    """Concatenate driving segments into one path.

    A spot-turn corner between two segments is simply the raw x/y/yaw
    concatenation -- ScenarioPath.from_arrays() recomputes curvature from
    (x, y, yaw) itself and recognizes a corner wherever that recomputed
    curvature is abnormally large, so no caller-side bookkeeping of where
    the corner sits is needed.
    """
    return ScenarioPath.from_arrays(
        np.concatenate([segment.x for segment in segments]),
        np.concatenate([segment.y for segment in segments]),
        np.concatenate([segment.yaw for segment in segments]),
        np.concatenate([segment.kappa for segment in segments]),
        np.concatenate([segment.mode for segment in segments]),
        map_yaw=np.concatenate([segment.map_yaw for segment in segments]),
        heading_semantics=segments[0].heading_semantics,
    )


def build_spot_turn_crab_course_phases(
    *,
    forward_leg_length: float = 36.0,
    crab_leg_length: float = 24.0,
    turn_degrees: tuple[float, float, float] = (40.0, -60.0, 70.0),
    curvature_after_turn1: float = 0.018,
    curvature_final_leg: float = -0.015,
    forward_speed: float = 1.5,
    crab_speed: float = 1.0,
    ds: float = 0.2,
) -> tuple["ScenarioPhase", ...]:
    """일반주행(중간에 제자리턴 내장) -> 크랩 -> 제자리턴 -> 주행 -> 크랩 ->
    제자리턴 -> 주행 -> 크랩 -> 그냥주행, 7-phase 코스.

    phase는 모드가 바뀔 때만 나뉜다 (forward/crab 다리마다 하나씩, 총 7개).
    세 제자리턴 중 -60도/70도는 crab -> forward 모드 전환과 겹치므로 phase
    경계 자체가 코너이고, 플래너(C++)는 그 경계에서 새 phase의 시작 헤딩과
    차량의 실측 헤딩을 비교해 판단한다. 반면 첫 제자리턴(40도)은 모드가
    바뀌지 않는 순수 헤딩 코너라 forward_1 phase 배열 "내부"에 곡률
    스파이크로 심겨 있다 -- 이어붙이는 지점의 좌표는 그대로 두고 헤딩만
    바꾸므로(_concat_segments), 그 지점의 순간곡률이 curvature_max를 훨씬
    넘는다. 플래너의 split_reference_path_at_corner()가 배열을 받을 때마다
    이 곡률 스파이크를 스스로 찾아 끊어 처리하므로, 시나리오 쪽에서 이
    코너의 위치를 따로 표시하거나 넘겨줄 필요가 없다.
    """
    turns = [math.radians(value) for value in turn_degrees]
    heading = 0.0
    x, y = 0.0, 0.0

    forward1a = _straight_segment(x, y, heading, forward_leg_length, mode=0, ds=ds)
    x, y = float(forward1a.x[-1]), float(forward1a.y[-1])
    heading += turns[0]
    forward1b = _curved_segment(x, y, heading, curvature_after_turn1, forward_leg_length, mode=0, ds=ds)
    heading = float(forward1b.yaw[-1])
    x, y = float(forward1b.x[-1]), float(forward1b.y[-1])
    forward1 = _concat_segments((forward1a, forward1b))

    crab1 = _straight_segment(x, y, heading + 0.5 * math.pi, crab_leg_length, mode=2, ds=ds)
    x, y = float(crab1.x[-1]), float(crab1.y[-1])

    heading += turns[1]
    forward2 = _straight_segment(x, y, heading, forward_leg_length, mode=0, ds=ds)
    x, y = float(forward2.x[-1]), float(forward2.y[-1])

    crab2 = _straight_segment(x, y, heading + 0.5 * math.pi, crab_leg_length, mode=2, ds=ds)
    x, y = float(crab2.x[-1]), float(crab2.y[-1])

    heading += turns[2]
    forward3 = _straight_segment(x, y, heading, forward_leg_length, mode=0, ds=ds)
    x, y = float(forward3.x[-1]), float(forward3.y[-1])

    crab3 = _straight_segment(x, y, heading + 0.5 * math.pi, crab_leg_length, mode=2, ds=ds)
    x, y = float(crab3.x[-1]), float(crab3.y[-1])

    forward4 = _curved_segment(x, y, heading, curvature_final_leg, forward_leg_length, mode=0, ds=ds)

    def _phase(name: str, path: ScenarioPath, cruise_speed: float, has_next: bool) -> "ScenarioPhase":
        return ScenarioPhase(
            name=name,
            path=path,
            cruise_speed=cruise_speed,
            switch_s=path.total_length if has_next else None,
        )

    return (
        _phase("forward_1", forward1, forward_speed, True),
        _phase("crab_1", crab1, crab_speed, True),
        _phase("forward_2_after_turn", forward2, forward_speed, True),
        _phase("crab_2", crab2, crab_speed, True),
        _phase("forward_3_after_turn", forward3, forward_speed, True),
        _phase("crab_3", crab3, crab_speed, True),
        _phase("forward_4_plain", forward4, forward_speed, False),
    )


def wall_segments_along_path(
    path: ScenarioPath,
    s_start: float,
    s_end: float,
    *,
    lateral_offset: float,
    segment_length: float = 1.6,
    spacing: float = 1.2,
    thickness: float = 0.25,
) -> list[ScenarioObstacle]:
    """Approximate a continuous wall alongside a (possibly curved) path
    segment as a sequence of short, overlapping rectangles.  Each segment
    picks up the local path tangent from obstacle_on_path, so the wall
    follows the curve instead of cutting a chord across it; the overlap
    between consecutive segments (segment_length > spacing) keeps the
    rasterized costmap from leaking a gap between them.
    """
    segments: list[ScenarioObstacle] = []
    s = float(s_start)
    end = float(s_end)
    while s <= end + 1.0e-6:
        segments.append(
            obstacle_on_path(
                path, s, lateral_offset=lateral_offset,
                length=segment_length, width=thickness, kind="wall",
            )
        )
        s += spacing
    return segments


def _clothoid_turn_segment(
    x0: float, y0: float, heading0: float, turn_angle: float, radius: float,
    transition_length: float, mode: int, *, ds: float = 0.2,
) -> ScenarioPath:
    """곡률이 0 -> ±1/radius로 선형 증가(완화곡선), 유지(원호), 다시 0으로 감소하는 회전.

    직선과 원호를 바로 붙이면 곡률이 계단식으로 튀므로 양 끝에 transition_length만큼
    완화곡선을 둔다. 샘플 격자가 세 구간 경계를 정확히 지나므로 총 헤딩 변화는 turn_angle과
    정확히 같다.
    """
    kappa_peak = math.copysign(1.0 / radius, turn_angle)
    arc_length = abs(turn_angle) * radius - transition_length
    if arc_length < 0.0:
        raise ValueError("Clothoid transition is longer than the turn allows")
    breaks = (0.0, transition_length, transition_length + arc_length,
              2.0 * transition_length + arc_length)
    s_values = np.concatenate([
        np.linspace(start, end, max(2, int(math.ceil((end - start) / ds)) + 1))[1 if i else 0:]
        for i, (start, end) in enumerate(zip(breaks, breaks[1:]))
    ])
    kappa = np.interp(s_values, breaks, (0.0, kappa_peak, kappa_peak, 0.0))
    yaw = heading0 + np.r_[0.0, np.cumsum(0.5 * (kappa[1:] + kappa[:-1]) * np.diff(s_values))]
    x = np.empty(len(s_values))
    y = np.empty(len(s_values))
    x[0], y[0] = x0, y0
    for i in range(1, len(s_values)):
        d = s_values[i] - s_values[i - 1]
        mean_yaw = 0.5 * (yaw[i - 1] + yaw[i])
        x[i] = x[i - 1] + d * math.cos(mean_yaw)
        y[i] = y[i - 1] + d * math.sin(mean_yaw)
    return ScenarioPath.from_arrays(x, y, yaw, kappa, np.full(len(x), mode, dtype=np.uint8))


def _drive_segments(
    x: float, y: float, heading: float, steps: tuple[tuple, ...], mode: int,
) -> list[ScenarioPath]:
    """("line", 길이) / ("turn", 각도deg, 원호반경, 완화곡선길이) 순서로 잇는 구간 목록.

    각도 + = 좌회전. 모든 연결점에서 곡률이 연속이다.
    """
    segments: list[ScenarioPath] = []
    for step in steps:
        if step[0] == "line":
            segment = _straight_segment(x, y, heading, step[1], mode)
        else:
            segment = _clothoid_turn_segment(
                x, y, heading, math.radians(step[1]), step[2], step[3], mode)
        segments.append(segment)
        x, y, heading = float(segment.x[-1]), float(segment.y[-1]), float(segment.yaw[-1])
    return segments


def _chain_segments(segments: list[ScenarioPath]) -> ScenarioPath:
    """연속 주행 구간들을 한 경로로 잇는다 (다음 구간 첫 점 = 앞 구간 끝 점이므로 중복점 제거)."""
    return ScenarioPath.from_arrays(
        np.concatenate([segments[0].x] + [segment.x[1:] for segment in segments[1:]]),
        np.concatenate([segments[0].y] + [segment.y[1:] for segment in segments[1:]]),
        np.concatenate([segments[0].yaw] + [segment.yaw[1:] for segment in segments[1:]]),
        np.concatenate([segments[0].kappa] + [segment.kappa[1:] for segment in segments[1:]]),
        np.concatenate([segments[0].mode] + [segment.mode[1:] for segment in segments[1:]]),
    )


def _box(
    x_min: float, y_min: float, x_max: float, y_max: float, kind: str = "pallet",
) -> ScenarioObstacle:
    return ScenarioObstacle(
        x=0.5 * (x_min + x_max), y=0.5 * (y_min + y_max),
        length=x_max - x_min, width=y_max - y_min, kind=kind,
    )


def _offset_points(
    path: ScenarioPath, lateral_offset: float, s_start: float, s_end: float,
    *, step: float = 1.0, close: bool = False,
) -> tuple[tuple[float, float], ...]:
    """경로를 진행 방향 왼쪽으로 lateral_offset만큼 평행이동한 폴리라인 (오른쪽은 음수).

    close=True면 마지막 점을 첫 점과 정확히 같게 둬서 닫힌 띠/점선이 이음매 없이 그려진다.
    """
    s_values = np.linspace(s_start, s_end, max(2, int(math.ceil((s_end - s_start) / step)) + 1))
    yaw = np.interp(s_values, path.s, np.unwrap(path.yaw))
    x = np.interp(s_values, path.s, path.x) - lateral_offset * np.sin(yaw)
    y = np.interp(s_values, path.s, path.y) + lateral_offset * np.cos(yaw)
    points = list(zip(x.tolist(), y.tolist()))
    if close:
        points[-1] = points[0]
    return tuple(points)


def _rect_points(
    x_min: float, y_min: float, x_max: float, y_max: float,
) -> tuple[tuple[float, float], ...]:
    return ((x_min, y_min), (x_max, y_min), (x_max, y_max), (x_min, y_max))


def _hatch_markings(
    x_min: float, y_min: float, x_max: float, y_max: float, spacing: float, color: str,
) -> list[FloorMarking]:
    """사각형 안을 45도 빗금으로 채우는 선들 (x - y = offset)."""
    markings: list[FloorMarking] = []
    offset = x_min - y_max + spacing
    while offset < x_max - y_min:
        y_start, y_end = max(y_min, x_min - offset), min(y_max, x_max - offset)
        if y_end > y_start:
            markings.append(FloorMarking(
                "line", ((y_start + offset, y_start), (y_end + offset, y_end)), color, 0.12))
        offset += spacing
    return markings


def _convex_hull(points) -> tuple[tuple[float, float], ...]:
    """점들의 볼록 껍질 (반시계 방향, Andrew monotone chain)."""
    ordered = sorted(set((float(x), float(y)) for x, y in points))

    def cross(o, a, b) -> float:
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower: list[tuple[float, float]] = []
    upper: list[tuple[float, float]] = []
    for point in ordered:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], point) <= 0.0:
            lower.pop()
        lower.append(point)
    for point in reversed(ordered):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], point) <= 0.0:
            upper.pop()
        upper.append(point)
    return tuple(lower[:-1] + upper[:-1])


def build_warehouse_logistics_scenario() -> "ScenarioDefinition":
    """물류센터 한 층 (도면 치수 기준). 진입램프(바깥 폭 약 37 m) / 사무실(10 m) / 하역장(폭 110 m =
    기둥 11 m 간격 10칸, 깊이 12 m) / 창고(10 m) / 진출램프가 서->동으로 놓이고, 북쪽 일방통로
    (폭 17 m)가 두 램프를 잇는다. 램프 바깥벽이 곧 건물 벽이다. 글자/화살표 표시는 두지 않는다.

    램프: 세로로 긴 타원 트랙(직선 - 완화곡선 - 원호 - 완화곡선), 2차선, 시계 방향. 차량은 바깥
    차선(바깥벽 쪽 가깝게, 안쪽 코어벽 쪽 멀게)을 달린다. 램프 꼭대기의 90도 회전이 끝나는
    높이가 곧 통로 주행선이라 통로 구간은 일자로 곧다. 램프 꼭대기는 북쪽 벽보다 낮으므로,
    통로 반대쪽 위 사분면의 바깥벽만 북쪽 벽 높이까지 끌어올려(직선 연장 + 같은 곡선) 북쪽
    벽과 이어 붙였다. 통로 쪽 옆면은 통로 남쪽 벽(사무실/창고 윗벽)에 닿을 때까지 이어진다.

      1 FORWARD  진입램프 동쪽 직선에서 출발, 270도 돌아 통로로 곧게 -> 도크 D4 앞
      2 FORWARD  제자리턴(동->남) -> 하역장을 가로질러 D4 도크 면까지 19 m 정면 도킹
      3 REVERSE  통로까지 후진
      4 FORWARD  제자리턴(남->동) -> D7 앞까지 33 m
      5 RIGHT    크랩(차체는 동쪽 그대로)으로 D7 도크 면까지 측면 도킹
      6 LEFT     크랩으로 통로 복귀
      7 FORWARD  통로 장애물 3개(북-남-북) 회피 -> 진출램프 바깥 차선으로 270도

    경로 곡률은 전 구간 연속이다(직선과 회전 사이마다 완화곡선). 제자리턴은 phase 경계에서만
    일어난다. 여유거리(차량 충돌원 3개, 반경 1.118 m 기준): 램프 바깥벽 0.88 m, 코어벽 약 4.9 m,
    도킹 정지 시 도크 면 0.88 m. 회피 대상은 마지막 phase의 통로 장애물 3개뿐이다.
    """
    north, south, east = 0.5 * math.pi, -0.5 * math.pi, 0.0
    circle_reach = math.hypot(0.5, 1.0)              # 플래너 충돌원 반경 (3원, 3.0 x 2.0 m)

    # 램프 트랙 치수는 "바깥 차선 중심 = 주행선" 기준이다.
    lane_width = 4.0
    ramp_radius, ramp_transition, ramp_straight = 16.0, 8.0, 8.0
    outer_face = 0.5 * lane_width                    # 주행선 -> 바깥벽 안쪽면 (왼쪽)
    divider = -0.5 * lane_width                      # 주행선 -> 차선 경계 점선
    core_face = -1.5 * lane_width                    # 주행선 -> 코어벽 바깥면
    wall = 0.3
    aisle_north_gap, aisle_south_gap = 7.0, 10.0     # 통로 주행선 -> 북쪽 벽 / 남쪽 경계
    bay_pitch, bay_count, loading_depth = 11.0, 10, 12.0   # 기둥 간격 / 도크 칸 수 / 하역장 깊이

    half_turn = _clothoid_turn_segment(0.0, 0.0, 0.0, -math.pi, ramp_radius, ramp_transition, 0)
    track_half_width = -0.5 * float(half_turn.y[-1])  # 트랙 중심 -> 동/서 직선 주행선
    track_reach = float(np.max(half_turn.x))         # 직선 끝 -> 꼭대기(바닥) 주행선
    # 꼭대기 90도 회전은 좌우 대칭이라 전진량 = 횡이동량
    quarter_turn = _clothoid_turn_segment(
        0.0, 0.0, 0.0, -0.5 * math.pi, ramp_radius, ramp_transition, 0)
    quarter_turn_offset = float(quarter_turn.x[-1])
    ramp_half_width = track_half_width + outer_face + wall

    ramp_center_y = 0.5 * ramp_straight + track_reach + outer_face + wall + 0.5
    aisle_y = ramp_center_y + 0.5 * ramp_straight + quarter_turn_offset   # 통로 주행선
    height = aisle_y + aisle_north_gap               # 북쪽 벽 안쪽면
    aisle_bottom_y = aisle_y - aisle_south_gap
    dock_face_y = aisle_bottom_y - loading_depth
    dock_stop_y = dock_face_y + 3.0                  # 정면 도킹: 앞 충돌원 ~ 도크 면 0.88 m
    crab_stop_y = dock_face_y + 2.0                  # 크랩 도킹: 옆 충돌원 ~ 도크 면 0.88 m
    wall_lift = height - (ramp_center_y + 0.5 * ramp_straight + track_reach + outer_face)
    if wall_lift <= 0.0:
        raise ValueError("Ramp top must stay below the aisle north wall")

    entry_center = (ramp_half_width + 0.5, ramp_center_y)
    room_left_x = entry_center[0] + ramp_half_width + wall    # 사무실 서쪽 벽
    loading_x0 = room_left_x + 10.0
    loading_x1 = loading_x0 + bay_count * bay_pitch
    room_right_x = loading_x1 + 10.0                 # 창고 동쪽 벽
    exit_center = (room_right_x + wall + ramp_half_width, ramp_center_y)
    dock_xs = tuple(loading_x0 + (k + 0.5) * bay_pitch for k in range(bay_count))   # D1~D10
    first_dock, second_dock = 3, 6                   # D4 정면 도킹, D7 크랩 도킹 (0부터 센 칸 번호)
    first_dock_x, second_dock_x = dock_xs[first_dock], dock_xs[second_dock]
    merge_dx = quarter_turn_offset - track_half_width   # 램프 중심 -> 꼭대기 회전 끝 x

    def track_loop(center: tuple[float, float]) -> ScenarioPath:
        """주행선 한 바퀴. 동쪽 직선 가운데에서 남쪽을 보고 출발 (시계 방향)."""
        return _chain_segments(_drive_segments(
            center[0] + track_half_width, center[1], south,
            (("line", 0.5 * ramp_straight),
             ("turn", -180.0, ramp_radius, ramp_transition),
             ("line", ramp_straight),
             ("turn", -180.0, ramp_radius, ramp_transition),
             ("line", 0.5 * ramp_straight)), 0))

    entry_route = _chain_segments(_drive_segments(
        entry_center[0] + track_half_width, entry_center[1], south,
        (("line", 0.5 * ramp_straight),
         ("turn", -180.0, ramp_radius, ramp_transition),
         ("line", ramp_straight),
         ("turn", -90.0, ramp_radius, ramp_transition),
         ("line", first_dock_x - entry_center[0] - merge_dx)), 0))

    def dock_in(x: float) -> ScenarioPath:
        return _straight_segment(x, aisle_y, south, aisle_y - dock_stop_y, 0)

    def dock_out(x: float) -> ScenarioPath:
        # 이동 방향은 북쪽, 차체(map_yaw)는 남쪽 그대로
        path = _straight_segment(x, dock_stop_y, north, aisle_y - dock_stop_y, 1)
        return ScenarioPath.from_arrays(
            path.x, path.y, path.yaw, path.kappa, path.mode,
            map_yaw=np.full(len(path.x), south),
            heading_semantics="MAP_HEADING_PLUS_MODE_OFFSET",
        )

    aisle_to_second_dock = _straight_segment(
        first_dock_x, aisle_y, east, second_dock_x - first_dock_x, 0)
    # 크랩: 차체는 동쪽을 본 채로 오른쪽(남쪽)으로 들어갔다가 왼쪽(북쪽)으로 나온다
    crab_in = _straight_segment(second_dock_x, aisle_y, south, aisle_y - crab_stop_y, 3)
    crab_out = _straight_segment(second_dock_x, crab_stop_y, north, aisle_y - crab_stop_y, 2)
    # 진출: 진입 꼭대기 회전의 좌우 대칭 -> 진출램프 동쪽 직선 위끝에 정확히 닿는다
    exit_route = _chain_segments(_drive_segments(
        second_dock_x, aisle_y, east,
        (("line", exit_center[0] - merge_dx - second_dock_x),
         ("turn", -90.0, ramp_radius, ramp_transition),
         ("line", ramp_straight),
         ("turn", -180.0, ramp_radius, ramp_transition),
         ("line", 0.5 * ramp_straight)), 0))

    # 목표 속도 (m/s): 일반 주행 / 램프 / 도킹(진입·복귀)
    drive_speed, ramp_speed, docking_speed = 2.8, 2.3, 0.9
    ramp_slowdown_lead = 5.0                         # 진출램프 회전 시작 전 미리 감속하는 거리
    ramp_length = (0.5 * ramp_straight + half_turn.total_length + ramp_straight
                   + quarter_turn.total_length)      # 램프 동쪽 직선 중간 ~ 꼭대기 90도 회전 끝
    exit_turn_s = exit_center[0] - merge_dx - second_dock_x
    entry_ramp_zone = ((0.0, ramp_length, ramp_speed),)
    exit_ramp_zone = ((exit_turn_s - ramp_slowdown_lead, exit_route.total_length, ramp_speed),)

    def phase(name: str, path: ScenarioPath, speed: float, last: bool = False,
              zones: tuple[tuple[float, float, float], ...] = ()) -> ScenarioPhase:
        return ScenarioPhase(name=name, path=path, cruise_speed=speed,
                             switch_s=None if last else path.total_length, speed_zones=zones)

    phases = (
        phase("entry_ramp_to_dock_d4", entry_route, drive_speed, zones=entry_ramp_zone),
        phase("dock_d4_in", dock_in(first_dock_x), docking_speed),
        phase("dock_d4_out_reverse", dock_out(first_dock_x), docking_speed),
        phase("aisle_to_dock_d7", aisle_to_second_dock, drive_speed),
        phase("crab_dock_d7_in_right", crab_in, docking_speed),
        phase("crab_dock_d7_out_left", crab_out, docking_speed),
        phase("aisle_obstacles_to_exit_ramp", exit_route, drive_speed, last=True,
              zones=exit_ramp_zone),
    )

    def junction(loop: ScenarioPath, s_start: float, s_end: float) -> tuple[float, float]:
        """[s_start, s_end]에서 바깥벽 안쪽면이 통로 남쪽 벽 높이를 지나는 (s, x)."""
        face = _offset_points(loop, outer_face, s_start, s_end, step=0.05)
        index = int(np.argmin([abs(y - aisle_bottom_y) for _, y in face]))
        return s_start + (s_end - s_start) * index / (len(face) - 1), face[index][0]

    walls = []
    markings = []
    junctions = {}
    for name, center in (("entry", entry_center), ("exit", exit_center)):
        loop = track_loop(center)
        lifted = loop.translated(0.0, wall_lift)
        total = loop.total_length
        apex_s = float(loop.s[int(np.argmax(loop.y))])
        west_top_s = 0.5 * ramp_straight + half_turn.total_length + ramp_straight
        east_top_s = total - 0.5 * ramp_straight
        if name == "entry":   # 통로는 동쪽: 동쪽 위 사분면이 열리고 서쪽 위 사분면을 들어 올린다
            junction_s, junctions[name] = junction(loop, apex_s, total)
            walled = ((0.0, west_top_s), (junction_s, total))
            lifted_range = (west_top_s, apex_s)
            lane_edges = ((0.0, apex_s), (junction_s, total))
            corner_x = center[0] - track_half_width - outer_face - 0.5 * wall
        else:                 # 통로는 서쪽: 서쪽 위 사분면이 열리고 동쪽 위 사분면을 들어 올린다
            junction_s, junctions[name] = junction(loop, west_top_s, apex_s)
            walled = ((0.0, junction_s), (east_top_s, total))
            lifted_range = (apex_s, east_top_s)
            lane_edges = ((0.0, junction_s), (apex_s, total))
            corner_x = center[0] + track_half_width + outer_face + 0.5 * wall
        for start, end in walled:
            walls += wall_segments_along_path(loop, start, end, lateral_offset=outer_face + 0.15,
                                              thickness=wall)
        walls += wall_segments_along_path(lifted, *lifted_range,
                                          lateral_offset=outer_face + 0.15, thickness=wall)
        straight_top_y = center[1] + 0.5 * ramp_straight
        walls.append(_box(corner_x - 0.5 * wall, straight_top_y - 0.6,
                          corner_x + 0.5 * wall, straight_top_y + wall_lift + 0.6, "wall"))
        walls += wall_segments_along_path(loop, 0.0, total,
                                          lateral_offset=core_face - 0.15, thickness=wall)

        markings += [
            FloorMarking("area", _convex_hull(
                _offset_points(loop, outer_face + wall, 0.0, total)
                + _offset_points(lifted, outer_face + wall, *lifted_range)), "floor"),
            FloorMarking("line", _offset_points(loop, divider, 0.0, total, close=True),
                         "ramp", 2.0 * lane_width),
            FloorMarking("area", _offset_points(loop, core_face, 0.0, total), "core"),
            FloorMarking("line", _offset_points(loop, core_face + 0.25, 0.0, total, close=True),
                         "white"),
            FloorMarking("dashed", _offset_points(loop, divider, 0.0, total, close=True), "white"),
        ]
        markings += [FloorMarking("line", _offset_points(loop, outer_face - 0.25, start, end),
                                  "white") for start, end in lane_edges]

    walls += [
        _box(entry_center[0] - 0.5, height, exit_center[0] + 0.5, height + wall, "wall"),  # 북
        # 통로 남쪽 벽: 램프 바깥벽 끝 ~ 사무실 윗벽 / 창고 윗벽 ~ 진출램프 바깥벽 끝
        _box(junctions["entry"] - wall, aisle_bottom_y - wall, loading_x0, aisle_bottom_y,
             "wall"),
        _box(loading_x1, aisle_bottom_y - wall, junctions["exit"] + wall, aisle_bottom_y,
             "wall"),
        _box(room_left_x - wall, dock_face_y - 0.4, room_left_x, aisle_bottom_y, "wall"),
        _box(room_right_x, dock_face_y - 0.4, room_right_x + wall, aisle_bottom_y, "wall"),
        _box(room_left_x - wall, dock_face_y - 0.4, room_right_x + wall, dock_face_y,
             "wall"),                                                          # 도크 면
        _box(loading_x0 - wall, dock_face_y, loading_x0, aisle_bottom_y, "wall"),  # 사무실
        _box(loading_x1, dock_face_y, loading_x1 + wall, aisle_bottom_y, "wall"),  # 창고
    ]
    # 하역장 입구 기둥 (도크 칸 경계마다, 1.2 m 각)
    pillar_half = 0.6
    walls += [_box(loading_x0 + bay_pitch * k - pillar_half, aisle_bottom_y - pillar_half,
                   loading_x0 + bay_pitch * k + pillar_half, aisle_bottom_y + pillar_half, "wall")
              for k in range(bay_count + 1)]

    # 상차 대기 화물 (사용하는 도크와 그 양옆 칸을 뺀 도크 앞)
    cargo = [_box(x - 1.2, dock_face_y + 0.6, x + 1.2, dock_face_y + 3.0)
             for k, x in enumerate(dock_xs)
             if min(abs(k - first_dock), abs(k - second_dock)) > 1]

    def blocking(x: float, side: float, length: float, width: float, yaw_deg: float,
                 intrusion: float) -> ScenarioObstacle:
        """통로 주행선 side쪽(+1 북 / -1 남)에 놓되 차량 충돌원을 intrusion만큼 침범하는 장애물."""
        yaw = math.radians(yaw_deg)
        half_y = 0.5 * (length * abs(math.sin(yaw)) + width * abs(math.cos(yaw)))
        return ScenarioObstacle(x=x, y=aisle_y + side * (circle_reach - intrusion + half_y),
                                length=length, width=width, yaw=yaw)

    # 마지막 phase 회피 대상: 북 - 남 - 북 순서라 차량이 남/북/남으로 번갈아 비켜 간다.
    # 첫 번째는 진로를 조금만(0.3 m) 막아 여유 있게 비켜 가도록 한다.
    avoidance = [
        blocking(second_dock_x + 14.0, +1.0, 3.0, 2.4, 12.0, intrusion=0.3),
        blocking(second_dock_x + 28.0, -1.0, 4.2, 2.4, -18.0, intrusion=0.3),
        blocking(second_dock_x + 42.0, +1.0, 3.0, 3.0, 25.0, intrusion=0.8),
    ]

    markings += [
        FloorMarking("area", _rect_points(entry_center[0], aisle_bottom_y,
                                          exit_center[0], height), "floor"),
        FloorMarking("area", _rect_points(room_left_x, dock_face_y, room_right_x, aisle_bottom_y),
                     "floor"),
        FloorMarking("area", _rect_points(loading_x0, dock_face_y, loading_x1, aisle_bottom_y),
                     "loading"),
        FloorMarking("area", _rect_points(room_left_x, dock_face_y, loading_x0, aisle_bottom_y),
                     "room"),
        FloorMarking("area", _rect_points(loading_x1, dock_face_y, room_right_x, aisle_bottom_y),
                     "room"),
    ]
    markings += _hatch_markings(loading_x0, dock_face_y + 0.8, loading_x1, aisle_bottom_y,
                                2.0, "hatch")  # 레벨러 띠 위부터
    markings += [FloorMarking("area", _rect_points(x - 1.2, dock_face_y, x + 1.2, dock_face_y + 0.8),
                              "green") for x in dock_xs]

    return ScenarioDefinition(
        name="warehouse_logistics",
        phases=phases,
        obstacles=tuple(walls + cargo + avoidance),
        terminal_margin=3.0,
        costmap_resolution=0.1,
        markings=tuple(markings),
    )


def load_scenario_definition(
    share_directory: Path | str,
    scenario_name: str,
) -> ScenarioDefinition:
    share_directory = Path(share_directory)
    name = str(scenario_name).strip().lower()
    aliases = {
        "narrow_10pct_corridor": "narrow_22m_stop_corridor",
        "narrow_28m_coarse_corridor": "narrow_28m_corridor",
    }
    name = aliases.get(name, name)

    if name == "stadium":
        phase = ScenarioPhase(
            name="stadium_lap",
            path=ScenarioPath.load_csv(
                _map_path(share_directory, "stadium_track.csv"),
                closed_loop=True,
            ),
            cruise_speed=2.0,
        )
        return ScenarioDefinition(
            name=name,
            phases=(phase,),
            repeat=True,
        )

    if name == "crab_switch":
        forward = ScenarioPhase(
            name="regular_forward",
            path=ScenarioPath.load_csv(
                _map_path(share_directory, "crab_forward.csv"),
                closed_loop=False,
            ),
            cruise_speed=1.5,
            switch_s=18.0,
        )
        crab = ScenarioPhase(
            name="left_crab",
            path=ScenarioPath.load_csv(
                _map_path(share_directory, "crab_left.csv"),
                closed_loop=False,
            ),
            cruise_speed=1.0,
        )
        return ScenarioDefinition(
            name=name,
            phases=(forward, crab),
            terminal_margin=3.0,
        )


    if name == "spot_turn_course":
        phases = build_spot_turn_crab_course_phases()
        return ScenarioDefinition(
            name=name,
            phases=phases,
            terminal_margin=3.0,
        )


    if name == "reverse_switch":
        forward = ScenarioPhase(
            name="regular_forward_stop",
            path=ScenarioPath.load_csv(
                _map_path(share_directory, "crab_forward.csv"),
                closed_loop=False,
                mode_aware_heading=True,
            ),
            cruise_speed=1.5,
            switch_s=18.0,
        )
        reverse = ScenarioPhase(
            name="reverse_departure",
            path=ScenarioPath.load_csv(
                _map_path(share_directory, "reverse_departure.csv"),
                closed_loop=False,
                mode_aware_heading=True,
            ),
            cruise_speed=1.0,
        )
        return ScenarioDefinition(
            name=name,
            phases=(forward, reverse),
            terminal_margin=3.0,
        )

    if name == "hdmap_crab1_switch":
        # HDMap/build_scenario_maps.py 로 생성한, 실제 HD map(큰트랙 + 크랩1)
        # 기반 3-phase 시나리오: 전진(Forward) -> 크랩 이탈(Left) -> 정지 후
        # 같은 경로를 되짚어 복귀(Right). switch_s=50.0은
        # HDMap/output/switch_stations.txt 의 크랩1 S_switch를
        # hdmap_crab1_forward.csv 자신의 로컬 원점 기준으로 다시 잰 값이다.
        forward = ScenarioPhase(
            name="hdmap_forward",
            path=ScenarioPath.load_csv(
                _map_path(share_directory, "hdmap_crab1_forward.csv"),
                closed_loop=False,
            ),
            cruise_speed=1.5,
            switch_s=50.0,
        )
        left = ScenarioPhase(
            name="hdmap_crab1_left",
            path=ScenarioPath.load_csv(
                _map_path(share_directory, "hdmap_crab1_left.csv"),
                closed_loop=False,
            ),
            cruise_speed=1.0,
        )
        right_return = ScenarioPhase(
            name="hdmap_crab1_right_return",
            path=ScenarioPath.load_csv(
                _map_path(share_directory, "hdmap_crab1_right_return.csv"),
                closed_loop=False,
            ),
            cruise_speed=1.0,
        )
        return ScenarioDefinition(
            name=name,
            phases=(forward, left, right_return),
            terminal_margin=3.0,
        )

    if name == "fmtc_demo":
        # HDMap/build_ref_ver1_path.py + HDMap/build_fmtc_demo_scenario.py 로
        # 생성한, ref_ver1.txt 기반 3-phase 시나리오: 레귤러1(Forward) ->
        # 크랩2(Left) -> 레귤러3(Forward). 레귤러1/크랩2는 각각 자신의 곡선에
        # 다음 세그먼트 시작점을 수직 투영해 얻은 S_switch에서 정확히 잘려
        # 있으므로, switch_s는 곧 그 phase csv 자신의 총 길이다
        # (HDMap/build_fmtc_demo_scenario.py 콘솔 출력 그대로).
        regular1 = ScenarioPhase(
            name="fmtc_demo_regular1",
            path=ScenarioPath.load_csv(
                _map_path(share_directory, "fmtc_demo_regular1.csv"),
                closed_loop=False,
            ),
            cruise_speed=1.5,
            switch_s=94.81,
        )
        crab2 = ScenarioPhase(
            name="fmtc_demo_crab2",
            path=ScenarioPath.load_csv(
                _map_path(share_directory, "fmtc_demo_crab2.csv"),
                closed_loop=False,
            ),
            cruise_speed=1.0,
            switch_s=152.53,
        )
        regular3 = ScenarioPhase(
            name="fmtc_demo_regular3",
            path=ScenarioPath.load_csv(
                _map_path(share_directory, "fmtc_demo_regular3.csv"),
                closed_loop=False,
            ),
            cruise_speed=1.5,
        )
        return ScenarioDefinition(
            name=name,
            phases=(regular1, crab2, regular3),
            terminal_margin=3.0,
        )

    if name == "hdmap_lap_switch":
        # HDMap/build_lap_scenario_maps.py 로 생성. 큰트랙을 따라가며 크랩
        # 1/2/3 switch를 순서대로 만나는 10-phase 연속 랩:
        # forward0 -> crab1_left -> crab1_right_return ->
        # forward1 -> crab2_left -> crab2_right_return ->
        # forward2 -> crab3_left -> crab3_right_return ->
        # forward3 (크랩3 switch ~ 큰트랙 끝, 마지막 phase). switch_s 값은
        # build_lap_scenario_maps.py 콘솔 출력 그대로다. 전 phase가 큰트랙
        # 자신의 시작점을 공통 원점으로 평행이동되어 있어 authored 상태로
        # 이미 서로 이어진다.
        def _lap_phase(csv_name, phase_name, cruise_speed, switch_s=None):
            return ScenarioPhase(
                name=phase_name,
                path=ScenarioPath.load_csv(
                    _map_path(share_directory, f"{csv_name}.csv"),
                    closed_loop=False,
                ),
                cruise_speed=cruise_speed,
                switch_s=switch_s,
            )

        phases = (
            _lap_phase("hdmap_lap_forward0", "hdmap_lap_forward0", 1.5, switch_s=279.19),
            _lap_phase("hdmap_lap_crab1_left", "hdmap_lap_crab1_left", 1.0),
            _lap_phase("hdmap_lap_crab1_right_return", "hdmap_lap_crab1_right_return", 1.0),
            _lap_phase("hdmap_lap_forward1", "hdmap_lap_forward1", 1.5, switch_s=183.79),
            _lap_phase("hdmap_lap_crab2_left", "hdmap_lap_crab2_left", 1.0),
            _lap_phase("hdmap_lap_crab2_right_return", "hdmap_lap_crab2_right_return", 1.0),
            _lap_phase("hdmap_lap_forward2", "hdmap_lap_forward2", 1.5, switch_s=226.77),
            _lap_phase("hdmap_lap_crab3_left", "hdmap_lap_crab3_left", 1.0),
            _lap_phase("hdmap_lap_crab3_right_return", "hdmap_lap_crab3_right_return", 1.0),
            _lap_phase("hdmap_lap_forward3", "hdmap_lap_forward3", 1.5),
        )
        return ScenarioDefinition(
            name=name,
            phases=phases,
            terminal_margin=3.0,
        )

    if name == "s_curve":
        phase = ScenarioPhase(
            name="s_curve_forward",
            path=ScenarioPath.load_csv(
                _map_path(share_directory, "s_curve.csv"),
                closed_loop=False,
            ),
            cruise_speed=2.0,
        )
        return ScenarioDefinition(name=name, phases=(phase,))

    if name == "straight_long":
        # A pure 1 km straight reference path with no obstacles or gates,
        # for sustained high-speed cruise/tracking checks without any
        # avoidance maneuver in the way.
        phase = ScenarioPhase(
            name="straight_long_cruise",
            path=ScenarioPath.load_csv(
                _map_path(share_directory, "straight_long.csv"),
                closed_loop=False,
            ),
            cruise_speed=2.0,
        )
        return ScenarioDefinition(name=name, phases=(phase,))


    if name == "s_curve_obstacles":
        path = ScenarioPath.load_csv(
            _map_path(share_directory, "s_curve_obstacles.csv"),
            closed_loop=False,
        )
        phase = ScenarioPhase(
            name="s_curve_three_obstacles",
            path=path,
            cruise_speed=1.2,
        )
        obstacles = (
            obstacle_on_path(path, 16.0, lateral_offset=0.25),
            obstacle_on_path(path, 38.0, lateral_offset=-0.25),
            obstacle_on_path(path, 58.0, lateral_offset=0.25),
        )
        return ScenarioDefinition(
            name=name,
            phases=(phase,),
            obstacles=obstacles,
        )

    if name == "obstacle_avoidance":
        phase = ScenarioPhase(
            name="straight_obstacle_avoidance",
            path=ScenarioPath.load_csv(
                _map_path(share_directory, "obstacle_straight.csv"),
                closed_loop=False,
            ),
            cruise_speed=1.5,
        )
        obstacle = ScenarioObstacle(
            x=35.0,
            y=0.0,
            length=4.0,
            width=2.5,
            yaw=0.0,
        )
        return ScenarioDefinition(
            name=name,
            phases=(phase,),
            obstacles=(obstacle,),
        )


    if name == "terminal_safe_region":
        path = ScenarioPath.load_csv(
            _map_path(share_directory, "obstacle_straight.csv"),
            closed_loop=False,
        )
        phase = ScenarioPhase(
            name="terminal_center_blocked_safe_offset_stop",
            path=path,
            cruise_speed=1.0,
        )
        # The validator clips the 80 m authored path at s=77 m.  The nominal
        # longitudinal stop target is therefore near x=76.94 m.  A centreline
        # obstacle blocks the exact reference endpoint while leaving both
        # lateral sides open.  The planner must preserve the longitudinal stop
        # coordinate and choose the smallest collision-free lateral offset.
        obstacle = ScenarioObstacle(
            x=76.95,
            y=0.0,
            length=3.0,
            width=2.0,
            yaw=0.0,
        )
        return ScenarioDefinition(
            name=name,
            phases=(phase,),
            obstacles=(obstacle,),
            terminal_margin=3.0,
        )


    if name == "alternating_gate_corridor":
        path = ScenarioPath.load_csv(
            _map_path(share_directory, "alternating_gate_corridor.csv"),
            closed_loop=False,
        )
        phase = ScenarioPhase(
            name="alternating_five_gate_corridor",
            path=path,
            cruise_speed=1.5,
        )
        gate_specs = (
            (30.0, 3.0),
            (65.0, -3.0),
            (100.0, 3.0),
            (135.0, -3.0),
            (170.0, 2.75),
        )
        gates: list[ScenarioGate] = []
        obstacles: list[ScenarioObstacle] = []
        for s_position, center in gate_specs:
            gate, lower, upper = gate_on_path(
                path, s_position, lateral_center=center,
                gap_width=4.8, barrier_extent=14.0, obstacle_length=5.0,
            )
            gates.append(gate)
            obstacles.extend((lower, upper))
        return ScenarioDefinition(
            name=name, phases=(phase,), obstacles=tuple(obstacles), gates=tuple(gates)
        )

    if name == "curved_gate_maze":
        path = ScenarioPath.load_csv(
            _map_path(share_directory, "curved_gate_maze.csv"),
            closed_loop=False,
        )
        phase = ScenarioPhase(
            name="curved_five_gate_maze",
            path=path,
            cruise_speed=1.6,
        )
        gate_specs = (
            (32.0, -2.5),
            (66.0, 2.5),
            (100.0, -2.5),
            (134.0, 2.5),
            (168.0, -2.25),
        )
        gates: list[ScenarioGate] = []
        obstacles: list[ScenarioObstacle] = []
        for s_position, center in gate_specs:
            gate, lower, upper = gate_on_path(
                path, s_position, lateral_center=center,
                gap_width=5.0, barrier_extent=14.0, obstacle_length=5.0,
            )
            gates.append(gate)
            obstacles.extend((lower, upper))
        return ScenarioDefinition(
            name=name, phases=(phase,), obstacles=tuple(obstacles), gates=tuple(gates)
        )



    if name == "narrow_22m_stop_corridor":
        path = ScenarioPath.load_csv(
            _map_path(share_directory, "narrow_22m_stop_corridor.csv"),
            closed_loop=False,
        )
        phase = ScenarioPhase(
            name="infeasible_2p2m_corridor_stop_only",
            path=path,
            cruise_speed=0.8,
        )

        # Vehicle width is 2.0 m.  The 2.2 m opening therefore provides only
        # 10% total width margin (0.10 m per side under perfect centering).
        # The 20 m long barriers form a true corridor rather than a thin gate.
        gate, lower, upper = gate_on_path(
            path, 25.0, lateral_center=0.0, gap_width=2.20,
            barrier_extent=8.0, obstacle_length=20.0,
        )
        return ScenarioDefinition(
            name=name, phases=(phase,), obstacles=(lower, upper), gates=(gate,),
            terminal_margin=3.0,
        )


    if name == "narrow_28m_corridor":
        path = ScenarioPath.load_csv(
            _map_path(share_directory, "narrow_28m_corridor.csv"),
            closed_loop=False,
        )
        phase = ScenarioPhase(
            name="long_2p8m_corridor_with_coarse_costmap",
            path=path,
            cruise_speed=0.6,
        )

        # Fixed deployment specification: 0.20 m costmap and the nominal
        # 3-circle oriented footprint.  The physical corridor is 2.80 m wide
        # and placed at a half-cell grid phase (global y=0.10 m), while the
        # reference path remains about 0.25 m above the corridor centre with a
        # small heading mismatch.  Resolution and footprint settings are not
        # relaxed for this scenario.
        corridor_x = 65.0
        corridor_y = 0.10
        corridor_length = 70.0
        gap_width = 2.80
        barrier_extent = 8.0
        half_gap = 0.5 * gap_width
        wall_width = barrier_extent - half_gap
        lower = ScenarioObstacle(
            x=corridor_x,
            y=corridor_y - 0.5 * (barrier_extent + half_gap),
            length=corridor_length, width=wall_width, yaw=0.0,
        )
        upper = ScenarioObstacle(
            x=corridor_x,
            y=corridor_y + 0.5 * (barrier_extent + half_gap),
            length=corridor_length, width=wall_width, yaw=0.0,
        )
        gate_s = float(np.interp(corridor_x, path.x, path.s))
        path_y = float(np.interp(gate_s, path.s, path.y))
        path_yaw = float(np.interp(gate_s, path.s, np.unwrap(path.yaw)))
        gate = ScenarioGate(
            s=gate_s,
            lateral_center=(corridor_y - path_y) * math.cos(path_yaw),
            gap_width=gap_width,
            barrier_extent=barrier_extent,
            obstacle_length=corridor_length,
        )
        return ScenarioDefinition(
            name=name, phases=(phase,), obstacles=(lower, upper), gates=(gate,),
            terminal_margin=3.0,
            costmap_resolution=0.20,
            footprint_circle_count=3,
        )


    if name == "narrow_offset_corridor":
        path = ScenarioPath.load_csv(
            _map_path(share_directory, "narrow_offset_corridor.csv"),
            closed_loop=False,
        )
        phase = ScenarioPhase(
            name="long_narrow_corridor_with_reference_offset",
            path=path,
            cruise_speed=1.0,
        )

        # The physical corridor is straight and centred at global y=0, while
        # the supplied reference path is approximately 0.25 m above it and
        # carries a small sinusoidal heading mismatch.  The planner must leave
        # the reference before entry, hold the corridor centre for 70 m, and
        # return to the reference after the exit.
        corridor_x = 65.0
        corridor_length = 70.0
        gap_width = 2.40
        barrier_extent = 8.0
        half_gap = 0.5 * gap_width
        lower_width = barrier_extent - half_gap
        upper_width = lower_width
        lower = ScenarioObstacle(
            x=corridor_x, y=-0.5 * (barrier_extent + half_gap),
            length=corridor_length, width=lower_width, yaw=0.0,
        )
        upper = ScenarioObstacle(
            x=corridor_x, y=0.5 * (barrier_extent + half_gap),
            length=corridor_length, width=upper_width, yaw=0.0,
        )
        gate_s = float(np.interp(corridor_x, path.x, path.s))
        path_y = float(np.interp(gate_s, path.s, path.y))
        path_yaw = float(np.interp(gate_s, path.s, np.unwrap(path.yaw)))
        gate = ScenarioGate(
            s=gate_s,
            lateral_center=-path_y * math.cos(path_yaw),
            gap_width=gap_width,
            barrier_extent=barrier_extent,
            obstacle_length=corridor_length,
        )
        return ScenarioDefinition(
            name=name, phases=(phase,), obstacles=(lower, upper), gates=(gate,),
            terminal_margin=3.0,
            costmap_resolution=0.05,
            footprint_circle_count=16,
        )

    if name == "winding_obstacle_course":
        path = ScenarioPath.load_csv(
            _map_path(share_directory, "winding_obstacle_course.csv"),
            closed_loop=False,
        )
        phase = ScenarioPhase(
            name="winding_mixed_obstacle_course",
            path=path,
            cruise_speed=1.8,
        )

        # Alternating constrained openings are interleaved with independently
        # rotated obstacles.  The layout cannot be solved by holding one
        # lateral offset; it requires repeated left/right reallocation while
        # the reference curvature also changes sign.
        gate_specs = (
            (32.0, 2.60, 4.90, 4.5),
            (86.0, -2.60, 4.80, 5.5),
            (148.0, 2.60, 4.80, 5.0),
            (202.0, -2.60, 4.90, 5.5),
            (252.0, 2.25, 4.90, 4.5),
        )
        gates: list[ScenarioGate] = []
        obstacles: list[ScenarioObstacle] = []
        for s_position, center, gap, obstacle_length in gate_specs:
            gate, lower, upper = gate_on_path(
                path, s_position, lateral_center=center, gap_width=gap,
                barrier_extent=14.0, obstacle_length=obstacle_length,
            )
            gates.append(gate)
            obstacles.extend((lower, upper))

        obstacles.extend(
            (
                obstacle_on_path(
                    path, 58.0, lateral_offset=-1.50, length=4.5, width=2.0,
                    yaw_offset=math.radians(25.0),
                ),
                obstacle_on_path(
                    path, 112.0, lateral_offset=2.80, length=5.0, width=3.0,
                    yaw_offset=math.radians(-18.0),
                ),
                obstacle_on_path(
                    path, 130.0, lateral_offset=-2.80, length=4.5, width=3.0,
                    yaw_offset=math.radians(22.0),
                ),
                obstacle_on_path(
                    path, 174.0, lateral_offset=0.10, length=6.0, width=2.7,
                    yaw_offset=math.radians(35.0),
                ),
            )
        )
        return ScenarioDefinition(
            name=name, phases=(phase,), obstacles=tuple(obstacles), gates=tuple(gates),
            terminal_margin=3.0,
        )

    if name == "winding_obstacle_course_wide_gates":
        # Same reference path, gate positions, and standalone obstacles as
        # "winding_obstacle_course". Only the gate barrier walls are made
        # thicker along the path direction (obstacle_length x3.5) so each
        # wall reaches further to the sides of the corridor as the
        # vehicle passes through, while the lateral reach (barrier_extent)
        # and gap openings stay unchanged. Kept as a separate scenario so
        # the baseline "winding_obstacle_course" is untouched.
        path = ScenarioPath.load_csv(
            _map_path(share_directory, "winding_obstacle_course.csv"),
            closed_loop=False,
        )
        phase = ScenarioPhase(
            name="winding_mixed_obstacle_course_wide_gates",
            path=path,
            cruise_speed=1.8,
        )

        gate_specs = (
            (32.0, 2.60, 4.90, 4.5),
            (86.0, -2.60, 4.80, 5.5),
            (148.0, 2.60, 4.80, 5.0),
            (202.0, -2.60, 4.90, 5.5),
            (252.0, 2.25, 4.90, 4.5),
        )
        gates: list[ScenarioGate] = []
        obstacles: list[ScenarioObstacle] = []
        for s_position, center, gap, obstacle_length in gate_specs:
            gate, lower, upper = gate_on_path(
                path, s_position, lateral_center=center, gap_width=gap,
                barrier_extent=14.0, obstacle_length=obstacle_length * 3.5,
            )
            gates.append(gate)
            obstacles.extend((lower, upper))

        obstacles.extend(
            (
                obstacle_on_path(
                    path, 58.0, lateral_offset=-1.50, length=4.5, width=2.0,
                    yaw_offset=math.radians(25.0),
                ),
                obstacle_on_path(
                    path, 112.0, lateral_offset=2.80, length=5.0, width=3.0,
                    yaw_offset=math.radians(-18.0),
                ),
                obstacle_on_path(
                    path, 130.0, lateral_offset=-2.80, length=4.5, width=3.0,
                    yaw_offset=math.radians(22.0),
                ),
                obstacle_on_path(
                    path, 174.0, lateral_offset=0.10, length=6.0, width=2.7,
                    yaw_offset=math.radians(35.0),
                ),
            )
        )
        return ScenarioDefinition(
            name=name, phases=(phase,), obstacles=tuple(obstacles), gates=tuple(gates),
            terminal_margin=3.0,
        )

    if name == "parking_ramp_loop":
        # Straight entry ramp -> loop most of the way around a circle ->
        # straight exit ramp, walled in on both sides the entire way (a
        # parking-garage-style ramp loop). This is a flat 2D plane (no Z
        # axis), so a near-360 deg sweep makes the entry/exit ramps
        # physically overlap the walled corridor near the start -- there is
        # no elevation change to separate them like a real multi-level ramp.
        # 270 deg with a 30 m circle radius keeps the entry- and exit-ramp
        # walls at least ~11 m apart (checked numerically, well clear of the
        # 4 m corridor width) while still reading as "most of the way
        # around a big circle".
        ramp_length = 15.0
        transition_length = 4.0
        circle_radius = 30.0
        sweep_deg = 270.0
        corridor_half_width = 2.0

        path = build_ramp_loop_path(
            ramp_length=ramp_length,
            transition_length=transition_length,
            circle_radius=circle_radius,
            sweep_deg=sweep_deg,
        )
        phase = ScenarioPhase(
            name="parking_ramp_loop",
            path=path,
            cruise_speed=1.2,
        )

        obstacles = tuple(
            wall_segments_along_path(
                path, 0.0, path.total_length, lateral_offset=corridor_half_width,
            )
            + wall_segments_along_path(
                path, 0.0, path.total_length, lateral_offset=-corridor_half_width,
            )
        )
        return ScenarioDefinition(
            name=name, phases=(phase,), obstacles=obstacles, gates=tuple(),
            terminal_margin=3.0,
        )

    if name == "warehouse_logistics":
        return build_warehouse_logistics_scenario()

    supported = (
        "stadium, crab_switch, spot_turn_course, reverse_switch, hdmap_crab1_switch, fmtc_demo, hdmap_lap_switch, s_curve, "
        "straight_long, obstacle_avoidance, terminal_safe_region, s_curve_obstacles, "
        "alternating_gate_corridor, curved_gate_maze, winding_obstacle_course, "
        "winding_obstacle_course_wide_gates, parking_ramp_loop, warehouse_logistics, "
        "narrow_22m_stop_corridor, narrow_28m_corridor, narrow_offset_corridor"
    )
    raise ValueError(f"Unsupported scenario '{scenario_name}'. Supported: {supported}")


def obstacle_vertices(obstacle: ScenarioObstacle) -> np.ndarray:
    half_length = 0.5 * float(obstacle.length)
    half_width = 0.5 * float(obstacle.width)
    local = np.asarray(
        [
            [-half_length, -half_width],
            [half_length, -half_width],
            [half_length, half_width],
            [-half_length, half_width],
        ],
        dtype=float,
    )
    c = math.cos(float(obstacle.yaw))
    s = math.sin(float(obstacle.yaw))
    rotation = np.asarray([[c, -s], [s, c]], dtype=float)
    return local @ rotation.T + np.asarray([obstacle.x, obstacle.y])


def _rasterize_obstacle_into_grid(
    grid: np.ndarray,
    obstacle: ScenarioObstacle,
    *,
    resolution: float,
    origin_x: float,
    origin_y: float,
) -> None:
    """Conservatively mark cells intersected by one rectangular obstacle."""
    height, width = grid.shape
    vertices = obstacle_vertices(obstacle)
    minimum_x = float(np.min(vertices[:, 0]))
    maximum_x = float(np.max(vertices[:, 0]))
    minimum_y = float(np.min(vertices[:, 1]))
    maximum_y = float(np.max(vertices[:, 1]))
    x_start = max(
        0,
        min(width, int(math.floor((minimum_x - origin_x) / resolution)) - 1),
    )
    x_stop = max(
        0,
        min(width, int(math.ceil((maximum_x - origin_x) / resolution)) + 1),
    )
    y_start = max(
        0,
        min(height, int(math.floor((minimum_y - origin_y) / resolution)) - 1),
    )
    y_stop = max(
        0,
        min(height, int(math.ceil((maximum_y - origin_y) / resolution)) + 1),
    )
    if x_start >= x_stop or y_start >= y_stop:
        return

    xs = origin_x + (np.arange(x_start, x_stop) + 0.5) * resolution
    ys = origin_y + (np.arange(y_start, y_stop) + 0.5) * resolution
    xx, yy = np.meshgrid(xs, ys)
    dx = xx - float(obstacle.x)
    dy = yy - float(obstacle.y)
    c = math.cos(float(obstacle.yaw))
    s = math.sin(float(obstacle.yaw))
    local_x = c * dx + s * dy
    local_y = -s * dx + c * dy

    # Exact separating-axis test between the rotated obstacle rectangle and
    # each axis-aligned costmap cell. A cell is occupied whenever its area
    # intersects the obstacle, not only when its centre lies inside.
    cell_half = 0.5 * resolution
    half_length = 0.5 * float(obstacle.length)
    half_width = 0.5 * float(obstacle.width)
    abs_c = abs(c)
    abs_s = abs(s)
    overlap_obstacle_x = (
        np.abs(local_x)
        <= half_length + cell_half * (abs_c + abs_s) + 1.0e-12
    )
    overlap_obstacle_y = (
        np.abs(local_y)
        <= half_width + cell_half * (abs_c + abs_s) + 1.0e-12
    )
    overlap_grid_x = (
        np.abs(dx)
        <= cell_half + half_length * abs_c + half_width * abs_s + 1.0e-12
    )
    overlap_grid_y = (
        np.abs(dy)
        <= cell_half + half_length * abs_s + half_width * abs_c + 1.0e-12
    )
    intersects = (
        overlap_obstacle_x
        & overlap_obstacle_y
        & overlap_grid_x
        & overlap_grid_y
    )
    grid[y_start:y_stop, x_start:x_stop][intersects] = 100


def rasterize_scenario_costmap(
    scenario: ScenarioDefinition,
    *,
    resolution: float = 0.2,
    margin: float = 10.0,
) -> tuple[np.ndarray, float, float]:
    if resolution <= 0.0 or margin <= 0.0:
        raise ValueError("Costmap resolution and margin must be positive")
    points = [
        np.c_[phase.path.x, phase.path.y]
        for phase in scenario.phases
    ]
    points.extend(obstacle_vertices(obstacle) for obstacle in scenario.obstacles)
    all_points = np.vstack(points)

    origin_x = float(
        math.floor((float(np.min(all_points[:, 0])) - margin) / resolution)
        * resolution
    )
    origin_y = float(
        math.floor((float(np.min(all_points[:, 1])) - margin) / resolution)
        * resolution
    )
    maximum_x = float(
        math.ceil((float(np.max(all_points[:, 0])) + margin) / resolution)
        * resolution
    )
    maximum_y = float(
        math.ceil((float(np.max(all_points[:, 1])) + margin) / resolution)
        * resolution
    )
    width = max(2, int(math.ceil((maximum_x - origin_x) / resolution)))
    height = max(2, int(math.ceil((maximum_y - origin_y) / resolution)))
    grid = np.zeros((height, width), dtype=np.int8)

    for obstacle in scenario.obstacles:
        _rasterize_obstacle_into_grid(
            grid,
            obstacle,
            resolution=resolution,
            origin_x=origin_x,
            origin_y=origin_y,
        )

    return grid, origin_x, origin_y


def rasterize_vehicle_costmap(
    scenario: ScenarioDefinition,
    *,
    vehicle_x: float,
    vehicle_y: float,
    vehicle_yaw: float,
    resolution: float = 0.2,
    size_m: float = 60.0,
) -> tuple[np.ndarray, float, float, float]:
    """Rasterize a vehicle-centred costmap whose cell axes follow body yaw.

    The returned origin is the lower-left grid corner expressed in the
    scenario's odom frame. Together with the returned yaw it is directly
    suitable for ``OccupancyGrid.info.origin``.
    """
    values = (vehicle_x, vehicle_y, vehicle_yaw, resolution, size_m)
    if not all(math.isfinite(float(value)) for value in values):
        raise ValueError("Vehicle costmap inputs must be finite")
    if resolution <= 0.0 or size_m <= 0.0:
        raise ValueError("Vehicle costmap resolution and size must be positive")

    cell_count = max(2, int(math.ceil(size_m / resolution)))
    extent = cell_count * resolution
    local_origin = -0.5 * extent
    grid = np.zeros((cell_count, cell_count), dtype=np.int8)

    vehicle_c = math.cos(vehicle_yaw)
    vehicle_s = math.sin(vehicle_yaw)
    for obstacle in scenario.obstacles:
        dx = float(obstacle.x) - vehicle_x
        dy = float(obstacle.y) - vehicle_y
        body_obstacle = ScenarioObstacle(
            x=vehicle_c * dx + vehicle_s * dy,
            y=-vehicle_s * dx + vehicle_c * dy,
            length=float(obstacle.length),
            width=float(obstacle.width),
            yaw=float(obstacle.yaw) - vehicle_yaw,
        )
        _rasterize_obstacle_into_grid(
            grid,
            body_obstacle,
            resolution=resolution,
            origin_x=local_origin,
            origin_y=local_origin,
        )

    origin_x = (
        vehicle_x + vehicle_c * local_origin - vehicle_s * local_origin
    )
    origin_y = (
        vehicle_y + vehicle_s * local_origin + vehicle_c * local_origin
    )
    return grid, float(origin_x), float(origin_y), float(vehicle_yaw)


def global_display_segments(
    scenario: ScenarioDefinition,
) -> tuple[tuple[np.ndarray, np.ndarray, np.ndarray], ...]:
    segments: list[tuple[np.ndarray, np.ndarray, np.ndarray]] = []
    for index, phase in enumerate(scenario.phases):
        path = phase.path
        if scenario.name in ("crab_switch", "reverse_switch"):
            usable_end = (
                float(phase.switch_s)
                if phase.switch_s is not None
                else max(0.0, path.total_length - scenario.terminal_margin)
            )
            stop = int(np.searchsorted(path.s, usable_end, side="right"))
            stop = max(2, min(stop, len(path.x)))
            segments.append((path.x[:stop], path.y[:stop], path.map_yaw[:stop]))
        else:
            segments.append((path.x, path.y, path.map_yaw))
    return tuple(segments)
