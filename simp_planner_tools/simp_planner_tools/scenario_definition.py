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

    @property
    def mode(self) -> int:
        values = np.unique(self.path.mode.astype(int))
        if len(values) != 1:
            raise ValueError(f"Scenario phase '{self.name}' must use one drive mode")
        return int(values[0])


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
                length=segment_length, width=thickness,
            )
        )
        s += spacing
    return segments


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

    supported = (
        "stadium, crab_switch, reverse_switch, hdmap_crab1_switch, fmtc_demo, hdmap_lap_switch, s_curve, "
        "straight_long, obstacle_avoidance, terminal_safe_region, s_curve_obstacles, "
        "alternating_gate_corridor, curved_gate_maze, winding_obstacle_course, "
        "winding_obstacle_course_wide_gates, parking_ramp_loop, "
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
