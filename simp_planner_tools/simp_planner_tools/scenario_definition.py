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

    supported = (
        "stadium, crab_switch, reverse_switch, s_curve, obstacle_avoidance, "
        "terminal_safe_region, s_curve_obstacles, alternating_gate_corridor, curved_gate_maze, "
        "winding_obstacle_course, narrow_22m_stop_corridor, narrow_28m_corridor, narrow_offset_corridor"
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
        vertices = obstacle_vertices(obstacle)
        minimum_x = float(np.min(vertices[:, 0]))
        maximum_x = float(np.max(vertices[:, 0]))
        minimum_y = float(np.min(vertices[:, 1]))
        maximum_y = float(np.max(vertices[:, 1]))
        x_start = max(0, int(math.floor((minimum_x - origin_x) / resolution)) - 1)
        x_stop = min(width, int(math.ceil((maximum_x - origin_x) / resolution)) + 1)
        y_start = max(0, int(math.floor((minimum_y - origin_y) / resolution)) - 1)
        y_stop = min(height, int(math.ceil((maximum_y - origin_y) / resolution)) + 1)

        xs = origin_x + (np.arange(x_start, x_stop) + 0.5) * resolution
        ys = origin_y + (np.arange(y_start, y_stop) + 0.5) * resolution
        xx, yy = np.meshgrid(xs, ys)
        dx = xx - float(obstacle.x)
        dy = yy - float(obstacle.y)
        c = math.cos(float(obstacle.yaw))
        s = math.sin(float(obstacle.yaw))
        local_x = c * dx + s * dy
        local_y = -s * dx + c * dy

        # Exact separating-axis test between the rotated obstacle rectangle
        # and each axis-aligned costmap cell.  A cell is occupied whenever
        # its area intersects the obstacle, rather than only when its centre
        # lies inside.  This makes the rasterized obstacle a conservative
        # superset of the continuous obstacle geometry.
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
        overlap_world_x = (
            np.abs(dx)
            <= cell_half + half_length * abs_c + half_width * abs_s + 1.0e-12
        )
        overlap_world_y = (
            np.abs(dy)
            <= cell_half + half_length * abs_s + half_width * abs_c + 1.0e-12
        )
        intersects = (
            overlap_obstacle_x
            & overlap_obstacle_y
            & overlap_world_x
            & overlap_world_y
        )
        grid[y_start:y_stop, x_start:x_stop][intersects] = 100

    return grid, origin_x, origin_y


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
