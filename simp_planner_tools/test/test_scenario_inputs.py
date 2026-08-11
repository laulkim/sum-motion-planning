from pathlib import Path

import numpy as np

from simp_planner_tools.scenario_definition import (
    load_scenario_definition,
    rasterize_scenario_costmap,
)


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_all_input_scenarios_and_costmaps_are_valid() -> None:
    names = (
        "stadium",
        "crab_switch",
        "reverse_switch",
        "s_curve",
        "obstacle_avoidance",
        "terminal_safe_region",
        "s_curve_obstacles",
        "alternating_gate_corridor",
        "curved_gate_maze",
        "winding_obstacle_course",
        "narrow_22m_stop_corridor",
        "narrow_28m_corridor",
        "narrow_offset_corridor",
    )
    for name in names:
        scenario = load_scenario_definition(PACKAGE_ROOT, name)
        assert scenario.phases
        for phase in scenario.phases:
            assert phase.path.total_length > 10.0
            assert np.max(np.abs(phase.path.kappa)) <= 0.2 + 1.0e-9
        grid, _, _ = rasterize_scenario_costmap(scenario)
        obstacle_cells = int(np.count_nonzero(grid >= 50))
        if name in (
            "obstacle_avoidance",
            "terminal_safe_region",
            "s_curve_obstacles",
            "alternating_gate_corridor",
            "curved_gate_maze",
            "winding_obstacle_course",
            "narrow_22m_stop_corridor",
            "narrow_28m_corridor",
            "narrow_offset_corridor",
        ):
            assert obstacle_cells > 0
        else:
            assert obstacle_cells == 0


def test_complex_gate_openings_are_physically_traversable() -> None:
    segment_length = 3.0 / 3.0
    circle_radius = float(np.hypot(0.5 * segment_length, 1.0))
    required_width = 2.0 * (circle_radius + 0.15)
    for name in ("alternating_gate_corridor", "curved_gate_maze", "winding_obstacle_course"):
        scenario = load_scenario_definition(PACKAGE_ROOT, name)
        if name == "winding_obstacle_course":
            assert len(scenario.obstacles) > 2 * len(scenario.gates)
        else:
            assert len(scenario.obstacles) == 2 * len(scenario.gates)
        assert len(scenario.gates) >= 5
        for gate in scenario.gates:
            assert gate.gap_width > required_width
            assert abs(gate.lateral_center) + 0.5 * gate.gap_width < gate.barrier_extent


def test_costmap_marks_every_cell_intersecting_an_obstacle() -> None:
    scenario = load_scenario_definition(PACKAGE_ROOT, "obstacle_avoidance")
    grid, origin_x, origin_y = rasterize_scenario_costmap(
        scenario, resolution=0.2, margin=10.0
    )
    obstacle = scenario.obstacles[0]
    ys, xs = np.nonzero(grid >= 50)
    cell_x = origin_x + (xs + 0.5) * 0.2
    cell_y = origin_y + (ys + 0.5) * 0.2
    dx = cell_x - float(obstacle.x)
    dy = cell_y - float(obstacle.y)
    c = np.cos(float(obstacle.yaw))
    s = np.sin(float(obstacle.yaw))
    local_x = c * dx + s * dy
    local_y = -s * dx + c * dy
    centre_inside = (
        (np.abs(local_x) <= 0.5 * float(obstacle.length) + 1.0e-12)
        & (np.abs(local_y) <= 0.5 * float(obstacle.width) + 1.0e-12)
    )
    # Intersection rasterization must include boundary cells whose centres are
    # outside the continuous obstacle, unlike the previous centre-only rule.
    assert np.count_nonzero(~centre_inside) > 0


def test_narrow_corridor_is_intentionally_below_current_safety_envelope() -> None:
    scenario = load_scenario_definition(PACKAGE_ROOT, "narrow_22m_stop_corridor")
    assert len(scenario.gates) == 1
    assert len(scenario.obstacles) == 2
    gate = scenario.gates[0]
    assert abs(gate.gap_width - 2.20) <= 1.0e-12
    assert abs(gate.gap_width / 2.0 - 1.10) <= 1.0e-12

    segment_length = 3.0 / 3.0
    circle_radius = float(np.hypot(0.5 * segment_length, 1.0))
    minimum_required_at_rest = 2.0 * (circle_radius + 0.05)
    assert gate.gap_width < minimum_required_at_rest



def test_coarse_28m_corridor_keeps_fixed_deployment_specification() -> None:
    scenario = load_scenario_definition(PACKAGE_ROOT, "narrow_28m_corridor")
    assert len(scenario.gates) == 1
    assert len(scenario.obstacles) == 2
    gate = scenario.gates[0]
    assert abs(gate.gap_width - 2.80) <= 1.0e-12
    assert abs(gate.obstacle_length - 70.0) <= 1.0e-12
    assert scenario.costmap_resolution == 0.20
    assert scenario.footprint_circle_count == 3
    assert abs(gate.lateral_center + 0.25) < 0.02

    path = scenario.phases[0].path
    inside = (path.x >= 30.0) & (path.x <= 100.0)
    assert abs(float(np.mean(path.y[inside])) - 0.35) < 0.01
    assert float(np.max(np.abs(path.yaw[inside]))) > np.deg2rad(0.1)

    grid, origin_x, origin_y = rasterize_scenario_costmap(scenario)
    resolution = scenario.costmap_resolution
    x_index = int(np.floor((65.0 - origin_x) / resolution))
    occupied = grid[:, x_index] >= 50
    cell_centres = origin_y + (np.arange(grid.shape[0]) + 0.5) * resolution
    centre_index = int(np.argmin(np.abs(cell_centres - 0.10)))
    lower = centre_index
    upper = centre_index
    while lower > 0 and not occupied[lower - 1]:
        lower -= 1
    while upper + 1 < len(occupied) and not occupied[upper + 1]:
        upper += 1
    raster_free_width = (upper - lower + 1) * resolution
    assert abs(raster_free_width - 2.60) <= 1.0e-12

    circle_radius = float(np.hypot(0.5, 1.0))
    distance_field_correction = 0.5 * np.sqrt(2.0) * resolution
    static_centre_clearance = 1.40 - distance_field_correction - circle_radius
    assert static_centre_clearance > 0.10

def test_narrow_offset_corridor_requires_reference_correction() -> None:
    scenario = load_scenario_definition(PACKAGE_ROOT, "narrow_offset_corridor")
    assert len(scenario.gates) == 1
    assert len(scenario.obstacles) == 2
    gate = scenario.gates[0]
    assert abs(gate.gap_width - 2.40) <= 1.0e-12
    assert abs(gate.obstacle_length - 70.0) <= 1.0e-12
    assert scenario.costmap_resolution == 0.05
    assert scenario.footprint_circle_count == 16
    assert abs(gate.lateral_center + 0.25) < 0.01
    path = scenario.phases[0].path
    inside = (path.x >= 30.0) & (path.x <= 100.0)
    assert float(np.mean(path.y[inside])) > 0.24
    assert float(np.max(np.abs(path.yaw[inside]))) > np.deg2rad(0.1)


def test_terminal_safe_region_blocks_only_reference_endpoint() -> None:
    scenario = load_scenario_definition(PACKAGE_ROOT, "terminal_safe_region")
    assert len(scenario.phases) == 1
    assert len(scenario.obstacles) == 1
    obstacle = scenario.obstacles[0]
    assert abs(obstacle.x - 76.95) < 1.0e-12
    assert abs(obstacle.y) < 1.0e-12
    assert obstacle.width == 2.0
    assert scenario.phases[0].path.total_length == 80.0
