import math
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

from simp_planner_tools.scenario_definition import (
    ScenarioObstacle,
    load_scenario_definition,
    rasterize_vehicle_costmap,
)


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_vehicle_costmap_is_fixed_size_and_centred_on_rotated_vehicle() -> None:
    scenario = load_scenario_definition(PACKAGE_ROOT, "stadium")
    vehicle_x = 12.0
    vehicle_y = -3.0
    vehicle_yaw = 0.5 * math.pi

    grid, origin_x, origin_y, origin_yaw = rasterize_vehicle_costmap(
        scenario,
        vehicle_x=vehicle_x,
        vehicle_y=vehicle_y,
        vehicle_yaw=vehicle_yaw,
        resolution=0.2,
    )

    assert grid.shape == (300, 300)
    assert grid.dtype == np.int8
    assert np.all(grid == 0)
    assert origin_yaw == vehicle_yaw

    extent = grid.shape[0] * 0.2
    half_extent = 0.5 * extent
    assert math.isclose(origin_x, vehicle_x + half_extent, abs_tol=1.0e-12)
    assert math.isclose(origin_y, vehicle_y - half_extent, abs_tol=1.0e-12)

    # OccupancyGrid origin is the lower-left corner. Moving half an extent
    # along both rotated grid axes must recover the capture-time vehicle pose.
    c = math.cos(origin_yaw)
    s = math.sin(origin_yaw)
    recovered_x = origin_x + c * half_extent - s * half_extent
    recovered_y = origin_y + s * half_extent + c * half_extent
    assert math.isclose(recovered_x, vehicle_x, abs_tol=1.0e-12)
    assert math.isclose(recovered_y, vehicle_y, abs_tol=1.0e-12)


def test_vehicle_costmap_places_world_obstacle_in_body_axes() -> None:
    scenario = load_scenario_definition(PACKAGE_ROOT, "obstacle_avoidance")
    obstacle = ScenarioObstacle(
        x=5.25,
        y=0.25,
        length=0.4,
        width=0.2,
        yaw=0.0,
    )
    scenario = replace(scenario, obstacles=(obstacle,))
    resolution = 0.5
    size_m = 20.0
    vehicle_yaw = 0.5 * math.pi

    grid, origin_x, origin_y, origin_yaw = rasterize_vehicle_costmap(
        scenario,
        vehicle_x=0.0,
        vehicle_y=0.0,
        vehicle_yaw=vehicle_yaw,
        resolution=resolution,
        size_m=size_m,
    )

    # R(-pi/2) maps world (5.25, 0.25) to body (0.25, -5.25).
    body_obstacle_x = 0.25
    body_obstacle_y = -5.25
    half_extent = 0.5 * grid.shape[0] * resolution
    column = int(math.floor((body_obstacle_x + half_extent) / resolution))
    row = int(math.floor((body_obstacle_y + half_extent) / resolution))
    assert grid[row, column] == 100

    # Reconstruct that cell centre in odom using the published origin pose.
    grid_x = (column + 0.5) * resolution
    grid_y = (row + 0.5) * resolution
    c = math.cos(origin_yaw)
    s = math.sin(origin_yaw)
    world_x = origin_x + c * grid_x - s * grid_y
    world_y = origin_y + s * grid_x + c * grid_y
    assert abs(world_x - obstacle.x) <= 0.5 * resolution
    assert abs(world_y - obstacle.y) <= 0.5 * resolution


def test_vehicle_costmap_conservatively_marks_boundary_cells() -> None:
    scenario = load_scenario_definition(PACKAGE_ROOT, "obstacle_avoidance")
    obstacle = scenario.obstacles[0]
    vehicle_yaw = 0.37
    resolution = 0.2
    grid, _, _, _ = rasterize_vehicle_costmap(
        scenario,
        vehicle_x=obstacle.x,
        vehicle_y=obstacle.y,
        vehicle_yaw=vehicle_yaw,
        resolution=resolution,
        size_m=10.0,
    )

    rows, columns = np.nonzero(grid >= 50)
    half_extent = 0.5 * grid.shape[0] * resolution
    cell_x = -half_extent + (columns + 0.5) * resolution
    cell_y = -half_extent + (rows + 0.5) * resolution
    relative_yaw = float(obstacle.yaw) - vehicle_yaw
    c = math.cos(relative_yaw)
    s = math.sin(relative_yaw)
    obstacle_x = c * cell_x + s * cell_y
    obstacle_y = -s * cell_x + c * cell_y
    centre_inside = (
        (np.abs(obstacle_x) <= 0.5 * obstacle.length + 1.0e-12)
        & (np.abs(obstacle_y) <= 0.5 * obstacle.width + 1.0e-12)
    )

    assert rows.size > 0
    assert np.count_nonzero(~centre_inside) > 0


def test_vehicle_costmap_ignores_obstacles_outside_sensor_extent() -> None:
    scenario = load_scenario_definition(PACKAGE_ROOT, "obstacle_avoidance")
    scenario = replace(
        scenario,
        obstacles=(ScenarioObstacle(1000.0, 1000.0, 4.0, 2.0),),
    )
    grid, _, _, _ = rasterize_vehicle_costmap(
        scenario,
        vehicle_x=0.0,
        vehicle_y=0.0,
        vehicle_yaw=-0.8,
        resolution=0.2,
        size_m=60.0,
    )
    assert np.all(grid == 0)


@pytest.mark.parametrize(
    "kwargs",
    (
        {"resolution": 0.0, "size_m": 60.0},
        {"resolution": 0.2, "size_m": 0.0},
        {"resolution": 0.2, "size_m": math.nan},
        {"resolution": 0.2, "size_m": 60.0, "vehicle_yaw": math.inf},
    ),
)
def test_vehicle_costmap_rejects_invalid_geometry(kwargs: dict[str, float]) -> None:
    scenario = load_scenario_definition(PACKAGE_ROOT, "stadium")
    values = {
        "vehicle_x": 0.0,
        "vehicle_y": 0.0,
        "vehicle_yaw": 0.0,
        **kwargs,
    }
    with pytest.raises(ValueError):
        rasterize_vehicle_costmap(scenario, **values)
