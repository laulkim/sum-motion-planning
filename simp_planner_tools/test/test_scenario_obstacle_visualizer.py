from __future__ import annotations

import math
from pathlib import Path

import pytest

from simp_planner_tools.scenario_definition import ScenarioObstacle, load_scenario_definition
from simp_planner_tools.scenario_obstacle_visualizer_node import (
    PALLET_HEIGHT,
    obstacle_pallet_markers,
    scenario_obstacle_markers,
)


PACKAGE_SHARE = Path(__file__).resolve().parents[1]


def _by_ns(markers):
    return {marker.ns: marker for marker in markers}


def _local_extent(marker):
    xs = [p.x for p in marker.points]
    ys = [p.y for p in marker.points]
    return (
        min(xs) - 0.5 * marker.scale.x, max(xs) + 0.5 * marker.scale.x,
        min(ys) - 0.5 * marker.scale.y, max(ys) + 0.5 * marker.scale.y,
    )


def test_pallets_exactly_cover_the_obstacle_rectangle() -> None:
    obstacle = ScenarioObstacle(x=3.0, y=-2.0, length=5.0, width=8.6, yaw=0.4)
    markers = _by_ns(obstacle_pallet_markers(obstacle, 0, "odom"))

    for ns in ("obstacle_pallet_runners", "obstacle_pallet_deck"):
        assert _local_extent(markers[ns]) == pytest.approx((-2.5, 2.5, -4.3, 4.3))
    # Cartons stay inside the footprint (inset by the carton gap).
    x0, x1, y0, y1 = _local_extent(markers["obstacle_cartons"])
    assert -2.5 < x0 and x1 < 2.5 and -4.3 < y0 and y1 < 4.3
    assert min(p.z for p in markers["obstacle_cartons"].points) > PALLET_HEIGHT


def test_thin_obstacle_is_drawn_as_cartons_without_pallet() -> None:
    curb = ScenarioObstacle(x=0.0, y=0.0, length=1.6, width=0.25)
    markers = _by_ns(obstacle_pallet_markers(curb, 7, "odom"))

    assert set(markers) == {"obstacle_cartons", "obstacle_carton_tape"}
    cartons = markers["obstacle_cartons"]
    assert min(p.z - 0.5 * cartons.scale.z for p in cartons.points) == pytest.approx(0.0)
    assert len(cartons.colors) == len(cartons.points)


def test_layout_is_deterministic_between_runs() -> None:
    obstacle = ScenarioObstacle(x=1.0, y=1.0, length=6.0, width=2.7, yaw=0.6)
    first = obstacle_pallet_markers(obstacle, 3, "odom")
    second = obstacle_pallet_markers(obstacle, 3, "odom")

    assert first == second


def test_winding_markers_follow_every_authored_obstacle_pose() -> None:
    scenario = load_scenario_definition(PACKAGE_SHARE, "winding_obstacle_course")
    markers = scenario_obstacle_markers(scenario.obstacles, "odom").markers
    cartons = [m for m in markers if m.ns == "obstacle_cartons"]

    assert len(cartons) == len(scenario.obstacles) > 0
    for marker, obstacle in zip(cartons, scenario.obstacles):
        assert marker.header.frame_id == "odom"
        assert marker.pose.position.x == pytest.approx(obstacle.x)
        assert marker.pose.position.y == pytest.approx(obstacle.y)
        yaw = 2.0 * math.atan2(marker.pose.orientation.z, marker.pose.orientation.w)
        assert math.cos(yaw - obstacle.yaw) == pytest.approx(1.0)
