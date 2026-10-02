from __future__ import annotations

import math
from pathlib import Path

import pytest

from simp_planner_tools.models import MODE_BETA_CENTER, DriveMode
from simp_planner_tools.scenario_definition import load_scenario_definition
from simp_planner_tools.scenario_obstacle_visualizer_node import (
    floor_marking_markers,
    scenario_obstacle_markers,
)
from visualization_msgs.msg import Marker


PACKAGE_SHARE = Path(__file__).resolve().parents[1]
AVOIDANCE_COUNT = 3  # 마지막 phase 통로의 회피 대상 (obstacles 끝에 있다)
# planner 기본 충돌 모델: 3.0 x 2.0 m 차량을 원 3개로 덮는다
CIRCLE_SPACING = 1.0
CIRCLE_RADIUS = math.hypot(0.5, 1.0)


@pytest.fixture(scope="module")
def scenario():
    return load_scenario_definition(PACKAGE_SHARE, "warehouse_logistics")


def _body_yaw(path, index: int) -> float:
    return float(path.yaw[index]) - MODE_BETA_CENTER[DriveMode(int(path.mode[index]))]


def _rect_distance(x: float, y: float, obstacle) -> float:
    c, s = math.cos(obstacle.yaw), math.sin(obstacle.yaw)
    local_x = c * (x - obstacle.x) + s * (y - obstacle.y)
    local_y = -s * (x - obstacle.x) + c * (y - obstacle.y)
    return math.hypot(max(abs(local_x) - 0.5 * obstacle.length, 0.0),
                      max(abs(local_y) - 0.5 * obstacle.width, 0.0))


def _footprint_clearance(path, obstacles) -> float:
    clearance = math.inf
    for index in range(len(path.x)):
        yaw = _body_yaw(path, index)
        for offset in (-CIRCLE_SPACING, 0.0, CIRCLE_SPACING):
            cx = path.x[index] + offset * math.cos(yaw)
            cy = path.y[index] + offset * math.sin(yaw)
            for obstacle in obstacles:
                clearance = min(clearance, _rect_distance(cx, cy, obstacle) - CIRCLE_RADIUS)
    return clearance


def test_spot_turn_docking_then_crab_docking(scenario) -> None:
    modes = [phase.mode for phase in scenario.phases]
    assert modes == [0, 0, 1, 0, 3, 2, 0]

    boundary_turns = []
    for before, after in zip(scenario.phases, scenario.phases[1:]):
        end, start = before.path, after.path
        assert math.hypot(end.x[-1] - start.x[0], end.y[-1] - start.y[0]) < 1.0e-6
        turn = math.remainder(_body_yaw(start, 0) - _body_yaw(end, -1), 2.0 * math.pi)
        if abs(turn) > math.radians(20.0):
            boundary_turns.append(round(math.degrees(turn)))
    # D2 정면 도킹은 들어갈 때(동->남), 나와서(남->동) 제자리턴. D4 크랩 도킹은 차체각 그대로.
    assert boundary_turns == [-90, 90]
    crab_in, crab_out = scenario.phases[4].path, scenario.phases[5].path
    assert _body_yaw(crab_in, 0) == pytest.approx(0.0)
    assert _body_yaw(crab_out, 0) == pytest.approx(0.0)
    assert crab_in.y[-1] < crab_in.y[0] and crab_out.y[-1] == pytest.approx(crab_in.y[0])
    assert crab_in.x[0] == pytest.approx(scenario.phases[3].path.x[-1])
    assert all(not phase.path.skip_continuity_at for phase in scenario.phases)


def test_only_the_last_aisle_obstacles_need_avoidance(scenario) -> None:
    static = scenario.obstacles[:-AVOIDANCE_COUNT]
    avoidance = scenario.obstacles[-AVOIDANCE_COUNT:]

    for phase in scenario.phases:
        assert _footprint_clearance(phase.path, static) > 0.8, phase.name
    for obstacle in avoidance:
        assert _footprint_clearance(scenario.phases[-1].path, (obstacle,)) < 0.0
        assert all(_footprint_clearance(phase.path, (obstacle,)) > 3.0
                   for phase in scenario.phases[:-1])


def test_reference_curvature_is_continuous(scenario) -> None:
    # 플래너처럼 헤딩 변화/거리로 곡률을 다시 계산해도 계단 없이 이어져야 한다
    for phase in scenario.phases:
        path = phase.path
        kappa = [(path.yaw[i + 1] - path.yaw[i]) / path.segment_length[i]
                 for i in range(len(path.x) - 1)]
        assert max(abs(value) for value in kappa) < 0.1, phase.name
        assert max(abs(b - a) for a, b in zip(kappa, kappa[1:])) < 0.003, phase.name


def test_ramp_outer_walls_continue_into_the_aisle_walls(scenario) -> None:
    walls = [obstacle for obstacle in scenario.obstacles if obstacle.kind == "wall"]
    north = max(walls, key=lambda wall: wall.length)
    north_ends = ((north.x - 0.5 * north.length, north.y), (north.x + 0.5 * north.length, north.y))
    curved = [wall for wall in walls if abs(math.remainder(wall.yaw, 0.5 * math.pi)) > 1.0e-6]
    for x, y in north_ends:
        assert min(math.hypot(wall.x - x, wall.y - y) for wall in curved) < 1.0


def test_floor_has_no_labels_or_arrows(scenario) -> None:
    assert not any(marking.kind in ("label", "arrow") for marking in scenario.markings)


def test_route_stays_inside_the_walled_floor(scenario) -> None:
    floors = [marking.points for marking in scenario.markings
              if marking.kind == "area" and marking.color == "floor"]

    def inside_convex(x: float, y: float, polygon) -> bool:
        signs = {math.copysign(1.0, (bx - ax) * (y - ay) - (by - ay) * (x - ax))
                 for (ax, ay), (bx, by) in zip(polygon, polygon[1:] + polygon[:1])
                 if math.hypot(bx - ax, by - ay) > 1.0e-9}
        return len(signs) == 1

    def inside(x: float, y: float) -> bool:
        return any(inside_convex(x, y, polygon) for polygon in floors)

    for phase in scenario.phases:
        assert all(inside(x, y) for x, y in zip(phase.path.x, phase.path.y)), phase.name


def test_floor_and_walls_render_as_rviz_markers(scenario) -> None:
    floor = floor_marking_markers(scenario.markings, "odom")
    triangle_lists = [marker for marker in floor if marker.type == Marker.TRIANGLE_LIST]

    assert {marker.ns for marker in triangle_lists} == {
        "floor_base", "floor_surface", "floor_paint"}
    for marker in triangle_lists:
        assert len(marker.points) % 3 == 0 and len(marker.points) == len(marker.colors)
        assert marker.pose.position.z < 0.0  # costmap(z=0) 아래
    assert not any(marker.type == Marker.TEXT_VIEW_FACING for marker in floor)

    walls = [marker for marker in scenario_obstacle_markers(scenario.obstacles, "odom").markers
             if marker.ns == "obstacle_walls"]
    assert len(walls) == sum(obstacle.kind == "wall" for obstacle in scenario.obstacles)
    assert all(marker.type == Marker.CUBE for marker in walls)
