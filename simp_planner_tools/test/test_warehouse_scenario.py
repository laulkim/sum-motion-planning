from __future__ import annotations

import math
from pathlib import Path

import numpy as np
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


def test_ramps_and_aisle_form_one_closed_enclosure(scenario) -> None:
    # 출발점에서 빈칸을 퍼뜨려도 맵 가장자리에 닿지 않아야 하고(벽 틈 없음), 전체 경로가
    # 같은 닫힌 공간 안에 있어야 한다.
    from collections import deque

    from simp_planner_tools.scenario_definition import rasterize_scenario_costmap

    resolution = 0.2
    grid, origin_x, origin_y = rasterize_scenario_costmap(
        scenario, resolution=resolution, margin=10.0)
    height, width = grid.shape

    def cell(x: float, y: float) -> tuple[int, int]:
        return int((y - origin_y) / resolution), int((x - origin_x) / resolution)

    start = cell(scenario.phases[0].path.x[0], scenario.phases[0].path.y[0])
    reached = np.zeros(grid.shape, dtype=bool)
    reached[start] = True
    queue = deque([start])
    while queue:
        row, col = queue.popleft()
        assert 0 < row < height - 1 and 0 < col < width - 1, "enclosure leaks to the map border"
        for d_row, d_col in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nxt = (row + d_row, col + d_col)
            if not reached[nxt] and grid[nxt] == 0:
                reached[nxt] = True
                queue.append(nxt)
    for phase in scenario.phases:
        assert all(reached[cell(x, y)] for x, y in zip(phase.path.x, phase.path.y)), phase.name


def test_single_lane_ramp_walls_are_half_a_lane_from_the_route(scenario) -> None:
    lane_half_width = 3.0      # 단일 차선 6.0 m
    walls = [obstacle for obstacle in scenario.obstacles if obstacle.kind == "wall"]
    entry = scenario.phases[0].path
    ramp_end = int(np.argmax(np.abs(entry.kappa) > 1.0e-6)) + 40   # 첫 회전 안쪽 샘플
    x, y = entry.x[ramp_end], entry.y[ramp_end]
    nearest = sorted(_rect_distance(x, y, wall) for wall in walls)[:2]
    assert nearest == pytest.approx([lane_half_width, lane_half_width], abs=0.05)


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


def test_ramps_run_slower_than_the_aisle_and_docking_is_slowest(scenario) -> None:
    ramp_speed, drive_speed, docking_speed = 2.3, 2.8, 0.9
    entry, exit_route = scenario.phases[0], scenario.phases[-1]
    for phase in (entry, exit_route):
        speeds = [phase.cruise_speed_at(float(s)) for s in phase.path.s]
        curved = [abs(k) > 1.0e-6 for k in phase.path.kappa]
        assert all(v == ramp_speed for v, c in zip(speeds, curved) if c), phase.name
        assert drive_speed in speeds, phase.name   # 통로 직선 구간
    # 진출 감속 구간은 마지막 회피 장애물을 지난 뒤에 시작한다
    last_obstacle_x = scenario.obstacles[-1].x
    zone_start = exit_route.speed_zones[0][0]
    zone_start_x = float(np.interp(zone_start, exit_route.path.s, exit_route.path.x))
    assert zone_start_x > last_obstacle_x + 3.0
    assert scenario.phases[3].cruise_speed == drive_speed
    assert all(phase.cruise_speed == docking_speed and not phase.speed_zones
               for phase in (scenario.phases[1], scenario.phases[2],
                             scenario.phases[4], scenario.phases[5]))


def test_scenario_manager_publishes_the_speed_of_the_current_zone(scenario) -> None:
    from types import SimpleNamespace

    from simp_planner_tools.scenario_manager_node import ScenarioManagerNode

    entry = scenario.phases[0]
    zone_end = entry.speed_zones[0][1]

    def speed(s, override=None):
        manager = SimpleNamespace(target_override=override, active_phase=entry,
                                  last_projection=None if s is None else SimpleNamespace(s=s))
        return ScenarioManagerNode.active_cruise_speed(manager)

    assert speed(None) == 2.3                    # 출발 직후 (투영 전) = 램프
    assert speed(0.5 * zone_end) == 2.3
    assert speed(zone_end + 5.0) == 2.8
    assert speed(zone_end + 5.0, override=1.2) == 1.2   # target_speed 지정 시 우선


def test_scenario_starts_immediately_and_start_delay_still_holds(scenario) -> None:
    from types import SimpleNamespace

    from simp_planner_tools.scenario_manager_node import ScenarioManagerNode, ScenarioState

    assert scenario.start_delay == 0.0           # 창고 시나리오는 바로 출발
    now = {"ns": 0}
    manager = SimpleNamespace(                   # start_delay_sec:=10 으로 실행한 경우
        start_delay=10.0, start_release_ns=None, start_released=False,
        received_odom=False, state=ScenarioState.RUNNING, target_override=None,
        active_phase=scenario.phases[0], last_projection=None,
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=now["ns"])),
        get_logger=lambda: SimpleNamespace(info=lambda *_: None))
    for name in ("start_hold_remaining", "update_start_hold", "active_cruise_speed"):
        setattr(manager, name, getattr(ScenarioManagerNode, name).__get__(manager))

    def speed() -> float:
        manager.update_start_hold()
        return ScenarioManagerNode.current_target_speed(manager)

    assert speed() == 0.0                       # 첫 odom 전
    manager.received_odom = True
    assert speed() == 0.0                       # 대기 시작
    now["ns"] = int(9.9e9)
    assert speed() == 0.0
    now["ns"] = int(10.1e9)
    assert speed() == scenario.phases[0].cruise_speed_at(0.0)
    assert manager.start_released
