from __future__ import annotations

from pathlib import Path

import numpy as np

from simp_planner_tools.debug_scenario_geometry import scenario_obstacle_polygons
from simp_planner_tools.scenario_definition import load_scenario_definition


PACKAGE_SHARE = Path(__file__).resolve().parents[1]


def test_debug_payload_contains_every_authored_obstacle() -> None:
    scenario = load_scenario_definition(PACKAGE_SHARE, "winding_obstacle_course")
    polygons = scenario_obstacle_polygons(
        PACKAGE_SHARE,
        "winding_obstacle_course",
    )

    assert len(polygons) == len(scenario.obstacles)
    assert len(polygons) > 0
    assert all(polygon.shape == (4, 2) for polygon in polygons)
    assert all(np.all(np.isfinite(polygon)) for polygon in polygons)


def test_track_map_debug_payload_is_explicitly_empty() -> None:
    assert scenario_obstacle_polygons(PACKAGE_SHARE, "track_map") == ()
