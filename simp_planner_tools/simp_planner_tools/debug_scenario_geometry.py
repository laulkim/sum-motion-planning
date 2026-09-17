from __future__ import annotations

from pathlib import Path

import numpy as np

from .scenario_definition import load_scenario_definition, obstacle_vertices


def scenario_obstacle_polygons(
    package_share: Path,
    scenario_name: str,
) -> tuple[np.ndarray, ...]:
    """Load all authored obstacle polygons for a debug snapshot."""
    if str(scenario_name).strip().lower() == "track_map":
        return ()
    scenario = load_scenario_definition(package_share, scenario_name)
    return tuple(obstacle_vertices(obstacle) for obstacle in scenario.obstacles)
