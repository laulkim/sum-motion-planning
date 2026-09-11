from pathlib import Path

import numpy as np
import pytest

from simp_planner_tools.scenario_definition import (
    _straight_segment,
    build_spot_turn_crab_course_phases,
    load_scenario_definition,
)
from simp_planner_tools.scenario_path import ScenarioPath


def test_turn_seams_survive_rolling_windows_clipping_and_translation():
    phases = build_spot_turn_crab_course_phases()
    assert [phase.mode for phase in phases] == [0, 2, 0, 2, 0, 2, 0]
    for phase in phases:
        path = phase.path
        for progress in np.linspace(0.0, path.total_length, 81):
            local = path.local_slice(float(progress), 5.0, 40.0)
            for seam in local.skip_continuity_at:
                assert local.segment_length[seam] <= 1.0e-6
                assert seam + 1 >= 3
                assert len(local.x) - 1 - (seam + 1) >= 3
            assert np.isfinite(path.project(*local.end_xy).s)
        for remaining in (0.05, 0.1, 1.0):
            clipped = path.clipped(path.total_length - remaining)
            assert clipped.skip_continuity_at == path.skip_continuity_at
            moved = clipped.translated(0.2, -0.1)
            assert moved.skip_continuity_at == clipped.skip_continuity_at


def test_seam_at_window_boundary_retains_post_turn_points():
    first = _straight_segment(0.0, 0.0, 0.0, 8.0, 0)
    second = _straight_segment(8.0, 0.0, np.pi / 2, 8.0, 0)
    path = ScenarioPath.join_with_turns([first, second])
    local = path.local_slice(0.0, 5.0, 8.0)
    seam = next(iter(local.skip_continuity_at))
    assert len(local.x) - 1 - (seam + 1) >= 3
    clipped = path.clipped(8.0)
    assert not clipped.skip_continuity_at
    assert clipped.yaw[-1] == 0.0
    for x, y in ((7.9, 0.0), (8.0, 0.0), (8.0, 0.3)):
        projection = path.project(x, y)
        assert np.isfinite(projection.s)
        assert projection.segment_index not in path.skip_continuity_at


def test_seam_exceptions_cannot_hide_bad_driving_geometry():
    first = _straight_segment(0.0, 0.0, 0.0, 8.0, 0)
    separated = _straight_segment(8.1, 0.0, np.pi / 2, 8.0, 0)
    with pytest.raises(ValueError, match="coincident"):
        ScenarioPath.join_with_turns([first, separated])
    with pytest.raises(ValueError, match="zero-length"):
        ScenarioPath.from_arrays([0, 1, 1, 2], [0] * 4, [0] * 4, [0] * 4, [0] * 4)
    with pytest.raises(ValueError, match="curvature"):
        ScenarioPath.from_arrays([0, 1, 2, 3], [0] * 4, [0] * 4, [0.3] * 4, [0] * 4)


def test_authored_course_can_be_published_without_removing_its_turns():
    root = Path(__file__).resolve().parents[1]
    scenario = load_scenario_definition(root, "spot_turn_course")
    assert sum(len(phase.path.skip_continuity_at) for phase in scenario.phases) == 3
    assert scenario.phases[0].path.total_length == pytest.approx(72.0)
