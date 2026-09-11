from pathlib import Path

import numpy as np
import pytest

from simp_planner_tools.models import MODE_BETA_CENTER, DriveMode
from simp_planner_tools.scenario_definition import (
    _concat_segments,
    _straight_segment,
    build_spot_turn_crab_course_phases,
    load_scenario_definition,
)
from simp_planner_tools.scenario_path import ScenarioPath


def _body_yaw(path: ScenarioPath, index: int) -> float:
    mode = DriveMode(int(path.mode[index]))
    return float(path.yaw[index]) - MODE_BETA_CENTER[mode]


def test_course_phases_are_seam_free_and_survive_rolling_windows_clipping_and_translation():
    phases = build_spot_turn_crab_course_phases()
    assert [phase.mode for phase in phases] == [0, 0, 2, 0, 2, 0, 2, 0]
    for phase in phases:
        path = phase.path
        assert not path.skip_continuity_at
        for progress in np.linspace(0.0, path.total_length, 41):
            local = path.local_slice(float(progress), 5.0, 40.0)
            assert not local.skip_continuity_at
            assert np.isfinite(path.project(*local.end_xy).s)
        for remaining in (0.05, 0.1, 1.0):
            clipped = path.clipped(path.total_length - remaining)
            assert not clipped.skip_continuity_at
            moved = clipped.translated(0.2, -0.1)
            assert not moved.skip_continuity_at


def test_seam_at_window_boundary_retains_post_turn_points():
    first = _straight_segment(0.0, 0.0, 0.0, 8.0, 0)
    second = _straight_segment(8.0, 0.0, np.pi / 2, 8.0, 0)
    path = _concat_segments((first, second))
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
    with pytest.raises(ValueError, match="discontinuity"):
        _concat_segments((first, separated))
    with pytest.raises(ValueError, match="zero-length"):
        ScenarioPath.from_arrays([0, 1, 1, 2], [0] * 4, [0] * 4, [0] * 4, [0] * 4)
    with pytest.raises(ValueError, match="curvature"):
        ScenarioPath.from_arrays([0, 1, 2, 3], [0] * 4, [0] * 4, [0.3] * 4, [0] * 4)


def test_authored_course_needs_a_spot_turn_at_exactly_three_phase_boundaries():
    root = Path(__file__).resolve().parents[1]
    scenario = load_scenario_definition(root, "spot_turn_course")
    assert len(scenario.phases) == 8
    assert scenario.phases[0].path.total_length == pytest.approx(36.0)

    body_yaw_jumps = []
    for before, after in zip(scenario.phases, scenario.phases[1:]):
        gap = np.hypot(
            before.path.x[-1] - after.path.x[0], before.path.y[-1] - after.path.y[0]
        )
        assert gap < 1.0e-9
        jump = _body_yaw(after.path, 0) - _body_yaw(before.path, -1)
        body_yaw_jumps.append(float(np.degrees((jump + np.pi) % (2 * np.pi) - np.pi)))

    turning_boundaries = [abs(jump) > 1.0 for jump in body_yaw_jumps]
    assert sum(turning_boundaries) == 3
    assert turning_boundaries == [True, False, True, False, True, False, False]
    assert body_yaw_jumps[0] == pytest.approx(40.0)
    assert body_yaw_jumps[2] == pytest.approx(-60.0)
    assert body_yaw_jumps[4] == pytest.approx(70.0)
