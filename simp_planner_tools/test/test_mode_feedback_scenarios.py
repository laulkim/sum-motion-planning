from pathlib import Path

import numpy as np

from planar_velocity_sim.mode_transition import DriveModeTransitionModel
from simp_planner_tools.scenario_definition import load_scenario_definition


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
EXPECTED_MODE_SEQUENCES = {
    "stadium": [0],
    "crab_switch": [0, 2],
    "reverse_switch": [0, 1],
    "s_curve": [0],
    "obstacle_avoidance": [0],
    "s_curve_obstacles": [0],
    "alternating_gate_corridor": [0],
    "curved_gate_maze": [0],
    "winding_obstacle_course": [0],
    "narrow_22m_stop_corridor": [0],
    "narrow_28m_corridor": [0],
    "narrow_offset_corridor": [0],
}


def planner_mode_ready(requested_mode: int, model: DriveModeTransitionModel) -> bool:
    feedback = model.feedback()
    return (
        feedback.current_mode == requested_mode
        and feedback.transition_complete
        and not feedback.transition_in_progress
    )


def test_all_scenarios_use_the_expected_vehicle_confirmed_mode_sequence() -> None:
    for name, expected_modes in EXPECTED_MODE_SEQUENCES.items():
        scenario = load_scenario_definition(PACKAGE_ROOT, name)
        actual_modes = [phase.mode for phase in scenario.phases]
        assert actual_modes == expected_modes

        vehicle = DriveModeTransitionModel(
            initial_mode=actual_modes[0],
            transition_duration_sec=2.0,
            stop_speed_threshold=0.03,
        )
        assert planner_mode_ready(actual_modes[0], vehicle)

        for transition_index, next_mode in enumerate(actual_modes[1:], start=1):
            start_time = 10.0 * transition_index

            # The lower controller/simulator must reject a mode command while moving.
            assert not vehicle.command(next_mode, measured_speed=0.20, now_sec=start_time)
            assert vehicle.current_mode == actual_modes[transition_index - 1]

            # Once stopped, the request is accepted but the planner remains blocked.
            assert vehicle.command(next_mode, measured_speed=0.0, now_sec=start_time + 1.0)
            assert vehicle.transition_in_progress
            assert not planner_mode_ready(next_mode, vehicle)
            assert vehicle.applied_velocity(1.0, 1.0, 0.2) == (0.0, 0.0, 0.0)

            assert not vehicle.update(start_time + 2.99)
            assert not planner_mode_ready(next_mode, vehicle)

            # New-mode planning is allowed only after vehicle confirmation.
            assert vehicle.update(start_time + 3.0)
            assert vehicle.current_mode == next_mode
            assert planner_mode_ready(next_mode, vehicle)


def test_each_scenario_definition_matches_its_intended_motion_task() -> None:
    stadium = load_scenario_definition(PACKAGE_ROOT, "stadium")
    assert stadium.repeat
    assert [phase.mode for phase in stadium.phases] == [0]
    assert not stadium.obstacles

    crab = load_scenario_definition(PACKAGE_ROOT, "crab_switch")
    assert [phase.mode for phase in crab.phases] == [0, 2]
    assert crab.phases[0].switch_s is not None
    crab_path = crab.phases[1].path
    assert np.max(np.abs(crab_path.x - crab_path.x[0])) <= 1.0e-9
    assert crab_path.y[-1] > crab_path.y[0] + 20.0
    assert np.all(crab_path.mode == 2)

    reverse = load_scenario_definition(PACKAGE_ROOT, "reverse_switch")
    assert [phase.mode for phase in reverse.phases] == [0, 1]
    assert reverse.phases[0].switch_s is not None
    reverse_path = reverse.phases[1].path
    assert reverse_path.x[-1] < reverse_path.x[0] - 20.0
    assert np.all(reverse_path.mode == 1)

    s_curve = load_scenario_definition(PACKAGE_ROOT, "s_curve")
    assert [phase.mode for phase in s_curve.phases] == [0]
    assert np.max(np.abs(s_curve.phases[0].path.kappa)) > 0.01
    assert not s_curve.obstacles

    obstacle = load_scenario_definition(PACKAGE_ROOT, "obstacle_avoidance")
    assert [phase.mode for phase in obstacle.phases] == [0]
    assert len(obstacle.obstacles) == 1

    multi = load_scenario_definition(PACKAGE_ROOT, "s_curve_obstacles")
    assert [phase.mode for phase in multi.phases] == [0]
    assert len(multi.obstacles) == 3
    assert np.max(np.abs(multi.phases[0].path.kappa)) > 0.01

    alternating = load_scenario_definition(PACKAGE_ROOT, "alternating_gate_corridor")
    assert [phase.mode for phase in alternating.phases] == [0]
    assert len(alternating.gates) == 5
    assert len(alternating.obstacles) == 10
    assert [gate.lateral_center for gate in alternating.gates] == [3.0, -3.0, 3.0, -3.0, 2.75]
    assert all(abs(gate.gap_width - 4.8) <= 1.0e-12 for gate in alternating.gates)

    curved = load_scenario_definition(PACKAGE_ROOT, "curved_gate_maze")
    assert [phase.mode for phase in curved.phases] == [0]
    assert len(curved.gates) == 5
    assert len(curved.obstacles) == 10
    assert np.max(np.abs(curved.phases[0].path.kappa)) > 0.01
    assert [gate.lateral_center for gate in curved.gates] == [-2.5, 2.5, -2.5, 2.5, -2.25]
    assert all(abs(gate.gap_width - 5.0) <= 1.0e-12 for gate in curved.gates)

    winding = load_scenario_definition(PACKAGE_ROOT, "winding_obstacle_course")
    assert [phase.mode for phase in winding.phases] == [0]
    assert len(winding.gates) == 5
    assert len(winding.obstacles) == 14
    assert np.max(np.abs(winding.phases[0].path.kappa)) > 0.08
    assert [gate.lateral_center for gate in winding.gates] == [2.6, -2.6, 2.6, -2.6, 2.25]
    aligned_yaws = []
    for obstacle in winding.obstacles[10:]:
        projection = winding.phases[0].path.project(obstacle.x, obstacle.y)
        aligned_yaws.append(abs(float(obstacle.yaw - projection.yaw)))
    assert max(aligned_yaws) > np.deg2rad(20.0)


def test_narrow_corridor_geometry_matches_ten_percent_width_margin() -> None:
    scenario = load_scenario_definition(PACKAGE_ROOT, "narrow_22m_stop_corridor")
    assert [phase.mode for phase in scenario.phases] == [0]
    assert len(scenario.gates) == 1
    assert len(scenario.obstacles) == 2
    gate = scenario.gates[0]
    assert abs(gate.lateral_center) <= 1.0e-12
    assert abs(gate.gap_width - 2.20) <= 1.0e-12
    assert abs(gate.obstacle_length - 20.0) <= 1.0e-12


def test_narrow_offset_corridor_motion_task() -> None:
    scenario = load_scenario_definition(PACKAGE_ROOT, "narrow_offset_corridor")
    assert [phase.mode for phase in scenario.phases] == [0]
    assert len(scenario.gates) == 1
    assert abs(scenario.gates[0].gap_width - 2.40) <= 1.0e-12
    assert scenario.gates[0].lateral_center < -0.20
    assert scenario.phases[0].path.total_length > 120.0


def test_coarse_28m_corridor_motion_task() -> None:
    scenario = load_scenario_definition(PACKAGE_ROOT, "narrow_28m_corridor")
    assert [phase.mode for phase in scenario.phases] == [0]
    assert len(scenario.gates) == 1
    assert abs(scenario.gates[0].gap_width - 2.80) <= 1.0e-12
    assert scenario.gates[0].lateral_center < -0.20
    assert scenario.costmap_resolution == 0.20
    assert scenario.footprint_circle_count == 3
    assert scenario.phases[0].path.total_length > 120.0
