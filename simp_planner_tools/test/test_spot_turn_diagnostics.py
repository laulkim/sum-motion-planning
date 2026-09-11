from types import SimpleNamespace

from simp_planner_tools.debug_plot_node import DebugPlotNode
from simp_planner_tools.vehicle_visualizer_node import VehicleVisualizerNode


def monitor(planner_status):
    return SimpleNamespace(
        last_seen=dict.fromkeys(("odom", "reference", "costmap", "target_speed",
                                "requested_drive_mode", "vehicle_mode", "planner_status"), 0.0),
        age=lambda _: 0.0,
        dynamic_topic_timeout=1.0,
        planner_status=planner_status,
        planner_section=lambda name: planner_status.get(name, {}),
    )


def test_safe_plan_failure_is_not_reported_as_ok():
    node = monitor({"state": "NO_SAFE_PLAN_SAFETY_STOP", "block_reason": "NO_SAFE_PLAN_SAFETY_STOP"})
    assert DebugPlotNode.diagnose(node)[0] == "PLANNER_BLOCKED"


def test_rotation_is_reported_as_maneuver_instead_of_mode_mismatch():
    node = monitor({"state": "SPOT_TURN_ROTATING", "spot_turn": {"state": "SPOT_TURN_ROTATING"}})
    assert DebugPlotNode.diagnose(node)[0] == "SPOT_TURN_ROTATING"
    visualizer = SimpleNamespace(vehicle_mode_state=None, spot_turn_state="SPOT_TURN_ALIGNING_REGULAR")
    assert "SPOT_TURN_ALIGNING_REGULAR" in VehicleVisualizerNode.drive_mode_status_text(visualizer)
