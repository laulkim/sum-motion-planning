from pathlib import Path


def test_execution_state_is_used_to_avoid_selected_path_fallback_during_safety_stop():
    source = (Path(__file__).resolve().parents[1] / "simp_planner_tools" / "debug_plot_node.py").read_text(encoding="utf-8")
    assert '"/planner/execution_state"' in source
    assert 'self.execution_state in {"SAFETY_STOP", "MODE_STOP", "MODE_WAIT"}' in source
    assert 'tracking_source = self.execution_state' in source
