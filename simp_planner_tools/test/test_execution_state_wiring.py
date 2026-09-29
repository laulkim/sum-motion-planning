from pathlib import Path


def test_debug_tracking_uses_timestamp_reference_without_execution_state_fallback():
    source = (Path(__file__).resolve().parents[1] / "simp_planner_tools" / "debug_plot_node.py").read_text(encoding="utf-8")
    assert '"/planner/tracking_trajectory"' in source
    assert 'sample_row(message, self.latest_trajectory' in source
    assert 'SELECTED_PATH_FALLBACK' not in source
    assert 'EXECUTED_COMMAND_SEGMENT' not in source
