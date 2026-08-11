from simp_planner_tools.debug_signal_history import SourceTimeAligner


def test_source_timestamp_alignment_preserves_spacing() -> None:
    aligner = SourceTimeAligner()
    first = aligner.align(10, 0, 1.0)
    second = aligner.align(10, 100_000_000, 1.7)
    assert first == 1.0
    assert abs(second - 1.1) < 1.0e-12
