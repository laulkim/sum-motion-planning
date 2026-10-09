from __future__ import annotations

import json
from pathlib import Path
from types import SimpleNamespace

from simp_planner_tools import planning_call_count_report as report


def _sample(time_s: float, compute_ms: float, count: int) -> dict[str, float | int]:
    return {
        "time_s": time_s,
        "total_compute_time_ms": compute_ms,
        "spatial_path_generation_calls": count,
        "trajectory_planning_calls": count + 1,
        "allocation_calls": count + 2,
    }


def test_renderer_uses_elapsed_time_and_overlays_planning_compute_time(
    tmp_path: Path, monkeypatch,
) -> None:
    captured: dict[str, object] = {}
    original_subplots = report.plt.subplots

    def capture_subplots(*args, **kwargs):
        figure, axis = original_subplots(*args, **kwargs)
        captured["figure"] = figure
        captured["count_axis"] = axis
        return figure, axis

    monkeypatch.setattr(report.plt, "subplots", capture_subplots)
    output_path = tmp_path / "planning_call_counts.png"
    samples = [
        _sample(0.25, 8.0, 1),
        _sample(0.90, 12.5, 2),
        _sample(2.10, 9.5, 3),
    ]

    report.render_call_count_report(samples, output_path)

    figure = captured["figure"]
    count_axis = captured["count_axis"]
    assert output_path.exists()
    assert count_axis.get_xlabel() == "time [s]"
    assert len(figure.axes) == 2

    expected_time = [0.25, 0.90, 2.10]
    for line in count_axis.lines:
        assert list(line.get_xdata()) == expected_time

    compute_axis = next(axis for axis in figure.axes if axis is not count_axis)
    assert compute_axis.get_ylabel() == "planning computation time [ms]"
    assert len(compute_axis.lines) == 1
    assert list(compute_axis.lines[0].get_xdata()) == expected_time
    assert list(compute_axis.lines[0].get_ydata()) == [8.0, 12.5, 9.5]


def test_status_callback_records_counts_compute_time_and_elapsed_time() -> None:
    fake_node = SimpleNamespace(samples=[], elapsed=lambda: 4.75)
    message = SimpleNamespace(
        data=json.dumps(
            {
                "plan": {
                    "total_compute_time_ms": 13.25,
                    "spatial_path_generation_calls": 2,
                    "trajectory_planning_calls": 3,
                    "allocation_calls": 4,
                }
            }
        )
    )

    report.PlanningCallCountReportNode.planner_status_callback(fake_node, message)

    assert fake_node.samples == [
        {
            "time_s": 4.75,
            "total_compute_time_ms": 13.25,
            "spatial_path_generation_calls": 2,
            "trajectory_planning_calls": 3,
            "allocation_calls": 4,
        }
    ]


def test_status_callback_ignores_status_without_compute_time() -> None:
    fake_node = SimpleNamespace(samples=[], elapsed=lambda: 1.0)
    message = SimpleNamespace(
        data=json.dumps(
            {
                "plan": {
                    "spatial_path_generation_calls": 1,
                    "trajectory_planning_calls": 1,
                    "allocation_calls": 1,
                }
            }
        )
    )

    report.PlanningCallCountReportNode.planner_status_callback(fake_node, message)

    assert fake_node.samples == []
