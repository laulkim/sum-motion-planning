import csv
from pathlib import Path

import numpy as np

from simp_planner_tools.estimation_error_plot import (
    build_error_series,
    load_odom_csv,
    percentile_summary,
    resolve_input_paths,
    write_error_csv,
)


def make_data(
    time: list[float],
    x: list[float],
    y: list[float],
    yaw: list[float],
    speed: list[float],
    lateral: list[float],
) -> dict[str, np.ndarray]:
    values = {
        "time": time,
        "receive_time": time,
        "x": x,
        "y": y,
        "body_yaw": yaw,
        "speed": speed,
        "global_lateral_deviation": lateral,
    }
    return {name: np.asarray(column, dtype=float) for name, column in values.items()}


def test_build_error_series_interpolates_ground_truth() -> None:
    truth = make_data(
        [0.0, 1.0, 2.0],
        [0.0, 1.0, 2.0],
        [0.0, 0.0, 0.0],
        [0.0, 0.1, 0.2],
        [1.0, 1.0, 1.0],
        [0.0, 0.1, 0.2],
    )
    estimate = make_data(
        [0.5, 1.5],
        [0.6, 1.6],
        [0.2, 0.2],
        [0.06, 0.16],
        [1.1, 1.1],
        [0.07, 0.17],
    )

    result = build_error_series(estimate, truth)

    assert np.allclose(result["error_x_m"], 0.1)
    assert np.allclose(result["error_y_m"], 0.2)
    assert np.allclose(result["position_error_m"], np.hypot(0.1, 0.2))
    assert np.allclose(result["yaw_error_deg"], np.degrees(0.01))
    assert np.allclose(result["speed_error_mps"], 0.1)
    assert np.allclose(result["global_lateral_error_m"], 0.02)


def test_build_error_series_wraps_yaw_error() -> None:
    truth = make_data(
        [0.0, 1.0], [0.0, 0.0], [0.0, 0.0],
        [3.13, -3.13], [1.0, 1.0], [0.0, 0.0],
    )
    estimate = make_data(
        [0.5], [0.0], [0.0], [-3.13], [1.0], [0.0],
    )

    result = build_error_series(estimate, truth)

    assert abs(result["yaw_error_deg"][0]) < 1.0


def test_csv_round_trip_and_default_path_resolution(tmp_path: Path) -> None:
    header = [
        "time", "receive_time", "x", "y", "body_yaw", "speed",
        "global_lateral_deviation",
    ]
    for filename in ("odom_history_estimate.csv", "odom_history_ground_truth.csv"):
        with (tmp_path / filename).open("w", newline="", encoding="utf-8") as file:
            writer = csv.writer(file)
            writer.writerow(header)
            writer.writerow([0.0, 0.0, 1.0, 2.0, 0.1, 3.0, 0.2])

    estimate_path, truth_path, session_dir = resolve_input_paths(tmp_path, None)
    estimate = load_odom_csv(estimate_path)
    truth = load_odom_csv(truth_path)
    series = build_error_series(estimate, truth)
    output_path = tmp_path / "comparison.csv"
    write_error_csv(output_path, series)
    summary = percentile_summary(series, min_speed_mps=0.1)

    assert truth_path.name == "odom_history_ground_truth.csv"
    assert session_dir == tmp_path.resolve()
    assert output_path.read_text(encoding="utf-8").startswith("time_sec,")
    assert summary["position_cm"] == (0.0, 0.0, 0.0, 0.0)
