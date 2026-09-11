import csv
import math

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pytest

from simp_planner_tools.compare_tracking_runs import load_tracking_errors
from simp_planner_tools.compare_simulation_runs import compare_runs


def write_csv(path, rows):
    with path.open("w", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)


def make_run(directory, offset=0.0):
    directory.mkdir()
    # Odometry is offset in receipt-relative time on purpose. Only source
    # timestamps give the right comparison at t=.5, 1.5, ... .
    epoch = 1_789_000_000_000_000_000
    odom = [dict(time=i + .2, source_stamp_ns=epoch + i * 1_000_000_000,
                 scenario="stadium", x=0., y=float(i), body_yaw=math.pi / 2,
                 speed=1., beta=0., global_lateral_deviation=0.,
                 global_direction_deviation=0.) for i in range(5)]
    commands = [dict(publish_time=i + .5, source_stamp_ns=epoch + i * 1_000_000_000 + 500_000_000,
                     vx=1., vy=0., yaw_rate=0., reference_valid=True,
                     reference_x=-.2 - offset, reference_y=i + .6 + offset,
                     reference_chi_rad=math.pi / 2,
                     reference_body_yaw_rad=math.pi / 2 + .03) for i in range(4)]
    write_csv(directory / "odom_history.csv", odom)
    write_csv(directory / "command_history.csv", commands)
    return odom, commands


def test_own_target_uses_source_stamps_and_reference_frame(tmp_path):
    make_run(tmp_path / "run")
    result = load_tracking_errors(tmp_path / "run")
    np.testing.assert_allclose(result["longitudinal_m"], .1, atol=1e-12)
    np.testing.assert_allclose(result["lateral_m"], .2, atol=1e-12)
    np.testing.assert_allclose(result["heading_deg"], math.degrees(.03), atol=1e-12)
    np.testing.assert_allclose(result["odom_y"], [.5, 1.5, 2.5, 3.5])
    np.testing.assert_allclose(result["reference_y"], [.6, 1.6, 2.6, 3.6])


def test_invalid_target_is_gap_and_replan_keeps_its_own_target(tmp_path):
    directory = tmp_path / "run"
    odom, commands = make_run(directory)
    commands[1]["reference_valid"] = False
    commands[2]["reference_y"] += 1.0  # new plan, no target interpolation across plans
    commands[-1]["source_stamp_ns"] += 2_000_000_000  # beyond odometry: do not extrapolate
    write_csv(directory / "command_history.csv", commands)
    result = load_tracking_errors(directory)
    assert math.isnan(result["longitudinal_m"][1])
    assert result["longitudinal_m"][2] == pytest.approx(1.1)
    assert math.isnan(result["longitudinal_m"][-1])
    assert math.isnan(result["reference_x"][1])
    assert math.isnan(result["odom_x"][-1])


def test_heading_interpolates_across_pi_then_wraps_error(tmp_path):
    directory = tmp_path / "run"
    odom, commands = make_run(directory)
    for row in odom:
        row["body_yaw"] = math.pi - .02 if row["y"] == 0 else -math.pi + .02
    commands[0]["reference_body_yaw_rad"] = -math.pi + .01
    write_csv(directory / "odom_history.csv", odom)
    write_csv(directory / "command_history.csv", commands)
    result = load_tracking_errors(directory)
    assert result["heading_deg"][0] == pytest.approx(math.degrees(.01))
    assert result["reference_yaw_deg"][0] - result["odom_yaw_deg"][0] == pytest.approx(math.degrees(.01))


def test_show_opens_old_and_new_figures_together(tmp_path, monkeypatch):
    for name, offset in (("ideal", 0.), ("off", .2), ("on", .05)):
        odom, _ = make_run(tmp_path / name, offset)
        truth = [dict(row, y=row["y"] - .4, body_yaw=row["body_yaw"] - .02) for row in odom]
        write_csv(tmp_path / name / "ground_truth_history.csv", truth)
    shown = []
    monkeypatch.setattr(plt, "show", lambda: shown.append(len(plt.get_fignums())))
    output = tmp_path / "output"
    compare_runs(tmp_path / "ideal", tmp_path / "on", output,
                 show=True, noisy_off_directory=tmp_path / "off")
    assert shown == [9]
    for name in ("ideal_noisy_overlay.png", "ideal_noisy_errors.png",
                 "tracking_errors.png", "tracking_error_metrics.png", "tracking_metrics.json",
                 "ideal_target_feedback.png", "noisy_target_feedback.png",
                 "noisy_trajectories.png", "noisy_state_errors.png", "noisy_state_error_metrics.png"):
        assert (output / name).is_file()
    import json
    metrics = json.loads((output / "tracking_metrics.json").read_text())
    assert metrics["runs"]["noisy P OFF"]["longitudinal_m"]["rmse"] == pytest.approx(.3)
    assert metrics["runs"]["noisy P ON"]["longitudinal_m"]["rmse"] == pytest.approx(.15)
    metrics = json.loads((output / "noisy_state_metrics.json").read_text())
    for label, expected in (("P OFF", .7), ("P ON", .55)):
        run = metrics["runs"][label]
        assert run["Target - truth"]["longitudinal_m"]["rmse"] == pytest.approx(expected)
        assert run["Estimate - truth"]["longitudinal_m"]["rmse"] == pytest.approx(.4)
        assert run["Estimate - truth"]["heading_deg"]["rmse"] == pytest.approx(math.degrees(.02))


def test_old_two_run_comparison_still_has_two_windows(tmp_path, monkeypatch):
    for name in ("ideal", "noisy"):
        make_run(tmp_path / name)
    shown = []
    monkeypatch.setattr(plt, "show", lambda: shown.append(len(plt.get_fignums())))
    compare_runs(tmp_path / "ideal", tmp_path / "noisy", None, show=True)
    assert shown == [2]
