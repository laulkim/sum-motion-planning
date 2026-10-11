import json
import math
from pathlib import Path

import pytest

from simp_planner_tools.gain_evaluation import aggregate, case_objective, load_manifest, metrics, validate_gains
from simp_planner_tools.gain_experiment import launch_command, main, optimize
from simp_planner_tools.gain_lut import active_points, axis_weights, load_lut, save_lut, smoothness, validate_axes

CONFIG = Path(__file__).resolve().parents[1] / "experiments/gain_smoke.json"


def test_metrics_and_normalized_objective():
    rows = [dict(valid=1, ex=3, ey=4, etheta=0.2, vx_cmd=i, vy_cmd=2*i, omega_cmd=3*i,
                 vx_ref=0, vy_ref=0, omega_ref=1) for i in range(3)]
    result = metrics(rows)
    assert result["position_rmse"] == 5
    assert result["rmse_ex"] == result["max_abs_ex"] == 3
    assert result["rms_delta_vx_cmd"] == 1
    assert result["rms_delta_vy_cmd"] == 2
    assert result["rms_delta_yaw_rate_cmd"] == 3
    config = load_manifest(CONFIG)["objective"]
    config["scales"] = dict(ex_m=3, ey_m=4, etheta_rad=0.2,
                            delta_vx_mps=1, delta_vy_mps=2, delta_yaw_rate_radps=3)
    objective, parts = case_objective(result, config)
    assert parts == {"position": 1, "yaw": 1, "control": 1}
    assert objective == 2.1
    assert result["spot_samples"] == 3
    with pytest.raises(ValueError):
        metrics([])
    rows[1]["ex"] = math.nan
    with pytest.raises(ValueError):
        metrics(rows)


def test_aggregation_is_case_weighted_not_sample_weighted():
    assert aggregate([{"case": {"weight": 1}, "objective": 2, "samples": 10000},
                      {"case": {"weight": 3}, "objective": 6, "samples": 10}]) == 5


def test_manifest_and_seed_protocol(tmp_path):
    config = load_manifest(CONFIG)
    case = config["cases"][0]
    command = launch_command(config, case, [1, 2, 1], tmp_path)
    assert command == launch_command(config, case, [1, 2, 1], tmp_path)
    assert "sensor_delay_sec:=0.0" in command
    assert "sensor_noise_seed:=42" in command
    assert "sensor_noise_enabled:=true" in command
    assert "kx:=1" in command and "ky:=2" in command and "ktheta:=1" in command
    assert not any("initial_" in arg for arg in command)
    assert "sensor_noise_seed:=42" in launch_command(config, case, [2, 3, 4], tmp_path)
    config["settings"]["sensor_delay_sec"] = 0.1
    path = tmp_path / "bad.json"
    path.write_text(json.dumps(config))
    with pytest.raises(ValueError):
        load_manifest(path)
    for gain in ([0, 2, 1], [-1, 2, 1], [1, math.inf, 1], [1, 2, math.nan]):
        with pytest.raises(ValueError):
            validate_gains(gain)
    config = load_manifest(CONFIG)
    config["cases"].append(config["cases"][0].copy())
    path.write_text(json.dumps(config))
    with pytest.raises(ValueError):
        load_manifest(path)


def test_lut_initialization_active_neighborhoods_and_smoothness(tmp_path):
    axes = {"vx": [-2, 0, 3], "vy": [0, 2], "yaw_rate": [-1, 0, 0.3]}
    path = tmp_path / "lut.csv"
    save_lut(path, axes, [[1, 2, 3] for _ in range(18)])
    assert load_lut(path) == (axes, [[1, 2, 3] for _ in range(18)])
    assert axis_weights(axes["vx"], -100) == [(0, 1)]
    assert axis_weights(axes["vx"], 100) == [(2, 1)]
    assert axis_weights(axes["vx"], 1.5) == [(1, 0.5), (2, 0.5)]
    samples = tmp_path / "samples.csv"
    samples.write_text("vx,vy,yaw_rate\n1.5,1,0.15\n")
    assert len(active_points(axes, samples)) == 8
    assert smoothness(axes, [[1, 2, 3] for _ in range(18)], 1) == 0
    with pytest.raises(ValueError):
        validate_axes({**axes, "vx": [0, 0]})
    with pytest.raises(ValueError):
        validate_axes({**axes, "vy": [math.nan]})
    path.write_text("vx,vy,yaw_rate,kx,ky,ktheta\n0,0,0,1,2,nan\n")
    with pytest.raises(ValueError):
        load_lut(path)
    best = tmp_path / "best.json"
    best.write_text(json.dumps({"policy": "fixed", "gains": [1, 2, 3], "success": True, "sensor_delay_sec": 0}))
    grid = tmp_path / "grid.json"
    grid.write_text(json.dumps({"axes": axes}))
    main(["init-lut", "--axes", str(grid), "--fixed-result", str(best), "--output", str(path)])
    assert load_lut(path)[1] == [[1, 2, 3] for _ in range(18)]


def test_optimizers_checkpoint_cache_and_validation_exclusion(tmp_path, monkeypatch):
    import simp_planner_tools.gain_experiment as experiment
    config = load_manifest(CONFIG)
    config["cases"].append({**config["cases"][0], "id": "held_out", "role": "validation"})
    calls = []
    monkeypatch.setattr(experiment, "load_references", lambda path, config: ([], {"test": "references"}))
    def evaluation(config, datasets, gains, output, lut=None):
        calls.append(("lut" if lut else "fixed", "tuning"))
        objective = (gains[0] - 1) ** 2
        if lut:
            objective = (lut[1][0][0] - 1) ** 2
        return {"objective": objective, "success": True, "elapsed_wall_sec": 0.001}
    monkeypatch.setattr(experiment, "evaluate_offline", evaluation)
    monkeypatch.setattr(experiment, "evaluate", lambda *args, **kwargs: pytest.fail("optimizer launched ROS"))
    fixed = tmp_path / "fixed"
    optimize(config, fixed, "references.json")
    assert all(role == "tuning" for _, role in calls)
    assert json.loads((fixed / "configuration.json").read_text())["backend"] == "offline_replay"
    assert json.loads((fixed / "best.json").read_text())["gains"][0] == 1
    before = len(calls)
    optimize(config, fixed, "references.json")
    assert len(calls) == before  # exact cached trials survive restart
    (fixed / "best.json").unlink()
    with (fixed / "history.jsonl").open("a") as file:
        file.write('{"interrupted":')
    optimize(config, fixed, "references.json")
    assert (fixed / "best.json").exists()
    assert len(calls) == before
    legacy = tmp_path / "legacy_ros"
    legacy.mkdir()
    (legacy / "configuration.json").write_text(json.dumps({"policy": "fixed", "provenance": {}}))
    with pytest.raises(ValueError, match="separate offline output"):
        optimize(config, legacy, "references.json")
    lut = tmp_path / "initial.csv"
    save_lut(lut, {name: [0] for name in ("vx", "vy", "yaw_rate")}, [[1, 2, 1]])
    samples = tmp_path / "samples.csv"
    samples.write_text("vx,vy,yaw_rate\n0,0,0\n")
    optimize(config, tmp_path / "lut", "references.json", lut, samples)
    assert (tmp_path / "lut/optimized_lut.csv").exists()
    assert json.loads((tmp_path / "lut/checkpoint.json").read_text())["sweep"] == 1
    assert json.loads((tmp_path / "lut/active_points.json").read_text())["active"] == [0]


def test_compare_rejects_unfair_protocol_and_keeps_metrics(tmp_path):
    from simp_planner_tools.gain_experiment import compare
    config = load_manifest(CONFIG)
    result = {"case": config["cases"][0], "success": True, "objective": 2,
              "metrics": {"rmse_ex": 0.1, "max_abs_ey": 0.2, "spot_samples": 0}}
    fixed = {"policy": "fixed", "role": "validation", "sensor_delay_sec": 0,
             "configuration": {"config": config, "provenance": {"common_signature": "same"}},
             "objective": 2, "results": [result]}
    lut = json.loads(json.dumps(fixed))
    lut["policy"] = "lut"
    a, b = tmp_path / "fixed.json", tmp_path / "lut.json"
    a.write_text(json.dumps(fixed)); b.write_text(json.dumps(lut))
    compare(a, b, tmp_path / "report")
    assert "fixed_rmse_ex" in (tmp_path / "report/comparison.csv").read_text()
    lut["configuration"]["provenance"]["common_signature"] = "different_planner"
    b.write_text(json.dumps(lut))
    with pytest.raises(ValueError):
        compare(a, b, tmp_path / "report")
