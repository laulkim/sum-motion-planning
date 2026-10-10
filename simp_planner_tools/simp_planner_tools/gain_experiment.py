"""Manifest-driven real ROS experiments, fixed/LUT optimization and reports.

Run with python3 -m simp_planner_tools.gain_experiment --help after sourcing ROS.
No planner, simulator, pose initialization or reference generation is replaced.
"""
import argparse
import copy
import csv
import hashlib
import inspect
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

from .gain_evaluation import (GAIN_NAMES, aggregate, case_objective, load_manifest,
                              metrics, operating_report, read_tracking, validate_gains, write_json)
from .gain_lut import (AXIS_NAMES, active_points, load_lut, save_lut, smoothness, validate_axes)


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, allow_nan=False).encode()).hexdigest()


def provenance():
    from ament_index_python.packages import get_package_prefix, get_package_share_directory
    binaries = {}
    for package, name in (("simp_controller", "tracking_controller_node"),
                          ("simp_planner_cpp", "planner_node_cpp"),
                          ("planar_velocity_sim", "planar_velocity_sim_node")):
        path = Path(get_package_prefix(package)) / "lib" / package / name
        binaries[package] = hashlib.sha256(path.read_bytes()).hexdigest()
    # Python entry-point executables do not include their module source.
    import planar_velocity_sim.planar_velocity_sim_node as sim
    import planar_velocity_sim.vehicle_response as response
    from . import scenario_definition, scenario_manager_node, tracking_shutdown_plot, gain_evaluation, gain_lut
    modules = (sim, response, scenario_definition, scenario_manager_node, tracking_shutdown_plot, gain_evaluation, gain_lut)
    sources = {module.__name__: hashlib.sha256(Path(module.__file__).read_bytes()).hexdigest() for module in modules}
    sources["experiment_runner"] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    share = Path(get_package_share_directory("simp_planner_tools"))
    launch = share / "launch/simulation.launch.py"
    sources["simulation_launch"] = hashlib.sha256(launch.read_bytes()).hexdigest()
    for prefix, directory, pattern in (("sim", Path(sim.__file__).parent, "*.py"),
                                       ("tools", Path(__file__).parent, "*.py"),
                                       ("maps", share / "maps", "*.csv")):
        for path in sorted(directory.rglob(pattern)):
            sources[prefix + "/" + str(path.relative_to(directory))] = hashlib.sha256(path.read_bytes()).hexdigest()
    result = subprocess.run(["git", "rev-parse", "HEAD"], text=True, capture_output=True)
    return {"commit": result.stdout.strip(), "binaries": binaries, "sources": sources,
            "common_signature": digest({"planner": binaries["simp_planner_cpp"], "sources": sources})}


def launch_command(config, case, gains, output, lut_file=None):
    validate_gains(gains)
    settings = config["settings"]
    if settings["sensor_delay_sec"] != 0.0:
        raise ValueError("sensor_delay_sec must be 0.0")
    values = {**settings, "scenario": case["scenario"], "target_speed": case["target_speed"],
              "sensor_noise_seed": case["sensor_noise_seed"], "sensor_delay_sec": 0.0,
              "use_rviz": False, "experiment_mode": True, "debug_output_dir": str(Path(output).resolve()),
              **dict(zip(GAIN_NAMES, gains))}
    if lut_file:
        values["gain_lut_file"] = str(Path(lut_file).resolve())
    return ["ros2", "launch", "simp_planner_tools", "simulation.launch.py", *[
        f"{name}:={str(value).lower() if isinstance(value, bool) else value}" for name, value in values.items()]]


def stop_launch(process):
    if process.poll() is None:
        process.send_signal(signal.SIGINT)  # launch forwards it once to its children
        try:
            process.wait(timeout=15)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()


def read_json_if_exists(path):
    return json.loads(path.read_text()) if path.exists() else {}


def stopped_pose(directory, hold_sec, speed_limit):
    path = directory / "odometry.csv"
    if not path.exists():
        return False, None
    with path.open() as file:
        rows = list(csv.DictReader(file))
    if len(rows) < 2:
        return False, None
    end = int(rows[-1]["stamp_ns"])
    recent = [r for r in rows if int(r["stamp_ns"]) >= end - hold_sec * 1e9]
    if len(recent) < 2 or (end - int(recent[0]["stamp_ns"])) * 1e-9 < hold_sec * 0.9:
        return False, None
    def distance(a, b):
        return math.hypot(float(b["x"]) - float(a["x"]), float(b["y"]) - float(a["y"]))
    # Pose has no injected noise; velocity fields DO, so do not test those for rest.
    travel = sum(distance(a, b) for a, b in zip(recent, recent[1:]))
    progress = sum(distance(a, b) for a, b in zip(rows, rows[1:]))
    return travel / ((end - int(recent[0]["stamp_ns"])) * 1e-9) <= speed_limit, progress


def run_case(config, case, gains, output, policy, domain, lut_file=None):
    output = Path(output)
    if list(output.glob("tracking_*")):
        # Preserve interrupted logs, but never use their stale completion state.
        output.rename(output.with_name(output.name + f".interrupted_{time.time_ns()}"))
    output.mkdir(parents=True, exist_ok=True)
    command = launch_command(config, case, gains, output, lut_file)
    write_json(output / "command.json", {"argv": command, "sensor_delay_sec": 0.0, "case": case})
    env = os.environ.copy()
    env.update(ROS_DOMAIN_ID=str(domain), ROS_AUTOMATIC_DISCOVERY_RANGE="LOCALHOST",
               ROS_LOG_DIR=str(output / "ros_logs"))
    env.pop("DISPLAY", None)
    started = time.monotonic()
    reason, directory, status, planner = "timeout", None, {}, {}
    node_died = False
    with (output / "launch.log").open("w") as log:
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, env=env, start_new_session=True)
        try:
            while time.monotonic() - started < case["timeout_sec"]:
                if "process has died" in (output / "launch.log").read_text():
                    node_died = True
                    reason = "node_died"
                    break
                if process.poll() is not None:
                    reason = "launch_exited"
                    break
                candidates = sorted(output.glob("tracking_*"))
                if candidates:
                    directory = candidates[-1]
                    progress = read_json_if_exists(directory / "progress.json")
                    status = read_json_if_exists(directory / "scenario_status.json")
                    planner = read_json_if_exists(directory / "planner_status.json")
                    if case["end_condition"] == "duration" and (progress.get("valid_elapsed_s") or 0) >= case["duration_sec"]:
                        reason = "duration"
                        break
                    if case["end_condition"] == "complete" and status.get("state") == "COMPLETE":
                        reason = "complete"
                        break
                    if case["end_condition"] == "safe_stop" and planner.get("state") == "NO_SAFE_PLAN_SAFETY_STOP":
                        stopped, progress_m = stopped_pose(directory, case["stop_hold_sec"], case["stop_speed_mps"])
                        if stopped and progress_m <= case["maximum_progress_m"]:
                            reason = "safe_stop"
                            break
                time.sleep(0.1)
        finally:
            stop_launch(process)
    result = {"case": case, "sensor_delay_sec": 0.0, "policy": policy, "reason": reason,
              "elapsed_wall_sec": time.monotonic() - started, "status": status, "planner_status": planner,
              "metrics": {}, "objective": config["objective"]["failure_penalty"], "success": False}
    log_text = (output / "launch.log").read_text()
    if f"gain_policy={policy}" not in log_text:
        raise RuntimeError("installed controller policy does not match requested policy; rebuild the branch")
    if node_died:
        result["reason"] = "node_died"
    if directory and (directory / "tracking.csv").exists():
        result["tracking_csv"] = str((directory / "tracking.csv").resolve())
        try:
            metric = metrics(read_tracking(result["tracking_csv"]))
            result["metrics"] = metric
            objective, parts = case_objective(metric, config["objective"])
            result["objective_parts"] = parts
            valid = (reason == case["end_condition"] and result["reason"] != "node_died" and
                     metric["valid_samples"] >= config["objective"]["minimum_samples"] and
                     metric["valid_fraction"] >= config["objective"]["minimum_valid_fraction"] and
                     all(metric.get(k, math.inf) <= limit for k, limit in case.get("metric_limits", {}).items()))
            if policy == "lut" and (metric["spot_max_abs_vx_cmd"] != 0 or metric["spot_max_abs_vy_cmd"] != 0):
                valid = False
                result["reason"] = "spot_constraint_violation"
            result["success"] = valid
            if valid:
                result["objective"] = objective
        except (ValueError, OverflowError) as error:
            result["metric_error"] = str(error)
    # Status publishers may use NaN for a not-yet-observed value.
    def finite_json(value):
        if isinstance(value, dict):
            return {k: finite_json(v) for k, v in value.items()}
        if isinstance(value, list):
            return [finite_json(v) for v in value]
        return None if isinstance(value, float) and not math.isfinite(value) else value
    if case["role"] == "safety" and reason == "safe_stop" and not node_died:
        # A stop-only safety case need not have a usable tracking trajectory.
        result["success"] = True
        result["objective"] = None
        result["safety_check"] = "planner safety stop + pose-derived rest + maximum travel; no swept collision certification"
    result = finite_json(result)
    write_json(output / "result.json", result)
    return result


def evaluate(config, gains, output, policy, domain, role="tuning", lut_file=None):
    selected = [c for c in config["cases"] if c["role"] == role]
    if not selected:
        raise ValueError(f"manifest contains no {role} cases")
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    identity = {"config": config, "gains": gains, "policy": policy, "role": role,
                "lut_sha256": hashlib.sha256(Path(lut_file).read_bytes()).hexdigest() if lut_file else None,
                "provenance": provenance(), "sensor_delay_sec": 0.0}
    metadata = output / "configuration.json"
    if metadata.exists() and read_json_if_exists(metadata) != identity:
        raise ValueError("output directory belongs to a different evaluation")
    write_json(metadata, identity)
    results = []
    for case in selected:
        result_path = output / case["id"] / "result.json"
        result = read_json_if_exists(result_path) if result_path.exists() else run_case(
            config, case, gains, output / case["id"], policy, domain, lut_file)
        results.append(result)
        print(f"{case['id']}: {result['reason']} success={result['success']} J={result['objective']}", flush=True)
    summary = {"policy": policy, "role": role, "sensor_delay_sec": 0.0, "gains": gains,
               "objective": aggregate(results) if role != "safety" else None, "success": all(r["success"] for r in results),
               "configuration": identity, "results": results}
    write_json(output / "summary.json", summary)
    flat = [{"case_id": r["case"]["id"], "scenario": r["case"]["scenario"], "target_speed": r["case"]["target_speed"],
             "seed": r["case"]["sensor_noise_seed"], "success": r["success"], "reason": r["reason"],
             "objective": r["objective"], **r["metrics"]} for r in results]
    columns = list(dict.fromkeys(k for row in flat for k in row))
    with (output / "case_metrics.csv").open("w", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=columns)
        writer.writeheader()
        writer.writerows(flat)
    return summary


def de_options(config, seed):
    from scipy.optimize import differential_evolution
    options = {key: config[key] for key in ("maxiter", "popsize", "tol")}
    options.update(polish=False, workers=1)
    options["rng" if "rng" in inspect.signature(differential_evolution).parameters else "seed"] = seed
    return options


def read_history(path):
    """Recover a trailing interrupted append; corruption elsewhere is an error."""
    if not path.exists():
        return []
    lines = path.read_text().splitlines()
    rows = []
    for index, line in enumerate(lines):
        try:
            rows.append(json.loads(line))
        except json.JSONDecodeError:
            if index != len(lines) - 1:
                raise
            path.write_text("".join(json.dumps(row) + "\n" for row in rows))
    return rows


def optimize(config, output, domain, initial_lut=None, samples=None):
    from scipy.optimize import differential_evolution
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    policy = "lut" if initial_lut else "fixed"
    settings = config["optimizer"]
    identity = {"manifest": config, "policy": policy, "provenance": provenance(), "sensor_delay_sec": 0.0,
                "initial_lut": hashlib.sha256(Path(initial_lut).read_bytes()).hexdigest() if initial_lut else None,
                "samples": hashlib.sha256(Path(samples).read_bytes()).hexdigest() if samples else None}
    configuration = output / "configuration.json"
    if configuration.exists() and read_json_if_exists(configuration) != identity:
        raise ValueError("resume requires identical manifest, controller build and input files")
    write_json(configuration, identity)
    history_path = output / "history.jsonl"
    history = read_history(history_path)
    cache = {row["candidate_key"]: row for row in history}
    checkpoint_path = output / "checkpoint.json"
    checkpoint = read_json_if_exists(checkpoint_path)
    bounds = [settings["bounds"][name] for name in GAIN_NAMES]

    def save_best():
        feasible = [r for r in history if r["success"]]
        if feasible:
            best = min(feasible, key=lambda r: r["objective"])
            write_json(output / "best.json", {**best, "policy": policy, "configuration": identity})
            if policy == "lut":
                best_axes, best_gains = load_lut(best["lut_file"])
                save_lut(output / "optimized_lut.csv", best_axes, best_gains)

    # Recover best if interruption occurred after history fsync but before its
    # atomic best/checkpoint write. History is the durable source of truth.
    save_best()

    def trial(gains, lut_gains=None, axes=None):
        validate_gains(gains)
        key = digest(lut_gains if lut_gains is not None else gains)
        if key in cache:
            return cache[key]
        directory = output / "trials" / key
        lut_file = None
        if lut_gains is not None:
            lut_file = directory / "candidate_lut.csv"
            save_lut(lut_file, axes, lut_gains)
        result = evaluate(config, gains, directory, policy, domain, lut_file=lut_file)
        penalty = smoothness(axes, lut_gains, settings["smoothness_lambda"]) if lut_gains is not None else 0.0
        row = {"trial": len(history), "candidate_key": key, "gains": list(gains),
               "objective": result["objective"] + penalty, "tracking_objective": result["objective"],
               "smoothness": penalty, "success": result["success"], "directory": str(directory.resolve()),
               "sensor_delay_sec": 0.0}
        if lut_gains is not None:
            row["lut_file"] = str(lut_file.resolve())
        with history_path.open("a") as file:
            file.write(json.dumps(row, allow_nan=False) + "\n")
            file.flush()
            os.fsync(file.fileno())
        with (output / "history.csv").open("a", newline="") as file:
            writer = csv.writer(file)
            if file.tell() == 0:
                writer.writerow(("trial", "candidate_key", *GAIN_NAMES, "objective", "tracking_objective", "smoothness", "success"))
            writer.writerow((row["trial"], key, *gains, row["objective"], row["tracking_objective"], penalty, row["success"]))
        history.append(row)
        cache[key] = row
        save_best()
        return row

    if not initial_lut:
        initial = checkpoint.get("gains", settings["initial_gains"])
        validate_gains(initial)
        def objective(gains):
            row = trial(list(map(float, gains)))
            best_path = output / "best.json"
            if best_path.exists():
                write_json(checkpoint_path, {"gains": read_json_if_exists(best_path)["gains"],
                                           "completed_trials": len(history), "sensor_delay_sec": 0.0,
                                           "random_seed": settings["random_seed"]})
            return row["objective"]
        # Resume is a seeded DE restart from best + persistent exact candidate
        # cache, not a claim of restoring SciPy's private population internals.
        result = differential_evolution(objective, bounds, x0=initial,
                                        **de_options(settings, settings["random_seed"]))
        write_json(output / "optimizer_result.json", {"x": result.x.tolist(), "fun": float(result.fun),
                   "message": str(result.message), "sensor_delay_sec": 0.0})
    else:
        axes, initial_gains = load_lut(initial_lut)
        active = active_points(axes, samples)
        write_json(output / "active_points.json", {"active": active, "inactive": [i for i in range(len(initial_gains)) if i not in active],
                                                   "axes": axes, "sensor_delay_sec": 0.0})
        gains = checkpoint.get("lut_gains", initial_gains)
        start_sweep, start_block = checkpoint.get("sweep", 0), checkpoint.get("block", 0)
        nominal = settings["initial_gains"]
        current = trial(nominal, gains, axes)
        for sweep in range(start_sweep, settings["max_sweeps"]):
            before = checkpoint.get("sweep_start_objective", current["objective"]) if sweep == start_sweep else current["objective"]
            for block in range(start_block if sweep == start_sweep else 0, len(active)):
                point = active[block]
                write_json(checkpoint_path, {"sweep": sweep, "block": block, "lut_gains": gains,
                           "sweep_start_objective": before, "completed_trials": len(history),
                           "random_seed": settings["random_seed"], "sensor_delay_sec": 0.0})
                def objective(values):
                    candidate = copy.deepcopy(gains)
                    candidate[point] = list(map(float, values))
                    return trial(nominal, candidate, axes)["objective"]
                result = differential_evolution(objective, bounds, x0=gains[point],
                         **de_options(settings["block_de"], settings["random_seed"] + sweep * len(active) + block))
                candidate = copy.deepcopy(gains)
                candidate[point] = result.x.tolist()
                row = trial(nominal, candidate, axes)
                if row["success"] and (not current["success"] or row["objective"] < current["objective"]):
                    gains, current = candidate, row
                write_json(checkpoint_path, {"sweep": sweep, "block": block + 1, "lut_gains": gains,
                           "sweep_start_objective": before, "completed_trials": len(history),
                           "random_seed": settings["random_seed"], "sensor_delay_sec": 0.0})
            improvement = before - current["objective"]
            write_json(checkpoint_path, {"sweep": sweep + 1, "block": 0, "lut_gains": gains,
                       "sweep_start_objective": current["objective"], "completed_trials": len(history),
                       "random_seed": settings["random_seed"], "sensor_delay_sec": 0.0})
            if improvement <= settings["sweep_tolerance"]:
                break
        write_json(output / "optimizer_result.json", {"objective": current["objective"], "sensor_delay_sec": 0.0,
                                                      "completed_trials": len(history)})
    if not (output / "best.json").exists():
        raise RuntimeError("no successful candidate: inspect trials; failure penalties are not an optimum")


def compare(fixed, lut, output):
    a, b = read_json_if_exists(Path(fixed)), read_json_if_exists(Path(lut))
    if a["policy"] != "fixed" or b["policy"] != "lut" or a["role"] != b["role"]:
        raise ValueError("need fixed and LUT summaries for the same role")
    ca, cb = a["configuration"], b["configuration"]
    if ca["config"] != cb["config"] or ca["provenance"]["common_signature"] != cb["provenance"]["common_signature"]:
        raise ValueError("unfair comparison: manifest/planner/vehicle/scenario mismatch")
    right = {r["case"]["id"]: r for r in b["results"]}
    records = []
    for left in a["results"]:
        other = right[left["case"]["id"]]
        if left["case"] != other["case"]:
            raise ValueError("case mismatch")
        row = {"case_id": left["case"]["id"], "fixed_success": left["success"], "lut_success": other["success"],
               "fixed_objective": left["objective"], "lut_objective": other["objective"]}
        for key in set(left["metrics"]) | set(other["metrics"]):
            row["fixed_" + key] = left["metrics"].get(key)
            row["lut_" + key] = other["metrics"].get(key)
        records.append(row)
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    columns = list(dict.fromkeys(k for row in records for k in row))
    with (output / "comparison.csv").open("w", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=columns)
        writer.writeheader()
        writer.writerows(records)
    write_json(output / "comparison.json", {"sensor_delay_sec": 0.0, "role": a["role"],
               "fixed_objective": a["objective"], "lut_objective": b["objective"], "cases": records})
    for left in a["results"]:
        other = right[left["case"]["id"]]
        if not max(left["metrics"].get("spot_samples", 0), other["metrics"].get("spot_samples", 0)):
            continue
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        figure, axes = plt.subplots(3, 1, sharex=True, figsize=(10, 7), layout="constrained")
        for result, label in ((left, "Fixed"), (other, "LUT")):
            rows = read_tracking(result["tracking_csv"])
            turns, turn = [], []
            for row in rows:
                pure = row["valid"] == 1 and row["vx_ref"] == 0 and row["vy_ref"] == 0 and row["omega_ref"] != 0
                if pure:
                    turn.append(row)
                elif turn:
                    turns.append(turn)
                    turn = []
            if turn:
                turns.append(turn)
            for number, turn in enumerate(turns, 1):
                for ax, key, scale in zip(axes, ("vx_cmd", "vy_cmd", "etheta"), (1, 1, 180 / math.pi)):
                    ax.plot([r["time_s"] - turn[0]["time_s"] for r in turn],
                            [r[key] * scale for r in turn], label=f"{label} turn {number}",
                            color="tab:blue" if label == "Fixed" else "tab:orange")
                    ax.legend()
        for ax, label in zip(axes, ("vx command [m/s]", "vy command [m/s]", "yaw error [deg]")):
            ax.set_ylabel(label)
            ax.grid(alpha=0.3)
        axes[-1].set_xlabel("Time from each pure-rotation reference interval start [s]")
        figure.suptitle(left["case"]["id"] + " — pure-rotation reference samples")
        figure.savefig(output / (left["case"]["id"] + "_spot_commands.png"), dpi=160)
        plt.close(figure)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    for command in ("evaluate", "collect", "optimize-fixed", "optimize-lut"):
        p = sub.add_parser(command)
        p.add_argument("--manifest", type=Path, required=True)
        p.add_argument("--output", type=Path, required=True)
        p.add_argument("--domain", type=int, default=173)
        if command in ("evaluate", "collect"):
            p.add_argument("--gains", type=float, nargs=3)
            p.add_argument("--fixed-result", type=Path)
            p.add_argument("--policy", choices=("fixed", "lut"), default="fixed")
            p.add_argument("--lut", type=Path)
            p.add_argument("--role", choices=("tuning", "validation", "safety"), default="tuning")
        if command == "optimize-lut":
            p.add_argument("--initial-lut", type=Path, required=True)
            p.add_argument("--samples", type=Path, required=True)
    p = sub.add_parser("operating-report")
    p.add_argument("--input", type=Path, nargs="+", required=True)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--quantiles", type=float, nargs="+", required=True)
    p.add_argument("--bins", type=int, required=True)
    p = sub.add_parser("grid")
    p.add_argument("--statistics", type=Path, required=True)
    p.add_argument("--quantiles", type=float, nargs="+", required=True)
    p.add_argument("--output", type=Path, required=True)
    p = sub.add_parser("init-lut")
    p.add_argument("--axes", type=Path, required=True)
    p.add_argument("--fixed-result", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    p = sub.add_parser("compare")
    p.add_argument("--fixed", type=Path, required=True)
    p.add_argument("--lut", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    p = sub.add_parser("launch-args")
    p.add_argument("--fixed-result", type=Path)
    p.add_argument("--lut", type=Path)
    args = parser.parse_args(argv)
    if args.command in ("evaluate", "collect", "optimize-fixed", "optimize-lut"):
        config = load_manifest(args.manifest)
        if not 0 <= args.domain <= 232:
            raise ValueError("ROS domain must be in [0,232]")
        if args.command.startswith("optimize"):
            optimize(config, args.output, args.domain, getattr(args, "initial_lut", None), getattr(args, "samples", None))
        else:
            gains = args.gains or (read_json_if_exists(args.fixed_result)["gains"] if args.fixed_result else None)
            if gains is None:
                raise ValueError("provide --gains or --fixed-result")
            if (args.policy == "lut") != bool(args.lut):
                raise ValueError("LUT evaluations require --policy lut --lut FILE; fixed cannot load LUT")
            result = evaluate(config, validate_gains(gains), args.output, args.policy, args.domain, args.role, args.lut)
            if args.command == "collect":
                paths = [Path(r["tracking_csv"]) for r in result["results"] if r["success"]]
                operating_report(paths, args.output / "operating", config["recorder"]["quantiles"], config["recorder"]["histogram_bins"])
            if not result["success"]:
                return 1
    elif args.command == "operating-report":
        operating_report(args.input, args.output, args.quantiles, args.bins)
    elif args.command == "grid":
        report = read_json_if_exists(args.statistics)
        axes = {name: sorted({0.0, *(report["axes"][name]["quantiles"][str(q)] for q in args.quantiles)}) for name in AXIS_NAMES}
        write_json(args.output, {"sensor_delay_sec": 0.0, "axes": validate_axes(axes), "source": str(args.statistics)})
    elif args.command == "init-lut":
        axes = validate_axes(read_json_if_exists(args.axes)["axes"])
        result = read_json_if_exists(args.fixed_result)
        if result["policy"] != "fixed" or not result["success"] or result["sensor_delay_sec"] != 0:
            raise ValueError("a successful fixed optimization result is required")
        gains = validate_gains(result["gains"])
        count = math.prod(len(axes[name]) for name in AXIS_NAMES)
        save_lut(args.output, axes, [gains[:] for _ in range(count)])
        write_json(args.output.with_suffix(".json"), {"axes": axes, "fixed_result": str(args.fixed_result),
                                                     "initial_gains": gains, "sensor_delay_sec": 0.0})
    elif args.command == "compare":
        compare(args.fixed, args.lut, args.output)
    elif args.command == "launch-args":
        if bool(args.fixed_result) == bool(args.lut):
            raise ValueError("choose --fixed-result or --lut")
        if args.lut:
            load_lut(args.lut)
            print(f"gain_lut_file:={args.lut.resolve()} sensor_delay_sec:=0.0")
        else:
            result = read_json_if_exists(args.fixed_result)
            if result["policy"] != "fixed" or result["sensor_delay_sec"] != 0:
                raise ValueError("invalid fixed result")
            print(" ".join(f"{n}:={v}" for n, v in zip(GAIN_NAMES, validate_gains(result["gains"]))) + " sensor_delay_sec:=0.0")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
