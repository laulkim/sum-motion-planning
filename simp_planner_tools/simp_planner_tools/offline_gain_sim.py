"""Replay selected ROS references through the existing plant, without ROS or waits.

The reference schedule is frozen. Search scores are approximations; the final
candidate must be validated with the live planner in full ROS closed loop.
"""
import csv
import hashlib
import json
import math
from pathlib import Path
import time

from planar_velocity_sim.kinematics import integrate_body_velocity, wrap_angle
from planar_velocity_sim.mode_transition import DriveModeTransitionModel
from planar_velocity_sim.vehicle_response import (
    AR2Noise, AR2_NOISE_DEFAULTS, FirstOrderResponse, RESPONSE_PARAMETER_DEFAULTS,
)
from .gain_evaluation import aggregate, case_objective, metrics, read_tracking, validate_gains, write_json
from .gain_lut import lookup

CHANNELS = ("vx", "vy", "yaw_rate")
REFERENCE_FIELDS = ("xr", "yr", "yaw_r", "vx_ref", "vy_ref", "omega_ref", "requested_mode",
                    "control_stamp_ns", "reference_stamp_ns")


def response_parameters():
    return {**RESPONSE_PARAMETER_DEFAULTS, "sensor_delay_sec": 0.0}


def load_references(path, config):
    """Read each tuning CSV once, and reject old/incomplete collection outputs."""
    path = Path(path)
    summary = json.loads(path.read_text())
    recorded = summary["configuration"]
    cases = [case for case in config["cases"] if case["role"] == "tuning"]
    if (summary["role"] != "tuning" or not summary["success"] or summary["sensor_delay_sec"] != 0.0
            or recorded["config"]["settings"] != config["settings"]
            or recorded.get("response_parameters") != response_parameters()
            or [r["case"] for r in summary["results"]] != cases):
        raise ValueError("collect complete matching tuning references with sensor_delay_sec=0.0 first")
    datasets, hashes = [], {}
    for result in summary["results"]:
        source = Path(result["tracking_csv"])
        rows = read_tracking(source)
        valid = [i for i, row in enumerate(rows) if row["valid"] == 1]
        if len(valid) < 2:
            raise ValueError(f"{source}: insufficient selected references")
        rows = rows[valid[0]:valid[-1] + 1]  # retain internal invalid zero-command periods
        for previous, row in zip(rows, rows[1:]):
            if row["control_stamp_ns"] <= previous["control_stamp_ns"]:
                raise ValueError(f"{source}: nonmonotonic control timestamps")
        for row in rows:
            if row["valid"] not in (0, 1) or not math.isfinite(row["control_stamp_ns"]):
                raise ValueError(f"{source}: invalid control snapshot")
            if row["valid"] == 1:
                if any(key not in row or not math.isfinite(row[key]) for key in REFERENCE_FIELDS):
                    raise ValueError(f"{source}: recollect CSV with requested_mode and selected-reference timing")
                if row["requested_mode"] not in (0, 1, 2, 3, 4) or row["reference_stamp_ns"] < row["control_stamp_ns"]:
                    raise ValueError(f"{source}: invalid reference mode/timing")
        datasets.append((result["case"], rows))
        hashes[result["case"]["id"]] = hashlib.sha256(source.read_bytes()).hexdigest()
    if not datasets:
        raise ValueError("no tuning reference datasets")
    return datasets, {"summary_sha256": hashlib.sha256(path.read_bytes()).hexdigest(), "csv_sha256": hashes}


def replay(rows, gains, seed, settings, lut=None):
    """100 Hz plant ticks + recorded controller events; sensor observation delay=0.

    Command and mode gates precede identified channel dead times. Measurement
    noise advances only on plant/odometry ticks, never on actual pose. The
    logged selected-reference timestamp also preserves C++ pose prediction.
    """
    validate_gains(gains)
    if settings["sensor_delay_sec"] != 0.0:
        raise ValueError("offline replay requires sensor_delay_sec=0.0")
    first = rows[0]
    pose = (first["xr"], first["yr"], first["yaw_r"])
    origin = first["control_stamp_ns"]
    parameters = response_parameters()
    responses = [FirstOrderResponse(parameters[f"{channel}_gain"], parameters[f"{channel}_tau_sec"],
                                    parameters[f"{channel}_delay_sec"], 0.0) for channel in CHANNELS]
    noise = [AR2Noise(a1, a2, sigma * settings.get("sensor_noise_scale", 1.0), seed + index)
             for index, (a1, a2, sigma) in enumerate(AR2_NOISE_DEFAULTS.values())]
    mode = DriveModeTransitionModel(initial_mode=int(first["requested_mode"]),
                                   transition_duration_sec=settings["mode_transition_duration_sec"])
    actual = measured = command = (0.0, 0.0, 0.0)
    tick, plant_time = 1, 0.0
    history = []
    for reference in rows:
        now = (reference["control_stamp_ns"] - origin) * 1e-9
        # Same update rate and ordering as PlanarVelocitySimNode.update().
        while tick * 0.01 <= now + 1e-12:
            next_time = tick * 0.01
            if mode.update(next_time):
                for response, value in zip(responses, mode.applied_velocity(*command)):
                    response.command(value, next_time)
            actual = tuple(response.update(next_time) for response in responses)
            pose = integrate_body_velocity(*pose, *actual, next_time - plant_time)
            measured = tuple(value + (process.sample() if settings.get("sensor_noise_enabled", True) else 0.0)
                             for value, process in zip(actual, noise))
            plant_time = next_time
            tick += 1
        row = dict(reference)
        row.update(time_s=now, x=pose[0], y=pose[1], yaw=pose[2],
                   vx_actual=actual[0], vy_actual=actual[1], omega_actual=actual[2],
                   vx=measured[0], vy=measured[1], omega=measured[2])
        command = (0.0, 0.0, 0.0)
        if reference["valid"] == 1:
            prediction_time = (reference["reference_stamp_ns"] - origin) * 1e-9 - plant_time
            predicted = integrate_body_velocity(*pose, *measured, prediction_time)
            dx, dy = reference["xr"] - predicted[0], reference["yr"] - predicted[1]
            c, s = math.cos(predicted[2]), math.sin(predicted[2])
            ex, ey = c * dx + s * dy, -s * dx + c * dy
            etheta = wrap_angle(reference["yaw_r"] - predicted[2])
            vx, vy, omega = reference["vx_ref"], reference["vy_ref"], reference["omega_ref"]
            kx, ky, ktheta = lookup(*lut, (vx, vy, omega)) if lut else gains
            if lut and vx == 0.0 and vy == 0.0 and omega != 0.0:
                kx = ky = 0.0  # structural constraint AFTER interpolation, not at full stop
            c, s = math.cos(etheta), math.sin(etheta)
            command = (vx * c - vy * s + kx * ex, vy * c + vx * s + ky * ey, omega + ktheta * etheta)
            row.update(predicted_x=predicted[0], predicted_y=predicted[1], predicted_yaw=predicted[2],
                       ex=ex, ey=ey, etheta=etheta)
            before = mode.feedback()
            mode.command(int(reference["requested_mode"]), math.hypot(actual[0], actual[1]), now)
            if mode.feedback() != before and mode.transition_in_progress:
                for response in responses:
                    response.command(0.0, now)
        # During ALIGNING retain the latest controller command for completion,
        # but do not queue it (matching cmd_vel_callback).
        if not mode.transition_in_progress:
            for response, value in zip(responses, command):
                response.command(value, now)
        if not all(math.isfinite(value) for value in (*pose, *actual, *measured, *command)):
            raise ValueError("nonfinite offline evolution")
        row.update(vx_cmd=command[0], vy_cmd=command[1], omega_cmd=command[2])
        history.append(row)
    return history


def evaluate_offline(config, datasets, gains, output, lut=None, save_history=False):
    started = time.perf_counter()
    output = Path(output)
    results = []
    for case, references in datasets:
        result = {"case": case, "success": False, "metrics": {},
                  "objective": config["objective"]["failure_penalty"], "reason": "offline_failure"}
        try:
            rows = replay(references, gains, case["sensor_noise_seed"], config["settings"], lut)
            metric = metrics(rows)
            objective, parts = case_objective(metric, config["objective"])
            valid = (metric["valid_samples"] >= config["objective"]["minimum_samples"]
                     and metric["valid_fraction"] >= config["objective"]["minimum_valid_fraction"]
                     and all(metric.get(key, math.inf) <= limit for key, limit in case.get("metric_limits", {}).items()))
            if lut and (metric["spot_max_abs_vx_cmd"] != 0 or metric["spot_max_abs_vy_cmd"] != 0):
                valid = False
            result.update(metrics=metric, objective_parts=parts, success=valid,
                          objective=objective if valid else result["objective"], reason="offline_replay" if valid else "metric_limits")
            if save_history:
                path = output / case["id"] / "tracking.csv"
                path.parent.mkdir(parents=True, exist_ok=True)
                columns = list(dict.fromkeys(key for row in rows for key in row))
                with path.open("w", newline="") as file:
                    writer = csv.DictWriter(file, fieldnames=columns)
                    writer.writeheader()
                    writer.writerows(rows)
                result["tracking_csv"] = str(path.resolve())
        except (ValueError, OverflowError) as error:
            result["metric_error"] = str(error)
        results.append(result)
    summary = {"backend": "offline_replay", "policy": "lut" if lut else "fixed", "role": "tuning",
               "sensor_delay_sec": 0.0, "gains": list(gains), "objective": aggregate(results),
               "success": all(result["success"] for result in results), "results": results,
               "elapsed_wall_sec": time.perf_counter() - started,
               "simulated_duration_sec": sum((rows[-1]["control_stamp_ns"] - rows[0]["control_stamp_ns"]) * 1e-9 for _, rows in datasets)}
    write_json(output / "summary.json", summary)
    print(f"offline candidate: J={summary['objective']:.6g} wall={summary['elapsed_wall_sec']:.6f}s "
          f"reference_duration={summary['simulated_duration_sec']:.3f}s success={summary['success']}", flush=True)
    return summary
