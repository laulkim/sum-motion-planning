"""Shared, ROS-independent evaluation of the controller's recorded snapshots.

Errors are controller body-frame errors at the selected reference timestamp;
positions are predicted odometry, not a new ground-truth error definition.
"""
import csv
import json
import math
import statistics
from pathlib import Path

ROLES = {"tuning", "validation", "safety"}
GAIN_NAMES = ("kx", "ky", "ktheta")


def write_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")
    temporary.replace(path)


def positive(value, name, zero=False):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f"{name} must be finite numeric")
    if value < 0 or (not zero and value == 0):
        raise ValueError(f"{name} must be {'nonnegative' if zero else 'positive'}")
    return value


def validate_gains(gains):
    if len(gains) != 3:
        raise ValueError("expected kx, ky, ktheta")
    return [positive(v, name) for name, v in zip(GAIN_NAMES, gains)]


def load_manifest(path):
    config = json.loads(Path(path).read_text())
    settings = config["settings"]
    # Reject rather than silently normalize a conflicting experimental protocol.
    if settings["sensor_delay_sec"] != 0.0:
        raise ValueError("gain experiments require sensor_delay_sec=0.0")
    if settings.get("sensor_noise_enabled") is not True or settings.get("sensor_noise_scale") != 1.0:
        raise ValueError("identified AR(2) measurement noise must remain enabled at scale 1.0")
    allowed = {"sensor_delay_sec", "sensor_noise_enabled", "sensor_noise_scale",
               "costmap_resolution", "command_frequency_hz", "mode_transition_duration_sec"}
    if set(settings) - allowed:
        raise ValueError(f"unsupported launch settings: {set(settings) - allowed}")
    for key in set(settings) - {"sensor_delay_sec", "sensor_noise_enabled"}:
        positive(settings[key], key)
    objective = config["objective"]
    for key in ("ex_m", "ey_m", "etheta_rad", "delta_vx_mps", "delta_vy_mps", "delta_yaw_rate_radps"):
        positive(objective["scales"][key], key)
    for key in ("position", "yaw", "control"):
        positive(objective["weights"][key], key, zero=True)
    if not sum(objective["weights"].values()):
        raise ValueError("at least one objective weight is required")
    positive(objective["failure_penalty"], "failure_penalty")
    if not 0 < objective["minimum_valid_fraction"] <= 1:
        raise ValueError("minimum_valid_fraction must be in (0,1]")
    if objective["minimum_samples"] < 2:
        raise ValueError("minimum_samples must be >=2")
    ids = set()
    for case in config["cases"]:
        if not case["id"] or case["id"] in ids or any(c not in "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-" for c in case["id"]):
            raise ValueError("case id must be unique and filesystem-safe")
        ids.add(case["id"])
        if not case["scenario"] or case["role"] not in ROLES:
            raise ValueError("scenario and tuning/validation/safety role required")
        positive(case["target_speed"], "target_speed")
        positive(case["weight"], "weight")
        positive(case["timeout_sec"], "timeout_sec")
        if not isinstance(case["sensor_noise_seed"], int) or isinstance(case["sensor_noise_seed"], bool):
            raise ValueError("noise seed must be an integer")
        if case["end_condition"] not in {"complete", "duration", "safe_stop"}:
            raise ValueError("invalid end_condition")
        if case["end_condition"] == "duration":
            positive(case["duration_sec"], "duration_sec")
            if case["duration_sec"] >= case["timeout_sec"]:
                raise ValueError("duration must be below timeout")
        if case["end_condition"] == "safe_stop":
            if case["role"] != "safety":
                raise ValueError("safe_stop is a safety regression condition, not a tuning objective")
            for key in ("stop_hold_sec", "stop_speed_mps", "maximum_progress_m"):
                positive(case[key], key)
        for value in case.get("metric_limits", {}).values():
            positive(value, "metric limit", zero=True)
    if not ids:
        raise ValueError("empty experiment manifest")
    for name in GAIN_NAMES:
        lower, upper = config["optimizer"]["bounds"][name]
        positive(lower, name)
        positive(upper, name)
        if lower > upper:
            raise ValueError("gain bounds must be ordered")
    optimizer = config["optimizer"]
    initial = validate_gains(optimizer["initial_gains"])
    for name, value in zip(GAIN_NAMES, initial):
        if not optimizer["bounds"][name][0] <= value <= optimizer["bounds"][name][1]:
            raise ValueError("initial gains must be inside bounds")
    for options in (optimizer, optimizer["block_de"]):
        for key, minimum in (("maxiter", 0), ("popsize", 1)):
            if not isinstance(options[key], int) or options[key] < minimum:
                raise ValueError(f"invalid optimizer {key}")
        positive(options["tol"], "tol", zero=True)
    if not isinstance(optimizer["random_seed"], int) or not isinstance(optimizer["max_sweeps"], int) or optimizer["max_sweeps"] < 1:
        raise ValueError("integer optimizer seed and positive max_sweeps required")
    positive(optimizer["smoothness_lambda"], "smoothness_lambda", zero=True)
    positive(optimizer["sweep_tolerance"], "sweep_tolerance", zero=True)
    return config


def read_tracking(path):
    with Path(path).open() as file:
        return [{k: float(v) for k, v in row.items()} for row in csv.DictReader(file)]


def rms(values):
    return math.sqrt(statistics.fmean(v * v for v in values))


def metrics(rows):
    valid_indices = [i for i, r in enumerate(rows) if r["valid"] == 1]
    if len(valid_indices) < 2:
        raise ValueError("at least two valid control snapshots required")
    valid = [rows[i] for i in valid_indices]
    keys = ("ex", "ey", "etheta", "vx_cmd", "vy_cmd", "omega_cmd", "vx_ref", "vy_ref", "omega_ref")
    if any(not math.isfinite(r[key]) for r in valid for key in keys):
        raise ValueError("nonfinite recorded control snapshot")
    out = {"valid_samples": len(valid),
           "valid_fraction": len(valid) / (valid_indices[-1] - valid_indices[0] + 1)}
    for key in ("ex", "ey", "etheta"):
        out[f"rmse_{key}"] = rms([r[key] for r in valid])
        out[f"max_abs_{key}"] = max(abs(r[key]) for r in valid)
    out["position_rmse"] = math.hypot(out["rmse_ex"], out["rmse_ey"])
    # Per-sample increments, NOT a derivative. Include internal invalid/zero
    # commands; exclude startup/after-end invalid output from the scored window.
    window = rows[valid_indices[0]:valid_indices[-1] + 1]
    for source, name in (("vx_cmd", "vx"), ("vy_cmd", "vy"), ("omega_cmd", "yaw_rate")):
        if any(not math.isfinite(r[source]) for r in window):
            raise ValueError("nonfinite command")
        out[f"rms_delta_{name}_cmd"] = rms([b[source] - a[source] for a, b in zip(window, window[1:])])
    turns = [r for r in valid if r["vx_ref"] == 0 and r["vy_ref"] == 0 and r["omega_ref"] != 0]
    out["spot_samples"] = len(turns)
    out["spot_max_abs_vx_cmd"] = max((abs(r["vx_cmd"]) for r in turns), default=0)
    out["spot_max_abs_vy_cmd"] = max((abs(r["vy_cmd"]) for r in turns), default=0)
    out["spot_rmse_etheta"] = rms([r["etheta"] for r in turns]) if turns else None
    return out


def case_objective(metric, config):
    scales, weights = config["scales"], config["weights"]
    parts = {
        "position": 0.5 * ((metric["rmse_ex"] / scales["ex_m"]) ** 2 +
                           (metric["rmse_ey"] / scales["ey_m"]) ** 2),
        "yaw": (metric["rmse_etheta"] / scales["etheta_rad"]) ** 2,
        "control": sum((metric[f"rms_delta_{name}_cmd"] / scales[scale]) ** 2 for name, scale in
                       (("vx", "delta_vx_mps"), ("vy", "delta_vy_mps"), ("yaw_rate", "delta_yaw_rate_radps"))) / 3,
    }
    return sum(weights[key] * value for key, value in parts.items()), parts


def aggregate(results):
    if not results:
        raise ValueError("no cases selected")
    return sum(r["case"]["weight"] * r["objective"] for r in results) / sum(r["case"]["weight"] for r in results)


def quantile(values, q):
    values = sorted(values)
    index = (len(values) - 1) * q
    lo = int(index)
    hi = min(lo + 1, len(values) - 1)
    return values[lo] + (values[hi] - values[lo]) * (index - lo)


def operating_report(paths, output, quantiles, bins):
    if bins < 1 or not quantiles or any(not 0 <= q <= 1 for q in quantiles):
        raise ValueError("bins >=1 and quantiles in [0,1] required")
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    samples = []
    for path in paths:
        metadata = next((parent / "command.json" for parent in Path(path).resolve().parents
                         if (parent / "command.json").exists()), None)
        if metadata is None or json.loads(metadata.read_text())["sensor_delay_sec"] != 0:
            raise ValueError("operating samples require recorded sensor_delay_sec=0.0 provenance")
        samples.extend((str(path), r["time_s"], r["vx_ref"], r["vy_ref"], r["omega_ref"])
                       for r in read_tracking(path) if r["valid"] == 1)
    if not samples or any(not math.isfinite(v) for row in samples for v in row[1:]):
        raise ValueError("no finite operating samples")
    with (output / "operating_points.csv").open("w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(("source", "time_s", "vx", "vy", "yaw_rate"))
        writer.writerows(samples)
    report = {"sensor_delay_sec": 0.0, "samples": len(samples), "axes": {}}
    for column, name in enumerate(("vx", "vy", "yaw_rate"), 2):
        values = [row[column] for row in samples]
        lo, hi = min(values), max(values)
        width = (hi - lo) / bins if hi != lo else 1.0
        counts = [0] * bins
        for value in values:
            counts[min(int((value - lo) / width), bins - 1)] += 1
        report["axes"][name] = {"min": lo, "max": hi, "mean": statistics.fmean(values),
                                  "std": statistics.pstdev(values), "zero_count": values.count(0),
                                  "quantiles": {str(q): quantile(values, q) for q in quantiles},
                                  "histogram_edges": [lo + i * width for i in range(bins + 1)],
                                  "histogram_counts": counts}
    write_json(output / "operating_statistics.json", report)
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    figure, axes = plt.subplots(1, 3, figsize=(12, 3), layout="constrained")
    for ax, name in zip(axes, ("vx", "vy", "yaw_rate")):
        item = report["axes"][name]
        edges = item["histogram_edges"]
        ax.bar(edges[:-1], item["histogram_counts"], width=edges[1] - edges[0], align="edge")
        ax.set_xlabel(name + (" [rad/s]" if name == "yaw_rate" else " [m/s]"))
    figure.savefig(output / "operating_histograms.png", dpi=160)
    plt.close(figure)
    return report
