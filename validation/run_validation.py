#!/usr/bin/env python3
"""Build and reproduce the selected SIMP Planner v8 regression matrix."""
from __future__ import annotations

import argparse
import csv
import json
import os
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor, as_completed
from dataclasses import dataclass
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
VALIDATION = ROOT / "validation"
BUILD = VALIDATION / "build"
BINARY = BUILD / "closed_loop_validator"
HEADER = [
    "scenario", "target_speed_mps", "pass", "reason", "simulated_time_s",
    "plans", "planning_failures", "primary_allocation_collisions",
    "minimum_vy_successes", "path_replans",
    "path_replan_successes", "bottleneck_limited_plans",
    "safety_collisions", "gate_passes", "gate_count", "minimum_clearance_m",
    "maximum_plan_ms", "maximum_plan_time_s", "maximum_plan_progress_m",
    "maximum_plan_replans", "maximum_lateral_execution_error_m",
    "maximum_direction_execution_error_deg", "final_progress_m",
    "final_speed_mps",
]

@dataclass(frozen=True)
class ValidationCase:
    scenario: str
    speed: float
    plan_dt: float
    initial_speed: float | None = None

CASES = [
    ValidationCase("terminal_safe_region", 0.2, 1.0, 0.2),
    ValidationCase("terminal_safe_region", 1.0, 0.1),
    ValidationCase("terminal_safe_region", 3.0, 0.1),
    ValidationCase("terminal_safe_region", 5.0, 0.1),
    *[ValidationCase("obstacle_avoidance", v, 0.5) for v in (1.0, 3.0, 5.0)],
    *[ValidationCase("s_curve_obstacles", v, 0.5) for v in (1.0, 3.0, 5.0)],
    *[ValidationCase("curved_gate_maze", v, 0.5) for v in (1.0, 3.0, 5.0)],
    *[ValidationCase("winding_obstacle_course", v, 1.0) for v in (1.0, 3.0, 5.0)],
    *[ValidationCase("alternating_gate_corridor", v, 1.0) for v in (1.0, 3.0, 5.0)],
    *[ValidationCase("narrow_28m_corridor", v, 0.5) for v in (1.0, 3.0, 5.0)],
    *[ValidationCase("narrow_offset_corridor", v, 0.5) for v in (1.0, 3.0)],
    ValidationCase("crab_switch", 1.0, 1.0),
    ValidationCase("reverse_switch", 1.0, 1.0),
]

def run(command: list[str], *, env: dict[str, str] | None = None) -> subprocess.CompletedProcess[str]:
    return subprocess.run(command, cwd=ROOT, env=env, text=True,
                          capture_output=True, check=False)

def build_validator() -> None:
    BUILD.mkdir(parents=True, exist_ok=True)
    command = [
        "g++", "-std=c++17", "-O3", "-DNDEBUG", "-Wall", "-Wextra",
        "-Wpedantic", f"-I{ROOT / 'simp_planner/simp_planner_cpp/include'}",
        str(ROOT / "simp_planner/simp_planner_cpp/src/core.cpp"),
        str(ROOT / "simp_planner/simp_planner_cpp/src/runtime.cpp"),
        str(VALIDATION / "src/closed_loop_validator.cpp"),
        "-o", str(BINARY),
    ]
    result = run(command)
    if result.returncode != 0:
        raise RuntimeError(f"validator build failed\n{result.stdout}\n{result.stderr}")

def export_scenario(name: str) -> Path:
    output = VALIDATION / "data" / f"{name}.svd"
    env = os.environ.copy()
    env["PYTHONPATH"] = str(ROOT / "simp_planner_tools")
    result = run([sys.executable, str(VALIDATION / "export_scenario_input.py"),
                  name, str(output)], env=env)
    if result.returncode != 0:
        raise RuntimeError(f"scenario export failed: {name}\n{result.stdout}\n{result.stderr}")
    return output

def validate_case(index: int, case: ValidationCase, scenario_file: Path) -> tuple[int, list[str]]:
    env = os.environ.copy()
    for name in ("SIMP_VALIDATION_START_S", "SIMP_VALIDATION_END_S",
                 "SIMP_VALIDATION_INITIAL_SPEED", "SIMP_VALIDATION_TRAJECTORY_CSV"):
        env.pop(name, None)
    env["SIMP_VALIDATION_PLAN_DT"] = f"{case.plan_dt:.9g}"
    if case.initial_speed is not None:
        env["SIMP_VALIDATION_INITIAL_SPEED"] = f"{case.initial_speed:.9g}"
    result = run([str(BINARY), str(scenario_file), f"{case.speed:.9g}"], env=env)
    lines = [line.strip() for line in result.stdout.splitlines() if line.strip()]
    if not lines:
        raise RuntimeError(f"no CSV output: {case}\n{result.stderr}")
    row = next(csv.reader([lines[-1]]))
    if len(row) != len(HEADER):
        raise RuntimeError(f"unexpected columns {len(row)}: {lines[-1]}")
    return index, row

def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--skip-build", action="store_true")
    parser.add_argument("--jobs", type=int, default=min(8, os.cpu_count() or 4))
    args = parser.parse_args()
    if not args.skip_build:
        build_validator()
    scenario_files = {name: export_scenario(name) for name in sorted({c.scenario for c in CASES})}
    rows: list[list[str] | None] = [None] * len(CASES)
    with ThreadPoolExecutor(max_workers=max(1, args.jobs)) as pool:
        futures = [pool.submit(validate_case, i, c, scenario_files[c.scenario])
                   for i, c in enumerate(CASES)]
        for future in as_completed(futures):
            index, row = future.result()
            rows[index] = row
            print(",".join(row), flush=True)
    final_rows = [row for row in rows if row is not None]
    output = VALIDATION / "selected_regression_results.csv"
    with output.open("w", newline="", encoding="utf-8") as file:
        writer = csv.writer(file); writer.writerow(HEADER); writer.writerows(final_rows)
    records = [dict(zip(HEADER, row)) for row in final_rows]
    maximum = max(records, key=lambda item: float(item["maximum_plan_ms"]))
    summary = {
        "version": "v8", "cases": len(records),
        "passed": sum(int(item["pass"]) for item in records),
        "failed": sum(1-int(item["pass"]) for item in records),
        "safety_collisions": sum(int(item["safety_collisions"]) for item in records),
        "minimum_clearance_m": min(float(item["minimum_clearance_m"]) for item in records),
        "maximum_plan_ms": float(maximum["maximum_plan_ms"]),
        "maximum_plan_case": f"{maximum['scenario']} @ {maximum['target_speed_mps']} m/s",
        "maximum_plan_time_s": float(maximum["maximum_plan_time_s"]),
        "maximum_plan_progress_m": float(maximum["maximum_plan_progress_m"]),
        "maximum_plan_replans": int(maximum["maximum_plan_replans"]),
        "path_replans": sum(int(item["path_replans"]) for item in records),
        "path_replan_successes": sum(int(item["path_replan_successes"]) for item in records),
        "notes": [
            "Static obstacle collision is screened in the spatial path stage and in the final allocated swept footprint stage.",
            "The temporal stage checks dynamic and terminal feasibility without repeating static collision rejection.",
            "0.2 m/s terminal validation uses a 1.0 s plan-hold interval; other intervals are listed in this script.",
        ],
    }
    (VALIDATION / "selected_regression_summary.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=False)+"\n", encoding="utf-8")
    failed = [row for row in final_rows if row[2] != "1"]
    print(f"v8 validation: {len(final_rows)-len(failed)}/{len(final_rows)} PASS")
    return 1 if failed else 0

if __name__ == "__main__":
    raise SystemExit(main())
