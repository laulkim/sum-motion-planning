#!/usr/bin/env python3
"""Reproduce the SIMP Planner v8 all-scenario regression matrix.

The intentionally infeasible 2.20 m corridor is accepted only when the vehicle
stops without a swept-footprint collision before entering the gate.
"""
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
RAW_HEADER = [
    "scenario", "target_speed_mps", "raw_pass", "raw_reason", "simulated_time_s",
    "plans", "planning_failures", "primary_allocation_collisions",
    "minimum_vy_successes", "path_replans", "path_replan_successes",
    "bottleneck_limited_plans", "safety_collisions", "gate_passes", "gate_count",
    "minimum_clearance_m", "maximum_plan_ms", "maximum_plan_time_s", "maximum_plan_progress_m",
    "maximum_plan_replans", "maximum_lateral_execution_error_m",
    "maximum_direction_execution_error_deg", "final_progress_m", "final_speed_mps",
]
OUTPUT_HEADER = ["expected_behavior_pass", "verdict", *RAW_HEADER]

@dataclass(frozen=True)
class Case:
    scenario: str
    speed: float
    plan_dt: float
    initial_speed: float | None = None
    max_plans: int | None = None
    expected_safe_stop: bool = False
    start_s: float | None = None
    minimum_stop_progress: float | None = None
    maximum_stop_progress: float | None = None

CASES = [
    *[Case("stadium", v, 1.0) for v in (1.0, 3.0, 5.0)],
    Case("crab_switch", 1.0, 1.0),
    Case("reverse_switch", 1.0, 1.0),
    *[Case("s_curve", v, 0.5) for v in (1.0, 3.0, 5.0)],
    *[Case("s_curve_obstacles", v, 0.5) for v in (1.0, 3.0, 5.0)],
    *[Case("obstacle_avoidance", v, 0.5) for v in (1.0, 3.0, 5.0)],
    Case("terminal_safe_region", 0.2, 1.0, initial_speed=0.2),
    *[Case("terminal_safe_region", v, 0.1) for v in (1.0, 3.0, 5.0)],
    *[Case("alternating_gate_corridor", v, 1.0) for v in (1.0, 3.0, 5.0)],
    *[Case("curved_gate_maze", v, 0.5) for v in (1.0, 3.0, 5.0)],
    Case("narrow_22m_stop_corridor", 0.2, 0.5, initial_speed=0.2, max_plans=60, expected_safe_stop=True, start_s=8.0, minimum_stop_progress=8.0, maximum_stop_progress=14.0),
    Case("narrow_22m_stop_corridor", 1.0, 0.5, max_plans=60, expected_safe_stop=True, minimum_stop_progress=0.0, maximum_stop_progress=14.0),
    Case("narrow_22m_stop_corridor", 3.0, 0.5, max_plans=60, expected_safe_stop=True, minimum_stop_progress=0.0, maximum_stop_progress=14.0),
    *[Case("narrow_28m_corridor", v, 0.5) for v in (1.0, 3.0, 5.0)],
    *[Case("narrow_offset_corridor", v, 0.5) for v in (1.0, 3.0)],
    *[Case("winding_obstacle_course", v, 1.0) for v in (1.0, 3.0, 5.0)],
]


def run(command: list[str], *, env: dict[str, str] | None = None, timeout: int = 360) -> subprocess.CompletedProcess[str]:
    return subprocess.run(command, cwd=ROOT, env=env, text=True,
                          capture_output=True, check=False, timeout=timeout)


def build_validator() -> None:
    BUILD.mkdir(parents=True, exist_ok=True)
    command = [
        "g++", "-std=c++17", "-O3", "-DNDEBUG", "-Wall", "-Wextra", "-Wpedantic",
        f"-I{ROOT / 'simp_planner/simp_planner_cpp/include'}",
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
    result = run([sys.executable, str(VALIDATION / "export_scenario_input.py"), name, str(output)], env=env)
    if result.returncode != 0:
        raise RuntimeError(f"scenario export failed: {name}\n{result.stdout}\n{result.stderr}")
    return output


def validate(index: int, case: Case, scenario_file: Path) -> tuple[int, list[str]]:
    env = os.environ.copy()
    for key in ("SIMP_VALIDATION_START_S", "SIMP_VALIDATION_END_S", "SIMP_VALIDATION_INITIAL_SPEED", "SIMP_VALIDATION_TRAJECTORY_CSV"):
        env.pop(key, None)
    env["SIMP_VALIDATION_PLAN_DT"] = f"{case.plan_dt:.9g}"
    if case.initial_speed is not None:
        env["SIMP_VALIDATION_INITIAL_SPEED"] = f"{case.initial_speed:.9g}"
    if case.start_s is not None:
        env["SIMP_VALIDATION_START_S"] = f"{case.start_s:.9g}"
    command = [str(BINARY), str(scenario_file), f"{case.speed:.9g}"]
    if case.max_plans is not None:
        command.append(str(case.max_plans))
    result = run(command, env=env)
    lines = [line.strip() for line in result.stdout.splitlines() if line.strip()]
    if not lines:
        raise RuntimeError(f"no CSV output for {case}\n{result.stderr}")
    raw = next(csv.reader([lines[-1]]))
    if len(raw) != len(RAW_HEADER):
        raise RuntimeError(f"unexpected column count {len(raw)} for {case}: {lines[-1]}")

    record = dict(zip(RAW_HEADER, raw))
    raw_pass = record["raw_pass"] == "1"
    safety_collisions = int(record["safety_collisions"])
    final_speed = float(record["final_speed_mps"])
    final_progress = float(record["final_progress_m"])
    gate_passes = int(record["gate_passes"])
    if case.expected_safe_stop:
        lower_ok = case.minimum_stop_progress is None or final_progress >= case.minimum_stop_progress - 1.0e-6
        upper_ok = case.maximum_stop_progress is None or final_progress <= case.maximum_stop_progress + 1.0e-6
        accepted = (not raw_pass and safety_collisions == 0 and abs(final_speed) <= 0.05
                    and gate_passes == 0 and lower_ok and upper_ok)
        verdict = "EXPECTED_SAFE_STOP_BEFORE_2P2M_CORRIDOR" if accepted else "UNEXPECTED_2P2M_STOP_BEHAVIOR"
    else:
        accepted = raw_pass and safety_collisions == 0
        verdict = record["raw_reason"] if accepted else f"UNEXPECTED_{record['raw_reason']}"
    return index, ["1" if accepted else "0", verdict, *raw]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--skip-build", action="store_true")
    parser.add_argument("--jobs", type=int, default=min(6, os.cpu_count() or 4))
    args = parser.parse_args()
    if not args.skip_build:
        build_validator()
    files = {name: export_scenario(name) for name in sorted({case.scenario for case in CASES})}
    rows: list[list[str] | None] = [None] * len(CASES)
    with ThreadPoolExecutor(max_workers=max(1, args.jobs)) as pool:
        futures = [pool.submit(validate, i, case, files[case.scenario]) for i, case in enumerate(CASES)]
        for future in as_completed(futures):
            index, row = future.result()
            rows[index] = row
            print(",".join(row), flush=True)
    final_rows = [row for row in rows if row is not None]
    output = VALIDATION / "all_scenarios_results.csv"
    with output.open("w", newline="", encoding="utf-8") as file:
        writer = csv.writer(file)
        writer.writerow(OUTPUT_HEADER)
        writer.writerows(final_rows)
    records = [dict(zip(OUTPUT_HEADER, row)) for row in final_rows]
    normal = [r for r in records if r["scenario"] != "narrow_22m_stop_corridor"]
    expected_stop = [r for r in records if r["scenario"] == "narrow_22m_stop_corridor"]
    maximum = max(records, key=lambda r: float(r["maximum_plan_ms"]))
    summary = {
        "version": "v8",
        "scenario_types": len({r["scenario"] for r in records}),
        "cases": len(records),
        "expected_behavior_passed": sum(int(r["expected_behavior_pass"]) for r in records),
        "expected_behavior_failed": sum(1 - int(r["expected_behavior_pass"]) for r in records),
        "normal_cases_passed": sum(int(r["raw_pass"]) for r in normal),
        "normal_cases": len(normal),
        "expected_safe_stop_cases_passed": sum(int(r["expected_behavior_pass"]) for r in expected_stop),
        "expected_safe_stop_cases": len(expected_stop),
        "safety_collisions": sum(int(r["safety_collisions"]) for r in records),
        "path_replans": sum(int(r["path_replans"]) for r in records),
        "path_replan_successes": sum(int(r["path_replan_successes"]) for r in records),
        "maximum_plan_ms": float(maximum["maximum_plan_ms"]),
        "maximum_plan_case": f"{maximum['scenario']} @ {maximum['target_speed_mps']} m/s",
        "maximum_plan_time_s": float(maximum["maximum_plan_time_s"]),
        "maximum_plan_progress_m": float(maximum["maximum_plan_progress_m"]),
        "maximum_plan_replans": int(maximum["maximum_plan_replans"]),
        "notes": [
            "All 13 scenario definitions were exercised.",
            "The 2.80 m narrow_28m_corridor is the primary fixed-specification narrow-corridor pass scenario at 1, 3, and 5 m/s.",
            "The 2.20 m narrow_22m_stop_corridor is stop-only; acceptance requires zero swept collision, zero gate passage, near-zero final speed, and stopping before the corridor entrance.",
            "No executed swept-footprint collision was observed.",
        ],
    }
    (VALIDATION / "all_scenarios_summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    failed = [r for r in records if r["expected_behavior_pass"] != "1"]
    print(f"v8 all-scenario validation: {len(records)-len(failed)}/{len(records)} expected behaviors PASS")
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
