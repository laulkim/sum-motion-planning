#!/usr/bin/env python3

from __future__ import annotations

import argparse
from datetime import datetime
import os
from pathlib import Path
import signal
import subprocess

from .compare_simulation_runs import compare_runs


DEFAULT_OUTPUT_ROOT = Path(
    "/home/sum/Desktop/simp_planner/simp_planner_debug"
)
SHUTDOWN_TIMEOUT_SEC = 30.0


def build_launch_command(
    scenario: str,
    target_speed: float,
    model: str,
    save_period: float,
    debug_output_root: Path,
    tracking_enabled: bool = True,
) -> list[str]:
    return [
        "ros2",
        "launch",
        "simp_planner_tools",
        "simulation.launch.py",
        f"scenario:={scenario}",
        f"target_speed:={target_speed}",
        f"kinematics_model:={model}",
        f"save_period:={save_period}",
        f"debug_output_dir:={debug_output_root}",
        f"tracking_enabled:={str(tracking_enabled).lower()}",
    ]


def create_batch_directory(output_root: Path, scenario: str) -> Path:
    parent = output_root / scenario
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    candidate = parent / f"compare_{timestamp}"
    suffix = 1
    while candidate.exists():
        candidate = parent / f"compare_{timestamp}_{suffix:02d}"
        suffix += 1
    candidate.mkdir(parents=True)
    return candidate


def find_run_directory(debug_output_root: Path, scenario: str) -> Path:
    scenario_directory = debug_output_root / scenario
    candidates = sorted(
        path
        for path in scenario_directory.iterdir()
        if path.is_dir()
        and (path / "odom_history.csv").is_file()
        and (path / "command_history.csv").is_file()
    ) if scenario_directory.is_dir() else []
    if len(candidates) != 1:
        raise RuntimeError(
            f"Expected one completed run under {scenario_directory}, "
            f"found {len(candidates)}"
        )
    return candidates[0]


def stop_launch_process(process: subprocess.Popen[bytes]) -> int:
    if process.poll() is None:
        # Signal the launch parent once. It then forwards SIGINT to every ROS
        # child and gives nodes a chance to flush their final debug data.
        process.send_signal(signal.SIGINT)
    try:
        return process.wait(timeout=SHUTDOWN_TIMEOUT_SEC)
    except subprocess.TimeoutExpired:
        # Escalation targets only the isolated subprocess group created below.
        os.killpg(process.pid, signal.SIGTERM)
        try:
            return process.wait(timeout=SHUTDOWN_TIMEOUT_SEC)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            return process.wait()


def run_for_duration(command: list[str], duration_sec: float) -> None:
    process = subprocess.Popen(command, start_new_session=True)
    try:
        return_code = process.wait(timeout=duration_sec)
    except subprocess.TimeoutExpired:
        stop_launch_process(process)
        return
    except KeyboardInterrupt:
        stop_launch_process(process)
        raise
    raise RuntimeError(
        f"Simulation ended before {duration_sec:.1f} s "
        f"(exit code {return_code})"
    )


def run_comparison(
    scenario: str,
    target_speed: float,
    duration_sec: float,
    output_root: Path,
    save_period: float,
    sample_period: float,
    show: bool,
) -> tuple[Path, Path, Path]:
    if duration_sec <= 0.0:
        raise ValueError("duration must be positive")
    if save_period <= 0.0:
        raise ValueError("save period must be positive")
    if sample_period <= 0.0:
        raise ValueError("sample period must be positive")

    batch_directory = create_batch_directory(output_root, scenario)
    runs: dict[str, Path] = {}
    for label, model, tracking_enabled in (
        ("ideal", "ideal", True),
        ("noisy", "noisy", True),
        ("noisy_off", "noisy", False),
    ):
        model_output_root = batch_directory / "raw" / label
        command = build_launch_command(
            scenario,
            target_speed,
            model,
            save_period,
            model_output_root,
            tracking_enabled=tracking_enabled,
        )
        print(f"\n[{label}, P {'ON' if tracking_enabled else 'OFF'}] running for {duration_sec:.1f} s", flush=True)
        run_for_duration(command, duration_sec)
        runs[label] = find_run_directory(model_output_root, scenario)
        print(f"[{label}] data: {runs[label]}", flush=True)

    comparison_directory = batch_directory / "comparison"
    compare_runs(
        runs["ideal"],
        runs["noisy"],
        comparison_directory,
        sample_period=sample_period,
        show=show,
        noisy_off_directory=runs["noisy_off"],
    )
    return runs["ideal"], runs["noisy"], comparison_directory


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "Run ideal (P ON), noisy P ON and noisy P OFF sequentially, "
            "then compare their saved data."
        )
    )
    parser.add_argument("--scenario", default="stadium")
    parser.add_argument("--target-speed", type=float, default=4.0)
    parser.add_argument(
        "--duration",
        type=float,
        default=120.0,
        help="Wall-clock seconds for EACH of the three runs (plus startup/shutdown time).",
    )
    parser.add_argument(
        "--output-root",
        type=Path,
        default=DEFAULT_OUTPUT_ROOT,
    )
    parser.add_argument("--save-period", type=float, default=10.0)
    parser.add_argument("--sample-period", type=float, default=0.01)
    parser.add_argument(
        "--show",
        action="store_true",
        help="Open comparison, tracking-error and per-run target/feedback plots after all three runs finish.",
    )
    args = parser.parse_args()

    try:
        ideal, noisy, comparison = run_comparison(
            scenario=args.scenario,
            target_speed=args.target_speed,
            duration_sec=args.duration,
            output_root=args.output_root.expanduser().resolve(),
            save_period=args.save_period,
            sample_period=args.sample_period,
            show=args.show,
        )
    except KeyboardInterrupt:
        print("\nComparison run interrupted; the active simulation was stopped.")
        return

    print("\nCompleted ideal/noisy comparison and P OFF/ON tracking evaluation")
    print(f"Ideal data: {ideal}")
    print(f"Noisy data: {noisy}")
    print(f"Comparison: {comparison}")


if __name__ == "__main__":
    main()
