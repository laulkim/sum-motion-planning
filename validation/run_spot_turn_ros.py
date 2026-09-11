#!/usr/bin/env python3
"""Exercise the installed nodes on an isolated ROS domain, without a GUI.

Run after colcon build and source install/setup.bash. Logs and the summary are
written to --output, never into another running simulation's debug directory.
"""
from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--domain", type=int, default=157)
    parser.add_argument("--speed", type=float, default=4.2)
    parser.add_argument("--timeout", type=float, default=240.0)
    parser.add_argument("--output", type=Path, default=Path("/tmp/simp_spot_turn_ros"))
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    os.environ["ROS_DOMAIN_ID"] = str(args.domain)
    os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
    os.environ["ROS_LOG_DIR"] = str(args.output / "ros_logs")
    os.environ["PYTHONDONTWRITEBYTECODE"] = "1"

    import rclpy
    from ament_index_python.packages import get_package_prefix, get_package_share_directory
    from nav_msgs.msg import Odometry
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from simp_planner_msgs.msg import DriveModeState
    from std_msgs.msg import String
    from simp_planner_tools.scenario_definition import load_scenario_definition

    definition = load_scenario_definition(Path(get_package_share_directory("simp_planner_tools")), "spot_turn_course")
    rclpy.init(args=[])
    node = rclpy.create_node("spot_turn_validation_monitor")
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    summary = {"pass": False, "phase_indices": [], "turns": [], "maximum_speed": 0.0}
    latest = {"scenario": {}, "planner": {}, "odom": None, "mode": 0}

    def scenario_callback(message):
        state = json.loads(message.data)
        latest["scenario"] = state
        phase = state["phase_index"]
        if phase not in summary["phase_indices"]:
            summary["phase_indices"].append(phase)
            print(f"phase {phase}: {state['phase_name']}", flush=True)

    def planner_callback(message):
        latest["planner"] = json.loads(message.data)

    def odom_callback(message):
        latest["odom"] = message
        speed = math.hypot(message.twist.twist.linear.x, message.twist.twist.linear.y)
        summary["maximum_speed"] = max(summary["maximum_speed"], speed)

    def mode_callback(message):
        old_mode = latest["mode"]
        latest["mode"] = message.current_mode
        if old_mode == 4 and message.current_mode != 4 and summary["turns"] and latest["odom"] is not None:
            q = latest["odom"].pose.pose.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            turn = summary["turns"][-1]
            turn["yaw_error"] = abs((yaw - turn["target_body_yaw"] + math.pi) % (2 * math.pi) - math.pi)
        if message.current_mode != 4 or old_mode == 4 or latest["odom"] is None:
            return
        phase = latest["scenario"].get("phase_index", 0)
        path = definition.phases[phase].path
        position = latest["odom"].pose.pose.position
        corners = list(path.skip_continuity_at)
        distance = min((math.hypot(position.x - path.x[i], position.y - path.y[i])
                        for i in corners), default=math.inf)
        target_yaw = float(path.yaw[corners[0] + 1]) if corners else 0.0
        summary["turns"].append({"phase": phase, "x": position.x, "y": position.y,
                                 "corner_distance": distance, "target_body_yaw": target_yaw})
        print(f"turn {len(summary['turns'])}: phase {phase}, corner distance {distance:.3f} m", flush=True)

    node.create_subscription(String, "/scenario/status", scenario_callback, qos)
    node.create_subscription(String, "/planner/status", planner_callback, qos)
    node.create_subscription(Odometry, "/odom", odom_callback, 50)
    node.create_subscription(DriveModeState, "/vehicle/drive_mode_state", mode_callback, qos)

    specs = [
        ("planar_velocity_sim", "planar_velocity_sim_node", {}),
        ("simp_planner_tools", "scenario_manager_node", {
            "scenario": "spot_turn_course", "target_speed": args.speed,
            "costmap_resolution": 0.1, "costmap_size_m": 60.0, "costmap_publish_hz": 5.0}),
        ("simp_planner_cpp", "planner_node_cpp", {}),
    ]
    processes, logs = [], []
    started = time.monotonic()
    stalled_since = None
    next_progress = started + 15.0
    try:
        for package, executable, params in specs:
            binary = Path(get_package_prefix(package)) / "lib" / package / executable
            command = [str(binary), "--ros-args"]
            for key, value in params.items():
                command += ["-p", f"{key}:={value}"]
            log = (args.output / f"{executable}.log").open("w")
            logs.append(log)
            processes.append(subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT))
        while time.monotonic() - started < args.timeout:
            rclpy.spin_once(node, timeout_sec=0.05)
            if any(process.poll() is not None for process in processes):
                raise RuntimeError("a simulation node exited; inspect output logs")
            planner_state = latest["planner"].get("state", "")
            if planner_state == "NO_SAFE_PLAN_SAFETY_STOP":
                stalled_since = stalled_since or time.monotonic()
                if time.monotonic() - stalled_since > 8.0:
                    raise RuntimeError("NO_SAFE_PLAN_SAFETY_STOP persisted for 8 seconds")
            else:
                stalled_since = None
            if any(turn["corner_distance"] > 0.25 for turn in summary["turns"]):
                raise RuntimeError("rotation started away from its corner")
            if any(turn.get("yaw_error", 0.0) > 0.025 for turn in summary["turns"]):
                raise RuntimeError("rotation returned to driving before actual heading settled")
            if latest["scenario"].get("state") == "COMPLETE":
                summary["pass"] = (
                    summary["phase_indices"] == list(range(7)) and
                    len(summary["turns"]) == 3 and summary["maximum_speed"] > 1.0)
                if not summary["pass"]:
                    raise RuntimeError("course ended without expected turns, phases, or acceleration")
                break
            if time.monotonic() >= next_progress:
                status = latest["scenario"]
                print(f"elapsed={time.monotonic() - started:.0f}s phase={status.get('phase_index')} "
                      f"remaining={status.get('remaining_to_terminal')} planner={planner_state}", flush=True)
                next_progress += 15.0
        else:
            raise RuntimeError("course completion timed out")
    except (RuntimeError, KeyboardInterrupt) as error:
        summary["error"] = str(error)
    finally:
        summary["elapsed_seconds"] = time.monotonic() - started
        summary["scenario"] = latest["scenario"]
        summary["planner"] = latest["planner"]
        for process in processes:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
        for process in processes:
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        for log in logs:
            log.close()
        node.destroy_node()
        rclpy.shutdown()
        (args.output / "summary.json").write_text(json.dumps(summary, indent=2))
    print(json.dumps(summary, indent=2), flush=True)
    return 0 if summary["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
