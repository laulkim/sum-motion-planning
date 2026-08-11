#!/usr/bin/env python3
"""Export a ROS-independent scenario binary for the C++ closed-loop validator."""
from __future__ import annotations

import argparse
import struct
import sys
from pathlib import Path

import numpy as np


def _write_string(file, value: str) -> None:
    data = value.encode("utf-8")
    file.write(struct.pack("<I", len(data)))
    file.write(data)


def export_scenario(
    package_root: Path, scenario_name: str, output_path: Path,
    *, narrow_gap_width: float | None = None,
) -> None:
    sys.path.insert(0, str(package_root))
    from simp_planner_tools.scenario_definition import (  # pylint: disable=import-outside-toplevel
        ScenarioDefinition,
        gate_on_path,
        load_scenario_definition,
        rasterize_scenario_costmap,
    )

    scenario = load_scenario_definition(package_root, scenario_name)
    if narrow_gap_width is not None:
        if scenario.name != "narrow_22m_stop_corridor":
            raise ValueError("--narrow-gap-width is valid only for narrow_22m_stop_corridor")
        path = scenario.phases[0].path
        gate, lower, upper = gate_on_path(
            path, 25.0, lateral_center=0.0, gap_width=float(narrow_gap_width),
            barrier_extent=8.0, obstacle_length=20.0,
        )
        scenario = ScenarioDefinition(
            name=scenario.name, phases=scenario.phases,
            obstacles=(lower, upper), gates=(gate,),
            terminal_margin=scenario.terminal_margin,
            stop_request_distance=scenario.stop_request_distance,
            repeat=scenario.repeat,
            costmap_resolution=scenario.costmap_resolution,
            footprint_circle_count=scenario.footprint_circle_count,
        )
    grid, origin_x, origin_y = rasterize_scenario_costmap(
        scenario, resolution=float(scenario.costmap_resolution), margin=10.0
    )
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("wb") as file:
        file.write(b"SIMPVAL3")
        _write_string(file, scenario.name)
        file.write(struct.pack("<ddBi", scenario.terminal_margin,
                               scenario.stop_request_distance,
                               int(scenario.repeat),
                               int(scenario.footprint_circle_count)))
        file.write(struct.pack("<I", len(scenario.phases)))
        for phase in scenario.phases:
            _write_string(file, phase.name)
            switch_s = -1.0 if phase.switch_s is None else float(phase.switch_s)
            file.write(struct.pack("<iddB", int(phase.mode),
                                   float(phase.cruise_speed), switch_s,
                                   int(phase.path.closed_loop)))
            count = len(phase.path.x)
            file.write(struct.pack("<I", count))
            for values in (phase.path.x, phase.path.y,
                           phase.path.yaw, phase.path.kappa):
                file.write(np.asarray(values, dtype="<f8").tobytes(order="C"))
        height, width = grid.shape
        file.write(struct.pack("<iiddd", width, height,
                               float(scenario.costmap_resolution), origin_x, origin_y))
        file.write(np.asarray(grid, dtype=np.int8).tobytes(order="C"))
        file.write(struct.pack("<I", len(scenario.gates)))
        for gate in scenario.gates:
            file.write(struct.pack("<ddd", gate.s, gate.lateral_center,
                                   gate.gap_width))


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("scenario")
    parser.add_argument("output", type=Path)
    parser.add_argument(
        "--package-root", type=Path,
        default=Path(__file__).resolve().parents[1] / "simp_planner_tools",
    )
    parser.add_argument(
        "--narrow-gap-width", type=float, default=None,
        help="Diagnostic override for narrow_22m_stop_corridor only.",
    )
    args = parser.parse_args()
    export_scenario(
        args.package_root.resolve(), args.scenario, args.output.resolve(),
        narrow_gap_width=args.narrow_gap_width,
    )


if __name__ == "__main__":
    main()
