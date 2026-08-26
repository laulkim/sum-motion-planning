from __future__ import annotations

import json
from datetime import datetime
from pathlib import Path
from statistics import mean

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import rclpy
from matplotlib.figure import Figure
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

# These are stage-level counts, not the number of candidates generated inside
# each stage.  Spatial/trajectory fallback passes and allocation searches may
# add another invocation within the same activated plan.  Not plotted on the
# summary panel (only computation time is), but still required/collected
# since the per-block timing panels below depend on the same JSON payload.
CATEGORIES = (
    ("spatial_path_generation_calls", "Spatial path generation"),
    ("trajectory_planning_calls", "Trajectory planning"),
    ("allocation_calls", "Allocation"),
)

# Matches the hardcoded "deadline_ms" the planner node publishes in its
# status JSON (10 Hz planning rate -> 100 ms budget per cycle).
PLANNING_DEADLINE_MS = 100.0

# Per-activated-plan accumulated wall-clock time (ms) for each measured
# block. Spatial path generation and trajectory generation each take a
# different code path (and cost) depending on whether the cycle is a normal
# cruise cycle or a terminal/stop-point cycle, so those two are split into
# separate series. Allocation (which also performs the trajectory-level
# collision check) is a single series. Handover state prediction is the
# pre-planning step that forward-integrates the currently active trajectory
# to the scheduled handover time -- it runs once per planning cycle, same
# cadence as the other three, so it is plotted the same way.
BLOCK_TIMING_PANELS = (
    (
        "Spatial path generation + collision check",
        (
            ("spatial_normal_ms", "normal", "tab:blue"),
            ("spatial_terminal_ms", "terminal / stop-region", "tab:orange"),
        ),
    ),
    (
        "Trajectory generation",
        (
            ("trajectory_normal_ms", "cruise", "tab:blue"),
            ("trajectory_terminal_ms", "stop", "tab:orange"),
        ),
    ),
    (
        "Motion allocation + collision check",
        (
            ("allocation_block_ms", "allocation", "tab:green"),
        ),
    ),
    (
        "Handover state prediction",
        (
            ("handover_prediction_ms", "handover prediction", "tab:purple"),
        ),
    ),
)

# Raw generate_spatial_path_candidate() invocation count, including every
# curvature-violation retry (up to 8x recursive re-attempts with an extended
# length) and the short-path fallback pass. Plotted against the Spatial path
# generation + collision check block time to check whether the two are
# proportional.
SPATIAL_ATTEMPTS_KEY = "spatial_candidate_generation_attempts"

# Costmap distance-field rebuild cost. Unlike the four panels above, this is
# event-driven (only runs when a new /costmap message arrives, on a
# completely different callback than planning), not once per activated plan.
# It is published as "the most recently known rebuild cost/count at the time
# of this plan" rather than something that happened during this plan cycle,
# so most samples will simply repeat the last observed value.
COSTMAP_BUILD_MS_KEY = "costmap_build_ms"
COSTMAP_REBUILD_COUNT_KEY = "costmap_rebuild_count"

# Per-activated-plan time (ms), broken down by computation kind rather than by
# pipeline stage. This is orthogonal to BLOCK_TIMING_PANELS above (each stage
# there is a mix of these kinds); a candidate/feasibility/collision/ranking
# operation that runs inside e.g. spatial screening is counted here too, so
# these do not sum to total_compute_time_ms and are not meant to.
# candidate_generation_ms sums every generate_spatial_path_candidate() call in
# the cycle (curvature retries and the short-path fallback pass included).
STAGE_BREAKDOWN_SERIES = (
    ("candidate_generation_ms", "Candidate path generation", "tab:blue"),
    ("feasibility_check_ms", "Feasibility check", "tab:orange"),
    ("collision_check_ms", "Collision check", "tab:red"),
    ("ranking_ms", "Ranking / selection", "tab:green"),
)

REQUIRED_KEYS = (
    tuple(key for key, _ in CATEGORIES)
    + tuple(key for _, series in BLOCK_TIMING_PANELS for key, _, _ in series)
    + (SPATIAL_ATTEMPTS_KEY, COSTMAP_BUILD_MS_KEY, COSTMAP_REBUILD_COUNT_KEY)
    + tuple(key for key, _, _ in STAGE_BREAKDOWN_SERIES)
)

# Detail panels are laid out in a grid below the summary panel instead of a
# single ever-growing column: BLOCK_TIMING_PANELS fill the grid in order,
# then the two dual-axis debug panels (candidate-generation attempts, and
# costmap rebuild cost/count) take the remaining slots.
_DETAIL_PANEL_COUNT = len(BLOCK_TIMING_PANELS) + 2
_DETAIL_GRID_COLUMNS = 2
_DETAIL_GRID_ROWS = -(-_DETAIL_PANEL_COUNT // _DETAIL_GRID_COLUMNS)


def create_call_count_figure(samples: list[dict[str, float | int]]) -> Figure:
    """Build a time-aligned call-count/compute-time summary panel plus a
    grid of one sub-plot per measured planner block."""
    time_sec = [float(sample["time_s"]) for sample in samples]

    fig = plt.figure(
        figsize=(7.0 * _DETAIL_GRID_COLUMNS, 4.5 + 3.2 * _DETAIL_GRID_ROWS)
    )
    gs = fig.add_gridspec(1 + _DETAIL_GRID_ROWS, _DETAIL_GRID_COLUMNS)

    top_ax = fig.add_subplot(gs[0, :])
    compute_ms = [float(sample["total_compute_time_ms"]) for sample in samples]
    top_ax.plot(
        time_sec,
        compute_ms,
        color="black",
        alpha=0.75,
        marker=".",
        markersize=3,
        linewidth=1,
        label="Planning computation time",
        zorder=3,
    )

    max_ms = max(compute_ms)
    min_ms = min(compute_ms)
    top_ax.axhspan(PLANNING_DEADLINE_MS, max(max_ms, PLANNING_DEADLINE_MS) * 1.05,
                   color="tab:red", alpha=0.08, zorder=0)
    top_ax.axhline(PLANNING_DEADLINE_MS, color="tab:red", linestyle="--", linewidth=1.3,
                   label=f"{PLANNING_DEADLINE_MS:.0f} ms deadline", zorder=1)
    top_ax.axhspan(max_ms * 0.985, max_ms * 1.015, color="tab:orange", alpha=0.20, zorder=0)
    top_ax.axhline(max_ms, color="tab:orange", linestyle=":", linewidth=1.3,
                   label=f"max {max_ms:.2f} ms", zorder=1)
    top_ax.axhspan(min_ms * 0.985, min_ms * 1.015, color="tab:blue", alpha=0.20, zorder=0)
    top_ax.axhline(min_ms, color="tab:blue", linestyle=":", linewidth=1.3,
                   label=f"min {min_ms:.2f} ms", zorder=1)

    top_ax.set_ylim(bottom=0)
    top_ax.set_ylabel("planning computation time [ms]")
    top_ax.grid(True, alpha=0.3)
    top_ax.legend(loc="best", fontsize=8)
    top_ax.set_title(
        f"Planner computation time ({len(samples)} activated plans)"
    )

    detail_axes = [
        fig.add_subplot(
            gs[1 + slot // _DETAIL_GRID_COLUMNS, slot % _DETAIL_GRID_COLUMNS],
            sharex=top_ax,
        )
        for slot in range(_DETAIL_PANEL_COUNT)
    ]

    for panel_ax, (title, series) in zip(detail_axes, BLOCK_TIMING_PANELS):
        for key, label, color in series:
            values = [float(sample[key]) for sample in samples]
            panel_ax.plot(
                time_sec, values, label=f"{label} (mean {mean(values):.2f} ms)",
                color=color, marker="o", markersize=2, linewidth=1,
            )
        panel_ax.set_ylim(bottom=0)
        panel_ax.set_ylabel("block time [ms]")
        panel_ax.set_title(title)
        panel_ax.grid(True, alpha=0.3)
        panel_ax.legend(loc="best", fontsize=8)

    # Debug panel: raw candidate-generation attempt count (every
    # generate_spatial_path_candidate() call, including curvature retries)
    # against the Spatial path generation + collision check block time, to
    # see at a glance whether the two track each other.
    attempts_ax = detail_axes[len(BLOCK_TIMING_PANELS)]
    attempts = [int(sample[SPATIAL_ATTEMPTS_KEY]) for sample in samples]
    attempts_line = attempts_ax.plot(
        time_sec, attempts, color="tab:red", marker="o", markersize=2, linewidth=1,
        label=f"Spatial candidate-generation attempts (mean {mean(attempts):.1f})",
    )
    attempts_ax.set_ylim(bottom=0)
    attempts_ax.set_ylabel("candidate-generation attempts", color="tab:red")
    attempts_ax.tick_params(axis="y", labelcolor="tab:red")
    attempts_ax.grid(True, alpha=0.3)

    spatial_time_ax = attempts_ax.twinx()
    spatial_ms = [
        float(sample["spatial_normal_ms"]) + float(sample["spatial_terminal_ms"])
        for sample in samples
    ]
    spatial_time_line = spatial_time_ax.plot(
        time_sec, spatial_ms, color="tab:blue", alpha=0.7,
        marker=".", markersize=3, linewidth=1,
        label=f"Spatial gen+check time (mean {mean(spatial_ms):.2f} ms)",
    )
    spatial_time_ax.set_ylim(bottom=0)
    spatial_time_ax.set_ylabel("spatial gen+check time [ms]", color="tab:blue")
    spatial_time_ax.tick_params(axis="y", labelcolor="tab:blue")

    attempts_ax.set_title(
        "Spatial candidate-generation attempts vs. block time (proportionality check)"
    )
    attempts_ax.legend(
        attempts_line + spatial_time_line,
        [line.get_label() for line in attempts_line + spatial_time_line],
        loc="best", fontsize=8,
    )

    # Debug panel: costmap distance-field rebuild cost against how many
    # rebuilds have happened so far. This is event-driven (a new /costmap
    # message), not once-per-plan like every other panel here, so most
    # samples repeat the last observed value and the count only steps up
    # when an actual rebuild lands.
    costmap_ax = detail_axes[len(BLOCK_TIMING_PANELS) + 1]
    costmap_ms = [float(sample[COSTMAP_BUILD_MS_KEY]) for sample in samples]
    costmap_ms_line = costmap_ax.plot(
        time_sec, costmap_ms, color="tab:brown", marker=".", markersize=3, linewidth=1,
        label=f"Distance-field rebuild time (mean {mean(costmap_ms):.2f} ms)",
    )
    costmap_ax.set_ylim(bottom=0)
    costmap_ax.set_ylabel("rebuild time [ms]", color="tab:brown")
    costmap_ax.tick_params(axis="y", labelcolor="tab:brown")
    costmap_ax.grid(True, alpha=0.3)

    costmap_count_ax = costmap_ax.twinx()
    costmap_counts = [int(sample[COSTMAP_REBUILD_COUNT_KEY]) for sample in samples]
    costmap_count_line = costmap_count_ax.step(
        time_sec, costmap_counts, color="tab:gray", alpha=0.7, where="post",
        linewidth=1, label="Cumulative rebuild count",
    )
    costmap_count_ax.set_ylabel("cumulative rebuild count", color="tab:gray")
    costmap_count_ax.tick_params(axis="y", labelcolor="tab:gray")

    costmap_ax.set_title("Costmap distance-field rebuild (event-driven, not per-plan)")
    costmap_ax.legend(
        costmap_ms_line + costmap_count_line,
        [line.get_label() for line in costmap_ms_line + costmap_count_line],
        loc="best", fontsize=8,
    )

    for slot in range(_DETAIL_PANEL_COUNT):
        if slot // _DETAIL_GRID_COLUMNS == _DETAIL_GRID_ROWS - 1:
            detail_axes[slot].set_xlabel("time [s]")

    fig.tight_layout()
    return fig


def create_stage_breakdown_figure(samples: list[dict[str, float | int]]) -> Figure:
    """Build a separate window with one subplot per requested computation
    kind (candidate generation, feasibility check, collision check, ranking)
    instead of overlaying all four on a single plot."""
    time_sec = [float(sample["time_s"]) for sample in samples]

    fig, axes = plt.subplots(2, 2, figsize=(13.0, 9.0), sharex=True)
    for ax, (key, label, color) in zip(axes.flat, STAGE_BREAKDOWN_SERIES):
        values = [float(sample[key]) for sample in samples]
        ax.plot(
            time_sec, values, color=color, marker="o", markersize=2, linewidth=1,
            label=f"mean {mean(values):.2f} ms",
        )
        ax.set_ylim(bottom=0)
        ax.set_ylabel("time [ms]")
        ax.set_title(label)
        ax.grid(True, alpha=0.3)
        ax.legend(loc="best", fontsize=8)

    for ax in axes[-1, :]:
        ax.set_xlabel("time [s]")

    fig.suptitle(f"Planning computation by stage ({len(samples)} activated plans)")
    fig.tight_layout()
    return fig


def render_call_count_report(
    samples: list[dict[str, float | int]], output_path: Path
) -> None:
    fig = create_call_count_figure(samples)
    fig.savefig(output_path, dpi=150)
    plt.close(fig)


def render_stage_breakdown_report(
    samples: list[dict[str, float | int]], output_path: Path
) -> None:
    fig = create_stage_breakdown_figure(samples)
    fig.savefig(output_path, dpi=150)
    plt.close(fig)


class PlanningCallCountReportNode(Node):
    """Counts how often spatial-path-generation / trajectory-planning /
    allocation run per activated plan, and how much wall-clock time each
    block spends, then renders a summary PNG on shutdown."""

    def __init__(self) -> None:
        super().__init__("planning_call_count_report_node")
        self.declare_parameter("scenario", "run")
        self.declare_parameter(
            "output_dir", "/home/sum/Desktop/simp_planner/simp_planner_debug"
        )

        scenario = str(self.get_parameter("scenario").value)
        base = Path(str(self.get_parameter("output_dir").value)).expanduser()
        self.session_dir = base / scenario / datetime.now().strftime("%Y%m%d_%H%M%S")
        self.session_dir.mkdir(parents=True, exist_ok=True)

        self.samples: list[dict[str, float | int]] = []
        self.start_time = self.get_clock().now()

        static_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            String, "/planner/status", self.planner_status_callback, static_qos
        )
        self.get_logger().info(
            f"Recording planning call counts, output: {self.session_dir}"
        )

    def elapsed(self) -> float:
        return (self.get_clock().now() - self.start_time).nanoseconds * 1.0e-9

    def planner_status_callback(self, message: String) -> None:
        try:
            value = json.loads(message.data)
        except json.JSONDecodeError:
            return
        plan = value.get("plan", {})
        if not all(key in plan for key in REQUIRED_KEYS):
            return
        if "total_compute_time_ms" not in plan:
            return
        sample: dict[str, float | int] = {
            "time_s": self.elapsed(),
            "total_compute_time_ms": float(plan["total_compute_time_ms"]),
        }
        sample.update({key: int(plan[key]) for key, _ in CATEGORIES})
        sample.update(
            {key: float(plan[key])
             for _, series in BLOCK_TIMING_PANELS for key, _, _ in series}
        )
        sample[SPATIAL_ATTEMPTS_KEY] = int(plan[SPATIAL_ATTEMPTS_KEY])
        sample[COSTMAP_BUILD_MS_KEY] = float(plan[COSTMAP_BUILD_MS_KEY])
        sample[COSTMAP_REBUILD_COUNT_KEY] = int(plan[COSTMAP_REBUILD_COUNT_KEY])
        sample.update(
            {key: float(plan[key]) for key, _, _ in STAGE_BREAKDOWN_SERIES}
        )
        self.samples.append(sample)

    def save_report(self) -> None:
        if not self.samples:
            self.get_logger().warning(
                "No activated plan samples observed; skipping PNG."
            )
            return
        png_path = self.session_dir / "planning_call_counts.png"
        render_call_count_report(self.samples, png_path)
        self.get_logger().info(
            f"Saved {len(self.samples)}-plan time-aligned call-count report to "
            f"{png_path}"
        )

        stage_png_path = self.session_dir / "planning_stage_breakdown.png"
        render_stage_breakdown_report(self.samples, stage_png_path)
        self.get_logger().info(
            f"Saved {len(self.samples)}-plan stage-breakdown report to "
            f"{stage_png_path}"
        )


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = PlanningCallCountReportNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save_report()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
