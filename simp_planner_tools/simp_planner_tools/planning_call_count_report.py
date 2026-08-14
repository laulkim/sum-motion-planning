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
from matplotlib.ticker import MultipleLocator
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

# These are stage-level counts, not the number of candidates generated inside
# each stage.  Spatial/trajectory fallback passes and allocation searches may
# add another invocation within the same activated plan.
CATEGORIES = (
    ("spatial_path_generation_calls", "Spatial path generation"),
    ("trajectory_planning_calls", "Trajectory planning"),
    ("allocation_calls", "Allocation"),
)

# Per-activated-plan accumulated wall-clock time (ms) for each measured
# block. Spatial path generation and trajectory generation each take a
# different code path (and cost) depending on whether the cycle is a normal
# cruise cycle or a terminal/stop-point cycle, so those two are split into
# separate series. Allocation (which also performs the trajectory-level
# collision check) is a single series.
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
)

# Raw generate_spatial_path_candidate() invocation count, including every
# curvature-violation retry (up to 8x recursive re-attempts with an extended
# length) and the short-path fallback pass. Plotted against the Spatial path
# generation + collision check block time to check whether the two are
# proportional.
SPATIAL_ATTEMPTS_KEY = "spatial_candidate_generation_attempts"

REQUIRED_KEYS = (
    tuple(key for key, _ in CATEGORIES)
    + tuple(key for _, series in BLOCK_TIMING_PANELS for key, _, _ in series)
    + (SPATIAL_ATTEMPTS_KEY,)
)


def create_call_count_figure(samples: list[dict[str, float | int]]) -> Figure:
    """Build a time-aligned call-count/compute-time figure plus one
    sub-plot per measured planner block."""
    time_sec = [float(sample["time_s"]) for sample in samples]

    panel_count = 1 + len(BLOCK_TIMING_PANELS) + 1
    fig, axes = plt.subplots(
        panel_count, 1, figsize=(10, 4.5 + 3.0 * (panel_count - 1)),
        sharex=True,
    )
    count_ax = axes[0]
    for key, label in CATEGORIES:
        values = [int(sample[key]) for sample in samples]
        count_ax.plot(
            time_sec, values, label=f"{label} (mean {mean(values):.2f})",
            marker="o", markersize=2, linewidth=1,
        )
    count_ax.yaxis.set_major_locator(MultipleLocator(1))
    count_ax.yaxis.set_major_formatter(lambda value, _pos: f"{value:.0f}")
    count_ax.set_ylim(bottom=0)
    count_ax.set_ylabel("stage invocations per activated plan")
    count_ax.grid(True, alpha=0.3)

    compute_ax = count_ax.twinx()
    compute_ms = [float(sample["total_compute_time_ms"]) for sample in samples]
    compute_ax.plot(
        time_sec,
        compute_ms,
        color="black",
        alpha=0.65,
        marker=".",
        markersize=3,
        linewidth=1,
        label="Planning computation time",
    )
    compute_ax.set_ylim(bottom=0)
    compute_ax.set_ylabel("planning computation time [ms]")

    count_lines, count_labels = count_ax.get_legend_handles_labels()
    compute_lines, compute_labels = compute_ax.get_legend_handles_labels()
    count_ax.legend(
        count_lines + compute_lines,
        count_labels + compute_labels,
        loc="best",
        fontsize=8,
    )
    count_ax.set_title(
        f"Planner stage invocations and computation time "
        f"({len(samples)} activated plans)"
    )

    for panel_ax, (title, series) in zip(axes[1:], BLOCK_TIMING_PANELS):
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
    attempts_ax = axes[-1]
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

    axes[-1].set_xlabel("time [s]")
    fig.tight_layout()
    return fig


def render_call_count_report(
    samples: list[dict[str, float | int]], output_path: Path
) -> None:
    fig = create_call_count_figure(samples)
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
