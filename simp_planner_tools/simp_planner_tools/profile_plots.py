"""시나리오 종료 시 실행 폴더에 저장하는 속도/각속도 프로필 그림.

debug_plot_node가 기록한 command_history.csv(플래너가 매 제어 주기 내보낸
/planner/executed_command)와 odom_history.csv(단계 이름)를 읽어 두 그림을 만든다.

  velocity_profile.png      V_x, V_y (차체 속도 명령), Planned speed (계획 속도에 주행 모드
                            부호를 붙인 값: 후진/우측 크랩은 음수)
  angular_rate_profile.png  yaw rate r, slip angle rate beta_dot, course-angle rate chi_dot
                            (chi_dot = r + beta_dot), 모두 deg/s

단계가 바뀌는 시점에는 글자 없는 옅은 세로 점선을 긋는다.
"""
from __future__ import annotations

import csv
import math
from pathlib import Path

import numpy as np
from matplotlib.figure import Figure

VELOCITY_PROFILE_NAME = "velocity_profile.png"
ANGULAR_RATE_PROFILE_NAME = "angular_rate_profile.png"

FIGURE_SIZE_IN = (12.0, 10.0)  # 가로:세로 = 1.2:1, 150 dpi -> 1800 x 1500 px
FIGURE_DPI = 150
TITLE_SIZE = 28
LABEL_SIZE = 24
TICK_SIZE = 18
LEGEND_SIZE = 18
LINE_WIDTH = 2.2
# planned_speed는 크기라서 주행 모드로 부호를 붙인다 (0 전진, 1 후진, 2 좌 크랩, 3 우 크랩)
NEGATIVE_SPEED_MODES = (1, 3)
LEGEND_HEADROOM = 0.40         # 범례가 선을 가리지 않도록 위쪽에 남기는 여백 (데이터 범위 비율)


def _read_columns(path: Path, names: tuple[str, ...]) -> dict[str, np.ndarray]:
    columns: dict[str, list[float]] = {name: [] for name in names}
    with path.open(newline="", encoding="utf-8") as file:
        for row in csv.DictReader(file):
            try:
                values = [float(row[name]) for name in names]
            except (KeyError, TypeError, ValueError):
                continue
            if all(math.isfinite(value) for value in values):
                for name, value in zip(names, values):
                    columns[name].append(value)
    return {name: np.asarray(values, dtype=float) for name, values in columns.items()}


def phase_change_times(odom_csv: Path) -> list[float]:
    """odom_history.csv에서 단계 이름이 바뀐 시각들."""
    times: list[float] = []
    previous = None
    with odom_csv.open(newline="", encoding="utf-8") as file:
        for row in csv.DictReader(file):
            phase = row.get("phase_name")
            if previous is not None and phase != previous:
                try:
                    times.append(float(row["time"]))
                except (KeyError, TypeError, ValueError):
                    pass
            previous = phase
    return times


def _style_axes(axes, title: str, ylabel: str) -> None:
    axes.set_title(title, fontsize=TITLE_SIZE, fontweight="bold")
    axes.set_xlabel("time [s]", fontsize=LABEL_SIZE, fontweight="bold")
    axes.set_ylabel(ylabel, fontsize=LABEL_SIZE, fontweight="bold")
    axes.tick_params(labelsize=TICK_SIZE)
    for label in axes.get_xticklabels() + axes.get_yticklabels():
        label.set_fontweight("bold")
    axes.grid(True)
    axes.axhline(0.0, color="#78909c", linewidth=0.8, zorder=1)
    axes.legend(loc="upper right", prop={"size": LEGEND_SIZE, "weight": "bold"})


def _set_limits(axes, time: np.ndarray, series: list[np.ndarray]) -> None:
    low = min(0.0, *(float(np.min(values)) for values in series))
    high = max(0.0, *(float(np.max(values)) for values in series))
    span = max(high - low, 1.0e-3)
    axes.set_ylim(low - 0.05 * span, high + LEGEND_HEADROOM * span)
    end = float(time[-1]) if len(time) else 1.0
    axes.set_xlim(-0.02 * end, 1.02 * end)


def _draw_phase_lines(axes, times: list[float]) -> None:
    for value in times:
        axes.axvline(value, color="#9e9e9e", linestyle=":", linewidth=1.3, alpha=0.8, zorder=1)


def render_profile_plots(session_dir: Path | str) -> list[Path]:
    """session_dir의 command_history.csv로 두 그림을 저장하고 저장한 경로를 돌려준다."""
    session_dir = Path(session_dir)
    data = _read_columns(
        session_dir / "command_history.csv",
        ("publish_time", "vx", "vy", "yaw_rate", "planned_speed",
         "motion_heading_rate", "beta_rate", "mode"),
    )
    if len(data["publish_time"]) < 2:
        return []
    origin = float(data["publish_time"][0])
    time = data["publish_time"] - origin
    odom_csv = session_dir / "odom_history.csv"
    phase_lines = ([value - origin for value in phase_change_times(odom_csv)]
                   if odom_csv.exists() else [])

    planned_speed = np.where(np.isin(np.rint(data["mode"]), NEGATIVE_SPEED_MODES),
                             -data["planned_speed"], data["planned_speed"])
    saved: list[Path] = []

    figure = Figure(figsize=FIGURE_SIZE_IN, dpi=FIGURE_DPI)
    axes = figure.add_subplot(1, 1, 1)
    _draw_phase_lines(axes, phase_lines)
    axes.plot(time, data["vx"], color="#1f77b4", linewidth=LINE_WIDTH, label=r"$V_x$")
    axes.plot(time, data["vy"], color="#ff7f0e", linewidth=LINE_WIDTH, label=r"$V_y$")
    axes.plot(time, planned_speed, color="black", linestyle="--",
              linewidth=LINE_WIDTH, label="Planned speed")
    _set_limits(axes, time, [data["vx"], data["vy"], planned_speed])
    _style_axes(axes, "Velocity profile", "velocity [m/s]")
    figure.tight_layout()
    path = session_dir / VELOCITY_PROFILE_NAME
    figure.savefig(path)
    saved.append(path)

    yaw_rate_deg = np.degrees(data["yaw_rate"])      # rad/s -> deg/s
    figure = Figure(figsize=FIGURE_SIZE_IN, dpi=FIGURE_DPI)
    axes = figure.add_subplot(1, 1, 1)
    _draw_phase_lines(axes, phase_lines)
    axes.plot(time, yaw_rate_deg, color="#1f77b4", linewidth=LINE_WIDTH,
              label=r"Yaw rate $r$")
    axes.plot(time, data["beta_rate"], color="#2ca02c", linewidth=LINE_WIDTH,
              label=r"Velocity-direction angle rate $\dot{\beta}$")
    axes.plot(time, data["motion_heading_rate"], color="black", linestyle="--",
              linewidth=LINE_WIDTH, label=r"Course-angle rate $\dot{\chi}$")
    _set_limits(axes, time, [yaw_rate_deg, data["beta_rate"], data["motion_heading_rate"]])
    _style_axes(axes, "Angular rate profile", "angular rate [deg/s]")
    figure.tight_layout()
    path = session_dir / ANGULAR_RATE_PROFILE_NAME
    figure.savefig(path)
    saved.append(path)
    return saved
