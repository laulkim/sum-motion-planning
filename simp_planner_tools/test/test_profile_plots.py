from __future__ import annotations

import csv
import math
import struct
from pathlib import Path

from simp_planner_tools.profile_plots import (
    ANGULAR_RATE_PROFILE_NAME,
    VELOCITY_PROFILE_NAME,
    phase_change_times,
    render_profile_plots,
)


def _write_run(directory: Path) -> None:
    with (directory / "command_history.csv").open("w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(["publish_time", "vx", "vy", "yaw_rate", "planned_speed",
                         "motion_heading_rate", "beta_rate", "mode"])
        for k in range(400):
            t = 5.0 + 0.1 * k
            writer.writerow([t, 2.0 * math.sin(0.1 * t), 0.3 * math.cos(0.2 * t), 0.1,
                             2.0, math.degrees(0.1) + 1.0, 1.0, k // 100])
        writer.writerow([45.0, "nan", 0.0, 0.0, 0.0, 0.0, 0.0, 0])   # 비정상 행은 건너뛴다
    with (directory / "odom_history.csv").open("w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(["time", "phase_name"])
        for k in range(40):
            writer.writerow([5.0 + k, "first" if k < 20 else "second"])


def _png_size(path: Path) -> tuple[int, int]:
    return struct.unpack(">II", path.read_bytes()[16:24])


def test_profile_plots_are_saved_in_the_run_directory(tmp_path: Path) -> None:
    _write_run(tmp_path)
    saved = render_profile_plots(tmp_path)
    assert [path.name for path in saved] == [VELOCITY_PROFILE_NAME, ANGULAR_RATE_PROFILE_NAME]
    for path in saved:
        assert path.parent == tmp_path
        assert _png_size(path) == (1800, 1500)   # 가로:세로 = 1.2:1
    assert phase_change_times(tmp_path / "odom_history.csv") == [25.0]


def test_profile_plots_skip_an_empty_run(tmp_path: Path) -> None:
    (tmp_path / "command_history.csv").write_text("publish_time,vx,vy,yaw_rate,planned_speed,"
                                                  "motion_heading_rate,beta_rate,mode\n")
    assert render_profile_plots(tmp_path) == []
    assert not (tmp_path / VELOCITY_PROFILE_NAME).exists()


def test_planned_speed_is_signed_by_drive_mode(tmp_path: Path, monkeypatch) -> None:
    from matplotlib.axes import Axes

    _write_run(tmp_path)   # mode 0, 1, 2, 3 가 100행씩, planned_speed = 2.0
    plotted = {}
    original_plot = Axes.plot

    def recording_plot(self, x, y, *args, **kwargs):
        plotted.setdefault(kwargs.get("label"), list(y))
        return original_plot(self, x, y, *args, **kwargs)

    monkeypatch.setattr(Axes, "plot", recording_plot)
    render_profile_plots(tmp_path)
    planned = plotted["Planned speed"]
    assert planned[:100] == [2.0] * 100          # 전진
    assert planned[100:200] == [-2.0] * 100      # 후진
    assert planned[200:300] == [2.0] * 100       # 좌 크랩
    assert planned[300:400] == [-2.0] * 100      # 우 크랩
