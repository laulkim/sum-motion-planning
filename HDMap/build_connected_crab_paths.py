"""
큰트랙 -> (직각 진입 커넥터) -> 크랩(1/2/3) 으로 이어지는 연결 경로를 만들고 플롯한다.

crab 기동은 차량이 자세를 바꾸지 않고 옆으로 이동하는 모드다 (scenario_manager_node.py
의 crab_switch 처리, motion_yaw = body_yaw + pi/2 와 동일한 규약). 그래서 큰트랙에서
크랩 경로로 넘어가는 진입 커넥터도 큰트랙 접선에 대해 스무스하게 곡선으로 붙는 게
아니라, 직각(perpendicular)으로 곧게 붙어야 한다.

이 스크립트는 그 커넥터를 다음처럼 만든다.

  1) 크랩 경로 자신의 시작점을 큰트랙 폴리라인에 투영해 최근접점(★, switch point)을
     구한다. 최근접점은 정의상 그 지점에서의 트랙 접선과 수직이므로(실제로 확인한
     결과 세 크랩 모두 89.9~90.0도), 별도 처리 없이 이미 직각 조건을 만족한다.
  2) ★과 크랩 경로 시작점을 "직선으로" 보간한다 (스플라인 아님). 0.2 m 간격으로
     재샘플링하고 yaw는 그 직선의 방위각으로 고정한다 -- crab 모드가 이동 방향을
     유지한 채 옆으로 미끄러지는 동작이라는 점과 일치한다.
  3) 그 뒤에 크랩 경로 자신의 스무딩된 정점({crab}_ref_smooth.csv)을 그대로 이어붙인다
     (크랩 경로 자체는 다시 스무딩하지 않는다).

큰트랙에서 각 크랩으로 넘어가는 station인 S_switch는 scenario_definition.py의
ScenarioPhase.switch_s 입력값과 동일한 의미이므로, 확인/플롯과 별도로
output/switch_stations.txt 에도 정리해 남긴다.

출력은 모두 output/ 서브디렉토리에 저장해 HDMap 폴더가 지저분해지지 않게 한다.
  output/크랩{N}_connected_ref.csv   - 최종 연결 경로 (x, y, yaw): 직선 커넥터 + 크랩 경로
  output/connected_crab_paths.png    - 확인용 플롯
  output/switch_stations.txt         - 크랩별 S_switch (큰트랙 기준 station, m)
"""
import csv
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from plot_route_switch_points import (  # noqa: E402
    BIG_TRACK,
    CRAB_COLORS,
    CRAB_TRACKS,
    DATA_DIR,
    cum_arclen,
    load_xy,
    project_point,
)

import matplotlib
matplotlib.use("Agg")
import matplotlib.font_manager as fm
import matplotlib.pyplot as plt

_CJK_FONT_PATH = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
if os.path.exists(_CJK_FONT_PATH):
    fm.fontManager.addfont(_CJK_FONT_PATH)
    matplotlib.rcParams["font.family"] = fm.FontProperties(fname=_CJK_FONT_PATH).get_name()
matplotlib.rcParams["axes.unicode_minus"] = False

OUTPUT_DIR = os.path.join(DATA_DIR, "output")
CONNECTOR_SPACING = 0.2  # m, 크랩 경로 스무딩과 동일한 간격


def load_smooth_xy_yaw(name):
    path = os.path.join(DATA_DIR, f"{name}_ref_smooth.csv")
    with open(path, encoding="utf-8") as f:
        rows = [(float(r["x"]), float(r["y"]), float(r["yaw"])) for r in csv.DictReader(f)]
    arr = np.asarray(rows, dtype=float)
    return arr[:, :2], arr[:, 2]


def straight_connector(start_xy, end_xy, spacing=CONNECTOR_SPACING):
    """start_xy -> end_xy 를 곧은 직선으로, 일정 간격으로 보간한다 (마지막 점은
    end_xy와 중복되므로 제외하고 반환)."""
    length = float(np.hypot(*(end_xy - start_xy)))
    heading = math.atan2(end_xy[1] - start_xy[1], end_xy[0] - start_xy[0])
    count = max(1, int(round(length / spacing)))
    t = np.linspace(0.0, 1.0, count, endpoint=False)
    xy = start_xy[None, :] + t[:, None] * (end_xy - start_xy)[None, :]
    yaw = np.full(count, heading)
    return xy, yaw, length, heading


def save_csv(path, xy, yaw):
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["x", "y", "yaw"])
        for (x, y), h in zip(xy, yaw):
            w.writerow([f"{x:.3f}", f"{y:.3f}", f"{h:.6f}"])


def main():
    os.makedirs(OUTPUT_DIR, exist_ok=True)

    big_xy = load_xy(BIG_TRACK)
    big_s = cum_arclen(big_xy)

    fig, ax = plt.subplots(figsize=(11, 11))
    ax.plot(big_xy[:, 0], big_xy[:, 1], "-", lw=1.2, color="#2a78d6",
            label=f"{BIG_TRACK} ({len(big_xy)}pt)", zorder=1)

    switch_lines = []
    for crab_no, crab_name in sorted(CRAB_TRACKS.items()):
        color = CRAB_COLORS[crab_no]
        crab_xy, crab_yaw = load_smooth_xy_yaw(crab_name)

        s_star, proj_star, dist_star = project_point(big_xy, big_s, crab_xy[0])
        conn_xy, conn_yaw, conn_len, conn_heading = straight_connector(proj_star, crab_xy[0])

        full_xy = np.vstack([conn_xy, crab_xy])
        full_yaw = np.concatenate([conn_yaw, crab_yaw])

        out_csv = os.path.join(OUTPUT_DIR, f"{crab_name}_connected_ref.csv")
        save_csv(out_csv, full_xy, full_yaw)

        line = (f"{crab_name}: S_switch={s_star:7.2f} m  (연결 커넥터 길이 {conn_len:.2f} m, "
                f"heading {math.degrees(conn_heading):.1f} deg)")
        print(line)
        print(f"   -> {out_csv}  ({len(conn_xy)}pt 커넥터 + {len(crab_xy)}pt 크랩 경로)")
        switch_lines.append((crab_name, s_star, conn_len, math.degrees(conn_heading)))

        ax.plot(full_xy[:, 0], full_xy[:, 1], "-", lw=1.6, color=color,
                label=f"{crab_name} connected ({len(full_xy)}pt)", zorder=3)
        ax.plot(conn_xy[:, 0], conn_xy[:, 1], "--", lw=2.2, color=color, alpha=0.6, zorder=3)
        ax.plot(*proj_star, "*", ms=16, color=color, markeredgecolor="black",
                markeredgewidth=0.6, zorder=4, label=f"{crab_name} switch (S_switch)")
        ax.annotate(f"{crab_name}\nS_switch={s_star:.1f} m", proj_star,
                    textcoords="offset points", xytext=(8, -10), fontsize=8, color=color)

    ax.set_title("큰트랙 -> 직각 진입 커넥터 -> 크랩(1/2/3) 연결 경로")
    ax.set_xlabel("Easting (m)")
    ax.set_ylabel("Northing (m)")
    ax.axis("equal")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=7, loc="best")

    plt.tight_layout()
    out_png = os.path.join(OUTPUT_DIR, "connected_crab_paths.png")
    plt.savefig(out_png, dpi=150)
    print("\nsaved:", out_png)

    txt_path = os.path.join(OUTPUT_DIR, "switch_stations.txt")
    with open(txt_path, "w", encoding="utf-8") as f:
        f.write("큰트랙 기준 크랩 switch station (S_switch)\n")
        f.write("scenario_definition.py의 ScenarioPhase.switch_s 입력값과 동일한 의미.\n")
        f.write(f"({BIG_TRACK} 총 길이 {big_s[-1]:.2f} m)\n\n")
        for crab_name, s_star, conn_len, heading_deg in switch_lines:
            f.write(f"{crab_name}: S_switch = {s_star:.2f} m "
                    f"(진입 커넥터 길이 {conn_len:.2f} m, heading {heading_deg:.1f} deg)\n")
    print("saved:", txt_path)


if __name__ == "__main__":
    main()
