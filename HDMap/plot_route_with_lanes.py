"""
큰트랙 + 크랩(1/2/3) 최종 연결 경로를 실제 차선(B2_SURFACELINEMARK) 위에 겹쳐
그리는 마지막 확인용 플롯. 동시에 큰트랙도 크랩과 같은 명명 규칙
(`{name}_connected_ref.csv`)으로 output/에 모아, 최종 path csv가 4개
(큰트랙 + 크랩1/2/3) 한 자리에 있도록 만든다.

입력
  큰트랙_ref_smooth.csv                 - 큰트랙 최종 경로 (output/으로 복사)
  output/크랩{1,2,3}_connected_ref.csv  - build_connected_crab_paths.py 결과
  HDMAP/B2_SURFACELINEMARK.shp          - 실제 차선(중앙선/차로경계선/규제선) 원본
                                           (raw shapefile들은 HDMAP/ 서브폴더로 옮겨짐)

출력 (전부 output/)
  큰트랙_connected_ref.csv   - 큰트랙_ref_smooth.csv 사본, 크랩과 동일한 명명
  route_with_lanes.png       - 차선 + 4개 최종 경로를 겹친 확인용 플롯
"""
import csv
import os
import shutil
import sys

import numpy as np
from osgeo import ogr

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from plot_route_switch_points import (  # noqa: E402
    BIG_TRACK,
    CRAB_COLORS,
    CRAB_TRACKS,
    DATA_DIR,
    SHP_DIR,
)

import matplotlib
matplotlib.use("Agg")
import matplotlib.font_manager as fm
import matplotlib.pyplot as plt

ogr.UseExceptions()

_CJK_FONT_PATH = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
if os.path.exists(_CJK_FONT_PATH):
    fm.fontManager.addfont(_CJK_FONT_PATH)
    matplotlib.rcParams["font.family"] = fm.FontProperties(fname=_CJK_FONT_PATH).get_name()
matplotlib.rcParams["axes.unicode_minus"] = False

OUTPUT_DIR = os.path.join(DATA_DIR, "output")

# REFERENCE_PATH_GUIDE.md 10절 기준: Type=111 황색실선, 211 백색실선,
# 212 백색점선, 311 청색실선. (211/212는 흰 배경 플롯에서 안 보이므로 회색으로 대체)
LANE_STYLE = {
    "111": ("#c9a400", "-"),
    "211": ("#666666", "-"),
    "212": ("#666666", "--"),
    "311": ("#2a78d6", "-"),
}
LANE_DEFAULT_STYLE = ("#999999", ":")


def load_lane_segments(shp_path):
    dataset = ogr.Open(shp_path, 0)
    layer = dataset.GetLayer(0)
    segments = []
    for feature in layer:
        geom = feature.GetGeometryRef()
        pts = np.array([geom.GetPoint(i)[:2] for i in range(geom.GetPointCount())])
        segments.append((feature["Type"], pts))
    return segments


def plot_lanes(ax, segments):
    seen_types = set()
    for lane_type, pts in segments:
        color, style = LANE_STYLE.get(lane_type, LANE_DEFAULT_STYLE)
        label = None
        if lane_type not in seen_types:
            label = f"lane mark Type={lane_type}"
            seen_types.add(lane_type)
        ax.plot(pts[:, 0], pts[:, 1], style, lw=0.8, color=color, alpha=0.8,
                zorder=0, label=label)


def load_xy_only(path):
    with open(path, encoding="utf-8") as f:
        rows = [(float(r["x"]), float(r["y"])) for r in csv.DictReader(f)]
    return np.asarray(rows, dtype=float)


def main():
    os.makedirs(OUTPUT_DIR, exist_ok=True)

    # 4번째 최종 path csv: 큰트랙도 크랩과 같은 명명 규칙으로 output/에 모은다.
    big_src = os.path.join(DATA_DIR, f"{BIG_TRACK}_ref_smooth.csv")
    big_dst = os.path.join(OUTPUT_DIR, f"{BIG_TRACK}_connected_ref.csv")
    shutil.copyfile(big_src, big_dst)

    lane_shp = os.path.join(SHP_DIR, "B2_SURFACELINEMARK.shp")
    lane_segments = load_lane_segments(lane_shp)
    print(f"lane marks loaded: {len(lane_segments)}개 ({lane_shp})")

    fig, ax = plt.subplots(figsize=(12, 12))
    plot_lanes(ax, lane_segments)

    big_xy = load_xy_only(big_dst)
    ax.plot(big_xy[:, 0], big_xy[:, 1], "-", lw=1.4, color="#d95926",
            label=f"{BIG_TRACK} ({len(big_xy)}pt)", zorder=3)

    for crab_no, crab_name in sorted(CRAB_TRACKS.items()):
        path = os.path.join(OUTPUT_DIR, f"{crab_name}_connected_ref.csv")
        xy = load_xy_only(path)
        ax.plot(xy[:, 0], xy[:, 1], "-", lw=1.8, color=CRAB_COLORS[crab_no],
                label=f"{crab_name} connected ({len(xy)}pt)", zorder=4)

    ax.set_title("차선(B2_SURFACELINEMARK) + 최종 경로 (큰트랙 / 크랩1 / 크랩2 / 크랩3)")
    ax.set_xlabel("Easting (m)")
    ax.set_ylabel("Northing (m)")
    ax.axis("equal")
    ax.grid(True, alpha=0.2)
    ax.legend(fontsize=7, loc="best", ncol=2)

    plt.tight_layout()
    out_png = os.path.join(OUTPUT_DIR, "route_with_lanes.png")
    plt.savefig(out_png, dpi=170)
    print("saved:", out_png)

    print("\noutput/ 최종 path csv 4개:")
    for name in [BIG_TRACK] + [CRAB_TRACKS[n] for n in sorted(CRAB_TRACKS)]:
        print(" -", os.path.join(OUTPUT_DIR, f"{name}_connected_ref.csv"))


if __name__ == "__main__":
    main()
