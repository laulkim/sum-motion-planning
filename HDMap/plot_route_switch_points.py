"""
큰트랙 + 크랩(1/2/3) 레퍼런스 경로와, ref.txt에 적힌 정보를 이용해 계산한
"체크포인트"(큰트랙을 이루는 A1 노드) / "switch 지점"(각 크랩이 큰트랙에서
갈라지는 위치)을 한 그림에 표시하는 확인용 플롯.

입력
  {트랙명}_ref_smooth.csv  (x, y, yaw)   - preprocess_and_plot_ref_paths.py 출력
  ref.txt                                 - 큰트랙을 이루는 30개 A2 링크와 그 경계
                                             31개 A1 노드 목록, 그리고 각 크랩이
                                             큰트랙의 어느 링크 부근에서 갈라지는지
                                             적어둔 메모
  A1_NODE.shp, A2_LINK.shp                - 체크포인트/anchor 링크의 실제 UTM 좌표
                                             조회용 (REFERENCE_PATH_GUIDE.md 3.3절과
                                             동일한 GDAL/OGR 접근 방식)

각 크랩의 switch 지점은 두 가지 방법으로 각각 계산해서 같이 표시한다.
  (a) 크랩 경로 자신의 시작점을 큰트랙 폴리라인에 투영한 최근접점
  (b) ref.txt에 적힌 anchor A2 링크의 시작점을 큰트랙 폴리라인에 투영한 최근접점
두 점이 서로 크게 어긋나면 ref.txt 해석(어느 링크가 anchor인지) 또는 좌표계
가정이 잘못됐다는 신호이므로, 콘솔에 두 station 차이를 같이 출력한다.

출력: route_switch_points.png
"""
import csv
import math
import os
import re

import numpy as np
from osgeo import ogr

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

DATA_DIR = os.path.dirname(os.path.abspath(__file__))
SHP_DIR = os.path.join(DATA_DIR, "HDMAP")  # 원본 HD map shapefile 보관 폴더
BIG_TRACK = "큰트랙"
CRAB_TRACKS = {1: "크랩1", 2: "크랩2", 3: "크랩3"}
CRAB_COLORS = {1: "tab:red", 2: "tab:green", 3: "tab:purple"}

TOKEN_RE = re.compile(r"A[12]235W\d{6}")
CRAB_ANCHOR_RE = re.compile(r"크랩(\d)[^A]*?(A2235W\d{6})\s*사이")


def load_xy(name):
    path = os.path.join(DATA_DIR, f"{name}_ref_smooth.csv")
    with open(path, encoding="utf-8") as f:
        rows = [(float(row["x"]), float(row["y"])) for row in csv.DictReader(f)]
    return np.asarray(rows, dtype=float)


def cum_arclen(xy):
    d = np.hypot(np.diff(xy[:, 0]), np.diff(xy[:, 1]))
    return np.concatenate([[0.0], np.cumsum(d)])


def project_point(xy_path, s_path, point):
    """xy_path 폴리라인 위에서 point에 가장 가까운 위치를 찾는다.

    Returns (station_s, projected_xy, distance).
    """
    best = None
    for i in range(len(xy_path) - 1):
        a, b = xy_path[i], xy_path[i + 1]
        ab = b - a
        l2 = float(ab @ ab)
        t = 0.0 if l2 < 1e-12 else float(np.clip(((point - a) @ ab) / l2, 0.0, 1.0))
        proj = a + t * ab
        dist = float(np.hypot(*(point - proj)))
        if best is None or dist < best[2]:
            s = s_path[i] + t * (s_path[i + 1] - s_path[i])
            best = (s, proj, dist)
    return best


def parse_ref_txt(path):
    """ref.txt를 블록(빈 줄로 구분) 단위로 읽어 큰트랙 링크/노드 목록과 각
    크랩의 anchor 링크·진입 링크·연결 노드를 뽑아낸다."""
    text = open(path, encoding="utf-8").read()
    blocks = [b for b in re.split(r"\n\s*\n", text) if b.strip()]

    big_track_links, big_track_nodes = [], []
    crabs = {}
    for block in blocks:
        crab_match = CRAB_ANCHOR_RE.search(block)
        tokens = TOKEN_RE.findall(block)
        tokens_a2 = [t for t in tokens if t.startswith("A2235W")]
        tokens_a1 = [t for t in tokens if t.startswith("A1235W")]
        if crab_match:
            crab_no = int(crab_match.group(1))
            anchor = crab_match.group(2)
            crabs[crab_no] = {
                "anchor_link": anchor,
                "approach_links": [t for t in tokens_a2 if t != anchor],
                "connector_nodes": tokens_a1,
            }
        elif tokens_a1 and not tokens_a2:
            big_track_nodes = tokens_a1
        elif tokens_a2 and not tokens_a1:
            big_track_links = tokens_a2

    if len(big_track_links) + 1 != len(big_track_nodes):
        raise ValueError(
            f"ref.txt parse mismatch: {len(big_track_links)} links but "
            f"{len(big_track_nodes)} nodes (expected links+1)"
        )
    return big_track_links, big_track_nodes, crabs


def load_point_field(shp_path, wanted_ids):
    """A1_NODE.shp 등 Point 레이어에서 ID -> (x, y)."""
    dataset = ogr.Open(shp_path, 0)
    layer = dataset.GetLayer(0)
    remaining = set(wanted_ids)
    out = {}
    for feature in layer:
        fid = feature["ID"]
        if fid in remaining:
            geom = feature.GetGeometryRef()
            out[fid] = (geom.GetX(), geom.GetY())
    return out


def load_link_start_xy(shp_path, wanted_ids):
    """A2_LINK.shp 등 LineString 레이어에서 ID -> 첫 점 (x, y)."""
    dataset = ogr.Open(shp_path, 0)
    layer = dataset.GetLayer(0)
    remaining = set(wanted_ids)
    out = {}
    for feature in layer:
        fid = feature["ID"]
        if fid in remaining:
            geom = feature.GetGeometryRef()
            x, y, *_ = geom.GetPoint(0)
            out[fid] = (x, y)
    return out


def main():
    big_xy = load_xy(BIG_TRACK)
    big_s = cum_arclen(big_xy)

    big_links, big_nodes, crabs = parse_ref_txt(os.path.join(DATA_DIR, "ref.txt"))

    all_connector_nodes = sum((c["connector_nodes"] for c in crabs.values()), [])
    node_xy = load_point_field(
        os.path.join(SHP_DIR, "A1_NODE.shp"), big_nodes + all_connector_nodes
    )
    anchor_ids = [c["anchor_link"] for c in crabs.values()]
    anchor_xy = load_link_start_xy(os.path.join(SHP_DIR, "A2_LINK.shp"), anchor_ids)

    checkpoints = []
    for idx, node_id in enumerate(big_nodes):
        if node_id not in node_xy:
            print(f"[warn] checkpoint node {node_id} not found in A1_NODE.shp")
            continue
        s, proj, dist = project_point(big_xy, big_s, np.asarray(node_xy[node_id]))
        checkpoints.append((idx, node_id, s, proj, dist))

    crab_results = {}
    for crab_no, crab_info in sorted(crabs.items()):
        crab_name = CRAB_TRACKS[crab_no]
        crab_xy = load_xy(crab_name)
        s_path, proj_path, dist_path = project_point(big_xy, big_s, crab_xy[0])
        entry = {
            "crab_xy": crab_xy,
            "anchor_link": crab_info["anchor_link"],
            "path_start_switch": (s_path, proj_path, dist_path),
        }
        anchor_id = crab_info["anchor_link"]
        if anchor_id in anchor_xy:
            s_a, proj_a, dist_a = project_point(big_xy, big_s, np.asarray(anchor_xy[anchor_id]))
            entry["anchor_link_switch"] = (s_a, proj_a, dist_a)
            entry["agreement_gap_m"] = float(np.hypot(*(proj_path - proj_a)))
        else:
            print(f"[warn] anchor link {anchor_id} not found in A2_LINK.shp")
        crab_results[crab_no] = entry

    # ---- console summary ------------------------------------------------
    print(f"big track: {len(big_links)} links / {len(big_nodes)} checkpoint nodes, "
          f"total length {big_s[-1]:.1f} m")
    for idx, node_id, s, _proj, dist in checkpoints:
        print(f"  checkpoint[{idx:2d}] {node_id}  s={s:7.2f} m  "
              f"(node-to-smoothed-path residual {dist * 100:.1f} cm)")
    print()
    for crab_no, entry in sorted(crab_results.items()):
        s_path, _, dist_path = entry["path_start_switch"]
        line = (f"{CRAB_TRACKS[crab_no]}: anchor_link={entry['anchor_link']}  "
                f"switch_s(from crab path start)={s_path:7.2f} m "
                f"(residual {dist_path * 100:.1f} cm)")
        if "anchor_link_switch" in entry:
            s_a, _, dist_a = entry["anchor_link_switch"]
            line += (f"  switch_s(from anchor link)={s_a:7.2f} m "
                     f"(residual {dist_a * 100:.1f} cm)  "
                     f"agreement_gap={entry['agreement_gap_m']:.2f} m")
        print(line)

    # ---- plot -------------------------------------------------------------
    fig, ax = plt.subplots(figsize=(11, 11))
    ax.plot(big_xy[:, 0], big_xy[:, 1], "-", lw=1.2, color="#2a78d6",
            label=f"{BIG_TRACK} ({len(big_xy)}pt)", zorder=1)

    cp_xy = np.asarray([proj for _, _, _, proj, _ in checkpoints])
    ax.plot(cp_xy[:, 0], cp_xy[:, 1], "+", ms=8, mew=1.3, color="#333333",
             label=f"checkpoint (A1 node, {len(checkpoints)}개)", zorder=2)
    for idx, node_id, _s, proj, _dist in checkpoints:
        ax.annotate(str(idx), proj, textcoords="offset points", xytext=(3, 3),
                    fontsize=6, color="#555555")

    for crab_no, entry in sorted(crab_results.items()):
        color = CRAB_COLORS[crab_no]
        crab_xy = entry["crab_xy"]
        ax.plot(crab_xy[:, 0], crab_xy[:, 1], "-", lw=1.4, color=color,
                 label=f"{CRAB_TRACKS[crab_no]} ({len(crab_xy)}pt)", zorder=3)

        s_path, proj_path, dist_path = entry["path_start_switch"]
        ax.plot(*proj_path, "*", ms=16, color=color, markeredgecolor="black",
                 markeredgewidth=0.6, zorder=4,
                 label=f"{CRAB_TRACKS[crab_no]} switch (from crab path start)")
        ax.plot(*crab_xy[0], "o", ms=6, color=color, markeredgecolor="black",
                 markeredgewidth=0.6, zorder=4)

        if "anchor_link_switch" in entry:
            _s_a, proj_a, _dist_a = entry["anchor_link_switch"]
            ax.plot(*proj_a, "x", ms=12, mew=2.2, color=color, zorder=4,
                     label=f"{CRAB_TRACKS[crab_no]} switch (from anchor link "
                           f"{entry['anchor_link']})")

        ax.annotate(
            f"{CRAB_TRACKS[crab_no]}\nanchor={entry['anchor_link']}\n"
            f"s={s_path:.1f} m",
            proj_path, textcoords="offset points", xytext=(8, -12),
            fontsize=8, color=color,
        )

    ax.set_title("큰트랙 checkpoints + 크랩(1/2/3) switch 지점")
    ax.set_xlabel("Easting (m)")
    ax.set_ylabel("Northing (m)")
    ax.axis("equal")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=7, loc="best")

    plt.tight_layout()
    out_png = os.path.join(DATA_DIR, "route_switch_points.png")
    plt.savefig(out_png, dpi=150)
    print("\nsaved:", out_png)


if __name__ == "__main__":
    main()
