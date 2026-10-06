"""
ref_ver1.txt 기반 새 레퍼런스 경로 생성.

ref_ver1.txt는 ref.txt와 형식이 다르다. "큰트랙 + 독립된 크랩 곁가지"가 아니라

    1번째 레귤러 경로 -> 2번째 크랩 경로 -> 3번째 레귤러 경로

로 이어지는 하나의 연속된 경로를 정의한다. 각 세그먼트는

    A2235W...          (그 세그먼트를 이루는 A2 링크 ID, 순서대로)
    ...
    N번째 (레귤러|크랩) 경로(...세그먼트 사이)   <- 라벨. 괄호 안은 "이 세그먼트가
                                                 이전 세그먼트의 어느 링크 부근에서
                                                 갈라지는지"에 대한 메모일 뿐, 좌표
                                                 계산에는 안 쓴다.
    (... 같은 ID들을 괄호로 다시 나열 ...)        <- 중복 확인용 블록. 실제로 좌표를
                                                 조회해 검증해 보면(1번째는 각 링크의
                                                 A1 노드 endpoint, 2번째는 A2 링크
                                                 자체 중복, 3번째는 다시 A1 노드
                                                 endpoint) 전부 위 링크 리스트에서
                                                 그대로 유도되는 값과 정확히 일치한다.
                                                 그래서 이 블록은 파싱만 하고 버린다.

형태로 반복된다.

세그먼트를 잇는 규칙 (사용자 확인, 여러 번 시행착오 끝에 확정):
  각 전환은 ref_ver1.txt의 anchor note가 가리키는 "기준 세그먼트"의 곡선에
  다음 세그먼트의 시작점을 수직으로 투영해 최근접점(S_switch)을 찾고,
  [투영점 -> 다음 세그먼트 시작점]을 직선 커넥터로 잇는다. 기준 세그먼트
  자신도 S_switch 이후는 잘라낸다 -- 그 지점에서 이미 다음 세그먼트로 갈아탔으니
  그 뒤는 다시 쓰이지 않기 때문이다.

    레귤러1 -> 크랩2: anchor note "(A2235W000108 세그먼트 사이)" -> 108은
                       레귤러1의 마지막 링크이므로 기준 곡선은 레귤러1.
                       크랩2 시작점을 레귤러1에 투영 -> S_switch=94.81m ->
                       레귤러1은 거기서 꼬리 잘림. 크랩2는 통째로 유지.
    크랩2 -> 레귤러3: anchor note "(A2235W000122 세그먼트 사이)" -> 122는
                       크랩2의 마지막 링크이므로 기준 곡선은 크랩2. 레귤러3
                       시작점(=A1235W000171, 레귤러3 블록에 적어둔 노드)을
                       크랩2에 투영 -> S_switch=143.93m -> 크랩2도 거기서
                       꼬리 잘림. 레귤러3는 통째로 유지.

즉 "기준 세그먼트가 잘리고, 다음 세그먼트는 통째로 유지"가 매번 반복된다 --
레귤러1->크랩2에서는 레귤러1이 기준(잘림)/크랩2가 다음(유지), 크랩2->레귤러3
에서는 크랩2가 기준(잘림)/레귤러3가 다음(유지)이라 방향이 바뀐 것처럼 보일
뿐, 규칙 자체는 하나다.

중간 산출물 (HDMap/, 트랙명_ref.csv / 트랙명_ref_smooth.csv 관례를 그대로 따름):
  레귤러1_ref.csv, 레귤러1_ref_smooth.csv
  크랩2_ref.csv,   크랩2_ref_smooth.csv
  레귤러3_ref.csv, 레귤러3_ref_smooth.csv

최종 출력 (HDMap/output/) -- 시나리오 단계에서 그대로 쓸 수 있도록 세그먼트별
로 별개 csv를 낸다 (하나로 합친 csv는 만들지 않는다):
  레귤러1_connected_ref.csv  - 레귤러1 (꼬리 잘림), 커넥터 없음 (첫 세그먼트)
  크랩2_connected_ref.csv    - 커넥터(레귤러1->크랩2) + 크랩2 (꼬리 잘림)
  레귤러3_connected_ref.csv  - 커넥터(크랩2->레귤러3) + 레귤러3 (통째로)
  ref_ver1_connected_path.png    - 확인용 플롯 (세그먼트별 색 구분 + switch 지점)
  ref_ver1_switch_stations.txt   - 세그먼트 전환 station(S_switch) 기록
"""
import csv
import math
import os
import re
import sys

import numpy as np
from osgeo import ogr

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from preprocess_and_plot_ref_paths import (  # noqa: E402
    dedupe,
    fit_and_resample,
)
from plot_route_switch_points import (  # noqa: E402
    SHP_DIR,
    cum_arclen,
    project_point,
)
from build_connected_crab_paths import (  # noqa: E402
    save_csv,
    smooth_seam,
    straight_connector,
)

ogr.UseExceptions()

import matplotlib
matplotlib.use("Agg")
import matplotlib.font_manager as fm
import matplotlib.pyplot as plt

_CJK_FONT_PATH = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
if os.path.exists(_CJK_FONT_PATH):
    fm.fontManager.addfont(_CJK_FONT_PATH)
    matplotlib.rcParams["font.family"] = fm.FontProperties(fname=_CJK_FONT_PATH).get_name()
matplotlib.rcParams["axes.unicode_minus"] = False

DATA_DIR = os.path.dirname(os.path.abspath(__file__))
OUTPUT_DIR = os.path.join(DATA_DIR, "output")
REF_TXT = os.path.join(DATA_DIR, "ref_ver1.txt")

SEGMENT_COLORS = {"레귤러": "#2a78d6", "크랩": "tab:red"}

LABEL_RE = re.compile(r"(\d+)번째\s*(레귤러|크랩)\s*경로(?:\(([^)]*)\))?")
TOKEN_A2_RE = re.compile(r"A2235W\d{6}")


def parse_ref_ver1(path):
    """ref_ver1.txt를 세그먼트(레귤러/크랩) 순서대로 파싱해 A2 링크 ID 리스트를 뽑는다.

    괄호로 다시 나열된 확인용 블록은 건너뛴다 (docstring 참고 -- 링크 리스트에서
    그대로 유도되는 중복 값임을 실측으로 확인함).
    """
    segments = []
    pending = []
    in_paren_block = False
    with open(path, encoding="utf-8") as f:
        for raw_line in f:
            line = raw_line.strip()
            if not line:
                continue
            if in_paren_block:
                if line.endswith(")"):
                    in_paren_block = False
                continue
            label_match = LABEL_RE.search(line)
            if label_match:
                if not pending:
                    raise ValueError(f"label '{line}' has no preceding link list")
                segments.append({
                    "ordinal": int(label_match.group(1)),
                    "kind": label_match.group(2),
                    "anchor_note": label_match.group(3),
                    "links": pending,
                })
                pending = []
                continue
            if line.startswith("("):
                in_paren_block = not line.endswith(")")
                continue
            tokens = TOKEN_A2_RE.findall(line)
            if tokens:
                pending.extend(tokens)
    if pending:
        raise ValueError(f"leftover unlabeled links (no trailing label found): {pending}")
    segments.sort(key=lambda s: s["ordinal"])
    return segments


def load_link_polylines(shp_path, wanted_ids):
    dataset = ogr.Open(shp_path, 0)
    layer = dataset.GetLayer(0)
    remaining = set(wanted_ids)
    out = {}
    for feature in layer:
        fid = feature["ID"]
        if fid in remaining:
            geom = feature.GetGeometryRef()
            out[fid] = [geom.GetPoint(i)[:2] for i in range(geom.GetPointCount())]
    missing = remaining - out.keys()
    if missing:
        raise ValueError(f"link id(s) not found in {shp_path}: {sorted(missing)}")
    return out


def build_segment_raw_xy(link_ids, link_xy):
    """세그먼트를 이루는 링크들을 순서대로 이어붙인다 (연속된 링크는 경계점을
    공유하므로 두 번째 링크부터는 첫 점을 버린다)."""
    pts = list(link_xy[link_ids[0]])
    for link_id in link_ids[1:]:
        seg_pts = link_xy[link_id]
        gap = math.hypot(seg_pts[0][0] - pts[-1][0], seg_pts[0][1] - pts[-1][1])
        if gap > 0.5:
            raise ValueError(
                f"link {link_id} does not start where the previous link ends "
                f"(gap {gap:.2f} m) -- 세그먼트 내 링크 순서를 확인할 것"
            )
        pts.extend(seg_pts[1:])
    return dedupe(pts)


def trim_regular_against_crab(regular_xy, regular_yaw, crab_anchor_point):
    """레귤러 세그먼트 자신의 곡선에 (인접한 크랩 세그먼트의 끝점인) crab_anchor_point를
    투영해 최근접점(수직으로 내린 점, S_switch)을 찾고, 그 지점 이후(레귤러 자신의
    꼬리)를 잘라낸다. 크랩 세그먼트 쪽은 이 함수로 건드리지 않는다 (모듈 docstring
    참고 -- 크랩은 항상 통째로 유지)."""
    reg_s = cum_arclen(regular_xy)
    s_star, proj_star, dist_star = project_point(regular_xy, reg_s, crab_anchor_point)

    keep_mask = reg_s <= s_star
    trunc_xy = regular_xy[keep_mask].copy()
    trunc_yaw = regular_yaw[keep_mask].copy()
    trunc_xy[-1] = proj_star  # 정확히 투영점에서 끊기도록 보정 (최대 0.2m 오차 제거)
    return trunc_xy, trunc_yaw, s_star, proj_star, dist_star


def main():
    os.makedirs(OUTPUT_DIR, exist_ok=True)

    segments = parse_ref_ver1(REF_TXT)
    print(f"parsed {len(segments)} segments from {REF_TXT}")
    for seg in segments:
        note = f"  (anchor note: {seg['anchor_note']})" if seg["anchor_note"] else ""
        print(f"  [{seg['ordinal']}] {seg['kind']}: {len(seg['links'])} links{note}")

    all_link_ids = [lid for seg in segments for lid in seg["links"]]
    link_xy = load_link_polylines(os.path.join(SHP_DIR, "A2_LINK.shp"), all_link_ids)

    for seg in segments:
        name = f"{seg['kind']}{seg['ordinal']}"
        raw_xy = build_segment_raw_xy(seg["links"], link_xy)
        smooth_xy, smooth_yaw = fit_and_resample(raw_xy)

        raw_csv = os.path.join(DATA_DIR, f"{name}_ref.csv")
        with open(raw_csv, "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["x", "y"])
            for x, y in raw_xy:
                w.writerow([f"{x:.3f}", f"{y:.3f}"])

        smooth_csv = os.path.join(DATA_DIR, f"{name}_ref_smooth.csv")
        save_csv(smooth_csv, np.asarray(smooth_xy), np.asarray(smooth_yaw))

        seg["name"] = name
        seg["raw_xy"] = np.asarray(raw_xy)
        seg["xy"] = np.asarray(smooth_xy)
        seg["yaw"] = np.asarray(smooth_yaw)
        print(f"{name}: raw {len(raw_xy)}pt -> smooth {len(smooth_xy)}pt "
              f"({raw_csv}, {smooth_csv})")

    reg1, crab2, reg3 = segments
    assert reg1["kind"] == "레귤러" and crab2["kind"] == "크랩" and reg3["kind"] == "레귤러", (
        "이 스크립트는 지금 ref_ver1.txt의 레귤러-크랩-레귤러 3세그먼트 구조를 그대로 가정한다"
    )

    # 레귤러1 -> 크랩2: 레귤러1의 곡선에 "크랩2의 시작점"을 투영해 레귤러1 꼬리를
    # 자른다. 크랩2는 통째로 유지.
    reg1_xy, reg1_yaw, s1, proj1, dist1 = trim_regular_against_crab(
        reg1["xy"], reg1["yaw"], crab2["xy"][0]
    )
    conn1_xy, conn1_yaw, conn1_len, conn1_heading = straight_connector(proj1, crab2["xy"][0])

    # 크랩2 -> 레귤러3: ref_ver1.txt에 "3번째 레귤러 경로(A2235W000122 세그먼트
    # 사이)"라고 적혀 있고 A2235W000122는 크랩2의 마지막 링크다. 즉 이 전환의
    # 기준 곡선은 크랩2. 레귤러3 자신의 시작점(A1235W000171, 레귤러3 블록에
    # 적어둔 노드)을 크랩2 곡선에 수직으로 투영해 S_switch를 구하고, 그
    # 투영점 -> 레귤러3 시작점을 직선으로 보간한다. S_switch 이후의 크랩2
    # 구간은 레귤러3로 갈아탄 뒤 다시 쓰이지 않으므로 크랩2 자신의 출력도
    # 그 지점에서 잘라낸다.
    # S_switch 이후의 크랩2 구간(143.93m ~ 자연 끝)은 레귤러3로 이미 갈아탄
    # 뒤라 다시는 안 쓰이므로 크랩2 자신의 출력에서도 잘라낸다.
    reg3_start = reg3["xy"][0]  # == A1235W000171 좌표
    crab2_s = cum_arclen(crab2["xy"])
    s2, proj2, dist2 = project_point(crab2["xy"], crab2_s, reg3_start)
    conn2_xy, conn2_yaw, conn2_len, conn2_heading = straight_connector(proj2, reg3_start)
    reg3_xy, reg3_yaw = reg3["xy"], reg3["yaw"]

    crab2_keep_mask = crab2_s <= s2
    crab2_xy = crab2["xy"][crab2_keep_mask].copy()
    crab2_yaw = crab2["yaw"][crab2_keep_mask].copy()
    crab2_xy[-1] = proj2

    switches = [
        (reg1, crab2["xy"][0], s1, proj1, dist1, conn1_len, conn1_heading,
         "크랩2 시작점 -> 레귤러1에 투영"),
        (reg3, reg3_start, s2, proj2, dist2, conn2_len, conn2_heading,
         "레귤러3 시작점(A1235W000171) -> 크랩2에 투영"),
    ]

    outputs = {}
    outputs[reg1["name"]] = (reg1_xy, reg1_yaw)

    crab2_out_xy = np.vstack([conn1_xy, crab2_xy])
    crab2_out_yaw = np.concatenate([conn1_yaw, crab2_yaw])
    crab2_out_xy, crab2_out_yaw = smooth_seam(crab2_out_xy, crab2_out_yaw)
    outputs[crab2["name"]] = (crab2_out_xy, crab2_out_yaw)

    reg3_out_xy = np.vstack([conn2_xy, reg3_xy])
    reg3_out_yaw = np.concatenate([conn2_yaw, reg3_yaw])
    reg3_out_xy, reg3_out_yaw = smooth_seam(reg3_out_xy, reg3_out_yaw)
    outputs[reg3["name"]] = (reg3_out_xy, reg3_out_yaw)

    for name, (xy, yaw) in outputs.items():
        out_csv = os.path.join(OUTPUT_DIR, f"{name}_connected_ref.csv")
        save_csv(out_csv, xy, yaw)
        print(f"saved: {out_csv}  ({len(xy)}pt)")

    txt_path = os.path.join(OUTPUT_DIR, "ref_ver1_switch_stations.txt")
    with open(txt_path, "w", encoding="utf-8") as f:
        f.write("ref_ver1 세그먼트 전환 station (S_switch, 기준 세그먼트 자신의 곡선 기준)\n")
        f.write("기준 세그먼트는 S_switch에서 꼬리가 잘리고, 다음 세그먼트는 통째로 유지된다.\n\n")
        for seg, anchor_pt, s_star, proj_star, dist_star, conn_len, conn_heading, desc in switches:
            line = (f"{seg['name']}: S_switch={s_star:7.2f} m ({desc}, "
                    f"투영 잔차 {dist_star*100:.1f} cm, 커넥터 길이 {conn_len:.2f} m, "
                    f"heading {math.degrees(conn_heading):.1f} deg)")
            print(line)
            f.write(line + "\n")
    print("saved:", txt_path)

    fig, ax = plt.subplots(figsize=(11, 11))
    for seg in segments:
        color = SEGMENT_COLORS.get(seg["kind"], "#999999")
        ax.plot(seg["xy"][:, 0], seg["xy"][:, 1], "-", lw=1.0, color=color, alpha=0.25,
                zorder=1, label=f"{seg['name']} raw (자르기 전)")
    for name, (xy, yaw) in outputs.items():
        ax.plot(xy[:, 0], xy[:, 1], "-", lw=1.8,
                color=SEGMENT_COLORS.get(name.rstrip("0123456789"), "#222222"),
                label=f"{name}_connected ({len(xy)}pt)", zorder=3)
    for seg, anchor_pt, s_star, proj_star, dist_star, conn_len, conn_heading, desc in switches:
        color = SEGMENT_COLORS.get(seg["kind"], "#999999")
        ax.plot(*proj_star, "*", ms=16, color=color, markeredgecolor="black",
                markeredgewidth=0.6, zorder=4, label=f"{seg['name']} switch (투영점)")
        ax.plot(*anchor_pt, "o", ms=7, color=color, markeredgecolor="black",
                markeredgewidth=0.6, zorder=4, label=f"{seg['name']} anchor(크랩 쪽 끝점)")
        ax.annotate(f"{seg['name']}\nS_switch={s_star:.1f} m", proj_star,
                    textcoords="offset points", xytext=(8, -10), fontsize=8, color=color)

    ax.set_title("ref_ver1: 레귤러1 -> 크랩2 -> 레귤러3 (세그먼트별 csv, 크랩은 안 잘림)")
    ax.set_xlabel("Easting (m)")
    ax.set_ylabel("Northing (m)")
    ax.axis("equal")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8, loc="best")

    plt.tight_layout()
    out_png = os.path.join(OUTPUT_DIR, "ref_ver1_connected_path.png")
    plt.savefig(out_png, dpi=150)
    print("saved:", out_png)


if __name__ == "__main__":
    main()
