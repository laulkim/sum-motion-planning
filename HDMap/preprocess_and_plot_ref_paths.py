"""
레퍼런스 경로 전처리(3차 스플라인 피팅 + 균일 간격 재샘플링) + 검증 플롯.

입력:  {트랙명}_ref.csv   (x,y,yaw)  - ref.txt 순서대로 이어붙인 원본(raw) 정점
출력:  {트랙명}_ref_smooth.csv (x,y,yaw) - 스플라인 피팅 후 RESAMPLE_SPACING 간격으로 재샘플링
       ref_path_preprocessing_check.png  - 트랙별 [raw vs smooth 비교] / [원본 폴리라인 대비 편차(오차)] 플롯

이 스크립트가 있는 폴더를 기준으로 raw csv를 찾으므로, 어디서 실행해도 상관없습니다.
"""
import csv
import math
import os

import numpy as np
from scipy.interpolate import splev, splprep

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
TRACKS = ["큰트랙", "크랩1", "크랩2", "크랩3"]

RESAMPLE_SPACING = 0.2   # m, 재샘플링 간격 - 필요에 맞게 조정
SPLINE_DEGREE = 3        # 3차 스플라인
SPLINE_SMOOTH = 0.0      # 0 = 원본 정점을 정확히 지나가는 보간 스플라인.
                          # 원본 정점 밀도가 아주 촘촘한 구간에서 진동이 보이면 0.01~0.1 정도로 올릴 것.
MIN_POINT_GAP = 1e-3     # m, 이보다 가까운 연속 정점은 스플라인 파라미터화 특이점 방지를 위해 병합


def load_raw(name):
    path = os.path.join(DATA_DIR, f"{name}_ref.csv")
    with open(path, encoding="utf-8") as f:
        r = csv.DictReader(f)
        pts = [(float(row["x"]), float(row["y"])) for row in r]
    return dedupe(pts)


def dedupe(pts):
    out = [pts[0]]
    for p in pts[1:]:
        if math.hypot(p[0] - out[-1][0], p[1] - out[-1][1]) > MIN_POINT_GAP:
            out.append(p)
    return out


def cum_arclen(pts):
    s = [0.0]
    for i in range(1, len(pts)):
        s.append(s[-1] + math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1]))
    return s


def point_to_segment_dist(p, a, b):
    ax, ay = a
    bx, by = b
    px, py = p
    dx, dy = bx - ax, by - ay
    l2 = dx * dx + dy * dy
    if l2 < 1e-12:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / l2))
    cx, cy = ax + t * dx, ay + t * dy
    return math.hypot(px - cx, py - cy)


def dist_to_polyline(p, pts):
    return min(point_to_segment_dist(p, pts[i], pts[i + 1]) for i in range(len(pts) - 1))


def fit_and_resample(raw_pts):
    x = np.array([p[0] for p in raw_pts])
    y = np.array([p[1] for p in raw_pts])
    tck, _ = splprep([x, y], s=SPLINE_SMOOTH, k=SPLINE_DEGREE)

    # 스플라인 자체의 호 길이를 촘촘히 샘플링해서 근사
    u_dense = np.linspace(0.0, 1.0, 20 * len(raw_pts))
    xd, yd = splev(u_dense, tck)
    seg = np.hypot(np.diff(xd), np.diff(yd))
    s_dense = np.concatenate([[0.0], np.cumsum(seg)])
    total_len = s_dense[-1]

    n_samples = max(2, int(round(total_len / RESAMPLE_SPACING)) + 1)
    s_target = np.linspace(0.0, total_len, n_samples)
    u_target = np.interp(s_target, s_dense, u_dense)

    xs, ys = splev(u_target, tck)
    dxs, dys = splev(u_target, tck, der=1)
    yaws = np.arctan2(dys, dxs)
    return list(zip(xs.tolist(), ys.tolist())), yaws.tolist()


def save_csv(path, pts, yaws):
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["x", "y", "yaw"])
        for (x, y), yaw in zip(pts, yaws):
            w.writerow([f"{x:.3f}", f"{y:.3f}", f"{yaw:.6f}"])


def main():
    fig, axes = plt.subplots(len(TRACKS), 2, figsize=(12, 4 * len(TRACKS)))

    for row, name in enumerate(TRACKS):
        raw = load_raw(name)
        smooth_pts, smooth_yaws = fit_and_resample(raw)
        save_csv(os.path.join(DATA_DIR, f"{name}_ref_smooth.csv"), smooth_pts, smooth_yaws)

        errs = [dist_to_polyline(p, raw) for p in smooth_pts]
        s_smooth = cum_arclen(smooth_pts)

        ax1 = axes[row, 0]
        rx, ry = [p[0] for p in raw], [p[1] for p in raw]
        sx, sy = [p[0] for p in smooth_pts], [p[1] for p in smooth_pts]
        ax1.plot(rx, ry, "o", ms=3, color="#999999", label=f"raw ({len(raw)}pt)")
        ax1.plot(sx, sy, "-", lw=1.2, color="#2a78d6", label=f"smooth ({len(smooth_pts)}pt, {RESAMPLE_SPACING}m)")
        ax1.set_title(f"{name} - raw vs preprocessed")
        ax1.set_xlabel("x (m)")
        ax1.set_ylabel("y (m)")
        ax1.axis("equal")
        ax1.legend(fontsize=8)

        ax2 = axes[row, 1]
        ax2.plot(s_smooth, [e * 100 for e in errs], "-", lw=1, color="#d95926")
        ax2.axhline(0, color="#cccccc", lw=0.5)
        ax2.set_title(f"{name} - deviation from raw polyline")
        ax2.set_xlabel("arc length (m)")
        ax2.set_ylabel("error (cm)")

        print(
            f"{name}: raw {len(raw)}pt -> smooth {len(smooth_pts)}pt "
            f"(spacing {RESAMPLE_SPACING}m) | max_err={max(errs)*100:.2f}cm mean_err={sum(errs)/len(errs)*100:.2f}cm"
        )

    plt.tight_layout()
    out_png = os.path.join(DATA_DIR, "ref_path_preprocessing_check.png")
    plt.savefig(out_png, dpi=150)
    print("saved:", out_png)


if __name__ == "__main__":
    main()
