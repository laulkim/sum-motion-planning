"""
HDMap에서 뽑아낸 큰트랙/크랩1 연결 경로를, simp_planner_tools 시나리오 CSV 포맷
(x,y,yaw,kappa,mode, 헤더 정확히 이 5개)으로 변환해 simp_planner_tools/maps/에
저장한다. "전진 -> 크랩(Left)로 이탈 -> 정지 -> 반대 크랩(Right)로 복귀"
3-phase 시나리오(hdmap_crab1_switch)를 만들기 위한 재료다.

기존 crab_switch 시나리오(crab_forward.csv/crab_left.csv)와 같은 로컬 좌표
관례를 따르기 위해, 사용하는 구간만 잘라서 그 시작점을 원점으로 평행이동한다
(REFERENCE_PATH_GUIDE.md 7절의 로컬 map frame 권장사항과 동일한 이유).

만드는 파일 (simp_planner_tools/maps/):
  hdmap_crab1_forward.csv        - 큰트랙에서 크랩1 switch 앞 LEAD_IN_LENGTH(m)
                                    구간 + switch 지점까지, mode=0 (Forward)
  hdmap_crab1_left.csv           - 크랩1 진입 커넥터 + 크랩1 기동, mode=2 (Left)
  hdmap_crab1_right_return.csv   - 위 left 경로를 역순으로 되짚는 복귀 경로,
                                    mode=3 (Right). 역방향이므로 yaw는 +pi,
                                    kappa는 부호 반전(3.2절 "진짜 종점" 논의에서
                                    확인한 것과 같은 이유: 곡률은 진행 방향
                                    기준이라 방향이 바뀌면 부호도 바뀐다)

switch_s(로컬, hdmap_crab1_forward.csv 원점 기준)도 같이 출력한다 --
scenario_definition.py의 ScenarioPhase(switch_s=...)에 그대로 넣을 값이다.
"""
import csv
import math
import os

import numpy as np

DATA_DIR = os.path.dirname(os.path.abspath(__file__))
OUTPUT_MAPS_DIR = os.path.join(DATA_DIR, "..", "simp_planner_tools", "maps")

LEAD_IN_LENGTH = 50.0   # m, switch 지점 앞쪽에서 가져올 큰트랙 구간 길이
SWITCH_MARGIN = 2.0     # m, switch_s를 넘어서 살짝 더 살려두는 여유 (clip 안전)


def load_xy_yaw(path):
    with open(path, encoding="utf-8") as f:
        rows = [(float(r["x"]), float(r["y"]), float(r["yaw"])) for r in csv.DictReader(f)]
    arr = np.asarray(rows, dtype=float)
    return arr[:, :2], arr[:, 2]


def cum_arclen(xy):
    d = np.hypot(np.diff(xy[:, 0]), np.diff(xy[:, 1]))
    return np.concatenate([[0.0], np.cumsum(d)])


def project_point(xy_path, s_path, point):
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


def central_diff_kappa(yaw_unwrapped, s):
    """REFERENCE_PATH_GUIDE.md 8.3절: 중앙차분 + unwrap yaw, 끝점은 단방향차분."""
    n = len(yaw_unwrapped)
    kappa = np.zeros(n)
    for i in range(1, n - 1):
        ds = s[i + 1] - s[i - 1]
        kappa[i] = (yaw_unwrapped[i + 1] - yaw_unwrapped[i - 1]) / ds if ds > 1e-9 else 0.0
    kappa[0] = (yaw_unwrapped[1] - yaw_unwrapped[0]) / max(s[1] - s[0], 1e-9)
    kappa[-1] = (yaw_unwrapped[-1] - yaw_unwrapped[-2]) / max(s[-1] - s[-2], 1e-9)
    return kappa


def wrap_angle(a):
    return np.arctan2(np.sin(a), np.cos(a))


def save_scenario_csv(path, xy, yaw, kappa, mode):
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["x", "y", "yaw", "kappa", "mode"])
        for (x, y), h, k in zip(xy, yaw, kappa):
            w.writerow([f"{x:.3f}", f"{y:.3f}", f"{h:.6f}", f"{k:.6f}", mode])


def main():
    os.makedirs(OUTPUT_MAPS_DIR, exist_ok=True)

    big_xy, big_yaw = load_xy_yaw(os.path.join(DATA_DIR, "output", "큰트랙_connected_ref.csv"))
    big_s = cum_arclen(big_xy)
    crab_xy, crab_yaw = load_xy_yaw(os.path.join(DATA_DIR, "output", "크랩1_connected_ref.csv"))

    s_switch, switch_xy, dist = project_point(big_xy, big_s, crab_xy[0])
    if dist > 0.05:
        raise RuntimeError(
            f"crab1 path start does not lie on the big track (residual {dist:.3f} m); "
            "output/크랩1_connected_ref.csv 가 최신 build_connected_crab_paths.py "
            "결과인지 확인할 것"
        )

    s_start = max(0.0, s_switch - LEAD_IN_LENGTH)
    s_end = min(big_s[-1], s_switch + SWITCH_MARGIN)
    mask = (big_s >= s_start) & (big_s <= s_end)
    fwd_xy = big_xy[mask]
    fwd_yaw = big_yaw[mask]

    origin = fwd_xy[0].copy()
    fwd_xy_local = fwd_xy - origin
    crab_xy_local = crab_xy - origin
    switch_s_local = s_switch - s_start

    fwd_s = cum_arclen(fwd_xy_local)
    fwd_yaw_unwrapped = np.unwrap(fwd_yaw)
    fwd_kappa = central_diff_kappa(fwd_yaw_unwrapped, fwd_s)

    crab_s = cum_arclen(crab_xy_local)
    crab_yaw_unwrapped = np.unwrap(crab_yaw)
    crab_kappa = central_diff_kappa(crab_yaw_unwrapped, crab_s)

    return_xy_local = crab_xy_local[::-1]
    return_yaw = wrap_angle(crab_yaw[::-1] + math.pi)
    return_kappa = -crab_kappa[::-1]

    fwd_path = os.path.join(OUTPUT_MAPS_DIR, "hdmap_crab1_forward.csv")
    left_path = os.path.join(OUTPUT_MAPS_DIR, "hdmap_crab1_left.csv")
    right_path = os.path.join(OUTPUT_MAPS_DIR, "hdmap_crab1_right_return.csv")

    save_scenario_csv(fwd_path, fwd_xy_local, fwd_yaw, fwd_kappa, mode=0)
    save_scenario_csv(left_path, crab_xy_local, crab_yaw, crab_kappa, mode=2)
    save_scenario_csv(right_path, return_xy_local, return_yaw, return_kappa, mode=3)

    print(f"origin (UTM) = {origin[0]:.3f}, {origin[1]:.3f}  "
          f"(= hdmap_crab1_forward.csv 시작점 = 큰트랙 s={s_start:.2f} m)")
    print(f"switch_s (local, hdmap_crab1_forward.csv 기준) = {switch_s_local:.2f} m")
    print(f"saved: {fwd_path}  ({len(fwd_xy_local)}pt, max|kappa|={np.max(np.abs(fwd_kappa)):.4f} 1/m)")
    print(f"saved: {left_path}  ({len(crab_xy_local)}pt, max|kappa|={np.max(np.abs(crab_kappa)):.4f} 1/m)")
    print(f"saved: {right_path}  ({len(return_xy_local)}pt, max|kappa|={np.max(np.abs(return_kappa)):.4f} 1/m)")

    origin_path = os.path.join(DATA_DIR, "output", "hdmap_crab1_switch_origin.txt")
    with open(origin_path, "w", encoding="utf-8") as f:
        f.write(f"{origin[0]:.3f} {origin[1]:.3f}\n")
    print(f"saved: {origin_path}  (origin_easting={origin[0]:.3f}, origin_northing={origin[1]:.3f})")


if __name__ == "__main__":
    main()
