"""
큰트랙 전체 구간 + 크랩1/2/3을 하나로 잇는 "연속 랩" 시나리오용 CSV를 만든다.

순서: 큰트랙 시작(s=0) -> [전진] -> 크랩1 switch -> [크랩1 이탈/복귀] ->
      [전진] -> 크랩2 switch -> [크랩2 이탈/복귀] ->
      [전진] -> 크랩3 switch -> [크랩3 이탈/복귀] ->
      [전진] 크랩3 switch ~ 큰트랙 끝(s_max) (마지막 phase, switch 없음)

build_scenario_maps.py(크랩1 단독 시나리오)와 다른 점: 모든 phase를 큰트랙
자신의 시작점(큰트랙_connected_ref.csv[0]) 하나를 공통 원점으로 평행이동한다.
그래서 전진 구간들이 크랩 switch 지점에서 서로 정확히 이어지고, 크랩 경로도
큰트랙과 같은 프레임에서 이미 붙어 있는 채로 만들어진다 (런타임 translate는
정지 위치 오차 보정용 안전장치로만 필요).

만드는 파일 (simp_planner_tools/maps/):
  hdmap_lap_forward0.csv              - 큰트랙 s=0 ~ 크랩1 switch, mode=0
  hdmap_lap_crab1_left.csv            - 크랩1 이탈, mode=2
  hdmap_lap_crab1_right_return.csv    - 크랩1 복귀 (역순, yaw+pi, kappa 부호반전), mode=3
  hdmap_lap_forward1.csv              - 크랩1 switch ~ 크랩2 switch, mode=0
  hdmap_lap_crab2_left.csv / _right_return.csv                      mode=2 / 3
  hdmap_lap_forward2.csv              - 크랩2 switch ~ 크랩3 switch, mode=0
  hdmap_lap_crab3_left.csv / _right_return.csv                      mode=2 / 3
  hdmap_lap_forward3.csv              - 크랩3 switch ~ 큰트랙 끝, mode=0 (마지막 phase)

각 forward 구간의 switch_s(자기 로컬 원점 기준)도 콘솔에 출력한다 --
scenario_definition.py의 ScenarioPhase(switch_s=...)에 그대로 쓸 값이다.
"""
import csv
import math
import os

import numpy as np

DATA_DIR = os.path.dirname(os.path.abspath(__file__))
OUTPUT_MAPS_DIR = os.path.join(DATA_DIR, "..", "simp_planner_tools", "maps")

CRAB_NAMES = ["크랩1", "크랩2", "크랩3"]
SWITCH_MARGIN = 2.0  # m, switch_s를 넘어 살짝 더 살려두는 여유 (clip 안전)


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


def build_forward_segment(big_xy, big_yaw, big_s, s_from, s_switch, s_to, origin):
    """[s_from, s_to] 구간을 자르되, switch_s는 실제 switch 지점(s_switch)까지의
    거리로 계산한다 -- s_to는 그 뒤로 SWITCH_MARGIN만큼 더 살려두는 여유일 뿐,
    거기서 멈추라는 뜻이 아니다."""
    mask = (big_s >= s_from) & (big_s <= s_to)
    xy = big_xy[mask] - origin
    yaw = big_yaw[mask]
    s = cum_arclen(xy)
    kappa = central_diff_kappa(np.unwrap(yaw), s)
    switch_s_local = s_switch - s_from
    return xy, yaw, kappa, switch_s_local


def build_crab_pair(crab_xy, crab_yaw, origin):
    xy = crab_xy - origin
    s = cum_arclen(xy)
    kappa = central_diff_kappa(np.unwrap(crab_yaw), s)
    return_xy = xy[::-1]
    return_yaw = wrap_angle(crab_yaw[::-1] + math.pi)
    return_kappa = -kappa[::-1]
    return (xy, crab_yaw, kappa), (return_xy, return_yaw, return_kappa)


def main():
    os.makedirs(OUTPUT_MAPS_DIR, exist_ok=True)

    big_xy, big_yaw = load_xy_yaw(os.path.join(DATA_DIR, "output", "큰트랙_connected_ref.csv"))
    big_s = cum_arclen(big_xy)
    origin = big_xy[0].copy()

    switches = []
    for name in CRAB_NAMES:
        crab_xy, crab_yaw = load_xy_yaw(os.path.join(DATA_DIR, "output", f"{name}_connected_ref.csv"))
        s_switch, _, dist = project_point(big_xy, big_s, crab_xy[0])
        if dist > 0.05:
            raise RuntimeError(f"{name} path start does not lie on the big track (residual {dist:.3f} m)")
        switches.append((name, s_switch, crab_xy, crab_yaw))
    switches.sort(key=lambda t: t[1])  # 큰트랙 진행 순서로 정렬

    bounds = [0.0] + [s for _, s, _, _ in switches]
    print(f"switch order (큰트랙 진행순): {[n for n, _, _, _ in switches]}")

    for i, (name, s_switch, crab_xy, crab_yaw) in enumerate(switches):
        s_from = bounds[i]
        s_to = min(big_s[-1], s_switch + SWITCH_MARGIN)
        fwd_xy, fwd_yaw, fwd_kappa, switch_s_local = build_forward_segment(
            big_xy, big_yaw, big_s, s_from, s_switch, s_to, origin
        )
        fwd_path = os.path.join(OUTPUT_MAPS_DIR, f"hdmap_lap_forward{i}.csv")
        save_scenario_csv(fwd_path, fwd_xy, fwd_yaw, fwd_kappa, mode=0)
        print(f"saved: {fwd_path}  ({len(fwd_xy)}pt, s=[{s_from:.2f},{s_to:.2f}]m, "
              f"switch_s={switch_s_local:.2f}m, max|kappa|={np.max(np.abs(fwd_kappa)):.4f} 1/m)")

        (left_xy, left_yaw, left_kappa), (right_xy, right_yaw, right_kappa) = build_crab_pair(
            crab_xy, crab_yaw, origin
        )
        crab_no = name.replace("크랩", "")
        left_path = os.path.join(OUTPUT_MAPS_DIR, f"hdmap_lap_crab{crab_no}_left.csv")
        right_path = os.path.join(OUTPUT_MAPS_DIR, f"hdmap_lap_crab{crab_no}_right_return.csv")
        save_scenario_csv(left_path, left_xy, left_yaw, left_kappa, mode=2)
        save_scenario_csv(right_path, right_xy, right_yaw, right_kappa, mode=3)
        print(f"saved: {left_path}  ({len(left_xy)}pt, max|kappa|={np.max(np.abs(left_kappa)):.4f} 1/m)")
        print(f"saved: {right_path}  ({len(right_xy)}pt, max|kappa|={np.max(np.abs(right_kappa)):.4f} 1/m)")

    # 마지막 크랩(크랩3) switch 이후 큰트랙의 남은 구간. switch가 더 없으므로
    # 이 phase는 switch_s 없이 자기 끝(terminal_margin 기준)에서 종료된다.
    s_from = bounds[-1]
    s_to = big_s[-1]
    tail_mask = (big_s >= s_from) & (big_s <= s_to)
    tail_xy = big_xy[tail_mask] - origin
    tail_yaw = big_yaw[tail_mask]
    tail_s = cum_arclen(tail_xy)
    tail_kappa = central_diff_kappa(np.unwrap(tail_yaw), tail_s)
    tail_index = len(switches)
    tail_path = os.path.join(OUTPUT_MAPS_DIR, f"hdmap_lap_forward{tail_index}.csv")
    save_scenario_csv(tail_path, tail_xy, tail_yaw, tail_kappa, mode=0)
    print(f"saved: {tail_path}  ({len(tail_xy)}pt, s=[{s_from:.2f},{s_to:.2f}]m, "
          f"(마지막 phase, switch_s 없음), max|kappa|={np.max(np.abs(tail_kappa)):.4f} 1/m)")

    # 이 시나리오가 쓰는 로컬 원점(UTM easting/northing) 기록 -- RViz에서 원본
    # HD map(차선 등)을 같은 "odom" 프레임에 겹쳐 그리려는 노드가 읽어간다.
    origin_path = os.path.join(DATA_DIR, "output", "hdmap_lap_switch_origin.txt")
    with open(origin_path, "w", encoding="utf-8") as f:
        f.write(f"{origin[0]:.3f} {origin[1]:.3f}\n")
    print(f"saved: {origin_path}  (origin_easting={origin[0]:.3f}, origin_northing={origin[1]:.3f})")


if __name__ == "__main__":
    main()
