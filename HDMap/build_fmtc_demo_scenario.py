"""
ref_ver1.txt 기반 레귤러1/크랩2/레귤러3 경로(HDMap/build_ref_ver1_path.py 출력,
output/{name}_connected_ref.csv)를 simp_planner_tools 시나리오 CSV 포맷
(x,y,yaw,kappa,mode, 헤더 정확히 이 5개)으로 변환해 simp_planner_tools/maps/에
저장한다. "fmtc_demo" 3-phase 시나리오(레귤러1 Forward -> 크랩2 Left -> 레귤러3
Forward)를 만들기 위한 재료다 (등록은 scenario_definition.py에 별도로 함).

로컬 좌표 관례(REFERENCE_PATH_GUIDE.md 7절, build_scenario_maps.py와 동일)를
따르기 위해 세 구간 모두 레귤러1의 시작점을 공통 원점으로 평행이동한다.

switch_s는 각 phase 자신의 로컬 CSV 총 길이다: build_ref_ver1_path.py가 이미
S_switch 지점에서 정확히 잘라 놨으므로(레귤러1->크랩2, 크랩2->레귤러3 둘 다),
그 CSV의 끝이 곧 전환 지점과 일치한다.

생성 파일:
  simp_planner_tools/maps/fmtc_demo_regular1.csv  - 레귤러1 (S_switch에서 꼬리 잘림), mode=0 Forward
  simp_planner_tools/maps/fmtc_demo_crab2.csv     - 커넥터 + 크랩2 (S_switch에서 꼬리 잘림), mode=2 Left
  simp_planner_tools/maps/fmtc_demo_regular3.csv  - 커넥터 + 레귤러3 (통째로), mode=0 Forward
  HDMap/output/fmtc_demo_origin.txt               - 로컬 원점(UTM). hdmap_lane_visualizer_node가
                                                     이 파일을 읽어 원본 차선 shapefile을 같은
                                                     로컬 프레임으로 겹쳐 그린다 (simulation.launch.py가
                                                     scenario 이름으로 파일명을 자동 조립하므로
                                                     `{scenario_name}_origin.txt` 규칙을 반드시 따라야 함).
"""
import csv
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from build_scenario_maps import central_diff_kappa, cum_arclen  # noqa: E402

DATA_DIR = os.path.dirname(os.path.abspath(__file__))
OUTPUT_DIR = os.path.join(DATA_DIR, "output")
MAPS_DIR = os.path.join(DATA_DIR, "..", "simp_planner_tools", "maps")


def load_xy_yaw(path):
    with open(path, encoding="utf-8") as f:
        rows = [(float(r["x"]), float(r["y"]), float(r["yaw"])) for r in csv.DictReader(f)]
    arr = np.asarray(rows, dtype=float)
    return arr[:, :2], arr[:, 2]


def save_scenario_csv(path, xy, yaw, kappa, mode):
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["x", "y", "yaw", "kappa", "mode"])
        for (x, y), h, k in zip(xy, yaw, kappa):
            w.writerow([f"{x:.3f}", f"{y:.3f}", f"{h:.6f}", f"{k:.6f}", mode])


def build_phase(name, xy, yaw, mode, origin):
    xy_local = xy - origin
    s = cum_arclen(xy_local)
    kappa = central_diff_kappa(np.unwrap(yaw), s)
    out_path = os.path.join(MAPS_DIR, f"{name}.csv")
    save_scenario_csv(out_path, xy_local, yaw, kappa, mode)
    total_length = float(s[-1])
    print(f"saved: {out_path}  ({len(xy_local)}pt, length={total_length:.2f} m, "
          f"max|kappa|={np.max(np.abs(kappa)):.4f} 1/m)")
    return total_length


def main():
    os.makedirs(MAPS_DIR, exist_ok=True)

    reg1_xy, reg1_yaw = load_xy_yaw(os.path.join(OUTPUT_DIR, "레귤러1_connected_ref.csv"))
    crab2_xy, crab2_yaw = load_xy_yaw(os.path.join(OUTPUT_DIR, "크랩2_connected_ref.csv"))
    reg3_xy, reg3_yaw = load_xy_yaw(os.path.join(OUTPUT_DIR, "레귤러3_connected_ref.csv"))

    origin = reg1_xy[0].copy()

    len1 = build_phase("fmtc_demo_regular1", reg1_xy, reg1_yaw, mode=0, origin=origin)
    len2 = build_phase("fmtc_demo_crab2", crab2_xy, crab2_yaw, mode=2, origin=origin)
    len3 = build_phase("fmtc_demo_regular3", reg3_xy, reg3_yaw, mode=0, origin=origin)

    print(f"\norigin (UTM) = {origin[0]:.3f}, {origin[1]:.3f}  (= 레귤러1 시작점)")
    print(f"switch_s (regular1 -> crab2) = {len1:.2f} m  (fmtc_demo_regular1.csv 자신의 총 길이)")
    print(f"switch_s (crab2 -> regular3) = {len2:.2f} m  (fmtc_demo_crab2.csv 자신의 총 길이)")
    print(f"regular3 총 길이 = {len3:.2f} m (마지막 phase, switch_s 없음)")

    origin_path = os.path.join(OUTPUT_DIR, "fmtc_demo_origin.txt")
    with open(origin_path, "w", encoding="utf-8") as f:
        f.write(f"{origin[0]:.3f} {origin[1]:.3f}\n")
    print(f"saved: {origin_path}  (hdmap_lane_visualizer_node용 로컬 원점)")


if __name__ == "__main__":
    main()
