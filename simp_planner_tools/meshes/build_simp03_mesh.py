"""SIMP03 CAD(STEP) -> RViz용 경량 DAE (바퀴 제외, footprint 1x1 정규화).

vehicle_visualizer_node가 그리는 simp03_body.dae를 다시 만드는 스크립트.
CAD가 갱신되면 이 스크립트로 재생성한다 (ROS 환경과 별개인 venv에서 실행):

  pip install cascadio trimesh fast-simplification pycollada numpy scipy networkx
  python build_simp03_mesh.py "SIMP03 Full Assy_SUM_260430.stp" simp03_body.dae

출력 메시 규약 (vehicle_visualizer_node의 CUBE와 동일한 scale 규약):
  - x: 전방(+x), y: 좌측(+y), z: 위
  - x, y는 차량 전체 bbox를 [-0.5, 0.5]로 정규화 -> marker.scale.x/y = vehicle_length/width
  - z는 실제 미터 단위, 지면(타이어 바닥) = 0 -> marker.scale.z = 1.0
"""
import collections
import io
import os
import re
import sys
import tempfile

import cascadio
import collada
import fast_simplification
import numpy as np
import trimesh

STEP, OUT = sys.argv[1], sys.argv[2]

COLORS = {
    "body": (0.93, 0.93, 0.92),
    "dark": (0.20, 0.21, 0.23),
    "alu": (0.66, 0.68, 0.71),
    "glass": (0.80, 0.86, 0.92),
    "lamp": (0.45, 0.70, 0.95),
    "red": (0.85, 0.10, 0.10),
}
# 그룹별 최대 face 수 (위에서 보이는 흰 외피에 예산을 몰아준다)
BUDGET = {"body": 36000, "dark": 18000, "alu": 8000, "glass": 6000, "lamp": 3000, "red": 2000}

# 말단 부품 이름 기준, 위에서부터 먼저 매칭. AllCATPart = 휠 림/허브캡 8개뿐이라 제외.
RULES = [
    ("skip", r"AllCATPart|Wheel|TIre|Tire|Screw|Bolt|Rivet|Nut|Washer|Magnet|Pin_|Snap Ring"
             r"|Plunger|Locator Hole|Shaft_|Antenna"),
    ("red", r"Brake_Red|Emergency"),
    ("lamp", r"Light_Head|LED|Signal"),
    ("glass", r"Clear"),
    ("alu", r"Frame|Connector_\d|Reflector"),
    ("dark", r"Buttom Panel|Back Panel|Case_|Vent|Steering Acuator|Display|Catcher|Receiver"
             r"|selector|Connector_Recharge|Braket|Hinge|^Part\d"),
    ("body", r"Cover_|Bumper_"),
]


def classify(leaf: str, center_z: float) -> str:
    # CAD 인스턴스 이름이 재사용돼서 상판 커버도 'Cover_Buttom_*' 이름을 갖는다 -> 높이로 구분
    if leaf.startswith("Cover_Buttom"):
        return "body" if center_z > 0.0 else "dark"
    for group, pattern in RULES:
        if re.search(pattern, leaf):
            return group
    return "dark"


with tempfile.TemporaryDirectory() as tmp:
    glb = os.path.join(tmp, "simp03.glb")
    # tol_linear/tol_angular: CAD 테셀레이션 허용오차 (mm / rad). 작을수록 촘촘하다.
    cascadio.step_to_glb(STEP, glb, tol_linear=3.0, tol_angular=0.6)
    scene = trimesh.load(glb)
graph = scene.graph
parents = graph.transforms.parents
full_lo, full_hi = scene.bounds  # 바퀴 포함 전체 bbox -> 정규화 기준

parts = collections.defaultdict(list)
raw = collections.Counter()
for node in graph.nodes_geometry:
    transform, geom_name = graph[node]
    mesh = scene.geometry[geom_name]
    if not hasattr(mesh, "faces") or len(mesh.faces) == 0:
        continue
    chain, n = [], node
    while n in parents:
        chain.append(n)
        n = parents[n]
    names = [c for c in chain if not c.startswith("=>")]
    leaf = re.sub(r"(\.\d+)(_\d+)*$", "", names[0])
    vertices = trimesh.transform_points(mesh.vertices, transform)
    group = classify(leaf, 0.5 * (vertices[:, 2].min() + vertices[:, 2].max()))
    raw[group] += len(mesh.faces)
    if group != "skip":
        parts[group].append(trimesh.Trimesh(vertices, mesh.faces.copy(), process=False))

print("raw faces per group:", dict(raw))
length, width = full_hi[0] - full_lo[0], full_hi[1] - full_lo[1]
origin = np.array([0.5 * (full_lo[0] + full_hi[0]), 0.5 * (full_lo[1] + full_hi[1]), full_lo[2]])
print(f"CAD bbox length={length:.4f} width={width:.4f} height={full_hi[2] - full_lo[2]:.4f}")

out_meshes = []
for group in COLORS:
    if group not in parts:
        continue
    merged = trimesh.util.concatenate(parts[group])
    merged.merge_vertices()
    v, f = merged.vertices, merged.faces
    if len(f) > BUDGET[group]:
        v, f = fast_simplification.simplify(v, f, target_reduction=1.0 - BUDGET[group] / len(f))
    v = v - origin
    v[:, :2] *= -1.0  # CAD는 앞범퍼가 -x -> z축 180도 회전으로 ROS(전방 +x, 좌측 +y)
    v[:, 0] /= length
    v[:, 1] /= width
    v[:, :2] = np.clip(v[:, :2], -0.5, 0.5)  # 감축 후 미세 돌출 제거 -> footprint와 정확히 일치
    mesh = trimesh.Trimesh(np.round(v, 5), f, process=True)
    mesh.fix_normals(multibody=True)
    rgba = np.array(COLORS[group] + (1.0,))
    mesh.visual = trimesh.visual.ColorVisuals(mesh, face_colors=(rgba * 255).astype(np.uint8))
    out_meshes.append(mesh)
    print(f"{group:5s} {len(merged.faces):7d} -> {len(mesh.faces):6d}")

allv = np.vstack([m.vertices for m in out_meshes])
print("final faces", sum(len(m.faces) for m in out_meshes),
      "normalized bounds", allv.min(0).round(4).tolist(), allv.max(0).round(4).tolist())

dae = collada.Collada(io.BytesIO(trimesh.exchange.dae.export_collada(out_meshes)))
# trimesh는 diffuse만 적으므로, 그늘진 면도 색이 보이도록 ambient를 넣고 반사는 약하게.
for effect in dae.effects:
    r, g, b = effect.diffuse[:3]
    effect.ambient = (0.6 * r, 0.6 * g, 0.6 * b, 1.0)
    effect.specular = (0.15, 0.15, 0.15, 1.0)
    effect.shininess = 20.0
for geometry in dae.geometries:  # 법선은 소수 3자리면 충분 (파일 크기 절감)
    for source in geometry.sourceById.values():
        if isinstance(source, collada.source.FloatSource) and "normals" in source.id:
            source.data = np.round(source.data, 3)
            source.save()
# trimesh는 무조건 Y_UP으로 적지만 데이터는 Z-up이다. RViz는 assimp 루트 변환(up축 보정)을
# 무시해서 상관없지만, 다른 뷰어(Gazebo, Blender 등)에서도 똑같이 보이도록 사실대로 표기한다.
dae.assetInfo.upaxis = "Z_UP"
dae.assetInfo.unitmeter = 1.0
dae.assetInfo.unitname = "meter"
dae.assetInfo.save()
dae.write(OUT)
