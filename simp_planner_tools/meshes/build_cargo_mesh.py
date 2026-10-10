"""박스형 화물 차량(섀시 + 둥근 모서리 화물칸 + 측면 양문) RViz용 DAE를 만든다.

vehicle_visualizer_node 기본 메시(cargo_body.dae)를 다시 만드는 스크립트. 바퀴는 넣지 않는다.

  pip install trimesh pycollada numpy
  python build_cargo_mesh.py cargo_body.dae

출력 메시 규약 (build_simp03_mesh.py와 같다):
  - x: 전방(+x), y: 좌측(+y), z: 위
  - x, y는 기준 차량 3.0 x 2.0 m를 [-0.5, 0.5]로 정규화 -> marker.scale.x/y = vehicle_length/width
  - z는 실제 미터 단위, 지면 = 0 -> marker.scale.z = 1.0
양문은 오른쪽(-y) 측면, 앞면 아래쪽에 작은 점검 패널이 있다.
"""
import io
import sys

import collada
import numpy as np
import trimesh

OUT = sys.argv[1]
NOMINAL_LENGTH, NOMINAL_WIDTH = 3.0, 2.0

COLORS = {
    "body": (0.93, 0.94, 0.95),
    "frame": (0.62, 0.65, 0.69),
    "door": (0.85, 0.87, 0.90),
    "dark": (0.20, 0.21, 0.23),
    "chassis": (0.27, 0.28, 0.30),
    "lamp_front": (0.60, 0.82, 1.00),
    "lamp_rear": (0.85, 0.10, 0.10),
}
parts: dict[str, list[trimesh.Trimesh]] = {name: [] for name in COLORS}


def box(group, x0, x1, y0, y1, z0, z1):
    mesh = trimesh.creation.box(extents=(x1 - x0, y1 - y0, z1 - z0))
    mesh.apply_translation((0.5 * (x0 + x1), 0.5 * (y0 + y1), 0.5 * (z0 + z1)))
    parts[group].append(mesh)


def rounded_prism(group, half_x, half_y, radius, z0, z1, segments=8):
    """모서리가 반경 radius로 둥근 직사각형 기둥 (볼록이라 위/아래는 부채꼴 분할)."""
    outline = []
    for cx, cy, start in ((half_x - radius, half_y - radius, 0.0),
                          (-half_x + radius, half_y - radius, 0.5 * np.pi),
                          (-half_x + radius, -half_y + radius, np.pi),
                          (half_x - radius, -half_y + radius, 1.5 * np.pi)):
        for t in np.linspace(start, start + 0.5 * np.pi, segments + 1):
            outline.append((cx + radius * np.cos(t), cy + radius * np.sin(t)))
    count = len(outline)
    ring = np.array(outline)
    vertices = np.vstack([np.c_[ring, np.full(count, z0)], np.c_[ring, np.full(count, z1)],
                          [[0.0, 0.0, z0], [0.0, 0.0, z1]]])
    bottom_center, top_center = 2 * count, 2 * count + 1
    faces = []
    for i in range(count):
        j = (i + 1) % count
        faces += [(i, j, count + j), (i, count + j, count + i),
                  (bottom_center, j, i), (top_center, count + i, count + j)]
    mesh = trimesh.Trimesh(vertices, faces, process=True)
    mesh.fix_normals()
    parts[group].append(mesh)


# 섀시와 범퍼
box("chassis", -1.40, 1.40, -0.90, 0.90, 0.12, 0.40)
box("dark", 1.40, 1.49, -0.85, 0.85, 0.15, 0.37)
box("dark", -1.49, -1.40, -0.85, 0.85, 0.15, 0.37)
for side in (1.0, -1.0):  # 앞 흰·파랑 등, 뒤 빨강 등 (범퍼 면)
    box("lamp_front", 1.49, 1.50, *sorted((side * 0.55, side * 0.80)), 0.27, 0.33)
    box("lamp_rear", -1.50, -1.49, *sorted((side * 0.55, side * 0.80)), 0.27, 0.33)

# 화물칸: 둥근 모서리 프레임 위에 흰 면판을 덧대 모서리 기둥과 위/아래 띠가 프레임처럼 보인다
half_x, half_y, body_z0, body_z1 = 1.40, 0.95, 0.40, 2.20
rounded_prism("frame", half_x, half_y, 0.12, body_z0, body_z1)
panel_z0, panel_z1, skin = body_z0 + 0.08, body_z1 - 0.08, 0.008
for side in (1.0, -1.0):
    box("body", -half_x + 0.14, half_x - 0.14,
        *sorted((side * half_y, side * (half_y + skin))), panel_z0, panel_z1)
    box("body", *sorted((side * half_x, side * (half_x + skin))),
        -half_y + 0.14, half_y - 0.14, panel_z0, panel_z1)

# 지붕: 3분할 지붕판 + 이음선
rounded_prism("body", half_x - 0.02, half_y - 0.02, 0.11, body_z1, body_z1 + 0.04)
for x in (-0.47, 0.47):
    box("frame", x - 0.015, x + 0.015, -half_y + 0.05, half_y - 0.05, body_z1 + 0.04, body_z1 + 0.05)

# 오른쪽(-y) 측면 양문: 문틀, 문짝 2개, 가운데 이음선, 손잡이
door_y = -(half_y + skin)
door_x, door_z0, door_z1 = 0.74, 0.56, 1.98
frame_width = 0.03
box("dark", -door_x - frame_width, door_x + frame_width, door_y - 0.006, door_y - 0.002,
    door_z0 - frame_width, door_z1 + frame_width)
for x0, x1 in ((-door_x, -0.012), (0.012, door_x)):
    box("door", x0, x1, door_y - 0.010, door_y - 0.004, door_z0, door_z1)
for x in (-0.08, 0.08):
    box("dark", x - 0.02, x + 0.02, door_y - 0.030, door_y - 0.008, 1.12, 1.38)

# 앞면 아래쪽 점검(PC) 패널
front_x = half_x + skin
box("dark", front_x + 0.002, front_x + 0.008, 0.18, 0.62, 0.70, 1.20)
box("frame", front_x + 0.008, front_x + 0.016, 0.24, 0.32, 1.05, 1.12)

out_meshes = []
for name, meshes in parts.items():
    mesh = trimesh.util.concatenate(meshes)
    v = mesh.vertices.copy()
    v[:, 0] /= NOMINAL_LENGTH
    v[:, 1] /= NOMINAL_WIDTH
    mesh = trimesh.Trimesh(np.round(v, 5), mesh.faces, process=True)
    mesh.fix_normals(multibody=True)
    rgba = np.array(COLORS[name] + (1.0,))
    mesh.visual = trimesh.visual.ColorVisuals(mesh, face_colors=(rgba * 255).astype(np.uint8))
    out_meshes.append(mesh)

allv = np.vstack([m.vertices for m in out_meshes])
assert np.all(np.abs(allv[:, :2]) <= 0.5 + 1e-9), "mesh must stay inside the code footprint"
print("faces", sum(len(m.faces) for m in out_meshes),
      "bounds", allv.min(0).round(4).tolist(), allv.max(0).round(4).tolist())

dae = collada.Collada(io.BytesIO(trimesh.exchange.dae.export_collada(out_meshes)))
# trimesh는 diffuse만 적으므로, 그늘진 면도 색이 보이도록 ambient를 넣고 반사는 약하게.
for effect in dae.effects:
    r, g, b = effect.diffuse[:3]
    effect.ambient = (0.6 * r, 0.6 * g, 0.6 * b, 1.0)
    effect.specular = (0.15, 0.15, 0.15, 1.0)
    effect.shininess = 20.0
dae.assetInfo.upaxis = "Z_UP"  # 실제 축(Z-up) 표기. Y_UP이면 RViz(assimp)가 90도 눕혀서 읽는다.
dae.assetInfo.unitmeter = 1.0
dae.assetInfo.unitname = "meter"
dae.assetInfo.save()
dae.write(OUT)
