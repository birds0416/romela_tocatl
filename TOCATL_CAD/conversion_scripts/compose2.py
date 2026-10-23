"""Isolation render: highlight selected CAD parts in red on the skeleton.

usage: compose2.py out.png [mesh_name ...]   (no names = all, natural colors)
"""
import sys
from pathlib import Path

import numpy as np
import mujoco
import cv2

for p in ("tocatl_sim", "tocatl_kinematics", "tocatl_gait", "tocatl_diagnostics"):
    sys.path.insert(0, f"/home/hojung/Desktop/romela_tocatl/romela-tocatl/src/{p}")

import importlib
import tocatl_sim.cad_layout as cl
import tocatl_sim.model_builder as mb
importlib.reload(cl)
importlib.reload(mb)

MESH_DIR = "/home/hojung/Desktop/romela_tocatl/romela-tocatl/src/tocatl_sim/meshes"
out = sys.argv[1]
only = set(sys.argv[2:])

if only:
    import dataclasses
    layout = [dataclasses.replace(s, rgba=(0.9, 0.15, 0.1, 1.0))
              for s in cl.MESH_LAYOUT if s.mesh in only]
    mb.MESH_LAYOUT = layout
else:
    mb.MESH_LAYOUT = cl.MESH_LAYOUT

xml = mb.build_mjcf(mb.ModelParams(mesh_dir=MESH_DIR))
# extra diagnostic cameras: close-up of leg 1 (mount angle 30 deg)
import math
a = math.radians(30)
hx, hy = 0.35 * math.cos(a), 0.35 * math.sin(a)
extra = f"""
    <camera name="leg1" mode="fixed" pos="{hx + 0.75*math.cos(a-1.2):.3f} {hy + 0.75*math.sin(a-1.2):.3f} 0.42"
            xyaxes="{-math.sin(a-1.2):.4f} {math.cos(a-1.2):.4f} 0 {-math.cos(a-1.2)*0.25:.4f} {-math.sin(a-1.2)*0.25:.4f} 0.97"/>
    <camera name="top" mode="fixed" pos="0 0 2.2" xyaxes="1 0 0 0 1 0"/>
"""
xml = xml.replace('<camera name="track34"', extra + '    <camera name="track34"')

model = mujoco.MjModel.from_xml_string(xml)
data = mujoco.MjData(model)
mujoco.mj_resetDataKeyframe(model, data, 0)
mujoco.mj_forward(model, data)
ren = mujoco.Renderer(model, 640, 640)
tiles = []
for cam in ("track34", "leg1", "front", "top"):
    ren.update_scene(data, camera=cam)
    img = cv2.putText(ren.render().copy(), cam, (8, 26),
                      cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)
    tiles.append(img)
sheet = np.vstack([np.hstack(tiles[:2]), np.hstack(tiles[2:])])
cv2.imwrite(out, cv2.cvtColor(sheet, cv2.COLOR_RGB2BGR))
print("wrote", out)
