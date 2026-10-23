"""Decimate + double-sided 4-view render of every converted part."""
import sys
from pathlib import Path

import numpy as np
import trimesh
import fast_simplification
import mujoco
import cv2

SRC = Path("/home/hojung/Desktop/romela_tocatl/TOCATL_CAD/converted_stl")
WORK = Path("/tmp/cad_convert/work")   # decimated, double-sided copies for viewing
VIEWS = Path("/tmp/cad_convert/views2")
WORK.mkdir(exist_ok=True)
VIEWS.mkdir(exist_ok=True)


def lookat(pos, target):
    f = np.array(target) - np.array(pos); f = f / np.linalg.norm(f)
    up = np.array([0, 0, 1.0])
    if abs(f @ up) > 0.98:
        up = np.array([0, 1.0, 0])
    rt = np.cross(f, up); rt /= np.linalg.norm(rt)
    u2 = np.cross(rt, f)
    return " ".join(f"{v:.6f}" for v in np.concatenate([rt, u2]))


def process(src: Path):
    m = trimesh.load(src, force="mesh")
    if len(m.faces) == 0:
        print(f"{src.name}: EMPTY")
        return
    n0 = len(m.faces)
    if n0 > 15000:
        v, f = fast_simplification.simplify(
            np.asarray(m.vertices, np.float32), np.asarray(m.faces, np.int64),
            target_count=15000)
        m = trimesh.Trimesh(v, f, process=False)
    # double-sided: append flipped faces so thin shells render from both sides
    ds = trimesh.Trimesh(m.vertices, np.vstack([m.faces, m.faces[:, ::-1]]),
                         process=False)
    stem = src.stem
    tmp = WORK / (stem + ".stl")
    ds.export(tmp)

    c = m.bounds.mean(axis=0)
    r = float(np.linalg.norm(m.extents)) / 2 + 1e-9
    views = {
        "iso":   c + np.array([1.2, -1.2, 0.9]) * r * 1.35,
        "front": c + np.array([2.3, 0.01, 0.01]) * r,
        "side":  c + np.array([0.01, 2.3, 0.01]) * r,
        "top":   c + np.array([0.01, 0.01, 2.3]) * r,
    }
    cams = "\n".join(
        f'<camera name="{n}" mode="fixed" pos="{p[0]:.5f} {p[1]:.5f} {p[2]:.5f}" '
        f'xyaxes="{lookat(p, c)}"/>' for n, p in views.items())
    ax = r * 1.2
    xml = f"""<mujoco>
      <visual><global offwidth=\"560\" offheight=\"560\"/>
        <headlight ambient=\"0.45 0.45 0.45\" diffuse=\"0.6 0.6 0.6\"/></visual>
      <asset><mesh name=\"part\" file=\"{tmp}\"/></asset>
      <worldbody>
        <light pos=\"{c[0]+r} {c[1]-r} {c[2]+2*r}\" dir=\"-0.3 0.3 -0.8\"/>
        {cams}
        <geom type=\"mesh\" mesh=\"part\" rgba=\"0.72 0.72 0.75 1\"/>
        <geom type=\"capsule\" fromto=\"0 0 0 {ax} 0 0\" size=\"{r*0.012}\" rgba=\"1 0 0 1\"/>
        <geom type=\"capsule\" fromto=\"0 0 0 0 {ax} 0\" size=\"{r*0.012}\" rgba=\"0 1 0 1\"/>
        <geom type=\"capsule\" fromto=\"0 0 0 0 0 {ax}\" size=\"{r*0.012}\" rgba=\"0 0 1 1\"/>
      </worldbody>
    </mujoco>"""
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    ren = mujoco.Renderer(model, 560, 560)
    tiles = []
    for n in views:
        ren.update_scene(data, camera=n)
        img = cv2.putText(ren.render().copy(), n, (8, 26),
                          cv2.FONT_HERSHEY_SIMPLEX, 0.8, (10, 10, 10), 2)
        tiles.append(img)
    ren.close()
    sheet = np.vstack([np.hstack(tiles[:2]), np.hstack(tiles[2:])])
    cv2.imwrite(str(VIEWS / (stem + ".png")), cv2.cvtColor(sheet, cv2.COLOR_RGB2BGR))
    print(f"{stem}: {n0} faces, extents(mm) {np.round(m.extents*1000,1).tolist()}, "
          f"bounds min {np.round(m.bounds[0]*1000,1).tolist()} max {np.round(m.bounds[1]*1000,1).tolist()}")


targets = sys.argv[1:] or sorted(p.stem for p in SRC.glob("*.stl"))
for t in targets:
    try:
        process(SRC / (t + ".stl"))
    except Exception as e:
        print(f"{t}: ERROR {type(e).__name__} {e}")
