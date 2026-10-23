"""Prepare final meshes for tocatl_sim: decimate, fix pairs, name for the layout."""
from pathlib import Path

import numpy as np
import trimesh
import fast_simplification

SRC = Path("/home/hojung/Desktop/romela_tocatl/TOCATL_CAD/converted_stl")
DST = Path("/home/hojung/Desktop/romela_tocatl/romela-tocatl/src/tocatl_sim/meshes")
DST.mkdir(exist_ok=True)

# name -> (source stem, target faces, double_sided)
PARTS = {
    "body_shell":        ("Body Shell - 1.2", 4000, True),
    "body_sextant":      ("Body Sextant 1.4", 8000, True),
    "body_plate":        ("Body Plate 1.1", 6000, True),
    "body_bottom":       ("Body Bottom", 3000, True),
    "hip_mount":         ("Hip Mount v4", 2000, True),
    "hip_turret":        ("Hip Turret v3", 20000, False),
    "hip_turret_shaft":  ("Hip Turret Shaft", 1500, False),
    "femur_beam":        ("Femur Beam", 8000, True),
    "femur_mount":       ("Femur Mounting Point", 4000, False),
    "bearing_attach_l":  ("Femur Bearing Attachment v3- MIrrored", 8000, False),
    "knee_motor_adapter": ("Knee Motor Adapter", 2000, False),
    "knee_bar":          ("Knee Bar 2.0", 4000, False),
    "knee_hinge":        ("Knee Hinge", 800, False),
    "tibia":             ("Tibia 3.5", 8000, True),
    "tarsus":            ("Tarsus", 5000, True),
    "toe":               ("Toe", 3000, False),
}


def load_dec(stem: str, target: int) -> trimesh.Trimesh:
    m = trimesh.load(SRC / (stem + ".stl"), force="mesh")
    if stem == "Body Sextant 1.4":
        # drop the two broken-tessellation disc artifacts (~288 mm in all axes)
        comps = m.split(only_watertight=False)
        keep = [c for c in comps if not (c.extents > 0.25).all()]
        m = trimesh.util.concatenate(keep)
    if stem.startswith("Femur Bearing Attachment"):
        # keep the drum body + one flange disc; drop dozens of coincident
        # duplicate discs that decimate into spike artifacts
        comps = sorted(m.split(only_watertight=False), key=lambda c: -len(c.faces))
        body = [c for c in comps if c.extents[2] > 0.02]
        disc = [c for c in comps if c.extents[2] <= 0.02][:1]
        m = trimesh.util.concatenate(body[:1] + disc)
    if len(m.faces) > target:
        v, f = fast_simplification.simplify(
            np.asarray(m.vertices, np.float32), np.asarray(m.faces, np.int64),
            target_count=target)
        m = trimesh.Trimesh(v, f, process=False)
    return m


for name, (stem, target, double) in PARTS.items():
    m = load_dec(stem, target)
    if double:
        m = trimesh.Trimesh(m.vertices, np.vstack([m.faces, m.faces[:, ::-1]]),
                            process=False)
    m.export(DST / (name + ".stl"))
    print(f"{name:22s} {len(m.faces):>6d} faces  extents(mm) {np.round(m.extents*1000,1).tolist()}")

# right-side bearing attachment = mirror of the complete left one (y -> -y)
m = load_dec("Femur Bearing Attachment v3- MIrrored", 8000)
mm = trimesh.Trimesh(m.vertices * np.array([1, -1, 1]), m.faces[:, ::-1], process=False)
mm.export(DST / "bearing_attach_r.stl")
print("bearing_attach_r        (mirrored copy)")
