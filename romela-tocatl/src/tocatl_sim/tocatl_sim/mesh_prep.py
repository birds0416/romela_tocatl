"""Prepare CAD meshes for the MuJoCo model: decimate + re-export binary STL.

Input: STLs converted from the TOCATL SolidWorks CAD (metres, part-local
frames). Output: compact binary STLs under tocatl_sim/meshes/ suitable as
MuJoCo visual geoms (a few thousand faces each).

    python3 -m tocatl_sim.mesh_prep <src_dir> <dst_dir> [--faces 8000]
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import trimesh


def prep_mesh(src: Path, dst: Path, target_faces: int) -> dict:
    m = trimesh.load(src, force="mesh")
    n0 = len(m.faces)
    if n0 > target_faces:
        try:
            import fast_simplification
            v, f = fast_simplification.simplify(
                np.asarray(m.vertices, dtype=np.float32),
                np.asarray(m.faces, dtype=np.int64),
                target_count=target_faces)
            m = trimesh.Trimesh(vertices=v, faces=f, process=False)
        except ImportError:
            m = m.simplify_quadric_decimation(target_faces)
    m.export(dst)
    return {"faces_in": n0, "faces_out": len(m.faces),
            "extents_mm": (m.extents * 1000).round(1).tolist(),
            "bounds_mm": (m.bounds * 1000).round(1).tolist()}


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("src_dir", type=Path)
    ap.add_argument("dst_dir", type=Path)
    ap.add_argument("--faces", type=int, default=8000)
    args = ap.parse_args()
    args.dst_dir.mkdir(parents=True, exist_ok=True)
    for src in sorted(args.src_dir.glob("*.stl")):
        dst = args.dst_dir / (src.stem.lower().replace(" ", "_").replace("-", "_")
                              .replace("__", "_") + ".stl")
        info = prep_mesh(src, dst, args.faces)
        print(f"{src.name:45s} {info['faces_in']:>8d} -> {info['faces_out']:>6d} faces  "
              f"extents {info['extents_mm']} mm -> {dst.name}")


if __name__ == "__main__":
    main()
