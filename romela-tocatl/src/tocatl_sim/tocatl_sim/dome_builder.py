"""Parametric TOCATL body dome (the printed shell could not be tessellated
from CAD, so the dome is rebuilt to match the real robot's shape: six creased
petals with an arched leg opening per sextant, plus a light trim band around
each arch).

Writes dome.stl and dome_trim.stl (metres, origin at body centre, z up,
sector centres at the leg mount angles 30 + k*60 deg).

    python3 -m tocatl_sim.dome_builder <out_dir>
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import trimesh

R_OUT = 0.405          # dome rim radius [m]
HEIGHT = 0.165         # dome apex height above the rim plane [m]
RIM_DROP = 0.020       # rim skirt drop below the base plane
CREASE = 0.055         # petal sag between the seam ribs (fraction of z)
ARCH_HALF = 0.42       # arch half-width as fraction of the sector half-angle
ARCH_DEPTH = 0.52      # arch reach inward from the rim (fraction of radius)
FIRST_SECTOR = np.pi / 6.0   # leg 1 mount angle

NS, NT = 72, 44        # radial / tangential resolution per sector


def _profile(s: np.ndarray) -> np.ndarray:
    """Dome height vs normalized radius s in [0, 1]."""
    z = HEIGHT * (1.0 - s ** 2.4) ** 0.85
    return z - (RIM_DROP + HEIGHT) * np.maximum(s - 0.97, 0.0) / 0.03 * 0.0 \
        - RIM_DROP * np.clip((s - 0.85) / 0.15, 0.0, 1.0) ** 2


def _in_arch(s: float, t: float) -> bool:
    """True when (s, t) falls inside the leg arch opening of a sector."""
    if abs(t) >= ARCH_HALF:
        return False
    lim = 1.0 - ARCH_DEPTH * (1.0 - (t / ARCH_HALF) ** 2)
    return s > lim


def _arch_boundary(t: float) -> float:
    return 1.0 - ARCH_DEPTH * (1.0 - (t / ARCH_HALF) ** 2)


def build_dome() -> tuple[trimesh.Trimesh, trimesh.Trimesh]:
    verts: list = []
    faces: list = []
    trim_faces: list = []

    def vid(i_s, i_t, sector, cache={}):
        key = (i_s, i_t, sector)
        if key in cache:
            return cache[key]
        s = i_s / (NS - 1)
        t = -1.0 + 2.0 * i_t / (NT - 1)
        ang = FIRST_SECTOR + sector * np.pi / 3.0 + t * np.pi / 6.0
        r = R_OUT * s
        z = float(_profile(np.array([s]))[0])
        z *= 1.0 - CREASE * (1.0 - t * t) * s ** 1.5
        v = (r * np.cos(ang), r * np.sin(ang), z)
        cache[key] = len(verts)
        verts.append(v)
        return cache[key]

    for sector in range(6):
        for i_s in range(NS - 1):
            for i_t in range(NT - 1):
                corners = [(i_s, i_t), (i_s + 1, i_t), (i_s + 1, i_t + 1), (i_s, i_t + 1)]
                sts = [((i / (NS - 1)), -1.0 + 2.0 * j / (NT - 1)) for i, j in corners]
                inside = [_in_arch(s, t) for s, t in sts]
                if all(inside):
                    continue
                a, b, c, d = [vid(i, j, sector) for i, j in corners]
                cell = [(a, b, c), (a, c, d)]
                faces.extend(cell)
                if any(inside):
                    trim_faces.extend(cell)  # boundary cells become the trim band

    verts_np = np.array(verts)
    dome = trimesh.Trimesh(verts_np, np.array(faces), process=False)
    trim = trimesh.Trimesh(verts_np, np.array(trim_faces), process=False) \
        if trim_faces else trimesh.Trimesh()
    # double-side both (thin shell)
    dome = trimesh.Trimesh(dome.vertices, np.vstack([dome.faces, dome.faces[:, ::-1]]),
                           process=False)
    if len(trim.faces):
        trim = trimesh.Trimesh(trim.vertices, np.vstack([trim.faces, trim.faces[:, ::-1]]),
                               process=False)
        # push the trim slightly outward along normals so it overdraws the dome
        trim = trimesh.Trimesh(trim.vertices + trim.vertex_normals * 0.0012,
                               trim.faces, process=False)
    return dome, trim


def main() -> None:
    out = Path(sys.argv[1] if len(sys.argv) > 1 else ".")
    out.mkdir(parents=True, exist_ok=True)
    dome, trim = build_dome()
    dome.export(out / "dome.stl")
    trim.export(out / "dome_trim.stl")
    print(f"dome: {len(dome.faces)} faces, extents(mm) {(dome.extents*1000).round(1).tolist()}")
    print(f"trim: {len(trim.faces)} faces")


if __name__ == "__main__":
    main()
