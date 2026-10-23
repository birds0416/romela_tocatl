"""Placement of the TOCATL CAD meshes on the sim's kinematic skeleton.

Each entry places one converted CAD part (STL, metres, SolidWorks part frame)
on one link of the simulated robot. The link frames are those of
``model_builder``:

  base   -- base_link centre, z up
  coxa   -- leg mount point on the body (radius 0.30 m, x radially outward)
  hip    -- hip/pitch-axis point [d2, 0, d1] relative to coxa; yaw rotates it
  femur  -- same origin as hip, rotates with hip_pitch; x toward the knee
  tibia  -- knee point; x toward the foot tip

``pos`` (m) and ``euler`` (xyz, radians) transform the part's STL frame into
the link frame. Values were derived by inspecting each part and the CAD
assembly, then verified visually against offscreen renders.
"""

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass(frozen=True)
class MeshSpec:
    """One CAD part attached to one link of the robot."""

    mesh: str                   # STL basename (without .stl) under the mesh dir
    parent: str                 # base | coxa | hip | femur | tibia
    pos: tuple = (0.0, 0.0, 0.0)
    euler: tuple = (0.0, 0.0, 0.0)   # xyz intrinsic, radians
    rgba: tuple = (0.35, 0.36, 0.38, 1.0)
    per_leg: bool = True        # replicate on all six legs (parent != base)


PI = 3.141592653589793
H = PI / 2

BLACK = (0.13, 0.13, 0.14, 1.0)
DARK = (0.18, 0.18, 0.20, 1.0)

# d-values from legacy_geometry: d1=0.11425, d2=0.0455561, d3=0.3303455, d4=0.458
MESH_LAYOUT: list[MeshSpec] = [
    # ---- body, replicated per sextant via the coxa bodies ----
    # part frames: x_p tangential, y_p vertical, z_p radial (origin at the rim)
    # cyclic remap x_p->y, y_p->z, z_p->x  ==  euler (H, H, 0)
    # dome shell: parametric rebuild (dome_builder.py), single instance at centre
    MeshSpec("dome",      "base", pos=(0, 0, 0.03), euler=(0, 0, 0), rgba=BLACK,
             per_leg=False),
    MeshSpec("dome_trim", "base", pos=(0, 0, 0.03), euler=(0, 0, 0),
             rgba=(0.85, 0.85, 0.86, 1.0), per_leg=False),
    MeshSpec("body_sextant", "coxa", pos=(0, 0.153, 0.0), euler=(H, H, 0), rgba=DARK),
    MeshSpec("body_plate",   "coxa", pos=(0, 0.1005, 0.0), euler=(H, H, 0), rgba=DARK),
    MeshSpec("body_bottom",  "coxa", pos=(0, 0, -0.06), euler=(H, H, 0), rgba=DARK),

    # ---- hip cluster ----
    # turret: drums (pitch-motor clamp) along part x at (y 59.5, z 52.7) mm;
    # map x_p->y, y_p->-z (yaw flange down), z_p->-x; drum centre -> hip origin
    MeshSpec("hip_mount",  "coxa", pos=(0.0455561, 0, 0.0), euler=(0, 0, 0), rgba=BLACK),
    MeshSpec("hip_turret", "hip",  pos=(0.0527, 0, 0.0595), euler=(-H, -H, 0), rgba=BLACK),

    # ---- femur (x toward the knee) ----
    MeshSpec("femur_mount", "femur", pos=(0.075, 0, 0), euler=(0, H, 0), rgba=BLACK),
    MeshSpec("femur_beam",  "femur", pos=(0.075, 0, 0), euler=(0, H, 0), rgba=BLACK),
    # knee cluster: motor clamped between the bearing attachment drums at the
    # knee end (drum axis z_p -> pitch axis y)
    MeshSpec("bearing_attach_l", "femur", pos=(0.275, 0.032, 0.025), euler=(-H, 0, 0), rgba=BLACK),
    MeshSpec("bearing_attach_r", "femur", pos=(0.275, -0.032, 0.025), euler=(-H, 0, 0), rgba=BLACK),
    MeshSpec("knee_bar", "femur", pos=(0.015, -0.005, 0.062), euler=(0, -0.08, -H), rgba=BLACK),
    MeshSpec("knee_motor_adapter", "femur", pos=(0.275, 0.028, 0.025), euler=(H, 0, 0), rgba=BLACK),

    # ---- lower leg (x toward the foot tip) ----
    MeshSpec("knee_hinge", "tibia", pos=(0, 0, 0), euler=(0, 0, 0), rgba=BLACK),
    MeshSpec("tibia",  "tibia", pos=(0.027, 0, 0), euler=(0, 0, H), rgba=BLACK),
    MeshSpec("tarsus", "tibia", pos=(0.30, 0, 0), euler=(0, 0, -H), rgba=BLACK),
    MeshSpec("toe",    "tibia", pos=(0.428, 0, 0), euler=(0, 0, -H), rgba=BLACK),
]
