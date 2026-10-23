"""Parametric MJCF builder for the TOCATL hexapod.

Generates the MuJoCo model from the recovered geometry in
``tocatl_kinematics.legacy_geometry`` so the simulated kinematic chain matches
the legacy FK/IK exactly:

    foot_leg = [d2, 0, d1] + Rz(hip_yaw) . Ry(hip_pitch) . [d3,0,0]
                            + Rz(hip_yaw) . Ry(hip_pitch+knee) . [d4,0,0]

Note the legacy controller models the yaw rotation about the point
``[d2, 0, d1]`` (the hip-pitch axis), not about the mount. The MJCF reproduces
that: the hip_yaw hinge is anchored at the pitch-axis point so IK targets are
tracked without kinematic mismatch. The 45 mm coxa stub is a fixed visual.

Simulation only: no CAN, no hardware. Units are SI (legacy mm are converted).
"""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass, field

from tocatl_kinematics.legacy_geometry import (
    BODY_REFERENCE,
    DEFAULT_LEG_GEOMETRY,
    Point3,
    inverse_kinematics_from_foot_position,
)

MM = 1e-3

try:
    from .cad_layout import MESH_LAYOUT
except ImportError:  # direct module use
    from cad_layout import MESH_LAYOUT

# Canonical joint order (Section 2.5 of the sim brief): leg 1..6, within each
# leg hip_yaw, hip_pitch, knee. Motor id leg*10+joint; buses noted in metadata
# only (see tocatl_diagnostics.motor_map) -- irrelevant in sim.
JOINT_SUFFIXES = ("hip_yaw", "hip_pitch", "knee")
JOINT_NAMES = tuple(
    f"leg{leg}_{suffix}" for leg in range(1, 7) for suffix in JOINT_SUFFIXES
)


def default_mesh_dir() -> str | None:
    """Locate the installed (or in-source) CAD mesh directory, if present."""
    import os

    try:
        from ament_index_python.packages import get_package_share_directory
        cand = os.path.join(get_package_share_directory("tocatl_sim"), "meshes")
        if os.path.isdir(cand):
            return cand
    except Exception:
        pass
    cand = os.path.join(os.path.dirname(__file__), "..", "meshes")
    return os.path.abspath(cand) if os.path.isdir(cand) else None


@dataclass
class ModelParams:
    """All tunables of the generated model (SI units)."""

    # stance defaults used for the spawn keyframe (match GaitConfig)
    stance_radius: float = 0.25   # leg-local x of the neutral foot point [m]
    body_height: float = 0.22     # hip plane height above foot tips [m]

    # masses [kg] -- CyberGears dominate; mass sits near the joints
    base_mass: float = 6.0
    coxa_mass: float = 0.25
    hip_mass: float = 0.45
    femur_mass: float = 0.35
    tibia_mass: float = 0.18
    foot_mass: float = 0.03

    # geometry of the visuals/collision
    base_radius: float = 0.30
    base_half_height: float = 0.05
    femur_capsule_radius: float = 0.030
    tibia_capsule_radius: float = 0.022
    foot_radius: float = 0.012

    # actuation / joints
    torque_limit: float = 12.0    # CyberGear peak torque [N m]
    joint_damping: float = 0.1
    joint_armature: float = 0.01

    # contact
    foot_friction: str = "1.2 0.02 0.001"
    foot_solref: str = "0.01 1"
    foot_solimp: str = "0.9 0.95 0.001"

    # integration
    timestep: float = 0.002

    # joint ranges (theta frame, generous mechanical range; the gait planner
    # stays inside the legacy motor windows via apply_legacy_motor_windows)
    hip_yaw_range: tuple = (-1.2, 1.2)
    hip_pitch_range: tuple = (-2.35, 2.55)
    knee_range: tuple = (0.0, 2.95)

    # directory of converted CAD meshes (STL, metres); None = capsule visuals
    mesh_dir: str | None = None

    geometry: object = field(default=DEFAULT_LEG_GEOMETRY)
    body: object = field(default=BODY_REFERENCE)


def neutral_joint_angles(params: ModelParams) -> tuple[float, float, float]:
    """(hip_yaw, hip_pitch, knee) for the neutral stand, via the repo IK."""
    foot = Point3(
        x_mm=params.stance_radius / MM + params.geometry.hip_horizontal_offset_mm,
        y_mm=0.0,
        z_mm=-params.body_height / MM,
    )
    return inverse_kinematics_from_foot_position(foot, params.geometry)


def _lookat_xyaxes(pos, target, up=(0.0, 0.0, 1.0)) -> str:
    """xyaxes string for a camera at pos looking at target."""
    f = [t - p for t, p in zip(target, pos)]
    n = math.sqrt(sum(c * c for c in f))
    f = [c / n for c in f]
    r = [f[1] * up[2] - f[2] * up[1], f[2] * up[0] - f[0] * up[2], f[0] * up[1] - f[1] * up[0]]
    n = math.sqrt(sum(c * c for c in r))
    r = [c / n for c in r]
    u = [r[1] * f[2] - r[2] * f[1], r[2] * f[0] - r[0] * f[2], r[0] * f[1] - r[1] * f[0]]
    return " ".join(f"{c:.6f}" for c in r + u)


def _mesh_setup(p: "ModelParams") -> tuple[str, dict]:
    """Collect available CAD meshes: (asset xml, {parent: [geom xml, ...]}).

    Mesh geoms are visual-only (no contact, no mass); the primitive geoms
    keep carrying the mass/inertia and are moved to group 3 (hidden) when a
    mesh covers their link, so dynamics are identical with or without CAD.
    """
    import os

    if not p.mesh_dir:
        return "", {}
    assets: list[str] = []
    by_parent: dict[str, list[str]] = {}
    seen: set[str] = set()
    for spec in MESH_LAYOUT:
        path = os.path.join(p.mesh_dir, spec.mesh + ".stl")
        if not os.path.isfile(path):
            continue
        if spec.mesh not in seen:
            seen.add(spec.mesh)
            assets.append(f'    <mesh name="{spec.mesh}" file="{path}"/>')
        by_parent.setdefault(spec.parent, []).append(
            f'<geom type="mesh" mesh="{spec.mesh}" '
            f'pos="{spec.pos[0]:.6f} {spec.pos[1]:.6f} {spec.pos[2]:.6f}" '
            f'euler="{spec.euler[0]:.6f} {spec.euler[1]:.6f} {spec.euler[2]:.6f}" '
            f'rgba="{spec.rgba[0]} {spec.rgba[1]} {spec.rgba[2]} {spec.rgba[3]}" '
            f'contype="0" conaffinity="0" group="1" mass="0"/>')
    return "\n".join(assets), by_parent


def build_mjcf(params: ModelParams | None = None) -> str:
    p = params or ModelParams()
    mesh_assets, mesh_geoms = _mesh_setup(p)

    def link_geoms(parent: str, indent: str) -> str:
        return "".join(f"\n{indent}{g}" for g in mesh_geoms.get(parent, []))

    def grp(parent: str) -> str:
        # hide the primitive visual when a CAD mesh covers this link
        return ' group="3"' if parent in mesh_geoms else ""

    if mesh_geoms:
        # stylized CyberGear motors (86 x 36.5 mm) — the real motor CAD did
        # not convert; visually they anchor each joint
        motor = ('<geom type="cylinder" size="0.043 0.0185" contype="0" '
                 'conaffinity="0" group="1" mass="0" rgba="0.09 0.09 0.10 1"')
        d1v, d2v = 0.11425, 0.0455561
        mesh_geoms.setdefault("coxa", []).append(
            f'{motor} pos="{d2v} 0 {d1v - 0.0775:.4f}"/>')          # yaw motor
        mesh_geoms.setdefault("hip", []).append(
            f'{motor} pos="0 0 0" euler="1.5708 0 0"/>')            # pitch motor
        mesh_geoms.setdefault("femur", []).append(
            f'{motor} pos="0.275 0 0.025" euler="1.5708 0 0"/>')    # knee motor
    g = p.geometry
    d1 = g.hip_vertical_offset_mm * MM
    d2 = g.hip_horizontal_offset_mm * MM
    d3 = g.upper_link_mm * MM
    d4 = g.lower_link_mm * MM
    body_r = p.body.body_radius_mm * MM

    th1, th2, th3 = neutral_joint_angles(p)
    spawn_z = p.body_height + p.foot_radius

    legs = []
    actuators = []
    sensors = []
    for k in range(1, 7):
        phi = p.body.leg_mount_angles_rad[k - 1]
        mx, my = body_r * math.cos(phi), body_r * math.sin(phi)
        c2, s2 = math.cos(phi / 2.0), math.sin(phi / 2.0)
        legs.append(f"""
      <body name="leg{k}_coxa" pos="{mx:.6f} {my:.6f} 0" quat="{c2:.8f} 0 0 {s2:.8f}">
        <geom name="leg{k}_coxa_stub" type="capsule" fromto="0 0 0 {d2:.6f} 0 {d1:.6f}"
              size="0.028" mass="{p.coxa_mass}" class="leglink"{grp('coxa')} rgba="0.25 0.25 0.27 1"/>{link_geoms('coxa', '        ')}
        <body name="leg{k}_hip" pos="{d2:.6f} 0 {d1:.6f}">
          <joint name="leg{k}_hip_yaw" axis="0 0 1"
                 range="{p.hip_yaw_range[0]} {p.hip_yaw_range[1]}"/>
          <geom name="leg{k}_hip_motor" type="sphere" size="0.045" mass="{p.hip_mass}"
                class="leglink"{grp('hip')} rgba="0.85 0.33 0.10 1"/>{link_geoms('hip', '          ')}
          <body name="leg{k}_femur" pos="0 0 0">
            <joint name="leg{k}_hip_pitch" axis="0 1 0"
                   range="{p.hip_pitch_range[0]} {p.hip_pitch_range[1]}"/>
            <geom name="leg{k}_femur_link" type="capsule" fromto="0 0 0 {d3:.6f} 0 0"
                  size="{p.femur_capsule_radius}" mass="{p.femur_mass}" class="leglink"{grp('femur')}
                  rgba="0.30 0.30 0.33 1"/>{link_geoms('femur', '            ')}
            <body name="leg{k}_tibia" pos="{d3:.6f} 0 0">
              <joint name="leg{k}_knee" axis="0 1 0"
                     range="{p.knee_range[0]} {p.knee_range[1]}"/>
              <geom name="leg{k}_knee_motor" type="sphere" size="0.036" mass="0.0"
                    class="leglink"{grp('tibia')} rgba="0.85 0.33 0.10 1"/>
              <geom name="leg{k}_tibia_link" type="capsule" fromto="0 0 0 {d4:.6f} 0 0"
                    size="{p.tibia_capsule_radius}" mass="{p.tibia_mass}" class="leglink"{grp('tibia')}
                    rgba="0.30 0.30 0.33 1"/>{link_geoms('tibia', '              ')}
              <geom name="leg{k}_foot" type="sphere" pos="{d4:.6f} 0 0"
                    size="{p.foot_radius}" mass="{p.foot_mass}" class="foot"/>
              <site name="leg{k}_foot_tip" pos="{d4:.6f} 0 0" size="0.024" group="4"/>
            </body>
          </body>
        </body>
      </body>""")
        for suffix in JOINT_SUFFIXES:
            actuators.append(
                f'    <motor name="leg{k}_{suffix}" joint="leg{k}_{suffix}" '
                f'ctrlrange="-{p.torque_limit} {p.torque_limit}" '
                f'forcerange="-{p.torque_limit} {p.torque_limit}"/>'
            )
            sensors.append(f'    <jointpos name="leg{k}_{suffix}_pos" joint="leg{k}_{suffix}"/>')
        sensors.append(f'    <touch name="leg{k}_foot_touch" site="leg{k}_foot_tip"/>')

    stand_qpos = f"0 0 {spawn_z:.6f} 1 0 0 0 " + " ".join(
        f"{th1:.6f} {th2:.6f} {th3:.6f}" for _ in range(6)
    )

    cam34 = _lookat_xyaxes((1.9, -1.9, 1.15), (0.0, 0.0, 0.25))
    cam_front = _lookat_xyaxes((2.4, 0.0, 0.85), (0.0, 0.0, 0.25))

    xml = f"""<mujoco model="tocatl">
  <!-- Generated by tocatl_sim.model_builder from tocatl_kinematics.legacy_geometry.
       Regenerate with: python3 -m tocatl_sim.model_builder -o tocatl.xml -->
  <compiler angle="radian" autolimits="true"/>
  <option timestep="{p.timestep}" gravity="0 0 -9.81" integrator="implicitfast" cone="elliptic"
          impratio="10"/>

  <visual>
    <global offwidth="1920" offheight="1080"/>
    <headlight ambient="0.35 0.35 0.35" diffuse="0.7 0.7 0.7" specular="0.2 0.2 0.2"/>
    <quality shadowsize="4096"/>
    <map znear="0.02" haze="0.15"/>
  </visual>

  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.45 0.60 0.75" rgb2="0.90 0.93 0.96" width="512" height="512"/>
    <texture name="grid" type="2d" builtin="checker" rgb1="0.22 0.24 0.26" rgb2="0.30 0.32 0.34"
             width="512" height="512" mark="edge" markrgb="0.45 0.47 0.50"/>
    <material name="grid" texture="grid" texrepeat="6 6" texuniform="true" reflectance="0.12"/>
{mesh_assets}
  </asset>

  <default>
    <joint type="hinge" damping="{p.joint_damping}" armature="{p.joint_armature}"/>
    <default class="leglink">
      <geom contype="0" conaffinity="0" group="1"/>
    </default>
    <default class="foot">
      <geom contype="1" conaffinity="1" condim="6" friction="{p.foot_friction}"
            solref="{p.foot_solref}" solimp="{p.foot_solimp}" rgba="0.10 0.10 0.10 1"/>
    </default>
  </default>

  <worldbody>
    <light pos="0 0 4" dir="0 0 -1" directional="true" castshadow="true"/>
    <light pos="2.5 -2.5 3" dir="-0.5 0.5 -0.8" diffuse="0.35 0.35 0.35" castshadow="false"/>
    <geom name="floor" type="plane" size="0 0 0.05" material="grid" condim="6"
          friction="{p.foot_friction}"/>

    <camera name="track34" mode="trackcom" pos="1.9 -1.9 1.15" xyaxes="{cam34}"/>
    <camera name="front" mode="trackcom" pos="2.4 0 0.85" xyaxes="{cam_front}"/>
    <camera name="fixed34" mode="fixed" pos="1.9 -1.9 1.15" xyaxes="{cam34}"/>

    <body name="base_link" pos="0 0 {spawn_z:.6f}">
      <freejoint name="base_free"/>
      <geom name="base_plate" type="cylinder" size="{p.base_radius} {p.base_half_height}"
            mass="{p.base_mass}"{grp('base')} rgba="0.16 0.17 0.19 1"/>
      <geom name="base_top" type="cylinder" size="{p.base_radius * 0.55} {p.base_half_height * 0.6}"
            pos="0 0 {p.base_half_height * 1.4:.4f}" mass="0.0" contype="0" conaffinity="0"
            {grp('base')} rgba="0.85 0.33 0.10 1"/>{link_geoms('base', '      ')}
      <site name="base_site" size="0.02" group="4"/>
{''.join(legs)}
    </body>
  </worldbody>

  <actuator>
{chr(10).join(actuators)}
  </actuator>

  <sensor>
    <framepos name="base_pos" objtype="site" objname="base_site"/>
    <framequat name="base_quat" objtype="site" objname="base_site"/>
    <framelinvel name="base_linvel" objtype="site" objname="base_site"/>
{chr(10).join(sensors)}
  </sensor>

  <keyframe>
    <key name="stand" qpos="{stand_qpos}"/>
  </keyframe>
</mujoco>
"""
    return xml


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("-o", "--out", default="tocatl.xml", help="output MJCF path")
    args = parser.parse_args()
    with open(args.out, "w") as f:
        f.write(build_mjcf())
    print(f"wrote {args.out}")


if __name__ == "__main__":
    main()
