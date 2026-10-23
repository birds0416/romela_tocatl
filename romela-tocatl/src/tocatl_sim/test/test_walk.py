"""Headless acceptance tests for the TOCATL MuJoCo walk (Section 9 metrics).

Runs physics + gait with no window, no ROS, no CAN. Execute with:

    PYTHONPATH=src/tocatl_sim:src/tocatl_kinematics:src/tocatl_gait:src/tocatl_diagnostics \
        python3 -m pytest src/tocatl_sim/test/test_walk.py -v

or via ``colcon test`` / plain pytest once the workspace is built and sourced.
"""

import math

import numpy as np
import pytest

import mujoco

from tocatl_kinematics.legacy_geometry import (
    Point3,
    foot_position_from_joint_angles,
    inverse_kinematics_from_foot_position,
)
from tocatl_sim.gait_engine import GaitEngine
from tocatl_sim.hexapod_sim import HexapodSim

CTRL_DT = 0.005  # 200 Hz command stream, as on the real robot


def make_rig():
    sim = HexapodSim()
    eng = GaitEngine()
    steps = round(CTRL_DT / sim.timestep)
    return sim, eng, steps


def drive(sim, eng, steps, seconds, cmd=(0.0, 0.0, 0.0), collect=False):
    touch_adr = [
        sim.model.sensor_adr[
            mujoco.mj_name2id(sim.model, mujoco.mjtObj.mjOBJ_SENSOR, f"leg{k}_foot_touch")]
        for k in range(1, 7)
    ]
    log = []
    for _ in range(int(seconds / CTRL_DT)):
        eng.set_velocity(*cmd)
        c = eng.update(CTRL_DT)
        sim.set_command(c.position, c.velocity, c.kp, c.kd, c.tau_ff)
        sim.step(steps)
        if collect:
            log.append({
                "base": sim.base_pose()[0],
                "rpy": sim.base_rpy(),
                "feet": sim.foot_positions(),
                "touch": np.array([sim.data.sensordata[a] for a in touch_adr]),
                "base_contact": sim.base_touching_ground(),
            })
    return log


def loaded_contact_slips(log, force_threshold=5.0, min_samples=5):
    """Max horizontal foot drift per loaded-contact window (physical slip)."""
    feet = np.array([e["feet"] for e in log])
    force = np.array([e["touch"] for e in log])
    slips = []
    for leg in range(6):
        start = None
        for t in range(len(log)):
            loaded = force[t, leg] > force_threshold
            if loaded and start is None:
                start = t
            if (not loaded or t == len(log) - 1) and start is not None:
                if t - start > min_samples:
                    seg = feet[start:t, leg, :2]
                    slips.append(float(np.linalg.norm(seg - seg[0], axis=1).max()))
                start = None
    return slips


def test_fk_ik_roundtrip():
    """Section 2.4: FK(IK(p)) == p over a grid of reachable foot points."""
    for x in np.linspace(120.0, 380.0, 7):
        for y in np.linspace(-150.0, 150.0, 7):
            for z in np.linspace(-380.0, -180.0, 5):
                th = inverse_kinematics_from_foot_position(Point3(x, y, z))
                p = foot_position_from_joint_angles(*th)
                err = math.dist((p.x_mm, p.y_mm, p.z_mm), (x, y, z))
                assert err < 1e-6, f"FK(IK) error {err} mm at ({x},{y},{z})"


def test_model_matches_legacy_fk():
    """The MJCF chain must reproduce the legacy leg FK exactly."""
    sim, _, _ = make_rig()
    rng = np.random.default_rng(1)
    for _ in range(50):
        th = rng.uniform([-1.0, -1.5, 0.2], [1.0, 1.5, 2.8])
        sim.data.qpos[:] = 0
        sim.data.qpos[3] = 1
        for leg in range(6):
            sim.data.qpos[sim.qpos_adr[leg * 3:leg * 3 + 3]] = th
        mujoco.mj_forward(sim.model, sim.data)
        f = foot_position_from_joint_angles(*th)
        from tocatl_kinematics.legacy_geometry import BODY_REFERENCE
        for leg in range(6):
            phi = BODY_REFERENCE.leg_mount_angles_rad[leg]
            r = BODY_REFERENCE.body_radius_mm * 1e-3
            c, s = math.cos(phi), math.sin(phi)
            pl = np.array([f.x_mm, f.y_mm, f.z_mm]) * 1e-3
            world = np.array([r * c + c * pl[0] - s * pl[1],
                              r * s + s * pl[0] + c * pl[1], pl[2]])
            err = np.linalg.norm(sim.foot_positions()[leg] - world)
            assert err < 1e-5, f"leg {leg + 1} FK mismatch {err} m"


def test_static_stand():
    """Rung 1: zero velocity for 10 s -> stable, level, feet planted."""
    sim, eng, steps = make_rig()
    log = drive(sim, eng, steps, 10.0, cmd=(0, 0, 0), collect=True)
    z = np.array([e["base"][2] for e in log])
    rpy = np.degrees(np.array([e["rpy"] for e in log]))
    assert z.max() - z.min() < 0.020, "base height not stable"
    assert np.abs(rpy[:, :2]).max() < 2.0, "roll/pitch during stand"
    feet0 = log[0]["feet"][:, :2]
    feet1 = log[-1]["feet"][:, :2]
    assert np.linalg.norm(feet1 - feet0, axis=1).max() < 0.010, "feet slid during stand"
    assert not any(e["base_contact"] for e in log)
    assert not np.isnan(sim.data.qpos).any()


def test_tripod_walk_forward():
    """Rung 3 + acceptance metrics: tripod, 5 s, nominal (full) speed."""
    sim, eng, steps = make_rig()
    drive(sim, eng, steps, 2.5, cmd=(1.0, 0, 0))          # ramp in
    p1 = sim.base_pose()[0].copy()
    log = drive(sim, eng, steps, 5.0, cmd=(1.0, 0, 0), collect=True)
    p2 = sim.base_pose()[0].copy()

    commanded = 0.140 * eng.speed_scale * 5.0  # legacy envelope x speed_scale
    disp = p2[0] - p1[0]
    assert disp >= 0.15, f"forward displacement {disp:.3f} m < 0.15"
    assert abs(disp - commanded) <= 0.25 * commanded, \
        f"displacement {disp:.3f} m vs commanded {commanded:.3f} m"
    assert abs(p2[1] - p1[1]) < 0.08, "walked sideways"

    rpy = np.degrees(np.array([e["rpy"] for e in log]))
    assert np.abs(rpy[:, 0]).mean() < 3.0, "mean roll too high"
    assert np.abs(rpy[:, 1]).mean() < 3.0, "mean pitch too high"
    z = np.array([e["base"][2] for e in log])
    assert z.std() < 0.015, f"base height std {z.std() * 1000:.1f} mm"
    assert not any(e["base_contact"] for e in log), "base hit the ground"
    assert not np.isnan(sim.data.qpos).any(), "joint NaNs"

    slips = loaded_contact_slips(log)
    assert slips, "no loaded stance windows found"
    assert max(slips) < 0.015, f"foot slip {max(slips) * 1000:.1f} mm >= 15 mm"


@pytest.mark.parametrize("gait", ["ripple", "wave"])
def test_other_gaits_stay_upright(gait):
    """Rung 4: ripple and wave walk without falling."""
    sim, eng, steps = make_rig()
    eng.set_gait(gait)
    drive(sim, eng, steps, 2.0, cmd=(1.0, 0, 0))
    p1 = sim.base_pose()[0].copy()
    log = drive(sim, eng, steps, 6.0, cmd=(1.0, 0, 0), collect=True)
    p2 = sim.base_pose()[0].copy()
    rpy = np.degrees(np.array([e["rpy"] for e in log]))
    assert p2[0] > p1[0] + 0.05, f"{gait}: no forward progress"
    assert np.abs(rpy[:, :2]).max() < 6.0, f"{gait}: attitude excursion"
    assert sim.base_pose()[0][2] > 0.15, f"{gait}: body collapsed"


def test_turn_and_gait_switch():
    """Rung 4: turning and switching gaits mid-walk without a stumble."""
    sim, eng, steps = make_rig()
    drive(sim, eng, steps, 4.0, cmd=(0, 0, 1.0))
    yaw = sim.base_rpy()[2]
    assert math.degrees(yaw) > 20.0, "did not turn"
    eng.set_gait("ripple")
    drive(sim, eng, steps, 3.0, cmd=(1.0, 0, 0))
    eng.set_gait("tripod")
    log = drive(sim, eng, steps, 3.0, cmd=(1.0, 0, 0), collect=True)
    rpy = np.degrees(np.array([e["rpy"] for e in log]))
    assert np.abs(rpy[:, :2]).max() < 6.0, "stumbled on gait switch"
    assert sim.base_pose()[0][2] > 0.15
    assert not np.isnan(sim.data.qpos).any()
