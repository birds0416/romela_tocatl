"""MuJoCo physics wrapper for the TOCATL hexapod (simulation only, no CAN).

Reproduces the CyberGear MIT operation-control mode on torque actuators:

    tau = kp * (pos_des - q) + kd * (vel_des - qdot) + tau_ff, clipped to +-12 N m

evaluated every physics step from the most recent command frame, like the
motor's internal loop running on the last streamed (pos, vel, kp, kd, tau).
"""

from __future__ import annotations

import numpy as np

import mujoco

from .model_builder import (JOINT_NAMES, ModelParams, build_mjcf,
                            default_mesh_dir, neutral_joint_angles)


class HexapodSim:
    """Owns MjModel/MjData and the MIT-mode command state for 18 joints."""

    def __init__(self, params: ModelParams | None = None, xml: str | None = None):
        self.params = params or ModelParams(mesh_dir=default_mesh_dir())
        self.model = mujoco.MjModel.from_xml_string(xml or build_mjcf(self.params))
        self.data = mujoco.MjData(self.model)

        self.qpos_adr = np.array([
            self.model.jnt_qposadr[
                mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, n)]
            for n in JOINT_NAMES
        ])
        self.qvel_adr = np.array([
            self.model.jnt_dofadr[
                mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, n)]
            for n in JOINT_NAMES
        ])
        # actuators were declared in JOINT_NAMES order; map by name to be safe
        self.act_ids = np.array([
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, n)
            for n in JOINT_NAMES
        ])
        self.foot_site_ids = np.array([
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, f"leg{k}_foot_tip")
            for k in range(1, 7)
        ])
        self.base_body_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
        self.floor_geom_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_GEOM, "floor")
        self.base_geom_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_GEOM, "base_plate")
        self.tau_limit = self.params.torque_limit

        # default command: hold the stand keyframe with firm stance gains
        th = neutral_joint_angles(self.params)
        self.cmd_pos = np.tile(np.array(th), 6)
        self.cmd_vel = np.zeros(18)
        self.cmd_kp = np.full(18, 100.0)
        self.cmd_kd = np.full(18, 3.0)
        self.cmd_tau = np.zeros(18)

        self.reset_stand()

    # ---------------- state ----------------

    def reset_stand(self) -> None:
        mujoco.mj_resetDataKeyframe(self.model, self.data, 0)
        mujoco.mj_forward(self.model, self.data)

    @property
    def timestep(self) -> float:
        return self.model.opt.timestep

    def joint_positions(self) -> np.ndarray:
        return self.data.qpos[self.qpos_adr].copy()

    def joint_velocities(self) -> np.ndarray:
        return self.data.qvel[self.qvel_adr].copy()

    def base_pose(self) -> tuple[np.ndarray, np.ndarray]:
        """(xyz, wxyz quaternion) of the floating base."""
        return self.data.qpos[0:3].copy(), self.data.qpos[3:7].copy()

    def base_velocity(self) -> tuple[np.ndarray, np.ndarray]:
        return self.data.qvel[0:3].copy(), self.data.qvel[3:6].copy()

    def base_rpy(self) -> tuple[float, float, float]:
        w, x, y, z = self.data.qpos[3:7]
        roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
        pitch = np.arcsin(np.clip(2 * (w * y - z * x), -1.0, 1.0))
        yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        return roll, pitch, yaw

    def foot_positions(self) -> np.ndarray:
        """(6,3) world positions of the foot tips."""
        return self.data.site_xpos[self.foot_site_ids].copy()

    def base_touching_ground(self) -> bool:
        for i in range(self.data.ncon):
            c = self.data.contact[i]
            pair = {c.geom1, c.geom2}
            if self.floor_geom_id in pair and self.base_geom_id in pair:
                return True
        return False

    # ---------------- control ----------------

    def set_command(self, pos, vel=None, kp=None, kd=None, tau_ff=None) -> None:
        self.cmd_pos = np.asarray(pos, dtype=float)
        if vel is not None:
            self.cmd_vel = np.asarray(vel, dtype=float)
        if kp is not None:
            self.cmd_kp = np.asarray(kp, dtype=float)
        if kd is not None:
            self.cmd_kd = np.asarray(kd, dtype=float)
        if tau_ff is not None:
            self.cmd_tau = np.asarray(tau_ff, dtype=float)

    def _apply_mit_torque(self) -> None:
        q = self.data.qpos[self.qpos_adr]
        qd = self.data.qvel[self.qvel_adr]
        tau = (self.cmd_kp * (self.cmd_pos - q)
               + self.cmd_kd * (self.cmd_vel - qd)
               + self.cmd_tau)
        self.data.ctrl[self.act_ids] = np.clip(tau, -self.tau_limit, self.tau_limit)

    def step(self, n: int = 1) -> None:
        for _ in range(n):
            self._apply_mit_torque()
            mujoco.mj_step(self.model, self.data)
