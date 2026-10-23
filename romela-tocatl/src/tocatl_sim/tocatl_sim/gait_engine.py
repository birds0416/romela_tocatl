"""Pure-python port of the recovered WalkingTestv3 gait for simulation.

Faithful port of the anchor-based world-frame gait from
``legacy/deployed-2026-03-15/tocatl_testing/WalkingTestv3`` with the CAN /
motor layer removed. Geometry and IK come from
``tocatl_kinematics.legacy_geometry``; gait timing/phase definitions come from
``tocatl_gait.legacy_reference``; joint ordering follows
``tocatl_diagnostics.motor_map`` (leg 1..6, hip_yaw / hip_pitch / knee).

The engine holds its own "gait world" frame: the commanded body pose is
integrated from the velocity command, stance feet are pinned to world anchors
(so in the real/simulated world they do not slip), and swing feet fly a
sinusoidal arch from lift-off to touchdown. Output per 200 Hz tick is the
18-joint theta target plus the phase-scheduled MIT gains (kp, kd, tau_ff).

No ROS, no MuJoCo, no CAN in this module. All distances in mm like the legacy
code; angles in radians.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from tocatl_kinematics.legacy_geometry import (
    BODY_REFERENCE,
    DEFAULT_LEG_GEOMETRY,
    Point3,
    apply_legacy_motor_windows,
    inverse_kinematics_from_foot_position,
    wrap_pi,
)
from tocatl_gait.legacy_reference import (
    GAIT_BY_NAME,
    LEGACY_WALKING_ENVELOPE,
    phase_for_leg,
)

ENV = LEGACY_WALKING_ENVELOPE

BODY_R_MM = BODY_REFERENCE.body_radius_mm
LEG_PHI = BODY_REFERENCE.leg_mount_angles_rad

# Legacy WalkingTestv3 constants (values ported verbatim unless noted)
X_MIN_MM = 40.0
CMD_SLEW_RATE = 4.0
TRANS_SPEED_MIN_FOR_STEPPING = 0.10
YAW_MIN_FOR_STEPPING = 0.08
YAW_STRIDE_SCALE = 0.2
RETURN_CYCLE_TIME = 0.6
RETURN_ORDER = (0, 2, 4, 1, 3, 5)
PARK_LOWER_RATE = 140.0  # mm/s
PARK_BODY_HEIGHT_MM = 120.0  # sim: settle low but keep feet reachable


def _clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else (hi if x > hi else x)


def _slew_exp(cur: float, target: float, rate: float, dt: float) -> float:
    alpha = 1.0 - math.exp(-rate * dt)
    return cur + (target - cur) * alpha


def _rz(th: float) -> np.ndarray:
    c, s = math.cos(th), math.sin(th)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


@dataclass
class GainSchedule:
    """Per-joint-group MIT gains and the stance/swing scheduler rates.

    Legacy WalkingTestv3 used kp_rate_up=1.0 / kp_rate_down=6.0 (soft, slow
    plant). The tuned defaults ramp stance stiffness in fast and bleed it off
    slowly, which plants harder and reads much firmer in MuJoCo.
    """

    kp_hip_yaw_stance: float = 100.0
    kp_hip_yaw_swing: float = 10.0
    kd_hip_yaw: float = 3.0
    kp_hip_pitch_stance: float = 100.0
    kp_hip_pitch_swing: float = 10.0
    kd_hip_pitch: float = 3.0
    kp_knee_stance: float = 100.0
    kp_knee_swing: float = 10.0
    kd_knee: float = 3.0
    kp_rate_up: float = 25.0
    kp_rate_down: float = 6.0
    hip_pitch_support_tau: float = 0.0  # constant feedforward, <= 9.25


@dataclass
class GaitConfig:
    """Envelope / stance tunables (mm, s). Defaults follow the legacy envelope."""

    stride_mm: float = ENV.default_stride_length_mm
    step_height_mm: float = ENV.default_step_height_mm
    # spider stance matched to the real robot: body low, feet wide, knees high
    body_height_mm: float = 220.0
    stance_radius_mm: float = 250.0  # foot x in leg frame, beyond the d2 offset
    body_height_min_mm: float = 160.0
    body_height_max_mm: float = 340.0
    stance_radius_min_mm: float = 140.0
    stance_radius_max_mm: float = 340.0
    cycle_fast_s: float = ENV.cycle_fast_s
    cycle_slow_s: float = ENV.cycle_slow_s
    foot_speed_max_mm_s: float = ENV.foot_speed_max_mm_s
    body_speed_max_mm_s: float = ENV.body_speed_max_mm_s
    body_yaw_rate_max_rad_s: float = ENV.body_yaw_rate_max_rad_s
    gains: GainSchedule = field(default_factory=GainSchedule)


@dataclass
class JointCommand:
    """One 200 Hz command frame for all 18 joints (Section 2.5 order)."""

    position: np.ndarray  # theta targets [rad]
    velocity: np.ndarray  # desired qdot [rad/s] (zero, like the legacy streamer)
    kp: np.ndarray
    kd: np.ndarray
    tau_ff: np.ndarray


class GaitEngine:
    """Ported HexapodWalkJoy locomotion core, keyboard/sim flavored."""

    def __init__(self, config: GaitConfig | None = None, gait_name: str = "tripod"):
        self.cfg = config or GaitConfig()
        self.gait = GAIT_BY_NAME[gait_name]
        self.gait_name = gait_name
        self.cycle_time = self.cfg.cycle_slow_s
        self.u_global = 0.0

        # normalized commands in [-1, 1] (joystick convention of the legacy code)
        self.vx_cmd = self.vy_cmd = self.yaw_cmd = 0.0
        self.vx_app = self.vy_app = self.yaw_app = 0.0
        self.speed_scale = 1.0

        # commanded body pose in the gait world frame
        self.body_xy = np.zeros(2)
        self.body_yaw = 0.0

        self.step_len = self.cfg.stride_mm
        self.step_h = self.cfg.step_height_mm
        self.body_height = self.cfg.body_height_mm
        self.stance_radius = self.cfg.stance_radius_mm

        nominal = self._nominal_leg_local()
        self.anchor_world = [self._nominal_world(i, nominal) for i in range(6)]
        self.p_cmd_leg = [self._world_to_leg(i, self.anchor_world[i]) for i in range(6)]
        self.swing_from = [None] * 6
        self.swing_to = [None] * 6
        self.prev_is_stance = [True] * 6
        self.kp_alpha = [1.0] * 6
        self.was_idle = True
        self.prev_body_height = self.body_height
        self.prev_stance_radius = self.stance_radius

        # graceful return-to-stance state machine (legacy LB behaviour)
        self.stance_adjust_mode = False
        self.return_ptr = 0
        self.return_active = None
        self.return_from = None
        self.return_to = None
        self.return_phase = 0.0

        # park (legacy RB: lower and settle; no motor zeroing in sim)
        self.park_mode = False
        self.park_done = False
        self.park_target_mm = PARK_BODY_HEIGHT_MM
        self.unparking = False

        self.theta = np.zeros(18)
        self._seed_theta()

    # ---------------- public command interface ----------------

    def set_velocity(self, vx: float, vy: float, yaw: float) -> None:
        """Normalized [-1, 1] body velocity command (legacy joystick frame)."""
        self.vx_cmd = _clamp(vx, -1.0, 1.0)
        self.vy_cmd = _clamp(vy, -1.0, 1.0)
        self.yaw_cmd = _clamp(yaw, -1.0, 1.0)
        if not self.park_mode:
            self.park_done = False

    def set_gait(self, name: str) -> bool:
        if name not in GAIT_BY_NAME or name == self.gait_name:
            return name in GAIT_BY_NAME
        self.gait = GAIT_BY_NAME[name]
        self.gait_name = name
        # switch cleanly: freeze feet and restart the phase clock
        self._freeze_feet_as_anchors()
        self.u_global = 0.0
        return True

    def adjust(self, stride=0.0, step_h=0.0, body_h=0.0, radius=0.0) -> None:
        self.step_len = _clamp(self.step_len + stride,
                               ENV.stride_length_min_mm, ENV.stride_length_max_mm)
        self.step_h = _clamp(self.step_h + step_h,
                             ENV.step_height_min_mm, ENV.step_height_max_mm)
        if not self.park_mode:
            self.body_height = _clamp(self.body_height + body_h,
                                      self.cfg.body_height_min_mm, self.cfg.body_height_max_mm)
            self.stance_radius = _clamp(self.stance_radius + radius,
                                        self.cfg.stance_radius_min_mm, self.cfg.stance_radius_max_mm)

    def stop(self) -> None:
        """Space bar: zero commands -> graceful stand via the return sequence."""
        self.park_mode = False
        self.park_done = False
        self._start_return_to_stance()

    def park(self) -> None:
        self.vx_cmd = self.vy_cmd = self.yaw_cmd = 0.0
        self.vx_app = self.vy_app = self.yaw_app = 0.0
        self.stance_adjust_mode = False
        self._freeze_feet_as_anchors()
        self.park_mode = True
        self.park_done = False
        self.park_target_mm = PARK_BODY_HEIGHT_MM
        self.unparking = False

    def unpark(self) -> None:
        if self.park_mode:
            self.park_done = False
            self.park_target_mm = self.cfg.body_height_mm
            self.unparking = True

    # ---------------- frames ----------------

    def _nominal_leg_local(self) -> list[np.ndarray]:
        x = DEFAULT_LEG_GEOMETRY.hip_horizontal_offset_mm + self.stance_radius
        return [np.array([x, 0.0, -self.body_height]) for _ in range(6)]

    def _mount_angle(self, i: int) -> float:
        return self.body_yaw + LEG_PHI[i]

    def _hip_world(self, i: int) -> np.ndarray:
        ang = self._mount_angle(i)
        return np.array([
            self.body_xy[0] + BODY_R_MM * math.cos(ang),
            self.body_xy[1] + BODY_R_MM * math.sin(ang),
            0.0,
        ])

    def _leg_to_world(self, i: int, p_leg: np.ndarray) -> np.ndarray:
        return self._hip_world(i) + _rz(self._mount_angle(i)) @ p_leg

    def _world_to_leg(self, i: int, p_world: np.ndarray) -> np.ndarray:
        return _rz(-self._mount_angle(i)) @ (p_world - self._hip_world(i))

    def _nominal_world(self, i: int, nominal_local: list[np.ndarray]) -> np.ndarray:
        return self._leg_to_world(i, nominal_local[i])

    # ---------------- legacy anchor bookkeeping ----------------

    def _freeze_feet_as_anchors(self) -> None:
        for i in range(6):
            if self.swing_to[i] is not None and not self.prev_is_stance[i]:
                self.anchor_world[i] = self.swing_to[i].copy()
            else:
                self.anchor_world[i] = self._leg_to_world(i, self.p_cmd_leg[i]).copy()
            self.swing_from[i] = None
            self.swing_to[i] = None
            self.prev_is_stance[i] = True

    def _recompute_idle_height_only(self, nominal_local) -> None:
        for i in range(6):
            self.anchor_world[i][2] = self._nominal_world(i, nominal_local)[2]
            self.swing_from[i] = None
            self.swing_to[i] = None
            self.prev_is_stance[i] = True

    def _recompute_idle_full_unison(self, nominal_local) -> None:
        for i in range(6):
            self.anchor_world[i] = self._nominal_world(i, nominal_local).copy()
            self.swing_from[i] = None
            self.swing_to[i] = None
            self.prev_is_stance[i] = True

    def _start_return_to_stance(self) -> None:
        self.vx_cmd = self.vy_cmd = self.yaw_cmd = 0.0
        self.vx_app = self.vy_app = self.yaw_app = 0.0
        self._freeze_feet_as_anchors()
        self.stance_adjust_mode = True
        self.return_ptr = 0
        self.return_active = None
        self.return_phase = 0.0

    def _seed_theta(self) -> None:
        for i in range(6):
            self.theta[i * 3:i * 3 + 3] = self._ik(self.p_cmd_leg[i])

    def _ik(self, p_leg: np.ndarray) -> tuple[float, float, float]:
        x = max(p_leg[0], X_MIN_MM)
        th1, th2, th3 = inverse_kinematics_from_foot_position(
            Point3(x_mm=x, y_mm=p_leg[1], z_mm=p_leg[2])
        )
        return apply_legacy_motor_windows(th1, th2, th3)

    def _update_kp_alphas(self, stance_mask, dt: float) -> None:
        g = self.cfg.gains
        for i in range(6):
            target = 1.0 if stance_mask[i] else 0.0
            rate = g.kp_rate_up if target > self.kp_alpha[i] else g.kp_rate_down
            self.kp_alpha[i] = _slew_exp(self.kp_alpha[i], target, rate, dt)

    # ---------------- main tick ----------------

    def update(self, dt: float) -> JointCommand:
        if dt <= 0.0 or dt > 0.2:
            dt = 0.005
        g = self.cfg.gains

        if self.park_mode:
            self._update_park(dt)
        elif self.stance_adjust_mode:
            self._update_return(dt)
        else:
            self._update_walk(dt)

        # IK -> theta (18-vector, Section 2.5 order)
        for i in range(6):
            self.theta[i * 3:i * 3 + 3] = self._ik(self.p_cmd_leg[i])

        kp = np.empty(18)
        kd = np.empty(18)
        tau = np.zeros(18)
        for i in range(6):
            a = self.kp_alpha[i]
            kp[i * 3 + 0] = g.kp_hip_yaw_swing + (g.kp_hip_yaw_stance - g.kp_hip_yaw_swing) * a
            kp[i * 3 + 1] = g.kp_hip_pitch_swing + (g.kp_hip_pitch_stance - g.kp_hip_pitch_swing) * a
            kp[i * 3 + 2] = g.kp_knee_swing + (g.kp_knee_stance - g.kp_knee_swing) * a
            kd[i * 3 + 0] = g.kd_hip_yaw
            kd[i * 3 + 1] = g.kd_hip_pitch
            kd[i * 3 + 2] = g.kd_knee
            # legacy manual hip-pitch support torque, as constant feedforward.
            # sign: positive theta2 torque pushes the foot down (supports body)
            tau[i * 3 + 1] = _clamp(g.hip_pitch_support_tau, 0.0,
                                    ENV.hip_pitch_support_torque_max_nm) * a
        return JointCommand(self.theta.copy(), np.zeros(18), kp, kd, tau)

    # ---------------- modes ----------------

    def _update_park(self, dt: float) -> None:
        step = PARK_LOWER_RATE * dt
        if self.body_height > self.park_target_mm:
            self.body_height = max(self.body_height - step, self.park_target_mm)
        else:
            self.body_height = min(self.body_height + step, self.park_target_mm)
        nominal = self._nominal_leg_local()
        self._recompute_idle_height_only(nominal)
        for i in range(6):
            self.p_cmd_leg[i] = self._world_to_leg(i, self.anchor_world[i])
        self._update_kp_alphas([True] * 6, dt)
        if abs(self.body_height - self.park_target_mm) <= 1e-6:
            if self.unparking:
                # back at ride height: square the feet up and resume normal ops
                self.park_mode = False
                self.unparking = False
                self._start_return_to_stance()
            else:
                self.park_done = True

    def _update_return(self, dt: float) -> None:
        nominal = self._nominal_leg_local()
        if self.return_active is None:
            while self.return_ptr < len(RETURN_ORDER):
                leg = RETURN_ORDER[self.return_ptr]
                nominal_w = self._nominal_world(leg, nominal)
                cur = self.anchor_world[leg]
                dist = np.linalg.norm((nominal_w - cur)[:2]) + abs(nominal_w[2] - cur[2])
                if dist < 5.0:  # mm; close enough, skip the step
                    self.anchor_world[leg] = nominal_w.copy()
                    self.return_ptr += 1
                    continue
                self.return_active = leg
                self.return_from = cur.copy()
                self.return_to = nominal_w.copy()
                self.return_phase = 0.0
                break
            else:
                self.stance_adjust_mode = False
                self._freeze_feet_as_anchors()
                self._update_kp_alphas([True] * 6, dt)
                return

        leg = self.return_active
        self.return_phase += dt / RETURN_CYCLE_TIME
        s = _clamp(self.return_phase, 0.0, 1.0)
        target = (1.0 - s) * self.return_from + s * self.return_to
        target = target.copy()
        target[2] += self.step_h * math.sin(math.pi * s)

        stance_mask = [True] * 6
        stance_mask[leg] = False
        for i in range(6):
            src = target if i == leg else self.anchor_world[i]
            self.p_cmd_leg[i] = self._world_to_leg(i, src)
        self._update_kp_alphas(stance_mask, dt)

        if self.return_phase >= 1.0:
            self.anchor_world[leg] = self.return_to.copy()
            self.return_active = None
            self.return_ptr += 1

    def _update_walk(self, dt: float) -> None:
        self.vx_app = _slew_exp(self.vx_app, self.vx_cmd, CMD_SLEW_RATE, dt)
        self.vy_app = _slew_exp(self.vy_app, self.vy_cmd, CMD_SLEW_RATE, dt)
        self.yaw_app = _slew_exp(self.yaw_app, self.yaw_cmd, CMD_SLEW_RATE, dt)

        trans_speed = math.hypot(self.vx_app, self.vy_app)
        v_world = _rz(self.body_yaw) @ np.array([self.vx_app, self.vy_app, 0.0])
        if trans_speed > 1e-9:
            dir_world = v_world[:2] / trans_speed
        else:
            dir_world = np.zeros(2)

        nominal = self._nominal_leg_local()
        idle = (trans_speed < TRANS_SPEED_MIN_FOR_STEPPING
                and abs(self.yaw_app) < YAW_MIN_FOR_STEPPING)

        height_changed = abs(self.body_height - self.prev_body_height) > 1e-12
        radius_changed = abs(self.stance_radius - self.prev_stance_radius) > 1e-12

        if idle:
            self.cycle_time = self.cfg.cycle_slow_s
            self.speed_scale = 1.0
            if not self.was_idle:
                self._freeze_feet_as_anchors()
                self.u_global = 0.0
            else:
                if height_changed:
                    self._recompute_idle_height_only(nominal)
                if radius_changed:
                    self._recompute_idle_full_unison(nominal)
        else:
            motion = max(trans_speed, abs(self.yaw_app))
            lo = min(TRANS_SPEED_MIN_FOR_STEPPING, YAW_MIN_FOR_STEPPING)
            s = _clamp((motion - lo) / (1.0 - lo), 0.0, 1.0)
            cycle_nominal = (1.0 - s) * self.cfg.cycle_slow_s + s * self.cfg.cycle_fast_s

            swing_dist = math.hypot(self.step_len, 2.0 * self.step_h)
            swing_time_min = swing_dist / max(1e-6, self.cfg.foot_speed_max_mm_s)
            cycle_min = swing_time_min / max(1e-6, 1.0 - self.gait.duty_factor)
            self.cycle_time = max(cycle_nominal, cycle_min)
            self.speed_scale = _clamp(cycle_nominal / self.cycle_time, 0.0, 1.0)
            self.u_global = (self.u_global + dt / max(1e-6, self.cycle_time)) % 1.0

        self.was_idle = idle
        self.prev_body_height = self.body_height
        self.prev_stance_radius = self.stance_radius

        if not idle:
            v_mm_s = self.cfg.body_speed_max_mm_s * _clamp(trans_speed, 0.0, 1.0) * self.speed_scale
            yaw_rate = self.cfg.body_yaw_rate_max_rad_s * _clamp(self.yaw_app, -1.0, 1.0) * self.speed_scale
            self.body_xy += dir_world * v_mm_s * dt
            self.body_yaw = wrap_pi(self.body_yaw + yaw_rate * dt)

        stance_mask = [True] * 6
        for i in range(6):
            if idle:
                is_stance, s_phase = True, 0.0
                self.prev_is_stance[i] = True
            else:
                _, is_stance, s_phase = phase_for_leg(self.gait, i, self.u_global)

                if self.prev_is_stance[i] and not is_stance:
                    # lift-off: plan the touchdown point
                    self.swing_from[i] = self.anchor_world[i].copy()
                    nominal_w = self._nominal_world(i, nominal)
                    trans_offset = np.array([0.5 * self.step_len * dir_world[0],
                                             0.5 * self.step_len * dir_world[1], 0.0])
                    rel = nominal_w - np.array([self.body_xy[0], self.body_xy[1], 0.0])
                    tangent = np.array([-rel[1], rel[0], 0.0])
                    norm = np.linalg.norm(tangent[:2])
                    tangent = tangent / norm if norm > 1e-9 else tangent * 0.0
                    yaw_offset = tangent * (YAW_STRIDE_SCALE * self.step_len * self.yaw_app)
                    self.swing_to[i] = nominal_w + trans_offset + yaw_offset
                if not self.prev_is_stance[i] and is_stance:
                    if self.swing_to[i] is not None:
                        self.anchor_world[i] = self.swing_to[i].copy()
                self.prev_is_stance[i] = is_stance

            stance_mask[i] = is_stance
            if is_stance:
                target = self.anchor_world[i].copy()
            else:
                p_a = self.swing_from[i] if self.swing_from[i] is not None else self.anchor_world[i]
                p_b = self.swing_to[i] if self.swing_to[i] is not None else p_a
                target = (1.0 - s_phase) * p_a + s_phase * p_b
                target = target.copy()
                target[2] += self.step_h * math.sin(math.pi * s_phase)

            self.p_cmd_leg[i] = self._world_to_leg(i, target)

        self._update_kp_alphas(stance_mask, dt)

    # ---------------- introspection for tests / HUD ----------------

    def stance_mask(self) -> list[bool]:
        if self.was_idle or self.stance_adjust_mode or self.park_mode:
            mask = [True] * 6
            if self.stance_adjust_mode and self.return_active is not None:
                mask[self.return_active] = False
            return mask
        return [phase_for_leg(self.gait, i, self.u_global)[1] for i in range(6)]

    def status(self) -> dict:
        return {
            "gait": self.gait_name,
            "cycle_time": self.cycle_time,
            "vx": self.vx_app, "vy": self.vy_app, "yaw": self.yaw_app,
            "stride_mm": self.step_len, "step_h_mm": self.step_h,
            "body_h_mm": self.body_height, "radius_mm": self.stance_radius,
            "park": self.park_mode, "returning": self.stance_adjust_mode,
        }
