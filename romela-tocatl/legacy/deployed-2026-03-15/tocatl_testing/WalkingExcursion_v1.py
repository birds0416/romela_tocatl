#!/usr/bin/env python3
# TOCATL WALKING CONTROLLER
# - Streamer
# - Keyboard E-Stop
# - Back-button soft E-Stop
# - Xbox controller drives walking gait on ALL legs using simulator gait logic
#
# UPDATED BEHAVIOR:
#   SOFT E-STOP:
#     - Back button OR keyboard e/q
#     - applies damping for longer
#     - stops all motors
#     - stays inside the script
#     - after you physically reposition the robot, press START to rerun startup
#
#   SUPER E-STOP:
#     - press BOTH stick buttons together (L3 + R3)
#     - applies damping
#     - stops motors
#     - shuts down CAN / joy / process and exits, like the old behavior
#
#   GAIT CYCLING:
#     - RB cycles gait: wave -> ripple -> tripod -> wave
#     - when RB is pressed:
#         1) robot stops commanded motion
#         2) waits 3 seconds
#         3) does a slower graceful return
#         4) then applies the next gait
#
#   LB:
#     - manual graceful return to stance
#
# Controller mapping:
#   Axes:
#     0 = Left Joy X   -> vy
#     1 = Left Joy Y   -> vx
#     2 = Left Trigger -> decrease hip-pitch support torque rate
#     3 = Right Joy X  -> yaw
#     5 = Right Trigger -> increase hip-pitch support torque rate
#     6 = DPAD_X       -> stride length
#     7 = DPAD_Y       -> step height
#
#   Buttons:
#     0 = A
#     1 = B
#     2 = X
#     3 = Y
#     4 = LB    -> graceful return
#     5 = RB    -> cycle gait
#     6 = Back  -> soft e-stop
#     7 = Start -> restart after soft e-stop
#     9 = L3
#    10 = R3
#
# NOTE:
#   If your controller maps L3/R3 differently, change BTN_L3 / BTN_R3 below.

import can, time, subprocess, threading
import sys, os, select, signal, shlex, math
import numpy as np
from dataclasses import dataclass

from CyberGearDriver import CyberGearMotor, RunMode, CyberMotorMessage

# -------------------------------------------------------
# ROS auto-bootstrap
# -------------------------------------------------------
try:
    import rclpy  # noqa
except ModuleNotFoundError:
    script = shlex.quote(os.path.abspath(__file__))
    py = shlex.quote(sys.executable)
    cmd = f"source /opt/ros/jazzy/setup.bash && exec {py} {script}"
    os.execvp("bash", ["bash", "-lc", cmd])

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy


# ======================================================
# joy_linux_node bring-up
# ======================================================
def start_joy_node():
    print("[JOYDRV] Starting joy_linux_node...")
    return subprocess.Popen(
        ['bash', '-lc', 'source /opt/ros/jazzy/setup.bash && exec ros2 run joy_linux joy_linux_node'],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        preexec_fn=os.setsid
    )

def stop_joy_node(proc):
    print("[JOYDRV] Stopping joy_linux_node...")
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
    except Exception:
        pass


# ======================================================
# Helpers
# ======================================================
def wrapPi(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))

def deg2rad(x: float) -> float:
    return x * math.pi / 180.0

def rad2deg(x: float) -> float:
    return x * 180.0 / math.pi

def clamp(x, lo, hi):
    return lo if x < lo else (hi if x > hi else x)

def slew_exp(cur: float, target: float, rate: float, dt: float) -> float:
    alpha = 1.0 - math.exp(-rate * dt)
    return cur + (target - cur) * alpha

def deadband(x, db):
    return 0.0 if abs(x) < db else x

def fmt(x, p=3):
    if x is None:
        return "   None"
    return f"{x:+.{p}f}"

def near_limit(x, lo, hi, margin=0.08):
    if x is None:
        return False
    return (x <= lo + margin) or (x >= hi - margin)

def trigger_to_01(x, db=0.08):
    """
    Robust trigger normalization:
      - Handles common ROS/Linux trigger ranges
      - Released should map near 0
      - Fully pressed should map near 1
      - Small residual noise maps to 0
    """
    a = clamp((1.0 - x) * 0.5, 0.0, 1.0)
    b = clamp(x, 0.0, 1.0)
    v = a if a < b else b

    if v < db:
        return 0.0
    return (v - db) / (1.0 - db)

def Ry(th: float) -> np.ndarray:
    c = math.cos(th)
    s = math.sin(th)
    return np.array([[ c, 0.0,  s],
                     [0.0, 1.0, 0.0],
                     [-s, 0.0,  c]], dtype=float)

def Rz(th: float) -> np.ndarray:
    c = math.cos(th)
    s = math.sin(th)
    return np.array([[ c, -s, 0.0],
                     [ s,  c, 0.0],
                     [0.0, 0.0, 1.0]], dtype=float)


# ======================================================
# Gait
# ======================================================
@dataclass
class Gait:
    cycle_time: float
    duty: float
    phase_offsets: list

class PhaseGait:
    def __init__(self, gait: Gait):
        self.gait = gait
        self.u_global = 0.0

    def reset_time(self):
        self.u_global = 0.0

    def advance(self, dt: float):
        self.u_global = (self.u_global + dt / max(1e-6, self.gait.cycle_time)) % 1.0

    def phase(self, leg_i: int):
        g = self.gait
        u = (self.u_global + g.phase_offsets[leg_i]) % 1.0
        if u < g.duty:
            is_stance = True
            s = u / max(1e-6, g.duty)
        else:
            is_stance = False
            s = (u - g.duty) / max(1e-6, (1.0 - g.duty))
        return u, is_stance, s

# gait selector:
#   "wave"   -> one leg at a time
#   "ripple" -> opposing pairs
#   "tripod" -> tripod gait
GAIT_MODE = "wave"
GAIT_SEQUENCE = ["wave", "ripple", "tripod"]

WAVE_STAGGERED = Gait(
    cycle_time=1.5,
    duty=5.0 / 6.0,
    phase_offsets=[
        5/6,  # leg 1
        1/3,  # leg 2
        2/3,  # leg 3
        1/6,  # leg 4
        1/2,  # leg 5
        0.0,  # leg 6
    ]
)

RIPPLE = Gait(
    cycle_time=1.5,
    duty=2.0 / 3.0,
    phase_offsets=[
        0.0,   # leg 1
        1/3,   # leg 2
        2/3,   # leg 3
        0.0,   # leg 4
        1/3,   # leg 5
        2/3,   # leg 6
    ]
)

TRIPOD = Gait(
    cycle_time=1.5,
    duty=0.5,
    phase_offsets=[
        0.0,   # leg 1
        0.5,   # leg 2
        0.0,   # leg 3
        0.5,   # leg 4
        0.0,   # leg 5
        0.5,   # leg 6
    ]
)

def build_selected_gait(mode: str) -> Gait:
    mode = mode.lower().strip()
    if mode == "wave":
        return Gait(
            cycle_time=WAVE_STAGGERED.cycle_time,
            duty=WAVE_STAGGERED.duty,
            phase_offsets=list(WAVE_STAGGERED.phase_offsets)
        )
    if mode == "ripple":
        return Gait(
            cycle_time=RIPPLE.cycle_time,
            duty=RIPPLE.duty,
            phase_offsets=list(RIPPLE.phase_offsets)
        )
    if mode == "tripod":
        return Gait(
            cycle_time=TRIPOD.cycle_time,
            duty=TRIPOD.duty,
            phase_offsets=list(TRIPOD.phase_offsets)
        )
    raise ValueError(f"Unknown GAIT_MODE '{mode}'. Use 'wave', 'ripple', or 'tripod'.")

RETURN_ORDER = [0, 2, 4, 1, 3, 5]


# ======================================================
# Geometry / offsets / limits
# ======================================================
d1 = 114.25
d2 = 45.5561
d3 = 330.3455
d4 = 458.0

BODY_DIAMETER = 600.0
BODY_R = BODY_DIAMETER / 2.0
BODY_Z = 0.0

LEG_PHI0 = deg2rad(30.0)
LEG_PHI = [LEG_PHI0 + deg2rad(a) for a in (0, 60, 120, 180, 240, 300)]

X_MIN_MM = 40.0

OFF1 = 0.0
OFF2 = deg2rad(45.44 - 180.0)
OFF3 = -OFF2 + deg2rad((45.44 - 11.18))

M1_MIN = -1.0
M1_MAX =  1.0

M2_MIN =  0.0
M2_MAX =  4.9

REL_MIN = -5.0
REL_MAX =  0.0


# ======================================================
# Walking / stance params
# ======================================================
JOY_DEADBAND = 0.08

CMD_SLEW_RATE = 4.0

STEP_H_RATE   = 100.0
STEP_LEN_RATE = 100.0
HEIGHT_RATE   = 100.0
RADIUS_RATE   = 100.0

STEP_H_MIN = 0.0
STEP_H_MAX = 400.0
STEP_LEN_MIN = 20.0
STEP_LEN_MAX = 400.0

BODY_HEIGHT_MIN = -100.0
BODY_HEIGHT_MAX =  500.0
RADIUS_MIN = 10.0
RADIUS_MAX = 450.0

FOOT_SPEED_MAX = 400.0

CYCLE_FAST = 1.2
CYCLE_SLOW = 4.5
TRANS_SPEED_MIN_FOR_STEPPING = 0.10
YAW_MIN_FOR_STEPPING = 0.08

BODY_SPEED_MAX_MM_S = 140.0
BODY_YAW_RATE_MAX_RAD_S = deg2rad(35.0)
YAW_STRIDE_SCALE = 0.2

# Slowed down from previous version to make graceful return actually graceful
RETURN_CYCLE_TIME = 1.35

# manual hip-pitch torque behavior
HIP_PITCH_TAU_MAX = 9.25
HIP_PITCH_TAU_RATE = 9.25

# phase-based Kp scheduling
KP_SCHEDULE_RATE_UP = 1.0
KP_SCHEDULE_RATE_DOWN = 6.0

# gait cycling behavior
GAIT_SWITCH_SETTLE_TIME = 3.0

# estop damping behavior
ESTOP_DAMP_KD = 8.0
ESTOP_DAMP_TIME = 1.25
ESTOP_DAMP_STREAM_DT = 0.01


# ======================================================
# Per-joint-group gains
# ======================================================
KP_HIPYAW_STANCE = 100.0
KP_HIPYAW_SWING  = 10.0
KD_HIPYAW        = 3.0

KP_HIPPITCH_STANCE = 100.0
KP_HIPPITCH_SWING  = 10.0
KD_HIPPITCH        = 3.0

KP_KNEE_STANCE = 100.0
KP_KNEE_SWING  = 10.0
KD_KNEE        = 3.0

JOY_VEL = 0.0
JOY_TAU = 0.0


# ======================================================
# Controller mapping
# ======================================================
AX_LX   = 0
AX_LY   = 1
AX_LT   = 2
AX_RX   = 3
AX_RT   = 5
AX_DX   = 6
AX_DY   = 7

BTN_A   = 0
BTN_B   = 1
BTN_X   = 2
BTN_Y   = 3
BTN_LB  = 4
BTN_RB  = 5
BTN_BACK_ESTOP = 6
BTN_START = 7
BTN_L3 = 9
BTN_R3 = 10


# ======================================================
# FK / IK
# ======================================================
def fk_from_theta(theta1: float, theta2: float, theta3: float):
    ct = math.cos(theta1)
    st = math.sin(theta1)
    Rz1 = np.array([[ ct, -st, 0.0],
                    [ st,  ct, 0.0],
                    [0.0, 0.0, 1.0]], dtype=float)

    base = np.array([[d2], [0.0], [d1]], dtype=float)
    u    = np.array([[d3], [0.0], [0.0]], dtype=float)
    l    = np.array([[d4], [0.0], [0.0]], dtype=float)

    knee = base + Rz1 @ (Ry(theta2)          @ u)
    foot = knee + Rz1 @ (Ry(theta2 + theta3) @ l)
    return base, knee, foot

def ik_theta_from_xyz(Xd: float, Yd: float, Zd: float):
    base_x, base_y, base_z = d2, 0.0, d1

    X = Xd - base_x
    Y = Yd - base_y
    Z = Zd - base_z

    theta1 = math.atan2(Y, X)
    r1 = math.sqrt(X * X + Y * Y)

    Xe = r1
    Ze = Z
    R  = math.sqrt(Xe * Xe + Ze * Ze)

    C3 = (R * R - d3 * d3 - d4 * d4) / (2.0 * d3 * d4)
    C3 = clamp(C3, -1.0, 1.0)

    theta3 = math.atan2(math.sqrt(max(0.0, 1.0 - C3 * C3)), C3)

    phi   = math.atan2(-Ze, Xe)
    gamma = math.atan2(d4 * math.sin(theta3), d3 + d4 * math.cos(theta3))
    theta2 = phi - gamma

    return theta1, theta2, theta3


# ======================================================
# θ <-> motor mapping
# ======================================================
def theta_to_motor(theta1, theta2, theta3, off1, off2, off3_rel):
    off3_abs = off2 + off3_rel

    m1 = wrapPi(theta1 - off1)

    a2 = theta2
    a3 = theta2 + theta3

    m2 = a2 - off2
    m3 = -(a3 - off3_abs)
    return m1, m2, m3

def motor_to_theta(m1, m2, m3, off1, off2, off3_rel):
    off3_abs = off2 + off3_rel

    theta1 = wrapPi(m1 + off1)

    a2 = m2 + off2
    a3 = -m3 + off3_abs

    theta2 = a2
    theta3 = a3 - a2
    return theta1, theta2, theta3

def apply_motor_limits(theta1, theta2, theta3, off1, off2, off3_rel):
    m1, m2, m3 = theta_to_motor(theta1, theta2, theta3, off1, off2, off3_rel)

    m1 = clamp(m1, M1_MIN, M1_MAX)
    m2 = clamp(m2, M2_MIN, M2_MAX)

    rel = m3 - m2
    rel = clamp(rel, REL_MIN, REL_MAX)
    m3 = m2 + rel

    return motor_to_theta(m1, m2, m3, off1, off2, off3_rel)


# ======================================================
# World / leg transforms
# ======================================================
def leg_mount_angle_world(i: int, body_yaw: float) -> float:
    return body_yaw + LEG_PHI[i]

def hip_pos_world(i: int, body_xy: np.ndarray, body_yaw: float) -> np.ndarray:
    ang = leg_mount_angle_world(i, body_yaw)
    return np.array([
        body_xy[0] + BODY_R * math.cos(ang),
        body_xy[1] + BODY_R * math.sin(ang),
        BODY_Z
    ], dtype=float)

def leg_to_world(i: int, p_leg: np.ndarray, body_xy: np.ndarray, body_yaw: float) -> np.ndarray:
    ang = leg_mount_angle_world(i, body_yaw)
    return hip_pos_world(i, body_xy, body_yaw) + (Rz(ang) @ p_leg)

def world_to_leg(i: int, p_world: np.ndarray, body_xy: np.ndarray, body_yaw: float) -> np.ndarray:
    ang = leg_mount_angle_world(i, body_yaw)
    return (Rz(-ang) @ (p_world - hip_pos_world(i, body_xy, body_yaw)))

def tip_from_theta(theta1, theta2, theta3):
    _, _, foot = fk_from_theta(theta1, theta2, theta3)
    return np.array([float(foot[0, 0]), float(foot[1, 0]), float(foot[2, 0])], dtype=float)


# ======================================================
# CAN bring-up
# ======================================================
def setup_can0():
    print("\nSetting up Interface for Can 0...\n")
    subprocess.run(["sudo", "ip", "link", "set", "can0", "down"], stderr=subprocess.DEVNULL)
    subprocess.run(["sudo", "ip", "link", "set", "can0", "type", "can", "bitrate", "1000000"], check=True)
    subprocess.run(["sudo", "ip", "link", "set", "can0", "up"], check=True)

def setup_can1():
    print("\nSetting up Interface for Can 1...\n")
    subprocess.run(["sudo", "ip", "link", "set", "can1", "down"], stderr=subprocess.DEVNULL)
    subprocess.run(["sudo", "ip", "link", "set", "can1", "type", "can", "bitrate", "1000000"], check=True)
    subprocess.run(["sudo", "ip", "link", "set", "can1", "up"], check=True)

setup_can0()
setup_can1()

bus0 = can.interface.Bus(interface='socketcan', channel='can0', bitrate=1000000)
bus1 = can.interface.Bus(interface='socketcan', channel='can1', bitrate=1000000)

def send_message0(message: CyberMotorMessage):
    bus0.send(
        can.Message(
            arbitration_id=message.arbitration_id,
            data=message.data,
            is_extended_id=message.is_extended_id,
        ),
        timeout=0.01,
    )

def send_message1(message: CyberMotorMessage):
    bus1.send(
        can.Message(
            arbitration_id=message.arbitration_id,
            data=message.data,
            is_extended_id=message.is_extended_id,
        ),
        timeout=0.01,
    )

print("\nCAN Setup Complete!\n")


# ======================================================
# Motors
# ======================================================
leg1hipYaw   = CyberGearMotor(motor_id=11, send_message=send_message1)
leg1hipPitch = CyberGearMotor(motor_id=12, send_message=send_message1)
leg1knee     = CyberGearMotor(motor_id=13, send_message=send_message1)

leg2hipYaw   = CyberGearMotor(motor_id=21, send_message=send_message0)
leg2hipPitch = CyberGearMotor(motor_id=22, send_message=send_message0)
leg2knee     = CyberGearMotor(motor_id=23, send_message=send_message0)

leg3hipYaw   = CyberGearMotor(motor_id=31, send_message=send_message0)
leg3hipPitch = CyberGearMotor(motor_id=32, send_message=send_message0)
leg3knee     = CyberGearMotor(motor_id=33, send_message=send_message0)

leg4hipYaw   = CyberGearMotor(motor_id=41, send_message=send_message0)
leg4hipPitch = CyberGearMotor(motor_id=42, send_message=send_message0)
leg4knee     = CyberGearMotor(motor_id=43, send_message=send_message0)

leg5hipYaw   = CyberGearMotor(motor_id=51, send_message=send_message1)
leg5hipPitch = CyberGearMotor(motor_id=52, send_message=send_message1)
leg5knee     = CyberGearMotor(motor_id=53, send_message=send_message1)

leg6hipYaw   = CyberGearMotor(motor_id=61, send_message=send_message1)
leg6hipPitch = CyberGearMotor(motor_id=62, send_message=send_message1)
leg6knee     = CyberGearMotor(motor_id=63, send_message=send_message1)

allMotors = [
    leg1hipYaw, leg1hipPitch, leg1knee,
    leg2hipYaw, leg2hipPitch, leg2knee,
    leg3hipYaw, leg3hipPitch, leg3knee,
    leg4hipYaw, leg4hipPitch, leg4knee,
    leg5hipYaw, leg5hipPitch, leg5knee,
    leg6hipYaw, leg6hipPitch, leg6knee
]

rightSideMotors = [
    leg1hipYaw, leg1hipPitch, leg1knee,
    leg5hipYaw, leg5hipPitch, leg5knee,
    leg6hipYaw, leg6hipPitch, leg6knee
]
leftSideMotors = [
    leg2hipYaw, leg2hipPitch, leg2knee,
    leg3hipYaw, leg3hipPitch, leg3knee,
    leg4hipYaw, leg4hipPitch, leg4knee
]

legs = [
    (leg1hipYaw, leg1hipPitch, leg1knee),
    (leg2hipYaw, leg2hipPitch, leg2knee),
    (leg3hipYaw, leg3hipPitch, leg3knee),
    (leg4hipYaw, leg4hipPitch, leg4knee),
    (leg5hipYaw, leg5hipPitch, leg5knee),
    (leg6hipYaw, leg6hipPitch, leg6knee),
]


# ======================================================
# RX dispatch
# ======================================================
def dispatch_message0(msg):
    for m in leftSideMotors:
        m.message_received(msg)

def dispatch_message1(msg):
    for m in rightSideMotors:
        m.message_received(msg)

notifier0 = can.Notifier(bus0, [dispatch_message0])
notifier1 = can.Notifier(bus1, [dispatch_message1])


# ======================================================
# Streamer
# ======================================================
cmd_lock = threading.Lock()
desired = {}
STREAM_ON = True
STREAM_HZ = 200

def set_cmd(motor, position, velocity=0.0, torque=0.0, kp=0.0, kd=0.0):
    with cmd_lock:
        desired[motor.motor_id] = (position, velocity, torque, kp, kd)

def clear_all_cmds():
    with cmd_lock:
        desired.clear()

def streamer_loop():
    global STREAM_ON
    period = 1.0 / STREAM_HZ
    next_t = time.perf_counter()

    while STREAM_ON:
        with cmd_lock:
            snap = dict(desired)

        for m in allMotors:
            c = snap.get(m.motor_id)
            if c is None:
                continue
            try:
                pos, vel, tau, kp, kd = c
                m.control(position=pos, velocity=vel, torque=tau, kp=kp, kd=kd)
            except Exception:
                pass

        next_t += period
        dt = next_t - time.perf_counter()
        if dt > 0:
            time.sleep(dt)
        else:
            next_t = time.perf_counter()

def stop_streamer():
    global STREAM_ON
    STREAM_ON = False

threading.Thread(target=streamer_loop, daemon=True).start()


# ======================================================
# Snapshot helpers
# ======================================================
def snapshot_state(timeout_s=0.08):
    for m in allMotors:
        try:
            m.request_motor_state()
        except Exception:
            pass
    time.sleep(timeout_s)

    out = {}
    for m in allMotors:
        st = m.state
        out[m.motor_id] = {
            "pos": st.get("position"),
            "vel": st.get("velocity"),
            "tau": st.get("torque"),
        }
    return out

def snapshot_positions(timeout_s=0.08):
    st = snapshot_state(timeout_s)
    pos = {}
    for m in allMotors:
        p = st[m.motor_id]["pos"]
        pos[m.motor_id] = 0.0 if p is None else p
    return pos


# ======================================================
# Global run-state control
# ======================================================
ESTOP = False
RESTARTING = False
RESTART_REQUEST = False
SHUTTING_DOWN = False
JOY_PROC = None
ACTIVE_NODE = None
STATE_LOCK = threading.Lock()


def request_restart():
    global RESTART_REQUEST
    with STATE_LOCK:
        if ESTOP and not RESTARTING and not SHUTTING_DOWN:
            RESTART_REQUEST = True
            print("\n[RESTART] START button pressed. Restart requested.\n")


# ======================================================
# E-STOP
# ======================================================
def apply_damping_and_stop(reason_label: str):
    print("\n\n==============================")
    print(f"!!! {reason_label} ACTIVATED !!!")
    print("==============================")
    print(f"[{reason_label}] Applying damping for {ESTOP_DAMP_TIME:.2f} s.\n")

    hold = snapshot_positions(timeout_s=0.05)

    t_end = time.monotonic() + ESTOP_DAMP_TIME
    while time.monotonic() < t_end:
        for m in allMotors:
            set_cmd(
                m,
                hold[m.motor_id],
                velocity=0.0,
                torque=0.0,
                kp=0.0,
                kd=ESTOP_DAMP_KD
            )
        time.sleep(ESTOP_DAMP_STREAM_DT)

    clear_all_cmds()

    for m in allMotors:
        try:
            m.stop()
        except Exception:
            pass

def estop(reason="E-STOP"):
    global ESTOP

    with STATE_LOCK:
        if ESTOP or RESTARTING or SHUTTING_DOWN:
            return
        ESTOP = True

    apply_damping_and_stop(reason)

    print("[ESTOP] Motors stopped.")
    print("[ESTOP] Put the robot back in its physical start pose.")
    print("[ESTOP] Then press START on the Xbox controller to rerun startup.\n")

def super_estop(reason="SUPER E-STOP"):
    global ESTOP, SHUTTING_DOWN

    with STATE_LOCK:
        if SHUTTING_DOWN or RESTARTING:
            return
        ESTOP = True
        SHUTTING_DOWN = True

    apply_damping_and_stop(reason)

    try:
        notifier0.stop()
    except Exception:
        pass
    try:
        notifier1.stop()
    except Exception:
        pass
    try:
        bus0.shutdown()
    except Exception:
        pass
    try:
        bus1.shutdown()
    except Exception:
        pass

    try:
        if JOY_PROC is not None:
            stop_joy_node(JOY_PROC)
    except Exception:
        pass

    print("[SUPER ESTOP] Motors stopped. CAN shutdown. Exiting now.")
    os._exit(1)

def keyboard_estop_listener():
    print("\nVirtual E-Stop ARMED: type 'e' + Enter (or 'q') at any time.\n")
    while True:
        r, _, _ = select.select([sys.stdin], [], [], 0.05)
        if r:
            key = sys.stdin.readline().strip().lower()
            if key in ("e", "q", "estop", "stop"):
                estop("KEYBOARD E-STOP")

threading.Thread(target=keyboard_estop_listener, daemon=True).start()


# ======================================================
# Startup / restart sequence
# ======================================================
def startup_sequence():
    global ESTOP, RESTARTING, RESTART_REQUEST

    with STATE_LOCK:
        RESTARTING = True
        RESTART_REQUEST = False

    print("\n[STARTUP] =====================================")
    print("[STARTUP] Running startup sequence...")
    print("[STARTUP] =====================================\n")

    clear_all_cmds()

    print("[STARTUP] Stopping ALL motors.")
    for m in allMotors:
        try:
            m.stop()
        except Exception:
            pass

    time.sleep(0.15)

    print("[STARTUP] Setting mode + enabling ALL motors.")
    for m in allMotors:
        try:
            m.mode(RunMode.OPERATION_CONTROL)
        except Exception:
            pass

    for m in allMotors:
        try:
            m.enable()
        except Exception:
            pass

    _ = snapshot_state(timeout_s=0.10)
    print("[STARTUP] Initial positions captured.")

    print("[STARTUP] Zeroing ALL motors.")
    for m in allMotors:
        try:
            m.set_zero_position()
        except Exception:
            pass

    time.sleep(0.75)

    hold = snapshot_positions(timeout_s=0.10)
    for m in allMotors:
        set_cmd(m, hold[m.motor_id], velocity=0.0, torque=0.0, kp=0.0, kd=6.0)

    if ACTIVE_NODE is not None:
        try:
            ACTIVE_NODE.reset_controller_state_from_robot()
        except Exception as e:
            print(f"[STARTUP] Warning: node state reset failed: {e}")

    with STATE_LOCK:
        ESTOP = False
        RESTARTING = False

    print("\n=== READY: WALK MODE ===")
    print(f"Gait mode = {GAIT_MODE}")
    print("Left stick UD=vx, Left stick LR=vy, Right stick LR=yaw")
    print("LT/RT = decrease/increase hip-pitch support torque")
    print("A/B=body height, X/Y=leg radius, D-pad UD=step height, D-pad LR=stride length")
    print("LB=graceful return")
    print("RB=cycle gait")
    print("Back = soft E-STOP")
    print("Start = restart after soft E-STOP")
    print("L3 + R3 = SUPER E-STOP\n")


# ======================================================
# Joy-based walking controller
# ======================================================
class HexapodWalkJoy(Node):
    def __init__(self):
        super().__init__('tocatl_walk_joy')
        self.sub = self.create_subscription(Joy, '/joy', self.cb, 10)
        self._last_t = time.monotonic()
        self._back_prev = 0
        self._lb_prev = 0
        self._rb_prev = 0
        self._start_prev = 0
        self._l3_prev = 0
        self._r3_prev = 0

        self.gait_mode = GAIT_MODE
        self.reset_controller_state_from_robot()

    def next_gait_mode(self):
        idx = GAIT_SEQUENCE.index(self.gait_mode)
        return GAIT_SEQUENCE[(idx + 1) % len(GAIT_SEQUENCE)]

    def reset_controller_state_from_robot(self):
        self.gait = PhaseGait(build_selected_gait(self.gait_mode))

        self.theta1_act = [wrapPi(OFF1) for _ in range(6)]
        self.theta2_act = [OFF2 for _ in range(6)]
        self.theta3_act = [OFF3 for _ in range(6)]

        self.theta1_des = self.theta1_act.copy()
        self.theta2_des = self.theta2_act.copy()
        self.theta3_des = self.theta3_act.copy()
        self.theta1_prev = [wrapPi(OFF1) for _ in range(6)]

        self.body_xy = np.array([0.0, 0.0], dtype=float)
        self.body_yaw = 0.0

        self.p_home_leg = [tip_from_theta(self.theta1_act[i], self.theta2_act[i], self.theta3_act[i]) for i in range(6)]
        self.nominal_base_leg = [p.copy() for p in self.p_home_leg]
        self.p_cmd_leg = [p.copy() for p in self.p_home_leg]

        self.anchor_world = [leg_to_world(i, self.p_cmd_leg[i], self.body_xy, self.body_yaw).copy() for i in range(6)]
        self.swing_from_world = [None for _ in range(6)]
        self.swing_to_world = [None for _ in range(6)]
        self.prev_is_stance = [True for _ in range(6)]

        self.vx_cmd = 0.0
        self.vy_cmd = 0.0
        self.yaw_cmd = 0.0

        self.vx_app = 0.0
        self.vy_app = 0.0
        self.yaw_app = 0.0

        self.step_len = 175.0
        self.step_h = 215.0
        self.body_height = 0.0
        self.radius_offset = 10.0

        self.hip_pitch_tau_cmd = 0.0
        self.kp_phase_alpha = [1.0 for _ in range(6)]

        self.stance_adjust_mode = False
        self.return_leg_ptr = 0
        self.return_leg_active = None
        self.return_from_world = None
        self.return_to_world = None
        self.return_phase = 0.0

        # Gait switching mode
        self.gait_switch_mode = False
        self.gait_switch_stage = "idle"     # idle / settle / return
        self.gait_switch_timer = 0.0
        self.pending_gait_mode = self.gait_mode

        self.was_idle = True
        self.prev_body_height = self.body_height
        self.prev_radius_offset = self.radius_offset

        self.last_print = 0.0
        self.err_leg1 = None
        self.speed_scale = 1.0
        self.leg6_debug = {}

        st = snapshot_state(timeout_s=0.10)
        for i, leg in enumerate(legs):
            m1 = st[leg[0].motor_id]["pos"] or 0.0
            m2 = st[leg[1].motor_id]["pos"] or 0.0
            m3 = st[leg[2].motor_id]["pos"] or 0.0

            th1, th2, th3 = motor_to_theta(m1, m2, m3, OFF1, OFF2, OFF3)

            self.theta1_act[i] = th1
            self.theta2_act[i] = th2
            self.theta3_act[i] = th3
            self.theta1_des[i] = th1
            self.theta2_des[i] = th2
            self.theta3_des[i] = th3
            self.theta1_prev[i] = th1

            p0 = tip_from_theta(th1, th2, th3)
            self.p_home_leg[i] = p0.copy()
            self.nominal_base_leg[i] = p0.copy()
            self.p_cmd_leg[i] = p0.copy()

        nominal_leg = self.current_nominal_leg()
        for i in range(6):
            self.anchor_world[i] = self.nominal_world_for_leg(i, nominal_leg).copy()

        self._last_t = time.monotonic()
        self._back_prev = 0
        self._lb_prev = 0
        self._rb_prev = 0
        self._start_prev = 0
        self._l3_prev = 0
        self._r3_prev = 0

        print("\n\n[WALK] Ready.")
        print(f"[WALK] GAIT_MODE = {self.gait_mode}")
        print("Controls:")
        print("  Left stick UD  (axis 1) -> vx")
        print("  Left stick LR  (axis 0) -> vy")
        print("  Right stick LR (axis 3) -> yaw")
        print("  LT / RT        -> decrease / increase hip-pitch support torque")
        print("  A / B          -> body height down / up")
        print("  X / Y          -> leg radius in / out")
        print("  D-pad UD       -> step height down / up")
        print("  D-pad LR       -> stride length down / up")
        print("  LB             -> graceful return")
        print("  RB             -> cycle gait (stop 3s -> return -> switch gait)")
        print("  Back           -> soft E-STOP")
        print("  Start          -> restart after soft E-STOP")
        print("  L3 + R3        -> SUPER E-STOP")
        print("  Keyboard e/q   -> soft E-STOP\n")

    def current_nominal_leg(self):
        out = []
        for i in range(6):
            p0 = self.nominal_base_leg[i].copy()
            p0[0] += self.radius_offset
            p0[2] -= self.body_height
            out.append(p0)
        return out

    def nominal_world_for_leg(self, i: int, nominal_leg_local):
        hipW = hip_pos_world(i, self.body_xy, self.body_yaw)
        return hipW + (Rz(leg_mount_angle_world(i, self.body_yaw)) @ nominal_leg_local[i])

    def recompute_idle_height_only(self, nominal_leg_local):
        for i in range(6):
            target = self.nominal_world_for_leg(i, nominal_leg_local)
            self.anchor_world[i][2] = target[2]
            self.swing_from_world[i] = None
            self.swing_to_world[i] = None
            self.prev_is_stance[i] = True

    def recompute_idle_full_unison(self, nominal_leg_local):
        for i in range(6):
            self.anchor_world[i] = self.nominal_world_for_leg(i, nominal_leg_local).copy()
            self.swing_from_world[i] = None
            self.swing_to_world[i] = None
            self.prev_is_stance[i] = True

    def freeze_current_feet_as_anchors(self):
        for i in range(6):
            if self.swing_to_world[i] is not None and (not self.prev_is_stance[i]):
                self.anchor_world[i] = self.swing_to_world[i].copy()
            else:
                self.anchor_world[i] = leg_to_world(i, self.p_cmd_leg[i], self.body_xy, self.body_yaw).copy()

            self.swing_from_world[i] = None
            self.swing_to_world[i] = None
            self.prev_is_stance[i] = True

    def update_kp_phase_alphas(self, is_stance_list, dt):
        for i in range(6):
            target = 1.0 if is_stance_list[i] else 0.0
            cur = self.kp_phase_alpha[i]

            if target > cur:
                rate = KP_SCHEDULE_RATE_UP
            elif target < cur:
                rate = KP_SCHEDULE_RATE_DOWN
            else:
                rate = KP_SCHEDULE_RATE_UP

            self.kp_phase_alpha[i] = slew_exp(cur, target, rate, dt)

    def leg_kp_values(self, i):
        a = self.kp_phase_alpha[i]

        hy_kp = KP_HIPYAW_SWING + (KP_HIPYAW_STANCE - KP_HIPYAW_SWING) * a
        hp_kp = KP_HIPPITCH_SWING + (KP_HIPPITCH_STANCE - KP_HIPPITCH_SWING) * a
        knee_kp = KP_KNEE_SWING + (KP_KNEE_STANCE - KP_KNEE_SWING) * a

        return hy_kp, hp_kp, knee_kp

    def start_return_to_stance(self):
        self.vx_cmd = 0.0
        self.vy_cmd = 0.0
        self.yaw_cmd = 0.0
        self.vx_app = 0.0
        self.vy_app = 0.0
        self.yaw_app = 0.0

        self.freeze_current_feet_as_anchors()

        self.stance_adjust_mode = True
        self.return_leg_ptr = 0
        self.return_leg_active = None
        self.return_from_world = None
        self.return_to_world = None
        self.return_phase = 0.0

    def start_gait_cycle(self):
        self.vx_cmd = 0.0
        self.vy_cmd = 0.0
        self.yaw_cmd = 0.0
        self.vx_app = 0.0
        self.vy_app = 0.0
        self.yaw_app = 0.0

        self.freeze_current_feet_as_anchors()

        self.gait_switch_mode = True
        self.gait_switch_stage = "settle"
        self.gait_switch_timer = GAIT_SWITCH_SETTLE_TIME
        self.pending_gait_mode = self.next_gait_mode()

        self.stance_adjust_mode = False
        self.return_leg_ptr = 0
        self.return_leg_active = None
        self.return_from_world = None
        self.return_to_world = None
        self.return_phase = 0.0

        print(f"\n[GAIT] Requested gait switch: {self.gait_mode} -> {self.pending_gait_mode}")
        print(f"[GAIT] Settling for {GAIT_SWITCH_SETTLE_TIME:.1f}s, then graceful return, then applying new gait.\n")

    def apply_pending_gait(self):
        self.gait_mode = self.pending_gait_mode
        self.gait = PhaseGait(build_selected_gait(self.gait_mode))
        self.gait.reset_time()
        self.was_idle = True
        print(f"\n[GAIT] New gait active: {self.gait_mode}\n")

    def print_hud(self):
        os.system("clear")
        print("=================================================")
        print("              TOCATL WALK CONTROLLER")
        print("=================================================\n")

        print("STATE")
        print(f"  ESTOP      : {ESTOP}")
        print(f"  RESTARTING : {RESTARTING}")
        print(f"  SHUTDOWN   : {SHUTTING_DOWN}\n")

        print("COMMANDS")
        print(f"  vx_cmd:  {fmt(self.vx_cmd,2)}   vy_cmd:  {fmt(self.vy_cmd,2)}   yaw_cmd: {fmt(self.yaw_cmd,2)}")
        print(f"  vx_app:  {fmt(self.vx_app,2)}   vy_app:  {fmt(self.vy_app,2)}   yaw_app: {fmt(self.yaw_app,2)}\n")

        print("MODE")
        active_leg = "None" if self.return_leg_active is None else str(self.return_leg_active + 1)
        print(f"  gait_mode        : {self.gait_mode}")
        print(f"  pending_gait     : {self.pending_gait_mode}")
        print(f"  gait_switch_mode : {self.gait_switch_mode}")
        print(f"  gait_switch_stage: {self.gait_switch_stage}")
        print(f"  gait_switch_t    : {fmt(self.gait_switch_timer,2)}")
        print(f"  stance_adjust    : {self.stance_adjust_mode}")
        print(f"  active_leg       : {active_leg}\n")

        print("BODY")
        print(f"  x: {fmt(self.body_xy[0],1)}   y: {fmt(self.body_xy[1],1)}")
        print(f"  yaw: {fmt(rad2deg(self.body_yaw),2)} deg")
        print(f"  body_height: {fmt(self.body_height,1)} mm")
        print(f"  speed_scale: {fmt(self.speed_scale,3)}\n")

        print("GAIT")
        print(f"  stride_len: {fmt(self.step_len,1)} mm")
        print(f"  step_h    : {fmt(self.step_h,1)} mm")
        print(f"  cycle_T   : {fmt(self.gait.gait.cycle_time,2)} s")
        print(f"  duty      : {fmt(self.gait.gait.duty,3)}")
        print(f"  foot_vmax : {fmt(FOOT_SPEED_MAX,1)} mm/s")
        print(f"  return_T  : {fmt(RETURN_CYCLE_TIME,2)} s\n")

        print("GAINS / TORQUE")
        print(f"  Sched     : up={KP_SCHEDULE_RATE_UP:.2f}  down={KP_SCHEDULE_RATE_DOWN:.2f}")
        print(f"  HipYaw    : swing={KP_HIPYAW_SWING:.2f}  stance={KP_HIPYAW_STANCE:.2f}  Kd={KD_HIPYAW:.2f}")
        print(f"  HipPitch  : swing={KP_HIPPITCH_SWING:.2f}  stance={KP_HIPPITCH_STANCE:.2f}  Kd={KD_HIPPITCH:.2f}  tau_cmd={self.hip_pitch_tau_cmd:+.2f}")
        print(f"  Knee      : swing={KP_KNEE_SWING:.2f}  stance={KP_KNEE_STANCE:.2f}  Kd={KD_KNEE:.2f}\n")

        print("ERROR")
        print(f"  Leg 1 XYZ error: {fmt(self.err_leg1,2)} mm\n")

        print("Controls:")
        print("  Left stick UD  (axis 1) -> vx")
        print("  Left stick LR  (axis 0) -> vy")
        print("  Right stick LR (axis 3) -> yaw")
        print("  LT / RT        -> decrease / increase hip-pitch support torque")
        print("  A / B          -> body height down / up")
        print("  X / Y          -> leg radius in / out")
        print("  D-pad UD       -> step height down / up")
        print("  D-pad LR       -> stride length down / up")
        print("  LB             -> graceful return")
        print("  RB             -> cycle gait (stop 3s -> return -> switch gait)")
        print("  Back           -> soft E-STOP")
        print("  Start          -> restart after soft E-STOP")
        print("  L3 + R3        -> SUPER E-STOP")
        print("  Keyboard e/q   -> soft E-STOP\n")

    def cb(self, msg: Joy):
        global ESTOP, RESTARTING, SHUTTING_DOWN

        now = time.monotonic()
        dt = now - self._last_t
        self._last_t = now
        if dt <= 0 or dt > 0.2:
            dt = 0.02

        start = msg.buttons[BTN_START] if BTN_START < len(msg.buttons) else 0
        l3 = msg.buttons[BTN_L3] if BTN_L3 < len(msg.buttons) else 0
        r3 = msg.buttons[BTN_R3] if BTN_R3 < len(msg.buttons) else 0

        # SUPER E-STOP priority
        if (not SHUTTING_DOWN) and l3 == 1 and r3 == 1 and not (self._l3_prev == 1 and self._r3_prev == 1):
            self._l3_prev = l3
            self._r3_prev = r3
            super_estop("DUAL STICK SUPER E-STOP")
            return

        # While soft-estopped, ONLY allow START to request restart.
        if ESTOP:
            if start == 1 and self._start_prev == 0:
                request_restart()
            self._start_prev = start
            self._l3_prev = l3
            self._r3_prev = r3
            return

        if RESTARTING or SHUTTING_DOWN:
            self._start_prev = start
            self._l3_prev = l3
            self._r3_prev = r3
            return

        self._start_prev = start
        self._l3_prev = l3
        self._r3_prev = r3

        # ---------------- soft E-STOP on Back ----------------
        back = msg.buttons[BTN_BACK_ESTOP] if BTN_BACK_ESTOP < len(msg.buttons) else 0
        if back == 1 and self._back_prev == 0:
            estop("BACK BUTTON E-STOP")
            return
        self._back_prev = back

        # ---------------- LB graceful return ----------------
        lb = msg.buttons[BTN_LB] if BTN_LB < len(msg.buttons) else 0
        if lb == 1 and self._lb_prev == 0 and not self.gait_switch_mode:
            self.start_return_to_stance()
        self._lb_prev = lb

        # ---------------- RB gait cycle ----------------
        rb = msg.buttons[BTN_RB] if BTN_RB < len(msg.buttons) else 0
        if rb == 1 and self._rb_prev == 0 and not self.gait_switch_mode:
            self.start_gait_cycle()
        self._rb_prev = rb

        # ---------------- axes ----------------
        lx = msg.axes[AX_LX] if AX_LX < len(msg.axes) else 0.0
        ly = msg.axes[AX_LY] if AX_LY < len(msg.axes) else 0.0
        lt_raw = msg.axes[AX_LT] if AX_LT < len(msg.axes) else 1.0
        rx = msg.axes[AX_RX] if AX_RX < len(msg.axes) else 0.0
        rt_raw = msg.axes[AX_RT] if AX_RT < len(msg.axes) else 1.0
        dpx = msg.axes[AX_DX] if AX_DX < len(msg.axes) else 0.0
        dpy = msg.axes[AX_DY] if AX_DY < len(msg.axes) else 0.0

        # If gait switching, freeze motion commands at zero.
        if self.gait_switch_mode:
            self.vx_cmd = 0.0
            self.vy_cmd = 0.0
            self.yaw_cmd = 0.0
        else:
            self.vx_cmd = deadband(ly, JOY_DEADBAND)
            self.vy_cmd = deadband(-lx, JOY_DEADBAND)
            self.yaw_cmd = deadband(-rx, JOY_DEADBAND)

        # RT increases commanded support torque, LT decreases it
        lt = trigger_to_01(lt_raw)
        rt = trigger_to_01(rt_raw)

        tau_rate_cmd = HIP_PITCH_TAU_RATE * (rt - lt)
        if abs(tau_rate_cmd) < 1e-6:
            tau_rate_cmd = 0.0

        self.hip_pitch_tau_cmd = clamp(
            self.hip_pitch_tau_cmd + tau_rate_cmd * dt,
            0.0,
            HIP_PITCH_TAU_MAX
        )

        # ---------------- slew motion commands ----------------
        if self.gait_switch_mode:
            self.vx_app = 0.0
            self.vy_app = 0.0
            self.yaw_app = 0.0
        else:
            self.vx_app = slew_exp(self.vx_app, self.vx_cmd, CMD_SLEW_RATE, dt)
            self.vy_app = slew_exp(self.vy_app, self.vy_cmd, CMD_SLEW_RATE, dt)
            self.yaw_app = slew_exp(self.yaw_app, self.yaw_cmd, CMD_SLEW_RATE, dt)

        # ---------------- buttons / dpad knobs ----------------
        a_pressed = msg.buttons[BTN_A] if BTN_A < len(msg.buttons) else 0
        b_pressed = msg.buttons[BTN_B] if BTN_B < len(msg.buttons) else 0
        x_pressed = msg.buttons[BTN_X] if BTN_X < len(msg.buttons) else 0
        y_pressed = msg.buttons[BTN_Y] if BTN_Y < len(msg.buttons) else 0

        if b_pressed:
            self.body_height += HEIGHT_RATE * dt
        if a_pressed:
            self.body_height -= HEIGHT_RATE * dt
        self.body_height = clamp(self.body_height, BODY_HEIGHT_MIN, BODY_HEIGHT_MAX)

        if y_pressed:
            self.radius_offset += RADIUS_RATE * dt
        if x_pressed:
            self.radius_offset -= RADIUS_RATE * dt
        self.radius_offset = clamp(self.radius_offset, RADIUS_MIN, RADIUS_MAX)

        if dpy > 0.5:
            self.step_h += STEP_H_RATE * dt
        elif dpy < -0.5:
            self.step_h -= STEP_H_RATE * dt
        self.step_h = clamp(self.step_h, STEP_H_MIN, STEP_H_MAX)

        if dpx > 0.5:
            self.step_len += STEP_LEN_RATE * dt
        elif dpx < -0.5:
            self.step_len -= STEP_LEN_RATE * dt
        self.step_len = clamp(self.step_len, STEP_LEN_MIN, STEP_LEN_MAX)

        nominal_leg = self.current_nominal_leg()

        # ==================================================
        # GAIT SWITCH MODE
        # ==================================================
        if self.gait_switch_mode:
            self.speed_scale = 1.0

            if self.gait_switch_stage == "settle":
                self.gait_switch_timer = max(0.0, self.gait_switch_timer - dt)

                # keep feet planted where they are while settling
                for i in range(6):
                    pL = world_to_leg(i, self.anchor_world[i], self.body_xy, self.body_yaw)
                    if pL[0] < X_MIN_MM:
                        pL[0] = X_MIN_MM
                    self.p_cmd_leg[i] = pL

                self.update_kp_phase_alphas([True] * 6, dt)

                if self.gait_switch_timer <= 1e-6:
                    self.gait_switch_stage = "return"
                    self.start_return_to_stance()

            elif self.gait_switch_stage == "return":
                # Let the normal stance-adjust logic run below.
                pass

        # ==================================================
        # STANCE ADJUST MODE
        # ==================================================
        if self.stance_adjust_mode:
            self.gait.gait.cycle_time = RETURN_CYCLE_TIME
            self.speed_scale = 1.0

            if self.return_leg_active is None:
                if self.return_leg_ptr >= len(RETURN_ORDER):
                    self.stance_adjust_mode = False
                    self.freeze_current_feet_as_anchors()

                    if self.gait_switch_mode and self.gait_switch_stage == "return":
                        self.apply_pending_gait()
                        self.gait_switch_mode = False
                        self.gait_switch_stage = "idle"
                else:
                    leg = RETURN_ORDER[self.return_leg_ptr]
                    nominalW = self.nominal_world_for_leg(leg, nominal_leg)
                    curW = self.anchor_world[leg].copy()

                    dist = np.linalg.norm((nominalW - curW)[:2]) + abs(nominalW[2] - curW[2])

                    if dist < 1e-6:
                        self.anchor_world[leg] = nominalW.copy()
                        self.return_leg_ptr += 1
                    else:
                        self.return_leg_active = leg
                        self.return_from_world = curW.copy()
                        self.return_to_world = nominalW.copy()
                        self.return_phase = 0.0

            if self.stance_adjust_mode and self.return_leg_active is not None:
                leg = self.return_leg_active
                self.return_phase += dt / RETURN_CYCLE_TIME
                s_phase = clamp(self.return_phase, 0.0, 1.0)

                targetW = (1.0 - s_phase) * self.return_from_world + s_phase * self.return_to_world
                targetW[2] += self.step_h * math.sin(math.pi * s_phase)

                stance_mask = [True] * 6
                stance_mask[leg] = False

                for i in range(6):
                    if i == leg:
                        continue
                    pL = world_to_leg(i, self.anchor_world[i], self.body_xy, self.body_yaw)
                    if pL[0] < X_MIN_MM:
                        pL[0] = X_MIN_MM
                    self.p_cmd_leg[i] = pL

                pL = world_to_leg(leg, targetW, self.body_xy, self.body_yaw)
                if pL[0] < X_MIN_MM:
                    pL[0] = X_MIN_MM
                self.p_cmd_leg[leg] = pL

                self.update_kp_phase_alphas(stance_mask, dt)

                if self.return_phase >= 1.0:
                    self.anchor_world[leg] = self.return_to_world.copy()
                    self.return_leg_active = None
                    self.return_from_world = None
                    self.return_to_world = None
                    self.return_phase = 0.0
                    self.return_leg_ptr += 1
            else:
                self.update_kp_phase_alphas([True] * 6, dt)

        # ==================================================
        # NORMAL WALK / IDLE MODE
        # ==================================================
        elif not self.gait_switch_mode:
            trans_speed = math.sqrt(self.vx_app * self.vx_app + self.vy_app * self.vy_app)
            v_world = Rz(self.body_yaw) @ np.array([self.vx_app, self.vy_app, 0.0], dtype=float)
            vx_world = float(v_world[0])
            vy_world = float(v_world[1])

            if trans_speed > 1e-9:
                dirx_world = vx_world / trans_speed
                diry_world = vy_world / trans_speed
            else:
                dirx_world = 0.0
                diry_world = 0.0

            idle = (trans_speed < TRANS_SPEED_MIN_FOR_STEPPING and abs(self.yaw_app) < YAW_MIN_FOR_STEPPING)

            body_height_changed = abs(self.body_height - self.prev_body_height) > 1e-12
            radius_changed = abs(self.radius_offset - self.prev_radius_offset) > 1e-12

            if idle:
                self.gait.gait.cycle_time = CYCLE_SLOW
                self.speed_scale = 1.0

                if not self.was_idle:
                    self.freeze_current_feet_as_anchors()
                    self.gait.reset_time()
                else:
                    if body_height_changed:
                        self.recompute_idle_height_only(nominal_leg)

                    if radius_changed:
                        if self.body_height <= 0.0:
                            self.recompute_idle_full_unison(nominal_leg)
                        else:
                            self.start_return_to_stance()

            else:
                motion_metric = max(trans_speed, abs(self.yaw_app))
                s = clamp((motion_metric - min(TRANS_SPEED_MIN_FOR_STEPPING, YAW_MIN_FOR_STEPPING)) /
                          (1.0 - min(TRANS_SPEED_MIN_FOR_STEPPING, YAW_MIN_FOR_STEPPING)), 0.0, 1.0)

                cycle_time_nominal = (1.0 - s) * CYCLE_SLOW + s * CYCLE_FAST

                swing_dist = math.sqrt(self.step_len**2 + (2.0 * self.step_h)**2)
                swing_time_min = swing_dist / max(1e-6, FOOT_SPEED_MAX)
                cycle_time_min_from_foot_speed = swing_time_min / max(1e-6, (1.0 - self.gait.gait.duty))

                self.gait.gait.cycle_time = max(cycle_time_nominal, cycle_time_min_from_foot_speed)

                self.speed_scale = cycle_time_nominal / self.gait.gait.cycle_time
                self.speed_scale = clamp(self.speed_scale, 0.0, 1.0)

                self.gait.advance(dt)

            self.was_idle = idle
            self.prev_body_height = self.body_height
            self.prev_radius_offset = self.radius_offset

            if not idle:
                v_mm_s = BODY_SPEED_MAX_MM_S * clamp(trans_speed, 0.0, 1.0) * self.speed_scale
                yaw_rate_rad_s = BODY_YAW_RATE_MAX_RAD_S * clamp(self.yaw_app, -1.0, 1.0) * self.speed_scale

                self.body_xy[0] += dirx_world * v_mm_s * dt
                self.body_xy[1] += diry_world * v_mm_s * dt
                self.body_yaw = wrapPi(self.body_yaw + yaw_rate_rad_s * dt)

            stance_mask = [True] * 6

            for i in range(6):
                if not idle:
                    _, is_stance, s_phase = self.gait.phase(i)
                else:
                    is_stance = True
                    s_phase = 0.0

                stance_mask[i] = is_stance

                if not idle:
                    if self.prev_is_stance[i] and (not is_stance):
                        self.swing_from_world[i] = self.anchor_world[i].copy()

                        hipW = hip_pos_world(i, self.body_xy, self.body_yaw)
                        mount_ang = leg_mount_angle_world(i, self.body_yaw)
                        nominalW = hipW + (Rz(mount_ang) @ nominal_leg[i])

                        trans_offset = np.array([0.5 * self.step_len * dirx_world,
                                                 0.5 * self.step_len * diry_world,
                                                 0.0], dtype=float)

                        rel = nominalW - np.array([self.body_xy[0], self.body_xy[1], 0.0], dtype=float)
                        tangent = np.array([-rel[1], rel[0], 0.0], dtype=float)
                        tan_norm = np.linalg.norm(tangent[:2])
                        if tan_norm > 1e-9:
                            tangent /= tan_norm
                        else:
                            tangent[:] = 0.0

                        yaw_offset = tangent * (YAW_STRIDE_SCALE * self.step_len * self.yaw_app)
                        touchdown = nominalW + trans_offset + yaw_offset
                        self.swing_to_world[i] = touchdown.copy()

                    if (not self.prev_is_stance[i]) and is_stance:
                        if self.swing_to_world[i] is not None:
                            self.anchor_world[i] = self.swing_to_world[i].copy()

                    self.prev_is_stance[i] = is_stance
                else:
                    self.prev_is_stance[i] = True

                if is_stance:
                    targetW = self.anchor_world[i].copy()
                else:
                    pA = self.swing_from_world[i] if self.swing_from_world[i] is not None else self.anchor_world[i].copy()
                    pB = self.swing_to_world[i] if self.swing_to_world[i] is not None else pA.copy()
                    targetW = (1.0 - s_phase) * pA + s_phase * pB
                    targetW[2] += self.step_h * math.sin(math.pi * s_phase)

                pL = world_to_leg(i, targetW, self.body_xy, self.body_yaw)
                if pL[0] < X_MIN_MM:
                    pL[0] = X_MIN_MM
                self.p_cmd_leg[i] = pL

            self.update_kp_phase_alphas(stance_mask, dt)

        # If gait-switch settle mode but not stance-adjust mode, keep all feet planted.
        elif self.gait_switch_mode and self.gait_switch_stage == "settle":
            for i in range(6):
                pL = world_to_leg(i, self.anchor_world[i], self.body_xy, self.body_yaw)
                if pL[0] < X_MIN_MM:
                    pL[0] = X_MIN_MM
                self.p_cmd_leg[i] = pL
            self.update_kp_phase_alphas([True] * 6, dt)

        # ---------------- IK -> motor commands ----------------
        self.err_leg1 = None
        self.leg6_debug = {}

        for i, leg in enumerate(legs):
            t1_raw, t2_raw, t3_raw = ik_theta_from_xyz(
                self.p_cmd_leg[i][0], self.p_cmd_leg[i][1], self.p_cmd_leg[i][2]
            )

            dt1 = math.atan2(math.sin(t1_raw - self.theta1_prev[i]), math.cos(t1_raw - self.theta1_prev[i]))
            t1_unwrapped = self.theta1_prev[i] + dt1

            t1_lim, t2_lim, t3_lim = apply_motor_limits(t1_unwrapped, t2_raw, t3_raw, OFF1, OFF2, OFF3)

            self.theta1_des[i] = t1_lim
            self.theta2_des[i] = t2_lim
            self.theta3_des[i] = t3_lim
            self.theta1_prev[i] = self.theta1_des[i]

            self.theta1_act[i] = wrapPi(self.theta1_des[i])
            self.theta2_act[i] = self.theta2_des[i]
            self.theta3_act[i] = self.theta3_des[i]

            m1_cmd, m2_cmd, m3_cmd = theta_to_motor(
                self.theta1_act[i], self.theta2_act[i], self.theta3_act[i], OFF1, OFF2, OFF3
            )

            hy_kp_leg, hp_kp_leg, knee_kp_leg = self.leg_kp_values(i)
            hp_tau_leg = self.hip_pitch_tau_cmd

            set_cmd(leg[0], position=m1_cmd, velocity=JOY_VEL, torque=JOY_TAU,    kp=hy_kp_leg,   kd=KD_HIPYAW)
            set_cmd(leg[1], position=m2_cmd, velocity=JOY_VEL, torque=hp_tau_leg, kp=hp_kp_leg,   kd=KD_HIPPITCH)
            set_cmd(leg[2], position=m3_cmd, velocity=JOY_VEL, torque=JOY_TAU,    kp=knee_kp_leg, kd=KD_KNEE)

            if i == 0:
                cmdW = leg_to_world(i, self.p_cmd_leg[i], self.body_xy, self.body_yaw)
                footW = leg_to_world(i, tip_from_theta(self.theta1_act[i], self.theta2_act[i], self.theta3_act[i]),
                                     self.body_xy, self.body_yaw)
                self.err_leg1 = float(np.linalg.norm(cmdW - footW))

        if now - self.last_print > 0.10:
            self.last_print = now
            self.print_hud()


# ======================================================
# ROS spin thread
# ======================================================
def ros_spin_thread():
    global ACTIVE_NODE
    rclpy.init(args=None)
    node = HexapodWalkJoy()
    ACTIVE_NODE = node
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        estop("CTRL+C")
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


# ======================================================
# STARTUP
# ======================================================
JOY_PROC = start_joy_node()
time.sleep(1.0)

threading.Thread(target=ros_spin_thread, daemon=True).start()
time.sleep(0.5)

startup_sequence()

try:
    while True:
        time.sleep(0.05)
        if RESTART_REQUEST and (not RESTARTING) and (not SHUTTING_DOWN):
            startup_sequence()
except KeyboardInterrupt:
    super_estop("CTRL+C SUPER E-STOP")