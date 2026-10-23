#!/usr/bin/env python3
# ONE LEG TEST: Streamer + Keyboard E-Stop + Controller control of ONE leg in CARTESIAN (XYZ) space
# - No plotting
# - No standing sequence
# - No enabling other legs
# - Controller mapping (rebound mapping you provided):
#   Axes: 0=Left Joy X, 1=Left Joy Y, 4=Right Joy Y, 5=Right Trigger
#   Buttons: 6=Back (E-STOP)
#
# Requested mapping:
#   Left stick vertical (axis 1)   -> X
#   Left stick horizontal (axis 0) -> Y
#   Right stick vertical (axis 4)  -> Z
#
# IMPORTANT: X is clamped to be >= X_MIN_MM (default 0.0), like the simulator.

import can, time, subprocess, threading
import sys, os, select, signal, shlex, math
import numpy as np

from CyberGearDriver import CyberGearMotor, RunMode, CyberMotorMessage

# -------------------------------------------------------
# ROS auto-bootstrap (so rclpy works even inside .venv)
# -------------------------------------------------------
try:
    import rclpy  # noqa
except ModuleNotFoundError:
    script = shlex.quote(os.path.abspath(__file__))
    py = shlex.quote(sys.executable)  # keep venv python
    cmd = f"source /opt/ros/jazzy/setup.bash && exec {py} {script}"
    os.execvp("bash", ["bash", "-lc", cmd])

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

# -------------------------------------------------------
# joy_linux_node bring-up
# -------------------------------------------------------
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

# -------------------------------------------------------
# Helpers
# -------------------------------------------------------
def wrapPi(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))

def deg2rad(x: float) -> float:
    return x * math.pi / 180.0

def rad2deg(x: float) -> float:
    return x * 180.0 / math.pi

def clamp(x, lo, hi):
    return lo if x < lo else (hi if x > hi else x)

# -------------------------------------------------------
# Geometry (mm) — SAME as your simulator
# -------------------------------------------------------
d1 = 114.25
d2 = 45.5561
d3 = 330.3455
d4 = 458.0

# -------------------------------------------------------
# OFFSETS (GLOBAL) — SIMULATOR STYLE
# NOTE: This script calls set_zero_position() at startup.
# If you want offsets to matter as "motors=0 => theta=offset",
# you should NOT call set_zero_position(), or set offsets to 0.
# Keeping your offsets as provided (you asked to include them).
# -------------------------------------------------------
OFF1 = 0.0   # yaw offset (rad)
OFF2 = deg2rad(45.44 - 180.0)                     # hip pitch offset (rad)
OFF3 = -OFF2 + deg2rad((45.44 - 11.18))           # knee RELATIVE offset (rad)

# -------------------------------------------------------
# Cartesian command limits / rates
# -------------------------------------------------------
DEADBAND = 0.08

XYZ_RATE_SLOW = 120.0   # mm/s at full stick
XYZ_RATE_FAST = 350.0   # mm/s at full stick

# Soft bounds (edit to your safe workspace)
X_MIN_MM = 0.0
X_MAX_MM = 900.0
Y_MIN_MM = -700.0
Y_MAX_MM = 700.0
Z_MIN_MM = -700.0
Z_MAX_MM = 700.0

def deadband(x):
    return 0.0 if abs(x) < DEADBAND else x

# -------------------------------------------------------
# FK / IK (SIMULATOR MATH ONLY)
# - theta1 = yaw
# - theta2 = hip pitch
# - theta3 = knee RELATIVE to thigh
# -------------------------------------------------------
def Ry(th: float) -> np.ndarray:
    c = math.cos(th)
    s = math.sin(th)
    return np.array([[ c, 0.0,  s],
                     [0.0, 1.0, 0.0],
                     [-s, 0.0,  c]], dtype=float)

def fk_from_theta(theta1: float, theta2: float, theta3: float):
    ct = math.cos(theta1)
    st = math.sin(theta1)
    Rz = np.array([[ ct, -st, 0.0],
                   [ st,  ct, 0.0],
                   [0.0, 0.0, 1.0]], dtype=float)

    base = np.array([[d2], [0.0], [d1]], dtype=float)
    u    = np.array([[d3], [0.0], [0.0]], dtype=float)
    l    = np.array([[d4], [0.0], [0.0]], dtype=float)

    knee = base + Rz @ (Ry(theta2)          @ u)
    foot = knee + Rz @ (Ry(theta2 + theta3) @ l)
    return base, knee, foot

def ik_theta_from_xyz(Xd: float, Yd: float, Zd: float):
    base_x, base_y, base_z = d2, 0.0, d1

    X = Xd - base_x
    Y = Yd - base_y
    Z = Zd - base_z

    theta1 = math.atan2(Y, X)
    r1 = math.sqrt(X*X + Y*Y)

    Xe = r1
    Ze = Z
    R  = math.sqrt(Xe*Xe + Ze*Ze)

    C3 = (R*R - d3*d3 - d4*d4) / (2.0 * d3 * d4)
    C3 = max(min(C3, 1.0), -1.0)

    theta3 = math.atan2(math.sqrt(max(0.0, 1.0 - C3*C3)), C3)

    phi   = math.atan2(-Ze, Xe)
    gamma = math.atan2(d4 * math.sin(theta3), d3 + d4 * math.cos(theta3))
    theta2 = phi - gamma

    return theta1, theta2, theta3

# -------------------------------------------------------
# θ <-> motor mapping (your UPDATED version with knee motor sign + abs/rel handling)
# -------------------------------------------------------
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
    a3 = -m3 + off3_abs  # invert back (because m3 was negated in theta_to_motor)

    theta2 = a2
    theta3 = a3 - a2
    return theta1, theta2, theta3

def foot_xyz_from_theta(theta1, theta2, theta3):
    _, _, foot = fk_from_theta(theta1, theta2, theta3)
    return float(foot[0,0]), float(foot[1,0]), float(foot[2,0])

# -------------------------------------------------------
# CAN bring-up
# -------------------------------------------------------
def setup_can(name: str):
    print(f"\nSetting up Interface for {name}...\n")
    subprocess.run(["sudo", "ip", "link", "set", name, "down"], stderr=subprocess.DEVNULL)
    subprocess.run(["sudo", "ip", "link", "set", name, "type", "can", "bitrate", "1000000"], check=True)
    subprocess.run(["sudo", "ip", "link", "set", name, "up"], check=True)

setup_can("can0")
setup_can("can1")

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

# -------------------------------------------------------
# Motors (same mapping as your script)
# -------------------------------------------------------
leg1hipYaw = CyberGearMotor(motor_id=11, send_message=send_message1)
leg1hipPitch = CyberGearMotor(motor_id=12, send_message=send_message1)
leg1knee = CyberGearMotor(motor_id=13, send_message=send_message1)

leg2hipYaw = CyberGearMotor(motor_id=21, send_message=send_message0)
leg2hipPitch = CyberGearMotor(motor_id=22, send_message=send_message0)
leg2knee = CyberGearMotor(motor_id=23, send_message=send_message0)

leg3hipYaw = CyberGearMotor(motor_id=31, send_message=send_message0)
leg3hipPitch = CyberGearMotor(motor_id=32, send_message=send_message0)
leg3knee = CyberGearMotor(motor_id=33, send_message=send_message0)

leg4hipYaw = CyberGearMotor(motor_id=41, send_message=send_message0)
leg4hipPitch = CyberGearMotor(motor_id=42, send_message=send_message0)
leg4knee = CyberGearMotor(motor_id=43, send_message=send_message0)

leg5hipYaw = CyberGearMotor(motor_id=51, send_message=send_message1)
leg5hipPitch = CyberGearMotor(motor_id=52, send_message=send_message1)
leg5knee = CyberGearMotor(motor_id=53, send_message=send_message1)

leg6hipYaw = CyberGearMotor(motor_id=61, send_message=send_message1)
leg6hipPitch = CyberGearMotor(motor_id=62, send_message=send_message1)
leg6knee = CyberGearMotor(motor_id=63, send_message=send_message1)

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

leg1 = [leg1hipYaw, leg1hipPitch, leg1knee]
leg2 = [leg2hipYaw, leg2hipPitch, leg2knee]
leg3 = [leg3hipYaw, leg3hipPitch, leg3knee]
leg4 = [leg4hipYaw, leg4hipPitch, leg4knee]
leg5 = [leg5hipYaw, leg5hipPitch, leg5knee]
leg6 = [leg6hipYaw, leg6hipPitch, leg6knee]

# -------------------------------------------------------
# Select ONE leg to enable + control
# -------------------------------------------------------
LEGS = {1: leg1, 2: leg2, 3: leg3, 4: leg4, 5: leg5, 6: leg6}
TEST_LEG_NUM = 5  # <-- CHANGE THIS (1..6)
TEST_LEG = LEGS[TEST_LEG_NUM]
TEST_HIPYAW, TEST_HIPPITCH, TEST_KNEE = TEST_LEG[0], TEST_LEG[1], TEST_LEG[2]
ACTIVE_MOTORS = [TEST_HIPYAW, TEST_HIPPITCH, TEST_KNEE]

print(f"\n=== ONE LEG CARTESIAN IK MODE (SIM MATH) ===")
print(f"Leg {TEST_LEG_NUM}: HipYaw={TEST_HIPYAW.motor_id}, HipPitch={TEST_HIPPITCH.motor_id}, Knee={TEST_KNEE.motor_id}")
print("NO other motors will be enabled or commanded.\n")

# -------------------------------------------------------
# RX dispatch (doesn't command anything)
# -------------------------------------------------------
def dispatch_message0(msg):
    for m in leftSideMotors:
        m.message_received(msg)

def dispatch_message1(msg):
    for m in rightSideMotors:
        m.message_received(msg)

notifier0 = can.Notifier(bus0, [dispatch_message0])
notifier1 = can.Notifier(bus1, [dispatch_message1])

# -------------------------------------------------------
# Streamer (ONLY streams ACTIVE_MOTORS)
# -------------------------------------------------------
cmd_lock = threading.Lock()
desired = {}         # motor_id -> (pos, vel, tau, kp, kd)
STREAM_ON = True
STREAM_HZ = 200

def set_cmd(motor, position, velocity=0.0, torque=0.0, kp=0.0, kd=0.0):
    with cmd_lock:
        desired[motor.motor_id] = (position, velocity, torque, kp, kd)

def streamer_loop():
    global STREAM_ON
    period = 1.0 / STREAM_HZ
    next_t = time.perf_counter()
    while STREAM_ON:
        with cmd_lock:
            snap = dict(desired)

        for m in ACTIVE_MOTORS:
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

# -------------------------------------------------------
# Helpers for state snapshot (ONLY active motors)
# -------------------------------------------------------
def snapshot_state(timeout_s=0.08):
    for m in ACTIVE_MOTORS:
        try:
            m.request_motor_state()
        except Exception:
            pass
    time.sleep(timeout_s)

    out = {}
    for m in ACTIVE_MOTORS:
        st = m.state
        out[m.motor_id] = {
            "pos": st.get("position"),
            "vel": st.get("velocity"),
            "tau": st.get("torque"),
        }
    return out

# -------------------------------------------------------
# VIRTUAL E-STOP (Keyboard + Back button)
# -------------------------------------------------------
ESTOP = False
JOY_PROC = None

def estop(reason="E-STOP"):
    global ESTOP
    if ESTOP:
        return
    ESTOP = True

    print("\n\n==============================")
    print(f"!!! {reason} ACTIVATED !!!")
    print("==============================\n")

    hold = snapshot_state(timeout_s=0.05)

    for m in ACTIVE_MOTORS:
        p = hold[m.motor_id]["pos"]
        if p is None:
            p = 0.0
        set_cmd(m, p, velocity=0.0, torque=0.0, kp=0.0, kd=6.0)
    time.sleep(0.25)

    stop_streamer()
    time.sleep(0.05)

    for m in ACTIVE_MOTORS:
        try:
            m.stop()
        except Exception:
            pass

    try: notifier0.stop()
    except Exception: pass
    try: notifier1.stop()
    except Exception: pass
    try: bus0.shutdown()
    except Exception: pass
    try: bus1.shutdown()
    except Exception: pass

    try:
        if JOY_PROC is not None:
            stop_joy_node(JOY_PROC)
    except Exception:
        pass

    print("Active motors stopped, CAN shutdown. Exiting now.")
    os._exit(1)

def keyboard_estop_listener():
    print("\nVirtual E-Stop ARMED: type 'e' + Enter (or 'q') at any time.\n")
    while True:
        if ESTOP:
            return
        r, _, _ = select.select([sys.stdin], [], [], 0.05)
        if r:
            key = sys.stdin.readline().strip().lower()
            if key in ("e", "q", "estop", "stop"):
                estop("KEYBOARD E-STOP")

threading.Thread(target=keyboard_estop_listener, daemon=True).start()

# -------------------------------------------------------
# Cartesian command state
# -------------------------------------------------------
xyz_lock = threading.Lock()
Xd_cmd = 0.0
Yd_cmd = 0.0
Zd_cmd = 0.0

# yaw unwrap state (prevents +pi/-pi snapping; does NOT change IK math)
theta1_prev = 0.0

# -------------------------------------------------------
# Telemetry thread (MATLAB-ish block + includes motor commands)
# -------------------------------------------------------
TELEM_ON = True
TELEM_HZ = 10.0

def fmt(x, p=3):
    if x is None:
        return "   None"
    return f"{x:+.{p}f}"

def telemetry_loop():
    period = 1.0 / TELEM_HZ

    while TELEM_ON and not ESTOP:
        st = snapshot_state(timeout_s=0.03)

        m1 = st[TEST_HIPYAW.motor_id]["pos"]
        m2 = st[TEST_HIPPITCH.motor_id]["pos"]
        m3 = st[TEST_KNEE.motor_id]["pos"]

        v1 = st[TEST_HIPYAW.motor_id]["vel"]
        v2 = st[TEST_HIPPITCH.motor_id]["vel"]
        v3 = st[TEST_KNEE.motor_id]["vel"]

        tau1 = st[TEST_HIPYAW.motor_id]["tau"]
        tau2 = st[TEST_HIPPITCH.motor_id]["tau"]
        tau3 = st[TEST_KNEE.motor_id]["tau"]

        if None in (m1, m2, m3):
            th1_act = th2_act = th3_act = None
            xyz_act = (None, None, None)
        else:
            th1_act, th2_act, th3_act = motor_to_theta(m1, m2, m3, OFF1, OFF2, OFF3)
            xyz_act = foot_xyz_from_theta(th1_act, th2_act, th3_act)

        with xyz_lock:
            xd, yd, zd = Xd_cmd, Yd_cmd, Zd_cmd

        th1_des, th2_des, th3_des = ik_theta_from_xyz(xd, yd, zd)
        m1_cmd, m2_cmd, m3_cmd = theta_to_motor(th1_des, th2_des, th3_des, OFF1, OFF2, OFF3)

        err = None
        if xyz_act[0] is not None:
            err = math.sqrt((xd - xyz_act[0])**2 + (yd - xyz_act[1])**2 + (zd - xyz_act[2])**2)

        sys.stdout.write("\033[2J\033[H")
        print("=================================================")
        print("            TOCATL SINGLE LEG CONTROL")
        print("=================================================\n")

        print("COMMAND XYZ (mm)")
        print(f"  X: {fmt(xd,1)}   Y: {fmt(yd,1)}   Z: {fmt(zd,1)}\n")

        print("ACTUAL XYZ (mm)")
        print(f"  X: {fmt(xyz_act[0],1)}   Y: {fmt(xyz_act[1],1)}   Z: {fmt(xyz_act[2],1)}\n")

        print(f"POSITION ERROR (mm): {fmt(err,2)}\n")

        print("JOINT ANGLES θ (deg)")
        print(f"  θ1 Yaw   : {fmt(None if th1_act is None else rad2deg(th1_act),2)}")
        print(f"  θ2 Hip   : {fmt(None if th2_act is None else rad2deg(th2_act),2)}")
        print(f"  θ3 Knee  : {fmt(None if th3_act is None else rad2deg(th3_act),2)}\n")

        print("MOTOR COMMANDS (rad)")
        print(f"  m1_cmd : {fmt(m1_cmd)}")
        print(f"  m2_cmd : {fmt(m2_cmd)}")
        print(f"  m3_cmd : {fmt(m3_cmd)}\n")

        print("MOTOR FEEDBACK (rad)")
        print(f"  m1_act : {fmt(m1)}")
        print(f"  m2_act : {fmt(m2)}")
        print(f"  m3_act : {fmt(m3)}\n")

        print("MOTOR VELOCITY")
        print(f"  v1: {fmt(v1)}   v2: {fmt(v2)}   v3: {fmt(v3)}\n")

        print("MOTOR TORQUE")
        print(f"  τ1: {fmt(tau1)}   τ2: {fmt(tau2)}   τ3: {fmt(tau3)}\n")

        print("-------------------------------------------------")
        print("Controls (requested mapping):")
        print("  Left Stick UD  (axis 1) -> X")
        print("  Left Stick LR  (axis 0) -> Y")
        print("  Right Stick UD (axis 4) -> Z")
        print("  Right Trigger  (axis 5) -> FAST")
        print("  Back Button    (button 6) -> E-STOP")
        print("-------------------------------------------------")
        sys.stdout.flush()

        time.sleep(period)

threading.Thread(target=telemetry_loop, daemon=True).start()

# -------------------------------------------------------
# ROS2 Joy -> update XYZ -> IK -> motor cmds -> set_cmd
# Requested mapping:
#   AX_LX=0 -> Y
#   AX_LY=1 -> X
#   AX_RY=4 -> Z
#   RT axis 5 -> fast mode
# -------------------------------------------------------
AX_LX = 0
AX_LY = 1
AX_RY = 4
AX_RT = 5
BTN_BACK_ESTOP = 6

JOY_KP = 6.0
JOY_KD = 1.5
JOY_TAU = 0.0
JOY_VEL = 0.0

class OneLegCartesianJoy(Node):
    def __init__(self):
        super().__init__('one_leg_cartesian_joy_sim_math')
        self.sub = self.create_subscription(Joy, '/joy', self.cb, 10)
        self._last_t = time.monotonic()
        self._back_prev = 0

        # Initialize XYZ target from current motor state (prevents jump)
        st = snapshot_state(timeout_s=0.12)
        m1 = st[TEST_HIPYAW.motor_id]["pos"] or 0.0
        m2 = st[TEST_HIPPITCH.motor_id]["pos"] or 0.0
        m3 = st[TEST_KNEE.motor_id]["pos"] or 0.0

        th1, th2, th3 = motor_to_theta(m1, m2, m3, OFF1, OFF2, OFF3)
        x0, y0, z0 = foot_xyz_from_theta(th1, th2, th3)

        global Xd_cmd, Yd_cmd, Zd_cmd, theta1_prev
        with xyz_lock:
            Xd_cmd, Yd_cmd, Zd_cmd = x0, y0, z0
        theta1_prev = th1

        # Hold current motor positions immediately
        set_cmd(TEST_HIPYAW,   m1, velocity=0.0, torque=0.0, kp=0.0, kd=6.0)
        set_cmd(TEST_HIPPITCH, m2, velocity=0.0, torque=0.0, kp=0.0, kd=6.0)
        set_cmd(TEST_KNEE,     m3, velocity=0.0, torque=0.0, kp=0.0, kd=6.0)

        print("\n\n[JOY] Ready. CARTESIAN control mapping (requested):")
        print("  Left  stick U/D -> X (axis 1)")
        print("  Left  stick L/R -> Y (axis 0)")
        print("  Right stick U/D -> Z (axis 4)")
        print("  Back button     -> E-STOP (button 6)")
        print("  Fast mode        -> Right Trigger (axis 5 > 0.2)")
        print("  Keyboard: type 'e' + Enter -> E-STOP\n")
        print(f"[HOME] XYZ start (mm): X={x0:.1f}  Y={y0:.1f}  Z={z0:.1f}")

    def cb(self, msg: Joy):
        if ESTOP:
            return

        now = time.monotonic()
        dt = now - self._last_t
        self._last_t = now
        if dt <= 0 or dt > 0.2:
            dt = 0.02

        # E-STOP on Back button (rising edge)
        back = msg.buttons[BTN_BACK_ESTOP] if BTN_BACK_ESTOP < len(msg.buttons) else 0
        if back == 1 and self._back_prev == 0:
            estop("BACK BUTTON E-STOP")
            return
        self._back_prev = back

        lx = msg.axes[AX_LX] if AX_LX < len(msg.axes) else 0.0
        ly = msg.axes[AX_LY] if AX_LY < len(msg.axes) else 0.0
        ry = msg.axes[AX_RY] if AX_RY < len(msg.axes) else 0.0

        # stick directions: up = positive for X and Z
        x_cmd = deadband(-ly)  # X from left stick vertical
        y_cmd = deadband(lx)   # Y from left stick horizontal
        z_cmd = deadband(-ry)  # Z from right stick vertical

        rt = msg.axes[AX_RT] if AX_RT < len(msg.axes) else 0.0
        fast = (rt > 0.2)
        rate = XYZ_RATE_FAST if fast else XYZ_RATE_SLOW

        with xyz_lock:
            global Xd_cmd, Yd_cmd, Zd_cmd
            Xd_cmd += x_cmd * rate * dt
            Yd_cmd += y_cmd * rate * dt
            Zd_cmd += z_cmd * rate * dt

            Xd_cmd = clamp(Xd_cmd, X_MIN_MM, X_MAX_MM)
            Yd_cmd = clamp(Yd_cmd, Y_MIN_MM, Y_MAX_MM)
            Zd_cmd = clamp(Zd_cmd, Z_MIN_MM, Z_MAX_MM)

            xd, yd, zd = Xd_cmd, Yd_cmd, Zd_cmd

        th1_raw, th2, th3 = ik_theta_from_xyz(xd, yd, zd)

        global theta1_prev
        dth = math.atan2(math.sin(th1_raw - theta1_prev), math.cos(th1_raw - theta1_prev))
        th1 = theta1_prev + dth
        theta1_prev = th1

        m1_cmd, m2_cmd, m3_cmd = theta_to_motor(th1, th2, th3, OFF1, OFF2, OFF3)

        set_cmd(TEST_HIPYAW,   position=m1_cmd, velocity=JOY_VEL, torque=JOY_TAU, kp=JOY_KP, kd=JOY_KD)
        set_cmd(TEST_HIPPITCH, position=m2_cmd, velocity=JOY_VEL, torque=JOY_TAU, kp=JOY_KP, kd=JOY_KD)
        set_cmd(TEST_KNEE,     position=m3_cmd, velocity=JOY_VEL, torque=JOY_TAU, kp=JOY_KP, kd=JOY_KD)

def ros_spin_thread():
    rclpy.init(args=None)
    node = OneLegCartesianJoy()
    try:
        while rclpy.ok() and not ESTOP:
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

# -------------------------------------------------------
# STARTUP: stop everyone, enable ONLY active leg, zero active leg, then controller
# -------------------------------------------------------
print("[START] Stopping ALL motors (no other legs will be enabled).")
for m in allMotors:
    try:
        m.stop()
    except Exception:
        pass

print("[START] Setting mode + enabling ONLY selected leg motors.")
for m in ACTIVE_MOTORS:
    m.mode(RunMode.OPERATION_CONTROL)
for m in ACTIVE_MOTORS:
    m.enable()

time.sleep(0.25)

print("[START] Zeroing ONLY the selected leg motors.")
for m in ACTIVE_MOTORS:
    try:
        m.set_zero_position()
    except Exception:
        pass

time.sleep(0.5)

st = snapshot_state(timeout_s=0.10)
m1_0 = st[TEST_HIPYAW.motor_id]["pos"] or 0.0
m2_0 = st[TEST_HIPPITCH.motor_id]["pos"] or 0.0
m3_0 = st[TEST_KNEE.motor_id]["pos"] or 0.0

set_cmd(TEST_HIPYAW,   m1_0, velocity=0.0, torque=0.0, kp=0.0, kd=6.0)
set_cmd(TEST_HIPPITCH, m2_0, velocity=0.0, torque=0.0, kp=0.0, kd=6.0)
set_cmd(TEST_KNEE,     m3_0, velocity=0.0, torque=0.0, kp=0.0, kd=6.0)

print("\n[START] Starting controller system...")
JOY_PROC = start_joy_node()
time.sleep(1.0)
threading.Thread(target=ros_spin_thread, daemon=True).start()

print("\n=== READY ===")
print("ONLY the selected leg is enabled and will move.")
print("Cartesian control: Left stick UD -> X, Left stick LR -> Y, Right stick UD -> Z")
print("Fast mode: squeeze Right Trigger (axis 5 > 0.2)")
print("E-STOP: Back button (button 6) OR type 'e' + Enter\n")

try:
    while True:
        time.sleep(0.25)
except KeyboardInterrupt:
    estop("CTRL+C")