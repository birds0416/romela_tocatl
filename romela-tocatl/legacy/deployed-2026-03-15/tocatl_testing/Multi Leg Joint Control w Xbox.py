#!/usr/bin/env python3
# ALL LEGS "JELLYFISH" TEST:
# - Streamer
# - Keyboard E-Stop
# - Controller controls all legs in unison (positions only)
# - NEW controller mapping:
#   Axes: 0 LX, 1 LY, 2 LT, 3 RX, 4 RY, 5 RT, 6 DPAD_X, 7 DPAD_Y
#   Buttons: 0 A, 1 B, 2 X, 3 Y, 4 LB, 5 RB, 6 Back, 7 Start, 8 LS, 9 RS, (10 may appear on some setups)

import can, time, subprocess, threading
import sys, os, select, signal, shlex

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
        # SIGTERM tends to avoid rclcpp SIGINT handler weirdness
        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
    except Exception:
        pass

# -------------------------------------------------------
# CAN bring-up
# -------------------------------------------------------
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

# -------------------------------------------------------
# Motors
# -------------------------------------------------------
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

allHipYaw   = [leg1hipYaw, leg2hipYaw, leg3hipYaw, leg4hipYaw, leg5hipYaw, leg6hipYaw]
allHipPitch = [leg1hipPitch, leg2hipPitch, leg3hipPitch, leg4hipPitch, leg5hipPitch, leg6hipPitch]
allKnee     = [leg1knee, leg2knee, leg3knee, leg4knee, leg5knee, leg6knee]

# -------------------------------------------------------
# RX dispatch
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
# Streamer (streams ALL motors)
# -------------------------------------------------------
cmd_lock = threading.Lock()
desired = {}         # motor_id -> (pos, vel, tau, kp, kd)
STREAM_ON = True
STREAM_HZ = 150

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

# -------------------------------------------------------
# Helpers for state snapshot
# -------------------------------------------------------
def snapshot_positions(timeout_s=0.08):
    for m in allMotors:
        try:
            m.request_motor_state()
        except Exception:
            pass
    time.sleep(timeout_s)

    pos = {}
    for m in allMotors:
        p = m.state.get("position")
        pos[m.motor_id] = 0.0 if p is None else p
    return pos

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

    hold = snapshot_positions(timeout_s=0.05)

    # damp in-place
    for m in allMotors:
        set_cmd(m, hold[m.motor_id], velocity=0, torque=0, kp=0.0, kd=6.0)
    time.sleep(0.25)

    stop_streamer()
    time.sleep(0.05)

    for m in allMotors:
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

    print("Motors stopped, CAN shutdown. Exiting now.")
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
# Live telemetry (group min/max)
# -------------------------------------------------------
TELEM_ON = True
TELEM_HZ = 10.0

def telemetry_loop():
    period = 1.0 / TELEM_HZ
    while TELEM_ON and not ESTOP:
        for m in allMotors:
            try:
                m.request_motor_state()
            except Exception:
                pass
        time.sleep(0.03)

        def get_positions(group):
            out = []
            for m in group:
                p = m.state.get("position")
                if p is not None:
                    out.append(float(p))
            return out

        hy = get_positions(allHipYaw)
        hp = get_positions(allHipPitch)
        kn = get_positions(allKnee)

        def mm(arr):
            if not arr:
                return (" None", " None")
            return (f"{min(arr):+.3f}", f"{max(arr):+.3f}")

        hy_min, hy_max = mm(hy)
        hp_min, hp_max = mm(hp)
        kn_min, kn_max = mm(kn)

        line = f"\rHY[{hy_min},{hy_max}]  HP[{hp_min},{hp_max}]  KN[{kn_min},{kn_max}]   "
        sys.stdout.write(line)
        sys.stdout.flush()

        time.sleep(max(0.0, period - 0.03))

threading.Thread(target=telemetry_loop, daemon=True).start()

# -------------------------------------------------------
# ROS2 Joy -> UNISON control (set_cmd), Back button -> estop
#   Controller mapping you confirmed:
#     Axes: 0 LX, 1 LY, 2 LT, 3 RX, 4 RY, 5 RT, 6 DPAD_X, 7 DPAD_Y
# -------------------------------------------------------
AX_LY = 1   # left stick vertical -> knees
AX_RX = 3   # right stick horizontal -> hipYaw
AX_RY = 4   # right stick vertical -> hipPitch
BTN_BACK_ESTOP = 6

DEADBAND = 0.08

# rates: radians/sec at full stick
HIPYAW_RATE   = 1.0
HIPPITCH_RATE = 1.0
KNEE_RATE     = 1.0

# Safe-ish group limits (EDIT THESE)
LIM_HIPYAW   = (-1.0,  1.0)
LIM_HIPPITCH = (0.0,   4.9)
LIM_KNEE     = (-1.0,  4.0)

# -------------------------------------------------------
# Per-joint-group stiffness/damping (SAME across all legs)
# -------------------------------------------------------
KP_HIPYAW   = 5
KD_HIPYAW   = 1

KP_HIPPITCH = 10
KD_HIPPITCH = 1

KP_KNEE     = 6
KD_KNEE     = 1

JOY_TAU = 0.0
JOY_VEL = 0.0

def clamp(x, lo, hi):
    return lo if x < lo else (hi if x > hi else x)

def deadband(x):
    return 0.0 if abs(x) < DEADBAND else x

class JellyfishJoy(Node):
    def __init__(self):
        super().__init__('jellyfish_joy')
        self.sub = self.create_subscription(Joy, '/joy', self.cb, 10)
        self._last_t = time.monotonic()

        # rising edge for Back button
        self._back_prev = 0

        # seed common targets from current robot pose
        hold = snapshot_positions(timeout_s=0.10)

        # use leg1 as a seed reference (but it’s a common target for everyone)
        self.hy = clamp(hold.get(leg1hipYaw.motor_id, 0.0), *LIM_HIPYAW)
        self.hp = clamp(hold.get(leg1hipPitch.motor_id, 0.0), *LIM_HIPPITCH)
        self.kn = clamp(hold.get(leg1knee.motor_id, 0.0), *LIM_KNEE)

        # hold everyone at the current pose initially (prevents surprise)
        for m in allMotors:
            set_cmd(m, hold[m.motor_id], velocity=0, torque=0, kp=0.0, kd=6.0)

        print("\n\n[JELLYFISH] Ready.")
        print("Controls:")
        print("  Right stick L/R -> ALL HipYaw")
        print("  Right stick U/D -> ALL HipPitch")
        print("  Left  stick U/D -> ALL Knees")
        print("E-STOP: Back button (button 6) OR type 'e' + Enter\n")

        print("Gains (code-only):")
        print(f"  HipYaw:   Kp={KP_HIPYAW:.2f}  Kd={KD_HIPYAW:.2f}")
        print(f"  HipPitch: Kp={KP_HIPPITCH:.2f}  Kd={KD_HIPPITCH:.2f}")
        print(f"  Knee:     Kp={KP_KNEE:.2f}  Kd={KD_KNEE:.2f}\n")

    def cb(self, msg: Joy):
        if ESTOP:
            return

        now = time.monotonic()
        dt = now - self._last_t
        self._last_t = now
        if dt <= 0 or dt > 0.2:
            dt = 0.02

        # -------- E-STOP on Back (button 6), rising edge --------
        back = msg.buttons[BTN_BACK_ESTOP] if BTN_BACK_ESTOP < len(msg.buttons) else 0
        if back == 1 and self._back_prev == 0:
            estop("BACK BUTTON E-STOP")
            return
        self._back_prev = back
        # --------------------------------------------------------

        rx = msg.axes[AX_RX] if AX_RX < len(msg.axes) else 0.0
        ry = msg.axes[AX_RY] if AX_RY < len(msg.axes) else 0.0
        ly = msg.axes[AX_LY] if AX_LY < len(msg.axes) else 0.0

        rx = deadband(rx)
        ry = deadband(-ry)  # invert vertical: stick up = +
        ly = deadband(-ly)

        # integrate common targets
        self.hy = clamp(self.hy + rx * HIPYAW_RATE * dt, *LIM_HIPYAW)
        self.hp = clamp(self.hp + ry * HIPPITCH_RATE * dt, *LIM_HIPPITCH)
        self.kn = clamp(self.kn + ly * KNEE_RATE * dt, *LIM_KNEE)

        # push to all motors by group with per-joint-group Kp/Kd
        for m in allHipYaw:
            set_cmd(m, position=self.hy, velocity=JOY_VEL, torque=JOY_TAU, kp=KP_HIPYAW, kd=KD_HIPYAW)
        for m in allHipPitch:
            set_cmd(m, position=self.hp, velocity=JOY_VEL, torque=JOY_TAU, kp=KP_HIPPITCH, kd=KD_HIPPITCH)
        for m in allKnee:
            set_cmd(m, position=self.kn, velocity=JOY_VEL, torque=JOY_TAU, kp=KP_KNEE, kd=KD_KNEE)

def ros_spin_thread():
    rclpy.init(args=None)
    node = JellyfishJoy()
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
# STARTUP: stop everyone, enable all, zero all, then controller
# -------------------------------------------------------
print("[START] Stopping ALL motors.")
for m in allMotors:
    try:
        m.stop()
    except Exception:
        pass

print("[START] Setting mode + enabling ALL motors.")
for m in allMotors:
    m.mode(RunMode.OPERATION_CONTROL)
for m in allMotors:
    m.enable()

_ = snapshot_positions(timeout_s=0.10)
print("[START] Initial positions captured.")

print("[START] Zeroing ALL motors (Yaw + Pitch + Knee).")
for m in allMotors:
    try:
        m.set_zero_position()
    except Exception:
        pass

time.sleep(0.75)

# seed a hold after zeroing so it doesn't drift
hold = snapshot_positions(timeout_s=0.10)
for m in allMotors:
    set_cmd(m, hold[m.motor_id], velocity=0, torque=0, kp=0.0, kd=6.0)

print("\n[START] Starting controller system...")
JOY_PROC = start_joy_node()
time.sleep(1.0)
threading.Thread(target=ros_spin_thread, daemon=True).start()

print("\n=== READY: JELLYFISH MODE ===")
print("All legs move in unison.")
print("Right stick LR=HipYaw, Right stick UD=HipPitch, Left stick UD=Knees")
print("E-STOP: Back button (button 6) OR type 'e' + Enter\n")

try:
    while True:
        time.sleep(0.25)
except KeyboardInterrupt:
    estop("CTRL+C")