#!/usr/bin/env python3
# ONLY ONE LEG TEST: Streamer + Keyboard E-Stop + Controller control of ONE leg
# No standing sequence. No enabling other legs. No commands sent to other legs.

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
        # SIGTERM avoids rclcpp SIGINT handler weirdness in some setups
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
# Motors (same mapping as your stand script)
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

print(f"\n=== ONE LEG TEST MODE ===")
print(f"Leg {TEST_LEG_NUM}: HipYaw={TEST_HIPYAW.motor_id}, HipPitch={TEST_HIPPITCH.motor_id}, Knee={TEST_KNEE.motor_id}")
print("NO other motors will be enabled or commanded.\n")

# -------------------------------------------------------
# RX dispatch (ok to keep; it doesn't command anything)
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
def snapshot_positions(timeout_s=0.08):
    for m in ACTIVE_MOTORS:
        try:
            m.request_motor_state()
        except Exception:
            pass
    time.sleep(timeout_s)

    pos = {}
    for m in ACTIVE_MOTORS:
        p = m.state.get("position")
        pos[m.motor_id] = 0.0 if p is None else p
    return pos

# -------------------------------------------------------
# VIRTUAL E-STOP (Keyboard + Back button will call this)
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

    for m in ACTIVE_MOTORS:
        set_cmd(m, hold[m.motor_id], velocity=0, torque=0, kp=0.0, kd=6.0)
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
# Live telemetry thread (streams ACTIVE motor positions)
# -------------------------------------------------------
TELEM_ON = True
TELEM_HZ = 10.0  # safe rate

def telemetry_loop():
    period = 1.0 / TELEM_HZ
    while TELEM_ON and not ESTOP:
        for m in ACTIVE_MOTORS:
            try:
                m.request_motor_state()
            except Exception:
                pass
        time.sleep(0.03)

        hy = TEST_HIPYAW.state.get("position")
        hp = TEST_HIPPITCH.state.get("position")
        kn = TEST_KNEE.state.get("position")

        hyv = TEST_HIPYAW.state.get("velocity")
        hpv = TEST_HIPPITCH.state.get("velocity")
        knv = TEST_KNEE.state.get("velocity")

        hyt = TEST_HIPYAW.state.get("torque")
        hpt = TEST_HIPPITCH.state.get("torque")
        knt = TEST_KNEE.state.get("torque")

        def f(x):
            return " None" if x is None else f"{x:+.3f}"

        line = (
            f"\rPOS hy{f(hy)} hp{f(hp)} kn{f(kn)} | "
            f"VEL hy{f(hyv)} hp{f(hpv)} kn{f(knv)} | "
            f"TAU hy{f(hyt)} hp{f(hpt)} kn{f(knt)}"
        )
        sys.stdout.write(line)
        sys.stdout.flush()
        time.sleep(max(0.0, period - 0.03))

threading.Thread(target=telemetry_loop, daemon=True).start()

# -------------------------------------------------------
# ROS2 Joy -> control one leg (set_cmd), Back button -> estop
#
# CONFIRMED mapping (from your working tester):
# Axes:
#   0: Left Joy X
#   1: Left Joy Y
#   2: Left Trigger
#   3: Right Joy X
#   4: Right Joy Y
#   5: Right Trigger
#   6: D-pad X
#   7: D-pad Y
# Buttons:
#   Back: button 6
# -------------------------------------------------------
AX_LY = 1   # Left Joy Y -> Knee
AX_RX = 3   # Right Joy X -> HipYaw
AX_RY = 4   # Right Joy Y -> HipPitch

BTN_BACK_ESTOP = 6  # Back button

DEADBAND = 0.08

HIPYAW_RATE   = 1.0
HIPPITCH_RATE = 1.0
KNEE_RATE     = 1.0

LIM_HIPYAW   = (-1.0,  1.0)
LIM_HIPPITCH = (0.0,  4.9)
LIM_KNEE     = (0.0,  2.35)

JOY_KP = 3.0
JOY_KD = 1.5
JOY_TAU = 0.0
JOY_VEL = 0.0

def clamp(x, lo, hi):
    return lo if x < lo else (hi if x > hi else x)

def deadband(x):
    return 0.0 if abs(x) < DEADBAND else x

class OneLegJoy(Node):
    def __init__(self):
        super().__init__('one_leg_joy')
        self.sub = self.create_subscription(Joy, '/joy', self.cb, 10)
        self._last_t = time.monotonic()
        self._back_prev = 0

        hold = snapshot_positions(timeout_s=0.10)
        self.hy = clamp(hold.get(TEST_HIPYAW.motor_id, 0.0), *LIM_HIPYAW)
        self.hp = clamp(hold.get(TEST_HIPPITCH.motor_id, 0.0), *LIM_HIPPITCH)
        self.kn = clamp(hold.get(TEST_KNEE.motor_id, 0.0), *LIM_KNEE)

        set_cmd(TEST_HIPYAW,   self.hy, velocity=0, torque=0, kp=0.0, kd=6.0)
        set_cmd(TEST_HIPPITCH, self.hp, velocity=0, torque=0, kp=0.0, kd=6.0)
        set_cmd(TEST_KNEE,     self.kn, velocity=0, torque=0, kp=0.0, kd=6.0)

        print("\n\n[JOY] Ready. Control mapping:")
        print("  Right stick L/R -> HipYaw   (axis 3)")
        print("  Right stick U/D -> HipPitch (axis 4)")
        print("  Left  stick U/D -> Knee     (axis 1)")
        print("  Back button     -> E-STOP   (button 6)")
        print("  Keyboard: type 'e' + Enter -> E-STOP\n")

    def cb(self, msg: Joy):
        if ESTOP:
            return

        now = time.monotonic()
        dt = now - self._last_t
        self._last_t = now
        if dt <= 0 or dt > 0.2:
            dt = 0.02

        # E-STOP on Back button (rising edge, single press)
        back = msg.buttons[BTN_BACK_ESTOP] if BTN_BACK_ESTOP < len(msg.buttons) else 0
        if back == 1 and self._back_prev == 0:
            estop("BACK BUTTON E-STOP")
            return
        self._back_prev = back

        rx = msg.axes[AX_RX] if AX_RX < len(msg.axes) else 0.0
        ry = msg.axes[AX_RY] if AX_RY < len(msg.axes) else 0.0
        ly = msg.axes[AX_LY] if AX_LY < len(msg.axes) else 0.0

        # invert verticals so stick up = positive
        rx = deadband(rx)
        ry = deadband(-ry)
        ly = deadband(-ly)

        self.hy = clamp(self.hy + rx * HIPYAW_RATE * dt, *LIM_HIPYAW)
        self.hp = clamp(self.hp + ry * HIPPITCH_RATE * dt, *LIM_HIPPITCH)
        self.kn = clamp(self.kn + ly * KNEE_RATE * dt, *LIM_KNEE)

        set_cmd(TEST_HIPYAW,   position=self.hy, velocity=JOY_VEL, torque=JOY_TAU, kp=JOY_KP, kd=JOY_KD)
        set_cmd(TEST_HIPPITCH, position=self.hp, velocity=JOY_VEL, torque=JOY_TAU, kp=JOY_KP, kd=JOY_KD)
        set_cmd(TEST_KNEE,     position=self.kn, velocity=JOY_VEL, torque=JOY_TAU, kp=JOY_KP, kd=JOY_KD)

def ros_spin_thread():
    rclpy.init(args=None)
    node = OneLegJoy()
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

initial = snapshot_positions(timeout_s=0.10)
for mid, p in initial.items():
    print(f"[START] Active Motor ID {mid} Init Pos: {p}")

print("[START] Zeroing ONLY the selected leg motors.")
for m in ACTIVE_MOTORS:
    try:
        m.set_zero_position()
    except Exception:
        pass

time.sleep(0.5)

hold = snapshot_positions(timeout_s=0.10)
hy0 = clamp(hold.get(TEST_HIPYAW.motor_id, 0.0), *LIM_HIPYAW)
hp0 = clamp(hold.get(TEST_HIPPITCH.motor_id, 0.0), *LIM_HIPPITCH)
kn0 = clamp(hold.get(TEST_KNEE.motor_id, 0.0), *LIM_KNEE)

set_cmd(TEST_HIPYAW,   hy0, velocity=0, torque=0, kp=0.0, kd=6.0)
set_cmd(TEST_HIPPITCH, hp0, velocity=0, torque=0, kp=0.0, kd=6.0)
set_cmd(TEST_KNEE,     kn0, velocity=0, torque=0, kp=0.0, kd=6.0)

print("\n[START] Starting controller system...")
JOY_PROC = start_joy_node()
time.sleep(1.0)
threading.Thread(target=ros_spin_thread, daemon=True).start()

print("\n=== READY ===")
print("ONLY the selected leg is enabled and will move.")
print("Controls: Right stick LR=HipYaw (axis 3), Right stick UD=HipPitch (axis 4), Left stick UD=Knee (axis 1)")
print("E-STOP: Back button (button 6) OR type 'e' + Enter\n")

try:
    while True:
        time.sleep(0.25)
except KeyboardInterrupt:
    estop("CTRL+C")