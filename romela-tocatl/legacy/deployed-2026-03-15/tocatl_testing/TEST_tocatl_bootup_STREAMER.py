import can, time, subprocess, threading
import sys, os, select
from CyberGearDriver import CyberGearMotor, RunMode, CyberMotorMessage


# -------------------------------------------------------
# CAN bring-up
# -------------------------------------------------------
def setup_can0():
    print("\nSetting up Interface for Can 0...\n")
    subprocess.run(["sudo", "ip", "link", "set", "can0", "down"], stderr=subprocess.DEVNULL)
    subprocess.run(["sudo", "ip", "link", "set", "can0", "type", "can", "bitrate", "1000000"], check=True)
    subprocess.run(["sudo", "ip", "link", "set", "can0", "up"], check=True)
    status0 = subprocess.run(["ifconfig", "can0"], capture_output=True, text=True).stdout
    print("\n--- CAN0 STATUS ---")
    print(status0)
    print("-------------------\n")

def setup_can1():
    print("\nSetting up Interface for Can 1...\n")
    subprocess.run(["sudo", "ip", "link", "set", "can1", "down"], stderr=subprocess.DEVNULL)
    subprocess.run(["sudo", "ip", "link", "set", "can1", "type", "can", "bitrate", "1000000"], check=True)
    subprocess.run(["sudo", "ip", "link", "set", "can1", "up"], check=True)
    status1 = subprocess.run(["ifconfig", "can1"], capture_output=True, text=True).stdout
    print("\n--- CAN1 STATUS ---")
    print(status1)
    print("-------------------\n")


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
        timeout=0.01,   # <-- backpressure, avoids "lost in the mess"
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

allHipYaw = [leg1hipYaw, leg2hipYaw, leg3hipYaw, leg4hipYaw, leg5hipYaw, leg6hipYaw]
allHipPitch = [leg1hipPitch, leg2hipPitch, leg3hipPitch, leg4hipPitch, leg5hipPitch, leg6hipPitch]
allKnee = [leg1knee, leg2knee, leg3knee, leg4knee, leg5knee, leg6knee]

leg1 = [leg1hipYaw, leg1hipPitch, leg1knee]
leg2 = [leg2hipYaw, leg2hipPitch, leg2knee]
leg3 = [leg3hipYaw, leg3hipPitch, leg3knee]
leg4 = [leg4hipYaw, leg4hipPitch, leg4knee]
leg5 = [leg5hipYaw, leg5hipPitch, leg5knee]
leg6 = [leg6hipYaw, leg6hipPitch, leg6knee]


# -------------------------------------------------------
# RX dispatch
# -------------------------------------------------------
def dispatch_message0(msg):
    for m in leftSideMotors:
        m.message_received(msg)

notifier0 = can.Notifier(bus0, [dispatch_message0])

def dispatch_message1(msg):
    for m in rightSideMotors:
        m.message_received(msg)

notifier1 = can.Notifier(bus1, [dispatch_message1])


# -------------------------------------------------------
# Streamer 
# -------------------------------------------------------
cmd_lock = threading.Lock()
desired = {}         # motor_id -> (pos, vel, tau, kp, kd)
STREAM_ON = True
STREAM_HZ = 150      # start here; bump to 200 later if stable

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

        # send all motors each tick (no sleeps here)
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
    # request state from everyone
    for m in allMotors:
        try:
            m.request_motor_state()
        except Exception:
            pass
    time.sleep(timeout_s)  # let notifiers fill in

    pos = {}
    for m in allMotors:
        p = m.state.get("position")
        pos[m.motor_id] = 0.0 if p is None else p
    return pos


# -------------------------------------------------------
# VIRTUAL E-STOP
# -------------------------------------------------------
ESTOP = False

def estop(reason="KEYBOARD E-STOP"):
    global ESTOP
    if ESTOP:
        return
    ESTOP = True

    print("\n\n==============================")
    print(f"!!! {reason} ACTIVATED !!!")
    print("==============================\n")

    # freeze where you are, then damp, then stop
    hold = snapshot_positions(timeout_s=0.05)

    for m in allMotors:
        set_cmd(m, hold[m.motor_id], velocity=0, torque=0, kp=0.0, kd=6.0)
    time.sleep(0.25)

    # stop streamer FIRST so no more control frames get sent
    stop_streamer()
    time.sleep(0.05)

    # now stop motors
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

    print("Motors stopped, CAN shutdown. Exiting now.")
    os._exit(1)

def keyboard_estop_listener():
    print("\nVirtual E-Stop ARMED: type 'e' + Enter (or 'q') at any time.\n")
    while True:
        r, _, _ = select.select([sys.stdin], [], [], 0.05)
        if r:
            key = sys.stdin.readline().strip().lower()
            if key in ("e", "q", "estop", "stop"):
                estop("VIRTUAL E-STOP")

threading.Thread(target=keyboard_estop_listener, daemon=True).start()


# -------------------------------------------------------
# Startup
# -------------------------------------------------------
for m in allMotors:
    m.stop()

for m in allMotors:
    m.mode(RunMode.OPERATION_CONTROL)

for m in allMotors:
    m.enable()

# init readout (optional; now done with one snapshot)
initial_positions = snapshot_positions(timeout_s=0.10)
for mid, p in initial_positions.items():
    print(f"Motor ID {mid} Init Pos:", p)

# zeroing
for m in allHipPitch + allKnee:
    m.set_zero_position()

for m in allHipYaw:
    m.set_zero_position()


time.sleep(1)

# -------------------------------------------------------
# Deploying Legs: now done via set_cmd + streamer
# -------------------------------------------------------
for leg in [leg1, leg2, leg3, leg4, leg5, leg6]:
    set_cmd(leg[1], position=1.40, velocity=0, torque=0, kp=3, kd=1)   # hipPitch
    set_cmd(leg[2], position=-0.6, velocity=0, torque=0, kp=3, kd=1)   # knee
    time.sleep(0.25)

time.sleep(2)

print("\nLocking Hip Yaw\n")

for m in allHipYaw:
    set_cmd(m, position=0, velocity=0, torque=0, kp=5, kd=3)

print("\nHip Yaw Locked!\n")
time.sleep(2)

print("Increasing torque to 8.5 N-m in 3...")
print("\n2...\n"); time.sleep(1)
print("\n1...\n"); time.sleep(1)

for leg in [leg1, leg2, leg3, leg4, leg5, leg6]:
    set_cmd(leg[1], position=1.70, velocity=0, torque=8.5, kp=3, kd=2)
    set_cmd(leg[2], position=-0.6, velocity=0, torque=0, kp=3, kd=2)

time.sleep(7.5)


# -------------------------------------------------------
# Shutdown 
# -------------------------------------------------------
print("Shutting Down")

hold = snapshot_positions(timeout_s=0.06)

# hold briefly (prevents sag/jump), then damp
for m in allMotors:
    set_cmd(m, hold[m.motor_id], velocity=0, torque=0, kp=0.0, kd=7.0)
time.sleep(2)

for leg in [leg1, leg2, leg3, leg4, leg5, leg6]:
    set_cmd(leg[1], position=0, velocity=0, torque=0, kp=3, kd=1)   # hipPitch
    set_cmd(leg[2], position=0, velocity=0, torque=0, kp=3, kd=1)   # knee
    time.sleep(0.25)

time.sleep(2)

# stop streamer first so it doesn't fight your stop()
stop_streamer()
time.sleep(0.05)

for m in allMotors:
    try:
        m.stop()
    except Exception:
        pass

time.sleep(0.1)
bus0.shutdown()
bus1.shutdown()
