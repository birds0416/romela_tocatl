import can, time, subprocess, threading, sys
from CyberGearDriver import CyberGearMotor, RunMode, CyberMotorMessage

# -------------------------------------------------------
# CAN bring-up
# -------------------------------------------------------
def setup_can0():
    subprocess.run(["sudo", "ip", "link", "set", "can0", "down"],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    subprocess.run(["sudo", "ip", "link", "set", "can0", "type", "can", "bitrate", "1000000"])
    subprocess.run(["sudo", "ip", "link", "set", "can0", "up"])

setup_can0()

bus = can.interface.Bus(
    interface='socketcan',
    channel='can0',
    bitrate=1000000,
)

# -------------------------------------------------------
# Send function
# -------------------------------------------------------
def send_message(message: CyberMotorMessage):
    bus.send(
        can.Message(
            arbitration_id=message.arbitration_id,
            data=message.data,
            is_extended_id=message.is_extended_id,
        )
    )

# -------------------------------------------------------
# Create motors
# -------------------------------------------------------
hipYaw   = CyberGearMotor(61, send_message)
hipPitch = CyberGearMotor(62, send_message)
knee     = CyberGearMotor(63, send_message)

motors = [hipYaw, hipPitch, knee]

# -------------------------------------------------------
# CAN → motor dispatcher
# -------------------------------------------------------
def dispatch(msg):
    for m in motors:
        m.message_received(msg)

notifier = can.Notifier(bus, [dispatch])

# -------------------------------------------------------
# State request helper
# -------------------------------------------------------
def get_pos(motor, timeout=0.25):
    """Request motor state and block until position arrives or timeout."""
    motor.state["position"] = None
    motor.request_motor_state()
    t0 = time.time()
    while time.time() - t0 < timeout:
        p = motor.state.get("position")
        if p is not None:
            return p
        time.sleep(0.002)
    return None

# -------------------------------------------------------
# Persistent Zero Function (writes MechPos_init)
# -------------------------------------------------------
def write_persistent_zero(motor):
    """
    Sets MechPos_init = current mechPos value.
    This is PERMANENT and survives power cycling.
    Manual parameter index 0x2006 (MechPos_init) (page 23–24) :contentReference[oaicite:1]{index=1}
    """
    # Request mechanical position (true absolute mechanical angle)
    motor.request_parameter("mechPos")
    time.sleep(0.05)

    mech_angle = motor.params.get("mechPos")
    if mech_angle is None:
        print(f"[ERROR] Could not read mechPos for motor {motor.motor_id}")
        return False

    print(f" → Writing MechPos_init = {mech_angle:.4f} for motor {motor.motor_id}")
    motor.set_parameter("MechPos_init", float(mech_angle))
    time.sleep(0.05)
    return True

# -------------------------------------------------------
# 1. Confirm all motors respond
# -------------------------------------------------------
print("Checking motors...\n")

results = {}
for m in motors:
    pos = get_pos(m)
    results[m.motor_id] = (pos is not None)
    status = "YES" if pos is not None else "NO"
    print(f" Motor {m.motor_id:2d} responding?  {status}")

# If any motor failed → stop script
if not all(results.values()):
    print("\n[ERROR] One or more motors are not responding — shutting down.")
    bus.shutdown()
    sys.exit(1)

print("\nAll motors OK.")

# -------------------------------------------------------
# 2. Send motors into control mode
# -------------------------------------------------------
for m in motors:
    m.mode(RunMode.OPERATION_CONTROL)
time.sleep(0.05)

# -------------------------------------------------------
# 3. Zero motors permanently (MechPos_init)
# -------------------------------------------------------
print("\nWriting persistent zero to all motors...")
for m in motors:
    if not write_persistent_zero(m):
        print(f"[ERROR] Failed to write zero for motor {m.motor_id}. Aborting.")
        bus.shutdown()
        sys.exit(1)

print("\nPersistent zero completed for all motors.")

# -------------------------------------------------------
# 4. Start streaming loop until ENTER is pressed
# -------------------------------------------------------
stop_flag = False

def wait_for_enter():
    global stop_flag
    input("\nPress ENTER to stop streaming...\n")
    stop_flag = True

threading.Thread(target=wait_for_enter, daemon=True).start()

print("\nStreaming motor positions...\n")
while not stop_flag:
    p1 = get_pos(hipPitch)
    p2 = get_pos(knee)
    p3 = get_pos(hipYaw)

    print(
        f" hipPitch: {p1:8.3f} |"
        f" knee: {p2:8.3f} |"
        f" hipYaw: {p3:8.3f}"
    )
    time.sleep(0.05)

bus.shutdown()
print("Stopped.")
