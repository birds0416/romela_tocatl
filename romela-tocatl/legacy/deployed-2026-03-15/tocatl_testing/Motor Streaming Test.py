import can
import time
import subprocess
import threading
import sys
from CyberGearDriver import CyberGearMotor, RunMode, CyberMotorMessage

# =====================================================
# 1. BRING UP CAN0 FOR GS_USB / CANABLE 2.0 ADAPTER
# =====================================================

print("Configuring can0 for GS_USB / Canable 2.0...")

# Bring down (ignore errors)
subprocess.run(["sudo", "ip", "link", "set", "can0", "down"],
               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

# Configure bitrate + bus-off recovery
subprocess.run([
    "sudo", "ip", "link", "set", "can0",
    "type", "can",
    "bitrate", "250000",
    "restart-ms", "100"
])

# Bring up the interface
subprocess.run(["sudo", "ip", "link", "set", "can0", "up"])

print("can0 UP at 250 kbps.\n")


# =====================================================
# 2. CREATE PYTHON-CAN BUS (SocketCAN)
# =====================================================

bus = can.interface.Bus(
    interface="socketcan",
    channel="can0",
    bitrate=250000
)

def send_message(message: CyberMotorMessage):
    bus.send(can.Message(
        arbitration_id=message.arbitration_id,
        data=message.data,
        is_extended_id=message.is_extended_id,
    ))


# =====================================================
# 3. DEFINE ALL 18 MOTORS
# =====================================================

MOTOR_IDS = [
    11,12,13,
    21,22,23,
    31,32,33,
    41,42,43,
    51,52,53,
    61,62,63
]

motors = [CyberGearMotor(motor_id=id, send_message=send_message)
          for id in MOTOR_IDS]

MOTOR_NAMES = {
    11:"L1_Yaw", 12:"L1_Pitch", 13:"L1_Knee",
    21:"L2_Yaw", 22:"L2_Pitch", 23:"L2_Knee",
    31:"L3_Yaw", 32:"L3_Pitch", 33:"L3_Knee",
    41:"L4_Yaw", 42:"L4_Pitch", 43:"L4_Knee",
    51:"L5_Yaw", 52:"L5_Pitch", 53:"L5_Knee",
    61:"L6_Yaw", 62:"L6_Pitch", 63:"L6_Knee",
}


# =====================================================
# 4. CREATE SHARED NOTIFIER
# =====================================================

def dispatch_message(msg):
    for m in motors:
        m.message_received(msg)

notifier = can.Notifier(bus, [dispatch_message])


# =====================================================
# 5. SET OPERATION MODE
# =====================================================

print("Setting motors to OPERATION_CONTROL...")
for m in motors:
    try:
        m.mode(RunMode.OPERATION_CONTROL)
        time.sleep(0.003)
    except Exception as e:
        print(f"Mode set failed for {m.motor_id}: {e}")

time.sleep(0.05)


# =====================================================
# 6. KEYBOARD STOP THREAD
# =====================================================

stop_flag = False

def wait_for_enter():
    global stop_flag
    input("\nPress ENTER to stop...\n")
    stop_flag = True

threading.Thread(target=wait_for_enter, daemon=True).start()


# =====================================================
# 7. STAGGERED POLLING LOOP (STAR-TOPOLOGY SAFE)
# =====================================================

PRINT_HZ = 20
PRINT_DT = 1.0 / PRINT_HZ

POLL_GROUP = 3
poll_index = 0

def fmt(x):
    return " None " if x is None else f"{x:8.3f}"

print("\nStarting streaming (star-safe mode)...\n")

last_print = time.time()

try:
    while not stop_flag:

        # Stagger requests — 3 motors every tick
        group = motors[poll_index:poll_index + POLL_GROUP]
        for m in group:
            try:
                m.request_motor_state()
            except can.CanOperationError:
                pass  # TX buffer full sometimes — safe to ignore

        poll_index = (poll_index + POLL_GROUP) % len(motors)

        # Print every PRINT_DT
        now = time.time()
        if now - last_print >= PRINT_DT:
            last_print = now

            parts = []
            for m in motors:
                name = MOTOR_NAMES[m.motor_id]
                pos = m.state.get("position")
                parts.append(f"{name}: {fmt(pos)}")

            print(" | ".join(parts))

        time.sleep(0.005)

except KeyboardInterrupt:
    print("\nKeyboardInterrupt detected")

finally:
    print("\nShutting down...")
    notifier.stop()
    bus.shutdown()
    print("Done.")

