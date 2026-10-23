import can, time, subprocess, threading, sys
from CyberGearDriver import CyberGearMotor, RunMode, CyberMotorMessage

# ------------------------------
# CAN bring-up
# ------------------------------
def setup_can0():
    subprocess.run(["sudo", "ip", "link", "set", "can0", "down"], stderr=subprocess.DEVNULL)
    subprocess.run(["sudo", "ip", "link", "set", "can0", "type", "can", "bitrate", "1000000"], check=True)
    subprocess.run(["sudo", "ip", "link", "set", "can0", "up"], check=True)

    status = subprocess.run(["ifconfig", "can0"], capture_output=True, text=True).stdout
    print("\n--- CAN0 STATUS ---")
    print(status)
    print("-------------------\n")


setup_can0()

# ------------------------------
# CAN Setup
# ------------------------------
bus = can.interface.Bus(
    interface='socketcan',
    channel='can0',
    bitrate=1000000,
)

def send_message(message: CyberMotorMessage):
    bus.send(
        can.Message(
            arbitration_id=message.arbitration_id,
            data=message.data,
            is_extended_id=message.is_extended_id,
        )
    )

# ------------------------------
# Create motors (leg)
# ------------------------------
hipYaw   = CyberGearMotor(motor_id=21, send_message=send_message)
hipPitch = CyberGearMotor(motor_id=22, send_message=send_message)
knee     = CyberGearMotor(motor_id=23, send_message=send_message)

motors = [hipYaw, hipPitch, knee]

motor_names = {
    hipYaw:   "HipYaw (ID 21)",
    hipPitch: "HipPitch (ID 22)",
    knee:     "Knee (ID 23)",
}

# ------------------------------
# Single dispatcher
# ------------------------------
def dispatch_message(msg):
    for m in motors:
        m.message_received(msg)

notifier = can.Notifier(bus, [dispatch_message])

# ------------------------------
# Blocking state read
# ------------------------------
def get_position_blocking(motor, timeout=0.2):
    motor.state["position"] = None
    motor.request_motor_state()
    start = time.time()

    while time.time() - start < timeout:
        pos = motor.state.get("position")
        if pos is not None:
            return pos
        time.sleep(0.002)

    return None

def detect_motor(motor, attempts=3, timeout=0.2):
    # returns True if it answers at least once
    for _ in range(attempts):
        pos = get_position_blocking(motor, timeout=timeout)
        if pos is not None:
            return True
        time.sleep(0.02)
    return False

# ------------------------------
# Safe shutdown helpers
# ------------------------------
def safe_stop_disable_all():
    # We try best-effort on every motor; never let one failure prevent the rest.
    for m in motors:
        try:
            m.stop()
        except Exception:
            pass
    # If your driver has disable(), use it. If not, this will just be skipped.
    for m in motors:
        try:
            m.disable()
        except Exception:
            pass

def safe_shutdown_can():
    try:
        notifier.stop()
    except Exception:
        pass
    try:
        bus.shutdown()
    except Exception:
        pass

# ------------------------------
# Keyboard listener
# ------------------------------
stop_flag = False
def wait_for_enter():
    global stop_flag
    input("\nPress ENTER to stop...\n")
    stop_flag = True

listener = threading.Thread(target=wait_for_enter, daemon=True)
listener.start()

# ------------------------------
# Main
# ------------------------------
try:
    # Put motors in mode + enable (best-effort)
    for m in motors:
        try:
            m.mode(RunMode.OPERATION_CONTROL)
        except Exception as e:
            print(f"[WARN] Failed to set mode for {motor_names.get(m,'Unknown motor')}: {e}")

    for m in motors:
        try:
            m.enable()
        except Exception as e:
            print(f"[WARN] Failed to enable {motor_names.get(m,'Unknown motor')}: {e}")

    time.sleep(0.05)

    # Detect presence
    missing = []
    for m in motors:
        ok = False
        try:
            ok = detect_motor(m, attempts=3, timeout=0.2)
        except Exception:
            ok = False
        if not ok:
            missing.append(motor_names.get(m, "Unknown motor"))

    if missing:
        print("\n[ERROR] Motor(s) not detected on this leg:")
        for name in missing:
            print(f"  - {name}")
        print("[ACTION] Stopping/disabling ALL motors in this leg.\n")
        safe_stop_disable_all()
        # exit cleanly
        sys.exit(1)

    print("Streaming motor positions...\n")

    while not stop_flag:
        # Note: any of these can still be None if a motor starts failing mid-run,
        # so format defensively.
        hy = get_position_blocking(hipYaw)
        hp = get_position_blocking(hipPitch)
        kn = get_position_blocking(knee)

        def fmt(x):
            return f"{x:8.3f}" if x is not None else "   None  "

        print(
            f" HipYaw: {fmt(hy)} |"
            f" HipPitch: {fmt(hp)} |"
            f" Knee: {fmt(kn)} |"
        )

        time.sleep(0.05)

except KeyboardInterrupt:
    print("\nKeyboardInterrupt - stopping...\n")

except Exception as e:
    print(f"\n[ERROR] Unhandled exception: {e}")
    print("[ACTION] Stopping/disabling ALL motors in this leg.\n")
    safe_stop_disable_all()
    raise

finally:
    # Always stop motors and shutdown CAN, no matter what happened.
    safe_stop_disable_all()
    safe_shutdown_can()
    print("Clean shutdown complete.")