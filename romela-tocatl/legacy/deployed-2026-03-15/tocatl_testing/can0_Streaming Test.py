import can, time, subprocess, threading
from CyberGearDriver import CyberGearMotor, RunMode, CyberMotorMessage
def setup_can0():
 # Always bring interface down first
    subprocess.run(["sudo", "ip", "link", "set", "can0", "down"], stderr=subprocess.DEVNULL)

    # Reconfigure bitrate
    subprocess.run(["sudo", "ip", "link", "set", "can0", "type", "can", "bitrate", "1000000"], check=True)

    # Bring up interface
    subprocess.run(["sudo", "ip", "link", "set", "can0", "up"], check=True)

    # Verify interface is up
    status = subprocess.run(["ifconfig", "can0"], capture_output=True, text=True).stdout
    print("\n--- CAN0 STATUS ---")
    print(status)
    print("-------------------\n")


setup_can0()


# -------------------------------------------------------
# CAN Setup
# -------------------------------------------------------
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

# -------------------------------------------------------
# Create motors
# -------------------------------------------------------
hipYaw = CyberGearMotor(motor_id=41, send_message=send_message)
hipPitch = CyberGearMotor(motor_id=42, send_message=send_message)
knee = CyberGearMotor(motor_id=43, send_message=send_message)


motors = [hipYaw, hipPitch, knee]


# -------------------------------------------------------
# Single dispatcher (correct)
# -------------------------------------------------------

def dispatch_message(msg):
    for m in motors:
        m.message_received(msg)


notifier = can.Notifier(bus, [dispatch_message])


# -------------------------------------------------------
# Blocking state read
# -------------------------------------------------------
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


# -------------------------------------------------------
# Set motor modes
# -------------------------------------------------------
hipPitch.mode(RunMode.OPERATION_CONTROL)
knee.mode(RunMode.OPERATION_CONTROL)
hipYaw.mode(RunMode.OPERATION_CONTROL)

hipPitch.enable()
knee.enable()
hipYaw.enable()

time.sleep(0.05)


# -------------------------------------------------------
# Keyboard listener thread
# -------------------------------------------------------
stop_flag = False

def wait_for_enter():
    global stop_flag
    input("\nPress ENTER to stop...\n")
    stop_flag = True


listener = threading.Thread(target=wait_for_enter)
listener.daemon = True
listener.start()


# -------------------------------------------------------
# Continuous streaming loop
# -------------------------------------------------------
print("Streaming motor positions...\n")

while not stop_flag:

    hipPitch_pos = get_position_blocking(hipPitch)
    knee_pos = get_position_blocking(knee)
    hipYaw_pos = get_position_blocking(hipYaw)

    print(
        f" HipYaw: {hipYaw_pos:8.3f}"
        f" HipPitch: {hipPitch_pos:8.3f} |"
        f" Knee: {knee_pos:8.3f} |"
    )

    time.sleep(0.05)  # ~20 Hz update rate


hipPitch.stop()
knee.stop()
hipYaw.stop()
notifier.stop()
bus.shutdown()



