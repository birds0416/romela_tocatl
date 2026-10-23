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
# Create motors and groups
# -------------------------------------------------------
leg1hipYaw = CyberGearMotor(motor_id=11, send_message=send_message)
leg1hipPitch = CyberGearMotor(motor_id=12, send_message=send_message)
leg1knee = CyberGearMotor(motor_id=13, send_message=send_message)

leg2hipYaw = CyberGearMotor(motor_id=21, send_message=send_message)
leg2hipPitch = CyberGearMotor(motor_id=22, send_message=send_message)
leg2knee = CyberGearMotor(motor_id=23, send_message=send_message)

leg3hipYaw = CyberGearMotor(motor_id=31, send_message=send_message)
leg3hipPitch = CyberGearMotor(motor_id=32, send_message=send_message)
leg3knee = CyberGearMotor(motor_id=33, send_message=send_message)

leg4hipYaw = CyberGearMotor(motor_id=41, send_message=send_message)
leg4hipPitch = CyberGearMotor(motor_id=42, send_message=send_message)
leg4knee = CyberGearMotor(motor_id=43, send_message=send_message)

leg5hipYaw = CyberGearMotor(motor_id=51, send_message=send_message)
leg5hipPitch = CyberGearMotor(motor_id=52, send_message=send_message)
leg5knee = CyberGearMotor(motor_id=53, send_message=send_message)


leg6hipYaw = CyberGearMotor(motor_id=61, send_message=send_message)
leg6hipPitch = CyberGearMotor(motor_id=62, send_message=send_message)
leg6knee = CyberGearMotor(motor_id=63, send_message=send_message)

allMotors = [  leg1hipYaw, leg1hipPitch, leg1knee,
            leg2hipYaw, leg2hipPitch, leg2knee,
            leg3hipYaw, leg3hipPitch, leg3knee,
            leg4hipYaw, leg4hipPitch, leg4knee,
            leg5hipYaw, leg5hipPitch, leg5knee,
            leg6hipYaw, leg6hipPitch, leg6knee]

rightSideMotors= [ leg1hipYaw, leg1hipPitch, leg1knee,
                leg5hipYaw, leg5hipPitch, leg5knee,
                leg6hipYaw, leg6hipPitch, leg6knee]

leftSideMotors= [ leg2hipYaw, leg2hipPitch, leg2knee,
                leg3hipYaw, leg3hipPitch, leg3knee,
                leg4hipYaw, leg4hipPitch, leg4knee]

allHipYaw= [ leg1hipYaw, leg2hipYaw, leg3hipYaw,
            leg4hipYaw, leg5hipYaw, leg6hipYaw]

allHipPitch= [ leg1hipPitch, leg2hipPitch, leg3hipPitch,
            leg4hipPitch, leg5hipPitch, leg6hipPitch]

allKnee= [ leg1knee, leg2knee, leg3knee,
        leg4knee, leg5knee, leg6knee]

leg1= [leg1hipYaw, leg1hipPitch, leg1knee]
leg2= [leg2hipYaw, leg2hipPitch, leg2knee]
leg3= [leg3hipYaw, leg3hipPitch, leg3knee]
leg4= [leg4hipYaw, leg4hipPitch, leg4knee]
leg5= [leg5hipYaw, leg5hipPitch, leg5knee]
leg6= [leg6hipYaw, leg6hipPitch, leg6knee]
# -------------------------------------------------------
# Variables
# -------------------------------------------------------




# -------------------------------------------------------
# Dispatcher
# -------------------------------------------------------

def dispatch_message(msg):
    for m in allMotors:
        m.message_received(msg)


notifier = can.Notifier(bus, [dispatch_message])

# -------------------------------------------------------
# Set motor modes
# -------------------------------------------------------

leg2hipYaw.mode(RunMode.OPERATION_CONTROL)
leg2hipPitch.mode(RunMode.OPERATION_CONTROL)
leg2knee.mode(RunMode.OPERATION_CONTROL)
time.sleep(0.01)

# -------------------------------------------------------
# enable Hip Pitch and Yaw motors
# -------------------------------------------------------
leg2hipYaw.enable()
leg2hipPitch.enable()
time.sleep(0.01)

# -------------------------------------------------------
# Initialization Sequence
# -------------------------------------------------------

#Showing Initial Positions
leg2hipYawInitPos=leg2hipYaw.state.get("position")
leg2hipPitchInitPos=leg2hipPitch.state.get("position")
print("Leg2 Hip Yaw Current Pos:", leg2hipYawInitPos)
print("Leg2 Hip Pitch Current Pos:", leg2hipPitchInitPos)
time.sleep(3)
subprocess.run(["clear"])


#Zeroing Hip Yaw
print("Lifting Leg2 Hip Pitch Slightly...")
leg2hipPitch.control(position=leg2hipPitchInitPos-0.2, velocity=0, torque=0, kp=1.75, kd=1)
time.sleep(3)
subprocess.run(["clear"])

print("Rotating Hip Right for Bonking")
leg2hipYaw.control(position=leg2hipYawInitPos+2.5, velocity=0, torque=0, kp=1.75, kd=1)
time.sleep(3)
subprocess.run(["clear"])

print("Zeroing at Bonk")
leg2hipYaw.stop()
time.sleep(0.01)
leg2hipYaw.set_zero_position()
time.sleep(0.01)
leg2hipYaw.enable()
time.sleep(0.01)
subprocess.run(["clear"])

print("Moving to Center")
leg2hipYaw.control(position=-1.118, velocity=0, torque=0, kp=1.75, kd=0.1)
time.sleep(3)
subprocess.run(["clear"])

print("Zeroing Hip at Center")
leg2hipYaw.stop()
time.sleep(0.01)
leg2hipYaw.set_zero_position()
time.sleep(0.01)
leg2hipYaw.enable()
time.sleep(0.01)
subprocess.run(["clear"])

#Zeroing Hip Pitch
leg2hipPitchInitPos=leg2hipPitch.state.get("position")
print("Lifting Hip Up for Bonking")
leg2hipPitch.control(position=leg2hipPitchInitPos-1.5, velocity=0, torque=0, kp=1.75, kd=2)
time.sleep(3)
subprocess.run(["clear"])

print("Zeroing Hip and Knee at Shell Contact")
leg2hipPitch.stop()
time.sleep(0.01)
leg2hipPitch.set_zero_position()
time.sleep(0.01)
leg2hipPitch.enable()
time.sleep(0.01)
subprocess.run(["clear"])

#Zeroing Knee
leg2hipPitch.enable()
time.sleep(0.01)
leg2knee.stop()
time.sleep(0.01)
leg2knee.set_zero_position()
time.sleep(0.01)
leg2knee.enable()
time.sleep(0.01)
subprocess.run(["clear"])

print("Deploying Leg!")


leg2hipPitch.control(position=0, velocity=0, torque=0, kp=9, kd=0.8)
leg2knee.control(position=-0.859, velocity=0, torque=0, kp=6, kd=0.8)
time.sleep(2)
leg2hipPitch.control(position=1.366, velocity=0, torque=0, kp=3, kd=1)

time.sleep(10)
subprocess.run(["clear"])



# -------------------------------------------------------
# Shutdown
# -------------------------------------------------------
print("Damping...")
leg2hipPitch.control(position=0, velocity=0, torque=0, kp=0.01, kd=1)
leg2hipYaw.control(position=0, velocity=0, torque=0, kp=0.01, kd=1)
leg2knee.control(position=0, velocity=0, torque=0, kp=0.01, kd=1)
time.sleep(2)
subprocess.run(["clear"])

print("Shutting Down")
for motor in allMotors:
    motor.stop()
    time.sleep(0.01)
bus.shutdown()