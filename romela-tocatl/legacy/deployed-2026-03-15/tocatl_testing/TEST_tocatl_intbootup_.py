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
    status0 = subprocess.run(["ifconfig", "can0"], capture_output=True, text=True).stdout
    print("\n--- CAN0 STATUS ---")
    print(status0)
    print("-------------------\n")

def setup_can1():
 # Always bring interface down first
    subprocess.run(["sudo", "ip", "link", "set", "can1", "down"], stderr=subprocess.DEVNULL)

    # Reconfigure bitrate
    subprocess.run(["sudo", "ip", "link", "set", "can1", "type", "can", "bitrate", "1000000"], check=True)

    # Bring up interface
    subprocess.run(["sudo", "ip", "link", "set", "can1", "up"], check=True)

    # Verify interface is up
    status1 = subprocess.run(["ifconfig", "can1"], capture_output=True, text=True).stdout
    print("\n--- CAN1 STATUS ---")
    print(status1)
    print("-------------------\n")


setup_can0()
setup_can1()

# -------------------------------------------------------
# CAN Setup
# -------------------------------------------------------
bus0 = can.interface.Bus(
    interface='socketcan',
    channel='can0',
    bitrate=1000000,
)


bus1 = can.interface.Bus(
    interface='socketcan',
    channel='can1',
    bitrate=1000000,
)


def send_message0(message: CyberMotorMessage):
    bus0.send(
        can.Message(
            arbitration_id=message.arbitration_id,
            data=message.data,
            is_extended_id=message.is_extended_id,
        )
    )

def send_message1(message: CyberMotorMessage):
    bus1.send(
        can.Message(
            arbitration_id=message.arbitration_id,
            data=message.data,
            is_extended_id=message.is_extended_id,
        )
    )


# -------------------------------------------------------
# Create motors and groups
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

allMotors = [  leg1hipYaw, leg1hipPitch, leg1knee,
            leg2hipYaw, leg2hipPitch, leg2knee,
            leg3hipYaw, leg3hipPitch, leg3knee,
            leg4hipYaw, leg4hipPitch, leg4knee,
            leg5hipYaw, leg5hipPitch, leg5knee,
            leg6hipYaw, leg6hipPitch, leg6knee]

rightSideMotors= [ leg1hipYaw, leg1hipPitch, leg1knee, #right side motors use CAN1
                leg5hipYaw, leg5hipPitch, leg5knee,
                leg6hipYaw, leg6hipPitch, leg6knee]

leftSideMotors= [ leg2hipYaw, leg2hipPitch, leg2knee, #left side motors use CAN0
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

oddYaw= [ leg1hipYaw, leg3hipYaw, leg5hipYaw]
evenYaw= [ leg2hipYaw, leg4hipYaw, leg6hipYaw]

oddPitch= [ leg1hipPitch, leg3hipPitch, leg5hipPitch]
evenPitch= [ leg2hipPitch, leg4hipPitch, leg6hipPitch]

oddKnee= [ leg1knee, leg3knee, leg5knee]
evenKnee= [ leg2knee, leg4knee, leg6knee]
# -------------------------------------------------------
# Variables
# -------------------------------------------------------




# -------------------------------------------------------
# Dispatcher and Notifiers
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
# Kill all motors just in case
# -------------------------------------------------------
for motor in allMotors:
    motor.stop()
    time.sleep(0.05)

# -------------------------------------------------------
# Set motor modes
# -------------------------------------------------------

for motor in allMotors:
    motor.mode(RunMode.OPERATION_CONTROL)
    time.sleep(0.05)


# -------------------------------------------------------
# Enable All Motors
# -------------------------------------------------------
for motor in allMotors:
    motor.enable()
    time.sleep(0.05)

# -------------------------------------------------------
# Initialization Sequence
# -------------------------------------------------------
initial_positions = {}
for motor in allMotors:
    motor.request_motor_state()
    time.sleep(0.05)
    pos = motor.state.get("position")
    initial_positions[motor.motor_id] = pos
    print(f"Motor ID {motor.motor_id} Init Pos:", pos)


# -------------------------------------------------------
# Zero Hip Pitch and Knee Motors
# -------------------------------------------------------

    #   For this to work, ensure that the legs are resting on 
    #   the wire protectors and fully retracted.

for motor in allHipPitch + allKnee:
    motor.set_zero_position()
    time.sleep(0.05)
    

# -------------------------------------------------------
# Physically Zero the Hip Yaw Motors
# -------------------------------------------------------
    #   The hip yaw will collide with the leg shells,
    #   and then use that position to determine the center.



""" (insert Bonk code here)"""

    #   For the sake of this test we are skipping the bonk 
    #   procedure, because the shells arent done yet...oops

for motor in allHipYaw:
    motor.set_zero_position()
    time.sleep(0.05)


# -------------------------------------------------------
# Deploying Legs to Pre-standing Position
# -------------------------------------------------------
for leg in [leg1, leg2, leg3, leg4, leg5, leg6]:
    leg[1].control(position=1.40, velocity=0, torque=0, kp=3, kd=1)  # hipPitch
    leg[2].control(position=-0.6, velocity=0, torque=0, kp=3, kd=1)  # knee
    time.sleep(0.2)

time.sleep(5)
# # -------------------------------------------------------
# # Locking Yaw
# # -------------------------------------------------------

# for motor in allHipYaw:
#     motor.control(position=0, velocity=0, torque=0, kp=5, kd=3)
#     time.sleep(0.05)

# time.sleep(5)
# print("Increasing torque")
# for leg in [leg1, leg2, leg3, leg4, leg5, leg6]:
#     leg[1].control(position=1.170, velocity=0, torque=8.55, kp=3, kd=1)  # hipPitch
#     leg[2].control(position=-0.6, velocity=0, torque=0, kp=3, kd=1)  # knee

# time.sleep(15)


# print("Moving all Hip Pitch motors up slightly...")

# for motor in allHipPitch:
#     init_pos = initial_positions[motor.motor_id]
#     if init_pos is not None:
#         target = init_pos - 0.25
#         motor.control(position=target, velocity=0, torque=0, kp=1.75, kd=0.9)
#         print(f"Motor {motor.motor_id}: {init_pos:.3f} → {target:.3f}")
#         time.sleep(0.18)
#     else:
#         print(f"Motor {motor.motor_id}: No initial position data!")

# time.sleep(2)       

# for motor in allHipYaw:
#     init_pos = initial_positions[motor.motor_id]
#     if init_pos is not None:
#         target = init_pos +2.0
#         motor.control(position=target, velocity=0, torque=0, kp=1, kd=1.1)
#         print(f"Motor {motor.motor_id}: {init_pos:.3f} → {target:.3f}")
#         time.sleep(1)
#     else:
#         print(f"Motor {motor.motor_id}: No initial position data!")


# time.sleep(10)


#

# print("Rotating Hip Right for Bonking")
# leg2hipYaw.control(position=leg2hipYawInitPos+2.5, velocity=0, torque=0, kp=1.75, kd=1)
# time.sleep(3)
# subprocess.run(["clear"])

# print("Zeroing at Bonk")
# leg2hipYaw.stop()
# time.sleep(0.01)
# leg2hipYaw.set_zero_position()
# time.sleep(0.01)
# leg2hipYaw.enable()
# time.sleep(0.01)
# subprocess.run(["clear"])

# print("Moving to Center")
# leg2hipYaw.control(position=-1.118, velocity=0, torque=0, kp=1.75, kd=0.1)
# time.sleep(3)
# subprocess.run(["clear"])

# print("Zeroing Hip at Center")
# leg2hipYaw.stop()
# time.sleep(0.01)
# leg2hipYaw.set_zero_position()
# time.sleep(0.01)
# leg2hipYaw.enable()
# time.sleep(0.01)
# subprocess.run(["clear"])

# #Zeroing Hip Pitch
# leg2hipPitchInitPos=leg2hipPitch.state.get("position")
# print("Lifting Hip Up for Bonking")
# leg2hipPitch.control(position=leg2hipPitchInitPos-1.5, velocity=0, torque=0, kp=1.75, kd=2)
# time.sleep(3)
# subprocess.run(["clear"])

# print("Zeroing Hip and Knee at Shell Contact")
# leg2hipPitch.stop()
# time.sleep(0.01)
# leg2hipPitch.set_zero_position()
# time.sleep(0.01)
# leg2hipPitch.enable()
# time.sleep(0.01)
# subprocess.run(["clear"])

# #Zeroing Knee
# leg2hipPitch.enable()
# time.sleep(0.01)
# leg2knee.stop()
# time.sleep(0.01)
# leg2knee.set_zero_position()
# time.sleep(0.01)
# leg2knee.enable()
# time.sleep(0.01)
# subprocess.run(["clear"])

# print("Deploying Leg!")


# leg2hipPitch.control(position=0, velocity=0, torque=0, kp=9, kd=0.8)
# leg2knee.control(position=-0.859, velocity=0, torque=0, kp=6, kd=0.8)
# time.sleep(2)
# leg2hipPitch.control(position=1.366, velocity=0, torque=0, kp=3, kd=1)

# time.sleep(10)
# subprocess.run(["clear"])



# -------------------------------------------------------
# Shutdown
# -------------------------------------------------------
# print("Damping...")
# for motor in allMotors:
#     motor.control(position=0, velocity=0, torque=0, kp=0, kd=1)
# time.sleep(2)

print("Shutting Down")
for motor in allMotors:
    motor.stop()
    time.sleep(0.01)
bus0.shutdown()
bus1.shutdown()