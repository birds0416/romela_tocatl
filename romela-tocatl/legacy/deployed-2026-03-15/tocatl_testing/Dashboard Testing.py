import can

bus = can.interface.Bus(channel='can0', interface='socketcan')

print("Waiting for CAN frame...")
msg = bus.recv(timeout=2)

print("Received:", msg)
    