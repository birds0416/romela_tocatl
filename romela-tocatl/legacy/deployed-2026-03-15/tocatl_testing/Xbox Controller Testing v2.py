#!/usr/bin/env python3

import os
import sys
import shlex
import signal
import subprocess
import threading
import time

# -------------------------------------------------------
# ROS auto-bootstrap (fixes rclpy missing in .venv)
# -------------------------------------------------------
try:
    import rclpy
except ModuleNotFoundError:
    script = shlex.quote(os.path.abspath(__file__))
    py = shlex.quote(sys.executable)
    cmd = f"source /opt/ros/jazzy/setup.bash && exec {py} {script}"
    os.execvp("bash", ["bash", "-lc", cmd])

from rclpy.node import Node
from sensor_msgs.msg import Joy


# -------------------------------------------------------
# joy_linux_node bring-up
# -------------------------------------------------------
def start_joy_node():
    print("[JOYDRV] Starting joy_linux_node...")
    return subprocess.Popen(
        ["bash", "-lc", "source /opt/ros/jazzy/setup.bash && exec ros2 run joy_linux joy_linux_node"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        preexec_fn=os.setsid
    )


def stop_joy_node(proc):
    print("[JOYDRV] Stopping joy_linux_node...")
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
    except Exception:
        pass


# -------------------------------------------------------
# ROS Node
# -------------------------------------------------------
class JoystickPrinter(Node):

    def __init__(self):
        super().__init__("joystick_printer")

        self.sub = self.create_subscription(
            Joy,
            "/joy",
            self.joy_callback,
            10
        )

        self.stop_flag = False

        # AXIS names (exactly as you specified)
        self.axes_names = [
            "Left Joy X",     # axis 0
            "Left Joy Y",     # axis 1
            "Left Trigger",   # axis 2
            "Right Joy X",    # axis 3
            "Right Joy Y",    # axis 4
            "Right Trigger",  # axis 5
            "DPAD_X",         # axis 6
            "DPAD_Y"          # axis 7
        ]

        # BUTTON mapping
        self.button_names = {
            0: "A",
            1: "B",
            2: "X",
            3: "Y",
            4: "LB",
            5: "RB",
            6: "Back",
            7: "Start",
            8: "Xbox",
            9: "Left Stick Click (L3)",
            10: "Right Stick Click (R3)"
        }

    def joy_callback(self, msg: Joy):

        if self.stop_flag:
            return

        print("\n=== AXES ===")

        for i, v in enumerate(msg.axes):

            if i < len(self.axes_names):
                name = self.axes_names[i]
            else:
                continue  # prevents printing axis 8 or anything beyond

            print(f"{name:24s}: {v:+.3f}")

        print("\n=== BUTTONS ===")

        pressed = []

        for i, b in enumerate(msg.buttons):

            name = self.button_names.get(i, f"Button {i}")

            print(f"{name:24s}: {b}")

            if b == 1:
                pressed.append((i, name))

        if pressed:
            print("\nPressed:")
            for i, name in pressed:
                print(f"  {i} -> {name}")


# -------------------------------------------------------
# Stop helper
# -------------------------------------------------------
def wait_for_enter(node: JoystickPrinter):
    input("\nPress ENTER to stop\n")
    node.stop_flag = True


# -------------------------------------------------------
# Main
# -------------------------------------------------------
def main():

    rclpy.init()

    joy_process = start_joy_node()
    time.sleep(1.0)

    node = JoystickPrinter()

    print("\nJoystick test running\n")

    threading.Thread(
        target=wait_for_enter,
        args=(node,),
        daemon=True
    ).start()

    try:

        while rclpy.ok() and not node.stop_flag:

            if joy_process.poll() is not None:
                print("\njoy_linux_node exited unexpectedly")
                break

            rclpy.spin_once(node, timeout_sec=0.1)

    except KeyboardInterrupt:
        pass

    print("\nStopping joystick test...")

    try:
        node.destroy_node()
    except Exception:
        pass

    try:
        rclpy.shutdown()
    except Exception:
        pass

    stop_joy_node(joy_process)

    print("Done")


if __name__ == "__main__":
    main()