"""Keyboard teleop for the TOCATL MuJoCo sim (replaces the Xbox controller).

Key map (Section 4.1 of the sim brief):

  W / S       walk forward / backward (vx)        Left stick Y
  A / D       strafe left / right (vy)            Left stick X
  Q / E       turn (yaw) left / right             Right stick X
  1 / 2 / 3   gait: tripod / ripple / wave
  [ / ]       stride length  - / +                DPAD X
  - / =       step height    - / +                DPAD Y
  R / F       body height up / down               A / B
  T / G       leg radius out / in                 Y / X
  Space       stop -> graceful stand              LB
  Z           park / unpark toggle                RB
  X or Esc    quit teleop cleanly                 Back

With ``pynput`` installed (preferred) the movement keys are true
hold-to-move: press and hold W and the robot walks until you release --
this reads best on camera. Keys are captured globally on X11, so they work
while the MuJoCo window has focus. Without pynput a raw-terminal fallback is
used where movement keys step the commanded velocity (press-to-set) and
Space stops.

Publishes /cmd_vel (geometry_msgs/Twist, physical m/s and rad/s, slewed by the
gait node) and /gait_cmd (std_msgs/String, JSON events).
"""

from __future__ import annotations

import json
import sys
import threading

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import String

from tocatl_gait.legacy_reference import LEGACY_WALKING_ENVELOPE as ENV

V_MAX = ENV.body_speed_max_mm_s * 1e-3       # 0.140 m/s
W_MAX = ENV.body_yaw_rate_max_rad_s          # ~0.611 rad/s
ADJUST_STEP_MM = 25.0
HEIGHT_STEP_MM = 20.0

HELP = (
    "W/S fwd/back  A/D strafe  Q/E turn  |  1/2/3 gait  [/] stride  -/= step h\n"
    "R/F body up/down  T/G radius out/in  |  Space stop  Z park  X/Esc quit"
)


class TeleopKeyboard(Node):
    def __init__(self):
        super().__init__("teleop_keyboard")
        self.pub_vel = self.create_publisher(Twist, "cmd_vel", 10)
        self.pub_cmd = self.create_publisher(String, "gait_cmd", 10)
        self.lock = threading.Lock()
        self.held: set[str] = set()          # pynput hold-to-move state
        self.vx = self.vy = self.wz = 0.0    # normalized targets [-1, 1]
        self.parked = False
        self.quit = False
        self.mode = "?"
        self.create_timer(1.0 / 30.0, self._publish)

    # ---------------- publishing ----------------

    def _publish(self) -> None:
        with self.lock:
            vx, vy, wz = self.vx, self.vy, self.wz
        msg = Twist()
        msg.linear.x = vx * V_MAX
        msg.linear.y = vy * V_MAX
        msg.angular.z = wz * W_MAX
        self.pub_vel.publish(msg)
        status = (f"\r[{self.mode}] vx {vx:+.2f}  vy {vy:+.2f}  yaw {wz:+.2f}"
                  f"  {'PARKED' if self.parked else 'active'}   ")
        sys.stdout.write(status)
        sys.stdout.flush()

    def _event(self, **payload) -> None:
        msg = String()
        msg.data = json.dumps(payload)
        self.pub_cmd.publish(msg)

    # ---------------- shared key handling ----------------

    def handle_action_key(self, ch: str) -> None:
        """Non-movement keys: single-shot actions."""
        if ch == "1":
            self._event(action="set_gait", name="tripod")
        elif ch == "2":
            self._event(action="set_gait", name="ripple")
        elif ch == "3":
            self._event(action="set_gait", name="wave")
        elif ch == "[":
            self._event(action="adjust", stride_mm=-ADJUST_STEP_MM)
        elif ch == "]":
            self._event(action="adjust", stride_mm=+ADJUST_STEP_MM)
        elif ch == "-":
            self._event(action="adjust", step_height_mm=-ADJUST_STEP_MM)
        elif ch == "=":
            self._event(action="adjust", step_height_mm=+ADJUST_STEP_MM)
        elif ch == "r":
            self._event(action="adjust", body_height_mm=+HEIGHT_STEP_MM)
        elif ch == "f":
            self._event(action="adjust", body_height_mm=-HEIGHT_STEP_MM)
        elif ch == "t":
            self._event(action="adjust", radius_mm=+HEIGHT_STEP_MM)
        elif ch == "g":
            self._event(action="adjust", radius_mm=-HEIGHT_STEP_MM)
        elif ch == " ":
            with self.lock:
                self.vx = self.vy = self.wz = 0.0
                self.held.clear()
            self._event(action="stop")
        elif ch == "z":
            self.parked = not self.parked
            with self.lock:
                self.vx = self.vy = self.wz = 0.0
                self.held.clear()
            self._event(action="park" if self.parked else "unpark")
        elif ch in ("x", "\x1b"):
            with self.lock:
                self.vx = self.vy = self.wz = 0.0
            self.quit = True

    # ---------------- pynput backend (hold-to-move) ----------------

    def _held_to_velocity(self) -> None:
        h = self.held
        self.vx = (1.0 if "w" in h else 0.0) - (1.0 if "s" in h else 0.0)
        self.vy = (1.0 if "a" in h else 0.0) - (1.0 if "d" in h else 0.0)
        self.wz = (1.0 if "q" in h else 0.0) - (1.0 if "e" in h else 0.0)

    def run_pynput(self) -> None:
        from pynput import keyboard

        self.mode = "hold-to-move (pynput)"
        move_keys = set("wsadqe")

        def to_char(key):
            if isinstance(key, keyboard.KeyCode) and key.char:
                return key.char.lower()
            if key == keyboard.Key.space:
                return " "
            if key == keyboard.Key.esc:
                return "\x1b"
            return None

        def on_press(key):
            ch = to_char(key)
            if ch is None:
                return
            if ch in move_keys:
                with self.lock:
                    self.held.add(ch)
                    self._held_to_velocity()
            else:
                self.handle_action_key(ch)
            return not self.quit

        def on_release(key):
            ch = to_char(key)
            if ch in move_keys:
                with self.lock:
                    self.held.discard(ch)
                    self._held_to_velocity()
            return not self.quit

        print(HELP)
        print("(keys are captured globally -- drive with the MuJoCo window focused)")
        listener = keyboard.Listener(on_press=on_press, on_release=on_release)
        listener.start()
        try:
            while rclpy.ok() and not self.quit:
                rclpy.spin_once(self, timeout_sec=0.05)
        finally:
            listener.stop()

    # ---------------- termios fallback (press-to-set) ----------------

    def run_termios(self) -> None:
        import select
        import termios
        import tty

        self.mode = "press-to-set (terminal)"
        move_step = {"w": ("vx", 0.25), "s": ("vx", -0.25),
                     "a": ("vy", 0.25), "d": ("vy", -0.25),
                     "q": ("wz", 0.25), "e": ("wz", -0.25)}
        print(HELP)
        print("(terminal mode: movement keys step velocity by 0.25, Space stops)")
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)
            while rclpy.ok() and not self.quit:
                rclpy.spin_once(self, timeout_sec=0.0)
                r, _, _ = select.select([sys.stdin], [], [], 0.05)
                if not r:
                    continue
                ch = sys.stdin.read(1).lower()
                if ch in move_step:
                    attr, step = move_step[ch]
                    with self.lock:
                        setattr(self, attr, max(-1.0, min(1.0, getattr(self, attr) + step)))
                else:
                    self.handle_action_key(ch)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)


def main(args=None):
    rclpy.init(args=args)
    node = TeleopKeyboard()
    try:
        try:
            import pynput  # noqa: F401
            node.run_pynput()
        except ImportError:
            node.run_termios()
    except KeyboardInterrupt:
        pass
    finally:
        # leave the robot standing, not walking off
        stop = Twist()
        node.pub_vel.publish(stop)
        print("\nteleop exiting; robot commanded to stop.")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
