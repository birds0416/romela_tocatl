"""ROS 2 locomotion brain for the TOCATL hexapod sim.

Runs the ported legacy gait (``gait_engine.GaitEngine``) at 200 Hz -- the real
robot's command-stream rate -- and publishes the MIT-mode command frames the
physics node consumes. Mirrors the real architecture (gait generator ->
command sender -> motors) so this node could later target real CAN via a
different backend without touching locomotion logic.

  in:  /cmd_vel    geometry_msgs/Twist  linear.x/.y [m/s], angular.z [rad/s]
       /gait_cmd   std_msgs/String      JSON events (set_gait / adjust / stop
                                        / park / unpark), see teleop_keyboard
  out: /joint_commands  sensor_msgs/JointState (position, velocity, effort=tau_ff)
       /joint_gains     std_msgs/Float64MultiArray [kp x18, kd x18]
"""

from __future__ import annotations

import json

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String

from tocatl_gait.legacy_reference import LEGACY_WALKING_ENVELOPE as ENV

from .gait_engine import GainSchedule, GaitConfig, GaitEngine
from .model_builder import JOINT_NAMES

CONTROL_RATE_HZ = ENV.command_stream_hz  # 200 Hz


class GaitNode(Node):
    def __init__(self):
        super().__init__("gait_node")
        self.declare_parameter("gait", "tripod")
        self.declare_parameter("stride_mm", ENV.default_stride_length_mm)
        self.declare_parameter("step_height_mm", ENV.default_step_height_mm)
        self.declare_parameter("body_height_mm", 300.0)
        self.declare_parameter("stance_radius_mm", 200.0)
        self.declare_parameter("kp_stance", 100.0)
        self.declare_parameter("kp_swing", 10.0)
        self.declare_parameter("kd", 3.0)
        self.declare_parameter("kp_rate_up", 25.0)
        self.declare_parameter("kp_rate_down", 6.0)
        self.declare_parameter("hip_pitch_support_tau", 0.0)

        gp = self.get_parameter
        gains = GainSchedule(
            kp_hip_yaw_stance=gp("kp_stance").value,
            kp_hip_pitch_stance=gp("kp_stance").value,
            kp_knee_stance=gp("kp_stance").value,
            kp_hip_yaw_swing=gp("kp_swing").value,
            kp_hip_pitch_swing=gp("kp_swing").value,
            kp_knee_swing=gp("kp_swing").value,
            kd_hip_yaw=gp("kd").value,
            kd_hip_pitch=gp("kd").value,
            kd_knee=gp("kd").value,
            kp_rate_up=gp("kp_rate_up").value,
            kp_rate_down=gp("kp_rate_down").value,
            hip_pitch_support_tau=gp("hip_pitch_support_tau").value,
        )
        config = GaitConfig(
            stride_mm=gp("stride_mm").value,
            step_height_mm=gp("step_height_mm").value,
            body_height_mm=gp("body_height_mm").value,
            stance_radius_mm=gp("stance_radius_mm").value,
            gains=gains,
        )
        self.engine = GaitEngine(config, gait_name=gp("gait").value)

        self.create_subscription(Twist, "cmd_vel", self._on_cmd_vel, 10)
        self.create_subscription(String, "gait_cmd", self._on_gait_cmd, 10)
        self.pub_cmd = self.create_publisher(JointState, "joint_commands", 10)
        self.pub_gains = self.create_publisher(Float64MultiArray, "joint_gains", 10)

        self.dt = 1.0 / CONTROL_RATE_HZ
        self.create_timer(self.dt, self._tick)
        self.get_logger().info(
            f"gait_node up: {self.engine.gait_name}, {CONTROL_RATE_HZ:.0f} Hz command stream")

    def _on_cmd_vel(self, msg: Twist) -> None:
        # physical units in the Twist; normalize by the legacy envelope maxima
        self.engine.set_velocity(
            msg.linear.x / (ENV.body_speed_max_mm_s * 1e-3),
            msg.linear.y / (ENV.body_speed_max_mm_s * 1e-3),
            msg.angular.z / ENV.body_yaw_rate_max_rad_s,
        )

    def _on_gait_cmd(self, msg: String) -> None:
        try:
            cmd = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(f"bad gait_cmd: {msg.data!r}")
            return
        action = cmd.get("action")
        if action == "set_gait":
            if self.engine.set_gait(cmd.get("name", "")):
                self.get_logger().info(f"gait -> {self.engine.gait_name}")
        elif action == "adjust":
            self.engine.adjust(
                stride=cmd.get("stride_mm", 0.0),
                step_h=cmd.get("step_height_mm", 0.0),
                body_h=cmd.get("body_height_mm", 0.0),
                radius=cmd.get("radius_mm", 0.0),
            )
        elif action == "stop":
            self.engine.stop()
        elif action == "park":
            self.engine.park()
        elif action == "unpark":
            self.engine.unpark()
        else:
            self.get_logger().warn(f"unknown gait_cmd action: {action!r}")

    def _tick(self) -> None:
        c = self.engine.update(self.dt)
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = list(JOINT_NAMES)
        js.position = c.position.tolist()
        js.velocity = c.velocity.tolist()
        js.effort = c.tau_ff.tolist()
        self.pub_cmd.publish(js)

        gains = Float64MultiArray()
        gains.data = c.kp.tolist() + c.kd.tolist()
        self.pub_gains.publish(gains)


def main(args=None):
    rclpy.init(args=args)
    node = GaitNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
