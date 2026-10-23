"""ROS 2 MuJoCo physics/rendering server for the TOCATL hexapod.

Simulation only -- no CAN, no motor drivers. Subscribes to the MIT-mode
command stream and steps physics at 500 Hz in (paced) real time:

  /joint_commands  sensor_msgs/JointState   name, position, velocity,
                                            effort (= feedforward torque)
  /joint_gains     std_msgs/Float64MultiArray  [kp x18, kd x18], Section 2.5 order

Publishes /joint_states, /odom and the world->base_link TF. A passive MuJoCo
viewer window opens unless the ``headless`` parameter is true.
"""

from __future__ import annotations

import threading
import time

import numpy as np

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from tf2_ros import TransformBroadcaster

from .hexapod_sim import HexapodSim
from .model_builder import JOINT_NAMES, ModelParams, default_mesh_dir


class MujocoSimNode(Node):
    def __init__(self):
        super().__init__("mujoco_sim_node")
        self.declare_parameter("headless", False)
        self.declare_parameter("realtime_factor", 1.0)
        self.declare_parameter("joint_state_rate", 100.0)
        self.declare_parameter("timestep", 0.002)
        self.declare_parameter("foot_friction", "1.2 0.02 0.001")
        self.declare_parameter("base_mass", 6.0)
        self.declare_parameter("use_meshes", True)

        params = ModelParams(
            timestep=self.get_parameter("timestep").value,
            foot_friction=self.get_parameter("foot_friction").value,
            base_mass=self.get_parameter("base_mass").value,
            mesh_dir=(default_mesh_dir()
                      if self.get_parameter("use_meshes").value else None),
        )
        self.sim = HexapodSim(params)
        self.lock = threading.Lock()
        self.name_to_index = {n: i for i, n in enumerate(JOINT_NAMES)}

        self.create_subscription(JointState, "joint_commands", self._on_command, 10)
        self.create_subscription(Float64MultiArray, "joint_gains", self._on_gains, 10)
        self.pub_js = self.create_publisher(JointState, "joint_states", 10)
        self.pub_odom = self.create_publisher(Odometry, "odom", 10)
        self.tf = TransformBroadcaster(self)

        rate = self.get_parameter("joint_state_rate").value
        self.create_timer(1.0 / rate, self._publish_state)
        self.get_logger().info(
            f"TOCATL MuJoCo sim up: {len(JOINT_NAMES)} joints, "
            f"dt={self.sim.timestep*1000:.1f} ms, headless="
            f"{self.get_parameter('headless').value}")

    # ---------------- command intake ----------------

    def _on_command(self, msg: JointState) -> None:
        pos = np.array(self.sim.cmd_pos)
        vel = np.array(self.sim.cmd_vel)
        tau = np.array(self.sim.cmd_tau)
        if msg.name:
            for j, n in enumerate(msg.name):
                i = self.name_to_index.get(n)
                if i is None:
                    continue
                if j < len(msg.position):
                    pos[i] = msg.position[j]
                if j < len(msg.velocity):
                    vel[i] = msg.velocity[j]
                if j < len(msg.effort):
                    tau[i] = msg.effort[j]
        else:  # no names: assume canonical Section 2.5 order
            pos[:len(msg.position)] = msg.position
            if msg.velocity:
                vel[:len(msg.velocity)] = msg.velocity
            if msg.effort:
                tau[:len(msg.effort)] = msg.effort
        with self.lock:
            self.sim.set_command(pos, vel, tau_ff=tau)

    def _on_gains(self, msg: Float64MultiArray) -> None:
        d = np.asarray(msg.data, dtype=float)
        if d.size != 36:
            self.get_logger().warn(f"joint_gains needs 36 values, got {d.size}")
            return
        with self.lock:
            self.sim.set_command(self.sim.cmd_pos, kp=d[:18], kd=d[18:])

    # ---------------- state output ----------------

    def _publish_state(self) -> None:
        now = self.get_clock().now().to_msg()
        with self.lock:
            q = self.sim.joint_positions()
            qd = self.sim.joint_velocities()
            tau_applied = self.sim.data.ctrl[self.sim.act_ids].copy()
            pos, quat = self.sim.base_pose()
            linvel, angvel = self.sim.base_velocity()

        js = JointState()
        js.header.stamp = now
        js.name = list(JOINT_NAMES)
        js.position = q.tolist()
        js.velocity = qd.tolist()
        js.effort = tau_applied.tolist()
        self.pub_js.publish(js)

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = "world"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z = pos
        (odom.pose.pose.orientation.w, odom.pose.pose.orientation.x,
         odom.pose.pose.orientation.y, odom.pose.pose.orientation.z) = quat
        (odom.twist.twist.linear.x, odom.twist.twist.linear.y,
         odom.twist.twist.linear.z) = linvel
        (odom.twist.twist.angular.x, odom.twist.twist.angular.y,
         odom.twist.twist.angular.z) = angvel
        self.pub_odom.publish(odom)

        tfm = TransformStamped()
        tfm.header.stamp = now
        tfm.header.frame_id = "world"
        tfm.child_frame_id = "base_link"
        tfm.transform.translation.x, tfm.transform.translation.y, tfm.transform.translation.z = pos
        (tfm.transform.rotation.w, tfm.transform.rotation.x,
         tfm.transform.rotation.y, tfm.transform.rotation.z) = quat
        self.tf.sendTransform(tfm)

    # ---------------- physics loop (main thread) ----------------

    def run_physics(self) -> None:
        headless = self.get_parameter("headless").value
        rtf = self.get_parameter("realtime_factor").value
        viewer_ctx = None
        if not headless:
            import mujoco.viewer
            viewer_ctx = mujoco.viewer.launch_passive(self.sim.model, self.sim.data)

        dt = self.sim.timestep
        next_t = time.perf_counter()
        last_sync = 0.0
        try:
            while rclpy.ok():
                with self.lock:
                    self.sim.step(1)
                if viewer_ctx is not None:
                    if not viewer_ctx.is_running():
                        break
                    now = time.perf_counter()
                    if now - last_sync > 1.0 / 60.0:
                        viewer_ctx.sync()
                        last_sync = now
                next_t += dt / max(1e-6, rtf)
                sleep = next_t - time.perf_counter()
                if sleep > 0:
                    time.sleep(sleep)
                else:
                    next_t = time.perf_counter()
        finally:
            if viewer_ctx is not None:
                viewer_ctx.close()


def main(args=None):
    rclpy.init(args=args)
    node = MujocoSimNode()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()
    try:
        node.run_physics()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
