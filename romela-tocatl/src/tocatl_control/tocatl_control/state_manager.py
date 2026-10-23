#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String, Float64MultiArray
from enum import Enum

# Import IK from kinematics package (will add later)
# from tocatl_kinematics.ik_solver import ik_solver

def ik_solver(leg_positions):
    """Placeholder - replace with actual IK"""
    angles = []
    for pos in leg_positions:
        angles.extend([0.0, 0.0, 0.0])
    return angles

class RobotState(Enum):
    INIT = 0
    INTERPOLATE = 1
    STAND = 2
    READY = 3

class StateManager(Node):
    def __init__(self):
        super().__init__('state_manager')
        
        # Declare parameters
        self.declare_parameter('body_height', 0.20)
        self.declare_parameter('interpolation_duration', 2.0)
        
        # Get parameters
        self.body_height = self.get_parameter('body_height').value
        self.interpolation_duration = self.get_parameter('interpolation_duration').value
        
        # Publishers & Subscribers
        self.motor_ready_sub = self.create_subscription(
            Bool, '/motors_ready', self.motor_ready_cb, 10)
        self.joint_pub = self.create_publisher(
            Float64MultiArray, '/joint_commands', 10)
        self.state_pub = self.create_publisher(
            String, '/robot_state', 10)
        self.gait_enable_pub = self.create_publisher(
            Bool, '/gait_enable', 10)
        
        self.timer = self.create_timer(0.02, self.update)
        
        # State
        self.state = RobotState.INIT
        self.state_start_time = self.get_clock().now()
        
        # Robot configuration
        self.init_pose = [0.0] * 18
        self.leg_base_positions = [
            (0.25, -0.15), (0.25, 0.15), (0.0, 0.20),
            (-0.25, 0.15), (-0.25, -0.15), (0.0, -0.20)
        ]
        
        self.get_logger().info('State Manager initialized - waiting for motors...')
    
    def motor_ready_cb(self, msg):
        if msg.data and self.state == RobotState.INIT:
            self.state = RobotState.INTERPOLATE
            self.state_start_time = self.get_clock().now()
            self.get_logger().info('Motors ready! Starting interpolation...')
    
    def get_state_time(self):
        return (self.get_clock().now() - self.state_start_time).nanoseconds / 1e9
    
    def update(self):
        # Publish current state
        state_msg = String()
        state_msg.data = self.state.name
        self.state_pub.publish(state_msg)
        
        if self.state == RobotState.INIT:
            msg = Float64MultiArray()
            msg.data = self.init_pose
            self.joint_pub.publish(msg)
            self.gait_enable_pub.publish(Bool(data=False))
        
        elif self.state == RobotState.INTERPOLATE:
            t = self.get_state_time()
            alpha = min(t / self.interpolation_duration, 1.0)
            
            # Standing pose
            standing_pos = [(x, y, -self.body_height) for x, y in self.leg_base_positions]
            target = ik_solver(standing_pos)
            
            # Smoothstep interpolation
            smooth = 3*alpha**2 - 2*alpha**3
            angles = [self.init_pose[i] + smooth*(target[i] - self.init_pose[i]) 
                     for i in range(18)]
            
            msg = Float64MultiArray()
            msg.data = angles
            self.joint_pub.publish(msg)
            self.gait_enable_pub.publish(Bool(data=False))
            
            if alpha >= 1.0:
                self.state = RobotState.STAND
                self.state_start_time = self.get_clock().now()
                self.get_logger().info('Standing pose reached!')
        
        elif self.state == RobotState.STAND:
            if self.get_state_time() > 1.0:
                self.state = RobotState.READY
                self.get_logger().info('READY - Gait enabled!')
            self.gait_enable_pub.publish(Bool(data=False))
        
        elif self.state == RobotState.READY:
            self.gait_enable_pub.publish(Bool(data=True))

def main(args=None):
    rclpy.init(args=args)
    node = StateManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()