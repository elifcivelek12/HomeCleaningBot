#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
import math

class StatePublisher(Node):
    def __init__(self):
        super().__init__('state_publisher')

        # Publisher for joint states
        self.joint_state_publisher = self.create_publisher(JointState, 'joint_states', 10)

        # Subscriber to cmd_vel for simulating wheel movement
        self.cmd_vel_subscriber = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Timer to publish joint states at regular intervals
        self.timer = self.create_timer(0.1, self.publish_joint_states)  # 10 Hz

        # Joint state variables
        self.left_wheel_position = 0.0
        self.right_wheel_position = 0.0
        self.left_wheel_velocity = 0.0
        self.right_wheel_velocity = 0.0

        # Robot parameters (adjust based on your URDF)
        self.wheel_radius = 1.0  # meters
        self.wheel_separation = 4.5  # meters (distance between wheels)

        self.get_logger().info('State Publisher Node has been started')

    def cmd_vel_callback(self, msg):
        # Convert linear and angular velocity to wheel velocities
        linear_vel = msg.linear.x
        angular_vel = msg.angular.z

        # Differential drive kinematics
        left_wheel_vel = (linear_vel - angular_vel * self.wheel_separation / 2.0) / self.wheel_radius
        right_wheel_vel = (linear_vel + angular_vel * self.wheel_separation / 2.0) / self.wheel_radius

        self.left_wheel_velocity = left_wheel_vel
        self.right_wheel_velocity = right_wheel_vel

    def publish_joint_states(self):
        # Update positions based on velocities (simple integration)
        dt = 0.1  # time step
        self.left_wheel_position += self.left_wheel_velocity * dt
        self.right_wheel_position += self.right_wheel_velocity * dt

        # Create joint state message
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.header.frame_id = 'base_link'

        # Joint names (must match URDF)
        joint_state.name = ['base_left_wheel_joint', 'base_right_wheel_joint']

        # Joint positions
        joint_state.position = [self.left_wheel_position, self.right_wheel_position]

        # Joint velocities
        joint_state.velocity = [self.left_wheel_velocity, self.right_wheel_velocity]

        # Joint efforts (not used for now)
        joint_state.effort = [0.0, 0.0]

        # Publish the message
        self.joint_state_publisher.publish(joint_state)

def main(args=None):
    rclpy.init(args=args)
    node = StatePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()