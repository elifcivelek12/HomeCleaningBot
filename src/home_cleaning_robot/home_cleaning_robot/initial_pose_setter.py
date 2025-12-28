#!/usr/bin/env python3
"""
Simple script to set initial pose for AMCL localization
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import time
import math

class InitialPoseSetter(Node):
    def __init__(self):
        super().__init__('initial_pose_setter')
        
        # Publisher for initial pose
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
        )
        
        # Wait for subscribers
        time.sleep(1.0)
        
        self.get_logger().info('Setting initial pose...')
        # Eski: self.set_initial_pose(x=0.0, y=-2.0, yaw=0.0)
        self.set_initial_pose(x=1.0, y=-2.0, yaw=0.0)
        
    def set_initial_pose(self, x, y, yaw):
        """
        Set the initial pose of the robot for AMCL
        
        Args:
            x: x position in map frame (meters)
            y: y position in map frame (meters)
            yaw: orientation in map frame (radians)
        """
        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.stamp = self.get_clock().now().to_msg()
        initial_pose.header.frame_id = 'map'
        
        # Set position
        initial_pose.pose.pose.position.x = x
        initial_pose.pose.pose.position.y = y
        initial_pose.pose.pose.position.z = 0.0
        
        # Set orientation (quaternion from yaw)
        initial_pose.pose.pose.orientation.x = 0.0
        initial_pose.pose.pose.orientation.y = 0.0
        initial_pose.pose.pose.orientation.z = math.sin(yaw / 2.0)
        initial_pose.pose.pose.orientation.w = math.cos(yaw / 2.0)
        
        # Set covariance (diagonal matrix)
        initial_pose.pose.covariance[0] = 0.25  # x variance
        initial_pose.pose.covariance[7] = 0.25  # y variance
        initial_pose.pose.covariance[35] = 0.06853  # yaw variance
        
        # Publish the initial pose
        self.pose_pub.publish(initial_pose)
        self.get_logger().info(f'Initial pose set to: x={x}, y={y}, yaw={yaw}')


def main(args=None):
    rclpy.init(args=args)
    node = InitialPoseSetter()
    
    # Keep node alive briefly to ensure message is sent
    rclpy.spin_once(node, timeout_sec=2.0)
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
