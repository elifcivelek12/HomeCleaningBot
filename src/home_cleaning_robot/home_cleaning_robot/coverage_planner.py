#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
import numpy as np
import math

class CoveragePlanner:
    """
    Coverage planner for cleaning robot using Boustrophedon (back-and-forth) pattern.
    This planner generates waypoints to cover the entire free space in the map.
    """
    
    def __init__(self, logger=None):
        # Parameters
        self.coverage_distance = 0.4  # Distance between parallel lines (meters)
        self.map_data = None
        self.map_resolution = None
        self.map_origin = None
        self.map_width = None
        self.map_height = None
        self.logger = logger
    
    def set_map(self, map_msg):
        """Set map data from OccupancyGrid message"""
        self.map_data = np.array(map_msg.data).reshape((map_msg.info.height, map_msg.info.width))
        self.map_resolution = map_msg.info.resolution
        self.map_origin = map_msg.info.origin
        self.map_width = map_msg.info.width
        self.map_height = map_msg.info.height
        if self.logger:
            self.logger.info(f'Map set: {self.map_width}x{self.map_height}, resolution: {self.map_resolution}m')
    
    def is_map_ready(self):
        """Check if map data is available"""
        return self.map_data is not None
    
    def world_to_map(self, x, y):
        """Convert world coordinates to map grid coordinates"""
        mx = int((x - self.map_origin.position.x) / self.map_resolution)
        my = int((y - self.map_origin.position.y) / self.map_resolution)
        return mx, my
    
    def map_to_world(self, mx, my):
        """Convert map grid coordinates to world coordinates"""
        x = mx * self.map_resolution + self.map_origin.position.x
        y = my * self.map_resolution + self.map_origin.position.y
        return x, y
    
    def is_free(self, mx, my):
        """Check if a map cell is free (not occupied or unknown)"""
        if mx < 0 or mx >= self.map_width or my < 0 or my >= self.map_height:
            return False
        
        # OccupancyGrid: 0 = free, 100 = occupied, -1 = unknown
        cell_value = self.map_data[my, mx]
        return cell_value >= 0 and cell_value < 50  # Consider cells < 50 as free
    
    def get_map_bounds(self):
        """Find the actual bounds of the free space in the map"""
        free_cells = np.argwhere((self.map_data >= 0) & (self.map_data < 50))
        
        if len(free_cells) == 0:
            if self.logger:
                self.logger.error('No free space found in the map!')
            return None
        
        min_y, min_x = free_cells.min(axis=0)
        max_y, max_x = free_cells.max(axis=0)
        
        # Add small margin
        margin = int(0.5 / self.map_resolution)  # 0.5 meter margin
        min_x = max(0, min_x - margin)
        max_x = min(self.map_width - 1, max_x + margin)
        min_y = max(0, min_y - margin)
        max_y = min(self.map_height - 1, max_y + margin)
        
        return min_x, max_x, min_y, max_y
    
    def generate_coverage_path(self, start_pose=None, clock=None):
        """
        Generate a boustrophedon (back-and-forth) coverage path
        
        Args:
            start_pose: Optional starting position (PoseStamped)
            clock: ROS clock for timestamps
        
        Returns:
            List of PoseStamped waypoints covering the area
        """
        if not self.is_map_ready():
            if self.logger:
                self.logger.error('Map not ready yet!')
            return []
        
        if self.logger:
            self.logger.info('Generating coverage path...')
        
        # Get map bounds
        bounds = self.get_map_bounds()
        if bounds is None:
            return []
        
        min_x, max_x, min_y, max_y = bounds
        
        # Convert to world coordinates
        x_start, y_start = self.map_to_world(min_x, min_y)
        x_end, y_end = self.map_to_world(max_x, max_y)
        
        if self.logger:
            self.logger.info(f'Coverage area: X[{x_start:.2f}, {x_end:.2f}], Y[{y_start:.2f}, {y_end:.2f}]')
        
        waypoints = []
        
        # Calculate number of rows based on coverage distance
        num_rows = int((y_end - y_start) / self.coverage_distance) + 1
        
        # Generate boustrophedon pattern
        for i in range(num_rows):
            y = y_start + i * self.coverage_distance
            
            # Alternate direction for each row
            if i % 2 == 0:
                # Left to right
                x_points = [x_start, x_end]
            else:
                # Right to left
                x_points = [x_end, x_start]
            
            for x in x_points:
                # Check if this point is in free space
                mx, my = self.world_to_map(x, y)
                
                if self.is_free(mx, my):
                    pose = PoseStamped()
                    pose.header.frame_id = 'map'
                    if clock:
                        pose.header.stamp = clock.now().to_msg()
                    pose.pose.position.x = x
                    pose.pose.position.y = y
                    pose.pose.position.z = 0.0
                    
                    # Calculate orientation based on direction
                    if i % 2 == 0:
                        # Moving right: 0 degrees
                        pose.pose.orientation.w = 1.0
                        pose.pose.orientation.z = 0.0
                    else:
                        # Moving left: 180 degrees
                        pose.pose.orientation.w = 0.0
                        pose.pose.orientation.z = 1.0
                    
                    waypoints.append(pose)
        
        if self.logger:
            self.logger.info(f'Generated {len(waypoints)} waypoints for coverage')
        return waypoints
    
    def optimize_path(self, waypoints, start_pose=None):
        """
        Optimize the path by removing waypoints that are too close to obstacles
        
        Args:
            waypoints: List of PoseStamped waypoints
            start_pose: Optional starting position
        
        Returns:
            Optimized list of waypoints
        """
        if not waypoints:
            return []
        
        optimized = []
        # Reduce obstacle margin to 0.15m (robot radius consideration)
        obstacle_margin = int(0.15 / self.map_resolution)
        
        if self.logger:
            self.logger.info(f'Optimizing with obstacle margin: {obstacle_margin} cells ({0.15}m)')
        
        for wp in waypoints:
            mx, my = self.world_to_map(wp.pose.position.x, wp.pose.position.y)
            
            # Check if the center point is free
            if not self.is_free(mx, my):
                continue
            
            # Check if there's enough clearance around this waypoint
            # Use a circular check instead of square for better results
            is_safe = True
            for dx in range(-obstacle_margin, obstacle_margin + 1):
                for dy in range(-obstacle_margin, obstacle_margin + 1):
                    # Only check within circular radius
                    distance = (dx**2 + dy**2) ** 0.5
                    if distance <= obstacle_margin:
                        if not self.is_free(mx + dx, my + dy):
                            is_safe = False
                            break
                if not is_safe:
                    break
            
            if is_safe:
                optimized.append(wp)
        
        if self.logger:
            self.logger.info(f'Path optimized: {len(waypoints)} -> {len(optimized)} waypoints')
        
        # If optimization removed too many waypoints, try with just center point check
        if len(optimized) < len(waypoints) * 0.3:  # If less than 30% survived
            if self.logger:
                self.logger.warn(f'Too many waypoints removed. Using relaxed optimization...')
            optimized = []
            for wp in waypoints:
                mx, my = self.world_to_map(wp.pose.position.x, wp.pose.position.y)
                # Just check if the waypoint itself is free
                if self.is_free(mx, my):
                    optimized.append(wp)
            if self.logger:
                self.logger.info(f'Relaxed optimization: {len(waypoints)} -> {len(optimized)} waypoints')
        
        return optimized
