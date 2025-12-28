#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from home_cleaning_robot.coverage_planner import CoveragePlanner
import time

class BehaviorManager(Node):
    def __init__(self):
        super().__init__('behavior_manager')
        
        # Navigasyon API'sini başlat
        self.navigator = BasicNavigator()
        
        # Coverage planner'ı başlat (artık Node değil, normal bir class)
        self.coverage_planner = CoveragePlanner(logger=self.get_logger())
        self.coverage_waypoints = []
        self.current_waypoint_index = 0
        self.is_cleaning = False
        self.is_docking = False
        
        # Map'i dinle
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )
        
        # Dock (Şarj İstasyonu) Hedef Noktası
        self.dock_pose = PoseStamped()
        self.dock_pose.header.frame_id = 'map'
        self.dock_pose.pose.position.x = 1.0   # Robotun duracağı yer
        self.dock_pose.pose.position.y = 0.0 # Fixed goal Y coordinate.
        # Robotun yüzü duvara (istasyona) dönük dursun
        self.dock_pose.pose.orientation.w = 1.0

        # Komutları dinlemek için bir topic (RViz'den veya terminalden tetiklemek için)
        self.command_sub = self.create_subscription(
            String,
            '/cleaning_command',
            self.command_callback,
            10)
        
        # Timer to process coverage waypoints
        self.timer = self.create_timer(0.5, self.check_cleaning_progress)
        
        self.get_logger().info('Behavior Manager Hazır. Komut bekleniyor... (start/stop/dock)')
    
    def map_callback(self, msg):
        """Receive map and pass to coverage planner"""
        self.coverage_planner.set_map(msg)

    def command_callback(self, msg):
        command = msg.data.lower()
        
        if command == 'start':
            self.get_logger().info('Temizlik Başlatılıyor...')
            self.start_cleaning()

        elif command == 'stop':
            self.get_logger().info('Temizlik Durduruldu! İptal ediliyor...')
            self.is_cleaning = False
            self.navigator.cancelTask()

        elif command == 'dock':
            self.get_logger().info('Temizlik Bitti veya Durduruldu. Şarj istasyonuna dönülüyor...')
            self.is_cleaning = False
            self.return_to_dock()

    def return_to_dock(self):
        """Start returning to dock (non-blocking)"""
        self.get_logger().info('Nav2 sistemi bekleniyor...')
        self.navigator.waitUntilNav2Active()
        
        self.get_logger().info('Şarj istasyonuna rota oluşturuluyor...')
        self.is_docking = True
        self.navigator.goToPose(self.dock_pose)
        self.get_logger().info('Docking komutu gönderildi. İlerleme izleniyor...')
    
    def start_cleaning(self):
        """Start the cleaning process with coverage planning"""
        self.get_logger().info('Harita bekleniyor...')
        
        # Wait for map to be ready
        max_wait = 30  # seconds
        start_time = time.time()
        while not self.coverage_planner.is_map_ready():
            if time.time() - start_time > max_wait:
                self.get_logger().error('Harita alınamadı! Lütfen SLAM/Nav2\'nin çalıştığından emin olun.')
                return
            time.sleep(0.5)
        clock=self.get_clock()
        self.get_logger().info('Harita alındı. Coverage path oluşturuluyor...')
        
        # Generate coverage path
        waypoints = self.coverage_planner.generate_coverage_path(clock=self.get_clock())
        
        if not waypoints:
            self.get_logger().error('Coverage path oluşturulamadı!')
            return
        
        # Optimize path (remove points too close to obstacles)
        self.coverage_waypoints = self.coverage_planner.optimize_path(waypoints)
        
        if not self.coverage_waypoints:
            self.get_logger().error('Optimize edilmiş path oluşturulamadı!')
            return
        
        self.get_logger().info(f'Coverage path hazır: {len(self.coverage_waypoints)} waypoint')
        
        # Start cleaning
        self.current_waypoint_index = 0
        self.is_cleaning = True
        
        # Wait for Nav2 to be ready
        self.navigator.waitUntilNav2Active()
        
        # Start following waypoints
        self.get_logger().info('Temizlik başlıyor!')
        self.follow_next_waypoint()
    
    def follow_next_waypoint(self):
        """Navigate to the next waypoint in the coverage path"""
        if not self.is_cleaning:
            return
        
        if self.current_waypoint_index >= len(self.coverage_waypoints):
            self.get_logger().info('✓ TÜM ALAN TEMİZLENDİ!')
            self.is_cleaning = False
            return
        
        waypoint = self.coverage_waypoints[self.current_waypoint_index]
        self.get_logger().info(
            f'Waypoint {self.current_waypoint_index + 1}/{len(self.coverage_waypoints)} hedefleniyor...'
        )
        
        self.navigator.goToPose(waypoint)
        self.current_waypoint_index += 1
    
    def check_cleaning_progress(self):
        """Timer callback to check if we need to navigate to next waypoint or check docking"""
        # Check docking progress
        if self.is_docking:
            if self.navigator.isTaskComplete():
                result = self.navigator.getResult()
                if result == TaskResult.SUCCEEDED:
                    self.get_logger().info('✓ Docking BAŞARILI!')
                elif result == TaskResult.FAILED:
                    self.get_logger().error('✗ Docking BAŞARISIZ! Engel olabilir.')
                elif result == TaskResult.CANCELED:
                    self.get_logger().warn('Docking iptal edildi')
                self.is_docking = False
            return
        
        # Check cleaning progress
        if not self.is_cleaning:
            return
        
        if self.navigator.isTaskComplete():
            result = self.navigator.getResult()
            
            if result == TaskResult.SUCCEEDED:
                self.get_logger().info('Waypoint tamamlandı ✓')
                self.follow_next_waypoint()
            elif result == TaskResult.CANCELED:
                self.get_logger().warn('Waypoint iptal edildi')
                self.is_cleaning = False
            elif result == TaskResult.FAILED:
                self.get_logger().error('Waypoint başarısız! Bir sonrakine geçiliyor...')
                self.follow_next_waypoint()
            
    def start_cleaning_demo(self):
        # Demo amaçlı salonun ortasına gitsin (artık kullanılmıyor)
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.pose.position.x = -3.0
        goal_pose.pose.position.y = 2.0
        goal_pose.pose.orientation.w = 1.0
        self.navigator.goToPose(goal_pose)

def main(args=None):
    rclpy.init(args=args)
    node = BehaviorManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()