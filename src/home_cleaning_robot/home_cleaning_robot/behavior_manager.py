#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

class BehaviorManager(Node):
    def __init__(self):
        super().__init__('behavior_manager')
        
        # Navigasyon API'sini başlat
        self.navigator = BasicNavigator()
        
        # Robotun başlangıç noktası (Charging Station / Dock)
        # Dock eklenince değiştirilicektir.
        self.dock_pose = PoseStamped()
        self.dock_pose.header.frame_id = 'map'
        self.dock_pose.pose.position.x = 0.0
        self.dock_pose.pose.position.y = -2.0
        self.dock_pose.pose.orientation.w = 1.0

        # Komutları dinlemek için bir topic (RViz'den veya terminalden tetiklemek için)
        self.command_sub = self.create_subscription(
            String,
            '/cleaning_command',
            self.command_callback,
            10)
        
        self.get_logger().info('Behavior Manager Hazır. Komut bekleniyor... (start/stop/dock)')

    def command_callback(self, msg):
        command = msg.data.lower()
        
        if command == 'start':
            self.get_logger().info('Temizlik Başlatılıyor...')
            # Buraya ilerde Coverage Planner gelecek
            # Şimdilik sadece örnek bir noktaya gidiyoruz
            self.start_cleaning_demo()

        elif command == 'stop':
            self.get_logger().info('Temizlik Durduruldu! İptal ediliyor...')
            self.navigator.cancelTask()

        elif command == 'dock':
            self.get_logger().info('Temizlik Bitti veya Durduruldu. Şarj istasyonuna dönülüyor...')
            self.return_to_dock()

    def return_to_dock(self):
        self.get_logger().info('Nav2 sistemi bekleniyor...')
        self.navigator.waitUntilNav2Active() # Nav2 hazır olana kadar bekle
        
        self.get_logger().info('Şarj istasyonuna rota oluşturuluyor...')
        self.navigator.goToPose(self.dock_pose)
        
        # Görev bitene kadar bekle
        while not self.navigator.isTaskComplete():
            pass

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('Docking BAŞARILI!')
        else:
            self.get_logger().error('Docking BAŞARISIZ! Engel olabilir.')
            
    def start_cleaning_demo(self):
        # Demo amaçlı salonun ortasına gitsin
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