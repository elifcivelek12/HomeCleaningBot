#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import tkinter as tk
from threading import Thread

class CleaningControlPanel(Node):
    def __init__(self):
        super().__init__('cleaning_control_panel')
        self.publisher_ = self.create_publisher(String, '/cleaning_command', 10)
        self.root = None
        
        # GUI'yi ayrı bir thread'de başlatıyoruz
        self.gui_thread = Thread(target=self.setup_gui, daemon=True)
        self.gui_thread.start()

    def setup_gui(self):
        self.root = tk.Tk()
        self.root.title("HomeCleanerBot Control")
        self.root.geometry("300x250")
        self.root.attributes('-topmost', True) # Hep üstte kalsın
        
        # Window kapatma protokolü
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

        label = tk.Label(self.root, text="Robot Kontrol Merkezi", font=('Helvetica', 12, 'bold'))
        label.pack(pady=10)

        # Butonlar
        tk.Button(self.root, text="START CLEANING", command=lambda: self.send_cmd("start"), 
                  bg="green", fg="white", width=20, height=2).pack(pady=5)
        
        tk.Button(self.root, text="STOP CLEANING", command=lambda: self.send_cmd("stop"), 
                  bg="red", fg="white", width=20, height=2).pack(pady=5)
        
        tk.Button(self.root, text="RETURN TO DOCK", command=lambda: self.send_cmd("dock"), 
                  bg="blue", fg="white", width=20, height=2).pack(pady=5)

        self.root.mainloop()
    
    def on_closing(self):
        """Window kapatıldığında çağrılır"""
        if self.root:
            self.root.quit()
            self.root.destroy()

    def send_cmd(self, cmd_string):
        msg = String()
        msg.data = cmd_string
        self.publisher_.publish(msg)
        self.get_logger().info(f'Komut Gönderildi: {cmd_string}')

def main(args=None):
    rclpy.init(args=args)
    node = CleaningControlPanel()
    # Node'u kapatana kadar çalıştır
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # GUI'yi kapat
        if node.root:
            try:
                node.root.quit()
            except:
                pass
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()