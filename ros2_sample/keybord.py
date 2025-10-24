#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import termios
import tty
import sys
import select

class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.settings = termios.tcgetattr(sys.stdin)
        self.linear_speed = 0.5
        self.angular_speed = 1.0
        
        self.get_logger().info("键盘控制节点已启动")
        self.get_logger().info("使用方向键或 WASD 控制，按 'q' 退出")
        
    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
    
    def run(self):
        try:
            while rclpy.ok():
                key = self.get_key()
                twist = Twist()
                
                if key == 'w' or key == '\x1b[A':  # 上箭头
                    twist.linear.x = self.linear_speed
                elif key == 's' or key == '\x1b[B':  # 下箭头
                    twist.linear.x = -self.linear_speed
                elif key == 'a' or key == '\x1b[D':  # 左箭头
                    twist.angular.z = self.angular_speed
                elif key == 'd' or key == '\x1b[C':  # 右箭头
                    twist.angular.z = -self.angular_speed
                elif key == 'q':
                    break
                else:
                    # 停止
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                self.publisher_.publish(twist)
                if key:
                    self.get_logger().info(f"按键: {key}, 线速度: {twist.linear.x}, 角速度: {twist.angular.z}")
                
        except Exception as e:
            self.get_logger().error(f"错误: {e}")
        finally:
            # 清理
            twist = Twist()
            self.publisher_.publish(twist)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

def main():
    rclpy.init()
    node = KeyboardTeleop()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()