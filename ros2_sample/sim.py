#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from math import sin, cos

class RobotSimulator(Node):
    def __init__(self):
        super().__init__('robot_simulator')

        # 发布 /pose_with_cov
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/pose_with_cov', 10)

        # 发布 /odom 里程计
        self.odom_pub = self.create_publisher(Odometry, '/diff_chassis_driver/odom', 10)

        # 发布 tf
        self.tf_broadcaster = TransformBroadcaster(self)

        # 订阅 /cmd_vel
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # 初始机器人状态
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

        # 定时器，每秒发布 /pose_with_cov, /odom 和 tf 20Hz
        self.dt = 1.0 / 20.0
        self.timer = self.create_timer(self.dt, self.update_pose_and_tf)

    def cmd_vel_callback(self, msg: Twist):
        # 从 /cmd_vel 话题更新机器人速度（机体坐标系）
        print("Received cmd_vel:", msg)
        self.linear_velocity = msg.linear.x
        self.angular_velocity = msg.angular.z

    def update_pose_and_tf(self):
        # 根据速度更新位姿（简化的差分驱动运动学，机体朝向为 self.theta）
        self.x += self.linear_velocity * self.dt * cos(self.theta)
        self.y += self.linear_velocity * self.dt * sin(self.theta)
        self.theta += self.angular_velocity * self.dt

        # ---- /pose_with_cov ----
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.pose.position.x = self.x
        pose_msg.pose.pose.position.y = self.y
        pose_msg.pose.pose.position.z = 0.0
        pose_msg.pose.pose.orientation.z = sin(self.theta / 2.0)
        pose_msg.pose.pose.orientation.w = cos(self.theta / 2.0)
        self.pose_pub.publish(pose_msg)

        # ---- /odom ----
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'         # 里程计参考系
        odom_msg.child_frame_id = 'base_link'     # 机器人机体坐标系

        # 位姿
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation.z = sin(self.theta / 2.0)
        odom_msg.pose.pose.orientation.w = cos(self.theta / 2.0)

        # ✅ 速度（机体坐标系下，通常填在 child_frame_id = base_link）
        odom_msg.twist.twist.linear.x  = self.linear_velocity   
        odom_msg.twist.twist.linear.y  = 0.0
        odom_msg.twist.twist.linear.z  = 0.0
        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = self.angular_velocity

        # （可选）简单给个协方差示例：不确定性适中
        # odom_msg.twist.covariance[0]  = 0.05   # vx 方差
        # odom_msg.twist.covariance[35] = 0.05   # wz 方差

        self.odom_pub.publish(odom_msg)

        # 广播 TF
        self.broadcast_tf()

    def broadcast_tf(self):
        t = self.get_clock().now().to_msg()
        transform = TransformStamped()
        transform.header.stamp = t
        transform.header.frame_id = 'map'       # 注意：这里用的是 map->base_link
        transform.child_frame_id = 'base_link'

        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.0
        transform.transform.rotation.z = sin(self.theta / 2.0)
        transform.transform.rotation.w = cos(self.theta / 2.0)

        self.tf_broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = RobotSimulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()