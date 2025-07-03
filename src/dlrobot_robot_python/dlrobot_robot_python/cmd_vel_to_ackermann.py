#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Twist消息到Ackermann驱动消息转换节点 - Python实现
=============================================
功能: 将geometry_msgs/Twist消息转换为ackermann_msgs/AckermannDriveStamped消息
支持不同轮距的机器人配置

作者: AI Assistant  
日期: 2024
基于: 原C++版本的cmd_vel_to_ackermann_drive.py
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
import math

from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped


class CmdVelToAckermann(Node):
    """Twist到Ackermann转换节点"""
    
    def __init__(self):
        super().__init__('cmd_vel_to_ackermann_drive')
        
        # 声明参数
        self.declare_parameters()
        self.get_parameters()
        
        # 创建发布者和订阅者
        self.create_publishers_subscribers()
        
        self.get_logger().info(f'🚗 Twist到Ackermann转换节点启动完成')
        self.get_logger().info(f'📏 轮距配置: {self.wheelbase}m')
        self.get_logger().info(f'📡 输入话题: {self.input_topic}')
        self.get_logger().info(f'📢 输出话题: {self.output_topic}')
    
    def declare_parameters(self):
        """声明ROS2参数"""
        self.declare_parameter('wheelbase', 0.143)  # mini_akm默认轮距
        self.declare_parameter('frame_id', 'odom_combined')
        self.declare_parameter('cmd_angle_instead_rotvel', False)
        self.declare_parameter('input_topic', 'cmd_vel')
        self.declare_parameter('output_topic', '/ackermann_cmd')
    
    def get_parameters(self):
        """获取ROS2参数"""
        self.wheelbase = self.get_parameter('wheelbase').value
        self.frame_id = self.get_parameter('frame_id').value
        self.cmd_angle_instead_rotvel = self.get_parameter('cmd_angle_instead_rotvel').value
        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        
        # 预设的轮距配置
        wheelbase_configs = {
            'mini_akm': 0.143,
            'senior_akm': 0.320,
            'top_akm_bs': 0.503,
            'top_akm_dl': 0.549
        }
        
        # 如果使用标准配置，记录配置名称
        config_name = 'custom'
        for name, value in wheelbase_configs.items():
            if abs(self.wheelbase - value) < 0.001:
                config_name = name
                break
        
        self.get_logger().info(f'🎛️ 机器人配置: {config_name} (轮距: {self.wheelbase}m)')
    
    def create_publishers_subscribers(self):
        """创建发布者和订阅者"""
        qos = QoSProfile(depth=10)
        
        # 创建发布者
        self.ackermann_publisher = self.create_publisher(
            AckermannDriveStamped, self.output_topic, qos)
        
        # 创建订阅者
        self.cmd_vel_subscriber = self.create_subscription(
            Twist, self.input_topic, self.cmd_vel_callback, qos)
    
    def convert_trans_rot_vel_to_steering_angle(self, v: float, omega: float, wheelbase: float) -> float:
        """
        将线速度和角速度转换为转向角
        
        Args:
            v: 线速度 (m/s)
            omega: 角速度 (rad/s)
            wheelbase: 轮距 (m)
            
        Returns:
            转向角 (rad)
        """
        if omega == 0 or v == 0:
            return 0.0
        
        # 计算转弯半径
        radius = v / omega
        
        # 可选：限制最小转弯半径
        # if abs(radius) < 0.38:
        #     radius = math.copysign(0.38, radius)
        
        # 使用阿克曼转向几何计算转向角
        steering_angle = math.atan(wheelbase / radius)
        
        return steering_angle
    
    def cmd_vel_callback(self, msg: Twist):
        """
        Twist消息回调函数
        
        Args:
            msg: 接收到的Twist消息
        """
        # 提取线速度
        v = msg.linear.x
        
        # 根据配置决定如何处理角度
        if self.cmd_angle_instead_rotvel:
            # 直接使用angular.z作为转向角
            steering = msg.angular.z
        else:
            # 将角速度转换为转向角
            steering = self.convert_trans_rot_vel_to_steering_angle(
                v, msg.angular.z, self.wheelbase)
        
        # 创建Ackermann消息
        ackermann_msg = AckermannDriveStamped()
        ackermann_msg.header.stamp = self.get_clock().now().to_msg()
        ackermann_msg.header.frame_id = self.frame_id
        ackermann_msg.drive.steering_angle = steering
        ackermann_msg.drive.speed = v
        
        # 发布消息
        self.ackermann_publisher.publish(ackermann_msg)
        
        # 记录转换信息
        self.get_logger().debug(
            f'🔄 转换: v={v:.3f}m/s, ω={msg.angular.z:.3f}rad/s → '
            f'speed={v:.3f}m/s, steering={steering:.3f}rad'
        )


def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    try:
        node = CmdVelToAckermann()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 