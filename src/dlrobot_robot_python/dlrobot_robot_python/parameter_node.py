#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS2参数演示节点 - Python实现
==========================
功能: 演示ROS2参数系统的使用
定期打印参数值

作者: AI Assistant
日期: 2024
基于: 原C++版本的cpp_parameters_node.cpp
"""

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter


class ParameterNode(Node):
    """参数演示节点"""
    
    def __init__(self):
        super().__init__('parameter_node')
        
        # 声明参数
        self.declare_parameter('my_parameter', 'world')
        
        # 创建定时器，每秒执行一次
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        self.get_logger().info('📋 参数节点启动完成')
    
    def timer_callback(self):
        """定时器回调函数"""
        # 获取参数值
        my_param = self.get_parameter('my_parameter').get_parameter_value().string_value
        
        # 打印参数值
        self.get_logger().info(f'Hello {my_param}')


def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    try:
        node = ParameterNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 