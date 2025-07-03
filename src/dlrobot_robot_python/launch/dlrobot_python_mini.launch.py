#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
DLRobot Python简化启动文件 - Mini版本
==================================
功能: 快速启动DLRobot Python版本的机器人底盘驱动
预配置mini_akm参数

作者: AI Assistant
日期: 2024
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import launch_ros.actions


def generate_launch_description():
    """生成启动描述"""
    
    # 声明启动参数
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='normal',
        choices=['normal', 'ackermann'],
        description='Robot control mode: normal or ackermann'
    )
    
    # 获取启动配置
    mode = LaunchConfiguration('mode')
    
    # 主控制节点
    dlrobot_node = launch_ros.actions.Node(
        package='dlrobot_robot_python',
        executable='dlrobot_robot_node',
        name='dlrobot_robot_node',
        parameters=[{
            'usart_port_name': '/dev/dlrobot_controller',
            'serial_baud_rate': 115200,
            'robot_frame_id': 'base_footprint',
            'odom_frame_id': 'odom_combined',
            'gyro_frame_id': 'gyro_link',
            'cmd_vel': 'cmd_vel',
            'akm_cmd_vel': 'none',  # 默认普通模式
        }],
        output='screen'
    )
    
    return LaunchDescription([
        mode_arg,
        dlrobot_node,
    ]) 