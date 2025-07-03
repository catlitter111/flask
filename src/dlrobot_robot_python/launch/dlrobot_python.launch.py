#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
DLRobot Python机器人底盘驱动启动文件
==================================
功能: 启动DLRobot Python版本的机器人底盘驱动节点
支持普通模式和阿克曼模式

作者: AI Assistant
日期: 2024
基于: 原C++版本的tank.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
import launch_ros.actions


def generate_launch_description():
    """生成启动描述"""
    
    # 声明启动参数
    akmcar_arg = DeclareLaunchArgument(
        'akmcar',
        default_value='false',
        description='Enable Ackermann mode if true, otherwise use differential drive mode'
    )
    
    usart_port_arg = DeclareLaunchArgument(
        'usart_port_name',
        default_value='/dev/dlrobot_controller',
        description='Serial port device name'
    )
    
    serial_baud_rate_arg = DeclareLaunchArgument(
        'serial_baud_rate',
        default_value='115200',
        description='Serial communication baud rate'
    )
    
    robot_frame_id_arg = DeclareLaunchArgument(
        'robot_frame_id',
        default_value='base_footprint',
        description='Robot base frame ID'
    )
    
    odom_frame_id_arg = DeclareLaunchArgument(
        'odom_frame_id',
        default_value='odom_combined',
        description='Odometry frame ID'
    )
    
    wheelbase_arg = DeclareLaunchArgument(
        'wheelbase',
        default_value='0.143',
        description='Wheelbase distance for Ackermann steering (m)'
    )
    
    # 获取启动配置
    akmcar = LaunchConfiguration('akmcar')
    usart_port_name = LaunchConfiguration('usart_port_name')
    serial_baud_rate = LaunchConfiguration('serial_baud_rate')
    robot_frame_id = LaunchConfiguration('robot_frame_id')
    odom_frame_id = LaunchConfiguration('odom_frame_id')
    wheelbase = LaunchConfiguration('wheelbase')
    
    # 阿克曼模式节点配置
    dlrobot_node_akm = launch_ros.actions.Node(
        condition=IfCondition(akmcar),
        package='dlrobot_robot_python',
        executable='dlrobot_robot_node',
        name='dlrobot_robot_node',
        parameters=[{
            'usart_port_name': usart_port_name,
            'serial_baud_rate': serial_baud_rate,
            'robot_frame_id': robot_frame_id,
            'odom_frame_id': odom_frame_id,
            'cmd_vel': 'cmd_vel',
            'akm_cmd_vel': 'ackermann_cmd',
        }],
        remappings=[
            ('/cmd_vel', 'cmd_vel'),
        ],
        output='screen'
    )
    
    # Twist到Ackermann转换节点（阿克曼模式）
    cmd_vel_to_ackermann_node = launch_ros.actions.Node(
        condition=IfCondition(akmcar),
        package='dlrobot_robot_python',
        executable='cmd_vel_to_ackermann',
        name='cmd_vel_to_ackermann_drive',
        parameters=[{
            'wheelbase': wheelbase,
            'frame_id': odom_frame_id,
            'input_topic': 'cmd_vel',
            'output_topic': '/ackermann_cmd',
        }],
        output='screen'
    )
    
    # 普通模式节点配置
    dlrobot_node_normal = launch_ros.actions.Node(
        condition=UnlessCondition(akmcar),
        package='dlrobot_robot_python',
        executable='dlrobot_robot_node',
        name='dlrobot_robot_node',
        parameters=[{
            'usart_port_name': usart_port_name,
            'serial_baud_rate': serial_baud_rate,
            'robot_frame_id': robot_frame_id,
            'odom_frame_id': odom_frame_id,
            'cmd_vel': 'cmd_vel',
            'akm_cmd_vel': 'none',
        }],
        output='screen'
    )
    
    return LaunchDescription([
        # 启动参数
        akmcar_arg,
        usart_port_arg,
        serial_baud_rate_arg,
        robot_frame_id_arg,
        odom_frame_id_arg,
        wheelbase_arg,
        
        # 节点配置
        dlrobot_node_akm,
        cmd_vel_to_ackermann_node,
        dlrobot_node_normal,
    ]) 