#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
小车控制启动文件
===============
启动小车控制节点和相关功能
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    """生成启动描述"""
    
    # 启动参数
    use_ackermann = LaunchConfiguration('use_ackermann', default='false')
    max_linear_speed = LaunchConfiguration('max_linear_speed', default='0.5')
    max_angular_speed = LaunchConfiguration('max_angular_speed', default='1.0')
    min_follow_distance = LaunchConfiguration('min_follow_distance', default='1.0')
    max_follow_distance = LaunchConfiguration('max_follow_distance', default='3.0')
    follow_speed_factor = LaunchConfiguration('follow_speed_factor', default='0.3')
    wheelbase = LaunchConfiguration('wheelbase', default='0.143')
    safety_enabled = LaunchConfiguration('safety_enabled', default='true')
    
    # 底层驱动参数
    usart_port_name = LaunchConfiguration('usart_port_name', default='/dev/dlrobot_controller')
    serial_baud_rate = LaunchConfiguration('serial_baud_rate', default='115200')
    robot_frame_id = LaunchConfiguration('robot_frame_id', default='base_footprint')
    odom_frame_id = LaunchConfiguration('odom_frame_id', default='odom_combined')
    product_number = LaunchConfiguration('product_number', default='0')
    
    return LaunchDescription([
        # 声明启动参数
        DeclareLaunchArgument(
            'use_ackermann',
            default_value='false',
            description='是否使用阿克曼转向模式'),
        
        DeclareLaunchArgument(
            'max_linear_speed',
            default_value='0.5',
            description='最大线速度 (m/s)'),
        
        DeclareLaunchArgument(
            'max_angular_speed',
            default_value='1.0',
            description='最大角速度 (rad/s)'),
        
        DeclareLaunchArgument(
            'min_follow_distance',
            default_value='1.0',
            description='最小跟随距离 (m)'),
        
        DeclareLaunchArgument(
            'max_follow_distance',
            default_value='3.0',
            description='最大跟随距离 (m)'),
        
        DeclareLaunchArgument(
            'follow_speed_factor',
            default_value='0.3',
            description='跟随速度因子'),
        
        DeclareLaunchArgument(
            'wheelbase',
            default_value='0.143',
            description='车辆轴距 (m)'),
        
        DeclareLaunchArgument(
            'safety_enabled',
            default_value='true',
            description='是否启用安全功能'),
        
        DeclareLaunchArgument(
            'usart_port_name',
            default_value='/dev/dlrobot_controller',
            description='串口设备名称'),
        
        DeclareLaunchArgument(
            'serial_baud_rate',
            default_value='115200',
            description='串口波特率'),
        
        DeclareLaunchArgument(
            'robot_frame_id',
            default_value='base_footprint',
            description='机器人坐标系ID'),
        
        DeclareLaunchArgument(
            'odom_frame_id',
            default_value='odom_combined',
            description='里程计坐标系ID'),
        
        DeclareLaunchArgument(
            'product_number',
            default_value='0',
            description='产品编号'),
        
        # 小车控制节点
        Node(
            package='following_robot',
            executable='robot_control_node',
            name='robot_control_node',
            output='screen',
            parameters=[{
                'use_ackermann': use_ackermann,
                'max_linear_speed': max_linear_speed,
                'max_angular_speed': max_angular_speed,
                'min_follow_distance': min_follow_distance,
                'max_follow_distance': max_follow_distance,
                'follow_speed_factor': follow_speed_factor,
                'wheelbase': wheelbase,
                'safety_enabled': safety_enabled,
            }],
            remappings=[
                ('/cmd_vel', '/cmd_vel'),
                ('/ackermann_cmd', '/ackermann_cmd'),
                ('/odom', '/odom'),
            ]
        ),
        
        # 阿克曼模式：启动底层驱动和转换节点
        GroupAction(
            condition=IfCondition(use_ackermann),
            actions=[
                # 底层驱动节点（阿克曼模式）
                Node(
                    package='turn_on_dlrobot_robot',
                    executable='dlrobot_robot_node',
                    name='dlrobot_robot_node',
                    output='screen',
                    parameters=[{
                        'usart_port_name': usart_port_name,
                        'serial_baud_rate': serial_baud_rate,
                        'robot_frame_id': robot_frame_id,
                        'odom_frame_id': odom_frame_id,
                        'cmd_vel': 'cmd_vel',
                        'akm_cmd_vel': 'ackermann_cmd',
                        'product_number': product_number,
                    }],
                    remappings=[('/cmd_vel', '/cmd_vel')],
                ),
                
                # cmd_vel到ackermann转换节点
                Node(
                    package='turn_on_dlrobot_robot',
                    executable='cmd_vel_to_ackermann_drive.py',
                    name='cmd_vel_to_ackermann_drive',
                    output='screen',
                ),
            ]
        ),
        
        # 标准模式：只启动底层驱动
        Node(
            condition=UnlessCondition(use_ackermann),
            package='turn_on_dlrobot_robot',
            executable='dlrobot_robot_node',
            name='dlrobot_robot_node',
            output='screen',
            parameters=[{
                'usart_port_name': usart_port_name,
                'serial_baud_rate': serial_baud_rate,
                'robot_frame_id': robot_frame_id,
                'odom_frame_id': odom_frame_id,
                'cmd_vel': 'cmd_vel',
                'akm_cmd_vel': 'none',
                'product_number': product_number,
            }],
        ),
    ])


if __name__ == '__main__':
    generate_launch_description() 