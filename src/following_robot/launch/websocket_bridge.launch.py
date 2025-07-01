#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
WebSocket桥接节点启动文件
========================

功能说明：
    启动WebSocket桥接节点，连接ROS2系统与WebSocket服务器
    
参数配置：
    - websocket_host: WebSocket服务器地址
    - websocket_port: WebSocket服务器端口
    - robot_id: 机器人唯一标识
    - image_quality: 图像质量设置
    - frame_rate: 视频帧率设置

使用方法：
    ros2 launch following_robot websocket_bridge.launch.py
    ros2 launch following_robot websocket_bridge.launch.py websocket_host:=192.168.1.100
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """生成启动描述"""
    
    # 声明启动参数
    declare_websocket_host = DeclareLaunchArgument(
        'websocket_host',
        default_value='172.20.39.181',
        description='WebSocket服务器地址'
    )
    
    declare_websocket_port = DeclareLaunchArgument(
        'websocket_port',
        default_value='1234',
        description='WebSocket服务器端口'
    )
    
    declare_robot_id = DeclareLaunchArgument(
        'robot_id',
        default_value='companion_robot_001',
        description='机器人唯一标识'
    )
    
    declare_image_quality = DeclareLaunchArgument(
        'image_quality',
        default_value='80',
        description='JPEG图像质量 (1-100)'
    )
    
    declare_image_width = DeclareLaunchArgument(
        'image_width',
        default_value='640',
        description='图像宽度（像素）'
    )
    
    declare_image_height = DeclareLaunchArgument(
        'image_height',
        default_value='480',
        description='图像高度（像素）'
    )
    
    declare_frame_rate = DeclareLaunchArgument(
        'frame_rate',
        default_value='15',
        description='视频帧率（FPS）'
    )
    
    declare_enable_image_stream = DeclareLaunchArgument(
        'enable_image_stream',
        default_value='true',
        description='是否启用图像流传输'
    )
    
    declare_enable_status_report = DeclareLaunchArgument(
        'enable_status_report',
        default_value='true',
        description='是否启用状态上报'
    )
    
    declare_enable_command_receive = DeclareLaunchArgument(
        'enable_command_receive',
        default_value='true',
        description='是否启用远程命令接收'
    )
    
    declare_reconnect_interval = DeclareLaunchArgument(
        'reconnect_interval',
        default_value='5.0',
        description='重连间隔时间（秒）'
    )
    
    # WebSocket桥接节点
    websocket_bridge_node = Node(
        package='following_robot',
        executable='websocket_bridge_node',
        name='websocket_bridge_node',
        output='screen',
        parameters=[{
            'websocket_host': LaunchConfiguration('websocket_host'),
            'websocket_port': LaunchConfiguration('websocket_port'),
            'robot_id': LaunchConfiguration('robot_id'),
            'image_quality': LaunchConfiguration('image_quality'),
            'image_width': LaunchConfiguration('image_width'),
            'image_height': LaunchConfiguration('image_height'),
            'frame_rate': LaunchConfiguration('frame_rate'),
            'enable_image_stream': LaunchConfiguration('enable_image_stream'),
            'enable_status_report': LaunchConfiguration('enable_status_report'),
            'enable_command_receive': LaunchConfiguration('enable_command_receive'),
            'reconnect_interval': LaunchConfiguration('reconnect_interval'),
        }],
        respawn=True,
        respawn_delay=5.0,
        emulate_tty=True,
    )
    
    return LaunchDescription([
        # 声明参数
        declare_websocket_host,
        declare_websocket_port,
        declare_robot_id,
        declare_image_quality,
        declare_image_width,
        declare_image_height,
        declare_frame_rate,
        declare_enable_image_stream,
        declare_enable_status_report,
        declare_enable_command_receive,
        declare_reconnect_interval,
        
        # 启动节点
        websocket_bridge_node,
    ]) 