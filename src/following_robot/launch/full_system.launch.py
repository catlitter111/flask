#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
完整系统启动文件 - ByteTracker + WebSocket桥接
==============================================

功能说明：
    同时启动以下组件：
    1. ByteTracker目标跟踪节点
    2. WebSocket桥接节点
    3. 相关的辅助节点
    
使用方法：
    ros2 launch following_robot full_system.launch.py
    ros2 launch following_robot full_system.launch.py websocket_host:=192.168.1.100
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """生成启动描述"""
    
    # 获取包路径
    following_robot_share = FindPackageShare('following_robot')
    
    # 声明通用参数
    declare_websocket_host = DeclareLaunchArgument(
        'websocket_host',
        default_value='101.201.150.96',
        description='WebSocket服务器地址'
    )
    
    declare_websocket_port = DeclareLaunchArgument(
        'websocket_port',
        default_value='1235',
        description='WebSocket服务器端口'
    )
    
    declare_robot_id = DeclareLaunchArgument(
        'robot_id',
        default_value='companion_robot_001',
        description='机器人唯一标识'
    )
    
    declare_camera_index = DeclareLaunchArgument(
        'camera_index',
        default_value='1',
        description='摄像头设备索引'
    )
    
    declare_enable_stereo = DeclareLaunchArgument(
        'enable_stereo',
        default_value='true',
        description='是否启用立体视觉'
    )
    
    declare_tracking_mode = DeclareLaunchArgument(
        'tracking_mode',
        default_value='single_target',
        description='跟踪模式: multi_target 或 single_target'
    )
    
    declare_image_quality = DeclareLaunchArgument(
        'image_quality',
        default_value='80',
        description='图像质量 (1-100)'
    )
    
    declare_frame_rate = DeclareLaunchArgument(
        'frame_rate',
        default_value='30',
        description='视频帧率'
    )
    
    # ByteTracker节点
    bytetracker_node = Node(
        package='following_robot',
        executable='bytetracker_node',
        name='bytetracker_node',
        output='screen',
        parameters=[{
            'camera_index': LaunchConfiguration('camera_index'),
            'enable_stereo_vision': LaunchConfiguration('enable_stereo'),
            'tracking_mode': LaunchConfiguration('tracking_mode'),
            'model_path': PathJoinSubstitution([
                following_robot_share, 'data', 'yolo11n.rknn'
            ]),
            'pose_model_path': PathJoinSubstitution([
                following_robot_share, 'data', 'yolo11n-pose.rknn'
            ]),
            'target_features_path': PathJoinSubstitution([
                following_robot_share, 'data', 'demo_person_features.xlsx'
            ]),
        }],
        remappings=[
            # 重映射话题名称以确保一致性
            ('/bytetracker/image_raw', '/camera/image_raw'),
        ],
        respawn=True,
        respawn_delay=3.0,
        emulate_tty=True,
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
            'frame_rate': LaunchConfiguration('frame_rate'),
            'enable_image_stream': True,
            'enable_status_report': True,
            'enable_command_receive': True,
            'reconnect_interval': 5.0,
        }],
        respawn=True,
        respawn_delay=5.0,
        emulate_tty=True,
    )
    
    # 状态监控节点（可选）
    status_monitor_node = Node(
        package='following_robot',
        executable='status_monitor_node',
        name='status_monitor_node',
        output='screen',
        parameters=[{
            'robot_id': LaunchConfiguration('robot_id'),
            'monitor_interval': 2.0,
            'publish_system_stats': True,
        }],
        respawn=True,
        respawn_delay=3.0,
        emulate_tty=True,
        # 这个节点是可选的，如果不存在不会影响主要功能
        condition=None,
    )
    
    return LaunchDescription([
        # 声明参数
        declare_websocket_host,
        declare_websocket_port,
        declare_robot_id,
        declare_camera_index,
        declare_enable_stereo,
        declare_tracking_mode,
        declare_image_quality,
        declare_frame_rate,
        
        # 启动节点
        bytetracker_node,
        websocket_bridge_node,
        # status_monitor_node,  # 注释掉直到实现
    ]) 