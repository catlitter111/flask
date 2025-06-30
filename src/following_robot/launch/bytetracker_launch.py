#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ByteTracker ROS2启动文件
=======================
启动ByteTracker节点及相关依赖节点
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """生成启动描述"""
    
    # 声明启动参数
    tracking_mode_arg = DeclareLaunchArgument(
        'tracking_mode',
        default_value='multi',
        description='Tracking mode: multi or single'
    )
    
    target_features_file_arg = DeclareLaunchArgument(
        'target_features_file',
        default_value='',
        description='Path to target features Excel file for single target tracking'
    )
    
    camera_id_arg = DeclareLaunchArgument(
        'camera_id',
        default_value='0',
        description='Camera device ID'
    )
    
    frame_width_arg = DeclareLaunchArgument(
        'frame_width',
        default_value='1280',
        description='Camera frame width (for stereo camera)'
    )
    
    frame_height_arg = DeclareLaunchArgument(
        'frame_height',
        default_value='480',
        description='Camera frame height'
    )
    
    fps_limit_arg = DeclareLaunchArgument(
        'fps_limit',
        default_value='30',
        description='Camera FPS limit'
    )
    
    enable_car_control_arg = DeclareLaunchArgument(
        'enable_car_control',
        default_value='false',
        description='Enable robot car control'
    )
    
    enable_distance_measure_arg = DeclareLaunchArgument(
        'enable_distance_measure',
        default_value='false',
        description='Enable stereo distance measurement'
    )
    
    # ByteTracker节点
    bytetracker_node = Node(
        package='following_robot',
        executable='bytetracker_node',
        name='bytetracker_node',
        output='screen',
        parameters=[{
            'tracking_mode': LaunchConfiguration('tracking_mode'),
            'target_features_file': LaunchConfiguration('target_features_file'),
            'camera_id': LaunchConfiguration('camera_id'),
            'frame_width': LaunchConfiguration('frame_width'),
            'frame_height': LaunchConfiguration('frame_height'),
            'fps_limit': LaunchConfiguration('fps_limit'),
            'enable_car_control': LaunchConfiguration('enable_car_control'),
            'enable_distance_measure': LaunchConfiguration('enable_distance_measure'),
            'track_thresh': 0.5,
            'track_buffer': 100,
            'match_thresh': 0.8,
            'color_weight': 0.5
        }],
        remappings=[
            # 重映射话题（如果需要）
        ]
    )
    
    # 特征提取节点（总是需要）
    feature_extraction_node = Node(
        package='following_robot',
        executable='feature_extraction_node',
        name='feature_extraction_node',
        output='screen'
    )
    
    # 创建启动描述
    ld = LaunchDescription()
    
    # 添加参数
    ld.add_action(tracking_mode_arg)
    ld.add_action(target_features_file_arg)
    ld.add_action(camera_id_arg)
    ld.add_action(frame_width_arg)
    ld.add_action(frame_height_arg)
    ld.add_action(fps_limit_arg)
    ld.add_action(enable_car_control_arg)
    ld.add_action(enable_distance_measure_arg)
    
    # 添加节点
    ld.add_action(bytetracker_node)
    ld.add_action(feature_extraction_node)
    
    return ld