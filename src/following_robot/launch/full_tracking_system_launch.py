#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
完整跟踪系统ROS2启动文件
========================
启动ByteTracker跟踪系统的所有相关节点
包括：ByteTracker、特征提取、双目视觉、小车控制
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """生成启动描述"""
    
    # 获取包的共享目录
    following_robot_dir = get_package_share_directory('following_robot')
    
    # ========== 声明启动参数 ==========
    
    # 跟踪模式参数
    tracking_mode_arg = DeclareLaunchArgument(
        'tracking_mode',
        default_value='single',
        description='Tracking mode: multi (多目标跟踪) or single (单目标跟踪)'
    )
    
    # 目标特征文件参数
    target_features_file_arg = DeclareLaunchArgument(
        'target_features_file',
        default_value=os.path.join(following_robot_dir, 'features-data', 'person.xlsx'),
        description='Path to target features Excel file for single target tracking'
    )
    
    # 相机话题参数
    camera_topic_arg = DeclareLaunchArgument(
        'camera_topic',
        default_value='/camera/image_raw',
        description='Camera image topic'
    )
    
    # 启用小车控制参数
    enable_car_control_arg = DeclareLaunchArgument(
        'enable_car_control',
        default_value='true',
        description='Enable robot car control'
    )
    
    # 启用双目测距参数
    enable_stereo_arg = DeclareLaunchArgument(
        'enable_stereo',
        default_value='true',
        description='Enable stereo vision for distance measurement'
    )
    
    # 启用可视化参数
    enable_visualization_arg = DeclareLaunchArgument(
        'enable_visualization',
        default_value='true',
        description='Enable visualization windows'
    )
    
    # ========== ByteTracker核心节点 ==========
    
    bytetracker_node = Node(
        package='following_robot',
        executable='bytetracker_node',
        name='bytetracker_node',
        output='screen',
        parameters=[{
            'tracking_mode': LaunchConfiguration('tracking_mode'),
            'target_features_file': LaunchConfiguration('target_features_file'),
            'camera_topic': LaunchConfiguration('camera_topic'),
            'enable_car_control': LaunchConfiguration('enable_car_control'),
            'enable_distance_measure': LaunchConfiguration('enable_stereo'),
            # ByteTracker参数
            'track_thresh': 0.5,      # 跟踪阈值
            'track_buffer': 100,      # 轨迹缓冲
            'match_thresh': 0.8,      # 匹配阈值
            'color_weight': 0.5       # 颜色权重
        }],
        remappings=[
            ('/camera/image_raw', LaunchConfiguration('camera_topic'))
        ]
    )
    
    # ========== 特征提取节点 ==========
    
    feature_extraction_node = Node(
        package='following_robot',
        executable='feature_extraction_node',
        name='feature_extraction_node',
        output='screen'
    )
    
    # ========== 双目视觉节点（条件启动） ==========
    
    stereo_vision_node = Node(
        package='following_robot',
        executable='stereo_vision_node',
        name='stereo_vision_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_stereo')),
        parameters=[{
            # 双目相机参数可以在这里配置
        }]
    )
    
    # ========== 小车控制节点（条件启动） ==========
    
    robot_control_node = Node(
        package='following_robot',
        executable='robot_control_node',
        name='robot_control_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_car_control')),
        parameters=[{
            'max_linear_speed': 0.5,
            'max_angular_speed': 1.0,
            'min_follow_distance': 1.0,
            'max_follow_distance': 3.0,
            'follow_speed_factor': 0.3,
            'wheelbase': 0.143,
            'use_ackermann': False,
            'safety_enabled': True
        }]
    )
    
    # ========== 可视化节点（条件启动） ==========
    
    # RViz2配置文件路径
    rviz_config_file = os.path.join(following_robot_dir, 'rviz', 'bytetracker.rviz')
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_visualization')),
        arguments=['-d', rviz_config_file] if os.path.exists(rviz_config_file) else []
    )
    
    # ========== 创建启动描述 ==========
    
    ld = LaunchDescription()
    
    # 添加启动参数
    ld.add_action(tracking_mode_arg)
    ld.add_action(target_features_file_arg)
    ld.add_action(camera_topic_arg)
    ld.add_action(enable_car_control_arg)
    ld.add_action(enable_stereo_arg)
    ld.add_action(enable_visualization_arg)
    
    # 添加日志信息
    ld.add_action(LogInfo(
        msg=['启动ByteTracker跟踪系统，模式: ', LaunchConfiguration('tracking_mode')]
    ))
    
    # 添加核心节点
    ld.add_action(bytetracker_node)
    ld.add_action(feature_extraction_node)
    
    # 添加条件节点
    ld.add_action(stereo_vision_node)
    ld.add_action(robot_control_node)
    ld.add_action(rviz_node)
    
    return ld