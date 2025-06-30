#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
完整跟踪系统ROS2启动文件
========================
启动ByteTracker跟踪系统的所有相关节点
包括：ByteTracker（集成立体视觉）、特征提取、小车控制

更新内容：
- ByteTracker已集成立体视觉距离测量功能
- 支持直接相机读取，无需外部图像话题
- 移除独立的stereo_vision_node
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
    # 尝试找到项目根目录下的features-data文件夹
    # 从install目录向上查找到工作空间根目录
    current_path = following_robot_dir
    project_root = None
    
    # 尝试从install路径找到工作空间根目录
    for _ in range(5):  # 最多向上查找5级目录
        current_path = os.path.dirname(current_path)
        features_path = os.path.join(current_path, 'features-data', 'person.xlsx')
        if os.path.exists(features_path):
            project_root = current_path
            break
    
    # 如果找不到，使用默认路径（用户需要手动指定）
    if project_root is None:
        default_features_file = '/tmp/person.xlsx'  # 临时路径，用户需要手动指定
    else:
        default_features_file = os.path.join(project_root, 'features-data', 'person.xlsx')
    
    target_features_file_arg = DeclareLaunchArgument(
        'target_features_file',
        default_value=default_features_file,
        description='Path to target features Excel file for single target tracking'
    )
    
    # 相机设备参数
    camera_id_arg = DeclareLaunchArgument(
        'camera_id',
        default_value='1',
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
    
    # 立体相机参数
    is_stereo_camera_arg = DeclareLaunchArgument(
        'is_stereo_camera',
        default_value='true',
        description='Whether using stereo camera for distance measurement'
    )
    
    # 启用小车控制参数
    enable_car_control_arg = DeclareLaunchArgument(
        'enable_car_control',
        default_value='true',
        description='Enable robot car control'
    )
    
    # 启用可视化参数
    enable_visualization_arg = DeclareLaunchArgument(
        'enable_visualization',
        default_value='false',
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
            # 相机参数
            'camera_id': LaunchConfiguration('camera_id'),
            'frame_width': LaunchConfiguration('frame_width'),
            'frame_height': LaunchConfiguration('frame_height'),
            'fps_limit': LaunchConfiguration('fps_limit'),
            'is_stereo_camera': LaunchConfiguration('is_stereo_camera'),
            # 功能控制参数
            'enable_car_control': LaunchConfiguration('enable_car_control'),
            'enable_distance_measure': LaunchConfiguration('is_stereo_camera'),
            # ByteTracker算法参数
            'track_thresh': 0.5,      # 跟踪阈值
            'track_buffer': 100,      # 轨迹缓冲
            'match_thresh': 0.8,      # 匹配阈值
            'color_weight': 0.5       # 颜色权重
        }]
    )
    
    # ========== 特征提取节点 ==========
    
    feature_extraction_node = Node(
        package='following_robot',
        executable='feature_extraction_node',
        name='feature_extraction_node',
        output='screen'
    )
    
    # ========== 双目视觉功能已集成到ByteTracker节点 ==========
    # 注意: stereo_vision_node已被移除，距离测量功能现在由bytetracker_node内置提供
    
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
    ld.add_action(camera_id_arg)
    ld.add_action(frame_width_arg)
    ld.add_action(frame_height_arg)
    ld.add_action(fps_limit_arg)
    ld.add_action(is_stereo_camera_arg)
    ld.add_action(enable_car_control_arg)
    ld.add_action(enable_visualization_arg)
    
    # 添加日志信息
    ld.add_action(LogInfo(
        msg=['启动ByteTracker跟踪系统，模式: ', LaunchConfiguration('tracking_mode'),
             ', 相机ID: ', LaunchConfiguration('camera_id'),
             ', 立体相机: ', LaunchConfiguration('is_stereo_camera'),
             ', 特征文件: ', LaunchConfiguration('target_features_file')]
    ))
    
    # 添加核心节点
    ld.add_action(bytetracker_node)
    ld.add_action(feature_extraction_node)
    
    # 添加条件节点
    ld.add_action(robot_control_node)
    ld.add_action(rviz_node)
    
    return ld