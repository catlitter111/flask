#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
特征提取节点启动文件
==================
启动人体特征提取服务节点
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """生成启动描述"""
    
    # 声明启动参数
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='日志级别 (debug, info, warn, error, fatal)'
    )
    
    # 特征提取节点
    feature_extraction_node = Node(
        package='following_robot',
        executable='feature_extraction_node',
        name='feature_extraction_node',
        output='screen',
        parameters=[
            {'use_sim_time': False}
        ],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
    )
    
    return LaunchDescription([
        log_level_arg,
        LogInfo(
            msg='启动人体特征提取服务节点...'
        ),
        feature_extraction_node,
        LogInfo(
            msg='特征提取服务已启动，服务地址: /features/extract_features'
        )
    ]) 