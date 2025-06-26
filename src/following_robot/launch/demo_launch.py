#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
演示系统启动文件
================
同时启动双目立体视觉节点和演示测试节点

作者: AI Assistant
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo, TimerAction
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """生成launch描述
    
    参数:
        无
        
    返回值:
        LaunchDescription: launch描述对象
    """
    
    # 声明launch参数
    camera_id_arg = DeclareLaunchArgument(
        'camera_id',
        default_value='1',
        description='相机设备ID'
    )
    
    # 双目立体视觉节点
    stereo_vision_node = Node(
        package='following_robot',
        executable='stereo_vision_node',
        name='stereo_vision_node',
        output='screen',
        parameters=[{
            'camera_id': LaunchConfiguration('camera_id'),
        }]
    )
    
    # 演示测试节点 - 延迟5秒启动，等待立体视觉节点完全启动
    demo_test_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='following_robot',
                executable='demo_test',
                name='demo_test_node',
                output='screen'
            )
        ]
    )
    
    return LaunchDescription([
        # Launch参数
        camera_id_arg,
        
        # 启动信息
        LogInfo(msg='启动双目立体视觉演示系统...'),
        
        # 节点
        stereo_vision_node,
        demo_test_node,
        
        LogInfo(msg='双目立体视觉演示系统启动完成'),
    ]) 