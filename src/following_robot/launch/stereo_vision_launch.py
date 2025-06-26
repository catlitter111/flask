#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
双目立体视觉节点启动文件
=======================
用于启动双目立体视觉节点的ROS2 launch文件

作者: AI Assistant
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
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
    
    frame_width_arg = DeclareLaunchArgument(
        'frame_width',
        default_value='1280',
        description='图像宽度'
    )
    
    frame_height_arg = DeclareLaunchArgument(
        'frame_height',
        default_value='480',
        description='图像高度'
    )
    
    # 双目立体视觉节点
    stereo_vision_node = Node(
        package='following_robot',
        executable='stereo_vision_node',
        name='stereo_vision_node',
        output='screen',
        parameters=[{
            'camera_id': LaunchConfiguration('camera_id'),
            'frame_width': LaunchConfiguration('frame_width'),
            'frame_height': LaunchConfiguration('frame_height'),
        }],
        remappings=[
            ('/stereo/left_image', '/following_robot/left_image'),
            ('/stereo/get_distance', '/following_robot/get_distance'),
        ]
    )
    
    return LaunchDescription([
        # Launch参数
        camera_id_arg,
        frame_width_arg,
        frame_height_arg,
        
        # 启动信息
        LogInfo(msg='启动双目立体视觉系统...'),
        
        # 节点
        stereo_vision_node,
        
        LogInfo(msg='双目立体视觉系统启动完成'),
    ]) 