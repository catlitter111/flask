#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
双目立体视觉节点启动文件
=======================
用于启动双目立体视觉节点的ROS2 launch文件
支持cv2.imshow直接显示模式

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
    
    enable_stereo_arg = DeclareLaunchArgument(
        'enable_stereo',
        default_value='true',
        description='是否启用立体视觉处理'
    )
    
    fps_limit_arg = DeclareLaunchArgument(
        'fps_limit',
        default_value='30',
        description='帧率限制'
    )
    
    # 双目立体视觉节点
    stereo_vision_node = Node(
        package='following_robot',
        executable='stereo_vision_node',
        name='stereo_vision_node',
        output='screen',
        parameters=[{
            'camera_id': LaunchConfiguration('camera_id'),
            'enable_stereo_processing': LaunchConfiguration('enable_stereo'),
            'fps_limit': LaunchConfiguration('fps_limit'),
        }],
        # 只保留距离测量服务的remapping
        remappings=[
            ('/stereo/get_distance', '/following_robot/get_distance'),
        ]
    )
    
    return LaunchDescription([
        # Launch参数
        camera_id_arg,
        enable_stereo_arg,
        fps_limit_arg,
        
        # 启动信息
        LogInfo(msg='启动双目立体视觉系统（cv2.imshow显示模式）...'),
        LogInfo(msg='控制键: q=退出, s=切换立体视觉处理'),
        
        # 节点
        stereo_vision_node,
        
        LogInfo(msg='双目立体视觉系统启动完成'),
    ]) 