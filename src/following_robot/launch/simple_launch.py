#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
简化的双目立体视觉启动文件
========================
快速启动双目立体视觉节点的简化版本

作者: AI Assistant
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import LogInfo


def generate_launch_description():
    """生成简化的launch描述
    
    参数:
        无
        
    返回值:
        LaunchDescription: launch描述对象
    """
    
    # 双目立体视觉节点
    stereo_vision_node = Node(
        package='following_robot',
        executable='stereo_vision_node',
        name='stereo_vision_node',
        output='screen'
    )
    
    return LaunchDescription([
        # 启动信息
        LogInfo(msg='🚀 启动双目立体视觉系统...'),
        LogInfo(msg='⌨️  控制键: q=退出, s=切换立体视觉'),
        LogInfo(msg='📋 窗口将显示实时相机图像和状态信息'),
        
        # 节点
        stereo_vision_node,
        
        LogInfo(msg='✅ 系统启动完成！'),
    ]) 