#!/usr/bin/env python3
"""
RFID读写器调试启动文件 - 包含rqt工具
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 声明启动参数
    reader_ip_arg = DeclareLaunchArgument(
        'reader_ip',
        default_value='192.168.0.178',
        description='RFID读写器IP地址'
    )
    
    # RFID读写器节点
    rfid_node = Node(
        package='rfid_reader',
        executable='rfid_reader_node.py',
        name='rfid_reader',
        output='screen',
        parameters=[{
            'reader_ip': LaunchConfiguration('reader_ip'),
            'reader_port': 4001,
            'reader_address': 255,
            'publish_rate': 2.0,  # 更高的发布频率用于调试
            'auto_start': False,  # 调试模式手动启动
            'antenna_id': 1,
        }]
    )

    # RQT图形界面 (可选，如果系统支持)
    rqt_node = Node(
        package='rqt_topic',
        executable='rqt_topic',
        name='rqt_topic',
        output='screen',
        condition=LaunchConfiguration('use_rqt', default='false')
    )

    # 启动信息
    start_info = LogInfo(
        msg='RFID读写器调试模式启动...'
    )

    return LaunchDescription([
        reader_ip_arg,
        DeclareLaunchArgument('use_rqt', default_value='false', description='启用RQT图形界面'),
        start_info,
        rfid_node,
        rqt_node,
    ])