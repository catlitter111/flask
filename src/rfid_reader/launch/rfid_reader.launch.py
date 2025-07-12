#!/usr/bin/env python3
"""
RFID读写器启动文件
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
    
    reader_port_arg = DeclareLaunchArgument(
        'reader_port',
        default_value='4001',
        description='RFID读写器端口'
    )
    
    reader_address_arg = DeclareLaunchArgument(
        'reader_address',
        default_value='255',  # 0xFF
        description='RFID读写器地址 (255=0xFF为广播地址)'
    )
    
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='1.0',
        description='发布频率 (Hz)'
    )
    
    auto_start_arg = DeclareLaunchArgument(
        'auto_start',
        default_value='true',
        description='是否自动开始盘存'
    )
    
    antenna_id_arg = DeclareLaunchArgument(
        'antenna_id',
        default_value='1',
        description='工作天线ID'
    )

    # RFID读写器节点
    rfid_node = Node(
        package='rfid_reader',
        executable='rfid_reader_node.py',
        name='rfid_reader',
        output='screen',
        parameters=[{
            'reader_ip': LaunchConfiguration('reader_ip'),
            'reader_port': LaunchConfiguration('reader_port'),
            'reader_address': LaunchConfiguration('reader_address'),
            'publish_rate': LaunchConfiguration('publish_rate'),
            'auto_start': LaunchConfiguration('auto_start'),
            'antenna_id': LaunchConfiguration('antenna_id'),
        }],
        remappings=[
            # 可以在这里重映射话题名称
        ]
    )

    # 启动信息
    start_info = LogInfo(
        msg='RFID读写器系统启动中...'
    )

    return LaunchDescription([
        reader_ip_arg,
        reader_port_arg,
        reader_address_arg,
        publish_rate_arg,
        auto_start_arg,
        antenna_id_arg,
        start_info,
        rfid_node,
    ])