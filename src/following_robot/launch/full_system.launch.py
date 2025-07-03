#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
完整系统启动文件 - ByteTracker + WebSocket桥接 + 特征提取
=========================================================

功能说明：
    同时启动以下组件：
    1. 特征提取服务节点 (提供人体特征提取服务)
    2. ByteTracker目标跟踪节点
    3. WebSocket桥接节点
    4. 机器人控制节点
    5. 底层硬件驱动节点 (Python版本 - dlrobot_robot_python)
    
使用方法：
    # 基础启动
    ros2 launch following_robot full_system.launch.py
    
    # 自定义WebSocket服务器
    ros2 launch following_robot full_system.launch.py websocket_host:=192.168.1.100
    
    # 启用阿克曼转向模式
    ros2 launch following_robot full_system.launch.py use_ackermann:=true
    
    # 配置不同机器人型号
    ros2 launch following_robot full_system.launch.py wheelbase:=0.320  # senior_akm
    ros2 launch following_robot full_system.launch.py wheelbase:=0.503  # top_akm_bs
    
    # 组合配置示例
    ros2 launch following_robot full_system.launch.py \
        use_ackermann:=true \
        wheelbase:=0.143 \
        serial_port:=/dev/dlrobot_controller \
        websocket_host:=192.168.1.100
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """生成启动描述"""
    
    # 获取包路径
    following_robot_share = FindPackageShare('following_robot')
    
    # 声明通用参数
    declare_websocket_host = DeclareLaunchArgument(
        'websocket_host',
        default_value='101.201.150.96',
        description='WebSocket服务器地址'
    )
    
    declare_websocket_port = DeclareLaunchArgument(
        'websocket_port',
        default_value='1234',
        description='WebSocket服务器端口'
    )
    
    declare_robot_id = DeclareLaunchArgument(
        'robot_id',
        default_value='companion_robot_001',
        description='机器人唯一标识'
    )
    
    declare_camera_index = DeclareLaunchArgument(
        'camera_index',
        default_value='0',
        description='摄像头设备索引'
    )
    
    declare_enable_stereo = DeclareLaunchArgument(
        'enable_stereo',
        default_value='true',
        description='是否启用立体视觉'
    )
    
    declare_tracking_mode = DeclareLaunchArgument(
        'tracking_mode',
        default_value='single',
        description='跟踪模式: multi 或 single'
    )
    
    declare_image_quality = DeclareLaunchArgument(
        'image_quality',
        default_value='80',
        description='图像质量 (1-100)'
    )
    
    declare_frame_rate = DeclareLaunchArgument(
        'frame_rate',
        default_value='30',
        description='视频帧率'
    )
    
    declare_use_ackermann = DeclareLaunchArgument(
        'use_ackermann',
        default_value='false',
        description='是否使用阿克曼转向控制'
    )
    
    declare_serial_port = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyS7',
        description='串口设备路径'
    )
    
    declare_serial_baud = DeclareLaunchArgument(
        'serial_baud',
        default_value='115200',
        description='串口波特率'
    )
    
    declare_feature_output_dir = DeclareLaunchArgument(
        'feature_output_dir',
        default_value='features-data',
        description='特征提取输出目录'
    )
    
    declare_wheelbase = DeclareLaunchArgument(
        'wheelbase',
        default_value='0.143',
        description='机器人轮距 (m): mini_akm=0.143, senior_akm=0.320, top_akm_bs=0.503, top_akm_dl=0.549'
    )
    
    # ByteTracker节点
    bytetracker_node = Node(
        package='following_robot',
        executable='bytetracker_node',
        name='bytetracker_node',
        output='screen',
        parameters=[{
            'camera_id': LaunchConfiguration('camera_index'),
            'is_stereo_camera': LaunchConfiguration('enable_stereo'),
            'tracking_mode': LaunchConfiguration('tracking_mode'),
            'model_path': PathJoinSubstitution([
                following_robot_share, 'data', 'best3.rknn'
            ]),
            'pose_model_path': PathJoinSubstitution([
                following_robot_share, 'data', 'yolov8_pose.rknn'
            ]),
            'target_features_file': PathJoinSubstitution([
                following_robot_share, 'data', 'demo_person_features.xlsx'
            ]),
        }],
        remappings=[
            # 重映射话题名称以确保一致性
            ('/bytetracker/image_raw', '/camera/image_raw'),
        ],
        respawn=True,
        respawn_delay=3.0,
        emulate_tty=True,
    )
    
    # 特征提取服务节点
    feature_extraction_node = Node(
        package='following_robot',
        executable='feature_extraction_node',
        name='feature_extraction_node',
        output='screen',
        parameters=[{
            'output_dir': LaunchConfiguration('feature_output_dir'),
        }],
        remappings=[
            # 确保服务名称一致
            ('/features/extract_features', '/features/extract_features'),
        ],
        respawn=True,
        respawn_delay=3.0,
        emulate_tty=True,
    )
    
    # WebSocket桥接节点
    websocket_bridge_node = Node(
        package='following_robot',
        executable='websocket_bridge_node',
        name='websocket_bridge_node',
        output='screen',
        parameters=[{
            'websocket_host': LaunchConfiguration('websocket_host'),
            'websocket_port': LaunchConfiguration('websocket_port'),
            'robot_id': LaunchConfiguration('robot_id'),
            'image_quality': LaunchConfiguration('image_quality'),
            'frame_rate': LaunchConfiguration('frame_rate'),
            'image_width': 640,
            'image_height': 480,
            'enable_image_stream': True,
            'enable_status_report': True,
            'enable_command_receive': True,
            'reconnect_interval': 5.0,
        }],
        respawn=True,
        respawn_delay=5.0,
        emulate_tty=True,
    )
    
    # 机器人控制节点
    robot_control_node = Node(
        package='following_robot',
        executable='robot_control_node',
        name='robot_control_node',
        output='screen',
        parameters=[{
            'max_linear_speed': 0.5,
            'max_angular_speed': 1.0,
            'min_follow_distance': 1.0,
            'max_follow_distance': 3.0,
            'follow_speed_factor': 0.3,
            'wheelbase': 0.143,
            'use_ackermann': LaunchConfiguration('use_ackermann'),
            'safety_enabled': True,
        }],
        remappings=[
            # 确保话题名称一致
            ('cmd_vel', '/cmd_vel'),
            ('ackermann_cmd', '/ackermann_cmd'),
        ],
        respawn=True,
        respawn_delay=3.0,
        emulate_tty=True,
    )
    
    # 底层硬件驱动节点 - 普通模式 (差分驱动) - Python版本
    dlrobot_driver_node = Node(
        package='dlrobot_robot_python',
        executable='dlrobot_robot_node',
        name='dlrobot_robot_node',
        output='screen',
        parameters=[{
            'usart_port_name': LaunchConfiguration('serial_port'),
            'serial_baud_rate': LaunchConfiguration('serial_baud'),
            'robot_frame_id': 'base_footprint',
            'odom_frame_id': 'odom_combined',
            'gyro_frame_id': 'gyro_link',
            'cmd_vel': 'cmd_vel',
            'akm_cmd_vel': 'none',
        }],
        remappings=[
            # 重映射话题到全局命名空间
            ('cmd_vel', '/cmd_vel'),
            ('odom_combined', '/odom'),
        ],
        condition=UnlessCondition(LaunchConfiguration('use_ackermann')),
        respawn=True,
        respawn_delay=3.0,
        emulate_tty=True,
    )
    
    # 底层硬件驱动节点 - 阿克曼模式 - Python版本
    dlrobot_driver_node_akm = Node(
        package='dlrobot_robot_python',
        executable='dlrobot_robot_node',
        name='dlrobot_robot_node',
        output='screen',
        parameters=[{
            'usart_port_name': LaunchConfiguration('serial_port'),
            'serial_baud_rate': LaunchConfiguration('serial_baud'),
            'robot_frame_id': 'base_footprint',
            'odom_frame_id': 'odom_combined',
            'gyro_frame_id': 'gyro_link',
            'cmd_vel': 'cmd_vel',
            'akm_cmd_vel': 'ackermann_cmd',
        }],
        remappings=[
            # 重映射话题到全局命名空间
            ('cmd_vel', '/cmd_vel'),
            ('ackermann_cmd', '/ackermann_cmd'),
            ('odom_combined', '/odom'),
        ],
        condition=IfCondition(LaunchConfiguration('use_ackermann')),
        respawn=True,
        respawn_delay=3.0,
        emulate_tty=True,
    )
    
    # Twist到Ackermann消息转换节点 (仅在阿克曼模式下使用) - Python版本
    cmd_vel_to_ackermann_node = Node(
        package='dlrobot_robot_python',
        executable='cmd_vel_to_ackermann',
        name='cmd_vel_to_ackermann_drive',
        output='screen',
        parameters=[{
            'wheelbase': LaunchConfiguration('wheelbase'),
            'frame_id': 'odom_combined',
            'input_topic': 'cmd_vel',
            'output_topic': '/ackermann_cmd',
        }],
        remappings=[
            # 重映射话题到全局命名空间
            ('cmd_vel', '/cmd_vel'),
            ('ackermann_cmd', '/ackermann_cmd'),
        ],
        condition=IfCondition(LaunchConfiguration('use_ackermann')),
        respawn=True,
        respawn_delay=3.0,
        emulate_tty=True,
    )
    
    # 状态监控节点（可选）
    # status_monitor_node = Node(
    #     package='following_robot',
    #     executable='status_monitor_node',
    #     name='status_monitor_node',
    #     output='screen',
    #     parameters=[{
    #         'robot_id': LaunchConfiguration('robot_id'),
    #         'monitor_interval': 2.0,
    #         'publish_system_stats': True,
    #     }],
    #     respawn=True,
    #     respawn_delay=3.0,
    #     emulate_tty=True,
    # )
    
    return LaunchDescription([
        # 声明参数
        declare_websocket_host,
        declare_websocket_port,
        declare_robot_id,
        declare_camera_index,
        declare_enable_stereo,
        declare_tracking_mode,
        declare_image_quality,
        declare_frame_rate,
        declare_use_ackermann,
        declare_serial_port,
        declare_serial_baud,
        declare_feature_output_dir,
        declare_wheelbase,
        
        # 启动节点 (特征提取节点先启动，确保服务就绪)
        feature_extraction_node,
        bytetracker_node,
        websocket_bridge_node,
        robot_control_node,
        dlrobot_driver_node,
        dlrobot_driver_node_akm,
        cmd_vel_to_ackermann_node,
        # status_monitor_node,  # 注释掉直到实现
    ]) 