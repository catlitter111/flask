#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
完整的人体跟随系统启动文件
========================
功能: 启动完整的人体跟随机器人系统
包含:
- 相机节点
- 深度服务节点
- 人体检测节点
- 人体跟随控制节点
- 机器人驱动节点

作者: AI Assistant
日期: 2024
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """生成完整的人体跟随系统启动描述"""
    
    # 声明启动参数
    camera_name_arg = DeclareLaunchArgument(
        'camera_name',
        default_value='camera',
        description='Camera name for topics'
    )
    
    use_astra_camera_arg = DeclareLaunchArgument(
        'use_astra_camera',
        default_value='true',
        description='Whether to launch Astra camera node'
    )
    
    use_depth_service_arg = DeclareLaunchArgument(
        'use_depth_service',
        default_value='true',
        description='Whether to launch depth service node'
    )
    
    # 机器人相关参数
    usart_port_arg = DeclareLaunchArgument(
        'usart_port_name',
        default_value='/dev/ttyS7',
        description='Serial port device name'
    )
    
    serial_baud_rate_arg = DeclareLaunchArgument(
        'serial_baud_rate',
        default_value='115200',
        description='Serial communication baud rate'
    )
    
    robot_frame_id_arg = DeclareLaunchArgument(
        'robot_frame_id',
        default_value='base_footprint',
        description='Robot base frame ID'
    )
    
    # 跟随控制参数
    target_distance_arg = DeclareLaunchArgument(
        'target_distance',
        default_value='1.0',
        description='Target following distance in meters'
    )
    
    max_linear_speed_arg = DeclareLaunchArgument(
        'max_linear_speed',
        default_value='0.3',
        description='Maximum linear speed in m/s'
    )
    
    max_angular_speed_arg = DeclareLaunchArgument(
        'max_angular_speed',
        default_value='0.5',
        description='Maximum angular speed in rad/s'
    )
    
    debug_mode_arg = DeclareLaunchArgument(
        'debug_mode',
        default_value='true',
        description='Enable debug mode for detailed logging'
    )
    
    # 获取启动配置
    camera_name = LaunchConfiguration('camera_name')
    use_astra_camera = LaunchConfiguration('use_astra_camera')
    use_depth_service = LaunchConfiguration('use_depth_service')
    usart_port_name = LaunchConfiguration('usart_port_name')
    serial_baud_rate = LaunchConfiguration('serial_baud_rate')
    robot_frame_id = LaunchConfiguration('robot_frame_id')
    target_distance = LaunchConfiguration('target_distance')
    max_linear_speed = LaunchConfiguration('max_linear_speed')
    max_angular_speed = LaunchConfiguration('max_angular_speed')
    debug_mode = LaunchConfiguration('debug_mode')
    
    # 1. 包含人体检测系统 launch 文件
    person_detection_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('following_robot'),
                'launch',
                'person_detection_system.launch.py'
            ])
        ]),
        launch_arguments={
            'camera_name': camera_name,
            'use_astra_camera': use_astra_camera,
            'use_depth_service': use_depth_service,
        }.items()
    )
    
    # 2. DLRobot机器人驱动节点
    dlrobot_driver_node = Node(
        package='dlrobot_robot_python',
        executable='dlrobot_robot_node',
        name='dlrobot_robot_node',
        parameters=[{
            'usart_port_name': usart_port_name,
            'serial_baud_rate': serial_baud_rate,
            'robot_frame_id': robot_frame_id,
            'odom_frame_id': 'odom_combined',
            'cmd_vel': 'cmd_vel',
            'akm_cmd_vel': 'none',  # 使用普通驱动模式
        }],
        output='screen'
    )
    
    # 3. 人体跟随控制节点
    person_following_controller_node = Node(
        package='dlrobot_robot_python',
        executable='person_following_controller',
        name='person_following_controller',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('dlrobot_robot_python'),
                'config',
                'person_following_params.yaml'
            ]),
            {
                'target_distance': target_distance,
                'max_linear_speed': max_linear_speed,
                'max_angular_speed': max_angular_speed,
                'debug_mode': debug_mode,
            }
        ],
        output='screen'
    )
    
    # 4. 状态监控节点（可选 - 先注释掉）
    # system_monitor_node = Node(
    #     package='dlrobot_robot_python',
    #     executable='parameter_node',
    #     name='system_monitor',
    #     parameters=[{
    #         'monitor_topics': [
    #             '/person_detection/person_positions',
    #             '/cmd_vel',
    #             '/odom_combined',
    #             '/person_following/status'
    #         ]
    #     }],
    #     output='screen',
    #     condition=IfCondition(debug_mode)
    # )
    
    return LaunchDescription([
        # 启动参数
        camera_name_arg,
        use_astra_camera_arg,
        use_depth_service_arg,
        usart_port_arg,
        serial_baud_rate_arg,
        robot_frame_id_arg,
        target_distance_arg,
        max_linear_speed_arg,
        max_angular_speed_arg,
        debug_mode_arg,
        
        # 节点和包含的launch文件
        person_detection_launch,
        dlrobot_driver_node,
        person_following_controller_node,
        # system_monitor_node,  # 暂时注释掉
    ])