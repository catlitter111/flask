#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():
    """
    启动人体检测系统的完整launch文件
    包含相机节点、深度服务节点和人体检测节点
    """
    
    # 声明launch参数
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
    
    use_dlrobot_driver_arg = DeclareLaunchArgument(
        'use_dlrobot_driver',
        default_value='true',
        description='Whether to launch DLRobot hardware driver'
    )
    
    dlrobot_port_arg = DeclareLaunchArgument(
        'dlrobot_port',
        default_value='/dev/ttyACM0',
        description='DLRobot serial port'
    )
    
    # 获取launch配置
    camera_name = LaunchConfiguration('camera_name')
    use_astra_camera = LaunchConfiguration('use_astra_camera')
    use_depth_service = LaunchConfiguration('use_depth_service')
    use_dlrobot_driver = LaunchConfiguration('use_dlrobot_driver')
    dlrobot_port = LaunchConfiguration('dlrobot_port')
    
    # Astra相机节点
    astra_camera_node = Node(
        package='astra_camera',
        executable='astra_camera_node',
        name='astra_camera_node',
        namespace=camera_name,
        output='screen',
        parameters=[{
            'camera_name': camera_name,
            'serial_number': 'ACRD233006M',
            'vendor_id': '0x2bc5',
            'product_id': '0x050f',
            'enable_depth': True,
            'enable_color': True,
            'enable_ir': False,
            'enable_point_cloud': False,
            'depth_width': 640,
            'depth_height': 480,
            'depth_fps': 30,
            'color_width': 640,
            'color_height': 480,
            'color_fps': 30,
            'use_uvc_camera': True,
            'uvc_vendor_id': 0x2bc5,
            'uvc_product_id': 0x050f,
            'uvc_camera_format': 'mjpeg',
            'publish_tf': True,
            'tf_publish_rate': 10.0,
            'connection_delay': 100,
        }]
    )
    
    # 深度服务节点
    depth_service_node = Node(
        package='astra_depth_reader',
        executable='depth_service',
        name='depth_service_node',
        output='screen',
        remappings=[
            ('/camera/color/image_raw', [camera_name, '/color/image_raw']),
            ('/camera/depth/image_raw', [camera_name, '/depth/image_raw']),
            ('/camera/depth/camera_info', [camera_name, '/depth/camera_info']),
        ],
        condition=IfCondition(use_depth_service)
    )
    
    # 人体检测节点
    person_detection_node = Node(
        package='following_robot',
        executable='person_detection_distance_node',
        name='person_detection_distance_node',
        output='screen',
        remappings=[
            ('/camera/color/image_raw', [camera_name, '/color/image_raw']),
        ],
        parameters=[
            {'detection_fps': 10.0},
            {'distance_query_timeout': 2.0},
        ]
    )
    
    # 机器人控制节点
    robot_control_node = Node(
        package='following_robot',
        executable='robot_control_node',
        name='robot_control_node',
        output='screen',
        parameters=[
            {'max_linear_speed': 0.3},
            {'max_angular_speed': 0.5},
            {'min_follow_distance': 1.0},
            {'max_follow_distance': 3.0},
            {'follow_speed_factor': 0.3},
            {'wheelbase': 0.143},
            {'use_ackermann': False},
            {'safety_enabled': True},
        ]
    )
    
    # WebSocket桥接节点
    websocket_bridge_node = Node(
        package='following_robot',
        executable='websocket_bridge_node',
        name='websocket_bridge_node',
        output='screen',
        parameters=[
            {'websocket_host': '101.201.150.96'},
            {'websocket_port': 1234},
            {'robot_id': 'companion_robot_001'},
            {'enable_image_stream': True},
            {'enable_status_report': True},
            {'enable_command_receive': True},
        ]
    )
    
    # DLRobot硬件驱动节点
    dlrobot_driver_node = Node(
        package='dlrobot_robot_python',
        executable='dlrobot_robot_node',
        name='dlrobot_robot_node',
        output='screen',
        parameters=[{
            'usart_port_name': dlrobot_port,
            'serial_baud_rate': 115200,
            'robot_frame_id': 'base_footprint',
            'odom_frame_id': 'odom_combined',
            'cmd_vel': 'cmd_vel',
            'akm_cmd_vel': 'none',
        }],
        condition=IfCondition(use_dlrobot_driver)
    )
    
    # RViz2可视化节点（可选）
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', '/userdata/try_again/SelfFollowingROS2/src/following_robot/rviz/person_detection.rviz'],
        condition=IfCondition('false')  # 默认不启动，可以通过参数控制
    )
    
    return LaunchDescription([
        camera_name_arg,
        use_astra_camera_arg,
        use_depth_service_arg,
        use_dlrobot_driver_arg,
        dlrobot_port_arg,
        
        astra_camera_node,
        depth_service_node,
        person_detection_node,
        robot_control_node,
        websocket_bridge_node,
        dlrobot_driver_node,
        rviz_node,
    ])