#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():
    """
    启动人体检测系统的完整launch文件
    包含相机节点、YOLO11检测节点、深度服务节点和人体检测节点
    增强功能：动态速度调节、PID控制、分段速度控制
    """
    
    # 声明基础launch参数
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
    
    use_rknn_yolo11_arg = DeclareLaunchArgument(
        'use_rknn_yolo11',
        default_value='true',
        description='Whether to launch RKNN YOLO11 detection node'
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
    
    # YOLO11检测节点相关参数
    yolo11_model_path_arg = DeclareLaunchArgument(
        'yolo11_model_path',
        default_value='/userdata/rknn_yolo11_ros2/model/cloth.rknn',
        description='Path to YOLO11 RKNN model file'
    )
    
    yolo11_confidence_threshold_arg = DeclareLaunchArgument(
        'yolo11_confidence_threshold',
        default_value='0.25',
        description='YOLO11 confidence threshold'
    )
    
    yolo11_nms_threshold_arg = DeclareLaunchArgument(
        'yolo11_nms_threshold',
        default_value='0.45',
        description='YOLO11 NMS threshold'
    )
    
    yolo11_enable_debug_image_arg = DeclareLaunchArgument(
        'yolo11_enable_debug_image',
        default_value='true',
        description='Enable YOLO11 debug image output'
    )
    
    # 声明基础控制参数
    max_linear_speed_arg = DeclareLaunchArgument(
        'max_linear_speed',
        default_value='0.8',
        description='Maximum linear speed (m/s)'
    )
    
    max_angular_speed_arg = DeclareLaunchArgument(
        'max_angular_speed',
        default_value='0.6',
        description='Maximum angular speed (rad/s)'
    )
    
    min_follow_distance_arg = DeclareLaunchArgument(
        'min_follow_distance',
        default_value='0.8',
        description='Minimum following distance (m)'
    )
    
    max_follow_distance_arg = DeclareLaunchArgument(
        'max_follow_distance',
        default_value='1.2',
        description='Maximum following distance (m)'
    )
    
    follow_speed_factor_arg = DeclareLaunchArgument(
        'follow_speed_factor',
        default_value='0.5',
        description='Following speed factor'
    )
    
    # 声明动态速度调节参数
    enable_dynamic_speed_arg = DeclareLaunchArgument(
        'enable_dynamic_speed',
        default_value='true',
        description='Enable dynamic speed adjustment'
    )
    
    enable_pid_control_arg = DeclareLaunchArgument(
        'enable_pid_control',
        default_value='true',
        description='Enable PID control for following'
    )
    
    enable_speed_smoothing_arg = DeclareLaunchArgument(
        'enable_speed_smoothing',
        default_value='true',
        description='Enable speed smoothing'
    )
    
    distance_prediction_enabled_arg = DeclareLaunchArgument(
        'distance_prediction_enabled',
        default_value='false',
        description='Enable distance prediction for speed adjustment'
    )
    
    # 声明PID控制参数
    distance_pid_kp_arg = DeclareLaunchArgument(
        'distance_pid_kp',
        default_value='1.2',
        description='Distance PID controller proportional gain'
    )
    
    distance_pid_ki_arg = DeclareLaunchArgument(
        'distance_pid_ki',
        default_value='0.15',
        description='Distance PID controller integral gain'
    )
    
    distance_pid_kd_arg = DeclareLaunchArgument(
        'distance_pid_kd',
        default_value='0.08',
        description='Distance PID controller derivative gain'
    )
    
    angle_pid_kp_arg = DeclareLaunchArgument(
        'angle_pid_kp',
        default_value='2.2',
        description='Angle PID controller proportional gain'
    )
    
    angle_pid_ki_arg = DeclareLaunchArgument(
        'angle_pid_ki',
        default_value='0.05',
        description='Angle PID controller integral gain'
    )
    
    angle_pid_kd_arg = DeclareLaunchArgument(
        'angle_pid_kd',
        default_value='0.12',
        description='Angle PID controller derivative gain'
    )
    
    # 声明距离分段参数
    very_close_distance_arg = DeclareLaunchArgument(
        'very_close_distance',
        default_value='0.5',
        description='Very close distance threshold (m)'
    )
    
    close_distance_arg = DeclareLaunchArgument(
        'close_distance',
        default_value='0.7',
        description='Close distance threshold (m)'
    )
    
    optimal_distance_near_arg = DeclareLaunchArgument(
        'optimal_distance_near',
        default_value='0.9',
        description='Optimal distance near threshold (m)'
    )
    
    optimal_distance_far_arg = DeclareLaunchArgument(
        'optimal_distance_far',
        default_value='1.3',
        description='Optimal distance far threshold (m)'
    )
    
    far_distance_arg = DeclareLaunchArgument(
        'far_distance',
        default_value='2.2',
        description='Far distance threshold (m)'
    )
    
    very_far_distance_arg = DeclareLaunchArgument(
        'very_far_distance',
        default_value='3.5',
        description='Very far distance threshold (m)'
    )
    
    # 声明速度分段参数
    very_close_speed_factor_arg = DeclareLaunchArgument(
        'very_close_speed_factor',
        default_value='-0.7',
        description='Speed factor for very close distance'
    )
    
    close_speed_factor_arg = DeclareLaunchArgument(
        'close_speed_factor',
        default_value='-0.4',
        description='Speed factor for close distance'
    )
    
    optimal_speed_factor_arg = DeclareLaunchArgument(
        'optimal_speed_factor',
        default_value='0.0',
        description='Speed factor for optimal distance'
    )
    
    far_speed_factor_arg = DeclareLaunchArgument(
        'far_speed_factor',
        default_value='0.5',
        description='Speed factor for far distance'
    )
    
    very_far_speed_factor_arg = DeclareLaunchArgument(
        'very_far_speed_factor',
        default_value='0.9',
        description='Speed factor for very far distance'
    )
    
    # 声明平滑控制参数
    speed_smoothing_factor_arg = DeclareLaunchArgument(
        'speed_smoothing_factor',
        default_value='0.75',
        description='Speed smoothing factor (0.0-1.0)'
    )
    
    max_acceleration_arg = DeclareLaunchArgument(
        'max_acceleration',
        default_value='1.2',
        description='Maximum acceleration (m/s²)'
    )
    
    min_speed_change_arg = DeclareLaunchArgument(
        'min_speed_change',
        default_value='0.015',
        description='Minimum speed change threshold'
    )
    
    # 声明转向方向修正参数
    angular_velocity_reverse_arg = DeclareLaunchArgument(
        'angular_velocity_reverse',
        default_value='true',
        description='Reverse angular velocity direction to fix turning direction'
    )
    
    # 获取launch配置
    camera_name = LaunchConfiguration('camera_name')
    use_astra_camera = LaunchConfiguration('use_astra_camera')
    use_rknn_yolo11 = LaunchConfiguration('use_rknn_yolo11')
    use_depth_service = LaunchConfiguration('use_depth_service')
    use_dlrobot_driver = LaunchConfiguration('use_dlrobot_driver')
    dlrobot_port = LaunchConfiguration('dlrobot_port')
    
    # 获取YOLO11参数配置
    yolo11_model_path = LaunchConfiguration('yolo11_model_path')
    yolo11_confidence_threshold = LaunchConfiguration('yolo11_confidence_threshold')
    yolo11_nms_threshold = LaunchConfiguration('yolo11_nms_threshold')
    yolo11_enable_debug_image = LaunchConfiguration('yolo11_enable_debug_image')
    
    # 获取控制参数配置
    max_linear_speed = LaunchConfiguration('max_linear_speed')
    max_angular_speed = LaunchConfiguration('max_angular_speed')
    min_follow_distance = LaunchConfiguration('min_follow_distance')
    max_follow_distance = LaunchConfiguration('max_follow_distance')
    follow_speed_factor = LaunchConfiguration('follow_speed_factor')
    
    # 获取动态速度调节参数配置
    enable_dynamic_speed = LaunchConfiguration('enable_dynamic_speed')
    enable_pid_control = LaunchConfiguration('enable_pid_control')
    enable_speed_smoothing = LaunchConfiguration('enable_speed_smoothing')
    distance_prediction_enabled = LaunchConfiguration('distance_prediction_enabled')
    
    # 获取PID参数配置
    distance_pid_kp = LaunchConfiguration('distance_pid_kp')
    distance_pid_ki = LaunchConfiguration('distance_pid_ki')
    distance_pid_kd = LaunchConfiguration('distance_pid_kd')
    angle_pid_kp = LaunchConfiguration('angle_pid_kp')
    angle_pid_ki = LaunchConfiguration('angle_pid_ki')
    angle_pid_kd = LaunchConfiguration('angle_pid_kd')
    
    # 获取距离分段参数配置
    very_close_distance = LaunchConfiguration('very_close_distance')
    close_distance = LaunchConfiguration('close_distance')
    optimal_distance_near = LaunchConfiguration('optimal_distance_near')
    optimal_distance_far = LaunchConfiguration('optimal_distance_far')
    far_distance = LaunchConfiguration('far_distance')
    very_far_distance = LaunchConfiguration('very_far_distance')
    
    # 获取速度分段参数配置
    very_close_speed_factor = LaunchConfiguration('very_close_speed_factor')
    close_speed_factor = LaunchConfiguration('close_speed_factor')
    optimal_speed_factor = LaunchConfiguration('optimal_speed_factor')
    far_speed_factor = LaunchConfiguration('far_speed_factor')
    very_far_speed_factor = LaunchConfiguration('very_far_speed_factor')
    
    # 获取平滑控制参数配置
    speed_smoothing_factor = LaunchConfiguration('speed_smoothing_factor')
    max_acceleration = LaunchConfiguration('max_acceleration')
    min_speed_change = LaunchConfiguration('min_speed_change')
    
    # 获取转向方向修正参数配置
    angular_velocity_reverse = LaunchConfiguration('angular_velocity_reverse')
    
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
        }],
        condition=IfCondition(use_astra_camera)
    )
    
    # RKNN YOLO11检测节点
    rknn_yolo11_node = Node(
        package='rknn_yolo11_ros2',
        executable='rknn_yolo11_ros2_node',
        name='rknn_yolo11_node',
        output='screen',
        parameters=[{
            'model_path': yolo11_model_path,
            'confidence_threshold': yolo11_confidence_threshold,
            'nms_threshold': yolo11_nms_threshold,
            'enable_debug_image': yolo11_enable_debug_image,
            'use_sim_time': False,
            # 性能优化参数
            'queue_size': 1,  # 减少队列延迟
            'use_intra_process_comms': True,  # 启用进程内通信优化
        }],
        remappings=[
            ('/camera/image_raw', [camera_name, '/color/image_raw']),
            ('/detections', '/detections'),
            ('/debug_image', '/debug_image'),
        ],
        condition=IfCondition(use_rknn_yolo11)
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
            {'detection_fps': 30.0},  # 提高检测频率
            {'distance_query_timeout': 2.0},
            {'use_sim_time': False},
            # 通信优化参数
            {'min_image_interval': 0.033},  # 30 FPS
            {'min_detection_interval': 0.025},  # 40 FPS
            {'enable_performance_monitor': True},
            {'message_pool_size': 10},
        ]
    )
    
    # 机器人控制节点（增强版）
    robot_control_node = Node(
        package='following_robot',
        executable='robot_control_node',
        name='robot_control_node',
        output='screen',
        parameters=[
            # 基础控制参数
            {'max_linear_speed': max_linear_speed},
            {'max_angular_speed': max_angular_speed},
            {'min_follow_distance': min_follow_distance},
            {'max_follow_distance': max_follow_distance},
            {'follow_speed_factor': follow_speed_factor},
            {'wheelbase': 0.143},
            {'use_ackermann': False},
            {'safety_enabled': True},
            
            # 动态速度调节功能开关
            {'enable_dynamic_speed': enable_dynamic_speed},
            {'enable_pid_control': enable_pid_control},
            {'enable_speed_smoothing': enable_speed_smoothing},
            {'distance_prediction_enabled': distance_prediction_enabled},
            
            # PID控制参数
            {'distance_pid_kp': distance_pid_kp},
            {'distance_pid_ki': distance_pid_ki},
            {'distance_pid_kd': distance_pid_kd},
            {'angle_pid_kp': angle_pid_kp},
            {'angle_pid_ki': angle_pid_ki},
            {'angle_pid_kd': angle_pid_kd},
            
            # 距离分段参数
            {'very_close_distance': very_close_distance},
            {'close_distance': close_distance},
            {'optimal_distance_near': optimal_distance_near},
            {'optimal_distance_far': optimal_distance_far},
            {'far_distance': far_distance},
            {'very_far_distance': very_far_distance},
            
            # 速度分段参数
            {'very_close_speed_factor': very_close_speed_factor},
            {'close_speed_factor': close_speed_factor},
            {'optimal_speed_factor': optimal_speed_factor},
            {'far_speed_factor': far_speed_factor},
            {'very_far_speed_factor': very_far_speed_factor},
            
            # 平滑控制参数
            {'speed_smoothing_factor': speed_smoothing_factor},
            {'max_acceleration': max_acceleration},
            {'min_speed_change': min_speed_change},
            
            # 转向方向修正参数
            {'angular_velocity_reverse': angular_velocity_reverse},
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
        # 基础参数
        camera_name_arg,
        use_astra_camera_arg,
        use_rknn_yolo11_arg,
        use_depth_service_arg,
        use_dlrobot_driver_arg,
        dlrobot_port_arg,
        
        # YOLO11检测节点参数
        yolo11_model_path_arg,
        yolo11_confidence_threshold_arg,
        yolo11_nms_threshold_arg,
        yolo11_enable_debug_image_arg,
        
        # 基础控制参数
        max_linear_speed_arg,
        max_angular_speed_arg,
        min_follow_distance_arg,
        max_follow_distance_arg,
        follow_speed_factor_arg,
        
        # 动态速度调节参数
        enable_dynamic_speed_arg,
        enable_pid_control_arg,
        enable_speed_smoothing_arg,
        distance_prediction_enabled_arg,
        
        # PID控制参数
        distance_pid_kp_arg,
        distance_pid_ki_arg,
        distance_pid_kd_arg,
        angle_pid_kp_arg,
        angle_pid_ki_arg,
        angle_pid_kd_arg,
        
        # 距离分段参数
        very_close_distance_arg,
        close_distance_arg,
        optimal_distance_near_arg,
        optimal_distance_far_arg,
        far_distance_arg,
        very_far_distance_arg,
        
        # 速度分段参数
        very_close_speed_factor_arg,
        close_speed_factor_arg,
        optimal_speed_factor_arg,
        far_speed_factor_arg,
        very_far_speed_factor_arg,
        
        # 平滑控制参数
        speed_smoothing_factor_arg,
        max_acceleration_arg,
        min_speed_change_arg,
        
        # 转向方向修正参数
        angular_velocity_reverse_arg,
        
        # 节点
        astra_camera_node,
        rknn_yolo11_node,
        depth_service_node,
        person_detection_node,
        robot_control_node,
        websocket_bridge_node,
        dlrobot_driver_node,
        rviz_node,
    ])