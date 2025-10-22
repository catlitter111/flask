#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    """
    启动人体检测系统的完整launch文件
    包含相机节点、YOLO11检测节点、深度服务节点和人体检测节点
    增强功能：动态速度调节、PID控制、分段速度控制
    """
    
    # 获取包的路径
    astra_distance_pkg = get_package_share_directory('astra_distance')
    
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
    
    # 相机相关参数
    depth_registration_arg = DeclareLaunchArgument(
        'depth_registration',
        default_value='false',
        description='Hardware depth registration'
    )
    
    enable_depth_arg = DeclareLaunchArgument(
        'enable_depth',
        default_value='true',
        description='Enable depth stream'
    )
    
    enable_color_arg = DeclareLaunchArgument(
        'enable_color',
        default_value='true',
        description='Enable color stream'
    )
    
    depth_width_arg = DeclareLaunchArgument(
        'depth_width',
        default_value='640',
        description='Depth image width'
    )
    
    depth_height_arg = DeclareLaunchArgument(
        'depth_height',
        default_value='480',
        description='Depth image height'
    )
    
    color_width_arg = DeclareLaunchArgument(
        'color_width',
        default_value='640',
        description='Color image width'
    )
    
    color_height_arg = DeclareLaunchArgument(
        'color_height',
        default_value='480',
        description='Color image height'
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
    
    # RFID相关参数
    enable_rfid_arg = DeclareLaunchArgument(
        'enable_rfid',
        default_value='true',
        description='Enable RFID reader functionality'
    )
    
    rfid_reader_ip_arg = DeclareLaunchArgument(
        'rfid_reader_ip',
        default_value='192.168.0.178',
        description='RFID reader IP address'
    )
    
    rfid_auto_start_arg = DeclareLaunchArgument(
        'rfid_auto_start',
        default_value='true',
        description='Auto start RFID inventory on node startup'
    )
    
    # Video UDP节点相关参数
    use_video_udp_arg = DeclareLaunchArgument(
        'use_video_udp',
        default_value='true',
        description='Whether to launch video UDP streaming node'
    )
    
    tcp_host_arg = DeclareLaunchArgument(
        'tcp_host',
        default_value='192.168.137.1',
        description='TCP host for video streaming'
    )
    
    tcp_port_arg = DeclareLaunchArgument(
        'tcp_port',
        default_value='5005',
        description='TCP port for video streaming'
    )
    

    
    video_min_conf_arg = DeclareLaunchArgument(
        'video_min_conf',
        default_value='0.3',
        description='Minimum confidence threshold for bounding boxes'
    )
    
    video_min_size_arg = DeclareLaunchArgument(
        'video_min_size',
        default_value='10',
        description='Minimum size threshold for bounding boxes'
    )
    
    video_ignore_tid0_arg = DeclareLaunchArgument(
        'video_ignore_tid0',
        default_value='false',
        description='Ignore bounding boxes with track_id 0 or None'
    )
    
    video_drop_zero_box_arg = DeclareLaunchArgument(
        'video_drop_zero_box',
        default_value='true',
        description='Drop bounding boxes with all zero coordinates'
    )
    
    video_enable_display_arg = DeclareLaunchArgument(
        'video_enable_display',
        default_value='true',
        description='Enable video display window'
    )
    
    video_jpeg_quality_arg = DeclareLaunchArgument(
        'video_jpeg_quality',
        default_value='80',
        description='JPEG compression quality (1-100)'
    )
    
    video_max_packet_size_arg = DeclareLaunchArgument(
        'video_max_packet_size',
        default_value='60000',
        description='Maximum packet size for video transmission'
    )
    
    # 获取launch配置
    camera_name = LaunchConfiguration('camera_name')
    use_astra_camera = LaunchConfiguration('use_astra_camera')
    use_rknn_yolo11 = LaunchConfiguration('use_rknn_yolo11')
    use_depth_service = LaunchConfiguration('use_depth_service')
    use_dlrobot_driver = LaunchConfiguration('use_dlrobot_driver')
    dlrobot_port = LaunchConfiguration('dlrobot_port')
    
    # 获取相机参数配置
    depth_registration = LaunchConfiguration('depth_registration')
    enable_depth = LaunchConfiguration('enable_depth')
    enable_color = LaunchConfiguration('enable_color')
    depth_width = LaunchConfiguration('depth_width')
    depth_height = LaunchConfiguration('depth_height')
    color_width = LaunchConfiguration('color_width')
    color_height = LaunchConfiguration('color_height')
    
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
    
    # 获取RFID参数配置
    enable_rfid = LaunchConfiguration('enable_rfid')
    rfid_reader_ip = LaunchConfiguration('rfid_reader_ip')
    rfid_auto_start = LaunchConfiguration('rfid_auto_start')
    
    # 获取Video TCP参数配置
    use_video_udp = LaunchConfiguration('use_video_udp')
    tcp_host = LaunchConfiguration('tcp_host')
    tcp_port = LaunchConfiguration('tcp_port')
    video_min_conf = LaunchConfiguration('video_min_conf')
    video_min_size = LaunchConfiguration('video_min_size')
    video_ignore_tid0 = LaunchConfiguration('video_ignore_tid0')
    video_drop_zero_box = LaunchConfiguration('video_drop_zero_box')
    video_enable_display = LaunchConfiguration('video_enable_display')
    video_jpeg_quality = LaunchConfiguration('video_jpeg_quality')
    video_max_packet_size = LaunchConfiguration('video_max_packet_size')
    
    # 包含astra_distance_system.launch.py文件
    astra_distance_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(astra_distance_pkg, 'launch', 'astra_distance_system.launch.py')
        ]),
        launch_arguments={
            'camera_name': camera_name,
            'depth_registration': depth_registration,
            'enable_depth': enable_depth,
            'enable_color': enable_color,
            'depth_width': depth_width,
            'depth_height': depth_height,
            'color_width': color_width,
            'color_height': color_height,
        }.items(),
        condition=IfCondition(use_astra_camera)
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
    
    # Video UDP节点
    video_udp_node = Node(
        package='following_robot',
        executable='video_udp_node',
        name='video_udp_node',
        output='screen',
        parameters=[
            {'tcp_host': tcp_host},
            {'tcp_port': tcp_port},
            {'min_conf': video_min_conf},
            {'min_size': video_min_size},
            {'ignore_tid0': video_ignore_tid0},
            {'drop_zero_box': video_drop_zero_box},
            {'enable_display': video_enable_display},
            {'jpeg_quality': video_jpeg_quality},
            {'max_packet_size': video_max_packet_size},
        ],
        condition=IfCondition(use_video_udp)
    )
    
    
    
    return LaunchDescription([
        # 基础参数
        camera_name_arg,
        use_astra_camera_arg,
        use_rknn_yolo11_arg,
        use_depth_service_arg,
        use_dlrobot_driver_arg,
        dlrobot_port_arg,
        
        # 相机参数
        depth_registration_arg,
        enable_depth_arg,
        enable_color_arg,
        depth_width_arg,
        depth_height_arg,
        color_width_arg,
        color_height_arg,
        
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
        
        # RFID参数
        enable_rfid_arg,
        rfid_reader_ip_arg,
        rfid_auto_start_arg,
        
        # Video TCP参数
        use_video_udp_arg,
        tcp_host_arg,
        tcp_port_arg,
        video_min_conf_arg,
        video_min_size_arg,
        video_ignore_tid0_arg,
        video_drop_zero_box_arg,
        video_enable_display_arg,
        video_jpeg_quality_arg,
        video_max_packet_size_arg,
        
        # 节点和launch文件
        astra_distance_launch,
        robot_control_node,
        websocket_bridge_node,
        dlrobot_driver_node,
        video_udp_node,
       
    ])