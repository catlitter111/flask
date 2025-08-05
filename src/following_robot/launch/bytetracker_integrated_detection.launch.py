#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition

def generate_launch_description():
    
    # 声明启动参数
    yolo11_model_path_arg = DeclareLaunchArgument(
        'yolo11_model_path',
        default_value=PathJoinSubstitution([
            FindPackageShare('rknn_yolo11_ros2'),
            'model',
            'cloth.rknn'
        ]),
        description='YOLO11服装检测模型文件路径'
    )
    
    camera_name_arg = DeclareLaunchArgument(
        'camera_name',
        default_value='camera',
        description='相机名称前缀'
    )
    
    input_topic_arg = DeclareLaunchArgument(
        'input_topic',
        default_value='/camera/color/image_raw',
        description='输入图像话题'
    )
    
    confidence_threshold_arg = DeclareLaunchArgument(
        'confidence_threshold',
        default_value='0.3',
        description='检测置信度阈值'
    )
    
    nms_threshold_arg = DeclareLaunchArgument(
        'nms_threshold',
        default_value='0.6',
        description='NMS阈值'
    )
    
    enable_debug_display_arg = DeclareLaunchArgument(
        'enable_debug_display',
        default_value='false',  # 关闭integrated_person_detection的显示，使用ByteTracker的显示
        description='是否启用integrated_person_detection的OpenCV调试显示'
    )
    
    use_depth_service_arg = DeclareLaunchArgument(
        'use_depth_service',
        default_value='true',
        description='是否启用深度服务节点'
    )
    
    # ByteTracker相关参数
    tracking_mode_arg = DeclareLaunchArgument(
        'tracking_mode',
        default_value='single',
        description='跟踪模式: multi或single'
    )
    
    track_thresh_arg = DeclareLaunchArgument(
        'track_thresh',
        default_value='0.5',
        description='跟踪置信度阈值'
    )
    
    color_weight_arg = DeclareLaunchArgument(
        'color_weight',
        default_value='0.5',
        description='颜色匹配权重'
    )
    
    target_features_file_arg = DeclareLaunchArgument(
        'target_features_file',
        default_value='/userdata/try_again/SelfFollowingROS2/src/following_robot/data/demo_person_features.xlsx',
        description='目标人物特征文件路径（用于单目标跟踪）'
    )
    
    enable_display_arg = DeclareLaunchArgument(
        'enable_display',
        default_value='true',
        description='是否启用ByteTracker的图像显示'
    )
    
    # 获取launch配置
    camera_name = LaunchConfiguration('camera_name')
    
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
        condition=IfCondition(LaunchConfiguration('use_depth_service'))
    )

    # 集成人员检测节点
    integrated_person_detection_node = Node(
        package='rknn_yolo11_ros2',
        executable='integrated_person_detection_node',
        name='integrated_person_detection_node',
        output='screen',
        parameters=[{
            'model_path': LaunchConfiguration('yolo11_model_path'),
            'input_topic': LaunchConfiguration('input_topic'),
            'person_topic': '/person_detection/person_positions',
            'distance_query_topic': '/depth_reader/get_depth_at',
            'distance_result_topic': '/depth_reader/depth_value',
            'debug_image_topic': '/integrated_person/debug_image',
            'confidence_threshold': LaunchConfiguration('confidence_threshold'),
            'nms_threshold': LaunchConfiguration('nms_threshold'),
            'enable_debug_display': LaunchConfiguration('enable_debug_display'),
        }],
        arguments=['--ros-args', '--log-level', 'info']
    )
    
    # ByteTracker节点
    bytetracker_node = Node(
        package='following_robot',
        executable='bytetracker_node',
        name='bytetracker_node',
        output='screen',
        parameters=[{
            'tracking_mode': LaunchConfiguration('tracking_mode'),
            'track_thresh': LaunchConfiguration('track_thresh'),
            'track_buffer': 100,
            'match_thresh': 0.8,
            'color_weight': LaunchConfiguration('color_weight'),
            'target_features_file': LaunchConfiguration('target_features_file'),
            'image_topic': LaunchConfiguration('input_topic'),
            'enable_display': LaunchConfiguration('enable_display'),
        }],
        arguments=['--ros-args', '--log-level', 'info']
    )
    
    return LaunchDescription([
        # 启动参数
        yolo11_model_path_arg,
        camera_name_arg,
        input_topic_arg,
        confidence_threshold_arg,
        nms_threshold_arg,
        enable_debug_display_arg,
        use_depth_service_arg,
        tracking_mode_arg,
        track_thresh_arg,
        color_weight_arg,
        target_features_file_arg,
        enable_display_arg,
        
        # 信息输出
        LogInfo(msg='启动整合人员检测和跟踪系统'),
        LogInfo(msg='=' * 60),
        LogInfo(msg='系统配置:'),
        LogInfo(msg=['  相机名称: ', LaunchConfiguration('camera_name')]),
        LogInfo(msg=['  输入话题: ', LaunchConfiguration('input_topic')]),
        LogInfo(msg=['  YOLO11模型: ', LaunchConfiguration('yolo11_model_path')]),
        LogInfo(msg=['  检测置信度阈值: ', LaunchConfiguration('confidence_threshold')]),
        LogInfo(msg=['  跟踪模式: ', LaunchConfiguration('tracking_mode')]),
        LogInfo(msg=['  跟踪置信度阈值: ', LaunchConfiguration('track_thresh')]),
        LogInfo(msg=['  颜色匹配权重: ', LaunchConfiguration('color_weight')]),
        LogInfo(msg=''),
        LogInfo(msg='功能说明:'),
        LogInfo(msg='  - Astra相机图像和深度数据采集'),
        LogInfo(msg='  - YOLO11服装检测（上衣、下装颜色）'),
        LogInfo(msg='  - 人员位置确定和距离测量'),
        LogInfo(msg='  - 多目标/单目标跟踪'),
        LogInfo(msg='  - 实时图像显示（红框=目标，绿框=其他人）'),
        LogInfo(msg=''),
        LogInfo(msg='控制说明:'),
        LogInfo(msg='  - 按 q 或 ESC 键退出'),
        LogInfo(msg='  - 发布 "/bytetracker/set_mode" 切换跟踪模式'),
        LogInfo(msg='  - 发布 "/bytetracker/set_target" 设置目标'),
        LogInfo(msg='=' * 60),
        
        # 节点启动顺序
        astra_camera_node,
        depth_service_node,
        integrated_person_detection_node,
        bytetracker_node,
    ]) 