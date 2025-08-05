#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition

def generate_launch_description():
    
    # 声明启动参数 - 基础跟踪参数
    tracking_mode_arg = DeclareLaunchArgument(
        'tracking_mode',
        default_value='single',
        description='跟踪模式: multi(多目标) 或 single(单目标)'
    )
    
    target_features_file_arg = DeclareLaunchArgument(
        'target_features_file',
        default_value='',
        description='单目标模式的目标特征文件路径'
    )
    
    # 声明启动参数 - 检测系统参数（来自integrated_person_detection.launch.py）
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
        default_value='true',
        description='是否启用OpenCV调试显示'
    )
    
    use_depth_service_arg = DeclareLaunchArgument(
        'use_depth_service',
        default_value='true',
        description='是否启用深度服务节点'
    )
    
    use_integrated_detection_arg = DeclareLaunchArgument(
        'use_integrated_detection',
        default_value='true',
        description='是否启用集成人员检测节点'
    )
    
    # ByteTracker参数
    track_thresh_arg = DeclareLaunchArgument(
        'track_thresh',
        default_value='0.5',
        description='跟踪置信度阈值'
    )
    
    track_buffer_arg = DeclareLaunchArgument(
        'track_buffer',
        default_value='100',
        description='跟踪缓冲区大小'
    )
    
    match_thresh_arg = DeclareLaunchArgument(
        'match_thresh',
        default_value='0.8',
        description='匹配阈值'
    )
    
    color_weight_arg = DeclareLaunchArgument(
        'color_weight',
        default_value='0.5',
        description='颜色特征权重'
    )
    
    # 集成人员检测节点
    integrated_detection_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('rknn_yolo11_ros2'),
                'launch',
                'integrated_person_detection.launch.py'
            ])
        ]),
        launch_arguments={
            'yolo11_model_path': LaunchConfiguration('yolo11_model_path'),
            'camera_name': LaunchConfiguration('camera_name'),
            'input_topic': LaunchConfiguration('input_topic'),
            'confidence_threshold': LaunchConfiguration('confidence_threshold'),
            'nms_threshold': LaunchConfiguration('nms_threshold'),
            'enable_debug_display': LaunchConfiguration('enable_debug_display'),
            'use_depth_service': LaunchConfiguration('use_depth_service'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('use_integrated_detection'))
    )
    
    # ByteTracker节点
    bytetracker_node = Node(
        package='following_robot',
        executable='bytetracker_node',
        name='bytetracker_node',
        output='screen',
        parameters=[{
            'tracking_mode': LaunchConfiguration('tracking_mode'),
            'target_features_file': LaunchConfiguration('target_features_file'),
            'track_thresh': LaunchConfiguration('track_thresh'),
            'track_buffer': LaunchConfiguration('track_buffer'),
            'match_thresh': LaunchConfiguration('match_thresh'),
            'color_weight': LaunchConfiguration('color_weight'),
            'person_positions_topic': '/person_detection/person_positions',
            'input_image_topic': LaunchConfiguration('input_topic'),
        }],
        arguments=['--ros-args', '--log-level', 'info']
    )
    
    return LaunchDescription([
        # 启动参数 - 跟踪系统
        tracking_mode_arg,
        target_features_file_arg,
        track_thresh_arg,
        track_buffer_arg,
        match_thresh_arg,
        color_weight_arg,
        
        # 启动参数 - 检测系统
        yolo11_model_path_arg,
        camera_name_arg,
        input_topic_arg,
        confidence_threshold_arg,
        nms_threshold_arg,
        enable_debug_display_arg,
        use_depth_service_arg,
        use_integrated_detection_arg,
        
        # 信息输出
        LogInfo(msg='🚀 启动ByteTracker与集成人员检测完整系统'),
        LogInfo(msg='========================================'),
        LogInfo(msg='📷 相机配置:'),
        LogInfo(msg=['  相机名称: ', LaunchConfiguration('camera_name')]),
        LogInfo(msg=['  输入话题: ', LaunchConfiguration('input_topic')]),
        LogInfo(msg='🔍 检测配置:'),
        LogInfo(msg=['  YOLO11模型: ', LaunchConfiguration('yolo11_model_path')]),
        LogInfo(msg=['  检测置信度阈值: ', LaunchConfiguration('confidence_threshold')]),
        LogInfo(msg=['  NMS阈值: ', LaunchConfiguration('nms_threshold')]),
        LogInfo(msg='🎯 跟踪配置:'),
        LogInfo(msg=['  跟踪模式: ', LaunchConfiguration('tracking_mode')]),
        LogInfo(msg=['  目标特征文件: ', LaunchConfiguration('target_features_file')]),
        LogInfo(msg=['  跟踪置信度阈值: ', LaunchConfiguration('track_thresh')]),
        LogInfo(msg=['  匹配阈值: ', LaunchConfiguration('match_thresh')]),
        LogInfo(msg=['  颜色特征权重: ', LaunchConfiguration('color_weight')]),
        LogInfo(msg='📡 系统组件:'),
        LogInfo(msg='  ✓ Astra相机节点'),
        LogInfo(msg='  ✓ 深度服务节点'),
        LogInfo(msg='  ✓ 集成人员检测节点'),
        LogInfo(msg='  ✓ ByteTracker跟踪节点'),
        LogInfo(msg='🔄 数据流:'),
        LogInfo(msg='  相机 → 集成检测 → /person_detection/person_positions → ByteTracker'),
        LogInfo(msg='📤 输出话题:'),
        LogInfo(msg='  - /bytetracker/tracked_persons (TrackedPersonArray)'),
        LogInfo(msg='  - /bytetracker/tracking_result (TrackingResult)'),
        LogInfo(msg='  - /bytetracker/detailed_tracking_data (String/JSON)'),
        LogInfo(msg='  - /bytetracker/status (String)'),
        LogInfo(msg='🎮 控制话题:'),
        LogInfo(msg='  - /bytetracker/set_mode (String): 切换跟踪模式 (multi/single)'),
        LogInfo(msg='  - /bytetracker/set_target (String): 设置目标人物'),
        LogInfo(msg='💡 调试信息:'),
        LogInfo(msg='  - 集成检测调试图像: /integrated_person/debug_image'),
        LogInfo(msg='  - 按q或ESC退出调试窗口'),
        LogInfo(msg='========================================'),
        
        # 节点启动顺序
        integrated_detection_launch,
        bytetracker_node,
    ]) 