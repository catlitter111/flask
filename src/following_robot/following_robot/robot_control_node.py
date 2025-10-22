#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
小车控制节点
============
整合turn_on_dlrobot_robot功能，提供小车运动控制接口
支持多种控制模式：跟随模式、手动控制模式、导航模式
增强功能：动态速度调节、PID控制、分段速度控制

🎯 运动控制设计：
- X轴速度：前进(+)/后退(-)线速度控制 (m/s)
- Z轴速度：左转(+)/右转(-)角速度控制 (rad/s)  
- Y轴速度：强制设为0.0 (差分驱动机器人不支持侧向运动)

适用于差分驱动机器人，如DLRobot系列底盘
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
import math
import time
import json
from enum import Enum
from collections import deque

# ROS消息类型
from geometry_msgs.msg import Twist, Point, PointStamped
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Float32, Bool
from turn_on_dlrobot_robot.msg import Position

# 自定义服务
from custom_msgs.srv import FeatureExtraction


class ControlMode(Enum):
    """控制模式枚举"""
    MANUAL = "manual"          # 手动控制
    FOLLOWING = "following"    # 人体跟随
    NAVIGATION = "navigation"  # 导航模式
    STOP = "stop"             # 停止模式
    INTERACTION = "interaction"  # 交互模式


class PIDController:
    """PID控制器"""
    
    def __init__(self, kp=1.0, ki=0.0, kd=0.0, output_limits=(-1.0, 1.0)):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limits = output_limits
        
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = time.time()
        
    def update(self, error, current_time=None):
        """更新PID控制器"""
        if current_time is None:
            current_time = time.time()
            
        dt = current_time - self.prev_time
        if dt <= 0.0:
            return 0.0
            
        # 比例项
        proportional = self.kp * error
        
        # 积分项
        self.integral += error * dt
        integral_term = self.ki * self.integral
        
        # 微分项
        derivative = (error - self.prev_error) / dt
        derivative_term = self.kd * derivative
        
        # 总输出
        output = proportional + integral_term + derivative_term
        
        # 输出限制
        if self.output_limits:
            output = max(self.output_limits[0], min(self.output_limits[1], output))
            
        # 积分饱和处理
        if abs(output) >= abs(self.output_limits[1]):
            self.integral -= error * dt  # 回退积分项
            
        self.prev_error = error
        self.prev_time = current_time
        
        return output
    
    def reset(self):
        """重置PID控制器"""
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = time.time()


class DynamicSpeedController:
    """动态速度控制器"""
    
    def __init__(self, max_linear_speed=0.5, max_angular_speed=1.0):
        self.max_linear_speed = max_linear_speed
        self.max_angular_speed = max_angular_speed
        
        # 距离分段参数
        self.distance_zones = {
            'very_close': 0.0,      # 非常接近
            'close': 0.8,           # 接近
            'optimal_near': 1.0,    # 最佳距离近端
            'optimal_far': 1.5,     # 最佳距离远端
            'far': 2.5,             # 远距离
            'very_far': 4.0         # 非常远
        }
        
        # 各距离段的速度参数
        self.speed_profiles = {
            'very_close': {'linear_factor': -0.6, 'angular_factor': 0.8},  # 后退
            'close': {'linear_factor': -0.3, 'angular_factor': 0.6},       # 缓慢后退
            'optimal': {'linear_factor': 0.0, 'angular_factor': 0.4},      # 停止/微调
            'far': {'linear_factor': 0.4, 'angular_factor': 0.5},          # 前进
            'very_far': {'linear_factor': 0.8, 'angular_factor': 0.6}      # 快速前进
        }
        
        # 速度平滑参数
        self.speed_smoothing_factor = 0.7  # 速度平滑系数
        self.min_speed_change = 0.02       # 最小速度变化阈值
        self.max_acceleration = 1.0        # 最大加速度 m/s²
        
        # 历史记录
        self.distance_history = deque(maxlen=10)
        self.speed_history = deque(maxlen=5)
        self.last_linear_speed = 0.0
        self.last_angular_speed = 0.0
        self.last_update_time = time.time()
        
    def update_parameters(self, **kwargs):
        """更新参数"""
        for key, value in kwargs.items():
            if hasattr(self, key):
                setattr(self, key, value)
                
    def get_distance_zone(self, distance):
        """获取距离区间"""
        if distance < self.distance_zones['very_close']:
            return 'very_close'
        elif distance < self.distance_zones['close']:
            return 'very_close'
        elif distance < self.distance_zones['optimal_near']:
            return 'close'
        elif distance < self.distance_zones['optimal_far']:
            return 'optimal'
        elif distance < self.distance_zones['far']:
            return 'far'
        else:
            return 'very_far'
    
    def calculate_target_speed(self, distance, angle_error):
        """计算目标速度"""
        # 获取距离区间
        zone = self.get_distance_zone(distance)
        
        # 获取速度配置
        if zone in self.speed_profiles:
            profile = self.speed_profiles[zone]
        else:
            profile = self.speed_profiles['optimal']
            
        # 计算基础线速度0
        if zone == 'optimal':
            # 在最佳距离范围内，根据距离微调
            optimal_center = (self.distance_zones['optimal_near'] + self.distance_zones['optimal_far']) / 2
            distance_error = distance - optimal_center
            linear_speed = profile['linear_factor'] * distance_error * 0.3
        else:
            # 其他区间的速度计算
            if zone == 'very_close' or zone == 'close':
                # 距离太近，后退速度与距离成反比
                distance_factor = max(0.3, 1.0 - distance / self.distance_zones['close'])
            elif zone == 'far':
                # 远距离，前进速度与距离成正比
                distance_factor = min(1.0, (distance - self.distance_zones['optimal_far']) / 
                                    (self.distance_zones['far'] - self.distance_zones['optimal_far']))
            else:  # very_far
                # 非常远距离，使用固定高速
                distance_factor = 1.0
                
            linear_speed = profile['linear_factor'] * distance_factor * self.max_linear_speed
        
        # 计算角速度
        angular_speed = -angle_error * profile['angular_factor'] * self.max_angular_speed
        
        # 速度限制
        linear_speed = max(-self.max_linear_speed, min(self.max_linear_speed, linear_speed))
        angular_speed = max(-self.max_angular_speed, min(self.max_angular_speed, angular_speed))
        
        return linear_speed, angular_speed, zone
    
    def smooth_speed_change(self, target_linear, target_angular):
        """平滑速度变化"""
        current_time = time.time()
        dt = current_time - self.last_update_time
        
        if dt <= 0:
            return self.last_linear_speed, self.last_angular_speed
            
        # 计算最大速度变化（基于加速度限制）
        max_linear_change = self.max_acceleration * dt
        max_angular_change = self.max_acceleration * dt * 2  # 角速度变化更快
        
        # 计算期望的速度变化
        linear_change = target_linear - self.last_linear_speed
        angular_change = target_angular - self.last_angular_speed
        
        # 应用加速度限制
        if abs(linear_change) > max_linear_change:
            linear_change = max_linear_change * (1 if linear_change > 0 else -1)
        if abs(angular_change) > max_angular_change:
            angular_change = max_angular_change * (1 if angular_change > 0 else -1)
            
        # 应用平滑系数
        smooth_linear = self.last_linear_speed + linear_change * self.speed_smoothing_factor
        smooth_angular = self.last_angular_speed + angular_change * self.speed_smoothing_factor
        
        # 最小变化阈值
        if abs(smooth_linear - self.last_linear_speed) < self.min_speed_change:
            smooth_linear = self.last_linear_speed
        if abs(smooth_angular - self.last_angular_speed) < self.min_speed_change:
            smooth_angular = self.last_angular_speed
            
        # 更新历史记录
        self.last_linear_speed = smooth_linear
        self.last_angular_speed = smooth_angular
        self.last_update_time = current_time
        
        return smooth_linear, smooth_angular
    
    def add_distance_sample(self, distance):
        """添加距离采样"""
        self.distance_history.append(distance)
        
    def get_distance_trend(self):
        """获取距离变化趋势"""
        if len(self.distance_history) < 3:
            return 0.0
            
        # 计算距离变化率
        recent_distances = list(self.distance_history)[-3:]
        trend = (recent_distances[-1] - recent_distances[0]) / len(recent_distances)
        return trend
    
    def reset(self):
        """重置控制器"""
        self.distance_history.clear()
        self.speed_history.clear()
        self.last_linear_speed = 0.0
        self.last_angular_speed = 0.0
        self.last_update_time = time.time()


class RobotControlNode(Node):
    """
    小车控制节点
    
    🎯 运动控制说明：
    - X轴速度：前进(+)/后退(-)线速度控制
    - Z轴速度：左转(+)/右转(-)角速度控制  
    - Y轴速度：强制设为0.0（差分驱动机器人不支持侧向运动）
    
    支持多种控制模式：跟随模式、手动控制模式、导航模式
    增强功能：动态速度调节、PID控制、分段速度控制
    """
    
    def __init__(self):
        super().__init__('robot_control_node')
        
        # 控制参数
        self.control_mode = ControlMode.STOP
        self.target_person_name = ""
        self.following_enabled = False
        self.safety_enabled = True
        
        # 运动参数
        self.max_linear_speed = 0.5   # 最大线速度 m/s
        self.max_angular_speed = 1.0  # 最大角速度 rad/s
        self.min_follow_distance = 1.0  # 最小跟随距离 m
        self.max_follow_distance = 3.0  # 最大跟随距离 m
        self.follow_speed_factor = 0.3  # 跟随速度因子
        
        # 新增：动态速度调节参数
        self.enable_dynamic_speed = True
        self.enable_pid_control = True
        self.enable_speed_smoothing = True
        self.distance_prediction_enabled = True
        
        # PID控制参数
        self.distance_pid_kp = 1.0
        self.distance_pid_ki = 0.1
        self.distance_pid_kd = 0.05
        self.angle_pid_kp = 2.0
        self.angle_pid_ki = 0.0
        self.angle_pid_kd = 0.1
        
        # 距离分段参数
        self.very_close_distance = 0.6
        self.close_distance = 0.8
        self.optimal_distance_near = 1.0
        self.optimal_distance_far = 1.5
        self.far_distance = 2.5
        self.very_far_distance = 4.0
        
        # 速度分段参数
        self.very_close_speed_factor = -0.6
        self.close_speed_factor = -0.3
        self.optimal_speed_factor = 0.0
        self.far_speed_factor = 0.4
        self.very_far_speed_factor = 0.8
        
        # 平滑控制参数
        self.speed_smoothing_factor = 0.7
        self.max_acceleration = 1.0
        self.min_speed_change = 0.02
        
        # 🔧 转向方向修正参数（解决小车转错方向问题）
        self.angular_velocity_reverse = False  # 是否反转角速度方向
        
        # 阿克曼参数
        self.wheelbase = 0.143  # 轴距（mini_akm）
        self.use_ackermann = False
        
        # 当前状态
        self.current_position = Point()
        self.current_orientation = 0.0
        self.last_person_position = None
        self.last_detection_time = 0.0
        self.detection_timeout = 2.0  # 检测超时时间
        
        # 安全状态
        self.obstacle_detected = False
        self.emergency_stop = False
        
        # 电机控制参数
        self.motor_speed = 50  # 电机速度百分比 (0-100)
        self.control_type = "motor"  # 控制类型：motor 或 companion
        
        # 初始化控制器
        self.setup_parameters()
        self.setup_controllers()
        self.setup_publishers()
        self.setup_subscribers()
        self.setup_services()
        self.setup_timers()
        
        self.get_logger().info("🚗 小车控制节点初始化完成")
        self.get_logger().info(f"🎮 当前控制模式: {self.control_mode.value}")
        self.get_logger().info(f"🎯 动态速度调节: {'启用' if self.enable_dynamic_speed else '禁用'}")
        self.get_logger().info(f"🎛️ PID控制: {'启用' if self.enable_pid_control else '禁用'}")
        self.get_logger().info("📐 运动控制配置: X轴(前进/后退) + Z轴(左转/右转), Y轴强制为0(差分驱动)")

    def setup_parameters(self):
        """设置参数"""
        # 声明基础参数
        self.declare_parameter('max_linear_speed', 0.5)
        self.declare_parameter('max_angular_speed', 1.0)
        self.declare_parameter('min_follow_distance', 1.0)
        self.declare_parameter('max_follow_distance', 3.0)
        self.declare_parameter('follow_speed_factor', 0.3)
        self.declare_parameter('wheelbase', 0.143)
        self.declare_parameter('use_ackermann', False)
        self.declare_parameter('safety_enabled', True)
        
        # 声明动态速度调节参数
        self.declare_parameter('enable_dynamic_speed', True)
        self.declare_parameter('enable_pid_control', True)
        self.declare_parameter('enable_speed_smoothing', True)
        self.declare_parameter('distance_prediction_enabled', True)
        
        # 声明PID参数
        self.declare_parameter('distance_pid_kp', 1.0)
        self.declare_parameter('distance_pid_ki', 0.1)
        self.declare_parameter('distance_pid_kd', 0.05)
        self.declare_parameter('angle_pid_kp', 2.0)
        self.declare_parameter('angle_pid_ki', 0.0)
        self.declare_parameter('angle_pid_kd', 0.1)
        
        # 声明距离分段参数
        self.declare_parameter('very_close_distance', 0.6)
        self.declare_parameter('close_distance', 0.8)
        self.declare_parameter('optimal_distance_near', 1.0)
        self.declare_parameter('optimal_distance_far', 1.5)
        self.declare_parameter('far_distance', 2.5)
        self.declare_parameter('very_far_distance', 4.0)
        
        # 声明速度分段参数
        self.declare_parameter('very_close_speed_factor', -0.6)
        self.declare_parameter('close_speed_factor', -0.3)
        self.declare_parameter('optimal_speed_factor', 0.0)
        self.declare_parameter('far_speed_factor', 0.4)
        self.declare_parameter('very_far_speed_factor', 0.8)
        
        # 声明平滑控制参数
        self.declare_parameter('speed_smoothing_factor', 0.7)
        self.declare_parameter('max_acceleration', 1.0)
        self.declare_parameter('min_speed_change', 0.02)
        
        # 声明转向方向修正参数
        self.declare_parameter('angular_velocity_reverse', False)
        
        # 获取参数值
        self.max_linear_speed = float(self.get_parameter('max_linear_speed').value or 0.5)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value or 1.0)
        self.min_follow_distance = float(self.get_parameter('min_follow_distance').value or 1.0)
        self.max_follow_distance = float(self.get_parameter('max_follow_distance').value or 3.0)
        self.follow_speed_factor = float(self.get_parameter('follow_speed_factor').value or 0.3)
        self.wheelbase = float(self.get_parameter('wheelbase').value or 0.143)
        self.use_ackermann = bool(self.get_parameter('use_ackermann').value or False)
        self.safety_enabled = bool(self.get_parameter('safety_enabled').value or True)
        
        # 获取动态速度调节参数
        self.enable_dynamic_speed = bool(self.get_parameter('enable_dynamic_speed').value or True)
        self.enable_pid_control = bool(self.get_parameter('enable_pid_control').value or True)
        self.enable_speed_smoothing = bool(self.get_parameter('enable_speed_smoothing').value or True)
        self.distance_prediction_enabled = bool(self.get_parameter('distance_prediction_enabled').value or True)
        
        # 获取PID参数
        self.distance_pid_kp = float(self.get_parameter('distance_pid_kp').value or 1.0)
        self.distance_pid_ki = float(self.get_parameter('distance_pid_ki').value or 0.1)
        self.distance_pid_kd = float(self.get_parameter('distance_pid_kd').value or 0.05)
        self.angle_pid_kp = float(self.get_parameter('angle_pid_kp').value or 2.0)
        self.angle_pid_ki = float(self.get_parameter('angle_pid_ki').value or 0.0)
        self.angle_pid_kd = float(self.get_parameter('angle_pid_kd').value or 0.1)
        
        # 获取距离分段参数
        self.very_close_distance = float(self.get_parameter('very_close_distance').value or 0.6)
        self.close_distance = float(self.get_parameter('close_distance').value or 0.8)
        self.optimal_distance_near = float(self.get_parameter('optimal_distance_near').value or 1.0)
        self.optimal_distance_far = float(self.get_parameter('optimal_distance_far').value or 1.5)
        self.far_distance = float(self.get_parameter('far_distance').value or 2.5)
        self.very_far_distance = float(self.get_parameter('very_far_distance').value or 4.0)
        
        # 获取速度分段参数
        self.very_close_speed_factor = float(self.get_parameter('very_close_speed_factor').value or -0.6)
        self.close_speed_factor = float(self.get_parameter('close_speed_factor').value or -0.3)
        self.optimal_speed_factor = float(self.get_parameter('optimal_speed_factor').value or 0.0)
        self.far_speed_factor = float(self.get_parameter('far_speed_factor').value or 0.4)
        self.very_far_speed_factor = float(self.get_parameter('very_far_speed_factor').value or 0.8)
        
        # 获取平滑控制参数
        self.speed_smoothing_factor = float(self.get_parameter('speed_smoothing_factor').value or 0.7)
        self.max_acceleration = float(self.get_parameter('max_acceleration').value or 1.0)
        self.min_speed_change = float(self.get_parameter('min_speed_change').value or 0.02)
        
        # 获取转向方向修正参数
        self.angular_velocity_reverse = bool(self.get_parameter('angular_velocity_reverse').value or False)

    def setup_controllers(self):
        """设置控制器"""
        # 创建动态速度控制器
        self.dynamic_speed_controller = DynamicSpeedController(
            self.max_linear_speed, 
            self.max_angular_speed
        )
        
        # 更新动态速度控制器参数
        self.dynamic_speed_controller.update_parameters(
            distance_zones={
                'very_close': self.very_close_distance,
                'close': self.close_distance,
                'optimal_near': self.optimal_distance_near,
                'optimal_far': self.optimal_distance_far,
                'far': self.far_distance,
                'very_far': self.very_far_distance
            },
            speed_profiles={
                'very_close': {'linear_factor': self.very_close_speed_factor, 'angular_factor': 0.8},
                'close': {'linear_factor': self.close_speed_factor, 'angular_factor': 0.6},
                'optimal': {'linear_factor': self.optimal_speed_factor, 'angular_factor': 0.4},
                'far': {'linear_factor': self.far_speed_factor, 'angular_factor': 0.5},
                'very_far': {'linear_factor': self.very_far_speed_factor, 'angular_factor': 0.6}
            },
            speed_smoothing_factor=self.speed_smoothing_factor,
            max_acceleration=self.max_acceleration,
            min_speed_change=self.min_speed_change
        )
        
        # 创建PID控制器
        if self.enable_pid_control:
            self.distance_pid = PIDController(
                self.distance_pid_kp, 
                self.distance_pid_ki, 
                self.distance_pid_kd,
                (-self.max_linear_speed, self.max_linear_speed)
            )
            
            self.angle_pid = PIDController(
                self.angle_pid_kp, 
                self.angle_pid_ki, 
                self.angle_pid_kd,
                (-self.max_angular_speed, self.max_angular_speed)
            )
        else:
            self.distance_pid = None
            self.angle_pid = None

    def setup_publishers(self):
        """设置发布者"""
        qos = QoSProfile(depth=10)
        
        # 控制命令发布者
        if self.use_ackermann:
            self.cmd_publisher = self.create_publisher(
                AckermannDriveStamped, 'ackermann_cmd', qos)
        else:
            self.cmd_publisher = self.create_publisher(
                Twist, 'cmd_vel', qos)
        
        # 状态发布者
        self.status_publisher = self.create_publisher(
            String, '/robot_control/status', qos)
        self.mode_publisher = self.create_publisher(
            String, '/robot_control/mode', qos)
        
        # 新增：控制指令信息发布者（用于WebSocket转发）
        self.control_command_publisher = self.create_publisher(
            String, '/robot_control/current_command', qos)
        
        # 新增：动态速度控制状态发布者
        self.speed_control_status_publisher = self.create_publisher(
            String, '/robot_control/speed_control_status', qos)

    def setup_subscribers(self):
        """设置订阅者"""
        qos = QoSProfile(depth=10)
        
        # 来自WebSocket桥接节点的控制命令订阅（重要！）
        self.command_subscription = self.create_subscription(
            String,
            '/robot_control/command',
            self.command_callback,
            qos
        )
        
        # 手动控制订阅
        self.manual_cmd_sub = self.create_subscription(
            Twist, '/robot_control/manual_cmd', 
            self.manual_cmd_callback, qos)
        
        # 人体位置订阅（兼容多种格式）
        self.person_position_sub = self.create_subscription(
            Position, '/robot_control/person_position',
            self.person_position_callback, qos)
            
        # 人体检测JSON数据订阅（来自person_detection_distance_node）
        self.person_positions_json_sub = self.create_subscription(
            String, '/person_detection/person_positions',
            self.person_positions_json_callback, qos)
        
        # 控制模式订阅
        self.mode_cmd_sub = self.create_subscription(
            String, '/robot_control/set_mode',
            self.mode_cmd_callback, qos)
        
        # 目标人物订阅
        self.target_person_sub = self.create_subscription(
            String, '/robot_control/target_person',
            self.target_person_callback, qos)
        
        # 里程计订阅
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, qos)
        
        # 安全相关订阅
        self.emergency_stop_sub = self.create_subscription(
            Bool, '/robot_control/emergency_stop',
            self.emergency_stop_callback, qos)
        
        # 带距离信息的点订阅（用于目标跟踪控制）
        self.point_with_distance_sub = self.create_subscription(
            PointStamped, '/point_with_distance',
            self.point_with_distance_callback, qos)

    def setup_services(self):
        """设置服务客户端"""
        # 特征提取服务客户端
        self.feature_extraction_client = self.create_client(
            FeatureExtraction, '/features/extract_features')

    def setup_timers(self):
        """设置定时器"""
        # 控制循环定时器（20Hz）
        self.control_timer = self.create_timer(0.05, self.control_loop)
        
        # 状态发布定时器（2Hz）
        self.status_timer = self.create_timer(0.5, self.publish_status)
        
        # 安全检查定时器（10Hz）
        self.safety_timer = self.create_timer(0.1, self.safety_check)

    def command_callback(self, msg):
        """处理来自WebSocket桥接节点的命令"""
        command = msg.data
        self.get_logger().info(f"🎮🔥 [小车控制节点] 收到WebSocket命令: {command}")
        
        try:
            # 解析命令
            if ':' in command:
                cmd_type, cmd_value = command.split(':', 1)
            else:
                cmd_type = command
                cmd_value = ""
            
            # 处理模式切换命令
            if cmd_type == 'start_auto_mode':
                self.set_control_mode(ControlMode.FOLLOWING)
                self.get_logger().info("🤖 启动自动跟随模式")
                
            elif cmd_type == 'pause_auto_mode':
                self.set_control_mode(ControlMode.STOP)
                self.get_logger().info("⏸️ 暂停自动模式")
                
            elif cmd_type == 'start_interaction':
                self.set_control_mode(ControlMode.INTERACTION)
                self.control_type = "companion"
                self.get_logger().info("🤝 启动交互模式")
                
            elif cmd_type == 'stop_interaction':
                self.set_control_mode(ControlMode.MANUAL)
                self.control_type = "motor"
                self.get_logger().info("🛑 停止交互模式")
                
            # 处理电机控制命令
            elif cmd_type.startswith('motor_'):
                self.handle_motor_command(cmd_type, cmd_value)
                
            # 处理伴侣交互命令
            elif cmd_type.startswith('companion_'):
                self.handle_companion_command(cmd_type, cmd_value)
                
            # 处理设置命令
            elif cmd_type == 'set_motor_speed':
                try:
                    self.motor_speed = max(0, min(100, int(cmd_value)))
                    self.get_logger().info(f"⚡ 设置电机速度: {self.motor_speed}%")
                except ValueError:
                    self.get_logger().warn(f"无效的电机速度值: {cmd_value}")
                    
            elif cmd_type == 'switch_control_type':
                if cmd_value in ['motor', 'companion']:
                    self.control_type = cmd_value
                    self.get_logger().info(f"🔄 切换控制类型: {self.control_type}")
                else:
                    self.get_logger().warn(f"无效的控制类型: {cmd_value}")
                    
            elif cmd_type == 'emergency_stop':
                self.emergency_stop = True
                self.send_stop_command()
                self.get_logger().warn("🚨 紧急停止激活")
                
            elif cmd_type == 'switch_to_manual':
                # 切换到手动模式，停止跟踪
                self.set_control_mode(ControlMode.MANUAL)
                self.send_stop_command()
                self.get_logger().info('🎯 已切换到手动模式，停止跟踪')
                
            elif cmd_type == 'switch_to_auto':
                # 切换到自动模式，开启跟踪
                self.set_control_mode(ControlMode.FOLLOWING)
                self.get_logger().info('🤖 已切换到自动模式，开启跟踪')
                
            else:
                self.get_logger().warn(f"⚠️ 未知命令: {command}")
                
        except Exception as e:
            self.get_logger().error(f"❌ 命令处理失败: {e}")

    def handle_motor_command(self, cmd_type, cmd_value):
        """
        处理电机控制命令
        
        🎯 电机控制说明：
        - 前进/后退：使用X轴线速度控制
        - 左转/右转：使用Z轴角速度控制
        - 侧向运动：不支持（Y轴速度强制为0）
        """
        try:
            # 解析速度参数
            if cmd_value:
                speed = max(0, min(100, int(cmd_value)))
            else:
                speed = self.motor_speed
            
            # 将百分比转换为实际速度值
            linear_vel = (speed / 100.0) * self.max_linear_speed
            angular_vel = (speed / 100.0) * self.max_angular_speed
            
            # 🎯 根据命令类型设置运动参数（只使用X轴和Z轴）
            if cmd_type == 'motor_forward':
                # X轴正向线速度，Z轴角速度为0
                self.send_velocity_command(linear_vel, 0.0)
                self.get_logger().info(f"🔺 电机前进: {speed}% (X轴={linear_vel:.3f})")
                
            elif cmd_type == 'motor_backward':
                # X轴负向线速度，Z轴角速度为0
                self.send_velocity_command(-linear_vel, 0.0)
                self.get_logger().info(f"🔻 电机后退: {speed}% (X轴={-linear_vel:.3f})")
                
            elif cmd_type == 'motor_left':
                # X轴线速度为0，Z轴正向角速度（左转）
                self.send_velocity_command(0.0, angular_vel)
                self.get_logger().info(f"◀️ 电机左转: {speed}% (Z轴={angular_vel:.3f})")
                
            elif cmd_type == 'motor_right':
                # X轴线速度为0，Z轴负向角速度（右转）
                self.send_velocity_command(0.0, -angular_vel)
                self.get_logger().info(f"▶️ 电机右转: {speed}% (Z轴={-angular_vel:.3f})")
                
            elif cmd_type == 'motor_stop':
                self.send_stop_command()
                self.get_logger().info("🛑 电机停止")
                
            # 切换到手动控制模式
            if self.control_mode != ControlMode.MANUAL:
                self.set_control_mode(ControlMode.MANUAL)
                
        except Exception as e:
            self.get_logger().error(f"❌ 电机命令处理失败: {e}")

    def handle_companion_command(self, cmd_type, cmd_value):
        """处理伴侣交互命令"""
        try:
            self.get_logger().info(f"🤝 伴侣交互命令: {cmd_type}")
            
            # 这里可以根据具体的伴侣机器人功能来实现
            # 目前先发送基本的运动命令作为演示
            if cmd_type == 'companion_look_up':
                # 可以控制头部或相机向上
                self.get_logger().info("👀 伴侣向上看")
                
            elif cmd_type == 'companion_look_down':
                # 可以控制头部或相机向下
                self.get_logger().info("👀 伴侣向下看")
                
            elif cmd_type == 'companion_turn_left':
                self.send_velocity_command(0.0, 0.3)
                self.get_logger().info("↰ 伴侣左转")
                
            elif cmd_type == 'companion_turn_right':
                self.send_velocity_command(0.0, -0.3)
                self.get_logger().info("↱ 伴侣右转")
                
            elif cmd_type == 'companion_stop':
                self.send_stop_command()
                self.get_logger().info("🛑 伴侣停止")
                
            # 确保处于交互模式
            if self.control_mode != ControlMode.INTERACTION:
                self.set_control_mode(ControlMode.INTERACTION)
                
        except Exception as e:
            self.get_logger().error(f"❌ 伴侣命令处理失败: {e}")

    def manual_cmd_callback(self, msg):
        """手动控制命令回调"""
        if self.control_mode == ControlMode.MANUAL:
            # 🎯 只使用X轴（前进/后退）和Z轴（旋转）速度，忽略Y轴（侧向）速度
            # X轴：前进/后退线速度，Z轴：旋转角速度，Y轴：强制设为0（差分驱动不支持侧向运动）
            linear_x = msg.linear.x  # 前进/后退速度
            angular_z = msg.angular.z  # 旋转角速度
            # 明确忽略 msg.linear.y（侧向速度）
            
            self.send_velocity_command(linear_x, angular_z)
            self.get_logger().debug(f"手动控制: X轴(前进)={linear_x:.2f}, Z轴(旋转)={angular_z:.2f}, Y轴(侧向)=0.0(忽略)")

    def person_position_callback(self, msg):
        """人体位置回调"""
        self.last_person_position = msg
        self.last_detection_time = time.time()
        
        if self.control_mode == ControlMode.FOLLOWING:
            self.process_following_control(msg)
    
    def person_positions_json_callback(self, msg):
        """处理JSON格式的人体位置数据（来自person_detection_distance_node）"""
        try:
            import json
            data = json.loads(msg.data)
            persons = data.get('persons', [])
            
            if not persons:
                return
            
            # 找到距离最近的有效人体
            closest_person = None
            min_distance = float('inf')
            
            for person in persons:
                if person.get('valid_distance', False) and person.get('distance') is not None:
                    distance = person['distance']
                    if distance < min_distance:
                        min_distance = distance
                        closest_person = person
            
            if closest_person:
                # 更新检测时间
                self.last_detection_time = time.time()
                
                # 处理跟随控制（仅在跟随模式下）
                if self.control_mode == ControlMode.FOLLOWING:
                    self.process_following_control_json(closest_person)
                    
        except Exception as e:
            self.get_logger().error(f"❌ 处理JSON人体位置数据失败: {e}")
    
    def process_following_control_json(self, person_data):
        """处理基于JSON数据的跟随控制逻辑（增强版）"""
        try:
            distance = person_data.get('distance', 0.0)
            center = person_data.get('center', [0, 0])  # [x, y]像素坐标
            
            # 假设图像宽度640像素，计算水平偏移角度
            image_width = 640
            image_center_x = image_width / 2
            horizontal_offset_pixels = center[0] - image_center_x
            
            # 转换为角度（假设相机水平视场角60度）
            camera_fov = 60.0  # 度
            angle_x = horizontal_offset_pixels * (camera_fov / image_width) * (3.14159 / 180)  # 转换为弧度
            
            # 使用增强的动态速度控制
            if self.enable_dynamic_speed:
                linear_vel, angular_vel = self.calculate_dynamic_speed(distance, angle_x)
            else:
                # 使用原始简单控制逻辑
                linear_vel, angular_vel = self.calculate_simple_speed(distance, angle_x)
            
            # 发送控制命令
            self.send_velocity_command(linear_vel, angular_vel)
            
            # 发布速度控制状态
            self.publish_speed_control_status(distance, angle_x, linear_vel, angular_vel)
            
            self.get_logger().debug(
                f"JSON跟随控制: dist={distance:.2f}, angle={angle_x:.2f}, "
                f"linear={linear_vel:.2f}, angular={angular_vel:.2f}"
            )
            
        except Exception as e:
            self.get_logger().error(f"JSON跟随控制处理错误: {e}")
    
    def calculate_dynamic_speed(self, distance, angle_x):
        """计算动态速度（增强版）"""
        # 添加距离采样到历史记录
        self.dynamic_speed_controller.add_distance_sample(distance)
        
        # 计算目标距离（最佳跟随距离的中点）
        target_distance = (self.optimal_distance_near + self.optimal_distance_far) / 2
        
        if self.enable_pid_control and self.distance_pid and self.angle_pid:
            # 使用PID控制
            distance_error = distance - target_distance
            linear_vel = self.distance_pid.update(distance_error)
            angular_vel = self.angle_pid.update(angle_x)
        else:
            # 使用动态速度控制器
            linear_vel, angular_vel, zone = self.dynamic_speed_controller.calculate_target_speed(
                distance, angle_x
            )
        
        # 应用速度平滑
        if self.enable_speed_smoothing:
            linear_vel, angular_vel = self.dynamic_speed_controller.smooth_speed_change(
                linear_vel, angular_vel
            )
        
        # 距离预测调整
        if self.distance_prediction_enabled:
            distance_trend = self.dynamic_speed_controller.get_distance_trend()
            # 根据距离变化趋势调整速度
            if distance_trend > 0.1:  # 距离快速增大
                linear_vel *= 1.2  # 稍微加速
            elif distance_trend < -0.1:  # 距离快速减小
                linear_vel *= 0.8  # 稍微减速
        
        return linear_vel, angular_vel
    
    def calculate_simple_speed(self, distance, angle_x):
        """计算简单速度（原始逻辑）"""
        # 距离控制
        linear_vel = 0.0
        if distance > self.max_follow_distance:
            # 距离太远，加速跟上
            linear_vel = self.follow_speed_factor * min(
                (distance - self.max_follow_distance) / 2.0,
                self.max_linear_speed
            )
        elif distance < self.min_follow_distance:
            # 距离太近，后退
            linear_vel = -self.follow_speed_factor * min(
                (self.min_follow_distance - distance) / 1.0,
                self.max_linear_speed * 0.5
            )
        
        # 角度控制
        angular_vel = 0.0
        if abs(angle_x) > 0.1:  # 角度阈值（弧度）
            angular_vel = -angle_x * 0.5  # 角度控制增益
            angular_vel = max(-self.max_angular_speed, 
                            min(self.max_angular_speed, angular_vel))
        
        return linear_vel, angular_vel
    
    def publish_speed_control_status(self, distance, angle_x, linear_vel, angular_vel):
        """发布速度控制状态"""
        try:
            zone = self.dynamic_speed_controller.get_distance_zone(distance)
            distance_trend = self.dynamic_speed_controller.get_distance_trend()
            
            status_info = {
                "timestamp": int(time.time() * 1000),
                "distance": round(distance, 3),
                "angle_x": round(angle_x, 3),
                "linear_vel": round(linear_vel, 3),
                "angular_vel": round(angular_vel, 3),
                "distance_zone": zone,
                "distance_trend": round(distance_trend, 3),
                "dynamic_speed_enabled": self.enable_dynamic_speed,
                "pid_enabled": self.enable_pid_control,
                "smoothing_enabled": self.enable_speed_smoothing,
                "prediction_enabled": self.distance_prediction_enabled
            }
            
            status_msg = String()
            status_msg.data = json.dumps(status_info)
            self.speed_control_status_publisher.publish(status_msg)
            
        except Exception as e:
            self.get_logger().error(f"发布速度控制状态失败: {e}")

    def mode_cmd_callback(self, msg):
        """控制模式切换回调"""
        try:
            new_mode = ControlMode(msg.data.lower())
            self.set_control_mode(new_mode)
        except ValueError:
            self.get_logger().warn(f"无效的控制模式: {msg.data}")

    def target_person_callback(self, msg):
        """目标人物设置回调"""
        self.target_person_name = msg.data
        self.get_logger().info(f"设置目标人物: {self.target_person_name}")

    def odom_callback(self, msg):
        """里程计回调"""
        self.current_position = msg.pose.pose.position
        # 从四元数转换为欧拉角
        orientation_q = msg.pose.pose.orientation
        self.current_orientation = self.quaternion_to_yaw(orientation_q)

    def emergency_stop_callback(self, msg):
        """紧急停止回调"""
        self.emergency_stop = msg.data
        if self.emergency_stop:
            self.send_stop_command()
            # 重置控制器
            if self.dynamic_speed_controller:
                self.dynamic_speed_controller.reset()
            if self.distance_pid:
                self.distance_pid.reset()
            if self.angle_pid:
                self.angle_pid.reset()
            self.get_logger().warn("🚨 紧急停止激活")

    def point_with_distance_callback(self, msg):
        """处理带距离信息的点数据回调"""
        try:
            # 提取数据：x, y为像素坐标，z为距离（米）
            pixel_x = msg.point.x
            pixel_y = msg.point.y
            distance_m = msg.point.z
            
            # 更新检测时间
            self.last_detection_time = time.time()
            
            # 仅在跟随模式下处理控制逻辑
            if self.control_mode != ControlMode.FOLLOWING:
                return
            
            # 安全检查
            if self.emergency_stop or not self.following_enabled:
                return
                
            # 距离有效性检查
            if distance_m <= 0 or distance_m > 10.0:  # 距离范围检查
                self.get_logger().debug(f"⚠️ 距离数据无效: {distance_m:.2f}m")
                return
            
            # 计算角度误差（基于像素坐标）
            # 假设图像中心为目标，计算偏移角度
            image_center_x = 320.0  # 假设图像宽度为640，中心为320
            pixel_offset = pixel_x - image_center_x
            
            # 将像素偏移转换为角度误差（简化计算）
            # 假设相机水平视角约60度，图像宽度640像素
            fov_horizontal = math.radians(60)  # 60度转弧度
            image_width = 640.0
            angle_error = (pixel_offset / image_width) * fov_horizontal
            
            # 处理跟随控制
            self.process_point_following_control(distance_m, angle_error, pixel_x, pixel_y)
            
            self.get_logger().debug(
                f"🎯 目标点控制: 像素({pixel_x:.1f}, {pixel_y:.1f}), "
                f"距离={distance_m:.2f}m, 角度误差={math.degrees(angle_error):.1f}°"
            )
            
        except Exception as e:
            self.get_logger().error(f"❌ 处理点距离数据失败: {e}")

    def process_point_following_control(self, distance, angle_error, pixel_x, pixel_y):
        """处理基于点和距离的跟随控制逻辑"""
        try:
            # 使用现有的动态速度控制逻辑
            if self.enable_dynamic_speed:
                linear_vel, angular_vel = self.calculate_dynamic_speed(distance, angle_error)
            else:
                linear_vel, angular_vel = self.calculate_simple_speed(distance, angle_error)
            
            # 应用角速度方向修正
            angular_vel = self.apply_angular_velocity_correction(angular_vel)
            
            # 发送速度命令
            self.send_velocity_command(linear_vel, angular_vel)
            
            # 发布控制状态
            self.publish_speed_control_status(distance, angle_error, linear_vel, angular_vel)
            
            self.get_logger().debug(
                f"🚗 点跟随控制: 距离={distance:.2f}m, 角度={math.degrees(angle_error):.1f}°, "
                f"线速度={linear_vel:.2f}m/s, 角速度={math.degrees(angular_vel):.1f}°/s"
            )
            
        except Exception as e:
            self.get_logger().error(f"❌ 点跟随控制处理失败: {e}")

    def set_control_mode(self, mode):
        """设置控制模式"""
        if mode != self.control_mode:
            self.get_logger().info(f"🎮 控制模式切换: {self.control_mode.value} -> {mode.value}")
            
            # 切换模式时先停止
            if mode == ControlMode.STOP:
                self.send_stop_command()
            
            # 重置控制器
            if self.dynamic_speed_controller:
                self.dynamic_speed_controller.reset()
            if self.distance_pid:
                self.distance_pid.reset()
            if self.angle_pid:
                self.angle_pid.reset()
            
            self.control_mode = mode
            
            # 发布模式变更
            mode_msg = String()
            mode_msg.data = mode.value
            self.mode_publisher.publish(mode_msg)

    def process_following_control(self, person_pos):
        """处理跟随控制逻辑"""
        try:
            # 计算距离和角度
            distance = float(person_pos.distance) if person_pos.distance is not None else 0.0
            angle_x = float(person_pos.angle_x) if person_pos.angle_x is not None else 0.0
            angle_y = float(person_pos.angle_y) if person_pos.angle_y is not None else 0.0
            
            # 使用增强的动态速度控制
            if self.enable_dynamic_speed:
                linear_vel, angular_vel = self.calculate_dynamic_speed(distance, angle_x)
            else:
                # 使用原始简单控制逻辑
                linear_vel, angular_vel = self.calculate_simple_speed(distance, angle_x)
            
            # 发送控制命令
            self.send_velocity_command(linear_vel, angular_vel)
            
            # 发布速度控制状态
            self.publish_speed_control_status(distance, angle_x, linear_vel, angular_vel)
            
            self.get_logger().debug(
                f"跟随控制: dist={distance:.2f}, angle={angle_x:.2f}, "
                f"linear={linear_vel:.2f}, angular={angular_vel:.2f}"
            )
            
        except Exception as e:
            self.get_logger().error(f"跟随控制处理错误: {e}")

    def apply_angular_velocity_correction(self, angular_z):
        """
        应用角速度方向修正
        
        🔧 解决小车转向方向错误的问题：
        - 如果angular_velocity_reverse为True，则反转角速度方向
        - 适用于不同机器人硬件的差异
        """
        if self.angular_velocity_reverse:
            return -angular_z
        return angular_z

    def send_velocity_command(self, linear_x, angular_z):
        """
        发送速度控制命令
        
        参数说明：
        - linear_x: X轴线速度 (m/s) - 前进(+)/后退(-)
        - angular_z: Z轴角速度 (rad/s) - 左转(+)/右转(-)
        - Y轴速度: 强制设为0.0 (差分驱动机器人不支持侧向运动)
        """
        if self.emergency_stop:
            return
        
        # 确保参数为浮点数并限制速度范围
        linear_x = float(linear_x) if linear_x is not None else 0.0
        angular_z = float(angular_z) if angular_z is not None else 0.0
        
        # ⚠️ Y轴速度强制设为0（差分驱动不支持侧向运动）
        linear_y = 0.0
        
        linear_x = max(-self.max_linear_speed, min(self.max_linear_speed, linear_x))
        angular_z = max(-self.max_angular_speed, min(self.max_angular_speed, angular_z))
        
        # 🔧 应用角速度方向修正（解决转向方向错误问题）
        angular_z = self.apply_angular_velocity_correction(angular_z)
        
        # 准备控制指令信息（用于WebSocket转发）
        command_info = {
            "type": "robot_control_command",
            "timestamp": int(time.time() * 1000),
            "linear_x": round(linear_x, 3),
            "linear_y": round(linear_y, 3),  # 明确记录Y轴为0
            "angular_z": round(angular_z, 3),
            "control_mode": self.control_mode.value,
            "control_type": self.control_type,
            "motor_speed": self.motor_speed,
            "emergency_stop": self.emergency_stop,
            "use_ackermann": self.use_ackermann,
            "dynamic_speed_enabled": self.enable_dynamic_speed,
            "pid_enabled": self.enable_pid_control
        }
        
        if self.use_ackermann:
            # 阿克曼控制
            steering_angle = self.convert_to_steering_angle(linear_x, angular_z)
            command_info["steering_angle"] = round(steering_angle, 3)
            
            msg = AckermannDriveStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "base_link"
            msg.drive.speed = linear_x
            msg.drive.steering_angle = steering_angle
            
            self.cmd_publisher.publish(msg)
            # 只在有实际运动时打印日志
            if abs(linear_x) > 0.001 or abs(steering_angle) > 0.001:
                self.get_logger().info(f"🚗📤 [robot_control_node] 发布阿克曼命令: 速度={linear_x:.3f}, 转向角={steering_angle:.3f}")
        else:
            # 标准Twist控制 - 明确设置各轴速度
            msg = Twist()
            # 🎯 机器人运动控制：只使用X轴和Z轴，Y轴强制设为0
            msg.linear.x = linear_x    # X轴：前进(+)/后退(-)线速度
            msg.linear.y = 0.0         # Y轴：侧向速度，强制设为0（差分驱动不支持）
            msg.linear.z = 0.0         # Z轴：垂直速度，强制设为0（2D平面运动）
            msg.angular.x = 0.0        # X轴旋转：俯仰角速度，强制设为0（2D平面运动）
            msg.angular.y = 0.0        # Y轴旋转：翻滚角速度，强制设为0（2D平面运动）
            msg.angular.z = angular_z  # Z轴：偏航角速度，左转(+)/右转(-)
            
            self.cmd_publisher.publish(msg)
            # 只在有实际运动时打印日志
            if abs(linear_x) > 0.001 or abs(angular_z) > 0.001:
                self.get_logger().info(f"🚗📤 [robot_control_node] 发布Twist命令: X轴(前进)={linear_x:.3f}, Z轴(旋转)={angular_z:.3f}, Y轴(侧向)=0.0")
        
        # 发布控制指令信息到WebSocket桥接节点
        command_msg = String()
        command_msg.data = json.dumps(command_info)
        self.control_command_publisher.publish(command_msg)
        
        # 只在有实际运动时打印WebSocket发布日志
        if abs(linear_x) > 0.001 or abs(angular_z) > 0.001:
            self.get_logger().info(f"🌐📤 [robot_control_node] 发布控制指令到WebSocket: X轴={linear_x:.3f}, Z轴={angular_z:.3f}, Y轴=0.0")

    def send_stop_command(self):
        """发送停止命令"""
        self.send_velocity_command(0.0, 0.0)

    def convert_to_steering_angle(self, linear_vel, angular_vel):
        """将线速度和角速度转换为阿克曼转向角"""
        if angular_vel == 0 or linear_vel == 0:
            return 0.0
        
        radius = linear_vel / angular_vel
        return math.atan(self.wheelbase / radius)

    def quaternion_to_yaw(self, q):
        """四元数转换为偏航角"""
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def control_loop(self):
        """主控制循环"""
        try:
            if self.control_mode == ControlMode.FOLLOWING:
                # 检查人体检测超时
                if (time.time() - self.last_detection_time) > self.detection_timeout:
                    self.send_stop_command()
                    self.get_logger().warn("人体检测超时，停止跟随")
            
            elif self.control_mode == ControlMode.STOP:
                self.send_stop_command()
                
        except Exception as e:
            self.get_logger().error(f"控制循环错误: {e}")

    def safety_check(self):
        """安全检查"""
        if not self.safety_enabled:
            return
        
        try:
            # 这里可以添加更多安全检查逻辑
            # 例如：障碍物检测、电池电量检查等
            pass
            
        except Exception as e:
            self.get_logger().error(f"安全检查错误: {e}")

    def publish_status(self):
        """发布状态信息"""
        try:
            status_msg = String()
            status_info = {
                "mode": self.control_mode.value,
                "is_following": self.control_mode == ControlMode.FOLLOWING,
                "manual_mode": self.control_mode == ControlMode.MANUAL,
                "target_person": self.target_person_name,
                "following_enabled": self.following_enabled,
                "emergency_stop": self.emergency_stop,
                "motor_speed": self.motor_speed,
                "control_type": self.control_type,
                "dynamic_speed_enabled": self.enable_dynamic_speed,
                "pid_enabled": self.enable_pid_control,
                "smoothing_enabled": self.enable_speed_smoothing,
                "prediction_enabled": self.distance_prediction_enabled,
                "last_detection": time.time() - self.last_detection_time if self.last_detection_time > 0 else -1
            }
            status_msg.data = str(status_info)
            self.status_publisher.publish(status_msg)
            
        except Exception as e:
            self.get_logger().error(f"状态发布错误: {e}")

    # 公共接口方法
    def start_following(self, person_name=""):
        """开始跟随模式"""
        if person_name:
            self.target_person_name = person_name
        self.set_control_mode(ControlMode.FOLLOWING)
        self.get_logger().info(f"🎯 开始跟随: {self.target_person_name}")

    def stop_following(self):
        """停止跟随"""
        self.set_control_mode(ControlMode.STOP)
        self.get_logger().info("⏹️ 停止跟随")

    def enable_manual_control(self):
        """启用手动控制"""
        self.set_control_mode(ControlMode.MANUAL)
        self.get_logger().info("🎮 启用手动控制")

    def set_safety_mode(self, enabled):
        """设置安全模式"""
        self.safety_enabled = enabled
        self.get_logger().info(f"🛡️ 安全模式: {'启用' if enabled else '禁用'}")

    def set_dynamic_speed_mode(self, enabled):
        """设置动态速度模式"""
        self.enable_dynamic_speed = enabled
        if self.dynamic_speed_controller:
            self.dynamic_speed_controller.reset()
        self.get_logger().info(f"⚡ 动态速度调节: {'启用' if enabled else '禁用'}")

    def set_pid_control_mode(self, enabled):
        """设置PID控制模式"""
        self.enable_pid_control = enabled
        if self.distance_pid:
            self.distance_pid.reset()
        if self.angle_pid:
            self.angle_pid.reset()
        self.get_logger().info(f"🎛️ PID控制: {'启用' if enabled else '禁用'}")


def main(args=None):
    """主函数"""
    try:
        rclpy.init(args=args)
        node = RobotControlNode()
        
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            node.get_logger().info("收到停止信号")
        finally:
            node.send_stop_command()  # 确保停止
            node.destroy_node()
            rclpy.shutdown()
            
    except Exception as e:
        print(f"主函数错误: {e}")


if __name__ == '__main__':
    main()