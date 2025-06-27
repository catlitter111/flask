#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
小车控制节点
============
整合turn_on_dlrobot_robot功能，提供小车运动控制接口
支持多种控制模式：跟随模式、手动控制模式、导航模式
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
import math
import time
from enum import Enum

# ROS消息类型
from geometry_msgs.msg import Twist, Point
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


class RobotControlNode(Node):
    """小车控制节点"""
    
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
        
        self.setup_parameters()
        self.setup_publishers()
        self.setup_subscribers()
        self.setup_services()
        self.setup_timers()
        
        self.get_logger().info("🚗 小车控制节点初始化完成")
        self.get_logger().info(f"🎮 当前控制模式: {self.control_mode.value}")

    def setup_parameters(self):
        """设置参数"""
        # 声明参数
        self.declare_parameter('max_linear_speed', 0.5)
        self.declare_parameter('max_angular_speed', 1.0)
        self.declare_parameter('min_follow_distance', 1.0)
        self.declare_parameter('max_follow_distance', 3.0)
        self.declare_parameter('follow_speed_factor', 0.3)
        self.declare_parameter('wheelbase', 0.143)
        self.declare_parameter('use_ackermann', False)
        self.declare_parameter('safety_enabled', True)
        
        # 获取参数值
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.min_follow_distance = self.get_parameter('min_follow_distance').value
        self.max_follow_distance = self.get_parameter('max_follow_distance').value
        self.follow_speed_factor = self.get_parameter('follow_speed_factor').value
        self.wheelbase = self.get_parameter('wheelbase').value
        self.use_ackermann = self.get_parameter('use_ackermann').value
        self.safety_enabled = self.get_parameter('safety_enabled').value

    def setup_publishers(self):
        """设置发布者"""
        qos = QoSProfile(depth=10)
        
        # 控制命令发布者
        if self.use_ackermann:
            self.cmd_publisher = self.create_publisher(
                AckermannDriveStamped, '/ackermann_cmd', qos)
        else:
            self.cmd_publisher = self.create_publisher(
                Twist, '/cmd_vel', qos)
        
        # 状态发布者
        self.status_publisher = self.create_publisher(
            String, '/robot_control/status', qos)
        self.mode_publisher = self.create_publisher(
            String, '/robot_control/mode', qos)

    def setup_subscribers(self):
        """设置订阅者"""
        qos = QoSProfile(depth=10)
        
        # 手动控制订阅
        self.manual_cmd_sub = self.create_subscription(
            Twist, '/robot_control/manual_cmd', 
            self.manual_cmd_callback, qos)
        
        # 人体位置订阅
        self.person_position_sub = self.create_subscription(
            Position, '/robot_control/person_position',
            self.person_position_callback, qos)
        
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

    def manual_cmd_callback(self, msg):
        """手动控制命令回调"""
        if self.control_mode == ControlMode.MANUAL:
            self.send_velocity_command(msg.linear.x, msg.angular.z)
            self.get_logger().debug(f"手动控制: linear={msg.linear.x:.2f}, angular={msg.angular.z:.2f}")

    def person_position_callback(self, msg):
        """人体位置回调"""
        self.last_person_position = msg
        self.last_detection_time = time.time()
        
        if self.control_mode == ControlMode.FOLLOWING:
            self.process_following_control(msg)

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
            self.get_logger().warn("🚨 紧急停止激活")

    def set_control_mode(self, mode):
        """设置控制模式"""
        if mode != self.control_mode:
            self.get_logger().info(f"🎮 控制模式切换: {self.control_mode.value} -> {mode.value}")
            
            # 切换模式时先停止
            if mode == ControlMode.STOP:
                self.send_stop_command()
            
            self.control_mode = mode
            
            # 发布模式变更
            mode_msg = String()
            mode_msg.data = mode.value
            self.mode_publisher.publish(mode_msg)

    def process_following_control(self, person_pos):
        """处理跟随控制逻辑"""
        try:
            # 计算距离和角度
            distance = person_pos.distance
            angle_x = person_pos.angle_x  # 水平角度
            angle_y = person_pos.angle_y  # 垂直角度（暂不使用）
            
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
            
            # 发送控制命令
            self.send_velocity_command(linear_vel, angular_vel)
            
            self.get_logger().debug(
                f"跟随控制: dist={distance:.2f}, angle={angle_x:.2f}, "
                f"linear={linear_vel:.2f}, angular={angular_vel:.2f}"
            )
            
        except Exception as e:
            self.get_logger().error(f"跟随控制处理错误: {e}")

    def send_velocity_command(self, linear_x, angular_z):
        """发送速度控制命令"""
        if self.emergency_stop:
            return
        
        # 限制速度范围
        linear_x = max(-self.max_linear_speed, min(self.max_linear_speed, linear_x))
        angular_z = max(-self.max_angular_speed, min(self.max_angular_speed, angular_z))
        
        if self.use_ackermann:
            # 阿克曼控制
            steering_angle = self.convert_to_steering_angle(linear_x, angular_z)
            
            msg = AckermannDriveStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "base_link"
            msg.drive.speed = linear_x
            msg.drive.steering_angle = steering_angle
            
            self.cmd_publisher.publish(msg)
        else:
            # 标准Twist控制
            msg = Twist()
            msg.linear.x = linear_x
            msg.linear.y = 0.0
            msg.linear.z = 0.0
            msg.angular.x = 0.0
            msg.angular.y = 0.0
            msg.angular.z = angular_z
            
            self.cmd_publisher.publish(msg)

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
                "target_person": self.target_person_name,
                "following_enabled": self.following_enabled,
                "emergency_stop": self.emergency_stop,
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