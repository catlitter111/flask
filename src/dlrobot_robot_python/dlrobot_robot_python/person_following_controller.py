#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
人体跟随控制节点
==============
功能: 
- 订阅人体检测距离信息
- 找到距离最近的人体
- 计算跟随控制指令
- 发布cmd_vel控制机器人运动

作者: AI Assistant
日期: 2024
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import json
import math
import time
from typing import Dict, Optional, Tuple


class PersonFollowingController(Node):
    """人体跟随控制器节点"""
    
    def __init__(self):
        super().__init__('person_following_controller')
        
        # 声明参数
        self.declare_node_parameters()
        self.get_node_parameters()
        
        # 控制状态
        self.current_target_person: Optional[str] = None
        self.persons_data: Dict[str, dict] = {}
        self.last_detection_time = 0.0
        self.is_following = False
        
        # 控制PID参数
        self.linear_pid = {'kp': 0.5, 'ki': 0.0, 'kd': 0.1}
        self.angular_pid = {'kp': 0.8, 'ki': 0.0, 'kd': 0.1}
        
        # PID状态
        self.linear_error_prev = 0.0
        self.angular_error_prev = 0.0
        self.linear_integral = 0.0
        self.angular_integral = 0.0
        
        # 创建发布者和订阅者
        self.create_publishers()
        self.create_subscribers()
        
        # 主控制循环定时器
        self.control_timer = self.create_timer(0.1, self.control_loop)  # 10Hz
        
        # 安全检查定时器
        self.safety_timer = self.create_timer(0.5, self.safety_check)  # 2Hz
        
        self.get_logger().info('🎯 人体跟随控制器节点已启动')
        self.get_logger().info(f'📐 跟随参数: 目标距离={self.target_distance:.2f}m, 最大线速度={self.max_linear_speed:.2f}m/s')
    
    def declare_node_parameters(self):
        """声明ROS2参数"""
        # 跟随控制参数
        self.declare_parameter('target_distance', 1.0)  # 目标跟随距离(m)
        self.declare_parameter('distance_tolerance', 0.2)  # 距离容忍度(m)
        self.declare_parameter('max_linear_speed', 0.3)  # 最大线速度(m/s)
        self.declare_parameter('max_angular_speed', 0.5)  # 最大角速度(rad/s)
        self.declare_parameter('min_distance', 0.5)  # 最小安全距离(m)
        self.declare_parameter('max_distance', 3.0)  # 最大跟随距离(m)
        
        # 图像相关参数
        self.declare_parameter('image_width', 640)  # 图像宽度
        self.declare_parameter('image_height', 480)  # 图像高度
        self.declare_parameter('camera_fov', 60.0)  # 相机视场角(度)
        
        # 超时参数
        self.declare_parameter('detection_timeout', 2.0)  # 检测超时时间(s)
        
        # 话题名称
        self.declare_parameter('person_positions_topic', '/person_detection/person_positions')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        
        # 调试参数
        self.declare_parameter('debug_mode', True)
    
    def get_node_parameters(self):
        """获取参数值"""
        self.target_distance = float(self.get_parameter('target_distance').value)
        self.distance_tolerance = float(self.get_parameter('distance_tolerance').value)
        self.max_linear_speed = float(self.get_parameter('max_linear_speed').value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)
        self.min_distance = float(self.get_parameter('min_distance').value)
        self.max_distance = float(self.get_parameter('max_distance').value)
        
        self.image_width = int(self.get_parameter('image_width').value)
        self.image_height = int(self.get_parameter('image_height').value)
        self.camera_fov = float(self.get_parameter('camera_fov').value)
        
        self.detection_timeout = float(self.get_parameter('detection_timeout').value)
        
        self.person_positions_topic = str(self.get_parameter('person_positions_topic').value)
        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        
        self.debug_mode = bool(self.get_parameter('debug_mode').value)
    
    def create_publishers(self):
        """创建发布者"""
        qos = QoSProfile(depth=10)
        
        # 速度控制发布者
        self.cmd_vel_pub = self.create_publisher(
            Twist, self.cmd_vel_topic, qos)
        
        # 跟随状态发布者
        self.following_status_pub = self.create_publisher(
            String, '/person_following/status', qos)
        
        self.get_logger().info(f'📡 发布者创建完成: {self.cmd_vel_topic}')
    
    def create_subscribers(self):
        """创建订阅者"""
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # 人体位置信息订阅者
        self.person_positions_sub = self.create_subscription(
            String,
            self.person_positions_topic,
            self.person_positions_callback,
            sensor_qos
        )
        
        self.get_logger().info(f'📡 订阅者创建完成: {self.person_positions_topic}')
    
    def person_positions_callback(self, msg: String):
        """人体位置信息回调函数"""
        try:
            data = json.loads(msg.data)
            self.last_detection_time = time.time()
            
            # 更新人体数据
            self.persons_data = {}
            for person in data.get('persons', []):
                person_id = person.get('id')
                if person_id and person.get('valid_distance', False):
                    self.persons_data[person_id] = {
                        'center': person.get('center', [0, 0]),
                        'distance': person.get('distance', float('inf')),
                        'bbox': person.get('bbox', [0, 0, 0, 0]),
                        'timestamp': time.time()
                    }
            
            # 如果检测到人体，启动跟随
            if self.persons_data:
                self.is_following = True
                # 更新目标人体
                self.update_target_person()
                
                if self.debug_mode:
                    self.get_logger().info(f'🎯 检测到 {len(self.persons_data)} 个人体')
            
        except Exception as e:
            self.get_logger().error(f'❌ 解析人体位置数据失败: {e}')
    
    def update_target_person(self):
        """更新目标跟随人体 - 选择最近的人"""
        if not self.persons_data:
            self.current_target_person = None
            return
        
        # 找到距离最近的人
        closest_person = None
        min_distance = float('inf')
        
        for person_id, person_data in self.persons_data.items():
            distance = person_data['distance']
            if distance < min_distance:
                min_distance = distance
                closest_person = person_id
        
        # 更新目标人体
        if closest_person != self.current_target_person:
            self.current_target_person = closest_person
            if self.debug_mode:
                self.get_logger().info(f'🎯 切换跟随目标: {closest_person} (距离: {min_distance:.2f}m)')
    
    def calculate_control_command(self) -> Tuple[float, float]:
        """计算控制指令"""
        if not self.current_target_person or self.current_target_person not in self.persons_data:
            return 0.0, 0.0
        
        target_data = self.persons_data[self.current_target_person]
        distance = float(target_data['distance'])
        center_x, center_y = target_data['center']
        
        # 计算线速度 - 基于距离误差
        distance_error = distance - self.target_distance
        
        # 距离PID控制
        linear_vel = self.linear_pid['kp'] * distance_error
        self.linear_integral += distance_error
        linear_vel += self.linear_pid['ki'] * self.linear_integral
        linear_vel += self.linear_pid['kd'] * (distance_error - self.linear_error_prev)
        self.linear_error_prev = distance_error
        
        # 计算角速度 - 基于水平偏移
        image_center_x = self.image_width / 2
        horizontal_error = center_x - image_center_x
        
        # 转换为角度误差 (弧度)
        angular_error = horizontal_error * (math.radians(self.camera_fov) / self.image_width)
        
        # 角速度PID控制
        angular_vel = self.angular_pid['kp'] * angular_error
        self.angular_integral += angular_error
        angular_vel += self.angular_pid['ki'] * self.angular_integral
        angular_vel += self.angular_pid['kd'] * (angular_error - self.angular_error_prev)
        self.angular_error_prev = angular_error
        
        # 安全限制
        if distance < self.min_distance:
            # 太近了，后退
            linear_vel = -0.1
        elif distance > self.max_distance:
            # 太远了，停止
            linear_vel = 0.0
            angular_vel = 0.0
        
        # 速度限制
        linear_vel = max(min(linear_vel, self.max_linear_speed), -self.max_linear_speed)
        angular_vel = max(min(angular_vel, self.max_angular_speed), -self.max_angular_speed)
        
        # 死区控制
        if abs(distance_error) < self.distance_tolerance:
            linear_vel = 0.0
        
        if abs(angular_error) < math.radians(5.0):  # 5度死区
            angular_vel = 0.0
        
        return linear_vel, angular_vel
    
    def control_loop(self):
        """主控制循环"""
        try:
            # 计算控制指令
            linear_vel, angular_vel = self.calculate_control_command()
            
            # 创建并发布Twist消息
            twist_msg = Twist()
            twist_msg.linear.x = linear_vel
            twist_msg.angular.z = angular_vel
            
            self.cmd_vel_pub.publish(twist_msg)
            
            # 发布跟随状态
            self.publish_following_status(linear_vel, angular_vel)
            
            # 调试信息
            if self.debug_mode and self.is_following and (abs(linear_vel) > 0.01 or abs(angular_vel) > 0.01):
                if self.current_target_person:
                    target_data = self.persons_data.get(self.current_target_person, {})
                    distance = target_data.get('distance', 0)
                    self.get_logger().info(
                        f'🎯 跟随控制: 目标={self.current_target_person}, 距离={distance:.2f}m, '
                        f'线速度={linear_vel:.2f}, 角速度={angular_vel:.2f}'
                    )
            
        except Exception as e:
            self.get_logger().error(f'❌ 控制循环错误: {e}')
    
    def publish_following_status(self, linear_vel: float, angular_vel: float):
        """发布跟随状态信息"""
        try:
            status_data = {
                'is_following': self.is_following,
                'target_person': self.current_target_person,
                'persons_count': len(self.persons_data),
                'linear_velocity': linear_vel,
                'angular_velocity': angular_vel,
                'timestamp': time.time()
            }
            
            if self.current_target_person and self.current_target_person in self.persons_data:
                target_data = self.persons_data[self.current_target_person]
                status_data['target_distance'] = target_data['distance']
                status_data['target_position'] = target_data['center']
            
            status_msg = String()
            status_msg.data = json.dumps(status_data)
            self.following_status_pub.publish(status_msg)
            
        except Exception as e:
            self.get_logger().error(f'❌ 发布状态信息失败: {e}')
    
    def safety_check(self):
        """安全检查"""
        current_time = time.time()
        
        # 检查检测超时
        if current_time - self.last_detection_time > self.detection_timeout:
            if self.is_following:
                self.get_logger().warn('⚠️ 人体检测超时，停止跟随')
                self.is_following = False
                self.current_target_person = None
                self.persons_data.clear()
                
                # 发送停止指令
                twist_msg = Twist()
                twist_msg.linear.x = 0.0
                twist_msg.angular.z = 0.0
                self.cmd_vel_pub.publish(twist_msg)
        
        # 清理过期的人体数据
        expired_persons = []
        for person_id, person_data in self.persons_data.items():
            if current_time - person_data['timestamp'] > 1.0:  # 1秒过期
                expired_persons.append(person_id)
        
        for person_id in expired_persons:
            del self.persons_data[person_id]
            if person_id == self.current_target_person:
                self.current_target_person = None
    
    def destroy_node(self):
        """节点销毁时发送停止指令"""
        self.get_logger().info('🛑 发送停止指令')
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(twist_msg)
        
        # 小延迟确保消息发送
        time.sleep(0.1)
        
        super().destroy_node()


def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    try:
        node = PersonFollowingController()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()