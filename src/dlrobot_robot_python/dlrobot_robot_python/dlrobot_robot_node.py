#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
DLRobot机器人底盘驱动节点 - Python实现
==================================
功能: 
- 通过串口与下位机通信
- 接收cmd_vel和ackermann_cmd控制指令
- 发布里程计、IMU、电压等传感器数据
- 支持普通驱动和阿克曼驱动模式

作者: AI Assistant
日期: 2024
基于: 原C++版本的turn_on_dlrobot_robot包
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
import serial
import struct
import time
import math
import numpy as np
import threading
from typing import Optional, Tuple

# ROS2 消息类型
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from ackermann_msgs.msg import AckermannDriveStamped
from dlrobot_robot_msg.msg import Data

# TF2相关
import tf2_ros
from tf2_ros import TransformBroadcaster
import tf2_geometry_msgs

from .quaternion_solution import QuaternionSolution


class DLRobotConfig:
    """DLRobot配置常量类"""
    
    # 串口通信协议
    FRAME_HEADER = 0x7B
    FRAME_TAIL = 0x7D
    SEND_DATA_SIZE = 11
    RECEIVE_DATA_SIZE = 24
    
    # 数据校验标志
    SEND_DATA_CHECK = 1
    READ_DATA_CHECK = 0
    
    # IMU数据转换系数
    GYROSCOPE_RATIO = 0.00026644  # ±500°/s量程
    ACCEL_RATIO = 1671.84  # ±2g量程
    
    # 数学常量
    PI = 3.1415926


class SerialProtocol:
    """串口通信协议处理类"""
    
    def __init__(self):
        self.send_data = [0] * DLRobotConfig.SEND_DATA_SIZE
        self.receive_data = [0] * DLRobotConfig.RECEIVE_DATA_SIZE
    
    def pack_send_data(self, x_speed: float, y_speed: float, z_speed: float) -> bytes:
        """打包发送数据"""
        self.send_data[0] = DLRobotConfig.FRAME_HEADER
        self.send_data[1] = 0  # 预留位
        self.send_data[2] = 0  # 预留位
        
        # X轴速度 (放大1000倍)
        x_speed_int = int(x_speed * 1000)
        self.send_data[3] = (x_speed_int >> 8) & 0xFF  # 高字节
        self.send_data[4] = x_speed_int & 0xFF         # 低字节
        
        # Y轴速度
        y_speed_int = int(y_speed * 1000)
        self.send_data[5] = (y_speed_int >> 8) & 0xFF
        self.send_data[6] = y_speed_int & 0xFF
        
        # Z轴角速度
        z_speed_int = int(z_speed * 1000)
        self.send_data[7] = (z_speed_int >> 8) & 0xFF
        self.send_data[8] = z_speed_int & 0xFF
        
        # BBC校验
        self.send_data[9] = self.calculate_checksum(9, DLRobotConfig.SEND_DATA_CHECK)
        self.send_data[10] = DLRobotConfig.FRAME_TAIL
        
        return bytes(self.send_data)
    
    def unpack_receive_data(self, data: bytes) -> Optional[dict]:
        """解析接收数据"""
        if len(data) < DLRobotConfig.RECEIVE_DATA_SIZE:
            return None
        
        # 找到帧头和帧尾位置
        header_pos = -1
        tail_pos = -1
        
        for i in range(len(data)):
            if data[i] == DLRobotConfig.FRAME_HEADER:
                header_pos = i
            elif data[i] == DLRobotConfig.FRAME_TAIL:
                tail_pos = i
        
        # 数据包验证
        if tail_pos == (header_pos + 23):
            # 帧尾在正确位置
            self.receive_data = list(data)
        elif header_pos == (1 + tail_pos):
            # 数据包需要重新排列
            for j in range(24):
                self.receive_data[j] = data[(j + header_pos) % 24]
        else:
            return None
        
        # 验证帧头帧尾
        if (self.receive_data[0] != DLRobotConfig.FRAME_HEADER or 
            self.receive_data[23] != DLRobotConfig.FRAME_TAIL):
            return None
        
        # BBC校验
        if (self.receive_data[22] != self.calculate_checksum(22, DLRobotConfig.READ_DATA_CHECK) and
            not (header_pos == (1 + tail_pos))):
            return None
        
        # 解析数据
        result = {
            'flag_stop': self.receive_data[1],
            'x_speed': self.convert_odom_data(self.receive_data[2], self.receive_data[3]),
            'y_speed': self.convert_odom_data(self.receive_data[4], self.receive_data[5]),
            'z_speed': self.convert_odom_data(self.receive_data[6], self.receive_data[7]),
            'accel_x': self.convert_imu_data(self.receive_data[8], self.receive_data[9]),
            'accel_y': self.convert_imu_data(self.receive_data[10], self.receive_data[11]),
            'accel_z': self.convert_imu_data(self.receive_data[12], self.receive_data[13]),
            'gyro_x': self.convert_imu_data(self.receive_data[14], self.receive_data[15]),
            'gyro_y': self.convert_imu_data(self.receive_data[16], self.receive_data[17]),
            'gyro_z': self.convert_imu_data(self.receive_data[18], self.receive_data[19]),
            'voltage': self.convert_voltage_data(self.receive_data[20], self.receive_data[21])
        }
        
        return result
    
    def calculate_checksum(self, count: int, mode: int) -> int:
        """计算BBC校验和"""
        checksum = 0
        data_source = self.receive_data if mode == DLRobotConfig.READ_DATA_CHECK else self.send_data
        
        for i in range(count):
            checksum ^= data_source[i]
        
        return checksum & 0xFF
    
    def convert_imu_data(self, high_byte: int, low_byte: int) -> int:
        """转换IMU数据"""
        value = (high_byte << 8) | low_byte
        # 处理有符号16位整数
        if value > 32767:
            value -= 65536
        return value
    
    def convert_odom_data(self, high_byte: int, low_byte: int) -> float:
        """转换里程计数据"""
        value = (high_byte << 8) | low_byte
        # 处理有符号16位整数
        if value > 32767:
            value -= 65536
        return (value / 1000) + (value % 1000) * 0.001
    
    def convert_voltage_data(self, high_byte: int, low_byte: int) -> float:
        """转换电压数据"""
        value = (high_byte << 8) | low_byte
        return (value / 1000) + (value % 1000) * 0.001


class DLRobotNode(Node):
    """DLRobot机器人控制节点"""
    
    def __init__(self):
        super().__init__('dlrobot_robot_node')
        
        # 初始化变量
        self.protocol = SerialProtocol()
        self.quaternion_solver = QuaternionSolution()
        self.serial_connection: Optional[serial.Serial] = None
        
        # 机器人状态
        self.robot_pos = {'x': 0.0, 'y': 0.0, 'z': 0.0}  # 位置
        self.robot_vel = {'x': 0.0, 'y': 0.0, 'z': 0.0}  # 速度
        self.power_voltage = 0.0
        
        # 时间相关
        self.last_time = self.get_clock().now()
        self.sampling_time = 0.0
        
        # 声明参数
        self.declare_node_parameters()
        self.get_node_parameters()
        
        # 创建发布者
        self.create_publishers()
        
        # 创建订阅者
        self.create_subscribers()
        
        # 创建TF广播器
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # 初始化串口
        self.init_serial()
        
        # 创建定时器用于主控制循环
        self.create_timer(0.02, self.control_loop)  # 50Hz
        
        # 电压发布计数器
        self.voltage_pub_counter = 0
        
        self.get_logger().info('🚀 DLRobot Python节点初始化完成!')
    
    def declare_node_parameters(self):
        """声明ROS2参数"""
        self.declare_parameter('serial_baud_rate', 115200)
        self.declare_parameter('usart_port_name', '/dev/dlrobot_controller')
        self.declare_parameter('cmd_vel', 'cmd_vel')
        self.declare_parameter('akm_cmd_vel', 'ackermann_cmd')
        self.declare_parameter('odom_frame_id', 'odom_combined')
        self.declare_parameter('robot_frame_id', 'base_footprint')
        self.declare_parameter('gyro_frame_id', 'gyro_link')
    
    def get_node_parameters(self):
        """获取ROS2参数"""
        self.serial_baud_rate = self.get_parameter('serial_baud_rate').value
        self.usart_port_name = self.get_parameter('usart_port_name').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel').value
        self.akm_cmd_vel_topic = self.get_parameter('akm_cmd_vel').value
        self.odom_frame_id = self.get_parameter('odom_frame_id').value
        self.robot_frame_id = self.get_parameter('robot_frame_id').value
        self.gyro_frame_id = self.get_parameter('gyro_frame_id').value
        
        self.get_logger().info(f'📡 参数配置: 串口={self.usart_port_name}, 波特率={self.serial_baud_rate}')
        self.get_logger().info(f'📡 话题配置: cmd_vel={self.cmd_vel_topic}, akm_cmd_vel={self.akm_cmd_vel_topic}')
    
    def create_publishers(self):
        """创建ROS2发布者"""
        qos = QoSProfile(depth=10)
        
        self.odom_publisher = self.create_publisher(Odometry, 'odom_combined', qos)
        self.imu_publisher = self.create_publisher(Imu, 'mobile_base/sensors/imu_data', qos)
        self.voltage_publisher = self.create_publisher(Float32, 'PowerVoltage', qos)
        self.robotpose_publisher = self.create_publisher(Data, 'robotpose', qos)
        self.robotvel_publisher = self.create_publisher(Data, 'robotvel', qos)
        
        self.get_logger().info('📢 发布者创建完成')
    
    def create_subscribers(self):
        """创建ROS2订阅者"""
        qos = QoSProfile(depth=100)
        
        # 普通速度控制订阅
        self.cmd_vel_sub = self.create_subscription(
            Twist, self.cmd_vel_topic, self.cmd_vel_callback, qos)
        
        # 阿克曼驱动控制订阅（如果需要）
        if self.akm_cmd_vel_topic != "none" and self.akm_cmd_vel_topic:
            self.akm_cmd_vel_sub = self.create_subscription(
                AckermannDriveStamped, self.akm_cmd_vel_topic, 
                self.akm_cmd_vel_callback, qos)
            self.get_logger().info(f'📡 创建了Ackermann话题订阅者: {self.akm_cmd_vel_topic}')
        
        self.get_logger().info(f'📡 话题订阅初始化完成: {self.cmd_vel_topic}')
    
    def init_serial(self):
        """初始化串口连接"""
        try:
            self.serial_connection = serial.Serial(
                port=self.usart_port_name,
                baudrate=self.serial_baud_rate,
                timeout=2.0
            )
            self.get_logger().info(f'✅ 串口连接成功: {self.usart_port_name}')
        except serial.SerialException as e:
            self.get_logger().error(f'❌ 串口连接失败: {e}')
            self.serial_connection = None
    
    def cmd_vel_callback(self, msg: Twist):
        """普通速度控制回调函数"""
        # 只在有实际运动时打印日志
        if abs(msg.linear.x) > 0.001 or abs(msg.angular.z) > 0.001:
            self.get_logger().info(f'🔄🔥 [dlrobot_robot_node] 收到Twist消息: linear.x={msg.linear.x:.3f}, angular.z={msg.angular.z:.3f}')
        
        # 只有在非阿克曼模式下才处理
        if self.akm_cmd_vel_topic == "none":
            self.send_velocity_command(msg.linear.x, msg.linear.y, msg.angular.z)
    
    def akm_cmd_vel_callback(self, msg: AckermannDriveStamped):
        """阿克曼驱动控制回调函数"""
        # 只在有实际运动时打印日志
        if abs(msg.drive.speed) > 0.001 or abs(msg.drive.steering_angle) > 0.001:
            self.get_logger().info(f'🔄🔥 [dlrobot_robot_node] 收到Ackermann消息: speed={msg.drive.speed:.3f}, steering_angle={msg.drive.steering_angle:.3f}')
        
        # 阿克曼模式下，转向角需要除以2（根据原C++代码）
        steering_scaled = msg.drive.steering_angle / 2.0
        self.send_velocity_command(msg.drive.speed, 0.0, steering_scaled)
    
    def send_velocity_command(self, x_speed: float, y_speed: float, z_speed: float):
        """发送速度控制命令到下位机"""
        if not self.serial_connection or not self.serial_connection.is_open:
            self.get_logger().warn(f'⚠️ 串口未连接，无法发送命令: X={x_speed:.3f}, Y={y_speed:.3f}, Z={z_speed:.3f}')
            return
        
        try:
            # 打包数据
            data_to_send = self.protocol.pack_send_data(x_speed, y_speed, z_speed)
            
            # 发送数据
            self.serial_connection.write(data_to_send)
            
            # 只在有实际运动时打印日志
            if abs(x_speed) > 0.001 or abs(y_speed) > 0.001 or abs(z_speed) > 0.001:
                self.get_logger().info(f'📤🔥 [dlrobot_robot_node] 发送串口数据: X={x_speed:.3f}, Y={y_speed:.3f}, Z={z_speed:.3f}')
                # 显示实际发送的字节数据（用于调试）
                hex_data = ' '.join([f'{b:02X}' for b in data_to_send])
                self.get_logger().info(f'📡🔥 [dlrobot_robot_node] 实际串口字节: {hex_data}')
            
        except serial.SerialException as e:
            self.get_logger().error(f'❌ 串口发送失败: {e}')
    
    def read_sensor_data(self) -> bool:
        """从串口读取传感器数据"""
        if not self.serial_connection or not self.serial_connection.is_open:
            return False
        
        try:
            # 检查是否有可用数据
            if self.serial_connection.in_waiting < DLRobotConfig.RECEIVE_DATA_SIZE:
                return False
            
            # 读取数据
            raw_data = self.serial_connection.read(DLRobotConfig.RECEIVE_DATA_SIZE)
            
            # 解析数据
            parsed_data = self.protocol.unpack_receive_data(raw_data)
            
            if parsed_data is None:
                return False
            
            # 更新机器人状态
            self.robot_vel['x'] = parsed_data['x_speed']
            self.robot_vel['y'] = parsed_data['y_speed']
            self.robot_vel['z'] = parsed_data['z_speed']
            self.power_voltage = parsed_data['voltage']
            
            # 处理IMU数据
            accel_x = parsed_data['accel_x'] / DLRobotConfig.ACCEL_RATIO
            accel_y = parsed_data['accel_y'] / DLRobotConfig.ACCEL_RATIO
            accel_z = parsed_data['accel_z'] / DLRobotConfig.ACCEL_RATIO
            
            gyro_x = parsed_data['gyro_x'] * DLRobotConfig.GYROSCOPE_RATIO
            gyro_y = parsed_data['gyro_y'] * DLRobotConfig.GYROSCOPE_RATIO
            gyro_z = parsed_data['gyro_z'] * DLRobotConfig.GYROSCOPE_RATIO
            
            # 四元数解算
            self.quaternion_solver.update(gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z)
            
            return True
            
        except serial.SerialException as e:
            self.get_logger().error(f'❌ 串口读取失败: {e}')
            return False
    
    def control_loop(self):
        """主控制循环"""
        current_time = self.get_clock().now()
        self.sampling_time = (current_time - self.last_time).nanoseconds / 1e9
        
        # 读取传感器数据
        if self.read_sensor_data():
            # 更新机器人位置（积分计算）
            self.update_robot_position()
            
            # 发布各种话题
            self.publish_odometry()
            self.publish_imu_data()
            self.publish_voltage()
            self.publish_robot_data()
        
        self.last_time = current_time
    
    def update_robot_position(self):
        """更新机器人位置"""
        if self.sampling_time <= 0:
            return
        
        # 考虑机器人当前姿态的运动学模型
        cos_theta = math.cos(self.robot_pos['z'])
        sin_theta = math.sin(self.robot_pos['z'])
        
        # 计算位置增量
        dx = (self.robot_vel['x'] * cos_theta - self.robot_vel['y'] * sin_theta) * self.sampling_time
        dy = (self.robot_vel['x'] * sin_theta + self.robot_vel['y'] * cos_theta) * self.sampling_time
        dz = self.robot_vel['z'] * self.sampling_time
        
        # 更新位置
        self.robot_pos['x'] += dx
        self.robot_pos['y'] += dy
        self.robot_pos['z'] += dz
    
    def publish_odometry(self):
        """发布里程计数据"""
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = self.odom_frame_id
        odom_msg.child_frame_id = self.robot_frame_id
        
        # 位置
        odom_msg.pose.pose.position.x = self.robot_pos['x']
        odom_msg.pose.pose.position.y = self.robot_pos['y']
        odom_msg.pose.pose.position.z = 0.0
        
        # 姿态（由Z轴旋转角度转换为四元数）
        quat = self.euler_to_quaternion(0, 0, self.robot_pos['z'])
        odom_msg.pose.pose.orientation = quat
        
        # 速度
        odom_msg.twist.twist.linear.x = self.robot_vel['x']
        odom_msg.twist.twist.linear.y = self.robot_vel['y']
        odom_msg.twist.twist.angular.z = self.robot_vel['z']
        
        # 协方差矩阵（简化版本）
        odom_msg.pose.covariance[0] = 1e-3   # x
        odom_msg.pose.covariance[7] = 1e-3   # y
        odom_msg.pose.covariance[35] = 1e3   # yaw
        
        odom_msg.twist.covariance[0] = 1e-3   # vx
        odom_msg.twist.covariance[7] = 1e-3   # vy
        odom_msg.twist.covariance[35] = 1e3   # vyaw
        
        self.odom_publisher.publish(odom_msg)
    
    def publish_imu_data(self):
        """发布IMU数据"""
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = self.gyro_frame_id
        
        # 四元数姿态
        q = self.quaternion_solver.get_quaternion()
        imu_msg.orientation.w = q[0]
        imu_msg.orientation.x = q[1]
        imu_msg.orientation.y = q[2]
        imu_msg.orientation.z = q[3]
        
        # 角速度
        gyro = self.quaternion_solver.get_gyroscope()
        imu_msg.angular_velocity.x = gyro[0]
        imu_msg.angular_velocity.y = gyro[1]
        imu_msg.angular_velocity.z = gyro[2]
        
        # 线性加速度
        accel = self.quaternion_solver.get_accelerometer()
        imu_msg.linear_acceleration.x = accel[0]
        imu_msg.linear_acceleration.y = accel[1]
        imu_msg.linear_acceleration.z = accel[2]
        
        # 协方差矩阵
        imu_msg.orientation_covariance[0] = 1e6
        imu_msg.orientation_covariance[4] = 1e6
        imu_msg.orientation_covariance[8] = 1e-6
        
        imu_msg.angular_velocity_covariance[0] = 1e6
        imu_msg.angular_velocity_covariance[4] = 1e6
        imu_msg.angular_velocity_covariance[8] = 1e-6
        
        self.imu_publisher.publish(imu_msg)
    
    def publish_voltage(self):
        """发布电压数据"""
        self.voltage_pub_counter += 1
        if self.voltage_pub_counter > 10:  # 降低发布频率
            self.voltage_pub_counter = 0
            voltage_msg = Float32()
            voltage_msg.data = self.power_voltage
            self.voltage_publisher.publish(voltage_msg)
    
    def publish_robot_data(self):
        """发布机器人位置和速度数据"""
        # 位置数据
        pose_msg = Data()
        pose_msg.x = self.robot_pos['x']
        pose_msg.y = self.robot_pos['y']
        pose_msg.z = self.robot_pos['z']
        self.robotpose_publisher.publish(pose_msg)
        
        # 速度数据
        vel_msg = Data()
        vel_msg.x = self.robot_vel['x']
        vel_msg.y = self.robot_vel['y']
        vel_msg.z = self.robot_vel['z']
        self.robotvel_publisher.publish(vel_msg)
    
    def euler_to_quaternion(self, roll: float, pitch: float, yaw: float) -> Quaternion:
        """欧拉角转四元数"""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        q = Quaternion()
        q.w = cy * cp * cr + sy * sp * sr
        q.x = cy * cp * sr - sy * sp * cr
        q.y = sy * cp * sr + cy * sp * cr
        q.z = sy * cp * cr - cy * sp * sr
        
        return q
    
    def destroy_node(self):
        """节点销毁时的清理工作"""
        # 发送停止命令
        if self.serial_connection and self.serial_connection.is_open:
            self.get_logger().info('📤 发送停止命令')
            self.send_velocity_command(0.0, 0.0, 0.0)
            time.sleep(0.1)  # 确保命令发送完成
            self.serial_connection.close()
        
        self.get_logger().info('✅ DLRobot Python节点已关闭')
        super().destroy_node()


def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    try:
        node = DLRobotNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 