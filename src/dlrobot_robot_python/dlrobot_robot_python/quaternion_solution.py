#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
四元数姿态解算模块 - Python实现
============================
基于Madgwick算法的IMU数据融合
用于从加速度计和陀螺仪数据计算四元数姿态

作者: AI Assistant
日期: 2024
基于: 原C++版本的Quaternion_Solution
"""

import math
from typing import List, Tuple


class QuaternionSolution:
    """四元数姿态解算类"""
    
    def __init__(self, sampling_freq: float = 20.0):
        """
        初始化四元数解算器
        
        Args:
            sampling_freq: 采样频率 (Hz)
        """
        self.sampling_freq = sampling_freq
        self.dt = 1.0 / sampling_freq
        
        # Madgwick算法参数
        self.beta = 1.0  # 2 * proportional gain (Kp)
        self.zeta = 0.0  # 2 * integral gain (Ki)
        
        # 四元数 [w, x, y, z]
        self.q = [1.0, 0.0, 0.0, 0.0]
        
        # 积分误差项
        self.integral_fb = [0.0, 0.0, 0.0]
        
        # 存储原始传感器数据
        self.gyro_data = [0.0, 0.0, 0.0]
        self.accel_data = [0.0, 0.0, 0.0]
    
    def inv_sqrt(self, number: float) -> float:
        """快速平方根倒数算法（魔法数字算法的Python实现）"""
        if number <= 0:
            return 0.0
        return 1.0 / math.sqrt(number)
    
    def normalize_vector(self, vector: List[float]) -> List[float]:
        """向量归一化"""
        magnitude = math.sqrt(sum(x*x for x in vector))
        if magnitude == 0:
            return vector
        return [x / magnitude for x in vector]
    
    def update(self, gx: float, gy: float, gz: float, 
               ax: float, ay: float, az: float):
        """
        更新四元数姿态
        
        Args:
            gx, gy, gz: 陀螺仪数据 (rad/s)
            ax, ay, az: 加速度计数据 (m/s²)
        """
        # 保存原始数据
        self.gyro_data = [gx, gy, gz]
        self.accel_data = [ax, ay, az]
        
        # 提取四元数分量
        q0, q1, q2, q3 = self.q
        
        # 如果加速度计数据无效，跳过姿态修正
        if not (ax == 0.0 and ay == 0.0 and az == 0.0):
            # 归一化加速度计数据
            accel_norm = self.normalize_vector([ax, ay, az])
            ax, ay, az = accel_norm
            
            # 从四元数计算重力方向的估计值
            # 这些是方向余弦矩阵第三行的元素（重力方向）
            vx = 2 * (q1 * q3 - q0 * q2)
            vy = 2 * (q0 * q1 + q2 * q3)
            vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3
            
            # 计算误差（测量重力方向与估计重力方向的叉积）
            ex = ay * vz - az * vy
            ey = az * vx - ax * vz
            ez = ax * vy - ay * vx
            
            # 应用积分反馈项（如果启用）
            if self.zeta > 0.0:
                self.integral_fb[0] += self.zeta * ex * self.dt
                self.integral_fb[1] += self.zeta * ey * self.dt
                self.integral_fb[2] += self.zeta * ez * self.dt
                gx += self.integral_fb[0]
                gy += self.integral_fb[1]
                gz += self.integral_fb[2]
            else:
                self.integral_fb = [0.0, 0.0, 0.0]
            
            # 应用比例反馈
            gx += self.beta * ex
            gy += self.beta * ey
            gz += self.beta * ez
        
        # 积分四元数变化率
        gx *= 0.5 * self.dt
        gy *= 0.5 * self.dt
        gz *= 0.5 * self.dt
        
        # 四元数微分方程
        qa, qb, qc, qd = q0, q1, q2, q3
        
        q0 += (-qb * gx - qc * gy - qd * gz)
        q1 += (qa * gx + qc * gz - qd * gy)
        q2 += (qa * gy - qb * gz + qd * gx)
        q3 += (qa * gz + qb * gy - qc * gx)
        
        # 归一化四元数
        quat_norm = self.normalize_vector([q0, q1, q2, q3])
        self.q = quat_norm
    
    def get_quaternion(self) -> Tuple[float, float, float, float]:
        """
        获取当前四元数
        
        Returns:
            四元数 (w, x, y, z)
        """
        return (self.q[0], self.q[1], self.q[2], self.q[3])
    
    def get_euler_angles(self) -> Tuple[float, float, float]:
        """
        获取欧拉角 (roll, pitch, yaw)
        
        Returns:
            欧拉角 (roll, pitch, yaw) in radians
        """
        q0, q1, q2, q3 = self.q
        
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (q0 * q1 + q2 * q3)
        cosr_cosp = 1 - 2 * (q1 * q1 + q2 * q2)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (q0 * q2 - q3 * q1)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)  # use 90 degrees if out of range
        else:
            pitch = math.asin(sinp)
        
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (q0 * q3 + q1 * q2)
        cosy_cosp = 1 - 2 * (q2 * q2 + q3 * q3)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return roll, pitch, yaw
    
    def get_gyroscope(self) -> Tuple[float, float, float]:
        """
        获取陀螺仪数据
        
        Returns:
            陀螺仪数据 (gx, gy, gz) in rad/s
        """
        return (self.gyro_data[0], self.gyro_data[1], self.gyro_data[2])
    
    def get_accelerometer(self) -> Tuple[float, float, float]:
        """
        获取加速度计数据
        
        Returns:
            加速度计数据 (ax, ay, az) in m/s²
        """
        return (self.accel_data[0], self.accel_data[1], self.accel_data[2])
    
    def reset(self):
        """重置四元数解算器"""
        self.q = [1.0, 0.0, 0.0, 0.0]
        self.integral_fb = [0.0, 0.0, 0.0]
        self.gyro_data = [0.0, 0.0, 0.0]
        self.accel_data = [0.0, 0.0, 0.0]
    
    def set_beta(self, beta: float):
        """设置比例增益"""
        self.beta = beta
    
    def set_zeta(self, zeta: float):
        """设置积分增益"""
        self.zeta = zeta
    
    def set_sampling_freq(self, freq: float):
        """设置采样频率"""
        self.sampling_freq = freq
        self.dt = 1.0 / freq 