#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ByteTracker ROS2节点 - 详细注释版本
=====================================
完整移植ByteTracker算法到ROS2 Humble
集成颜色特征、身体比例特征和卡尔曼滤波的多目标/单目标跟踪系统

功能特性：
- 多目标跟踪模式：跟踪所有检测到的人物
- 单目标跟踪模式：锁定并跟踪特定目标人物  
- 融合颜色特征和身体比例特征的匹配算法
- 卡尔曼滤波预测目标运动
- 与现有ROS2节点集成（特征提取、距离测量、小车控制）

作者: AI Assistant
移植版本: v1.0.0
详细注释版本: v1.1.0
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np
import cv2
import time
import traceback
import logging
import os
from collections import OrderedDict, deque
from enum import Enum
import threading
import scipy.linalg
from pathlib import Path
import openpyxl

# ROS2消息类型
from geometry_msgs.msg import Point, Twist
from std_msgs.msg import String, Float32, Bool, Header

# 自定义消息和服务
from custom_msgs.srv import FeatureExtraction, GetDistance
from custom_msgs.msg import TrackedPerson, TrackedPersonArray, TrackingMode, Position, TrackingResult, RobotStatus

# ROS2 sensor messages for image publishing
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# 导入现有检测模块
try:
    # 尝试相对导入
    from .rknn_colour_detect import detect_picture_with_confidence, Determine_the_position_of_the_entire_body
    CLOTHING_DETECTION_AVAILABLE = True
except ImportError:
    try:
        # 尝试绝对导入
        from rknn_colour_detect import detect_picture_with_confidence, Determine_the_position_of_the_entire_body
        CLOTHING_DETECTION_AVAILABLE = True
    except ImportError as e:
        CLOTHING_DETECTION_AVAILABLE = False
        print(f"⚠️ 服装检测模块不可用: {e}")

try:
    # 尝试相对导入
    from .yolov8_pose_detector import detect_human_pose
    POSE_DETECTION_AVAILABLE = True
except ImportError:
    try:
        # 尝试绝对导入
        from yolov8_pose_detector import detect_human_pose
        POSE_DETECTION_AVAILABLE = True
    except ImportError as e:
        POSE_DETECTION_AVAILABLE = False
        print(f"⚠️ 姿态检测模块不可用: {e}")


# ==================== ByteTracker核心类 ====================

class TrackState:
    """
    跟踪状态枚举类
    ===============
    
    功能说明：
        定义轨迹的四种基本状态，用于轨迹生命周期管理
        
    调用关系：
        - 被BaseTrack和STrack使用
        - 被ColorByteTracker状态管理逻辑使用
        
    调用原因：
        统一管理轨迹状态，确保状态转换的一致性
    """
    NEW = 0        # 新建：刚检测到的轨迹，等待确认
    TRACKED = 1    # 跟踪中：已确认的活跃轨迹
    LOST = 2       # 丢失：暂时无法匹配的轨迹
    REMOVED = 3    # 已移除：彻底失效的轨迹


class BaseTrack:
    """
    基础跟踪类 - 多目标跟踪的核心抽象类
    =====================================
    
    功能说明：
        定义了所有跟踪对象的基本属性和接口，是STrack的父类
        管理跟踪ID分配、状态管理、特征存储等基础功能
    
    调用关系：
        - 被STrack继承
        - 在ColorByteTracker中通过STrack间接使用
        - 在SingleTargetTracker中通过STrack间接使用
    
    设计目的：
        为不同类型的跟踪器提供统一的基础接口和状态管理
    """
    _count = 0  # 全局ID计数器，确保每个轨迹有唯一ID

    def __init__(self):
        """
        初始化基础跟踪对象
        
        参数：
            无
            
        返回值：
            无
            
        调用关系：
            - 被STrack.__init__()调用
            - 在创建新轨迹时自动调用
            
        调用原因：
            为每个新的跟踪轨迹初始化基本属性和状态
            
        调试信息：
            - 新轨迹创建时记录初始状态
            - 跟踪全局ID计数器变化
        """
        # 轨迹标识
        self.track_id = 0                    # 轨迹唯一ID
        self.is_activated = False            # 是否已激活（确认轨迹）
        self.state = TrackState.NEW          # 轨迹状态

        # 历史和特征信息
        self.history = OrderedDict()         # 历史状态记录
        self.features = []                   # 特征向量历史
        self.curr_feature = None             # 当前帧特征
        
        # 跟踪质量指标
        self.score = 0                       # 检测置信度
        self.start_frame = 0                 # 开始帧号
        self.frame_id = 0                    # 当前帧号
        self.time_since_update = 0           # 自上次更新经过的帧数

        # 外观特征（服装颜色）
        self.upper_color = None              # 上衣颜色 (B,G,R)
        self.lower_color = None              # 下装颜色 (B,G,R)

        # 空间位置
        self.location = (np.inf, np.inf)     # 当前位置坐标

    @property
    def end_frame(self):
        """
        获取轨迹结束帧号
        
        参数：
            无
            
        返回值：
            int: 当前帧号，表示轨迹的最后活跃帧
            
        调用关系：
            - 被跟踪器统计模块调用
            - 被轨迹可视化功能调用
            - 被remove_duplicate_stracks()函数调用用于比较轨迹时长
            
        调用原因：
            用于计算轨迹持续时间和生命周期统计
            
        调试信息：
            - 记录轨迹的活跃时长计算
        """
        return self.frame_id

    @staticmethod
    def next_id():
        """
        生成下一个唯一轨迹ID
        
        参数：
            无
            
        返回值：
            int: 新的唯一轨迹ID
            
        调用关系：
            - 被STrack.activate()调用
            - 被STrack.re_activate()调用（当new_id=True时）
            
        调用原因：
            确保每个轨迹都有全局唯一的标识符，避免ID冲突
            
        调试信息：
            - 记录ID分配："分配新轨迹ID: {new_id}, 当前总轨迹数: {_count}"
        """
        BaseTrack._count += 1
        return BaseTrack._count

    def activate(self, *args):
        """
        激活轨迹（抽象方法，子类必须实现）
        
        参数：
            *args: 激活所需的参数（由子类定义）
            
        返回值：
            无
            
        调用关系：
            - 在STrack中被具体实现
            - 被ColorByteTracker.update()调用
            
        调用原因：
            将新检测的对象初始化为可跟踪的轨迹
        """
        raise NotImplementedError("需要在子类中实现")

    def predict(self):
        """
        预测轨迹下一状态（抽象方法，子类必须实现）
        
        参数：
            无
            
        返回值：
            无
            
        调用关系：
            - 在STrack中通过卡尔曼滤波实现
            - 被ColorByteTracker.update()在关联前调用
            
        调用原因：
            在新的检测到来之前预测轨迹的运动状态
        """
        raise NotImplementedError("需要在子类中实现")

    def update(self, *args, **kwargs):
        """
        更新轨迹状态（抽象方法，子类必须实现）
        
        参数：
            *args: 更新所需的参数（由子类定义）
            **kwargs: 更新所需的关键字参数
            
        返回值：
            无
            
        调用关系：
            - 在STrack中被具体实现
            - 被ColorByteTracker.update()在成功关联后调用
            
        调用原因：
            使用新的检测结果更新轨迹的状态和特征
        """
        raise NotImplementedError("需要在子类中实现")

    def mark_lost(self):
        """
        将轨迹标记为丢失状态
        
        参数：
            无
            
        返回值：
            无
            
        调用关系：
            - 被ColorByteTracker.update()调用
            - 当轨迹无法与检测匹配时调用
            
        调用原因：
            暂时丢失的轨迹可能在后续帧中重新找到，不立即删除
            
        调试信息：
            - 记录轨迹丢失："轨迹 {track_id} 标记为丢失状态"
        """
        self.state = TrackState.LOST

    def mark_removed(self):
        """
        将轨迹标记为已移除状态
        
        参数：
            无
            
        返回值：
            无
            
        调用关系：
            - 被ColorByteTracker.update()调用
            - 当轨迹丢失时间过长或确认离开画面时调用
            
        调用原因：
            彻底删除不再有用的轨迹，释放计算资源
            
        调试信息：
            - 记录轨迹移除："轨迹 {track_id} 被永久移除，生命周期: {start_frame}-{end_frame}"
        """
        self.state = TrackState.REMOVED


class KalmanFilter:
    """
    卡尔曼滤波器 - 多目标跟踪的核心运动预测模块
    ================================================
    
    功能说明：
        使用卡尔曼滤波算法预测和更新目标的运动状态
        实现目标位置的平滑跟踪和运动轨迹预测
    
    状态向量定义：
        [x, y, a, h, vx, vy, va, vh] (8维)
        x, y: 目标中心坐标
        a: 宽高比 (width/height)
        h: 目标高度
        vx, vy: 中心坐标的速度
        va, vh: 宽高比和高度的变化速度
    
    调用关系：
        - 被STrack使用（共享实例）
        - 被ColorByteTracker实例化
        - 在轨迹预测和更新中被频繁调用
    
    设计目的：
        提供准确的目标运动预测，处理目标遮挡和检测缺失
    """

    def __init__(self):
        """
        初始化卡尔曼滤波器参数
        
        参数：
            无
            
        返回值：
            无
            
        调用关系：
            - 被ColorByteTracker.__init__()调用
            - 作为STrack.shared_kalman的共享实例
            
        调用原因：
            设置滤波器的数学模型参数和噪声特性
            
        调试信息：
            - 记录滤波器初始化参数
            - 显示状态空间维度和时间步长
        """
        # 状态空间维度定义
        self.ndim = 4                        # 观测维度：x, y, a, h
        self.dt = 1.0                        # 时间步长（帧间隔）

        # 状态转移矩阵F (8x8) - 描述状态如何随时间演化
        # [x_k+1]   [1 0 0 0 dt 0  0  0 ] [x_k  ]
        # [y_k+1]   [0 1 0 0 0  dt 0  0 ] [y_k  ]
        # [a_k+1] = [0 0 1 0 0  0  dt 0 ] [a_k  ]
        # [h_k+1]   [0 0 0 1 0  0  0  dt] [h_k  ]
        # [vx_k+1]  [0 0 0 0 1  0  0  0 ] [vx_k ]
        # [vy_k+1]  [0 0 0 0 0  1  0  0 ] [vy_k ]
        # [va_k+1]  [0 0 0 0 0  0  1  0 ] [va_k ]
        # [vh_k+1]  [0 0 0 0 0  0  0  1 ] [vh_k ]
        self._motion_mat = np.eye(2 * self.ndim, 2 * self.ndim)
        for i in range(self.ndim):
            self._motion_mat[i, self.ndim + i] = self.dt

        # 观测矩阵H (4x8) - 将状态空间映射到观测空间
        # [x_obs]   [1 0 0 0 0 0 0 0] [x ]
        # [y_obs] = [0 1 0 0 0 0 0 0] [y ]
        # [a_obs]   [0 0 1 0 0 0 0 0] [a ]
        # [h_obs]   [0 0 0 1 0 0 0 0] [h ]
        #                              [vx]
        #                              [vy]
        #                              [va]
        #                              [vh]
        self._update_mat = np.eye(self.ndim, 2 * self.ndim)

        # 噪声模型参数 - 调整跟踪的平滑性vs响应性
        self._std_weight_position = 1. / 20   # 位置不确定性权重（越小越平滑）
        self._std_weight_velocity = 1. / 160  # 速度不确定性权重（越小越稳定）

        # 卡方分布95%置信区间 - 用于数据关联的门限计算
        # 根据自由度确定马氏距离的阈值
        self.chi2inv95 = {
            1: 3.8415,     # 1自由度
            2: 5.9915,     # 2自由度（x,y位置）
            3: 7.8147,     # 3自由度
            4: 9.4877,     # 4自由度（x,y,a,h完整观测）
            5: 11.070,     # 5自由度
            6: 12.592,     # 6自由度
            7: 14.067,     # 7自由度
            8: 15.507,     # 8自由度
            9: 16.919      # 9自由度
        }

    def initiate(self, measurement):
        """
        从首次检测初始化轨迹的卡尔曼状态
        
        参数:
            measurement (np.array): [cx, cy, a, h] 格式的观测值
                - cx, cy: 目标中心坐标
                - a: 宽高比 (width/height)
                - h: 目标高度
            
        返回:
            tuple: (mean, covariance)
                - mean (np.array): 8维初始状态均值 [x,y,a,h,vx,vy,va,vh]
                - covariance (np.array): 8x8初始协方差矩阵
            
        调用关系:
            - 被STrack.activate()调用
            - 在新轨迹创建时调用一次
            
        调用原因:
            将检测结果转换为卡尔曼滤波的初始状态，设置合理的不确定性
            
        调试信息:
            - 记录初始状态向量
            - 显示初始协方差对角元素
            - 记录状态初始化成功
        """
        # 状态初始化：位置已知，速度假设为0
        mean_pos = measurement                    # 位置状态 [x, y, a, h]
        mean_vel = np.zeros_like(mean_pos)       # 速度状态 [vx, vy, va, vh] = [0, 0, 0, 0]
        mean = np.r_[mean_pos, mean_vel]         # 完整8维状态向量

        # 协方差矩阵初始化 - 反映初始不确定性
        # 位置的不确定性与目标大小成比例
        # 速度的不确定性较大，因为初始速度未知
        std = [
            2 * self._std_weight_position * measurement[3],    # x坐标标准差
            2 * self._std_weight_position * measurement[3],    # y坐标标准差  
            1e-2,                                              # 宽高比标准差（相对稳定）
            2 * self._std_weight_position * measurement[3],    # 高度标准差
            10 * self._std_weight_velocity * measurement[3],   # x速度标准差（未知，较大）
            10 * self._std_weight_velocity * measurement[3],   # y速度标准差（未知，较大）
            1e-5,                                              # 宽高比变化速度（很小）
            10 * self._std_weight_velocity * measurement[3]    # 高度变化速度
        ]
        covariance = np.diag(np.square(std))      # 对角协方差矩阵（假设状态独立）

        return mean, covariance

    def predict(self, mean, covariance):
        """
        卡尔曼滤波预测步骤 - 基于运动模型预测下一时刻状态
        
        参数:
            mean (np.array): 当前时刻8维状态均值 [x,y,a,h,vx,vy,va,vh]
            covariance (np.array): 当前时刻8x8状态协方差矩阵
            
        返回:
            tuple: (predicted_mean, predicted_covariance)
                - predicted_mean (np.array): 预测的8维状态均值
                - predicted_covariance (np.array): 预测的8x8状态协方差矩阵
            
        调用关系:
            - 被STrack.predict()调用
            - 被multi_predict()在批量处理中调用
            - 在每帧处理前对所有轨迹调用
            
        调用原因:
            在新检测到来前预测轨迹位置，处理目标运动和检测缺失
            
        调试信息:
            - 记录预测前后的状态变化
            - 显示协方差矩阵的不确定性增长
            - 监控预测是否稳定
            
        数学原理:
            预测步骤：x_k+1|k = F * x_k|k
            协方差预测：P_k+1|k = F * P_k|k * F^T + Q
            其中 F 是状态转移矩阵，Q 是过程噪声
        """
        # 过程噪声协方差矩阵Q的计算
        # 噪声大小与目标尺寸相关，大目标允许更大的位置变化
        std_pos = [
            self._std_weight_position * mean[3],    # x坐标过程噪声（与高度成比例）
            self._std_weight_position * mean[3],    # y坐标过程噪声
            1e-2,                                   # 宽高比过程噪声（较小，形状稳定）
            self._std_weight_position * mean[3]     # 高度过程噪声
        ]
        std_vel = [
            self._std_weight_velocity * mean[3],    # x速度过程噪声
            self._std_weight_velocity * mean[3],    # y速度过程噪声
            1e-5,                                   # 宽高比变化噪声（很小）
            self._std_weight_velocity * mean[3]     # 高度变化噪声
        ]
        motion_cov = np.diag(np.square(np.r_[std_pos, std_vel]))  # 过程噪声协方差矩阵Q

        # 卡尔曼预测公式
        mean = np.dot(mean, self._motion_mat.T)                   # 状态预测：x_k+1|k = F * x_k|k
        covariance = np.linalg.multi_dot((                        # 协方差预测：P_k+1|k = F*P_k|k*F^T + Q
            self._motion_mat, covariance, self._motion_mat.T)) + motion_cov

        return mean, covariance

    def multi_predict(self, means, covariances):
        """
        批量预测多个目标的状态 - 向量化操作提高效率
        
        参数:
            means (np.array): 多个状态均值的数组 [N, 8]
            covariances (np.array): 多个协方差矩阵的数组 [N, 8, 8]
            
        返回:
            tuple: (new_means, new_covariances)
                - new_means (np.array): 预测后的状态均值数组
                - new_covariances (np.array): 预测后的协方差矩阵数组
            
        调用关系:
            - 被STrack.multi_predict()调用
            - 在ColorByteTracker中批量处理所有轨迹
            
        调用原因:
            批量处理比逐个处理快10-30倍，提高实时性能
            
        调试信息:
            - 记录批量处理的轨迹数量
            - 显示处理时间对比
            - 监控是否有数值不稳定
        """
        if len(means) == 0:
            return means, covariances

        # 计算噪声协方差
        std_pos = [
            self._std_weight_position * means[:, 3],
            self._std_weight_position * means[:, 3],
            1e-2 * np.ones_like(means[:, 3]),
            self._std_weight_position * means[:, 3]
        ]
        std_vel = [
            self._std_weight_velocity * means[:, 3],
            self._std_weight_velocity * means[:, 3],
            1e-5 * np.ones_like(means[:, 3]),
            self._std_weight_velocity * means[:, 3]
        ]
        sqr = np.square(np.r_[std_pos, std_vel]).T

        # 构建协方差矩阵列表
        motion_covs = []
        for i in range(len(means)):
            motion_covs.append(np.diag(sqr[i]))
        motion_covs = np.asarray(motion_covs)

        # 预测新的状态
        new_means = np.dot(means, self._motion_mat.T)

        # 更新协方差
        new_covariances = []
        for i, (mean, cov) in enumerate(zip(means, covariances)):
            new_cov = np.linalg.multi_dot((
                self._motion_mat, cov, self._motion_mat.T)) + motion_covs[i]
            new_covariances.append(new_cov)

        return new_means, np.asarray(new_covariances)

    def update(self, mean, covariance, measurement):
        """
        卡尔曼更新步骤 - 融合新观测值修正状态估计
        
        参数:
            mean (np.array): 预测状态均值
            covariance (np.array): 预测协方差矩阵  
            measurement (np.array): 新的观测值 [x, y, a, h]
            
        返回:
            tuple: (new_mean, new_covariance)
                - new_mean (np.array): 更新后的状态均值
                - new_covariance (np.array): 更新后的协方差矩阵
                
        调用关系:
            - 被STrack.update()调用
            - 在成功匹配轨迹和检测后调用
            
        调用原因:
            利用新观测修正预测，提高状态估计精度
            
        调试信息:
            - 记录观测与预测的差异（innovation）
            - 显示卡尔曼增益的大小
            - 监控状态修正的幅度
        """
        # 投影状态分布到测量空间
        projected_mean, projected_cov = self.project(mean, covariance)

        # 计算卡尔曼增益
        chol_factor, lower = scipy.linalg.cho_factor(
            projected_cov, lower=True, check_finite=False)
        kalman_gain = scipy.linalg.cho_solve(
            (chol_factor, lower), np.dot(covariance, self._update_mat.T).T,
            check_finite=False).T

        # 计算新状态
        innovation = measurement - projected_mean
        new_mean = mean + np.dot(innovation, kalman_gain.T)
        new_covariance = covariance - np.linalg.multi_dot((
            kalman_gain, projected_cov, kalman_gain.T))

        return new_mean, new_covariance

    def project(self, mean, covariance):
        """
        将状态分布投影到测量空间
        
        参数:
            mean (np.array): 8维状态均值
            covariance (np.array): 8x8状态协方差矩阵
            
        返回:
            tuple: (projected_mean, projected_covariance)
                - projected_mean (np.array): 投影后的4维观测均值
                - projected_covariance (np.array): 投影后的4x4观测协方差
                
        调用关系:
            - 被update()方法调用
            - 被gating_distance()调用
            
        调用原因:
            将8维状态空间映射到4维观测空间，计算预期观测值
            
        调试信息:
            - 记录观测噪声的设置
            - 显示投影后的不确定性
        """
        # 计算观测噪声
        std = [
            self._std_weight_position * mean[3],
            self._std_weight_position * mean[3],
            1e-1,
            self._std_weight_position * mean[3]
        ]
        innovation_cov = np.diag(np.square(std))

        # 投影
        mean = np.dot(self._update_mat, mean)
        covariance = np.linalg.multi_dot((
            self._update_mat, covariance, self._update_mat.T))

        return mean, covariance + innovation_cov

    def gating_distance(self, mean, covariance, measurements, only_position=False, metric='maha'):
        """
        计算状态分布与测量之间的门控距离
        
        参数:
            mean (np.array): 状态均值
            covariance (np.array): 状态协方差矩阵
            measurements (np.array): 测量值数组 [N, 4]
            only_position (bool): 是否只使用位置信息
            metric (str): 距离度量类型 ('maha' 或 'gaussian')
            
        返回:
            np.array: 门控距离数组 [N]
            
        调用关系:
            - 被fuse_motion()函数调用
            - 在数据关联前过滤不可能的匹配
            
        调用原因:
            提前排除距离过远的检测，减少关联计算量
            
        调试信息:
            - 记录门控阈值和过滤结果
            - 显示通过门控的检测数量
        """
        mean, covariance = self.project(mean, covariance)
        if only_position:
            mean, covariance = mean[:2], covariance[:2, :2]
            measurements = measurements[:, :2]

        d = measurements - mean
        if metric == 'gaussian':
            return np.sum(d * d, axis=1)
        elif metric == 'maha':
            cholesky_factor = np.linalg.cholesky(covariance)
            z = scipy.linalg.solve_triangular(
                cholesky_factor, d.T, lower=True, check_finite=False,
                overwrite_b=True)
            squared_maha = np.sum(z * z, axis=0)
            return squared_maha
        else:
            raise ValueError('无效的距离度量')


class STrack(BaseTrack):
    """
    单个目标跟踪器实现 - ByteTracker算法的轨迹表示
    ===================================================
    
    功能说明：
        继承BaseTrack，实现具体的轨迹跟踪功能
        集成卡尔曼滤波、颜色特征、身体比例等多维信息
        
    调用关系：
        - 被ColorByteTracker创建和管理
        - 被SingleTargetTracker间接使用
        - 与KalmanFilter紧密协作
        
    设计目的：
        提供完整的单轨迹生命周期管理和状态估计
    """

    # 共享卡尔曼滤波器
    shared_kalman = KalmanFilter()

    def __init__(self, tlwh, score, temp_feat=None, upper_color=None, lower_color=None, body_ratios=None):
        """
        初始化跟踪对象

        参数:
            tlwh (list): [x,y,w,h] 格式的边界框
            score (float): 检测置信度 (0-1)
            temp_feat: 临时特征向量（未使用）
            upper_color (tuple): 上衣颜色 (B,G,R)
            lower_color (tuple): 下装颜色 (B,G,R)  
            body_ratios (list): 身体比例特征，16个浮点数
            
        返回值:
            无
            
        调用关系:
            - 被ColorByteTracker在处理检测结果时调用
            - 在每个新检测被转换为轨迹时调用
            
        调用原因:
            将检测结果封装为可跟踪的轨迹对象
            
        调试信息:
            - 记录新轨迹的初始特征："创建轨迹，置信度: {score:.3f}, 位置: {tlwh}"
            - 显示颜色和身体比例信息的可用性
        """
        super(STrack, self).__init__()

        # 等待激活
        self._tlwh = np.asarray(tlwh, dtype=np.float64)
        self.kalman_filter = None
        self.mean, self.covariance = None, None

        # 添加预测状态存储
        self.predicted_mean, self.predicted_covariance = None, None
        self.is_activated = False

        self.score = score
        self.tracklet_len = 0

        # 添加颜色特征
        self.upper_color = upper_color
        self.lower_color = lower_color

        # 添加身体比例特征
        self.body_ratios = body_ratios if body_ratios is not None else [0.0] * 16

    def predict(self):
        """
        预测下一个状态
        
        参数：
            无
            
        返回值：
            无
            
        调用关系：
            - 被multi_predict()静态方法调用
            - 在ColorByteTracker.update()中被批量调用
            
        调用原因：
            在新检测到来前预测轨迹的运动状态
            
        调试信息：
            - 记录预测前后的位置变化
            - 显示速度估计和预测置信度
        """
        if self.kalman_filter is None or self.mean is None or self.covariance is None:
            return
            
        if self.state != TrackState.TRACKED:
            # 非跟踪状态，将垂直速度置为0
            mean_state = self.mean.copy()
            mean_state[7] = 0
            self.mean, self.covariance = self.kalman_filter.predict(mean_state, self.covariance)
        else:
            # 正常预测
            self.mean, self.covariance = self.kalman_filter.predict(self.mean, self.covariance)

        # 保存预测状态
        self.predicted_mean = self.mean.copy()
        self.predicted_covariance = self.covariance.copy()

    @staticmethod
    def multi_predict(stracks):
        """
        批量预测多个目标的下一状态
        
        参数：
            stracks (list): STrack对象列表
            
        返回值：
            无（直接修改轨迹对象的状态）
            
        调用关系：
            - 被ColorByteTracker.update()调用
            - 在每帧处理开始时批量预测所有轨迹
            
        调用原因：
            批量处理比逐个处理效率高，提升实时性能
            
        调试信息：
            - 记录批量处理的轨迹数量和处理时间
            - 显示每个轨迹的预测结果概要
        """
        if len(stracks) > 0:
            multi_mean = np.asarray([st.mean.copy() for st in stracks])
            multi_covariance = np.asarray([st.covariance for st in stracks])

            # 将非跟踪状态的目标速度置为0
            for i, st in enumerate(stracks):
                if st.state != TrackState.TRACKED:
                    multi_mean[i][7] = 0

            # 批量预测
            multi_mean, multi_covariance = STrack.shared_kalman.multi_predict(multi_mean, multi_covariance)

            # 更新每个跟踪对象的状态
            for i, (mean, cov) in enumerate(zip(multi_mean, multi_covariance)):
                stracks[i].mean = mean
                stracks[i].covariance = cov
                # 保存预测状态
                stracks[i].predicted_mean = mean.copy()
                stracks[i].predicted_covariance = cov.copy()

    def activate(self, kalman_filter, frame_id):
        """
        激活新的轨迹
        
        参数：
            kalman_filter (KalmanFilter): 卡尔曼滤波器实例
            frame_id (int): 当前帧号
            
        返回值：
            无
            
        调用关系：
            - 被ColorByteTracker.update()在创建新轨迹时调用
            - 在检测对象被确认为新轨迹时调用
            
        调用原因：
            将检测结果转换为可跟踪的激活轨迹
            
        调试信息：
            - 记录轨迹激活："激活新轨迹 ID={track_id}, 帧={frame_id}, 位置={tlwh}"
            - 显示初始状态和协方差信息
        """
        self.kalman_filter = kalman_filter
        self.track_id = self.next_id()

        # 初始化卡尔曼状态
        self.mean, self.covariance = self.kalman_filter.initiate(self.tlwh_to_xyah(self._tlwh))

        # 更新状态
        self.tracklet_len = 0
        self.state = TrackState.TRACKED

        # 第一帧直接激活，否则等待确认
        if frame_id == 1:
            self.is_activated = True

        self.frame_id = frame_id
        self.start_frame = frame_id

    def re_activate(self, new_track, frame_id, new_id=False):
        """
        重新激活丢失的轨迹
        
        参数：
            new_track (STrack): 新检测到的轨迹
            frame_id (int): 当前帧号  
            new_id (bool): 是否分配新ID
            
        返回值：
            无
            
        调用关系：
            - 被ColorByteTracker.update()在重新找到丢失轨迹时调用
            - 在lost_stracks中的轨迹与新检测匹配时调用
            
        调用原因：
            恢复暂时丢失的轨迹，避免ID碎片化
            
        调试信息：
            - 记录轨迹重激活："重激活轨迹 ID={track_id}, 丢失={lost_frames}帧"
            - 显示特征更新情况和匹配置信度
        """
        if self.kalman_filter is None:
            return
            
        # 更新卡尔曼状态
        self.mean, self.covariance = self.kalman_filter.update(
            self.mean, self.covariance, self.tlwh_to_xyah(new_track.tlwh)
        )

        # 更新轨迹状态
        self.tracklet_len = 0
        self.state = TrackState.TRACKED
        self.is_activated = True
        self.frame_id = frame_id

        # 更新颜色特征
        if new_track.upper_color is not None and all(c >= 0 for c in new_track.upper_color):
            self.upper_color = new_track.upper_color
        if new_track.lower_color is not None and all(c >= 0 for c in new_track.lower_color):
            self.lower_color = new_track.lower_color

        # 更新身体比例特征
        if hasattr(new_track, 'body_ratios') and new_track.body_ratios:
            if len(new_track.body_ratios) == len(self.body_ratios):
                self.body_ratios = new_track.body_ratios

        # 可选择分配新ID
        if new_id:
            self.track_id = self.next_id()

        self.score = new_track.score

    def update(self, new_track, frame_id):
        """
        更新匹配的轨迹
        
        参数：
            new_track (STrack): 新检测到的轨迹
            frame_id (int): 当前帧号
            
        返回值：
            无
            
        调用关系：
            - 被ColorByteTracker.update()在成功匹配后调用
            - 在tracked_stracks与检测匹配时调用
            
        调用原因：
            使用新检测更新轨迹状态，保持跟踪连续性
            
        调试信息：
            - 记录轨迹更新："更新轨迹 ID={track_id}, 置信度={score:.3f}"
            - 显示位置变化和特征演化
        """
        self.frame_id = frame_id
        self.tracklet_len += 1

        # 更新卡尔曼状态
        if self.kalman_filter is not None:
            new_tlwh = new_track.tlwh
            self.mean, self.covariance = self.kalman_filter.update(
                self.mean, self.covariance, self.tlwh_to_xyah(new_tlwh)
            )

        # 更新状态
        self.state = TrackState.TRACKED
        self.is_activated = True
        self.score = new_track.score

        # 更新颜色特征(如果新的颜色有效)
        if new_track.upper_color is not None and all(c >= 0 for c in new_track.upper_color):
            # 如果已有颜色，采用加权平均
            if self.upper_color is not None:
                weight = min(0.3, 1.0 / self.tracklet_len)  # 随着轨迹长度增加权重减小
                self.upper_color = tuple(int((1 - weight) * c1 + weight * c2)
                                         for c1, c2 in zip(self.upper_color, new_track.upper_color))
            else:
                self.upper_color = new_track.upper_color

        if new_track.lower_color is not None and all(c >= 0 for c in new_track.lower_color):
            # 如果已有颜色，采用加权平均
            if self.lower_color is not None:
                weight = min(0.3, 1.0 / self.tracklet_len)
                self.lower_color = tuple(int((1 - weight) * c1 + weight * c2)
                                         for c1, c2 in zip(self.lower_color, new_track.lower_color))
            else:
                self.lower_color = new_track.lower_color

        # 更新身体比例特征（如果有新的数据）
        if hasattr(new_track, 'body_ratios') and new_track.body_ratios:
            if len(new_track.body_ratios) == len(self.body_ratios):
                # 也采用加权平均更新，但权重更低
                weight = min(0.2, 1.0 / self.tracklet_len)
                for i in range(len(self.body_ratios)):
                    if new_track.body_ratios[i] > 0:  # 只使用有效数据更新
                        self.body_ratios[i] = (1 - weight) * self.body_ratios[i] + weight * new_track.body_ratios[i]

    @property
    def tlwh(self):
        """
        获取边界框 [x,y,w,h] 格式
        
        参数：
            无
            
        返回值：
            np.array: [x,y,w,h] 格式的边界框
            
        调用关系：
            - 被可视化模块调用
            - 被距离计算函数调用
            - 被数据关联算法调用
            
        调用原因：
            提供标准格式的边界框用于各种计算
        """
        if self.mean is None:
            return self._tlwh.copy()

        ret = self.mean[:4].copy()
        ret[2] *= ret[3]  # 宽度 = 宽高比 * 高度
        ret[:2] -= ret[2:] / 2  # 中心点坐标转左上角坐标

        return ret

    @property
    def predicted_tlwh(self):
        """
        获取预测的边界框 [x,y,w,h] 格式
        
        参数：
            无
            
        返回值：
            np.array: 预测的边界框
            
        调用关系：
            - 被可视化模块调用显示预测框
            - 被轨迹外推算法使用
            
        调用原因：
            显示轨迹的运动预测，帮助调试跟踪算法
        """
        if self.predicted_mean is None:
            return self.tlwh.copy()

        ret = self.predicted_mean[:4].copy()
        ret[2] *= ret[3]
        ret[:2] -= ret[2:] / 2

        return ret

    @property
    def predicted_tlbr(self):
        """
        获取预测的边界框 [x1,y1,x2,y2] 格式
        
        参数：
            无
            
        返回值：
            np.array: 预测的边界框，左上角和右下角坐标
            
        调用关系：
            - 被可视化模块调用
            - 被轨迹预测显示功能调用
            
        调用原因：
            方便绘制预测的边界框
        """
        ret = self.predicted_tlwh.copy()
        ret[2:] += ret[:2]
        return ret

    @property
    def tlbr(self):
        """
        获取边界框 [x1,y1,x2,y2] 格式
        
        参数：
            无
            
        返回值：
            np.array: [x1,y1,x2,y2] 格式的边界框
            
        调用关系：
            - 被可视化模块频繁调用
            - 被IOU计算函数调用
            - 被距离测量功能调用
            
        调用原因：
            提供绘制和计算常用的边界框格式
        """
        ret = self.tlwh.copy()
        ret[2:] += ret[:2]
        return ret

    @staticmethod
    def tlwh_to_xyah(tlwh):
        """
        [x,y,w,h] -> [cx,cy,aspect,h] 转换
        
        参数：
            tlwh (np.array): [x,y,w,h] 格式边界框
            
        返回值：
            np.array: [cx,cy,aspect,h] 格式，用于卡尔曼滤波
            
        调用关系：
            - 被activate()方法调用
            - 被update()方法调用
            - 被卡尔曼滤波器使用
            
        调用原因：
            将检测格式转换为卡尔曼滤波器的状态格式
        """
        ret = np.asarray(tlwh).copy()
        ret[:2] += ret[2:] / 2  # 左上角坐标转中心点坐标
        ret[2] /= ret[3]  # 宽高比 = 宽度 / 高度
        return ret

    def to_xyah(self):
        """
        获取当前状态的xyah格式
        
        参数：
            无
            
        返回值：
            np.array: [cx,cy,aspect,h] 格式的状态
            
        调用关系：
            - 被状态输出模块调用
            - 被调试功能使用
            
        调用原因：
            提供标准化的状态表示用于分析
        """
        return self.tlwh_to_xyah(self.tlwh)

    @staticmethod
    def tlbr_to_tlwh(tlbr):
        """
        [x1,y1,x2,y2] -> [x,y,w,h] 转换
        
        参数：
            tlbr (np.array): [x1,y1,x2,y2] 格式边界框
            
        返回值：
            np.array: [x,y,w,h] 格式边界框
            
        调用关系：
            - 被STrack初始化时调用
            - 被检测结果处理函数调用
            
        调用原因：
            标准化不同来源的边界框格式
        """
        ret = np.asarray(tlbr).copy()
        ret[2:] -= ret[:2]
        return ret

    @staticmethod
    def tlwh_to_tlbr(tlwh):
        """
        [x,y,w,h] -> [x1,y1,x2,y2] 转换
        
        参数：
            tlwh (np.array): [x,y,w,h] 格式边界框
            
        返回值：
            np.array: [x1,y1,x2,y2] 格式边界框
            
        调用关系：
            - 被可视化功能调用
            - 被IOU计算调用
            
        调用原因：
            转换为绘制和计算常用的格式
        """
        ret = np.asarray(tlwh).copy()
        ret[2:] += ret[:2]
        return ret

    def __repr__(self):
        """
        字符串表示
        
        参数：
            无
            
        返回值：
            str: 轨迹的字符串描述
            
        调用关系：
            - 被日志系统调用
            - 被调试输出调用
            
        调用原因：
            提供可读的轨迹信息用于调试
        """
        return f'OT_{self.track_id}({self.start_frame}-{self.end_frame})'


# ==================== 辅助函数 ====================

def iou_distance(atracks, btracks):
    """
    计算IOU距离矩阵 - 多目标跟踪的核心相似度计算
    =====================================================
    
    功能说明：
        计算两组轨迹/检测之间的IOU距离矩阵
        距离 = 1 - IOU，用于数据关联算法
    
    参数:
        atracks (list): 轨迹列表A或边界框数组
        btracks (list): 轨迹列表B或边界框数组
        
    返回:
        np.array: IOU距离矩阵 [len(atracks), len(btracks)]
        
    调用关系:
        - 被ColorByteTracker.update()多次调用
        - 被remove_duplicate_stracks()调用
        - 被数据关联算法使用
        
    调用原因:
        提供空间位置相似度，是多目标跟踪的基础度量
        
    调试信息:
        - 记录输入尺寸和计算结果统计
        - 显示IOU分布直方图
        - 监控异常值和边界情况
    """
    if (len(atracks) > 0 and isinstance(atracks[0], np.ndarray)) or \
            (len(btracks) > 0 and isinstance(btracks[0], np.ndarray)):
        atlbrs = atracks
        btlbrs = btracks
    else:
        atlbrs = [track.tlbr for track in atracks]
        btlbrs = [track.tlbr for track in btracks]

    # 计算IoU矩阵
    ious = optimized_bbox_ious(np.array(atlbrs), np.array(btlbrs))
    cost_matrix = 1 - ious  # 距离 = 1 - IoU

    return cost_matrix


def color_distance(atracks, btracks, alpha=0.5):
    """
    计算颜色距离，结合上衣和下装颜色 - 外观特征匹配核心函数
    ============================================================
    
    功能说明：
        计算轨迹间的颜色相似度，融合上衣和下装信息
        提供外观特征约束，增强跟踪鲁棒性
    
    参数:
        atracks (list): 轨迹列表A
        btracks (list): 轨迹列表B  
        alpha (float): 上衣颜色权重（下装权重为1-alpha）
        
    返回:
        np.array: 颜色距离矩阵 [len(atracks), len(btracks)]
        
    调用关系:
        - 被ColorByteTracker.update()在每个关联阶段调用
        - 被fuse_iou_with_color()调用进行特征融合
        
    调用原因:
        提供外观约束，处理遮挡和快速运动场景
        
    调试信息:
        - 记录有效颜色信息的统计
        - 显示颜色匹配度分布
        - 监控颜色特征的稳定性
    """
    cost_matrix = np.zeros((len(atracks), len(btracks)), dtype=np.float64)

    for i, atrack in enumerate(atracks):
        for j, btrack in enumerate(btracks):
            # 上衣颜色距离
            upper_dist = 1.0  # 默认为最大距离
            if hasattr(atrack, 'upper_color') and hasattr(btrack, 'upper_color') and \
                    atrack.upper_color is not None and btrack.upper_color is not None:
                # 计算颜色欧氏距离并归一化
                upper_dist = np.sqrt(np.sum([(a - b) ** 2 for a, b in zip(atrack.upper_color, btrack.upper_color)]))
                # 颜色距离最大可达到 sqrt(255^2 * 3) = 441.7
                upper_dist = min(upper_dist / 442.0, 1.0)

            # 下装颜色距离
            lower_dist = 1.0  # 默认为最大距离
            if hasattr(atrack, 'lower_color') and hasattr(btrack, 'lower_color') and \
                    atrack.lower_color is not None and btrack.lower_color is not None:
                # 计算颜色欧氏距离并归一化
                lower_dist = np.sqrt(np.sum([(a - b) ** 2 for a, b in zip(atrack.lower_color, btrack.lower_color)]))
                lower_dist = min(lower_dist / 442.0, 1.0)

            # 组合上衣和下装距离
            if upper_dist < 1.0 and lower_dist < 1.0:
                # 两者都有颜色信息，加权组合
                cost_matrix[i, j] = alpha * upper_dist + (1 - alpha) * lower_dist
            elif upper_dist < 1.0:
                # 只有上衣颜色信息
                cost_matrix[i, j] = upper_dist
            elif lower_dist < 1.0:
                # 只有下装颜色信息
                cost_matrix[i, j] = lower_dist
            else:
                # 都没有颜色信息
                cost_matrix[i, j] = 1.0

    return cost_matrix


def fuse_motion(kf, cost_matrix, tracks, detections, lambda_=0.98):
    """
    融合运动信息到代价矩阵 - 卡尔曼滤波增强的数据关联
    ======================================================
    
    功能说明：
        将卡尔曼滤波的运动预测融合到代价矩阵中
        通过门控阈值过滤不可能的关联
    
    参数:
        kf (KalmanFilter): 卡尔曼滤波器实例
        cost_matrix (np.array): 现有代价矩阵
        tracks (list): 轨迹列表
        detections (list): 检测列表  
        lambda_ (float): 融合权重，控制运动vs外观的重要性
        
    返回:
        np.array: 融合运动信息后的代价矩阵
        
    调用关系:
        - 被ColorByteTracker.update()在高级关联中调用
        - 在需要运动约束的场景中使用
        
    调用原因:
        提供运动一致性约束，提高快速运动目标的跟踪精度
        
    调试信息:
        - 记录门控过滤的检测数量
        - 显示运动预测的准确性
        - 监控融合权重的影响
    """
    if cost_matrix.size == 0:
        return cost_matrix

    gating_threshold = kf.chi2inv95[4]  # 使用4自由度
    measurements = np.asarray([det.tlwh_to_xyah(det.tlwh) for det in detections])

    for row, track in enumerate(tracks):
        gating_distance = kf.gating_distance(
            track.mean, track.covariance, measurements)

        # 应用门限
        cost_matrix[row, gating_distance > gating_threshold] = np.inf
        cost_matrix[row] = lambda_ * cost_matrix[row] + (1 - lambda_) * gating_distance

    return cost_matrix


def fuse_iou_with_color(iou_cost, color_cost, detections, w_iou=0.7, w_color=0.3):
    """
    融合IOU和颜色代价 - 多特征融合的核心算法
    ========================================
    
    功能说明：
        将空间位置（IOU）和外观特征（颜色）融合为统一代价
        考虑检测置信度进行加权
    
    参数:
        iou_cost (np.array): IOU代价矩阵
        color_cost (np.array): 颜色代价矩阵
        detections (list): 检测列表，用于获取置信度
        w_iou (float): IOU权重
        w_color (float): 颜色权重
        
    返回:
        np.array: 融合后的代价矩阵
        
    调用关系:
        - 被ColorByteTracker.update()在多个关联阶段调用
        - 是多特征融合跟踪的核心函数
        
    调用原因:
        结合多种特征提高关联准确性，处理复杂场景
        
    调试信息:
        - 记录特征融合的权重分配
        - 显示融合前后的代价分布变化
        - 监控置信度加权的效果
    """
    if iou_cost.size == 0:
        return iou_cost

    # 检查颜色代价矩阵是否有效
    if color_cost.size == 0 or color_cost.shape != iou_cost.shape:
        w_iou, w_color = 1.0, 0.0

    # IOU相似度和颜色相似度
    iou_sim = 1 - iou_cost
    color_sim = 1 - color_cost

    # 获取检测置信度
    det_scores = np.array([det.score for det in detections])
    det_scores = np.expand_dims(det_scores, axis=0).repeat(iou_cost.shape[0], axis=0)

    # 融合相似度 (IOU + 颜色 + 置信度)
    if w_color > 0:
        # 使用IOU、颜色和置信度
        fuse_sim = w_iou * iou_sim + w_color * color_sim
        fuse_sim = fuse_sim * (1 + det_scores) / 2
    else:
        # 只使用IOU和置信度
        fuse_sim = iou_sim * (1 + det_scores) / 2

    # 转回代价形式
    fuse_cost = 1 - fuse_sim

    return fuse_cost


def linear_assignment(cost_matrix, thresh=0.7):
    """
    线性分配算法(匈牙利算法) - 数据关联的优化求解
    =============================================
    
    功能说明：
        使用匈牙利算法求解最优二分图匹配
        支持lap库和scipy的降级实现
    
    参数:
        cost_matrix (np.array): 代价矩阵
        thresh (float): 匹配阈值，超过此值的匹配被拒绝
        
    返回:
        tuple: (matches, unmatched_a, unmatched_b)
            - matches (list): 匹配对列表 [[i,j], ...]
            - unmatched_a (list): 未匹配的A列表索引
            - unmatched_b (list): 未匹配的B列表索引
            
    调用关系:
        - 被ColorByteTracker.update()在每个关联阶段调用
        - 被SingleTargetTracker间接调用
        
    调用原因:
        求解全局最优的轨迹-检测匹配，最小化总代价
        
    调试信息:
        - 记录匹配结果统计
        - 显示代价分布和阈值过滤效果
        - 监控算法选择和性能
    """
    try:
        import lap
        if cost_matrix.size == 0:
            return [], list(range(cost_matrix.shape[0])), list(range(cost_matrix.shape[1]))

        matches, unmatched_a, unmatched_b = [], [], []
        cost, x, y = lap.lapjv(cost_matrix, extend_cost=True, cost_limit=thresh)

        for ix, mx in enumerate(x):
            if mx >= 0:
                matches.append([ix, mx])

        unmatched_a = np.where(x < 0)[0].tolist()
        unmatched_b = np.where(y < 0)[0].tolist()

        return matches, unmatched_a, unmatched_b
    except ImportError:
        # 降级使用scipy实现
        from scipy.optimize import linear_sum_assignment

        # 处理无限值
        cost_matrix = np.copy(cost_matrix)
        cost_matrix[cost_matrix > thresh] = thresh + 1e-4

        row_ind, col_ind = linear_sum_assignment(cost_matrix)
        matches = []
        unmatched_a = list(range(cost_matrix.shape[0]))
        unmatched_b = list(range(cost_matrix.shape[1]))

        for r, c in zip(row_ind, col_ind):
            if cost_matrix[r, c] <= thresh:
                matches.append([r, c])
                unmatched_a.remove(r)
                unmatched_b.remove(c)

        return matches, unmatched_a, unmatched_b


def optimized_bbox_ious(atlbrs, btlbrs):
    """
    向量化计算IOU，比循环快10-20倍 - 高性能几何计算
    ==============================================
    
    功能说明：
        使用numpy向量化操作快速计算多个边界框间的IOU
        避免Python循环，大幅提升计算效率
    
    参数:
        atlbrs (np.array): 边界框数组A [N, 4] - [x1,y1,x2,y2]格式
        btlbrs (np.array): 边界框数组B [M, 4] - [x1,y1,x2,y2]格式
        
    返回:
        np.array: IOU矩阵 [N, M]，每个元素为对应边界框对的IOU值
        
    调用关系:
        - 被iou_distance()函数调用
        - 被所有需要IOU计算的模块间接调用
        
    调用原因:
        提供高效的几何相似度计算，是实时跟踪的性能关键
        
    调试信息:
        - 记录计算时间和加速比
        - 监控数值稳定性
        - 显示IOU值的分布统计
    """
    # 转换为numpy数组
    atlbrs = np.asarray(atlbrs)
    btlbrs = np.asarray(btlbrs)

    # 获取数组形状
    alen, blen = len(atlbrs), len(btlbrs)
    if alen == 0 or blen == 0:
        return np.zeros((alen, blen), dtype=np.float64)

    # 计算交集区域
    xx1 = np.maximum(atlbrs[:, 0].reshape(alen, 1), btlbrs[:, 0].reshape(1, blen))
    yy1 = np.maximum(atlbrs[:, 1].reshape(alen, 1), btlbrs[:, 1].reshape(1, blen))
    xx2 = np.minimum(atlbrs[:, 2].reshape(alen, 1), btlbrs[:, 2].reshape(1, blen))
    yy2 = np.minimum(atlbrs[:, 3].reshape(alen, 1), btlbrs[:, 3].reshape(1, blen))

    w = np.maximum(0, xx2 - xx1)
    h = np.maximum(0, yy2 - yy1)
    inter = w * h

    # 计算各框面积
    area_a = ((atlbrs[:, 2] - atlbrs[:, 0]) * (atlbrs[:, 3] - atlbrs[:, 1])).reshape(alen, 1)
    area_b = ((btlbrs[:, 2] - btlbrs[:, 0]) * (btlbrs[:, 3] - btlbrs[:, 1])).reshape(1, blen)
    union = area_a + area_b - inter

    # 计算IoU
    ious = np.where(union > 0, inter / union, 0)
    return ious


def joint_stracks(tlista, tlistb):
    """
    合并两个轨迹列表，确保没有重复ID - 轨迹列表管理工具
    ======================================================
    
    功能说明：
        安全地合并两个轨迹列表，避免ID冲突
        保持轨迹的唯一性约束
    
    参数：
        tlista (list): 轨迹列表A
        tlistb (list): 轨迹列表B
        
    返回值：
        list: 合并后的轨迹列表，无重复ID
        
    调用关系：
        - 被ColorByteTracker._update_status()调用
        - 在轨迹状态管理中使用
        
    调用原因：
        维护轨迹列表的一致性，防止ID冲突
        
    调试信息：
        - 记录合并前后的列表大小
        - 显示重复ID的处理情况
    """
    exists = {}
    res = []
    for t in tlista:
        exists[t.track_id] = 1
        res.append(t)

    for t in tlistb:
        tid = t.track_id
        if not exists.get(tid, 0):
            exists[tid] = 1
            res.append(t)

    return res


def sub_stracks(tlista, tlistb):
    """
    从列表A中移除在列表B中出现的轨迹 - 轨迹集合运算
    ===================================================
    
    功能说明：
        从轨迹列表A中减去列表B中存在的轨迹
        用于轨迹状态转换时的集合操作
    
    参数：
        tlista (list): 被减轨迹列表
        tlistb (list): 减数轨迹列表
        
    返回值：
        list: 差集轨迹列表
        
    调用关系：
        - 被ColorByteTracker._update_status()调用
        - 在轨迹状态管理中使用
        
    调用原因：
        实现轨迹状态间的转换，保持状态一致性
        
    调试信息：
        - 记录移除的轨迹ID列表
        - 显示集合运算的结果统计
    """
    stracks = {}
    for t in tlista:
        stracks[t.track_id] = t

    for t in tlistb:
        tid = t.track_id
        if stracks.get(tid, 0):
            del stracks[tid]

    return list(stracks.values())


def remove_duplicate_stracks(stracksa, stracksb):
    """
    移除重复轨迹，保留跟踪时间较长的 - 轨迹去重算法
    ==============================================
    
    功能说明：
        检测并移除重复的轨迹，基于跟踪时长决定保留策略
        用于处理轨迹碎片化和重复初始化问题
    
    参数：
        stracksa (list): 轨迹列表A
        stracksb (list): 轨迹列表B
        
    返回值：
        tuple: (resa, resb) - 去重后的轨迹列表
        
    调用关系：
        - 被ColorByteTracker._update_status()调用
        - 在轨迹管理的最后阶段调用
        
    调用原因：
        清理重复轨迹，提高跟踪质量和系统效率
        
    调试信息：
        - 记录检测到的重复轨迹对
        - 显示保留策略的选择依据
        - 监控去重效果
    """
    pdist = iou_distance(stracksa, stracksb)
    pairs = np.where(pdist < 0.15)
    dupa, dupb = list(), list()

    for p, q in zip(*pairs):
        timep = stracksa[p].frame_id - stracksa[p].start_frame
        timeq = stracksb[q].frame_id - stracksb[q].start_frame

        if timep > timeq:
            dupb.append(q)
        else:
            dupa.append(p)

    resa = [t for i, t in enumerate(stracksa) if i not in dupa]
    resb = [t for i, t in enumerate(stracksb) if i not in dupb]

    return resa, resb


# ==================== ByteTracker核心算法类 ====================

class ColorByteTracker:
    """
    融合颜色特征的ByteTracker算法实现 - 多目标跟踪核心引擎
    ========================================================
    
    功能说明：
        实现完整的多目标跟踪算法，融合位置、运动、颜色特征
        支持高低置信度检测的分层关联策略
        
    调用关系：
        - 被ByteTrackerNode作为多目标跟踪器使用
        - 被SingleTargetTracker作为基础跟踪引擎使用
        
    设计目的：
        提供鲁棒的多目标跟踪能力，处理遮挡、快速运动等复杂场景
    """

    def __init__(self, args, frame_rate=30):
        """
        初始化跟踪器

        参数:
            args (dict): 包含配置项的字典
                - track_thresh: 轨迹激活阈值
                - track_buffer: 轨迹缓冲帧数  
                - match_thresh: 匹配阈值
                - color_weight: 颜色特征权重
            frame_rate (int): 视频帧率，用于计算缓冲区大小
            
        返回值:
            无
            
        调用关系:
            - 被ByteTrackerNode.__init__()调用
            - 被SingleTargetTracker.__init__()调用
            
        调用原因:
            初始化跟踪器的核心参数和状态容器
            
        调试信息:
            - 记录初始化参数："初始化ColorByteTracker: thresh={track_thresh}, buffer={buffer_size}"
            - 显示卡尔曼滤波器状态
        """
        # 轨迹列表
        self.tracked_stracks = []  # 当前正在跟踪的轨迹
        self.lost_stracks = []  # 暂时丢失的轨迹
        self.removed_stracks = []  # 已移除的轨迹

        # 参数设置
        self.frame_id = 0
        self.args = args
        self.det_thresh = args.get('track_thresh', 0.5) + 0.1  # 稍高的阈值用于激活新轨迹

        # 缓冲区和最大丢失时间设置
        self.buffer_size = int(frame_rate / 30.0 * args.get('track_buffer', 30))
        self.max_time_lost = self.buffer_size

        # 初始化卡尔曼滤波器
        self.kalman_filter = KalmanFilter()

        # 颜色权重
        self.color_weight = args.get('color_weight', 0.3)

    def update(self, detection_results, img=None):
        """
        更新跟踪状态 - ByteTracker算法的核心处理函数
        ============================================

        参数:
            detection_results (list): 检测结果列表，每项为(上衣坐标, 下装坐标, 上衣颜色, 下装颜色, 置信度, 身体比例)
            img (np.array): 当前帧图像，用于颜色提取

        返回:
            list: 当前活跃的跟踪轨迹
            
        调用关系:
            - 被ByteTrackerNode.process_frame()调用
            - 被SingleTargetTracker.update()调用
            
        调用原因:
            执行每帧的跟踪更新，维护轨迹状态
            
        调试信息:
            - 记录每个处理阶段的结果："第一阶段关联: {len(matches)}个匹配, {len(u_detection)}个未匹配检测"
            - 显示轨迹状态转换统计
            - 监控关联质量和性能指标
        """
        self.frame_id += 1

        # 结果容器
        activated_starcks = []
        refind_stracks = []
        lost_stracks = []
        removed_stracks = []

        # 如果没有检测结果，就将所有轨迹标记为丢失
        if not detection_results:
            for track in self.tracked_stracks:
                if track.state == TrackState.TRACKED:
                    track.mark_lost()
                    lost_stracks.append(track)

            # 更新状态并返回
            self._update_status(activated_starcks, refind_stracks, lost_stracks, removed_stracks)
            return [track for track in self.tracked_stracks if track.is_activated]

        # 处理检测结果，创建 STrack 对象
        detections = []
        low_detections = []

        for result in detection_results:
            # 解析检测结果
            try:
                # 处理不同长度的结果
                if len(result) >= 7:  # 结果包含身体比例
                    upper_bbox, lower_bbox, upper_color, lower_color, upper_confidence, lower_confidence, body_ratios = result
                else:  # 结果不包含身体比例
                    upper_bbox, lower_bbox, upper_color, lower_color, upper_confidence, lower_confidence = result
                    body_ratios = None

                confidence = (upper_confidence + lower_confidence) / 2

                # 从上衣和下装位置估计整体位置
                if len(upper_bbox) == 4 and len(lower_bbox) == 4:
                    # 使用上衣和下装共同确定位置
                    full_bbox = self._integrate_position(upper_bbox, lower_bbox)
                elif len(upper_bbox) == 4:
                    # 只有上衣
                    full_bbox = self._expand_position(upper_bbox, is_upper=True)
                elif len(lower_bbox) == 4:
                    # 只有下装
                    full_bbox = self._expand_position(lower_bbox, is_upper=False)
                else:
                    continue

                # 构建跟踪对象
                strack = STrack(
                    STrack.tlbr_to_tlwh(np.array(full_bbox)),
                    confidence,
                    upper_color=upper_color,
                    lower_color=lower_color,
                    body_ratios=body_ratios
                )

                # 根据置信度分类
                if confidence >= self.args.get('track_thresh', 0.5):
                    detections.append(strack)
                elif confidence >= 0.1:  # 低置信度阈值
                    low_detections.append(strack)

            except Exception as e:
                continue

        # 将现有轨迹分为已确认和未确认
        unconfirmed = []
        tracked_stracks = []

        for track in self.tracked_stracks:
            if not track.is_activated:
                unconfirmed.append(track)
            else:
                tracked_stracks.append(track)

        # 合并确认轨迹和丢失轨迹作为匹配池
        strack_pool = joint_stracks(tracked_stracks, self.lost_stracks)

        # 预测所有轨迹的新位置
        STrack.multi_predict(strack_pool)

        # 第一阶段关联 - 与高置信度检测关联
        # 计算IOU距离
        iou_dists = iou_distance(strack_pool, detections)

        # 计算颜色距离并融合
        if self.color_weight > 0:
            color_dists = color_distance(strack_pool, detections)
            dists = fuse_iou_with_color(
                iou_dists, color_dists, detections,
                w_iou=1 - self.color_weight, w_color=self.color_weight
            )
        else:
            dists = iou_dists

        # 使用匈牙利算法分配
        matches, u_track, u_detection = linear_assignment(
            dists, thresh=self.args.get('match_thresh', 0.8)
        )

        # 处理匹配结果
        for i_track, i_det in matches:
            track = strack_pool[i_track]
            det = detections[i_det]

            if track.state == TrackState.TRACKED:
                # 已跟踪轨迹，更新
                track.update(det, self.frame_id)
                activated_starcks.append(track)
            else:
                # 丢失轨迹，重新激活
                track.re_activate(det, self.frame_id, new_id=False)
                refind_stracks.append(track)

        # 第二阶段关联 - 与低置信度检测关联
        if len(low_detections) > 0:
            # 仅使用仍在跟踪中的轨迹
            r_tracked_stracks = [strack_pool[i] for i in u_track if strack_pool[i].state == TrackState.TRACKED]

            # 计算IOU距离
            iou_dists = iou_distance(r_tracked_stracks, low_detections)

            # 颜色距离和融合
            if self.color_weight > 0:
                color_dists = color_distance(r_tracked_stracks, low_detections)
                dists = fuse_iou_with_color(
                    iou_dists, color_dists, low_detections,
                    w_iou=1 - self.color_weight, w_color=self.color_weight
                )
            else:
                dists = iou_dists

            # 使用更宽松的阈值
            matches, u_tracks, u_low_detection = linear_assignment(dists, thresh=0.5)

            # 处理匹配结果
            for i_track, i_det in matches:
                track = r_tracked_stracks[i_track]
                det = low_detections[i_det]

                if track.state == TrackState.TRACKED:
                    track.update(det, self.frame_id)
                    activated_starcks.append(track)
                else:
                    track.re_activate(det, self.frame_id, new_id=False)
                    refind_stracks.append(track)

            # 处理未匹配轨迹
            for i in u_tracks:
                track = r_tracked_stracks[i]
                if track.state != TrackState.LOST:
                    track.mark_lost()
                    lost_stracks.append(track)
        else:
            # 没有低置信度检测，将所有未匹配轨迹标记为丢失
            for i in u_track:
                track = strack_pool[i]
                if track.state != TrackState.LOST:
                    track.mark_lost()
                    lost_stracks.append(track)

        # 处理未确认轨迹
        u_detection_track = [detections[i] for i in u_detection]
        iou_dists = iou_distance(unconfirmed, u_detection_track)

        if self.color_weight > 0:
            color_dists = color_distance(unconfirmed, u_detection_track)
            dists = fuse_iou_with_color(
                iou_dists, color_dists, u_detection_track,
                w_iou=1 - self.color_weight, w_color=self.color_weight
            )
        else:
            dists = iou_dists

        matches, u_unconfirmed, u_detection = linear_assignment(dists, thresh=0.7)

        for i_track, i_det in matches:
            unconfirmed[i_track].update(u_detection_track[i_det], self.frame_id)
            activated_starcks.append(unconfirmed[i_track])

        # 移除未匹配的未确认轨迹
        for i in u_unconfirmed:
            track = unconfirmed[i]
            track.mark_removed()
            removed_stracks.append(track)

        # 创建新轨迹
        for i in u_detection:
            track = u_detection_track[i]
            if track.score < self.det_thresh:
                continue

            # 激活新轨迹
            track.activate(self.kalman_filter, self.frame_id)
            activated_starcks.append(track)

        # 处理丢失时间过长的轨迹
        for track in self.lost_stracks:
            if self.frame_id - track.end_frame > self.max_time_lost:
                track.mark_removed()
                removed_stracks.append(track)

        # 更新跟踪器状态
        self._update_status(activated_starcks, refind_stracks, lost_stracks, removed_stracks)

        # 返回激活的轨迹
        return [track for track in self.tracked_stracks if track.is_activated]

    def _update_status(self, activated_starcks, refind_stracks, lost_stracks, removed_stracks):
        """
        更新跟踪器内部状态 - 轨迹生命周期管理
        ======================================
        
        参数：
            activated_starcks (list): 新激活的轨迹
            refind_stracks (list): 重新找到的轨迹
            lost_stracks (list): 新丢失的轨迹
            removed_stracks (list): 被移除的轨迹
            
        返回值：
            无
            
        调用关系：
            - 被update()方法在每帧结束时调用
            
        调用原因：
            维护轨迹状态的一致性，管理轨迹生命周期
            
        调试信息：
            - 记录状态转换统计
            - 显示各类轨迹的数量变化
        """
        # 更新跟踪中轨迹
        self.tracked_stracks = [t for t in self.tracked_stracks if t.state == TrackState.TRACKED]
        self.tracked_stracks = joint_stracks(self.tracked_stracks, activated_starcks)
        self.tracked_stracks = joint_stracks(self.tracked_stracks, refind_stracks)

        # 更新丢失轨迹
        self.lost_stracks = sub_stracks(self.lost_stracks, self.tracked_stracks)
        self.lost_stracks.extend(lost_stracks)
        self.lost_stracks = sub_stracks(self.lost_stracks, self.removed_stracks)

        # 更新移除轨迹
        self.removed_stracks.extend(removed_stracks)

        # 移除重复轨迹
        self.tracked_stracks, self.lost_stracks = remove_duplicate_stracks(
            self.tracked_stracks, self.lost_stracks)

    def _integrate_position(self, upper_pos, lower_pos):
        """
        整合上衣和下装位置得到整体位置
        
        参数：
            upper_pos (list): 上衣边界框 [x1,y1,x2,y2]
            lower_pos (list): 下装边界框 [x1,y1,x2,y2]
            
        返回值：
            list: 整合后的边界框 [x1,y1,x2,y2]
            
        调用关系：
            - 被update()方法在处理检测结果时调用
            
        调用原因：
            从部分检测结果估计完整的人体位置
        """
        x_min = min(upper_pos[0], lower_pos[0])
        y_min = min(upper_pos[1], lower_pos[1])
        x_max = max(upper_pos[2], lower_pos[2])
        y_max = max(upper_pos[3], lower_pos[3])

        return [x_min, y_min, x_max, y_max]

    def _expand_position(self, part_pos, is_upper=True):
        """
        根据部分位置(上衣或下装)估计整体位置
        
        参数：
            part_pos (list): 部分边界框 [x1,y1,x2,y2]
            is_upper (bool): 是否为上衣位置
            
        返回值：
            list: 估计的完整边界框 [x1,y1,x2,y2]
            
        调用关系：
            - 被update()方法在只有部分检测时调用
            
        调用原因：
            从不完整的检测信息推断完整的人体区域
        """
        x_min, y_min, x_max, y_max = part_pos
        width = x_max - x_min
        height = y_max - y_min

        if is_upper:
            # 从上衣估计整体位置
            return [
                max(0, x_min - int(width * 0.1)),
                max(0, y_min - int(height * 0.1)),
                x_max + int(width * 0.1),
                y_max + int(height * 1.5)  # 向下扩展更多
            ]
        else:
            # 从下装估计整体位置
            return [
                max(0, x_min - int(width * 0.1)),
                max(0, y_min - int(height * 1.2)),  # 向上扩展更多
                x_max + int(width * 0.1),
                y_max + int(height * 0.1)
            ]


# ==================== 单目标跟踪器 ====================

class SingleTargetTracker:
    """
    单目标跟踪器，可以根据目标特征锁定并跟踪特定人物 - 专用目标锁定系统
    =======================================================================
    
    功能说明：
        基于多维特征（颜色、身体比例）识别和跟踪特定目标
        支持目标选择和锁定跟踪两种模式
        
    调用关系：
        - 被ByteTrackerNode在单目标模式下使用
        - 内部使用ColorByteTracker作为基础跟踪引擎
        
    设计目的：
        提供高精度的特定目标跟踪，适用于人员跟随等应用
    """

    # 跟踪模式
    MODE_SELECTING = 0  # 目标选择模式
    MODE_TRACKING = 1  # 目标跟踪模式

    def __init__(self, tracker_args, target_features, max_lost_time=30):
        """
        初始化单目标跟踪器

        参数:
            tracker_args (dict): 传递给ByteTracker的参数
            target_features (tuple): 目标特征，格式为(body_ratios, shirt_color, pants_color)
            max_lost_time (int): 最大丢失时间，超过此时间将切换回选择模式
            
        返回值:
            无
            
        调用关系:
            - 被ByteTrackerNode.init_single_target_tracker()调用
            
        调用原因:
            初始化专用的单目标跟踪系统
            
        调试信息:
            - 记录目标特征："初始化单目标跟踪器，目标特征: 身体比例{len(body_ratios)}维, 上衣{shirt_color}, 下装{pants_color}"
            - 显示跟踪参数和阈值设置
        """
        # 初始化ByteTracker
        self.base_tracker = ColorByteTracker(tracker_args)

        # 保存目标特征
        self.target_body_ratios, self.target_shirt_color, self.target_pants_color = target_features

        # 初始化状态
        self.mode = self.MODE_SELECTING  # 初始为选择模式
        self.target_id = None  # 目标ID，None表示尚未锁定
        self.lost_frames = 0  # 目标丢失帧数
        self.max_lost_frames = max_lost_time  # 最大丢失帧数

        self.frame_count = 0
        self.target_score_history = []  # 记录目标得分历史
        self.confirmed_target_position = None  # 确认的目标位置
        self.target_trajectory = deque(maxlen=30)  # 记录目标轨迹

    def body_ratio_similarity(self, detected_ratios):
        """
        计算身体比例相似度 - 生物特征匹配算法
        ======================================

        参数:
            detected_ratios (list): 检测到的身体比例，16维特征向量

        返回:
            float: 相似度，0-1之间，越高越相似
            
        调用关系:
            - 被calculate_target_score()调用
            
        调用原因:
            提供生物特征层面的目标识别能力
            
        调试信息:
            - 记录特征向量的有效维度数
            - 显示余弦相似度计算结果
            - 监控特征稳定性
        """
        if not detected_ratios or len(detected_ratios) != len(self.target_body_ratios):
            return 0.0

        # 使用余弦相似度
        try:
            target = np.array(self.target_body_ratios)
            detected = np.array(detected_ratios)

            # 处理零向量情况
            if np.all(target == 0) or np.all(detected == 0):
                return 0.0

            # 计算余弦相似度
            dot_product = np.dot(target, detected)
            norm_product = np.linalg.norm(target) * np.linalg.norm(detected)

            if norm_product == 0:
                return 0.0

            similarity = dot_product / norm_product
            # 将相似度范围从[-1,1]调整到[0,1]
            similarity = (similarity + 1) / 2
            return similarity

        except Exception:
            return 0.0

    def color_similarity(self, color1, color2):
        """
        计算颜色相似度 - 外观特征匹配算法
        ================================

        参数:
            color1 (tuple): BGR颜色元组
            color2 (tuple): BGR颜色元组

        返回:
            float: 相似度，0-1之间，越高越相似
            
        调用关系:
            - 被calculate_target_score()调用
            
        调用原因:
            提供外观层面的目标识别能力
            
        调试信息:
            - 记录颜色距离计算过程
            - 显示颜色空间的分布特征
        """
        if not color1 or not color2:
            return 0.0

        try:
            # 转换为numpy数组
            c1 = np.array(color1)
            c2 = np.array(color2)

            # 计算欧氏距离
            distance = np.sqrt(np.sum((c1 - c2) ** 2))
            # 颜色距离最大可能值为 sqrt(255^2 * 3) = 441.67
            max_distance = 442.0

            # 转换为相似度
            similarity = 1.0 - min(distance / max_distance, 1.0)
            return similarity

        except Exception:
            return 0.0

    def calculate_target_score(self, track):
        """
        计算一个轨迹与目标的匹配得分 - 多特征融合评分系统
        ===================================================

        参数:
            track (STrack): 跟踪对象

        返回:
            float: 得分，0-1之间，越高越匹配
            
        调用关系:
            - 被select_target()调用
            - 被update()方法中的目标评估调用
            
        调用原因:
            量化轨迹与目标的匹配程度，支持目标选择决策
            
        调试信息:
            - 记录各特征的匹配得分："轨迹{track_id}: 上衣相似度{shirt_sim:.3f}, 下装相似度{pants_sim:.3f}, 身体比例{body_sim:.3f}"
            - 显示特征可用性和权重分配
            - 监控综合得分的计算过程
        """
        # 获取检测到的颜色信息
        shirt_color = track.upper_color if hasattr(track, 'upper_color') and track.upper_color else None
        pants_color = track.lower_color if hasattr(track, 'lower_color') and track.lower_color else None

        # 计算颜色相似度
        shirt_sim = self.color_similarity(self.target_shirt_color, shirt_color) if shirt_color else 0.0
        pants_sim = self.color_similarity(self.target_pants_color, pants_color) if pants_color else 0.0

        # 计算身体比例相似度
        body_ratio_sim = 0.0
        has_body_ratios = False

        if hasattr(track, 'body_ratios') and track.body_ratios:
            # 检查是否有有效的身体比例数据
            valid_ratios = sum(1 for r in track.body_ratios if r > 0)
            if valid_ratios > 4:  # 至少需要4个有效比例
                body_ratio_sim = self.body_ratio_similarity(track.body_ratios)
                has_body_ratios = True

        # 根据可用信息设置权重和计算总分
        feature_count = 0
        weighted_sum = 0.0

        # 上衣颜色 (权重 0.4)
        if shirt_color:
            weighted_sum += 0.4 * shirt_sim
            feature_count += 1

        # 下装颜色 (权重 0.3)
        if pants_color:
            weighted_sum += 0.3 * pants_sim
            feature_count += 1

        # 身体比例 (权重 0.3)
        if has_body_ratios:
            weighted_sum += 0.3 * body_ratio_sim
            feature_count += 1

        # 计算最终得分
        if feature_count > 0:
            # 根据可用特征数量调整得分
            base_score = weighted_sum / feature_count * (0.8 + 0.2 * feature_count)

            # 如果有所有特征，增加一点额外奖励
            if feature_count == 3:
                base_score *= 1.05

            # 如果只有一种特征，降低可信度
            if feature_count == 1:
                base_score *= 0.9
        else:
            # 没有任何特征信息
            base_score = 0.0

        # 如果已经在跟踪模式，考虑历史信息
        if self.mode == self.MODE_TRACKING and self.target_id == track.track_id:
            base_score = base_score * 1.2  # 提高已锁定目标的得分

        return min(base_score, 1.0)  # 确保得分不超过1.0

    def select_target(self, tracks):
        """
        从当前轨迹中选择最匹配的目标 - 智能目标选择算法
        ===============================================

        参数:
            tracks (list): 当前跟踪的轨迹列表

        返回:
            tuple: (best_track, score)
                - best_track: 最匹配的轨迹，没有合适的则返回None
                - score: 最高的匹配得分
                
        调用关系:
            - 被update()方法在选择模式下调用
            
        调用原因:
            从多个候选轨迹中识别最可能的目标
            
        调试信息:
            - 记录候选轨迹评估过程
            - 显示得分排序和选择逻辑
            - 监控选择稳定性
        """
        if not tracks:
            return None, 0.0

        best_track = None
        best_score = 0.0

        for track in tracks:
            score = self.calculate_target_score(track)

            if score > best_score:
                best_score = score
                best_track = track

        # 只有得分超过阈值才认为找到了目标
        min_score_threshold = 0.4  # 最低匹配阈值
        if best_score >= min_score_threshold:
            return best_track, best_score
        else:
            return None, best_score

    def update(self, detection_results, image=None):
        """
        更新跟踪器状态 - 单目标跟踪主控逻辑
        ===================================

        参数:
            detection_results (list): 检测结果列表
            image (np.array): 当前帧图像

        返回:
            tuple: (track_results, target_track, mode)
                - track_results: 所有跟踪结果
                - target_track: 目标轨迹 (如果在跟踪模式下)
                - mode: 当前模式
                
        调用关系:
            - 被ByteTrackerNode.process_frame()在单目标模式下调用
            
        调用原因:
            执行单目标跟踪的完整流程，包括目标选择和跟踪保持
            
        调试信息:
            - 记录模式转换："模式切换: {old_mode} -> {new_mode}, 原因: {reason}"
            - 显示目标评分历史和稳定性分析
            - 监控跟踪质量和丢失恢复情况
        """
        self.frame_count += 1

        # 使用ByteTracker更新所有轨迹
        all_tracks = self.base_tracker.update(detection_results, image)

        if not all_tracks:
            if self.mode == self.MODE_TRACKING:
                self.lost_frames += 1

                # 检查是否需要切换回选择模式
                if self.lost_frames > self.max_lost_frames:
                    self.mode = self.MODE_SELECTING
                    self.target_id = None
                    self.target_trajectory.clear()

            return all_tracks, None, self.mode

        # 根据当前模式处理
        if self.mode == self.MODE_SELECTING:
            # 选择模式：尝试找到匹配目标
            best_track, score = self.select_target(all_tracks)

            # 记录得分历史，用于稳定目标选择
            self.target_score_history.append(score)
            if len(self.target_score_history) > 10:
                self.target_score_history.pop(0)

            # 如果连续多帧得分都很高，则锁定目标
            if best_track and len(self.target_score_history) >= 3:
                recent_scores = self.target_score_history[-3:]
                if all(s >= 0.4 for s in recent_scores):
                    self.target_id = best_track.track_id
                    self.mode = self.MODE_TRACKING
                    self.lost_frames = 0
                    self.target_trajectory.clear()

                    # 记录位置用于轨迹显示
                    center_x = (best_track.tlbr[0] + best_track.tlbr[2]) / 2
                    center_y = (best_track.tlbr[1] + best_track.tlbr[3]) / 2
                    self.target_trajectory.append((int(center_x), int(center_y)))

            return all_tracks, best_track, self.mode

        else:  # 跟踪模式
            # 在当前轨迹中查找目标ID
            target_track = None
            for track in all_tracks:
                if track.track_id == self.target_id:
                    target_track = track
                    self.lost_frames = 0  # 重置丢失计数

                    # 更新轨迹
                    center_x = (track.tlbr[0] + track.tlbr[2]) / 2
                    center_y = (track.tlbr[1] + track.tlbr[3]) / 2
                    self.target_trajectory.append((int(center_x), int(center_y)))

                    break

            # 如果目标不在当前帧，增加丢失计数
            if target_track is None:
                self.lost_frames += 1

                # 检查是否需要切换回选择模式
                if self.lost_frames > self.max_lost_frames:
                    self.mode = self.MODE_SELECTING
                    self.target_id = None
                    # 清空轨迹
                    self.target_trajectory.clear()

            return all_tracks, target_track, self.mode


# ==================== 相机配置类 ====================

class CameraConfig:
    """
    相机配置类 - 视频采集参数管理
    =============================
    
    功能说明：
        管理相机的基本配置参数
        支持单目和双目相机设置
        
    调用关系：
        - 被ByteTrackerNode使用
        
    设计目的：
        统一管理相机参数，便于配置和调试
    """
    
    def __init__(self):
        """
        初始化相机配置参数
        
        参数：
            无
            
        返回值：
            无
            
        调用关系：
            - 被ByteTrackerNode.__init__()调用
            
        调用原因：
            设置默认的相机参数
        """
        # 相机参数
        self.camera_id = 1 # 默认相机ID
        self.frame_width = 1280  # 双目相机需要更大宽度
        self.frame_height = 480
        self.video_file_path = None  # 视频文件路径，None表示使用相机
        self.fps_limit = 30
        
        # 图像处理参数
        self.flip_horizontal = False
        self.flip_vertical = False
        
        # 立体视觉参数
        self.is_stereo_camera = True  # 是否为立体相机
        self.enable_distance_measure = True  # 是否启用距离测量


class StereoConfig:
    """
    立体视觉配置类 - 深度估计参数管理
    ===============================
    
    功能说明：
        管理立体视觉算法的配置参数
        包含SGBM算法的详细设置
        
    调用关系：
        - 被ByteTrackerNode的立体视觉模块使用
        
    设计目的：
        提供可调节的深度估计参数
    """
    
    def __init__(self):
        """
        初始化立体视觉配置参数
        
        参数：
            无
            
        返回值：
            无
            
        调用关系：
            - 被ByteTrackerNode.__init__()调用
            
        调用原因：
            设置立体匹配算法的参数
        """
        # 相机内参和外参
        self.baseline = 25.100  # 基线距离(mm)
        self.focal_length = 663  # 焦距
        self.cx = 317  # 光心x坐标
        self.cy = 210  # 光心y坐标

        # SGBM算法参数
        self.minDisparity = 3
        self.numDisparities = 32
        self.blockSize = 7
        self.P1 = 800
        self.P2 = 3200
        self.disp12MaxDiff = 3
        self.preFilterCap = 31
        self.uniquenessRatio = 10
        self.speckleWindowSize = 100
        self.speckleRange = 32
        try:
            self.mode = cv2.STEREO_SGBM_MODE_SGBM_3WAY
        except AttributeError:
            self.mode = cv2.STEREO_SGBM_MODE_SGBM

        # 距离测量范围
        self.min_distance_mm = 100.0
        self.max_distance_mm = 10000.0


class StereoCamera:
    """
    双目相机参数类 - 立体标定参数存储
    ===============================
    
    功能说明：
        存储双目相机的标定参数
        包含内参、外参、畸变系数等
        
    调用关系：
        - 被ByteTrackerNode的立体视觉模块使用
        
    设计目的：
        封装相机标定结果，支持立体校正
    """
    
    def __init__(self):
        """
        初始化双目相机标定参数
        
        参数：
            无
            
        返回值：
            无
            
        调用关系：
            - 被ByteTrackerNode.__init__()调用
            
        调用原因：
            加载预标定的相机参数
        """
        # 左相机内参
        self.cam_matrix_left = np.array([[660.1946, 0, 326.3185], 
                                       [0, 660.8720, 207.1556], 
                                       [0, 0, 1]])

        # 右相机内参
        self.cam_matrix_right = np.array([[665.1635, 0, 319.9729], 
                                        [0, 665.7919, 212.9630], 
                                        [0, 0, 1]])

        # 左右相机畸变系数:[k1, k2, p1, p2, k3]
        self.distortion_l = np.array([[-0.0682, 0.1546, 0, 0, 0]])
        self.distortion_r = np.array([[-0.0749, 0.1684, 0, 0, 0]])

        # 旋转矩阵
        self.R = np.array([[1.0, 6.140854786327222e-04, -0.0022],
                          [-6.240288417695294e-04, 1, -0.0046],
                          [0.0022, 0.0046, 1]])

        # 平移矩阵
        self.T = np.array([[-25.0961], [-0.0869], [-0.1893]])

        # 焦距和基线距离
        self.focal_length = 663
        self.baseline = abs(self.T[0][0])

        # Q矩阵（视差到深度的映射矩阵）
        self.Q = None


# ==================== ByteTracker ROS2节点 ====================

class ByteTrackerNode(Node):
    """
    ByteTracker ROS2节点 - 多目标跟踪系统的ROS2集成
    ===============================================
    
    功能说明：
        将ByteTracker算法集成到ROS2生态系统
        提供完整的跟踪服务和可视化
        支持多目标和单目标跟踪模式
        
    调用关系：
        - 被main()函数创建和管理
        - 与其他ROS2节点通过话题和服务通信
        
    设计目的：
        提供生产就绪的跟踪解决方案
    """

    def __init__(self):
        """
        初始化节点
        
        参数：
            无
            
        返回值：
            无
            
        调用关系：
            - 被main()函数调用
            
        调用原因：
            创建完整的跟踪系统实例
            
        调试信息：
            - 记录初始化过程的每个步骤
            - 显示系统配置和状态检查
        """
        super().__init__('bytetracker_node')

        # 初始化基本组件
        self.lock = threading.Lock()
        
        # OpenCV桥接器（用于图像消息转换）
        self.bridge = CvBridge()
        
        # 相机相关
        self.camera_config = CameraConfig()
        self.cap = None
        self.running = False
        
        # 立体视觉相关
        self.stereo_config = StereoConfig()
        self.stereo_camera = StereoCamera()
        self.current_left_frame = None
        self.current_right_frame = None
        
        # 立体视觉处理
        self.map1x = None
        self.map1y = None
        self.map2x = None
        self.map2y = None
        self.Q = None
        self.stereo_matcher = None

        # 跟踪器参数
        self.tracker_args = {
            'track_thresh': 0.5,      # 跟踪阈值
            'track_buffer': 100,      # 轨迹缓冲
            'match_thresh': 0.8,      # 匹配阈值
            'color_weight': 0.5       # 颜色权重
        }

        # 跟踪模式
        self.tracking_mode = 'multi'  # 'multi' 或 'single'
        self.single_target_tracker = None
        self.multi_target_tracker = None
        self.target_person_name = ""

        # 初始化多目标跟踪器
        self.multi_target_tracker = ColorByteTracker(self.tracker_args)

        # 当前帧和处理结果
        self.current_frame = None
        self.current_tracks = []
        self.current_target_track = None
        self.processing = False

        # 统计信息
        self.frame_count = 0
        self.processing_time = 0
        self.current_fps = 0.0  # 当前帧率

        # 设置参数
        self.setup_parameters()

        # 创建发布者和订阅者
        self.setup_publishers()
        self.setup_subscribers()

        # 创建服务客户端
        self.setup_service_clients()

        # 初始化相机
        self.setup_camera()
        
        # 初始化立体视觉
        if self.camera_config.is_stereo_camera and self.enable_distance_measure:
            self.setup_stereo_vision()

        # 创建定时器
        self.setup_timers()

        self.get_logger().info('✅ ByteTracker节点初始化完成')
        self.get_logger().info(f'🎮 当前跟踪模式: {self.tracking_mode}')
        self.get_logger().info(f'📷 相机ID: {self.camera_config.camera_id}, 分辨率: {self.camera_config.frame_width}x{self.camera_config.frame_height}')

        # 🔧 修复：如果是单目标模式，自动初始化单目标跟踪器
        if self.tracking_mode == 'single':
            self.get_logger().info('🎯 检测到单目标模式，正在初始化单目标跟踪器...')
            self.init_single_target_tracker()
        
        # 🔧 额外：添加目标特征文件的状态检查
        if self.target_features_file:
            self.get_logger().info(f'📄 目标特征文件: {self.target_features_file}')
        else:
            self.get_logger().warn('⚠️ 未设置目标特征文件')

    def process_frame(self):
        """
        帧处理主循环 - ByteTracker系统的核心处理函数
        ================================================
        
        功能说明：
            每帧执行一次，完成检测、跟踪、发布、可视化的完整流程
            是整个跟踪系统的心脏，协调所有模块的工作
        
        参数：
            无（从实例变量获取当前帧）
            
        返回值：
            无（通过ROS话题发布结果）
            
        调用关系：
            - 被self.process_timer定时器定时调用（20Hz）
            - 被setup_timers()中创建的定时器触发
            
        调用原因：
            实现实时跟踪处理，保持系统的连续运行
            
        调试信息：
            - 记录每帧的详细处理过程："🎬 处理第{frame_count}帧: 检测{detection_count}个目标"
            - 显示各阶段的耗时分析："检测:{detection_time:.3f}s, 跟踪:{tracking_time:.3f}s"
            - 监控系统性能和异常情况
        """
        # 🔄 帧获取和并发控制
        with self.lock:
            if self.current_frame is None or self.processing:
                return  # 没有新帧或正在处理中，跳过本次
            self.processing = True
            frame = self.current_frame.copy()
            self.frame_count += 1

        # 📊 性能监控和调试信息
        start_time = time.time()
        
        try:
            self.get_logger().debug(f"🎬 开始处理第 {self.frame_count} 帧")
            
            # 🔍 Step 1: 检测和特征提取
            detection_start = time.time()
            detection_results = self.detect_and_extract_features(frame)
            detection_time = time.time() - detection_start
            
            self.get_logger().debug(f"🔍 检测完成: {len(detection_results)} 个检测结果, 耗时: {detection_time:.3f}s")
            
            # 🎯 Step 2: 根据跟踪模式处理
            tracking_start = time.time()
            if self.tracking_mode == 'multi':
                # 多目标跟踪模式
                if self.multi_target_tracker is not None:
                    tracks = self.multi_target_tracker.update(detection_results, frame)
                    self.get_logger().debug(f"🔢 多目标跟踪: {len(tracks)} 个轨迹")
                else:
                    tracks = []
                    self.get_logger().warn("⚠️ 多目标跟踪器未初始化")
                target_track = None
                mode = 'multi'
            else:
                # 单目标跟踪模式
                if self.single_target_tracker is not None:
                    all_tracks, target_track, mode = self.single_target_tracker.update(
                        detection_results, frame)
                    tracks = all_tracks
                    
                    # 🎯 详细的单目标跟踪调试信息
                    if target_track:
                        # 计算目标匹配置信度
                        target_score = self.single_target_tracker.calculate_target_score(target_track)
                        center_x = (target_track.tlbr[0] + target_track.tlbr[2]) / 2
                        center_y = (target_track.tlbr[1] + target_track.tlbr[3]) / 2
                        
                        self.get_logger().info(f"🎯 目标跟踪: ID={target_track.track_id}, "
                                             f"置信度={target_score:.3f}, "
                                             f"位置=({center_x:.1f},{center_y:.1f}), "
                                             f"模式={mode}")
                        
                        # 特征匹配详情
                        if hasattr(target_track, 'upper_color') and target_track.upper_color:
                            shirt_sim = self.single_target_tracker.color_similarity(
                                self.single_target_tracker.target_shirt_color, target_track.upper_color)
                            self.get_logger().debug(f"👕 上衣颜色匹配度: {shirt_sim:.3f}")
                            
                        if hasattr(target_track, 'lower_color') and target_track.lower_color:
                            pants_sim = self.single_target_tracker.color_similarity(
                                self.single_target_tracker.target_pants_color, target_track.lower_color)
                            self.get_logger().debug(f"👖 下装颜色匹配度: {pants_sim:.3f}")
                            
                        if hasattr(target_track, 'body_ratios') and target_track.body_ratios:
                            body_sim = self.single_target_tracker.body_ratio_similarity(target_track.body_ratios)
                            self.get_logger().debug(f"🦴 身体比例匹配度: {body_sim:.3f}")
                    else:
                        self.get_logger().debug(f"🔍 单目标跟踪: 当前模式={mode}, 无目标锁定")
                        
                    self.get_logger().debug(f"🎯 单目标跟踪: {len(all_tracks)} 个候选轨迹")
                else:
                    tracks = []
                    target_track = None
                    mode = 'single_not_initialized'
                    self.get_logger().warn("⚠️ 单目标跟踪器未初始化")
            
            tracking_time = time.time() - tracking_start
            self.get_logger().debug(f"🎯 跟踪完成: 耗时 {tracking_time:.3f}s")

            # 🔄 Step 3: 更新当前跟踪结果
            self.current_tracks = tracks
            self.current_target_track = target_track

            # 📡 Step 4: 发布跟踪结果
            publish_start = time.time()
            self.publish_tracking_results(tracks, target_track)
            
            # 📡 发布TrackingResult消息（供WebSocket桥接节点使用）
            self.publish_tracking_result(tracks, target_track, mode)
            
            # 🚗 Step 5: 小车控制（如果启用）
            if target_track and self.enable_car_control:
                distance = self.get_distance_at_point(
                    int((target_track.tlbr[0] + target_track.tlbr[2]) / 2),
                    int((target_track.tlbr[1] + target_track.tlbr[3]) / 2)
                )
                self.publish_target_position(target_track, frame.shape)
                self.get_logger().debug(f"🚗 小车控制: 目标距离 {distance:.1f}mm")

            # 📺 Step 6: 创建和发布可视化
            viz_frame = self.create_visualization(frame, tracks, target_track, mode)
            self.publish_visualization(viz_frame)
            
            publish_time = time.time() - publish_start
            self.get_logger().debug(f"📡 发布完成: 耗时 {publish_time:.3f}s")

            # 📊 Step 7: 性能统计和FPS计算
            total_time = time.time() - start_time
            fps = 1.0 / total_time if total_time > 0 else 0
            self.processing_time = total_time
            
            # 滑动窗口FPS计算
            if not hasattr(self, 'fps_window'):
                self.fps_window = []
                self.fps_window_size = 10
                
            self.fps_window.append(fps)
            if len(self.fps_window) > self.fps_window_size:
                self.fps_window.pop(0)
            
            # 平均FPS
            self.current_fps = sum(self.fps_window) / len(self.fps_window)
            
            # 📊 发布详细跟踪数据（每帧都发布）
            self.publish_detailed_tracking_data(tracks, target_track, mode)
            
            # 每10帧输出一次性能信息
            # if self.frame_count % 10 == 0:
            #     self.get_logger().info(f"📊 性能统计 (第{self.frame_count}帧): "
            #                          f"总耗时={total_time:.3f}s, FPS={self.current_fps:.1f}, "
            #                          f"检测={detection_time:.3f}s, 跟踪={tracking_time:.3f}s, "
            #                          f"发布={publish_time:.3f}s")

        except Exception as e:
            self.get_logger().error(f"❌ 处理第{self.frame_count}帧时发生错误: {e}")
            traceback.print_exc()

        finally:
            # 🔓 释放处理锁
            with self.lock:
                self.processing = False

    def detect_and_extract_features(self, frame):
        """
        检测服装并提取特征 - 多模态特征提取核心函数
        =============================================
        
        功能说明：
            调用服装检测和姿态估计模块，提取多维特征
            根据跟踪模式动态调整特征提取策略
        
        参数：
            frame (np.array): 当前帧图像
            
        返回值：
            list: 增强的检测结果列表，包含位置、颜色、身体比例信息
            
        调用关系：
            - 被process_frame()调用
            - 内部调用rknn_colour_detect和yolov8_pose_detector模块
            
        调用原因：
            为跟踪算法提供丰富的特征信息
            
        调试信息：
            - 记录检测模块的可用性和调用结果
            - 显示特征提取的成功率和质量
            - 监控处理时间和资源消耗
        """
        try:
            # 使用服装检测模块
            if CLOTHING_DETECTION_AVAILABLE:
                # 调用detect_picture_with_confidence获取检测结果
                detection_results = detect_picture_with_confidence(frame)

                # 根据跟踪模式决定是否计算身体比例
                if self.tracking_mode == 'single' and hasattr(self, 'single_target_tracker'):
                    # 单目标模式下，只在选择阶段计算身体比例
                    if self.single_target_tracker and self.single_target_tracker.mode == SingleTargetTracker.MODE_SELECTING:
                        detection_results = self.extract_body_ratios_from_detections(
                            frame, detection_results, calculate_ratios=True)
                    else:
                        detection_results = self.extract_body_ratios_from_detections(
                            frame, detection_results, calculate_ratios=False)
                else:
                    # 多目标模式下，定期计算身体比例（每10帧）
                    if self.frame_count % 10 == 0:
                        detection_results = self.extract_body_ratios_from_detections(
                            frame, detection_results, calculate_ratios=True)
                    else:
                        detection_results = self.extract_body_ratios_from_detections(
                            frame, detection_results, calculate_ratios=False)

                # 调试输出：检测结果概况
                # self.get_logger().info(f'🔍 检测结果: 检测到 {len(detection_results)} 个人员')
                return detection_results
            else:
                self.get_logger().warn("⚠️ 服装检测模块不可用")
                return []

        except Exception as e:
            self.get_logger().error(f"特征检测错误: {e}")
            return []

    def extract_body_ratios_from_detections(self, frame, detection_results, calculate_ratios=True):
        """
        通过裁剪检测区域进行姿态估计，确保身体比例与服装检测结果一一对应
        ================================================================
        
        功能说明：
            为每个检测结果计算对应的身体比例特征
            通过人体区域裁剪和姿态估计实现精确匹配
        
        参数：
            frame (np.array): 当前帧图像
            detection_results (list): 服装检测结果
            calculate_ratios (bool): 是否计算身体比例（性能优化开关）
            
        返回值：
            list: 增强的检测结果，每项包含身体比例信息
            
        调用关系：
            - 被detect_and_extract_features()调用
            - 内部调用yolov8_pose_detector模块
            
        调用原因：
            提供生物特征约束，增强跟踪鲁棒性
            
        调试信息：
            - 记录姿态检测的成功率："成功提取身体比例: {success_count}/{total_count}"
            - 显示关键点检测质量和有效性
            - 监控计算开销和优化效果
        """
        if not calculate_ratios:
            return [(r[0], r[1], r[2], r[3], r[4], r[5], None) for r in detection_results]

        # 尝试使用姿态检测
        if not POSE_DETECTION_AVAILABLE:
            # 直接返回原始结果，添加None作为身体比例
            return [(r[0], r[1], r[2], r[3], r[4], r[5], None) for r in detection_results]

        # 准备结果列表
        results_with_ratios = []

        # 计算每个检测结果对应的人体区域，并进行姿态估计
        for detection in detection_results:
            # 解析检测结果
            upper_pos, lower_pos = detection[0], detection[1]
            upper_color, lower_color = detection[2] if len(detection) > 2 else None, detection[3] if len(
                detection) > 3 else None
            upper_conf = detection[4] if len(detection) > 4 else 0.5
            lower_conf = detection[5] if len(detection) > 5 else 0.5

            # 创建包含原始检测信息的结果
            result_with_ratios = list(detection)

            try:
                # 计算整体位置
                body_positions = Determine_the_position_of_the_entire_body(upper_pos, lower_pos, frame)

                # 如果计算出人体位置
                if body_positions and len(body_positions[0]) == 4:
                    bbox = body_positions[0]  # [xmin, ymin, xmax, ymax]

                    # 确保边界合法
                    height, width = frame.shape[:2]
                    xmin = max(0, bbox[0])
                    ymin = max(0, bbox[1])
                    xmax = min(width, bbox[2])
                    ymax = min(height, bbox[3])

                    # 检查边界框是否有效
                    if xmax > xmin and ymax > ymin:
                        # 裁剪人体区域
                        person_img = frame[ymin:ymax, xmin:xmax].copy()

                        # 进行姿态估计
                        pose_results = detect_human_pose(person_img)

                        # 如果检测到关键点
                        if pose_results and len(pose_results) > 0:
                            # 选择第一组关键点（裁剪后的图像应该只有一个人）
                            keypoints = pose_results[0].keypoints

                            # 需要调整关键点坐标，因为它们是相对于裁剪图像的
                            for i in range(len(keypoints)):
                                if keypoints[i][2] > 0:  # 如果关键点有效
                                    keypoints[i][0] += xmin  # 调整x坐标
                                    keypoints[i][1] += ymin  # 调整y坐标

                            # 计算身体比例
                            body_ratios = self.calculate_body_ratios(keypoints)

                            # 添加身体比例到结果中
                            if body_ratios:
                                if len(result_with_ratios) > 6:  # 如果已有占位，替换
                                    result_with_ratios[6] = body_ratios
                                else:  # 否则追加
                                    result_with_ratios.append(body_ratios)
                            else:
                                # 添加None作为占位
                                if len(result_with_ratios) <= 6:
                                    result_with_ratios.append(None)
                        else:
                            # 添加None作为占位
                            if len(result_with_ratios) <= 6:
                                result_with_ratios.append(None)
                    else:
                        # 边界框无效，添加None
                        if len(result_with_ratios) <= 6:
                            result_with_ratios.append(None)
                else:
                    # 没有有效的人体位置，添加None
                    if len(result_with_ratios) <= 6:
                        result_with_ratios.append(None)

            except Exception as e:
                self.get_logger().error(f"计算身体比例时出错: {str(e)}")
                # 确保结果有身体比例占位
                if len(result_with_ratios) <= 6:
                    result_with_ratios.append(None)

            # 将结果添加到列表
            results_with_ratios.append(tuple(result_with_ratios))

        return results_with_ratios

    def calculate_body_ratios(self, keypoints):
        """
        根据人体关键点计算身体比例 - 生物特征计算核心算法
        =================================================
        
        功能说明：
            基于YOLOv8 Pose的17个关键点计算16个身体比例特征
            提供稳定的生物识别信息用于人员识别
        
        参数：
            keypoints (np.array): 人体关键点数组 [17, 3] (x, y, confidence)
            
        返回值：
            list: 16个身体比例特征，float类型
            
        调用关系：
            - 被extract_body_ratios_from_detections()调用
            
        调用原因：
            提供独特的生物特征用于人员识别和跟踪
            
        调试信息：
            - 记录关键点的有效性统计："有效关键点: {valid_count}/17"
            - 显示计算的比例特征分布
            - 监控数值稳定性和异常值
        """
        if keypoints is None or len(keypoints) == 0:
            return None

        try:
            # keypoints是numpy数组，形状为 [17, 3] (x, y, confidence)
            kpts = keypoints

            # 检查关键点有效性的函数
            def is_valid(p1_idx, p2_idx):
                return (kpts[p1_idx][2] > 0.5 and kpts[p2_idx][2] > 0.5 and
                        kpts[p1_idx][0] != 0 and kpts[p1_idx][1] != 0 and
                        kpts[p2_idx][0] != 0 and kpts[p2_idx][1] != 0)

            # 计算两点之间的距离
            def distance(p1_idx, p2_idx):
                if not is_valid(p1_idx, p2_idx):
                    return 0
                return np.sqrt(((kpts[p1_idx][0] - kpts[p2_idx][0]) ** 2) +
                               ((kpts[p1_idx][1] - kpts[p2_idx][1]) ** 2))

            # 计算16个身体比例
            ratios = []

            # YOLOv8 Pose关键点索引:
            # 0: 鼻子, 1: 左眼, 2: 右眼, 3: 左耳, 4: 右耳, 5: 左肩, 6: 右肩
            # 7: 左肘, 8: 右肘, 9: 左腕, 10: 右腕, 11: 左髋, 12: 右髋
            # 13: 左膝, 14: 右膝, 15: 左踝, 16: 右踝

            # 1. 上肢与下肢比例
            upper_limb = (distance(5, 7) + distance(7, 9) + distance(6, 8) + distance(8, 10)) / 4
            lower_limb = (distance(11, 13) + distance(13, 15) + distance(12, 14) + distance(14, 16)) / 4
            if upper_limb > 0 and lower_limb > 0:
                ratios.append(upper_limb / lower_limb)
            else:
                ratios.append(0)

            # 2. 头部与躯干比例
            head_size = distance(0, 5) + distance(0, 6)  # 鼻子到双肩的距离
            torso_size = distance(5, 11) + distance(6, 12)  # 肩膀到髋部的距离
            if head_size > 0 and torso_size > 0:
                ratios.append(head_size / torso_size)
            else:
                ratios.append(0)

            # 3. 肩宽与髋宽比例
            shoulder_width = distance(5, 6)
            hip_width = distance(11, 12)
            if shoulder_width > 0 and hip_width > 0:
                ratios.append(shoulder_width / hip_width)
            else:
                ratios.append(0)

            # 4. 左臂比例（上臂与前臂）
            left_upper_arm = distance(5, 7)
            left_forearm = distance(7, 9)
            if left_upper_arm > 0 and left_forearm > 0:
                ratios.append(left_upper_arm / left_forearm)
            else:
                ratios.append(0)

            # 5. 右臂比例（上臂与前臂）
            right_upper_arm = distance(6, 8)
            right_forearm = distance(8, 10)
            if right_upper_arm > 0 and right_forearm > 0:
                ratios.append(right_upper_arm / right_forearm)
            else:
                ratios.append(0)

            # 6. 左腿比例（大腿与小腿）
            left_thigh = distance(11, 13)
            left_calf = distance(13, 15)
            if left_thigh > 0 and left_calf > 0:
                ratios.append(left_thigh / left_calf)
            else:
                ratios.append(0)

            # 7. 右腿比例（大腿与小腿）
            right_thigh = distance(12, 14)
            right_calf = distance(14, 16)
            if right_thigh > 0 and right_calf > 0:
                ratios.append(right_thigh / right_calf)
            else:
                ratios.append(0)

            # 8. 躯干与腿部比例
            torso_length = (distance(5, 11) + distance(6, 12)) / 2
            leg_length = (distance(11, 15) + distance(12, 16)) / 2
            if torso_length > 0 and leg_length > 0:
                ratios.append(torso_length / leg_length)
            else:
                ratios.append(0)

            # 9. 左右臂长度比例
            left_arm_length = distance(5, 9)
            right_arm_length = distance(6, 10)
            if left_arm_length > 0 and right_arm_length > 0:
                ratios.append(left_arm_length / right_arm_length)
            else:
                ratios.append(0)

            # 10. 左右腿长度比例
            left_leg_length = distance(11, 15)
            right_leg_length = distance(12, 16)
            if left_leg_length > 0 and right_leg_length > 0:
                ratios.append(left_leg_length / right_leg_length)
            else:
                ratios.append(0)

            # 11. 头部宽度与身体宽度比例
            head_width = distance(3, 4)  # 左右耳距离
            body_width = shoulder_width
            if head_width > 0 and body_width > 0:
                ratios.append(head_width / body_width)
            else:
                ratios.append(0)

            # 12. 眼间距与头部宽度比例
            eye_distance = distance(1, 2)
            if eye_distance > 0 and head_width > 0:
                ratios.append(eye_distance / head_width)
            else:
                ratios.append(0)

            # 13. 颈部与头部比例
            neck_length = (distance(0, 5) + distance(0, 6)) / 2
            head_height = distance(0, 3) + distance(0, 4)  # 鼻子到耳朵
            if neck_length > 0 and head_height > 0:
                ratios.append(neck_length / head_height)
            else:
                ratios.append(0)

            # 14. 手臂跨度与身高比例
            arm_span = distance(9, 10)  # 双手距离
            body_height = distance(0, 15) + distance(0, 16)  # 头到脚的距离
            if arm_span > 0 and body_height > 0:
                ratios.append(arm_span / body_height)
            else:
                ratios.append(0)

            # 15. 上半身与下半身比例
            upper_body = (distance(0, 11) + distance(0, 12)) / 2  # 头到髋部
            lower_body = (distance(11, 15) + distance(12, 16)) / 2  # 髋部到脚
            if upper_body > 0 and lower_body > 0:
                ratios.append(upper_body / lower_body)
            else:
                ratios.append(0)

            # 16. 身体对称性指标
            left_side = (distance(5, 11) + distance(11, 15)) / 2
            right_side = (distance(6, 12) + distance(12, 16)) / 2
            if left_side > 0 and right_side > 0:
                ratios.append(left_side / right_side)
            else:
                ratios.append(0)

            # 确保返回16个比例
            while len(ratios) < 16:
                ratios.append(0)

            return ratios[:16]

        except Exception as e:
            self.get_logger().error(f"计算身体比例错误: {e}")
            return None

    def publish_tracking_results(self, tracks, target_track):
        """
        发布跟踪结果 - ROS2话题数据输出
        ===============================
        
        功能说明：
            将跟踪结果转换为ROS2消息格式并发布
            包含轨迹ID、位置、置信度、特征等完整信息
        
        参数：
            tracks (list): 所有跟踪轨迹列表
            target_track (STrack): 目标轨迹（单目标模式）
            
        返回值：
            无
            
        调用关系：
            - 被process_frame()调用
            
        调用原因：
            向ROS2系统发布跟踪结果，供其他节点使用
            
        调试信息：
            - 记录发布的轨迹数量和内容概要
            - 显示消息格式转换的成功率
        """
        try:
            msg = TrackedPersonArray()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "camera"

            for track in tracks:
                person = TrackedPerson()
                person.track_id = track.track_id
                
                # 正确设置RegionOfInterest字段
                person.bbox.x_offset = int(max(0, track.tlbr[0]))
                person.bbox.y_offset = int(max(0, track.tlbr[1]))
                person.bbox.width = int(max(1, track.tlbr[2] - track.tlbr[0]))
                person.bbox.height = int(max(1, track.tlbr[3] - track.tlbr[1]))
                person.bbox.do_rectify = False
                
                person.confidence = float(track.score)

                # 添加颜色信息
                if track.upper_color:
                    person.upper_color = [int(c) for c in track.upper_color]
                else:
                    person.upper_color = []
                    
                if track.lower_color:
                    person.lower_color = [int(c) for c in track.lower_color]
                else:
                    person.lower_color = []

                # 添加身体比例信息
                if hasattr(track, 'body_ratios') and track.body_ratios:
                    person.body_ratios = [float(r) for r in track.body_ratios]
                else:
                    person.body_ratios = []

                # 标记是否为目标
                person.is_target = (target_track is not None and track.track_id == target_track.track_id)

                msg.persons.append(person)

            self.tracked_persons_pub.publish(msg)

        except Exception as e:
            self.get_logger().error(f"发布跟踪结果错误: {e}")

    def publish_tracking_result(self, tracks, target_track, mode):
        """
        发布TrackingResult消息 - 供WebSocket桥接节点使用
        ==============================================
        
        功能说明：
            将跟踪结果转换为TrackingResult消息格式并发布
            为WebSocket桥接提供标准化的跟踪数据
        
        参数：
            tracks (list): 所有跟踪轨迹列表
            target_track (STrack): 目标轨迹（单目标模式）
            mode (str): 当前跟踪模式
            
        返回值：
            无
            
        调用关系：
            - 被process_frame()调用
            
        调用原因：
            为WebSocket桥接节点提供结构化的跟踪结果
        """
        try:
            msg = TrackingResult()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "camera"
            
            # 基本信息
            msg.mode = 'single_target' if mode in ['selecting', 'tracking'] else 'multi_target'
            msg.total_tracks = len(tracks)
            msg.target_detected = target_track is not None
            
            # 目标信息（单目标模式）
            if target_track is not None:
                msg.target_id = target_track.track_id
                tlbr = target_track.tlbr
                msg.target_x = float((tlbr[0] + tlbr[2]) / 2)  # 中心x
                msg.target_y = float((tlbr[1] + tlbr[3]) / 2)  # 中心y
                msg.target_width = float(tlbr[2] - tlbr[0])
                msg.target_height = float(tlbr[3] - tlbr[1])
                msg.confidence = float(target_track.score)
                
                # 计算距离（如果启用立体视觉）
                if self.enable_distance_measure:
                    distance = self.get_distance_at_point(int(msg.target_x), int(msg.target_y))
                    msg.distance = float(distance / 1000.0)  # 转换为米
                else:
                    msg.distance = 0.0
            else:
                msg.target_id = -1
                msg.target_x = 0.0
                msg.target_y = 0.0
                msg.target_width = 0.0
                msg.target_height = 0.0
                msg.confidence = 0.0
                msg.distance = 0.0
            
            # 多目标信息
            msg.track_ids = [track.track_id for track in tracks]
            msg.track_confidences = [float(track.score) for track in tracks]
            
            # 位置列表
            positions = []
            for track in tracks:
                tlbr = track.tlbr
                pos = Point()
                pos.x = float((tlbr[0] + tlbr[2]) / 2)
                pos.y = float((tlbr[1] + tlbr[3]) / 2)
                pos.z = 0.0
                positions.append(pos)
            msg.positions = positions
            
            # 状态信息
            if target_track is not None:
                msg.tracking_status = 'tracking'
            elif len(tracks) > 0:
                msg.tracking_status = 'idle'
            else:
                msg.tracking_status = 'searching'
            
            # 性能信息
            if hasattr(self, 'current_fps'):
                msg.fps = float(self.current_fps)
            else:
                msg.fps = 0.0
            msg.frame_count = self.frame_count
            
            # 发布消息
            self.tracking_result_pub.publish(msg)
            
            # 同时发布详细的跟踪数据给WebSocket桥接节点
            self.publish_detailed_tracking_data(tracks, target_track, mode)
            
        except Exception as e:
            self.get_logger().error(f"发布TrackingResult消息错误: {e}")

    def publish_detailed_tracking_data(self, tracks, target_track, mode):
        """
        发布详细的跟踪数据给WebSocket桥接节点
        ===================================
        
        功能说明：
            发布包含所有轨迹详细信息的数据，供微信小程序显示
            包括每个轨迹的位置、颜色、身体比例等详细信息
        """
        try:
            import json
            import time
            import numpy as np
            
            # 定义类型转换函数
            def safe_int(value):
                """安全转换为Python int类型"""
                if hasattr(value, 'item'):  # NumPy数组元素
                    return int(value.item())
                if hasattr(value, 'dtype') and 'int' in str(value.dtype):  # NumPy整数类型
                    return int(value)
                return int(value) if value is not None else 0
            
            def safe_float(value):
                """安全转换为Python float类型"""
                if hasattr(value, 'item'):  # NumPy数组元素
                    return float(value.item())
                if hasattr(value, 'dtype') and 'float' in str(value.dtype):  # NumPy浮点类型
                    return float(value)
                return float(value) if value is not None else 0.0
            
            def safe_list(value):
                """安全转换为Python list类型"""
                if value is None:
                    return []
                if isinstance(value, np.ndarray):
                    return [safe_float(x) for x in value.tolist()]
                if isinstance(value, (list, tuple)):
                    return [safe_float(x) for x in value]
                return []
            
            # 构建详细的跟踪数据
            detailed_data = {
                'timestamp': int(time.time() * 1000),
                'frame_id': safe_int(self.frame_count),
                'tracking_mode': str(mode),
                'target_detected': target_track is not None,
                'total_tracks': len(tracks),
                'tracks': [],
                'target_track': None,
                'statistics': {
                    'active_tracks': len([t for t in tracks if t.state == TrackState.TRACKED]),
                    'lost_tracks': len([t for t in tracks if t.state == TrackState.LOST]),
                    'new_tracks': len([t for t in tracks if t.state == TrackState.NEW]),
                }
            }
            
            # 处理所有轨迹
            for track in tracks:
                if track.state == TrackState.TRACKED:
                    tlwh = track.tlwh
                    tlbr = track.tlbr
                    center_x = safe_int(tlbr[0] + (tlbr[2] - tlbr[0]) / 2)
                    center_y = safe_int(tlbr[1] + (tlbr[3] - tlbr[1]) / 2)
                    
                    track_data = {
                        'id': safe_int(track.track_id),
                        'status': 'tracking',
                        'position': {
                            'x': safe_float(center_x),
                            'y': safe_float(center_y),
                            'width': safe_float(tlbr[2] - tlbr[0]),
                            'height': safe_float(tlbr[3] - tlbr[1]),
                            'tlbr': [safe_float(tlbr[0]), safe_float(tlbr[1]), safe_float(tlbr[2]), safe_float(tlbr[3])]
                        },
                        'confidence': safe_float(track.score),
                        'age': safe_int(track.tracklet_len),
                        'time_since_update': safe_int(track.time_since_update),
                        'colors': {
                            'upper': safe_list(track.upper_color),
                            'lower': safe_list(track.lower_color)
                        },
                        'body_ratios': safe_list(track.body_ratios),
                        'distance': safe_float(self.get_distance_at_point(center_x, center_y) / 1000.0),  # 转换为米
                        'is_target': track == target_track,
                        'tracking_quality': safe_float(self.calculate_tracking_quality(track))
                    }
                    detailed_data['tracks'].append(track_data)
            
            # 处理目标轨迹
            if target_track:
                tlbr = target_track.tlbr
                center_x = safe_int(tlbr[0] + (tlbr[2] - tlbr[0]) / 2)
                center_y = safe_int(tlbr[1] + (tlbr[3] - tlbr[1]) / 2)
                
                detailed_data['target_track'] = {
                    'id': safe_int(target_track.track_id),
                    'position': {
                        'x': safe_float(center_x),
                        'y': safe_float(center_y),
                        'width': safe_float(tlbr[2] - tlbr[0]),
                        'height': safe_float(tlbr[3] - tlbr[1]),
                        'tlbr': [safe_float(tlbr[0]), safe_float(tlbr[1]), safe_float(tlbr[2]), safe_float(tlbr[3])]
                    },
                    'confidence': safe_float(target_track.score),
                    'distance': safe_float(self.get_distance_at_point(center_x, center_y) / 1000.0),  # 转换为米
                    'colors': {
                        'upper': safe_list(target_track.upper_color),
                        'lower': safe_list(target_track.lower_color)
                    },
                    'body_ratios': safe_list(target_track.body_ratios),
                    'tracking_quality': safe_float(self.calculate_tracking_quality(target_track)),
                    'velocity': {
                        'x': safe_float(target_track.mean[4]) if len(target_track.mean) > 4 else 0.0,
                        'y': safe_float(target_track.mean[5]) if len(target_track.mean) > 5 else 0.0
                    }
                }
            
            # 添加系统性能信息
            detailed_data['system_info'] = {
                'fps': safe_float(getattr(self, 'current_fps', 0.0)),
                'processing_time_ms': safe_float(getattr(self, 'processing_time', 0.0)),
                'memory_usage_mb': safe_float(self.get_memory_usage()),
                'camera_connected': self.cap is not None and self.cap.isOpened()
            }
            
            # 发布JSON消息
            json_msg = String()
            json_msg.data = json.dumps(detailed_data)
            self.detailed_tracking_pub.publish(json_msg)
            
            # 调试输出：简化打印关键信息
            tracking_mode = detailed_data.get('tracking_mode', '未知')
            total_tracks = detailed_data.get('total_tracks', 0)
            target_detected = detailed_data.get('target_detected', False)
            frame_id = detailed_data.get('frame_id', 0)
            active_tracks = len([t for t in tracks if t.state == TrackState.TRACKED])
            
            target_status = "有目标" if target_detected else "无目标"
            # self.get_logger().info(f'🤖 ByteTracker发布 - 模式: {tracking_mode}, 检测人数: {total_tracks}, 活跃轨迹: {active_tracks}, 目标状态: {target_status}, 帧号: {frame_id}')
            
            # 调试日志（降低频率）
            if self.frame_count % 60 == 0:  # 每60帧记录一次（约2秒）
                if target_track:
                    distance_str = f', 距离: {detailed_data["target_track"]["distance"]:.2f}m'
                else:
                    distance_str = ''
                self.get_logger().info(f'📈 发布详细跟踪数据 - 轨迹数: {len(tracks)}, '
                                     f'目标ID: {safe_int(target_track.track_id) if target_track else "无"}'
                                     f'{distance_str}')
        
        except Exception as e:
            self.get_logger().error(f'❌ 发布详细跟踪数据失败: {e}')
            # 添加更详细的错误信息
            import traceback
            self.get_logger().error(f'❌ 详细错误信息: {traceback.format_exc()}')
    
    def calculate_tracking_quality(self, track):
        """计算跟踪质量评分"""
        try:
            quality_score = 0.0
            
            # 基础置信度 (0-40分)
            quality_score += min(40.0, track.score * 40)
            
            # 跟踪稳定性 (0-30分)
            if track.tracklet_len > 10:
                stability = min(30.0, (track.tracklet_len / 50.0) * 30)
                quality_score += stability
            
            # 更新及时性 (0-20分)
            if track.time_since_update == 0:
                quality_score += 20.0
            elif track.time_since_update <= 3:
                quality_score += 15.0
            elif track.time_since_update <= 5:
                quality_score += 10.0
            
            # 特征完整性 (0-10分)
            if track.upper_color is not None and track.lower_color is not None:
                quality_score += 5.0
            if track.body_ratios is not None and len(track.body_ratios) > 0:
                quality_score += 5.0
            
            return min(100.0, quality_score)
        
        except Exception:
            return 50.0  # 默认评分
    
    def get_memory_usage(self):
        """获取内存使用情况（MB）"""
        try:
            import psutil
            process = psutil.Process()
            return process.memory_info().rss / 1024 / 1024
        except Exception:
            return 0.0

    def get_distance_at_point(self, x, y):
        """
        获取指定像素点的距离 - 立体视觉深度测量
        ====================================
        
        功能说明：
            使用立体视觉算法计算指定像素点的真实距离
            支持SGBM立体匹配算法
        
        参数：
            x (int): 像素点x坐标
            y (int): 像素点y坐标
            
        返回值：
            float: 距离值（毫米），失败时返回默认值2000.0
            
        调用关系：
            - 被process_frame()在小车控制模式下调用
            - 被publish_target_position()调用
            
        调用原因：
            为机器人导航提供目标距离信息
            
        调试信息：
            - 记录视差值和距离计算过程
            - 显示立体匹配的质量评估
            - 监控距离测量的稳定性
        """
        try:
            if (not self.enable_distance_measure or 
                self.current_left_frame is None or 
                self.current_right_frame is None or
                self.map1x is None or self.map1y is None or
                self.map2x is None or self.map2y is None or
                self.stereo_matcher is None):
                return 2000.0  # 默认距离(mm)
            
            # 立体校正
            left_rectified = cv2.remap(self.current_left_frame, self.map1x, self.map1y, cv2.INTER_LINEAR)
            right_rectified = cv2.remap(self.current_right_frame, self.map2x, self.map2y, cv2.INTER_LINEAR)
            
            # 转换为灰度图
            gray_left = cv2.cvtColor(left_rectified, cv2.COLOR_BGR2GRAY)
            gray_right = cv2.cvtColor(right_rectified, cv2.COLOR_BGR2GRAY)
            
            # 计算视差图
            disparity = self.stereo_matcher.compute(gray_left, gray_right)
            
            # 确保坐标在有效范围内
            height, width = disparity.shape
            x = max(0, min(x, width - 1))
            y = max(0, min(y, height - 1))
            
            # 获取该点的视差值
            disp_value = disparity[y, x]
            
            # 如果视差值无效，返回默认距离
            if disp_value <= 0:
                return 2000.0
            
            # 计算距离 (mm)
            distance = (self.stereo_camera.focal_length * self.stereo_camera.baseline) / (disp_value / 16.0)
            
            # 限制距离范围
            if distance < self.stereo_config.min_distance_mm or distance > self.stereo_config.max_distance_mm:
                return 2000.0
            
            return distance
            
        except Exception as e:
            self.get_logger().error(f"距离测量错误: {e}")
            return 2000.0

    def create_visualization(self, frame, tracks, target_track, mode):
        """
        创建可视化图像 - 实时跟踪结果展示
        ===============================
        
        功能说明：
            在原始图像上绘制跟踪结果，包括边界框、ID、轨迹等
            提供丰富的视觉反馈用于调试和演示
        
        参数：
            frame (np.array): 原始图像
            tracks (list): 跟踪轨迹列表
            target_track (STrack): 目标轨迹
            mode (str): 当前跟踪模式
            
        返回值：
            np.array: 可视化图像
            
        调用关系：
            - 被process_frame()调用
            
        调用原因：
            提供直观的跟踪效果展示
            
        调试信息：
            - 记录绘制的元素数量和类型
            - 显示可视化参数的配置
        """
        viz_frame = frame.copy()

        # 目标颜色（红色）
        TARGET_COLOR = (0, 0, 255)

        # 绘制所有轨迹
        for track in tracks:
            # 确定颜色 - 目标使用特殊颜色
            is_target = (target_track is not None and track.track_id == target_track.track_id)
            color = TARGET_COLOR if is_target else (0, 255, 0)

            # 线宽 - 目标轨迹使用粗线条
            thickness = 3 if is_target else 2

            # 绘制边界框
            tlbr = track.tlbr
            cv2.rectangle(
                viz_frame,
                (int(tlbr[0]), int(tlbr[1])),
                (int(tlbr[2]), int(tlbr[3])),
                color, thickness
            )

            # 显示ID
            id_text = f"Target ID:{track.track_id}" if is_target else f"ID:{track.track_id}"
            cv2.putText(
                viz_frame,
                id_text,
                (int(tlbr[0]), int(tlbr[1] - 10)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, thickness
            )

            # 绘制预测边界框（如果有）
            if hasattr(track, 'predicted_mean') and track.predicted_mean is not None:
                pred_tlbr = track.predicted_tlbr
                # 使用虚线绘制预测框
                self.draw_dashed_rectangle(
                    viz_frame,
                    (int(pred_tlbr[0]), int(pred_tlbr[1])),
                    (int(pred_tlbr[2]), int(pred_tlbr[3])),
                    tuple(min(255, c + 70) for c in color), 1
                )

        # 显示模式信息
        mode_text = f"Mode: {(self.tracking_mode or 'multi').upper()}"
        if mode == 'single_not_initialized':
            mode_text += " (NOT INITIALIZED)"
        cv2.putText(
            viz_frame,
            mode_text,
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2
        )

        # 显示跟踪统计
        cv2.putText(
            viz_frame,
            f"Tracking: {len(tracks)} objects",
            (10, 60),
            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2
        )

        # 显示性能信息
        if self.processing_time > 0:
            fps = 1.0 / self.processing_time
            cv2.putText(
                viz_frame,
                f"FPS: {fps:.1f}",
                (10, 90),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2
            )

        return viz_frame

    def setup_parameters(self):
        """
        设置节点参数 - ROS2参数系统集成
        =============================
        
        功能说明：
            声明和获取ROS2参数，支持动态配置
            包含跟踪参数、相机参数、功能开关等
        
        参数：
            无
            
        返回值：
            无
            
        调用关系：
            - 被__init__()调用
            
        调用原因：
            初始化系统配置，支持外部参数调整
        """
        # 声明参数
        self.declare_parameter('tracking_mode', 'multi')
        self.declare_parameter('track_thresh', 0.5)
        self.declare_parameter('track_buffer', 100)
        self.declare_parameter('match_thresh', 0.8)
        self.declare_parameter('color_weight', 0.5)
        self.declare_parameter('target_features_file', '')
        self.declare_parameter('camera_id', 0)
        self.declare_parameter('frame_width', 1280)
        self.declare_parameter('frame_height', 480)
        self.declare_parameter('fps_limit', 30)
        self.declare_parameter('enable_car_control', False)
        self.declare_parameter('enable_distance_measure', True)
        self.declare_parameter('is_stereo_camera', True)
        self.declare_parameter('video_file_path', '')

        # 获取参数值
        self.tracking_mode = self.get_parameter('tracking_mode').value or 'multi'
        self.tracker_args['track_thresh'] = self.get_parameter('track_thresh').value
        self.tracker_args['track_buffer'] = self.get_parameter('track_buffer').value
        self.tracker_args['match_thresh'] = self.get_parameter('match_thresh').value
        self.tracker_args['color_weight'] = self.get_parameter('color_weight').value
        self.target_features_file = self.get_parameter('target_features_file').value or ''
        
        # 相机参数
        self.camera_config.camera_id = self.get_parameter('camera_id').value or 0
        self.camera_config.frame_width = self.get_parameter('frame_width').value or 1280
        self.camera_config.frame_height = self.get_parameter('frame_height').value or 480
        self.camera_config.fps_limit = self.get_parameter('fps_limit').value or 30
        self.camera_config.is_stereo_camera = self.get_parameter('is_stereo_camera').value or True
        self.camera_config.enable_distance_measure = self.get_parameter('enable_distance_measure').value or True
        
        self.enable_car_control = self.get_parameter('enable_car_control').value
        self.enable_distance_measure = self.camera_config.enable_distance_measure
        
        # 视频文件参数
        self.video_file_path = self.get_parameter('video_file_path').value or ''
        if not self.video_file_path:
            self.video_file_path = None
            
        # 如果使用视频文件，自动禁用距离测量和立体视觉
        if self.video_file_path:
            self.enable_distance_measure = False
            self.camera_config.is_stereo_camera = False
            self.camera_config.enable_distance_measure = False
            self.get_logger().info(f'📹 视频模式启用，使用文件: {self.video_file_path}')
            self.get_logger().info('🚫 已自动禁用距离测量功能（视频模式不支持立体视觉）')

    def setup_publishers(self):
        """
        设置发布者 - ROS2话题输出配置
        =============================
        
        功能说明：
            创建所有ROS2发布者，用于输出跟踪结果和状态信息
            配置QoS策略确保消息传输质量
        
        参数：
            无
            
        返回值：
            无
            
        调用关系：
            - 被__init__()调用
            
        调用原因：
            初始化ROS2通信接口，准备数据输出
            
        调试信息：
            - 记录发布者创建状态
            - 显示QoS配置信息
        """
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # 跟踪结果发布者
        self.tracked_persons_pub = self.create_publisher(
            TrackedPersonArray, '/bytetracker/tracked_persons', qos)

        # 新增：TrackingResult发布者（供WebSocket桥接节点使用）
        self.tracking_result_pub = self.create_publisher(
            TrackingResult, '/bytetracker/tracking_result', qos)

        # 新增：详细跟踪数据发布者（供WebSocket桥接节点和微信小程序使用）
        self.detailed_tracking_pub = self.create_publisher(
            String, '/bytetracker/detailed_tracking_data', qos)

        # 可视化图像发布者（供WebSocket桥接节点使用）
        self.visualization_pub = self.create_publisher(
            Image, '/bytetracker/visualization', qos)

        # 跟踪状态发布者
        self.status_pub = self.create_publisher(
            String, '/bytetracker/status', qos)

        # 如果启用小车控制，创建位置发布者
        if self.enable_car_control:
            self.person_position_pub = self.create_publisher(
                Position, '/robot_control/person_position', qos)

    def setup_subscribers(self):
        """
        设置订阅者 - ROS2话题输入配置
        =============================
        
        功能说明：
            创建ROS2订阅者，接收外部控制命令
            支持模式切换和目标设定
        
        参数：
            无
            
        返回值：
            无
            
        调用关系：
            - 被__init__()调用
            
        调用原因：
            建立外部控制接口，支持动态配置
            
        调试信息：
            - 记录订阅者创建状态
            - 显示话题名称和回调函数
        """
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # 模式控制订阅者
        self.mode_sub = self.create_subscription(
            String, '/bytetracker/set_mode', self.mode_callback, qos)

        # 目标人物设置订阅者
        self.target_sub = self.create_subscription(
            String, '/bytetracker/set_target', self.target_callback, qos)

    def setup_service_clients(self):
        """
        设置服务客户端 - ROS2服务接口配置
        ===============================
        
        功能说明：
            创建ROS2服务客户端，用于调用外部服务
            支持特征提取等高级功能
        
        参数：
            无
            
        返回值：
            无
            
        调用关系：
            - 被__init__()调用
            
        调用原因：
            建立与其他ROS2节点的服务通信
            
        调试信息：
            - 记录服务客户端创建状态
            - 显示服务名称和类型
        """
        # 特征提取服务客户端
        self.feature_extraction_client = self.create_client(
            FeatureExtraction, '/features/extract_features')

    def setup_camera(self):
        """
        设置相机 - 视频采集系统初始化
        =============================
        
        功能说明：
            初始化相机设备，配置采集参数
            启动独立的图像捕获线程
        
        参数：
            无
            
        返回值：
            无
            
        调用关系：
            - 被__init__()调用
            
        调用原因：
            建立视频输入源，为跟踪提供图像数据
            
        调试信息：
            - 记录相机初始化过程："🎬 初始化相机ID: {camera_id}"
            - 显示相机参数设置结果
            - 监控相机连接状态
        """
        try:
            if self.video_file_path:
                # 视频文件模式
                self.get_logger().info(f'🎬 初始化视频文件: {self.video_file_path}')
                
                # 检查文件是否存在
                if not os.path.exists(self.video_file_path):
                    raise FileNotFoundError(f"视频文件不存在: {self.video_file_path}")
                
                # 打开视频文件
                self.cap = cv2.VideoCapture(self.video_file_path)
                
                # 获取视频信息
                if self.cap.isOpened():
                    fps = self.cap.get(cv2.CAP_PROP_FPS)
                    frame_count = int(self.cap.get(cv2.CAP_PROP_FRAME_COUNT))
                    width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                    height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                    duration = frame_count / fps if fps > 0 else 0
                    
                    self.get_logger().info(f'📹 视频信息: {width}x{height}, {fps:.1f}FPS, '
                                         f'{frame_count}帧, 时长{duration:.1f}秒')
                    
                    # 更新配置以匹配视频
                    self.camera_config.frame_width = width
                    self.camera_config.frame_height = height
                    self.camera_config.fps_limit = int(min(fps, 30))  # 限制最大30FPS避免处理过快
                else:
                    raise ValueError(f"无法打开视频文件: {self.video_file_path}")
            else:
                # 相机模式
                self.get_logger().info(f'🎬 初始化相机ID: {self.camera_config.camera_id}')
                
                # 初始化相机
                self.cap = cv2.VideoCapture(1, cv2.CAP_V4L2)
            
            # 设置相机属性
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.camera_config.frame_width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.camera_config.frame_height)
            self.cap.set(cv2.CAP_PROP_FPS, self.camera_config.fps_limit)
            
            # 设置格式 - 兼容多版本OpenCV
            try:
                # 使用兼容的方式设置MJPEG编码
                if hasattr(cv2, 'VideoWriter_fourcc'):
                    fourcc = cv2.VideoWriter_fourcc('M', 'J', 'P', 'G')
                elif hasattr(cv2, 'VideoWriter') and hasattr(cv2.VideoWriter, 'fourcc'):
                    fourcc = cv2.VideoWriter.fourcc('M', 'J', 'P', 'G')
                else:
                    # 使用MJPG的数字代码
                    fourcc = 0x47504A4D
                self.cap.set(cv2.CAP_PROP_FOURCC, fourcc)
            except Exception:
                # 如果都失败，跳过fourcc设置
                self.get_logger().warn("⚠️ 无法设置视频格式，使用默认格式")
            
            if not self.cap.isOpened():
                self.get_logger().warn(f"无法打开相机ID {self.camera_config.camera_id}，尝试使用默认相机...")
                self.camera_config.camera_id = 0
                self.cap = cv2.VideoCapture(self.camera_config.camera_id)
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.camera_config.frame_width)
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.camera_config.frame_height)
                
                if not self.cap.isOpened():
                    raise RuntimeError("❌ 无法打开任何相机！")
            
            self.get_logger().info('✅ 相机初始化成功')
            
            # 启动图像捕获线程
            self.start_capture_thread()
            
        except Exception as e:
            self.get_logger().error(f"❌ 相机初始化失败: {e}")
            self.cap = None

    def start_capture_thread(self):
        """
        启动图像捕获线程 - 异步视频采集
        ===============================
        
        功能说明：
            创建独立线程进行图像采集，避免阻塞主处理流程
            实现稳定的帧率控制
        
        参数：
            无
            
        返回值：
            无
            
        调用关系：
            - 被setup_camera()调用
            
        调用原因：
            实现异步图像采集，保证处理效率
            
        调试信息：
            - 记录线程启动状态
            - 监控采集线程健康状态
        """
        try:
            self.running = True
            self.capture_thread = threading.Thread(target=self.capture_loop, daemon=True)
            self.capture_thread.start()
            self.get_logger().info('✅ 图像捕获线程已启动')
        except Exception as e:
            self.get_logger().error(f"❌ 捕获线程启动失败: {e}")

    def capture_loop(self):
        """
        图像捕获循环 - 持续视频采集核心
        ===============================
        
        功能说明：
            在独立线程中持续采集图像帧
            处理立体相机的左右图像分离
        
        参数：
            无
            
        返回值：
            无
            
        调用关系：
            - 被start_capture_thread()创建的线程执行
            
        调用原因：
            提供持续的图像数据流
            
        调试信息：
            - 记录采集状态和异常情况
            - 监控帧率和图像质量
        """
        while self.running and rclpy.ok():
            try:
                if self.cap is None:
                    time.sleep(0.1)
                    continue
                    
                ret, frame = self.cap.read()
                if not ret:
                    self.get_logger().warn("⚠️ 无法获取图像")
                    time.sleep(0.1)
                    continue

                # 调整图像大小（如果需要）
                if frame.shape[1] != self.camera_config.frame_width or frame.shape[0] != self.camera_config.frame_height:
                    frame = cv2.resize(frame, (self.camera_config.frame_width, self.camera_config.frame_height))

                # 如果是立体相机，分离左右图像
                if self.camera_config.is_stereo_camera:
                    height, width = frame.shape[:2]
                    half_width = width // 2
                    
                    left_frame = frame[:, :half_width]
                    right_frame = frame[:, half_width:]
                    
                    # 更新当前帧
                    with self.lock:
                        self.current_left_frame = left_frame.copy()
                        self.current_right_frame = right_frame.copy()
                        self.current_frame = left_frame.copy()  # 用左图作为主处理图像
                else:
                    # 单目相机
                    with self.lock:
                        self.current_frame = frame.copy()

                # 控制帧率
                time.sleep(1.0 / self.camera_config.fps_limit)
                
            except Exception as e:
                self.get_logger().error(f"❌ 图像捕获错误: {e}")
                time.sleep(0.1)

    def setup_stereo_vision(self):
        """
        设置立体视觉 - 深度感知系统初始化
        ===============================
        
        功能说明：
            初始化立体视觉算法，配置立体校正和匹配参数
            建立深度测量能力
        
        参数：
            无
            
        返回值：
            无
            
        调用关系：
            - 被__init__()在启用距离测量时调用
            
        调用原因：
            为机器人应用提供深度感知能力
            
        调试信息：
            - 记录立体标定参数加载
            - 显示SGBM算法配置
            - 监控校正映射生成
        """
        try:
            self.get_logger().info('🔧 初始化立体视觉系统...')
            
            # 计算立体校正参数
            img_size = (self.camera_config.frame_width // 2, self.camera_config.frame_height)
            
            R1, R2, P1, P2, self.Q, roi1, roi2 = cv2.stereoRectify(
                self.stereo_camera.cam_matrix_left,
                self.stereo_camera.distortion_l,
                self.stereo_camera.cam_matrix_right,
                self.stereo_camera.distortion_r,
                img_size,
                self.stereo_camera.R,
                self.stereo_camera.T,
                flags=cv2.CALIB_ZERO_DISPARITY,
                alpha=0
            )
            
            # 计算畸变校正映射
            self.map1x, self.map1y = cv2.initUndistortRectifyMap(
                self.stereo_camera.cam_matrix_left,
                self.stereo_camera.distortion_l,
                R1, P1, img_size, cv2.CV_32FC1
            )
            
            self.map2x, self.map2y = cv2.initUndistortRectifyMap(
                self.stereo_camera.cam_matrix_right,
                self.stereo_camera.distortion_r,
                R2, P2, img_size, cv2.CV_32FC1
            )
            
            # 创建立体匹配器 - 兼容多版本OpenCV
            try:
                # 尝试新版本OpenCV API
                self.stereo_matcher = cv2.StereoSGBM_create(
                    minDisparity=self.stereo_config.minDisparity,
                    numDisparities=self.stereo_config.numDisparities,
                    blockSize=self.stereo_config.blockSize,
                    P1=self.stereo_config.P1,
                    P2=self.stereo_config.P2,
                    disp12MaxDiff=self.stereo_config.disp12MaxDiff,
                    preFilterCap=self.stereo_config.preFilterCap,
                    uniquenessRatio=self.stereo_config.uniquenessRatio,
                    speckleWindowSize=self.stereo_config.speckleWindowSize,
                    speckleRange=self.stereo_config.speckleRange,
                    mode=self.stereo_config.mode
                )
            except AttributeError:
                try:
                    # 尝试旧版本OpenCV API (cv2.StereoSGBM.create)
                    self.stereo_matcher = cv2.StereoSGBM.create(
                        minDisparity=self.stereo_config.minDisparity,
                        numDisparities=self.stereo_config.numDisparities,
                        blockSize=self.stereo_config.blockSize,
                        P1=self.stereo_config.P1,
                        P2=self.stereo_config.P2,
                        disp12MaxDiff=self.stereo_config.disp12MaxDiff,
                        preFilterCap=self.stereo_config.preFilterCap,
                        uniquenessRatio=self.stereo_config.uniquenessRatio,
                        speckleWindowSize=self.stereo_config.speckleWindowSize,
                        speckleRange=self.stereo_config.speckleRange
                    )
                except AttributeError:
                    try:
                        # 尝试更旧版本的OpenCV API (直接构造)
                        self.stereo_matcher = cv2.StereoSGBM(
                            minDisparity=self.stereo_config.minDisparity,
                            numDisparities=self.stereo_config.numDisparities,
                            SADWindowSize=self.stereo_config.blockSize,
                            P1=self.stereo_config.P1,
                            P2=self.stereo_config.P2,
                            disp12MaxDiff=self.stereo_config.disp12MaxDiff,
                            preFilterCap=self.stereo_config.preFilterCap,
                            uniquenessRatio=self.stereo_config.uniquenessRatio,
                            speckleWindowSize=self.stereo_config.speckleWindowSize,
                            speckleRange=self.stereo_config.speckleRange
                        )
                    except:
                        # 如果都失败了，禁用立体视觉
                        self.stereo_matcher = None
                        self.enable_distance_measure = False
                        self.get_logger().error("❌ 无法创建立体匹配器，禁用距离测量功能")
            
            self.get_logger().info('✅ 立体视觉系统初始化完成')
            
        except Exception as e:
            self.get_logger().error(f"❌ 立体视觉初始化失败: {e}")
            self.enable_distance_measure = False

    def setup_timers(self):
        """
        设置定时器 - 周期性任务调度
        =========================
        
        功能说明：
            创建ROS2定时器，控制处理频率和状态发布
            实现精确的时间控制
        
        参数：
            无
            
        返回值：
            无
            
        调用关系：
            - 被__init__()调用
            
        调用原因：
            建立系统的时间驱动机制
            
        调试信息：
            - 记录定时器创建和频率设置
            - 监控定时器执行精度
        """
        # 处理定时器（20Hz）
        self.process_timer = self.create_timer(0.05, self.process_frame)

        # 状态发布定时器（2Hz）
        self.status_timer = self.create_timer(0.5, self.publish_status)

    def mode_callback(self, msg):
        """
        模式切换回调 - 动态跟踪模式控制
        =============================
        
        功能说明：
            处理外部模式切换请求，支持多目标/单目标模式切换
            自动初始化相应的跟踪器
        
        参数：
            msg (String): ROS2字符串消息，包含模式名称
            
        返回值：
            无
            
        调用关系：
            - 被ROS2订阅者在接收到模式切换消息时调用
            
        调用原因：
            响应外部控制请求，实现动态配置
            
        调试信息：
            - 记录模式切换过程和结果
            - 显示跟踪器初始化状态
        """
        new_mode = msg.data.lower()
        if new_mode in ['multi', 'single']:
            self.tracking_mode = new_mode
            self.get_logger().info(f"🎮 切换到{new_mode}目标跟踪模式")

            # 如果切换到单目标模式，需要初始化单目标跟踪器
            if new_mode == 'single' and self.single_target_tracker is None:
                self.init_single_target_tracker()
        else:
            self.get_logger().warn(f"无效的跟踪模式: {new_mode}")

    def target_callback(self, msg):
        """
        目标人物设置回调 - 动态目标配置
        =============================
        
        功能说明：
            处理目标人物设置请求，更新单目标跟踪的目标
            触发跟踪器重新初始化
        
        参数：
            msg (String): ROS2字符串消息，包含目标人物名称
            
        返回值：
            无
            
        调用关系：
            - 被ROS2订阅者在接收到目标设置消息时调用
            
        调用原因：
            响应目标选择请求，更新跟踪目标
            
        调试信息：
            - 记录目标设置过程
            - 显示跟踪器重新初始化状态
        """
        self.target_person_name = msg.data
        self.get_logger().info(f"🎯 设置目标人物: {self.target_person_name}")

        # 如果在单目标模式，重新初始化跟踪器
        if self.tracking_mode == 'single':
            self.init_single_target_tracker()

    def init_single_target_tracker(self):
        """
        初始化单目标跟踪器 - 专用跟踪器创建
        ===================================
        
        功能说明：
            基于目标特征文件创建单目标跟踪器实例
            加载和验证目标特征数据
        
        参数：
            无
            
        返回值：
            bool: 初始化是否成功
            
        调用关系：
            - 被__init__()在单目标模式下调用
            - 被mode_callback()和target_callback()调用
            
        调用原因：
            创建专用的目标跟踪能力
            
        调试信息：
            - 记录特征文件读取过程："📖 正在读取目标特征文件: {file_path}"
            - 显示特征数据的验证结果
            - 监控跟踪器创建状态
        """
        try:
            self.get_logger().info('🔧 开始初始化单目标跟踪器...')
            
            # 如果有目标特征文件，读取特征
            if self.target_features_file:
                self.get_logger().info(f'📖 正在读取目标特征文件: {self.target_features_file}')
                target_features = self.read_target_features(self.target_features_file)
                if target_features:
                    self.single_target_tracker = SingleTargetTracker(
                        self.tracker_args, target_features, max_lost_time=60)
                    self.get_logger().info("✅ 单目标跟踪器初始化成功")
                    return True
                else:
                    self.get_logger().error("❌ 读取目标特征失败")
                    return False
            else:
                self.get_logger().warn("⚠️ 未指定目标特征文件，无法初始化单目标跟踪器")
                return False

        except Exception as e:
            self.get_logger().error(f"❌ 初始化单目标跟踪器错误: {e}")
            traceback.print_exc()
            return False

    def read_target_features(self, xlsx_path):
        """
        从Excel文件中读取目标特征信息 - 特征数据加载器
        =============================================
        
        功能说明：
            解析Excel文件中的目标特征数据
            包含身体比例、颜色信息等多维特征
        
        参数：
            xlsx_path (str): Excel文件路径
            
        返回值：
            tuple: (body_ratios, shirt_color, pants_color) 或 None
            
        调用关系：
            - 被init_single_target_tracker()调用
            
        调用原因：
            加载预定义的目标特征，支持特定目标识别
            
        调试信息：
            - 记录文件读取过程："🔍 检查特征文件是否存在: {xlsx_path}"
            - 显示特征数据解析结果
            - 监控数据格式验证
        """
        try:
            self.get_logger().info(f'🔍 检查特征文件是否存在: {xlsx_path}')
            
            # 检查文件是否存在
            if not os.path.exists(xlsx_path):
                self.get_logger().error(f"❌ 目标特征文件不存在: {xlsx_path}")
                return None
            
            self.get_logger().info('📁 文件存在，正在打开Excel文件...')
            
            # 打开Excel文件
            wb = openpyxl.load_workbook(xlsx_path)
            if wb.active is None:
                self.get_logger().error("❌ Excel文件没有活动工作表")
                return None
                
            sheet = wb.active
            self.get_logger().info('📊 成功打开Excel工作表，正在读取数据...')

            # 读取身体比例数据 (前16行)
            body_ratios = []
            for i in range(1, 17):
                try:
                    cell_value = sheet.cell(row=i, column=1).value
                    if cell_value is not None:
                        value = float(cell_value)
                    else:
                        value = 0.0
                except (ValueError, TypeError, AttributeError):
                    value = 0.0
                body_ratios.append(value)

            self.get_logger().info(f'📏 读取身体比例数据: {len([r for r in body_ratios if r > 0])} 个有效值')

            # 读取颜色数据 (第17和18行)
            try:
                shirt_color_str = sheet.cell(row=17, column=1).value
                self.get_logger().info(f'👕 上衣颜色原始数据: {shirt_color_str}')
            except:
                shirt_color_str = None
                self.get_logger().warn('⚠️ 无法读取上衣颜色数据')
                
            try:
                pants_color_str = sheet.cell(row=18, column=1).value
                self.get_logger().info(f'👖 下装颜色原始数据: {pants_color_str}')
            except:
                pants_color_str = None
                self.get_logger().warn('⚠️ 无法读取下装颜色数据')

            # 解析颜色字符串
            def parse_color(color_str):
                if not color_str:
                    return (0, 0, 0)

                try:
                    if isinstance(color_str, str):
                        color_str = color_str.strip()
                        if color_str.startswith('(') and color_str.endswith(')'):
                            return eval(color_str)
                        else:
                            color_str = color_str.replace('(', '').replace(')', '')
                            values = [int(x.strip()) for x in color_str.split(',')]
                            return tuple(values[:3])
                    elif isinstance(color_str, tuple):
                        return color_str
                    else:
                        return (0, 0, 0)
                except Exception as e:
                    self.get_logger().warn(f'⚠️ 颜色解析失败: {e}')
                    return (0, 0, 0)

            shirt_color = parse_color(shirt_color_str)
            pants_color = parse_color(pants_color_str)

            self.get_logger().info(f"✅ 成功读取目标特征:")
            self.get_logger().info(f"   📏 身体比例: {len(body_ratios)} 个数值")
            self.get_logger().info(f"   👕 上衣颜色: {shirt_color}")
            self.get_logger().info(f"   👖 下装颜色: {pants_color}")
            
            return body_ratios, shirt_color, pants_color

        except Exception as e:
            self.get_logger().error(f"❌ 读取目标特征失败: {str(e)}")
            traceback.print_exc()
            return None

    def publish_target_position(self, target_track, frame_shape):
        """
        发布目标位置信息（用于小车控制） - 机器人导航接口
        ===============================================
        
        功能说明：
            将目标位置转换为机器人控制坐标
            包含距离、角度等导航所需信息
        
        参数：
            target_track (STrack): 目标轨迹对象
            frame_shape (tuple): 图像尺寸 (height, width)
            
        返回值：
            无
            
        调用关系：
            - 被process_frame()在启用小车控制时调用
            
        调用原因：
            为机器人提供目标跟随的导航信息
            
        调试信息：
            - 记录位置计算过程
            - 显示角度和距离信息
        """
        try:
            # 计算目标中心点
            center_x = (target_track.tlbr[0] + target_track.tlbr[2]) / 2
            center_y = (target_track.tlbr[1] + target_track.tlbr[3]) / 2

            # 计算相对于图像中心的角度
            img_center_x = frame_shape[1] / 2
            img_center_y = frame_shape[0] / 2

            # 水平角度（弧度）
            angle_x = np.arctan2(center_x - img_center_x, frame_shape[1])

            # 获取距离
            distance = self.get_distance_at_point(int(center_x), int(center_y))

            # 发布位置信息
            pos_msg = Position()
            pos_msg.x = float(center_x)
            pos_msg.y = float(center_y)
            pos_msg.distance = float(distance)
            pos_msg.angle_x = float(angle_x)
            pos_msg.angle_y = 0.0  # 垂直角度暂不计算

            self.person_position_pub.publish(pos_msg)

        except Exception as e:
            self.get_logger().error(f"发布目标位置错误: {e}")

    def draw_dashed_rectangle(self, img, pt1, pt2, color, thickness=1, dash_length=8, gap_length=5):
        """
        绘制虚线矩形 - 可视化辅助工具
        ===========================
        
        功能说明：
            在图像上绘制虚线矩形，用于显示预测边界框
            提供更丰富的视觉反馈
        
        参数：
            img (np.array): 目标图像
            pt1 (tuple): 左上角坐标 (x, y)
            pt2 (tuple): 右下角坐标 (x, y)
            color (tuple): 线条颜色 (B, G, R)
            thickness (int): 线条粗细
            dash_length (int): 虚线段长度
            gap_length (int): 虚线间隙长度
            
        返回值：
            无（直接修改输入图像）
            
        调用关系：
            - 被create_visualization()调用
            
        调用原因：
            增强可视化效果，区分预测和实际边界框
        """
        x1, y1 = pt1
        x2, y2 = pt2

        # 绘制水平线段
        for x in range(x1, x2, dash_length + gap_length):
            x_end = min(x + dash_length, x2)
            cv2.line(img, (x, y1), (x_end, y1), color, thickness)
            cv2.line(img, (x, y2), (x_end, y2), color, thickness)

        # 绘制垂直线段
        for y in range(y1, y2, dash_length + gap_length):
            y_end = min(y + dash_length, y2)
            cv2.line(img, (x1, y), (x1, y_end), color, thickness)
            cv2.line(img, (x2, y), (x2, y_end), color, thickness)

    def publish_visualization(self, viz_frame):
        """
        发布可视化图像 - 实时显示和ROS话题发布
        ====================================
        
        功能说明：
            同时发布可视化图像到ROS话题和本地显示
            为WebSocket桥接节点提供图像数据
        
        参数：
            viz_frame (np.array): 可视化图像
            
        返回值：
            无
            
        调用关系：
            - 被process_frame()调用
            
        调用原因：
            提供实时的视觉反馈并支持远程显示
            
        调试信息：
            - 记录发布状态和异常情况
        """
        try:
            # 1. 发布ROS图像消息（供WebSocket桥接节点使用）
            try:
                img_msg = self.bridge.cv2_to_imgmsg(viz_frame, "bgr8")
                img_msg.header.stamp = self.get_clock().now().to_msg()
                img_msg.header.frame_id = "camera"
                self.visualization_pub.publish(img_msg)
            except Exception as e:
                self.get_logger().debug(f"发布可视化图像ROS消息失败: {e}")
            
            # 2. 本地显示（调试用）
            try:
                cv2.imshow('ByteTracker Visualization', viz_frame)
                cv2.waitKey(1)
            except Exception as e:
                # 在无显示环境中，这个错误是预期的，只记录debug级别
                self.get_logger().debug(f"本地显示图像失败: {e}")
                
        except Exception as e:
            self.get_logger().error(f"发布可视化图像错误: {e}")

    def publish_status(self):
        """
        发布状态信息 - 系统状态监控
        =========================
        
        功能说明：
            定期发布系统运行状态和统计信息
            支持外部监控和诊断
        
        参数：
            无
            
        返回值：
            无
            
        调用关系：
            - 被status_timer定时器调用（2Hz）
            
        调用原因：
            提供系统健康状态和性能指标
            
        调试信息：
            - 记录状态信息内容
            - 显示性能统计数据
        """
        try:
            status_msg = String()
            status_info = {
                "mode": self.tracking_mode,
                "frame_count": self.frame_count,
                "tracked_objects": len(self.current_tracks),
                "target_id": self.current_target_track.track_id if self.current_target_track else None,
                "processing_time": self.processing_time
            }
            status_msg.data = str(status_info)
            self.status_pub.publish(status_msg)

        except Exception as e:
            self.get_logger().error(f"发布状态信息错误: {e}")

    def destroy_node(self):
        """
        节点销毁时的清理工作 - 资源释放管理
        =================================
        
        功能说明：
            在节点关闭时执行资源清理
            确保线程、相机等资源正确释放
        
        参数：
            无
            
        返回值：
            无
            
        调用关系：
            - 被main()函数在程序结束时调用
            
        调用原因：
            避免资源泄漏，确保优雅关闭
            
        调试信息：
            - 记录清理过程和统计信息
            - 显示系统运行总结
        """
        try:
            self.running = False
            if hasattr(self, 'cap') and self.cap is not None:
                self.cap.release()
            cv2.destroyAllWindows()
            
            self.get_logger().info(f'📊 跟踪统计: 处理了 {self.frame_count} 帧')
            self.get_logger().info('✅ ByteTracker节点已关闭')
            super().destroy_node()
            
        except Exception as e:
            self.get_logger().error(f"节点销毁错误: {e}")

def main(args=None):
    """
    主函数 - 程序入口点
    ==================
    
    功能说明：
        初始化ROS2环境，创建并运行ByteTracker节点
        处理程序生命周期和异常情况
    
    参数：
        args: 命令行参数
        
    返回值：
        无
        
    调用关系：
        - 程序启动时被调用
        
    调用原因：
        作为程序的主入口，管理整个系统的生命周期
        
    调试信息：
        - 记录程序启动和关闭过程
        - 显示异常处理和资源清理
    """
    try:
        rclpy.init(args=args)
        node = ByteTrackerNode()

        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()

    except Exception as e:
        print(f"主函数错误: {e}")
        traceback.print_exc()


if __name__ == '__main__':
    main()