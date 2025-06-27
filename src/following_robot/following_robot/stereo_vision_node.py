#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
按需双目立体视觉ROS2节点
=========================
优化版本：只在距离查询服务被调用时才进行立体视觉处理
- 实时显示左目图像（带帧数和FPS显示）
- 按需提供距离测量服务（只在服务调用时处理立体视觉）
- 大幅提升性能和降低CPU占用
- 键盘交互控制（按'q'退出）
- 集成人体衣服检测功能

作者: AI Assistant
优化: 按需处理立体视觉，提升系统性能
新增: 人体检测和身体框定功能
"""

import rclpy
from rclpy.node import Node
from custom_msgs.srv import GetDistance

import cv2
import numpy as np
import traceback
import threading
import time
from functools import lru_cache

# 导入人体检测模块
try:
    from .rknn_colour_detect import detect_picture_with_confidence, Determine_the_position_of_the_entire_body
    HUMAN_DETECTION_AVAILABLE = True
    print("✅ 人体检测模块导入成功")
except ImportError as e:
    HUMAN_DETECTION_AVAILABLE = False
    print(f"⚠️ 人体检测模块导入失败: {e}")
    print("继续运行但不提供人体检测功能")

# 导入姿态检测模块
try:
    from .yolov8_pose_detector import detect_human_pose, draw_human_pose, get_pose_detector
    POSE_DETECTION_AVAILABLE = True
    print("✅ 姿态检测模块导入成功")
except ImportError as e:
    POSE_DETECTION_AVAILABLE = False
    print(f"⚠️ 姿态检测模块导入失败: {e}")
    print("继续运行但不提供姿态检测功能")


class StereoConfig:
    """立体视觉系统配置类"""
    
    def __init__(self):
        try:
            # 相机内参和外参
            self.baseline = 25.100  # 基线距离
            self.focal_length = 663  # 焦距
            self.cx = 317  # 光心x坐标
            self.cy = 210  # 光心y坐标

            # SGBM算法参数 - 针对按需处理优化的参数
            self.minDisparity = 3
            self.numDisparities = 32  # 适中的视差范围，平衡精度和速度
            self.blockSize = 7  # 适中的块大小
            self.P1 = 800
            self.P2 = 3200
            self.disp12MaxDiff = 3
            self.preFilterCap = 31
            self.uniquenessRatio = 10
            self.speckleWindowSize = 100
            self.speckleRange = 32
            self.mode = cv2.STEREO_SGBM_MODE_SGBM_3WAY  # 使用更精确的模式，因为不需要实时处理

            # 相机参数
            self.camera_id = 1
            self.frame_width = 1280
            self.frame_height = 480
            self.fps_limit = 60

            # 距离测量范围
            self.min_distance_mm = 100.0
            self.max_distance_mm = 10000.0
            
        except Exception as e:
            print(f"配置初始化错误: {e}")
            traceback.print_exc()


class StereoCamera:
    """双目相机参数类"""
    
    def __init__(self):
        try:
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
            self.Q = None  # 在getRectifyTransform中计算
            
        except Exception as e:
            print(f"相机参数初始化错误: {e}")
            traceback.print_exc()


class StereoVisionNode(Node):
    """按需双目立体视觉ROS2节点类"""
    
    def __init__(self):
        try:
            super().__init__('stereo_vision_node')
            
            # 基本属性初始化
            self.config = None
            self.stereo_config = None
            self.cap = None
            self.running = False
            
            # 帧数据缓存 - 用于按需处理
            self.current_left_frame = None
            self.current_right_frame = None
            self.frame_lock = threading.Lock()
            self.frame_timestamp = 0
            
            # 立体视觉相关
            self.map1x = None
            self.map1y = None
            self.map2x = None
            self.map2y = None
            self.Q = None
            
            # 统计信息
            self.stereo_processing_count = 0
            self.last_stereo_processing_time = 0
            
            # 人体检测相关属性
            self.human_detection_enabled = HUMAN_DETECTION_AVAILABLE
            self.human_detection_count = 0
            self.last_human_detection_time = 0
            self.current_human_boxes = []  # 当前检测到的人体框
            self.current_clothing_detections = []  # 当前检测到的衣服和裤子详细信息
            
            # 姿态检测相关属性
            self.pose_detection_enabled = POSE_DETECTION_AVAILABLE
            self.pose_detection_count = 0
            self.last_pose_detection_time = 0
            self.current_pose_results = []  # 当前检测到的姿态结果
            
            # 初始化配置
            try:
                self.config = StereoConfig()
                self.get_logger().info('✅ 配置初始化完成')
            except Exception as e:
                self.get_logger().error(f"❌ 配置初始化失败: {e}")
                raise
            
            try:
                self.stereo_config = StereoCamera()
                self.get_logger().info('✅ 立体相机配置初始化完成')
            except Exception as e:
                self.get_logger().error(f"❌ 立体相机配置初始化失败: {e}")
                raise
            
            # 预先计算校正变换矩阵
            try:
                self.setup_stereo_rectification()
                self.get_logger().info('✅ 立体校正设置完成')
            except Exception as e:
                self.get_logger().error(f"❌ 立体校正设置失败: {e}")
                raise
            
            # 初始化相机
            try:
                self.init_camera()
                self.get_logger().info('✅ 相机初始化完成')
            except Exception as e:
                self.get_logger().error(f"❌ 相机初始化失败: {e}")
                pass
            
            # 创建服务
            try:
                self.distance_service = self.create_service(
                    GetDistance,
                    '/stereo/get_distance',
                    self.get_distance_callback
                )
                self.get_logger().info('✅ ROS2距离测量服务创建完成')
            except Exception as e:
                self.get_logger().error(f"❌ ROS2服务创建失败: {e}")
                raise
            
            # 启动图像捕获线程
            if self.cap is not None:
                try:
                    self.start_capture_thread()
                    self.get_logger().info('✅ 图像捕获线程启动完成')
                except Exception as e:
                    self.get_logger().error(f"❌ 图像捕获线程启动失败: {e}")
                    pass
            
            self.get_logger().info('✅ 按需双目立体视觉节点初始化完成!')
            self.get_logger().info('💡 立体视觉处理将仅在距离查询服务被调用时进行')
            if HUMAN_DETECTION_AVAILABLE:
                self.get_logger().info('🤖 人体检测功能已启用（按"h"键切换开关）')
            else:
                self.get_logger().warn('⚠️ 人体检测功能不可用')
            if POSE_DETECTION_AVAILABLE:
                self.get_logger().info('🦴 姿态检测功能已启用（按"p"键切换开关）')
            else:
                self.get_logger().warn('⚠️ 姿态检测功能不可用')
            self.get_logger().info('🔧 按键说明: "q"退出, "h"切换人体检测, "p"切换姿态检测, "s"测试立体视觉')
            
        except Exception as e:
            self.get_logger().error(f"❌ 节点初始化错误: {e}")
            traceback.print_exc()
            self.get_logger().warn("⚠️ 节点在有限功能下启动")

    def setup_stereo_rectification(self):
        """设置立体校正参数"""
        try:
            if self.config is None or self.stereo_config is None:
                raise RuntimeError("配置对象未初始化")
                
            height, width = self.config.frame_height, self.config.frame_width // 2
            result = self.get_rectify_transform(height, width, self.stereo_config)
            
            if result is None or any(item is None for item in result):
                raise RuntimeError("校正变换计算失败")
                
            self.map1x, self.map1y, self.map2x, self.map2y, self.Q = result
            self.get_logger().info('立体校正参数设置完成')
            
        except Exception as e:
            self.get_logger().error(f"立体校正设置错误: {e}")
            traceback.print_exc()
            raise

    def get_rectify_transform(self, height, width, stereo_config):
        """获取畸变校正和立体校正的映射变换矩阵"""
        try:
            left_K = stereo_config.cam_matrix_left
            right_K = stereo_config.cam_matrix_right
            left_dist = stereo_config.distortion_l
            right_dist = stereo_config.distortion_r
            R = stereo_config.R
            T = stereo_config.T

            # 计算立体校正参数
            R1, R2, P1, P2, Q, roi1, roi2 = cv2.stereoRectify(
                left_K, left_dist, right_K, right_dist,
                (width, height), R, T,
                flags=cv2.CALIB_ZERO_DISPARITY,
                alpha=0.5
            )

            # 保存Q矩阵
            stereo_config.Q = Q

            # 生成映射矩阵
            map1x, map1y = cv2.initUndistortRectifyMap(
                left_K, left_dist, R1, P1, (width, height), cv2.CV_32FC1
            )
            map2x, map2y = cv2.initUndistortRectifyMap(
                right_K, right_dist, R2, P2, (width, height), cv2.CV_32FC1
            )

            return map1x, map1y, map2x, map2y, Q
            
        except Exception as e:
            self.get_logger().error(f"校正变换计算错误: {e}")
            traceback.print_exc()
            return None, None, None, None, None

    def init_camera(self):
        """初始化相机"""
        try:
            if self.config is None:
                raise RuntimeError("配置对象未初始化")
                
            self.cap = cv2.VideoCapture(self.config.camera_id, cv2.CAP_V4L2)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.config.frame_width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.config.frame_height)
            # 兼容不同OpenCV版本的fourcc设置
            try:
                self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
            except AttributeError:
                # 如果VideoWriter_fourcc不可用，尝试使用数值
                self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('M', 'J', 'P', 'G'))
            self.cap.set(cv2.CAP_PROP_CONVERT_RGB, 1)
            
            if not self.cap.isOpened():
                self.get_logger().warn(f"无法打开相机ID {self.config.camera_id}，尝试使用默认相机...")
                self.config.camera_id = 0
                self.cap = cv2.VideoCapture(self.config.camera_id)
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.config.frame_width)
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.config.frame_height)
                self.cap.set(cv2.CAP_PROP_CONVERT_RGB, 1)

                if not self.cap.isOpened():
                    raise RuntimeError("错误：无法打开相机！")
                    
            self.get_logger().info(f'相机初始化成功，ID: {self.config.camera_id}')
            
        except Exception as e:
            self.get_logger().error(f"相机初始化错误: {e}")
            traceback.print_exc()
            raise

    def start_capture_thread(self):
        """启动图像捕获线程"""
        try:
            self.running = True
            self.capture_thread = threading.Thread(target=self.capture_loop)
            self.capture_thread.daemon = True
            self.capture_thread.start()
            self.get_logger().info('图像捕获线程已启动')
            
        except Exception as e:
            self.get_logger().error(f"捕获线程启动错误: {e}")
            traceback.print_exc()

    def capture_loop(self):
        """图像捕获循环 - 仅捕获和显示，不进行立体处理"""
        frame_counter = 0
        start_time = time.time()
        fps_counter = 0
        current_fps = 0.0
        
        while self.running:
            try:
                if self.cap is None:
                    time.sleep(0.1)
                    continue
                    
                ret, frame = self.cap.read()
                if not ret:
                    self.get_logger().warn("无法获取图像")
                    time.sleep(0.5)
                    continue

                frame_counter += 1
                fps_counter += 1

                # 调整图像大小
                if self.config is not None and (frame.shape[1] != self.config.frame_width or frame.shape[0] != self.config.frame_height):
                    frame = cv2.resize(frame, (self.config.frame_width, self.config.frame_height))

                # 分割左右图像
                mid_x = frame.shape[1] // 2
                left_half = frame[:, :mid_x]
                right_half = frame[:, mid_x:]

                # 缓存当前帧数据供距离服务使用
                with self.frame_lock:
                    self.current_left_frame = left_half.copy()
                    self.current_right_frame = right_half.copy()
                    self.frame_timestamp = time.time()

                # 人体检测处理（每5帧执行一次以保持性能）
                if self.human_detection_enabled and frame_counter % 5 == 0:
                    self.process_human_detection(left_half)
                
                # 姿态检测处理（每8帧执行一次以保持性能）
                if self.pose_detection_enabled and frame_counter % 8 == 0:
                    self.process_pose_detection(left_half)

                # 计算实时FPS
                current_time = time.time()
                elapsed_time = current_time - start_time
                if elapsed_time >= 1.0:  # 每秒更新一次FPS
                    current_fps = fps_counter / elapsed_time
                    fps_counter = 0
                    start_time = current_time

                # 创建显示图像
                display_image = left_half.copy()
                
                # 绘制姿态检测结果
                if self.current_pose_results:
                    display_image = draw_human_pose(display_image, self.current_pose_results)
                
                # 绘制衣服和裤子检测框
                if self.current_clothing_detections:
                    for person_clothes in self.current_clothing_detections:
                        person_id = person_clothes['person_id']
                        upper_info = person_clothes['upper']
                        lower_info = person_clothes['lower']
                        
                        # 绘制上装检测框（蓝色）
                        if upper_info and len(upper_info) >= 4:
                            x1, y1, x2, y2 = upper_info[:4]
                            cv2.rectangle(display_image, (int(x1), int(y1)), (int(x2), int(y2)), (255, 0, 0), 2)
                            # 添加上装标签
                            upper_label = f"Upper {person_id+1}"
                            if person_clothes['upper_color']:
                                upper_label += f" {person_clothes['upper_color']}"
                            cv2.putText(display_image, upper_label, (int(x1), int(y1)-10), 
                                      cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                        
                        # 绘制下装检测框（红色）
                        if lower_info and len(lower_info) >= 4:
                            x1, y1, x2, y2 = lower_info[:4]
                            cv2.rectangle(display_image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 0, 255), 2)
                            # 添加下装标签
                            lower_label = f"Lower {person_id+1}"
                            if person_clothes['lower_color']:
                                lower_label += f" {person_clothes['lower_color']}"
                            cv2.putText(display_image, lower_label, (int(x1), int(y1)-10), 
                                      cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                
                # 绘制整体人体检测框（绿色）
                if self.current_human_boxes:
                    for i, box in enumerate(self.current_human_boxes):
                        if len(box) >= 4:
                            x1, y1, x2, y2 = box[:4]
                            cv2.rectangle(display_image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                            cv2.putText(display_image, f"Person {i+1}", (int(x1), int(y1)-10), 
                                      cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                
                # 添加文本信息
                font = cv2.FONT_HERSHEY_SIMPLEX
                font_scale = 0.7
                color = (0, 255, 0)  # 绿色
                thickness = 2
                
                # 显示帧数
                frame_text = f"Frame: {frame_counter}"
                cv2.putText(display_image, frame_text, (10, 30), font, font_scale, color, thickness)
                
                # 显示FPS
                fps_text = f"FPS: {current_fps:.1f}"
                cv2.putText(display_image, fps_text, (10, 60), font, font_scale, color, thickness)
                
                # 显示图像分辨率
                resolution_text = f"Size: {display_image.shape[1]}x{display_image.shape[0]}"
                cv2.putText(display_image, resolution_text, (10, 90), font, font_scale, color, thickness)
                
                # 显示立体处理统计信息
                stereo_info_text = f"Stereo Calls: {self.stereo_processing_count}"
                cv2.putText(display_image, stereo_info_text, (10, 120), font, font_scale, (255, 255, 0), thickness)
                
                # 显示最后一次立体处理时间
                if self.last_stereo_processing_time > 0:
                    time_since_last = current_time - self.last_stereo_processing_time
                    last_process_text = f"Last Stereo: {time_since_last:.1f}s ago"
                    cv2.putText(display_image, last_process_text, (10, 150), font, font_scale, (255, 255, 0), thickness)
                
                # 显示人体检测信息
                if self.human_detection_enabled:
                    human_text = f"Humans: {len(self.current_human_boxes)} detected"
                    cv2.putText(display_image, human_text, (10, 180), font, font_scale, (255, 0, 255), thickness)
                    
                    # 显示衣服检测详情
                    clothing_text = f"Clothes: {len(self.current_clothing_detections)} sets"
                    cv2.putText(display_image, clothing_text, (10, 210), font, font_scale, (255, 0, 255), thickness)
                    
                    detection_count_text = f"Detection Calls: {self.human_detection_count}"
                    cv2.putText(display_image, detection_count_text, (10, 240), font, font_scale, (255, 0, 255), thickness)
                
                # 显示姿态检测信息
                if self.pose_detection_enabled:
                    pose_text = f"Poses: {len(self.current_pose_results)} detected"
                    cv2.putText(display_image, pose_text, (10, 270), font, font_scale, (0, 255, 255), thickness)
                    
                    pose_count_text = f"Pose Calls: {self.pose_detection_count}"
                    cv2.putText(display_image, pose_count_text, (10, 300), font, font_scale, (0, 255, 255), thickness)
                
                # 显示模式信息
                mode_text = "Mode: ON-DEMAND STEREO + CLOTHING + POSE DETECTION"
                cv2.putText(display_image, mode_text, (10, 330), font, font_scale, (0, 255, 255), thickness)

                # 使用cv2.imshow显示图像
                cv2.imshow('Left Camera View (On-Demand Stereo)', display_image)
                
                # 处理键盘事件
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):  # 按'q'退出
                    self.get_logger().info("用户按下'q'键，退出程序")
                    self.running = False
                    break
                elif key == ord('h'):  # 按'h'切换人体检测
                    if HUMAN_DETECTION_AVAILABLE:
                        self.human_detection_enabled = not self.human_detection_enabled
                        status = "开启" if self.human_detection_enabled else "关闭"
                        self.get_logger().info(f"人体检测已{status}")
                        if not self.human_detection_enabled:
                            self.current_human_boxes = []  # 清空检测框
                            self.current_clothing_detections = []  # 清空衣服检测
                    else:
                        self.get_logger().warn("人体检测模块不可用")
                elif key == ord('p'):  # 按'p'切换姿态检测
                    if POSE_DETECTION_AVAILABLE:
                        self.pose_detection_enabled = not self.pose_detection_enabled
                        status = "开启" if self.pose_detection_enabled else "关闭"
                        self.get_logger().info(f"姿态检测已{status}")
                        if not self.pose_detection_enabled:
                            self.current_pose_results = []  # 清空姿态检测结果
                    else:
                        self.get_logger().warn("姿态检测模块不可用")
                elif key == ord('s'):  # 按's'手动触发立体视觉处理测试
                    self.get_logger().info("手动触发立体视觉处理测试")
                    with self.frame_lock:
                        if self.current_left_frame is not None and self.current_right_frame is not None:
                            test_result = self.process_stereo_on_demand(self.current_left_frame, self.current_right_frame)
                            if test_result is not None:
                                self.get_logger().info("✅ 立体视觉处理测试成功")
                            else:
                                self.get_logger().warn("❌ 立体视觉处理测试失败")

                # 高帧率运行（因为没有立体处理的开销）
                if self.config is not None:
                        time.sleep(1.0 / self.config.fps_limit)
                else:
                    time.sleep(1.0 / 60)  # 默认60fps
                
            except Exception as e:
                self.get_logger().error(f"图像捕获循环错误: {e}")
                traceback.print_exc()
                time.sleep(0.01)

    def process_human_detection(self, image):
        """处理人体检测"""
        try:
            if not self.human_detection_enabled:
                return
            
            self.get_logger().debug("🤖 开始人体检测...")
            detection_start_time = time.time()
            
            # 调用人体检测函数
            detection_results = detect_picture_with_confidence(image)
            
            if detection_results:
                # 提取人体位置和衣服详细信息
                human_boxes = []
                clothing_detections = []
                
                for person_idx, result in enumerate(detection_results):
                    if len(result) >= 2:  # 确保有上装和下装信息
                        upper_info = result[0] if result[0] else None
                        lower_info = result[1] if result[1] else None
                        
                        # 保存衣服和裤子的详细检测信息
                        person_clothes = {
                            'person_id': person_idx,
                            'upper': upper_info,
                            'lower': lower_info,
                            'upper_color': result[2] if len(result) > 2 else None,
                            'lower_color': result[3] if len(result) > 3 else None
                        }
                        clothing_detections.append(person_clothes)
                        
                        # 使用Determine_the_position_of_the_entire_body函数获取整体框
                        try:
                            body_box = Determine_the_position_of_the_entire_body(
                                upper_info, lower_info, image
                            )
                            if body_box:
                                human_boxes.append(body_box)
                        except Exception as e:
                            self.get_logger().debug(f"身体框计算失败: {e}")
                
                # 更新检测结果
                self.current_human_boxes = human_boxes
                self.current_clothing_detections = clothing_detections
                self.human_detection_count += 1
                self.last_human_detection_time = time.time()
                
                detection_time = (self.last_human_detection_time - detection_start_time) * 1000
                self.get_logger().debug(f"✅ 检测到 {len(human_boxes)} 个人体，{len(clothing_detections)} 套衣服，耗时: {detection_time:.2f}ms")
            else:
                self.current_human_boxes = []
                self.current_clothing_detections = []
                
        except Exception as e:
            self.get_logger().error(f"人体检测处理错误: {e}")
            traceback.print_exc()
            self.current_human_boxes = []
            self.current_clothing_detections = []

    def process_pose_detection(self, image):
        """处理姿态检测"""
        try:
            if not self.pose_detection_enabled:
                return
            
            self.get_logger().debug("🤖 开始姿态检测...")
            detection_start_time = time.time()
            
            # 调用姿态检测函数
            pose_results = detect_human_pose(image)
            
            if pose_results:
                # 更新检测结果
                self.current_pose_results = pose_results
                self.pose_detection_count += 1
                self.last_pose_detection_time = time.time()
                
                detection_time = (self.last_pose_detection_time - detection_start_time) * 1000
                self.get_logger().debug(f"✅ 检测到 {len(pose_results)} 个人体姿态，耗时: {detection_time:.2f}ms")
            else:
                self.current_pose_results = []
                
        except Exception as e:
            self.get_logger().error(f"姿态检测处理错误: {e}")
            import traceback
            traceback.print_exc()
            self.current_pose_results = []

    def process_stereo_on_demand(self, left_image, right_image):
        """按需处理立体视觉 - 仅在服务调用时执行"""
        try:
            self.get_logger().info("🔄 开始按需立体视觉处理...")
            process_start_time = time.time()
            
            # 检查必要的属性是否存在
            if not hasattr(self, 'stereo_config') or self.stereo_config is None:
                self.get_logger().error("❌ stereo_config未初始化")
                return None
                
            if not hasattr(self, 'map1x') or self.map1x is None:
                self.get_logger().error("❌ 校正映射矩阵未初始化")
                return None
                
            if not hasattr(self, 'Q') or self.Q is None:
                self.get_logger().error("❌ 重投影矩阵Q未初始化")
                return None
            
            # 消除畸变
            iml = self.undistortion(left_image, self.stereo_config.cam_matrix_left, 
                                  self.stereo_config.distortion_l)
            imr = self.undistortion(right_image, self.stereo_config.cam_matrix_right, 
                                  self.stereo_config.distortion_r)

            # 预处理图像
            iml_processed, imr_processed = self.preprocess(iml, imr)

            # 图像校正
            iml_rectified, imr_rectified = self.rectify_image(
                iml_processed, imr_processed, 
                self.map1x, self.map1y, self.map2x, self.map2y
            )

            # 计算视差图
            disparity = self.stereo_match_sgbm(iml_rectified, imr_rectified)

            # 计算3D点云
            points_3d = self.reproject_to_3d(disparity, self.Q)

            # 更新统计信息
            self.stereo_processing_count += 1
            self.last_stereo_processing_time = time.time()
            
            process_time = self.last_stereo_processing_time - process_start_time
            self.get_logger().info(f"✅ 立体视觉处理完成，耗时: {process_time:.3f}秒")

            return points_3d
            
        except Exception as e:
            self.get_logger().error(f"按需立体视觉处理错误: {e}")
            traceback.print_exc()
            return None

    def undistortion(self, image, camera_matrix, dist_coeff):
        """消除图像畸变"""
        try:
            h, w = image.shape[:2]
            new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(
                camera_matrix, dist_coeff, (w, h), 1, (w, h)
            )
            undistorted = cv2.undistort(image, camera_matrix, dist_coeff, None, new_camera_matrix)

            x, y, w, h = roi
            if w > 0 and h > 0:
                undistorted = undistorted[y:y + h, x:x + w]

            return undistorted
            
        except Exception as e:
            self.get_logger().error(f"畸变校正错误: {e}")
            traceback.print_exc()
            return image

    def preprocess(self, img1, img2):
        """图像预处理"""
        try:
            # 转换为灰度图
            if img1.ndim == 3:
                img1_gray = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
            else:
                img1_gray = img1.copy()

            if img2.ndim == 3:
                img2_gray = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)
            else:
                img2_gray = img2.copy()

            # 应用CLAHE增强对比度
            clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
            img1_eq = clahe.apply(img1_gray)
            img2_eq = clahe.apply(img2_gray)

            return img1_eq, img2_eq
            
        except Exception as e:
            self.get_logger().error(f"图像预处理错误: {e}")
            traceback.print_exc()
            return img1, img2

    def rectify_image(self, image1, image2, map1x, map1y, map2x, map2y):
        """对图像应用畸变校正和立体校正"""
        try:
            rectified_img1 = cv2.remap(image1, map1x, map1y, cv2.INTER_AREA)
            rectified_img2 = cv2.remap(image2, map2x, map2y, cv2.INTER_AREA)
            return rectified_img1, rectified_img2
            
        except Exception as e:
            self.get_logger().error(f"图像校正错误: {e}")
            traceback.print_exc()
            return image1, image2

    def stereo_match_sgbm(self, left_image, right_image):
        """使用SGBM算法计算视差图"""
        try:
            # 确保输入图像是灰度图
            if left_image.ndim != 2:
                left_image = cv2.cvtColor(left_image, cv2.COLOR_BGR2GRAY)
            if right_image.ndim != 2:
                right_image = cv2.cvtColor(right_image, cv2.COLOR_BGR2GRAY)

            # 检查配置是否存在
            if self.config is None:
                self.get_logger().error("❌ config未初始化")
                return None
                
            # 创建SGBM匹配器（兼容不同OpenCV版本）
            try:
                left_matcher = cv2.StereoSGBM_create(
                    minDisparity=self.config.minDisparity,
                    numDisparities=self.config.numDisparities,
                    blockSize=self.config.blockSize,
                    P1=self.config.P1,
                    P2=self.config.P2,
                    disp12MaxDiff=self.config.disp12MaxDiff,
                    preFilterCap=self.config.preFilterCap,
                    uniquenessRatio=self.config.uniquenessRatio,
                    speckleWindowSize=self.config.speckleWindowSize,
                    speckleRange=self.config.speckleRange,
                    mode=self.config.mode
                )
            except AttributeError:
                # 如果StereoSGBM_create不可用，使用旧版API
                left_matcher = cv2.createStereoSGBM(
                    minDisparity=self.config.minDisparity,
                    numDisparities=self.config.numDisparities,
                    blockSize=self.config.blockSize,
                    P1=self.config.P1,
                    P2=self.config.P2,
                    disp12MaxDiff=self.config.disp12MaxDiff,
                    preFilterCap=self.config.preFilterCap,
                    uniquenessRatio=self.config.uniquenessRatio,
                    speckleWindowSize=self.config.speckleWindowSize,
                    speckleRange=self.config.speckleRange,
                    mode=self.config.mode
                )

            # 计算视差图
            disparity = left_matcher.compute(left_image, right_image)
            disparity = disparity.astype(np.float32) / 16.0

            # 过滤小视差值
            min_valid_disp = 1.0
            disparity[disparity < min_valid_disp] = 0

            return disparity
            
        except Exception as e:
            self.get_logger().error(f"立体匹配错误: {e}")
            traceback.print_exc()
            return None

    def reproject_to_3d(self, disparity, Q):
        """将视差图转换为3D点云"""
        try:
            if disparity is None or Q is None:
                return None
                
            # 过滤太小的视差值
            filtered_disp = disparity.copy()
            min_disparity = 1.0
            filtered_disp[filtered_disp < min_disparity] = 0

            # 使用OpenCV的reprojectImageTo3D进行重投影
            points_3d = cv2.reprojectImageTo3D(filtered_disp, Q)

            # 过滤深度值异常的点
            max_depth = self.config.max_distance_mm if self.config else 10000.0
            mask = (points_3d[:, :, 2] > 0) & (points_3d[:, :, 2] < max_depth)
            points_3d[~mask] = [0, 0, 0]

            return points_3d
            
        except Exception as e:
            self.get_logger().error(f"3D重投影错误: {e}")
            traceback.print_exc()
            return None

    def get_distance_callback(self, request, response):
        """距离测量服务回调函数 - 按需进行立体视觉处理"""
        try:
            self.get_logger().info(f"📞 收到距离查询请求: 坐标({request.x}, {request.y})")
            
            # 获取当前帧数据
            with self.frame_lock:
                if self.current_left_frame is None or self.current_right_frame is None:
                    response.success = False
                    response.distance = 0.0
                    response.message = "当前无可用图像数据"
                    self.get_logger().warn("❌ 无可用图像数据")
                    return response
                
                # 复制当前帧数据
                left_frame = self.current_left_frame.copy()
                right_frame = self.current_right_frame.copy()
                frame_age = time.time() - self.frame_timestamp

            self.get_logger().info(f"📸 使用图像数据，年龄: {frame_age:.3f}秒")
            
            # 按需进行立体视觉处理
            points_3d = self.process_stereo_on_demand(left_frame, right_frame)

            if points_3d is None:
                response.success = False
                response.distance = 0.0
                response.message = "立体视觉处理失败"
                self.get_logger().error("❌ 立体视觉处理失败")
                return response

            # 测量指定点的距离
            distance = self.measure_distance(points_3d, request.x, request.y)

            if distance is not None:
                response.success = True
                response.distance = distance
                response.message = f"成功测量距离: {distance:.2f}米"
                self.get_logger().info(f"✅ 测量点({request.x}, {request.y})距离: {distance:.2f}米")
            else:
                response.success = False
                response.distance = 0.0
                response.message = "无法测量该点距离，可能是无效点"
                self.get_logger().warn(f"⚠️ 无法测量点({request.x}, {request.y})的距离")

            return response
            
        except Exception as e:
            self.get_logger().error(f"距离服务回调错误: {e}")
            traceback.print_exc()
            response.success = False
            response.distance = 0.0
            response.message = f"服务处理错误: {str(e)}"
            return response

    def measure_distance(self, points_3d, x, y):
        """测量指定像素点到相机的距离"""
        try:
            h, w = points_3d.shape[:2]

            # 检查坐标是否在有效范围内
            if not (0 <= x < w and 0 <= y < h):
                self.get_logger().warn(f"坐标({x}, {y})超出范围({w}x{h})")
                return None

            # 获取点的3D坐标
            point_3d = points_3d[y, x]

            # 检查点的有效性
            if np.all(np.isfinite(point_3d)) and not np.all(point_3d == 0):
                # 计算欧几里得距离
                distance = np.sqrt(np.sum(point_3d ** 2))
                return distance / 1000.0  # 转换为米

            return None
            
        except Exception as e:
            self.get_logger().error(f"距离测量错误: {e}")
            traceback.print_exc()
            return None

    def destroy_node(self):
        """节点销毁时的清理工作"""
        try:
            self.running = False
            if hasattr(self, 'cap') and self.cap is not None:
                self.cap.release()
            cv2.destroyAllWindows()
            
            self.get_logger().info(f'📊 立体视觉处理统计: 总共处理了 {self.stereo_processing_count} 次')
            self.get_logger().info('✅ 按需双目立体视觉节点已关闭')
            super().destroy_node()
            
        except Exception as e:
            self.get_logger().error(f"节点销毁错误: {e}")
            traceback.print_exc()


def main(args=None):
    """主函数"""
    try:
        rclpy.init(args=args)
        node = StereoVisionNode()
        
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