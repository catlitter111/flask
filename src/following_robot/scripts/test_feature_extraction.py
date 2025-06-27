#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
特征提取节点测试脚本
==================
测试特征提取服务的功能
"""

import rclpy
from rclpy.node import Node
from custom_msgs.srv import FeatureExtraction
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import sys
import os


class FeatureExtractionTester(Node):
    """特征提取测试节点"""
    
    def __init__(self):
        super().__init__('feature_extraction_tester')
        
        # 初始化CV Bridge
        self.bridge = CvBridge()
        
        # 创建服务客户端
        self.client = self.create_client(FeatureExtraction, '/features/extract_features')
        
        # 等待服务可用
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待特征提取服务...')
        
        self.get_logger().info('✅ 特征提取服务已连接')

    def test_with_image_file(self, image_path, person_name="test_person"):
        """使用图像文件测试特征提取"""
        try:
            # 读取图像
            if not os.path.exists(image_path):
                self.get_logger().error(f"图像文件不存在: {image_path}")
                return False
                
            cv_image = cv2.imread(image_path)
            if cv_image is None:
                self.get_logger().error(f"无法读取图像: {image_path}")
                return False
                
            self.get_logger().info(f"📸 读取图像成功: {image_path}, 尺寸: {cv_image.shape}")
            
            # 转换为ROS图像消息
            ros_image = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            
            # 创建请求
            request = FeatureExtraction.Request()
            request.image = ros_image
            request.person_name = person_name
            request.save_to_file = True
            request.output_path = ""  # 使用默认路径
            
            # 发送请求
            self.get_logger().info(f"🚀 发送特征提取请求: {person_name}")
            future = self.client.call_async(request)
            
            # 等待响应
            rclpy.spin_until_future_complete(self, future)
            
            if future.result() is not None:
                response = future.result()
                self.print_response(response)
                return response.success
            else:
                self.get_logger().error('❌ 服务调用失败')
                return False
                
        except Exception as e:
            self.get_logger().error(f"测试失败: {e}")
            return False

    def test_with_camera(self, camera_id=1, person_name="camera_person"):
        """使用摄像头测试特征提取"""
        try:
            # 打开摄像头
            cap = cv2.VideoCapture(camera_id)
            if not cap.isOpened():
                self.get_logger().error(f"无法打开摄像头 ID: {camera_id}")
                return False
                
            self.get_logger().info(f"📹 摄像头 {camera_id} 已打开")
            self.get_logger().info("按空格键捕获图像进行特征提取，按'q'退出")
            
            while True:
                ret, frame = cap.read()
                if not ret:
                    self.get_logger().error("无法读取摄像头帧")
                    break
                    
                # 如果是双目摄像头，只使用左半部分
                if frame.shape[1] > frame.shape[0] * 1.5:  # 宽高比大于1.5，可能是双目
                    mid_x = frame.shape[1] // 2
                    frame = frame[:, :mid_x]
                
                # 显示图像
                cv2.imshow('摄像头预览 - 按空格键提取特征', frame)
                
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                elif key == ord(' '):  # 空格键
                    self.get_logger().info("🎯 捕获图像，开始特征提取...")
                    
                    # 转换为ROS图像消息
                    ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                    
                    # 创建请求
                    request = FeatureExtraction.Request()
                    request.image = ros_image
                    request.person_name = person_name
                    request.save_to_file = True
                    request.output_path = ""
                    
                    # 发送请求
                    future = self.client.call_async(request)
                    
                    # 等待响应
                    rclpy.spin_until_future_complete(self, future)
                    
                    if future.result() is not None:
                        response = future.result()
                        self.print_response(response)
                        if response.success:
                            self.get_logger().info("✅ 特征提取成功！继续预览...")
                        else:
                            self.get_logger().error("❌ 特征提取失败")
                    else:
                        self.get_logger().error('❌ 服务调用失败')
            
            cap.release()
            cv2.destroyAllWindows()
            return True
            
        except Exception as e:
            self.get_logger().error(f"摄像头测试失败: {e}")
            return False

    def print_response(self, response):
        """打印响应结果"""
        self.get_logger().info("=" * 50)
        self.get_logger().info("特征提取结果:")
        self.get_logger().info(f"成功: {response.success}")
        self.get_logger().info(f"消息: {response.message}")
        self.get_logger().info(f"检测到的人数: {response.person_count}")
        
        if response.success and response.person_count > 0:
            self.get_logger().info(f"身体比例数量: {len(response.body_ratios)}")
            if len(response.body_ratios) > 0:
                self.get_logger().info(f"身体比例样例: {response.body_ratios[:5]}...")
            
            self.get_logger().info(f"上装颜色 (RGB): {response.shirt_color}")
            self.get_logger().info(f"下装颜色 (RGB): {response.pants_color}")
            
            if response.result_image_path:
                self.get_logger().info(f"结果图像: {response.result_image_path}")
            
            if response.feature_data_path:
                self.get_logger().info(f"特征数据: {response.feature_data_path}")
        
        self.get_logger().info("=" * 50)


def main():
    """主函数"""
    rclpy.init()
    
    try:
        tester = FeatureExtractionTester()
        
        # 检查命令行参数
        if len(sys.argv) > 1:
            if sys.argv[1] == '--camera':
                # 摄像头测试
                camera_id = int(sys.argv[2]) if len(sys.argv) > 2 else 1
                person_name = sys.argv[3] if len(sys.argv) > 3 else "camera_test"
                tester.test_with_camera(camera_id, person_name)
            else:
                # 图像文件测试
                image_path = sys.argv[1]
                person_name = sys.argv[2] if len(sys.argv) > 2 else "file_test"
                success = tester.test_with_image_file(image_path, person_name)
                if success:
                    tester.get_logger().info("🎉 测试成功完成!")
                else:
                    tester.get_logger().error("💥 测试失败!")
        else:
            tester.get_logger().info("使用方法:")
            tester.get_logger().info("  图像文件测试: python3 test_feature_extraction.py <image_path> [person_name]")
            tester.get_logger().info("  摄像头测试: python3 test_feature_extraction.py --camera [camera_id] [person_name]")
            
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main() 