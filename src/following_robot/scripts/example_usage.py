#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
特征提取服务使用示例
==================
展示如何在Python代码中调用特征提取服务
"""

import rclpy
from rclpy.node import Node
from custom_msgs.srv import FeatureExtraction
from cv_bridge import CvBridge
import cv2
import numpy as np


class FeatureExtractionExample(Node):
    """特征提取使用示例类"""
    
    def __init__(self):
        super().__init__('feature_extraction_example')
        
        # 初始化CV Bridge
        self.bridge = CvBridge()
        
        # 创建服务客户端
        self.client = self.create_client(FeatureExtraction, '/features/extract_features')
        
        self.get_logger().info('特征提取示例节点已启动')

    def extract_features_from_image(self, image_path, person_name):
        """从图像文件提取特征"""
        try:
            # 等待服务可用
            if not self.client.wait_for_service(timeout_sec=5.0):
                self.get_logger().error('特征提取服务不可用')
                return None
            
            # 读取图像
            cv_image = cv2.imread(image_path)
            if cv_image is None:
                self.get_logger().error(f'无法读取图像: {image_path}')
                return None
            
            # 转换为ROS消息
            ros_image = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            
            # 创建服务请求
            request = FeatureExtraction.Request()
            request.image = ros_image
            request.person_name = person_name
            request.save_to_file = True
            request.output_path = ""
            
            # 调用服务
            future = self.client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            
            return future.result()
            
        except Exception as e:
            self.get_logger().error(f'特征提取失败: {e}')
            return None

    def extract_features_from_opencv_image(self, cv_image, person_name):
        """从OpenCV图像提取特征"""
        try:
            # 等待服务可用
            if not self.client.wait_for_service(timeout_sec=5.0):
                self.get_logger().error('特征提取服务不可用')
                return None
            
            # 转换为ROS消息
            ros_image = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            
            # 创建服务请求
            request = FeatureExtraction.Request()
            request.image = ros_image
            request.person_name = person_name
            request.save_to_file = True
            request.output_path = ""
            
            # 调用服务
            future = self.client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            
            return future.result()
            
        except Exception as e:
            self.get_logger().error(f'特征提取失败: {e}')
            return None

    def print_feature_results(self, response):
        """打印特征提取结果"""
        if response is None:
            print("❌ 没有获得响应")
            return
        
        print(f"\n{'='*50}")
        print("🎯 特征提取结果")
        print(f"{'='*50}")
        print(f"状态: {'✅ 成功' if response.success else '❌ 失败'}")
        print(f"消息: {response.message}")
        print(f"检测人数: {response.person_count}")
        
        if response.success and response.person_count > 0:
            print(f"\n📊 身体比例特征 ({len(response.body_ratios)}个):")
            for i, ratio in enumerate(response.body_ratios):
                print(f"  特征{i+1}: {ratio:.4f}")
            
            print(f"\n🎨 服装颜色:")
            print(f"  上装: RGB{tuple(response.shirt_color)}")
            print(f"  下装: RGB{tuple(response.pants_color)}")
            
            if response.result_image_path:
                print(f"\n📁 输出文件:")
                print(f"  结果图像: {response.result_image_path}")
            if response.feature_data_path:
                print(f"  Excel文件: {response.feature_data_path}")
        
        print(f"{'='*50}")


def demo_image_file():
    """演示从图像文件提取特征"""
    print("🎬 演示1: 从图像文件提取特征")
    
    rclpy.init()
    
    try:
        extractor = FeatureExtractionExample()
        
        # 创建测试图像
        test_image = np.ones((480, 640, 3), dtype=np.uint8) * 255
        cv2.rectangle(test_image, (250, 100), (390, 200), (0, 255, 0), -1)
        cv2.rectangle(test_image, (200, 200), (440, 350), (255, 0, 0), -1)
        cv2.rectangle(test_image, (220, 350), (420, 450), (0, 0, 255), -1)
        cv2.imwrite('demo_image.jpg', test_image)
        
        # 提取特征
        response = extractor.extract_features_from_image('demo_image.jpg', 'demo_person')
        extractor.print_feature_results(response)
        
    finally:
        rclpy.shutdown()


def demo_opencv_image():
    """演示从OpenCV图像提取特征"""
    print("🎬 演示2: 从OpenCV图像提取特征")
    
    rclpy.init()
    
    try:
        extractor = FeatureExtractionExample()
        
        # 创建测试图像
        test_image = np.ones((480, 640, 3), dtype=np.uint8) * 255
        cv2.rectangle(test_image, (250, 100), (390, 200), (0, 255, 0), -1)
        cv2.rectangle(test_image, (200, 200), (440, 350), (255, 0, 0), -1)
        cv2.rectangle(test_image, (220, 350), (420, 450), (0, 0, 255), -1)

        test_image = cv2.imread('/userdata/try_again/SelfFollowingROS2/src/following_robot/data/people.jpeg')
        
        # 直接从OpenCV图像提取特征
        response = extractor.extract_features_from_opencv_image(test_image, 'opencv_person')
        extractor.print_feature_results(response)
        
    finally:
        rclpy.shutdown()


def main():
    """主函数"""
    print("🚀 特征提取服务使用示例")
    print("请确保特征提取节点正在运行:")
    print("  ros2 run following_robot feature_extraction_node")
    print()
    
    # # 演示1: 图像文件
    # demo_image_file()
    
    # print("\n" + "-"*60 + "\n")
    
    # 演示2: OpenCV图像
    demo_opencv_image()
    
    print("\n🎉 演示完成!")


if __name__ == '__main__':
    main() 