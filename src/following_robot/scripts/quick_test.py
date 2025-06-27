#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
快速特征提取测试
==============
快速测试特征提取功能的简单脚本
"""

import rclpy
from rclpy.node import Node
from custom_msgs.srv import FeatureExtraction
from cv_bridge import CvBridge
import cv2
import sys


def quick_test(image_path, person_name="quick_test"):
    """快速测试函数"""
    print(f"🚀 开始快速测试: {image_path}")
    
    # 初始化ROS2
    rclpy.init()
    
    try:
        # 创建节点
        node = Node('quick_tester')
        bridge = CvBridge()
        
        # 创建服务客户端
        client = node.create_client(FeatureExtraction, '/features/extract_features')
        
        # 等待服务
        print("⏳ 等待服务...")
        if not client.wait_for_service(timeout_sec=5.0):
            print("❌ 服务不可用，请先启动特征提取节点:")
            print("   ros2 run following_robot feature_extraction_node")
            return False
        
        # 读取图像
        print(f"📸 读取图像: {image_path}")
        cv_image = cv2.imread(image_path)
        if cv_image is None:
            print(f"❌ 无法读取图像: {image_path}")
            return False
        
        # 转换图像
        ros_image = bridge.cv2_to_imgmsg(cv_image, "bgr8")
        
        # 创建请求
        request = FeatureExtraction.Request()
        request.image = ros_image
        request.person_name = person_name
        request.save_to_file = True
        request.output_path = ""
        
        # 发送请求
        print("🔄 发送特征提取请求...")
        future = client.call_async(request)
        
        # 等待结果
        rclpy.spin_until_future_complete(node, future, timeout_sec=30.0)
        
        if future.result() is not None:
            response = future.result()
            
            # 打印结果
            print("\n" + "="*50)
            print("🎯 测试结果")
            print("="*50)
            print(f"状态: {'✅ 成功' if response.success else '❌ 失败'}")
            print(f"消息: {response.message}")
            print(f"检测人数: {response.person_count}")
            
            if response.success:
                print(f"身体比例数量: {len(response.body_ratios)}")
                print(f"上装颜色: {response.shirt_color}")
                print(f"下装颜色: {response.pants_color}")
                
                if response.result_image_path:
                    print(f"结果图像: {response.result_image_path}")
                if response.feature_data_path:
                    print(f"Excel文件: {response.feature_data_path}")
            
            print("="*50)
            return response.success
        else:
            print("❌ 服务调用失败")
            return False
            
    except Exception as e:
        print(f"❌ 错误: {e}")
        return False
    finally:
        rclpy.shutdown()


def main():
    """主函数"""
    if len(sys.argv) < 2:
        print("使用方法: python3 quick_test.py <图像路径> [人物名称]")
        print("示例: python3 quick_test.py test.jpg my_person")
        return
    
    image_path = sys.argv[1]
    person_name = sys.argv[2] if len(sys.argv) > 2 else "quick_test"
    
    success = quick_test(image_path, person_name)
    
    if success:
        print("🎉 测试成功！")
    else:
        print("💥 测试失败！")


if __name__ == '__main__':
    main() 