#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
简单特征提取测试节点
==================
用于测试特征提取服务的简单测试程序
- 读取指定图像文件
- 调用特征提取服务
- 输出结果并保存Excel文件
"""

import rclpy
from rclpy.node import Node
from custom_msgs.srv import FeatureExtraction
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import sys
import argparse
from pathlib import Path


class SimpleFeatureTester(Node):
    """简单特征提取测试节点"""
    
    def __init__(self):
        super().__init__('simple_feature_tester')
        
        # 初始化CV Bridge
        self.bridge = CvBridge()
        
        # 创建服务客户端
        self.client = self.create_client(FeatureExtraction, '/features/extract_features')
        
        self.get_logger().info('🚀 简单特征提取测试节点已启动')

    def wait_for_service(self, timeout_sec=10.0):
        """等待服务可用"""
        self.get_logger().info('⏳ 等待特征提取服务...')
        
        if not self.client.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().error(f'❌ 特征提取服务在 {timeout_sec} 秒内未响应')
            self.get_logger().error('请确保特征提取节点正在运行:')
            self.get_logger().error('  ros2 run following_robot feature_extraction_node')
            return False
        
        self.get_logger().info('✅ 特征提取服务已连接')
        return True

    def test_image_file(self, image_path, person_name="test_person", output_path=""):
        """测试图像文件的特征提取"""
        try:
            # 检查图像文件是否存在
            if not os.path.exists(image_path):
                self.get_logger().error(f"❌ 图像文件不存在: {image_path}")
                return False
            
            # 读取图像
            self.get_logger().info(f"📸 读取图像: {image_path}")
            cv_image = cv2.imread(image_path)
            
            if cv_image is None:
                self.get_logger().error(f"❌ 无法读取图像文件: {image_path}")
                return False
            
            self.get_logger().info(f"✅ 图像读取成功，尺寸: {cv_image.shape}")
            
            # 转换为ROS图像消息
            try:
                ros_image = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
                self.get_logger().info("✅ 图像转换为ROS消息成功")
            except Exception as e:
                self.get_logger().error(f"❌ 图像转换失败: {e}")
                return False
            
            # 创建服务请求
            request = FeatureExtraction.Request()
            request.image = ros_image
            request.person_name = person_name
            request.save_to_file = True
            request.output_path = output_path
            
            # 发送请求
            self.get_logger().info(f"🚀 发送特征提取请求: 人物名称='{person_name}'")
            future = self.client.call_async(request)
            
            # 等待响应
            self.get_logger().info("⏳ 等待特征提取结果...")
            rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
            
            if future.result() is not None:
                response = future.result()
                self.print_results(response)
                return response.success
            else:
                self.get_logger().error('❌ 服务调用超时或失败')
                return False
                
        except Exception as e:
            self.get_logger().error(f"❌ 测试过程中出错: {e}")
            return False

    def print_results(self, response):
        """打印测试结果"""
        print("\n" + "="*60)
        print("🎯 特征提取结果")
        print("="*60)
        
        print(f"状态: {'✅ 成功' if response.success else '❌ 失败'}")
        print(f"消息: {response.message}")
        print(f"检测到的人数: {response.person_count}")
        
        if response.success and response.person_count > 0:
            print(f"\n📊 身体比例特征 (共{len(response.body_ratios)}个):")
            if len(response.body_ratios) >= 16:
                ratio_names = [
                    "上肢/下肢比例", "躯干/身高比例", "肩宽/身高比例", "臀宽/肩宽比例",
                    "头部/躯干比例", "手臂/身高比例", "腿长/身高比例", "上臂/下臂比例",
                    "大腿/小腿比例", "躯干/腿长比例", "手臂/腿长比例", "肩宽/髋宽比例",
                    "头围/身高比例", "脚长/身高比例", "脚踝宽/身高比例", "腰围/身高比例"
                ]
                
                for i, (name, ratio) in enumerate(zip(ratio_names, response.body_ratios)):
                    print(f"  {i+1:2d}. {name:<15}: {ratio:.4f}")
            else:
                print(f"  比例数据: {response.body_ratios}")
            
            print(f"\n🎨 服装颜色信息:")
            print(f"  上装颜色 (RGB): {response.shirt_color}")
            print(f"  下装颜色 (RGB): {response.pants_color}")
            
            print(f"\n📁 输出文件:")
            if response.result_image_path:
                print(f"  结果图像: {response.result_image_path}")
            if response.feature_data_path:
                print(f"  Excel特征数据: {response.feature_data_path}")
                
        print("="*60)

    def run_test(self, image_path, person_name="test_person", output_path=""):
        """运行完整测试"""
        self.get_logger().info("🎬 开始特征提取测试")
        
        # 等待服务
        if not self.wait_for_service():
            return False
        
        # 执行测试
        success = self.test_image_file(image_path, person_name, output_path)
        
        if success:
            self.get_logger().info("🎉 测试成功完成！")
        else:
            self.get_logger().error("💥 测试失败！")
        
        return success


def main():
    """主函数"""
    # 解析命令行参数
    parser = argparse.ArgumentParser(description='简单特征提取测试程序')
    parser.add_argument('image_path', help='输入图像文件路径')
    parser.add_argument('--name', '-n', default='test_person', help='人物名称 (默认: test_person)')
    parser.add_argument('--output', '-o', default='', help='输出路径 (默认: features-data)')
    
    args = parser.parse_args()
    
    # 检查图像文件
    if not os.path.exists(args.image_path):
        print(f"❌ 错误: 图像文件不存在: {args.image_path}")
        return False
    
    # 初始化ROS2
    rclpy.init()
    
    try:
        # 创建测试节点
        tester = SimpleFeatureTester()
        
        # 运行测试
        success = tester.run_test(args.image_path, args.name, args.output)
        
        return success
        
    except KeyboardInterrupt:
        print("\n⏹️ 测试被用户中断")
        return False
    except Exception as e:
        print(f"💥 测试过程中出错: {e}")
        return False
    finally:
        try:
            rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    success = main()
    sys.exit(0 if success else 1) 