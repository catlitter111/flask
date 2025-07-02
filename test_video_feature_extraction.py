#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
视频特征提取测试脚本
==================
测试从视频文件中提取人体特征的功能

使用方法:
    python test_video_feature_extraction.py <video_path>
"""

import rclpy
from rclpy.node import Node
from custom_msgs.srv import FeatureExtraction
from sensor_msgs.msg import Image
import sys
import cv2
from pathlib import Path

class VideoFeatureExtractionTester(Node):
    """视频特征提取测试类"""
    
    def __init__(self):
        super().__init__('video_feature_extraction_tester')
        
        # 创建服务客户端
        self.client = self.create_client(FeatureExtraction, '/features/extract_features')
        
        # 等待服务可用
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待特征提取服务...')
        
        self.get_logger().info('✅ 特征提取服务已连接')

    def test_video_extraction(self, video_path):
        """测试视频特征提取"""
        # 检查视频文件是否存在
        if not Path(video_path).exists():
            self.get_logger().error(f"视频文件不存在: {video_path}")
            return False
        
        self.get_logger().info(f"🎬 开始测试视频: {video_path}")
        
        # 创建请求
        request = FeatureExtraction.Request()
        request.person_name = f"VIDEO:{video_path}"  # 使用特殊前缀标识视频处理
        request.save_to_file = True
        request.output_path = ""  # 使用默认输出路径
        
        # 创建一个空的图像消息（视频处理时不使用）
        request.image = Image()
        
        try:
            # 发送请求
            self.get_logger().info("📤 发送视频特征提取请求...")
            future = self.client.call_async(request)
            
            # 等待结果
            rclpy.spin_until_future_complete(self, future, timeout_sec=300.0)  # 5分钟超时
            
            if future.result() is not None:
                response = future.result()
                
                if response.success:
                    self.get_logger().info("✅ 视频特征提取成功!")
                    self.get_logger().info(f"📊 检测结果: {response.message}")
                    self.get_logger().info(f"👥 检测到人数: {response.person_count}")
                    self.get_logger().info(f"📏 身体比例数据: {len(response.body_ratios)} 项")
                    self.get_logger().info(f"👕 上装颜色: {response.shirt_color}")
                    self.get_logger().info(f"👖 下装颜色: {response.pants_color}")
                    
                    if response.result_image_path:
                        self.get_logger().info(f"🖼️ 结果图像: {response.result_image_path}")
                    
                    if response.feature_data_path:
                        self.get_logger().info(f"📊 特征数据: {response.feature_data_path}")
                    
                    # 显示部分身体比例数据
                    if response.body_ratios:
                        self.get_logger().info("📐 前5个身体比例值:")
                        for i, ratio in enumerate(response.body_ratios[:5]):
                            self.get_logger().info(f"  比例{i+1}: {ratio:.4f}")
                    
                    return True
                else:
                    self.get_logger().error(f"❌ 视频特征提取失败: {response.message}")
                    return False
            else:
                self.get_logger().error("❌ 服务调用超时或失败")
                return False
                
        except Exception as e:
            self.get_logger().error(f"❌ 测试过程中出错: {e}")
            return False

def main():
    if len(sys.argv) != 2:
        print("使用方法: python test_video_feature_extraction.py <video_path>")
        print("示例: python test_video_feature_extraction.py /path/to/video.mp4")
        sys.exit(1)
    
    video_path = sys.argv[1]
    
    try:
        rclpy.init()
        
        # 创建测试节点
        tester = VideoFeatureExtractionTester()
        
        # 执行测试
        success = tester.test_video_extraction(video_path)
        
        if success:
            print("🎉 视频特征提取测试成功!")
        else:
            print("💥 视频特征提取测试失败!")
            sys.exit(1)
        
    except Exception as e:
        print(f"测试过程中发生错误: {e}")
        sys.exit(1)
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main() 