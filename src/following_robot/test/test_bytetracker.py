#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ByteTracker集成测试脚本
=====================
测试ByteTracker节点的各项功能
"""

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import time
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from custom_msgs.msg import TrackedPersonArray


class ByteTrackerTester(Node):
    """ByteTracker测试节点"""
    
    def __init__(self):
        super().__init__('bytetracker_tester')
        
        self.bridge = CvBridge()
        self.test_passed = False
        
        # 创建发布者
        self.image_pub = self.create_publisher(
            Image, '/camera/image_raw', 10)
        self.mode_pub = self.create_publisher(
            String, '/bytetracker/set_mode', 10)
        
        # 创建订阅者
        self.tracked_sub = self.create_subscription(
            TrackedPersonArray,
            '/bytetracker/tracked_persons',
            self.tracked_callback,
            10)
        
        self.viz_sub = self.create_subscription(
            Image,
            '/bytetracker/visualization',
            self.viz_callback,
            10)
        
        # 测试计数器
        self.frame_count = 0
        self.tracked_count = 0
        self.viz_count = 0
        
        # 创建定时器
        self.timer = self.create_timer(0.1, self.publish_test_image)
        
        self.get_logger().info('ByteTracker测试节点已启动')
        
    def publish_test_image(self):
        """发布测试图像"""
        # 创建测试图像（640x480）
        test_img = np.zeros((480, 640, 3), dtype=np.uint8)
        
        # 添加一些移动的矩形模拟人物
        offset = int(self.frame_count * 5) % 400
        
        # 模拟人物1（穿蓝色上衣）
        cv2.rectangle(test_img, (50 + offset, 100), (150 + offset, 200), (255, 0, 0), -1)  # 上衣
        cv2.rectangle(test_img, (50 + offset, 200), (150 + offset, 350), (0, 0, 255), -1)  # 下装
        
        # 模拟人物2（穿绿色上衣）
        cv2.rectangle(test_img, (300, 100), (400, 200), (0, 255, 0), -1)  # 上衣
        cv2.rectangle(test_img, (300, 200), (400, 350), (128, 128, 128), -1)  # 下装
        
        # 添加帧号
        cv2.putText(test_img, f"Frame: {self.frame_count}", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        
        # 发布图像
        img_msg = self.bridge.cv2_to_imgmsg(test_img, "bgr8")
        self.image_pub.publish(img_msg)
        
        self.frame_count += 1
        
        # 在第50帧切换到单目标模式
        if self.frame_count == 50:
            self.get_logger().info('切换到单目标跟踪模式')
            mode_msg = String()
            mode_msg.data = "single"
            self.mode_pub.publish(mode_msg)
        
        # 100帧后结束测试
        if self.frame_count >= 100:
            self.timer.cancel()
            self.print_test_results()
    
    def tracked_callback(self, msg):
        """跟踪结果回调"""
        self.tracked_count += 1
        self.get_logger().info(f'收到跟踪结果: {len(msg.persons)}个人物')
        
        for person in msg.persons:
            self.get_logger().debug(
                f'  ID: {person.track_id}, '
                f'位置: ({person.bbox.x_offset}, {person.bbox.y_offset}), '
                f'置信度: {person.confidence:.2f}'
            )
    
    def viz_callback(self, msg):
        """可视化回调"""
        self.viz_count += 1
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            # 可以在这里显示或保存图像
            # cv2.imshow('ByteTracker Visualization', cv_img)
            # cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f'可视化图像转换失败: {e}')
    
    def print_test_results(self):
        """打印测试结果"""
        self.get_logger().info('=' * 50)
        self.get_logger().info('ByteTracker测试结果:')
        self.get_logger().info(f'  发送帧数: {self.frame_count}')
        self.get_logger().info(f'  收到跟踪结果: {self.tracked_count}')
        self.get_logger().info(f'  收到可视化图像: {self.viz_count}')
        
        # 判断测试是否通过
        if self.tracked_count > 0 and self.viz_count > 0:
            self.get_logger().info('✅ 测试通过！ByteTracker节点工作正常')
            self.test_passed = True
        else:
            self.get_logger().error('❌ 测试失败！请检查ByteTracker节点')
            
        self.get_logger().info('=' * 50)
        
        # 关闭节点
        rclpy.shutdown()


def main(args=None):
    """主函数"""
    print("\n" + "="*50)
    print("ByteTracker集成测试")
    print("="*50)
    print("测试说明：")
    print("1. 确保ByteTracker节点已启动")
    print("2. 本测试将发送模拟图像并验证跟踪结果")
    print("3. 测试将自动切换跟踪模式")
    print("="*50 + "\n")
    
    rclpy.init(args=args)
    
    tester = ByteTrackerTester()
    
    try:
        rclpy.spin(tester)
    except KeyboardInterrupt:
        pass
    finally:
        tester.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()