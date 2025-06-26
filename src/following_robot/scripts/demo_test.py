#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
双目视觉系统演示测试脚本
=====================
演示如何使用距离测量服务

作者: AI Assistant
"""

import rclpy
from rclpy.node import Node
from custom_msgs.srv import GetDistance
import traceback
import time


class DemoTester(Node):
    """演示测试节点
    
    参数:
        无
        
    返回值:
        节点实例
    """
    
    def __init__(self):
        try:
            super().__init__('demo_tester')
            
            # 创建服务客户端
            self.distance_client = self.create_client(
                GetDistance,
                '/stereo/get_distance'
            )
            
            self.get_logger().info('演示测试节点启动')
            
        except Exception as e:
            self.get_logger().error(f"演示节点初始化错误: {e}")
            traceback.print_exc()

    def wait_for_service(self):
        """等待距离测量服务
        
        参数:
            无
            
        返回值:
            bool: 服务是否可用
        """
        try:
            self.get_logger().info('等待距离测量服务...')
            if self.distance_client.wait_for_service(timeout_sec=10.0):
                self.get_logger().info('距离测量服务已连接')
                return True
            else:
                self.get_logger().error('距离测量服务连接超时')
                return False
                
        except Exception as e:
            self.get_logger().error(f"等待服务错误: {e}")
            traceback.print_exc()
            return False

    def test_distance_service(self):
        """测试距离测量服务
        
        参数:
            无
            
        返回值:
            无
        """
        try:
            if not self.wait_for_service():
                return
            
            # 测试点列表 (x, y)
            test_points = [
                (320, 240),  # 图像中心
                (160, 120),  # 左上角
                (480, 360),  # 右下角
                (0, 0),      # 边界测试
                (640, 480),  # 边界测试
            ]
            
            self.get_logger().info('开始距离测量测试...')
            
            for i, (x, y) in enumerate(test_points):
                self.get_logger().info(f'测试点 {i+1}: ({x}, {y})')
                
                # 创建服务请求
                request = GetDistance.Request()
                request.x = x
                request.y = y
                
                try:
                    # 同步调用服务
                    future = self.distance_client.call_async(request)
                    rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
                    
                    if future.done():
                        response = future.result()
                        if response.success:
                            self.get_logger().info(
                                f'  ✓ 距离: {response.distance:.2f}m - {response.message}'
                            )
                        else:
                            self.get_logger().warn(
                                f'  ✗ 测量失败: {response.message}'
                            )
                    else:
                        self.get_logger().warn(f'  ✗ 服务调用超时')
                        
                except Exception as e:
                    self.get_logger().error(f'  ✗ 服务调用错误: {e}')
                    traceback.print_exc()
                
                time.sleep(1)  # 间隔1秒
            
            self.get_logger().info('距离测量测试完成')
            
        except Exception as e:
            self.get_logger().error(f"距离服务测试错误: {e}")
            traceback.print_exc()


def main(args=None):
    """主函数
    
    参数:
        args: 命令行参数
        
    返回值:
        无
    """
    try:
        rclpy.init(args=args)
        node = DemoTester()
        
        # 执行测试
        node.test_distance_service()
        
        # 保持节点运行一会儿
        rclpy.spin_once(node)
        
        node.destroy_node()
        rclpy.shutdown()
        
        print("\n演示测试完成!")
        print("如果双目相机可用，您应该能看到实际的距离测量结果。")
        print("如果相机不可用，服务调用会返回相应的错误信息。")
        
    except Exception as e:
        print(f"主函数错误: {e}")
        traceback.print_exc()


if __name__ == '__main__':
    main() 