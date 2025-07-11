#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
深度集成测试脚本
===============

功能说明：
    测试following_robot包中的深度读取功能
    验证ByteTracker节点和深度读取节点的集成
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time


class DepthIntegrationTest(Node):
    """深度集成测试节点"""
    
    def __init__(self):
        super().__init__('depth_integration_test')
        
        # 创建发布者和订阅者
        self.depth_query_pub = self.create_publisher(
            String,
            '/following_robot/get_depth_at',
            10
        )
        
        self.depth_result_sub = self.create_subscription(
            String,
            '/following_robot/depth_value',
            self.depth_result_callback,
            10
        )
        
        self.results = []
        self.test_points = [
            (320, 240),  # 图像中心
            (160, 120),  # 左上
            (480, 360),  # 右下
            (100, 200),  # 随机点1
            (500, 300),  # 随机点2
        ]
        
        self.get_logger().info('🧪 深度集成测试启动')
        
        # 等待系统初始化
        time.sleep(2)
        self.run_tests()
    
    def depth_result_callback(self, msg):
        """接收深度查询结果"""
        try:
            result = json.loads(msg.data)
            self.results.append(result)
            
            x = result.get('x', 0)
            y = result.get('y', 0)
            depth_m = result.get('depth_m', None)
            valid = result.get('valid', False)
            
            if valid and depth_m is not None:
                self.get_logger().info(f'✅ 深度查询成功: ({x},{y}) = {depth_m:.3f}m')
            else:
                error = result.get('error', 'Unknown error')
                self.get_logger().warn(f'⚠️ 深度查询失败: ({x},{y}) - {error}')
                
        except Exception as e:
            self.get_logger().error(f'❌ 结果解析错误: {e}')
    
    def run_tests(self):
        """运行测试"""
        self.get_logger().info('🚀 开始深度查询测试...')
        
        for i, (x, y) in enumerate(self.test_points):
            self.get_logger().info(f'📍 测试点 {i+1}: ({x}, {y})')
            
            # 发送查询请求
            query = {
                "x": x,
                "y": y,
                "mode": "single"
            }
            
            query_msg = String()
            query_msg.data = json.dumps(query)
            self.depth_query_pub.publish(query_msg)
            
            # 等待结果
            time.sleep(0.5)
        
        # 等待所有结果
        time.sleep(2)
        
        # 显示测试总结
        self.show_test_summary()
    
    def show_test_summary(self):
        """显示测试总结"""
        self.get_logger().info('📊 测试总结:')
        self.get_logger().info(f'   总查询数: {len(self.test_points)}')
        self.get_logger().info(f'   收到结果: {len(self.results)}')
        
        valid_results = [r for r in self.results if r.get('valid', False)]
        self.get_logger().info(f'   有效结果: {len(valid_results)}')
        
        if valid_results:
            depths = [r['depth_m'] for r in valid_results]
            avg_depth = sum(depths) / len(depths)
            min_depth = min(depths)
            max_depth = max(depths)
            
            self.get_logger().info(f'   平均深度: {avg_depth:.3f}m')
            self.get_logger().info(f'   深度范围: {min_depth:.3f}m - {max_depth:.3f}m')
        
        # 测试区域查询
        self.test_region_query()
    
    def test_region_query(self):
        """测试区域查询功能"""
        self.get_logger().info('🔍 测试区域查询功能...')
        
        query = {
            "x": 320,
            "y": 240,
            "mode": "region",
            "radius": 10
        }
        
        query_msg = String()
        query_msg.data = json.dumps(query)
        self.depth_query_pub.publish(query_msg)
        
        time.sleep(1)
        
        # 测试统计查询
        self.test_statistics_query()
    
    def test_statistics_query(self):
        """测试统计查询功能"""
        self.get_logger().info('📈 测试统计查询功能...')
        
        query = {
            "x": 0,
            "y": 0,
            "mode": "statistics"
        }
        
        query_msg = String()
        query_msg.data = json.dumps(query)
        self.depth_query_pub.publish(query_msg)
        
        time.sleep(1)
        
        self.get_logger().info('🎉 所有测试完成！')


def main():
    """主函数"""
    rclpy.init()
    
    try:
        test_node = DepthIntegrationTest()
        
        # 运行测试一段时间
        start_time = time.time()
        while rclpy.ok() and (time.time() - start_time) < 10:
            rclpy.spin_once(test_node, timeout_sec=0.1)
            
    except KeyboardInterrupt:
        print('测试被中断')
    finally:
        if 'test_node' in locals():
            test_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()