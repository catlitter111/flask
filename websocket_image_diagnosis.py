#!/usr/bin/env python3
"""
WebSocket图像传输诊断脚本
========================

功能：
1. 检查图像话题状态
2. 检查WebSocket连接状态
3. 监控图像传输频率
4. 检查QoS兼容性
5. 提供修复建议

作者：AI Assistant
日期：2024
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import time
import subprocess
import json
import threading
import websocket
import sys


class WebSocketImageDiagnosisNode(Node):
    """WebSocket图像传输诊断节点"""
    
    def __init__(self):
        super().__init__('websocket_image_diagnosis')
        
        # 统计信息
        self.image_count = 0
        self.last_image_time = 0
        self.start_time = time.time()
        
        # 创建QoS兼容的订阅者
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
        
        image_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE
        )
        
        # 订阅图像话题
        self.image_sub = self.create_subscription(
            Image,
            '/bytetracker/visualization',
            self.image_callback,
            image_qos
        )
        
        # 启动诊断
        self.run_diagnosis()
        
        # 启动定时器
        self.timer = self.create_timer(5.0, self.print_stats)
    
    def image_callback(self, msg):
        """图像回调函数"""
        self.image_count += 1
        self.last_image_time = time.time()
        
        # 检查图像数据
        if hasattr(msg, 'width') and hasattr(msg, 'height'):
            if self.image_count % 100 == 0:
                self.get_logger().info(f'📹 接收图像 #{self.image_count}: {msg.width}x{msg.height}')
    
    def print_stats(self):
        """打印统计信息"""
        current_time = time.time()
        duration = current_time - self.start_time
        
        if duration > 0:
            fps = self.image_count / duration
            self.get_logger().info(f'📊 图像接收统计: {self.image_count}帧, 平均FPS: {fps:.2f}')
            
            if self.last_image_time > 0:
                age = current_time - self.last_image_time
                if age > 2:
                    self.get_logger().warning(f'⚠️ 图像数据过期 ({age:.1f}秒前)')
    
    def run_diagnosis(self):
        """运行诊断"""
        print("\n🔍 开始WebSocket图像传输诊断...")
        
        # 1. 检查话题状态
        print("\n1. 检查话题状态...")
        self.check_topic_status()
        
        # 2. 检查节点状态
        print("\n2. 检查节点状态...")
        self.check_node_status()
        
        # 3. 检查网络连接
        print("\n3. 检查网络连接...")
        self.check_network_connection()
        
        # 4. 检查参数配置
        print("\n4. 检查参数配置...")
        self.check_parameters()
        
        print("\n✅ 诊断完成！")
    
    def check_topic_status(self):
        """检查话题状态"""
        try:
            # 检查话题信息
            result = subprocess.run(
                ['ros2', 'topic', 'info', '/bytetracker/visualization'],
                capture_output=True, text=True, timeout=5
            )
            
            if result.returncode == 0:
                lines = result.stdout.strip().split('\n')
                for line in lines:
                    if 'Publisher count' in line:
                        pub_count = line.split(':')[1].strip()
                        print(f"  📤 发布者数量: {pub_count}")
                    elif 'Subscription count' in line:
                        sub_count = line.split(':')[1].strip()
                        print(f"  📥 订阅者数量: {sub_count}")
                        
                # 检查发布频率
                print("  📊 检查发布频率...")
                result = subprocess.run(
                    ['timeout', '3', 'ros2', 'topic', 'hz', '/bytetracker/visualization'],
                    capture_output=True, text=True
                )
                
                if result.returncode == 0 and 'average rate' in result.stdout:
                    lines = result.stdout.strip().split('\n')
                    for line in lines:
                        if 'average rate' in line:
                            rate = line.split(':')[1].strip().split()[0]
                            print(f"  📈 发布频率: {rate} Hz")
                            break
                else:
                    print("  ❌ 无法获取发布频率或话题无数据")
            else:
                print("  ❌ 话题不存在或无法访问")
                
        except Exception as e:
            print(f"  ❌ 检查话题状态失败: {e}")
    
    def check_node_status(self):
        """检查节点状态"""
        try:
            # 检查相关节点
            result = subprocess.run(
                ['ros2', 'node', 'list'],
                capture_output=True, text=True, timeout=5
            )
            
            if result.returncode == 0:
                nodes = result.stdout.strip().split('\n')
                
                person_node = '/person_detection_distance_node'
                websocket_node = '/websocket_bridge_node'
                
                if person_node in nodes:
                    print(f"  ✅ Person检测节点运行中: {person_node}")
                else:
                    print(f"  ❌ Person检测节点未运行: {person_node}")
                
                if websocket_node in nodes:
                    print(f"  ✅ WebSocket桥接节点运行中: {websocket_node}")
                else:
                    print(f"  ❌ WebSocket桥接节点未运行: {websocket_node}")
            else:
                print("  ❌ 无法获取节点列表")
                
        except Exception as e:
            print(f"  ❌ 检查节点状态失败: {e}")
    
    def check_network_connection(self):
        """检查网络连接"""
        try:
            # 检查WebSocket服务器连接
            result = subprocess.run(
                ['nc', '-z', '-v', '101.201.150.96', '1234'],
                capture_output=True, text=True, timeout=5
            )
            
            if result.returncode == 0:
                print("  ✅ WebSocket服务器端口可达")
            else:
                print("  ❌ WebSocket服务器端口不可达")
                print(f"     错误信息: {result.stderr}")
                
        except Exception as e:
            print(f"  ❌ 检查网络连接失败: {e}")
    
    def check_parameters(self):
        """检查参数配置"""
        try:
            # 检查关键参数
            params_to_check = [
                ('enable_image_stream', 'Boolean'),
                ('websocket_host', 'String'),
                ('websocket_port', 'Integer')
            ]
            
            for param_name, param_type in params_to_check:
                result = subprocess.run(
                    ['ros2', 'param', 'get', '/websocket_bridge_node', param_name],
                    capture_output=True, text=True, timeout=5
                )
                
                if result.returncode == 0:
                    value = result.stdout.strip().split(':')[1].strip()
                    print(f"  📋 {param_name}: {value}")
                else:
                    print(f"  ❌ 无法获取参数: {param_name}")
                    
        except Exception as e:
            print(f"  ❌ 检查参数配置失败: {e}")


def main(args=None):
    """主函数"""
    print("🔍 WebSocket图像传输诊断工具")
    print("=" * 50)
    
    rclpy.init(args=args)
    
    try:
        node = WebSocketImageDiagnosisNode()
        
        print("\n📡 开始监控图像传输...")
        print("按 Ctrl+C 停止监控")
        
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        print("\n🛑 监控已停止")
    except Exception as e:
        print(f"\n❌ 诊断失败: {e}")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 