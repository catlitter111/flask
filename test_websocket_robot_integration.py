#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
WebSocket和机器人控制节点集成测试
================================

测试WebSocket桥接节点和机器人控制节点之间的通信
验证控制命令的传递和执行
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time
import threading

class IntegrationTester(Node):
    """集成测试节点"""
    
    def __init__(self):
        super().__init__('integration_tester')
        
        # 发布者 - 模拟WebSocket命令
        self.command_publisher = self.create_publisher(
            String, '/robot_control/command', 10)
        
        # 订阅者 - 监听状态
        self.status_subscriber = self.create_subscription(
            String, '/robot_control/status', 
            self.status_callback, 10)
        
        self.mode_subscriber = self.create_subscription(
            String, '/robot_control/mode',
            self.mode_callback, 10)
        
        # 测试状态
        self.test_results = []
        self.current_mode = ""
        self.last_status = ""
        
        self.get_logger().info("🧪 集成测试节点启动")
        
        # 启动测试序列
        self.start_test_sequence()
    
    def status_callback(self, msg):
        """状态回调"""
        self.last_status = msg.data
        self.get_logger().debug(f"收到状态: {msg.data}")
    
    def mode_callback(self, msg):
        """模式回调"""
        self.current_mode = msg.data
        self.get_logger().info(f"🔄 模式变更: {msg.data}")
    
    def send_command(self, command):
        """发送控制命令"""
        msg = String()
        msg.data = command
        self.command_publisher.publish(msg)
        self.get_logger().info(f"📤 发送命令: {command}")
    
    def start_test_sequence(self):
        """启动测试序列"""
        def test_thread():
            time.sleep(2)  # 等待系统初始化
            
            # 测试序列
            test_commands = [
                ("motor_forward:50", "测试电机前进"),
                ("motor_stop", "测试电机停止"),
                ("motor_left:30", "测试电机左转"),
                ("motor_stop", "测试电机停止"),
                ("motor_right:30", "测试电机右转"),
                ("motor_stop", "测试电机停止"),
                ("motor_backward:40", "测试电机后退"),
                ("motor_stop", "测试电机停止"),
                ("start_auto_mode", "测试自动模式启动"),
                ("pause_auto_mode", "测试自动模式暂停"),
                ("start_interaction", "测试交互模式启动"),
                ("companion_turn_left", "测试伴侣左转"),
                ("companion_stop", "测试伴侣停止"),
                ("stop_interaction", "测试停止交互模式"),
                ("set_motor_speed:75", "测试设置电机速度"),
                ("switch_control_type:companion", "测试切换控制类型"),
                ("emergency_stop", "测试紧急停止"),
            ]
            
            for command, description in test_commands:
                self.get_logger().info(f"🧪 {description}")
                self.send_command(command)
                time.sleep(3)  # 等待命令执行
            
            self.get_logger().info("✅ 测试序列完成")
            
        # 在单独线程中运行测试
        test_thread_obj = threading.Thread(target=test_thread)
        test_thread_obj.daemon = True
        test_thread_obj.start()


def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    try:
        tester = IntegrationTester()
        rclpy.spin(tester)
    except KeyboardInterrupt:
        print("\n🛑 测试被中断")
    finally:
        if 'tester' in locals():
            tester.destroy_node()
        rclpy.shutdown()
        print("👋 测试结束")


if __name__ == '__main__':
    main() 