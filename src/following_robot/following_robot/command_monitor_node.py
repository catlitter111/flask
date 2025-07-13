#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
命令监控节点 - 新增功能
===================
功能：
- 监控并显示发送给小车的所有命令
- 实时显示串口指令和用户友好的描述
- 提供命令统计和调试信息

作者: AI Assistant
日期: 2024
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
import json
import time
from datetime import datetime

from std_msgs.msg import String
from geometry_msgs.msg import Twist


class CommandMonitorNode(Node):
    """命令监控节点"""
    
    def __init__(self):
        super().__init__('command_monitor_node')
        
        # 统计信息
        self.command_stats = {
            "total_commands": 0,
            "motor_commands": 0,
            "following_commands": 0,
            "mode_switches": 0,
            "last_action": "启动监控",
            "start_time": time.time()
        }
        
        self.setup_subscribers()
        self.setup_timer()
        
        self.get_logger().info("🖥️ 命令监控节点启动")
        self.get_logger().info("📊 开始监控小车控制命令...")
    
    def setup_subscribers(self):
        """设置订阅者"""
        qos = QoSProfile(depth=10)
        
        # 监控命令监控话题
        self.command_monitor_sub = self.create_subscription(
            String, '/robot_control/command_monitor',
            self.command_monitor_callback, qos)
        
        # 监控串口命令话题
        self.serial_monitor_sub = self.create_subscription(
            String, '/robot_control/serial_monitor',
            self.serial_monitor_callback, qos)
        
        # 监控WebSocket命令（直接从bridge节点）
        self.websocket_command_sub = self.create_subscription(
            String, '/websocket_bridge/command',
            self.websocket_command_callback, qos)
        
        # 监控实际cmd_vel（发送给硬件的最终命令）
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel',
            self.cmd_vel_callback, qos)
    
    def setup_timer(self):
        """设置定时器"""
        # 统计信息发布定时器（每5秒）
        self.stats_timer = self.create_timer(5.0, self.publish_stats)
    
    def command_monitor_callback(self, msg):
        """命令监控回调"""
        try:
            data = json.loads(msg.data)
            category = data.get('category', 'Unknown')
            action = data.get('action', 'Unknown')
            details = data.get('details', '')
            mode = data.get('mode', 'Unknown')
            
            self.command_stats["total_commands"] += 1
            self.command_stats["last_action"] = action
            
            # 分类统计
            if category == 'Motor':
                self.command_stats["motor_commands"] += 1
            elif category == 'Mode':
                self.command_stats["mode_switches"] += 1
            
            # 显示命令信息
            timestamp = datetime.now().strftime("%H:%M:%S")
            self.get_logger().info(
                f"🎮 [{timestamp}] {category}: {action} | 详情: {details} | 模式: {mode}"
            )
            
        except Exception as e:
            self.get_logger().error(f"解析命令监控数据失败: {e}")
    
    def serial_monitor_callback(self, msg):
        """串口监控回调"""
        try:
            data = json.loads(msg.data)
            action = data.get('action', 'Unknown')
            speed_percent = data.get('speed_percent', 0)
            linear_vel = data.get('linear_velocity', 0.0)
            angular_vel = data.get('angular_velocity', 0.0)
            serial_cmd = data.get('serial_command', '')
            cmd_count = data.get('command_count', 0)
            
            self.command_stats["following_commands"] += 1
            
            # 显示串口命令信息
            timestamp = datetime.now().strftime("%H:%M:%S")
            self.get_logger().info(
                f"📡 [{timestamp}] 小车动作: {action}, 速度{speed_percent}% "
                f"(线:{linear_vel:.3f}, 角:{angular_vel:.3f})"
            )
            self.get_logger().info(f"    ➤ 实际串口指令: {serial_cmd}")
            self.get_logger().info(f"    ➤ 命令序号: #{cmd_count}")
            
        except Exception as e:
            self.get_logger().error(f"解析串口监控数据失败: {e}")
    
    def websocket_command_callback(self, msg):
        """WebSocket命令回调"""
        command = msg.data
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.get_logger().info(f"🌐 [{timestamp}] WebSocket命令: {command}")
    
    def cmd_vel_callback(self, msg):
        """cmd_vel监控回调"""
        # 只在有实际运动时显示
        if abs(msg.linear.x) > 0.001 or abs(msg.angular.z) > 0.001:
            timestamp = datetime.now().strftime("%H:%M:%S")
            self.get_logger().debug(
                f"🚗 [{timestamp}] 最终cmd_vel: 线速度={msg.linear.x:.3f}, 角速度={msg.angular.z:.3f}"
            )
    
    def publish_stats(self):
        """发布统计信息"""
        uptime = time.time() - self.command_stats["start_time"]
        uptime_str = f"{int(uptime//60)}分{int(uptime%60)}秒"
        
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"📊 命令监控统计 (运行时间: {uptime_str})")
        self.get_logger().info(f"   总命令数: {self.command_stats['total_commands']}")
        self.get_logger().info(f"   电机命令: {self.command_stats['motor_commands']}")
        self.get_logger().info(f"   跟随命令: {self.command_stats['following_commands']}")
        self.get_logger().info(f"   模式切换: {self.command_stats['mode_switches']}")
        self.get_logger().info(f"   最后动作: {self.command_stats['last_action']}")
        self.get_logger().info("=" * 60)


def main(args=None):
    """主函数"""
    try:
        rclpy.init(args=args)
        node = CommandMonitorNode()
        
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            node.get_logger().info("🛑 命令监控节点停止")
        finally:
            node.destroy_node()
            rclpy.shutdown()
            
    except Exception as e:
        print(f"❌ 命令监控节点错误: {e}")


if __name__ == '__main__':
    main()