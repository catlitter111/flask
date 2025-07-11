#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
人体跟随系统测试脚本
==================
功能: 测试完整的人体跟随机器人系统
包含:
- 系统启动测试
- 话题连接测试
- 控制逻辑测试
- 安全性测试

作者: AI Assistant
日期: 2024
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import json
import time
import threading
import subprocess
import sys
import os


class PersonFollowingSystemTester(Node):
    """人体跟随系统测试节点"""
    
    def __init__(self):
        super().__init__('person_following_system_tester')
        
        # 测试结果
        self.test_results = {}
        self.received_messages = {}
        
        # 创建订阅者监听关键话题
        self.create_test_subscribers()
        
        # 创建发布者用于测试
        self.create_test_publishers()
        
        self.get_logger().info('🧪 人体跟随系统测试节点启动')
    
    def create_test_subscribers(self):
        """创建测试订阅者"""
        # 监听人体检测结果
        self.person_detection_sub = self.create_subscription(
            String, '/person_detection/person_positions',
            self.person_detection_callback, 10)
        
        # 监听控制指令
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel',
            self.cmd_vel_callback, 10)
        
        # 监听跟随状态
        self.following_status_sub = self.create_subscription(
            String, '/person_following/status',
            self.following_status_callback, 10)
        
        # 监听相机图像
        self.camera_image_sub = self.create_subscription(
            Image, '/camera/color/image_raw',
            self.camera_image_callback, 10)
    
    def create_test_publishers(self):
        """创建测试发布者"""
        # 用于发送模拟人体检测数据
        self.test_person_pub = self.create_publisher(
            String, '/person_detection/person_positions', 10)
    
    def person_detection_callback(self, msg):
        """人体检测回调"""
        try:
            data = json.loads(msg.data)
            self.received_messages['person_detection'] = {
                'timestamp': time.time(),
                'person_count': data.get('person_count', 0),
                'persons': data.get('persons', [])
            }
            self.get_logger().info(f'✅ 接收到人体检测数据: {data.get("person_count", 0)} 个人')
        except Exception as e:
            self.get_logger().error(f'❌ 人体检测数据解析失败: {e}')
    
    def cmd_vel_callback(self, msg):
        """控制指令回调"""
        self.received_messages['cmd_vel'] = {
            'timestamp': time.time(),
            'linear_x': msg.linear.x,
            'angular_z': msg.angular.z
        }
        if abs(msg.linear.x) > 0.01 or abs(msg.angular.z) > 0.01:
            self.get_logger().info(f'✅ 接收到控制指令: 线速度={msg.linear.x:.3f}, 角速度={msg.angular.z:.3f}')
    
    def following_status_callback(self, msg):
        """跟随状态回调"""
        try:
            data = json.loads(msg.data)
            self.received_messages['following_status'] = {
                'timestamp': time.time(),
                'is_following': data.get('is_following', False),
                'target_person': data.get('target_person'),
                'persons_count': data.get('persons_count', 0)
            }
            self.get_logger().info(f'✅ 跟随状态: 跟随={data.get("is_following")}, 目标={data.get("target_person")}')
        except Exception as e:
            self.get_logger().error(f'❌ 跟随状态解析失败: {e}')
    
    def camera_image_callback(self, msg):
        """相机图像回调"""
        self.received_messages['camera_image'] = {
            'timestamp': time.time(),
            'width': msg.width,
            'height': msg.height,
            'encoding': msg.encoding
        }
        self.get_logger().info(f'✅ 接收到相机图像: {msg.width}x{msg.height}, 编码={msg.encoding}')
    
    def publish_test_person_data(self, distance=1.5, x_position=320):
        """发布测试人体数据"""
        test_data = {
            'timestamp': time.time(),
            'person_count': 1,
            'persons': [
                {
                    'id': 'test_person_1',
                    'bbox': [x_position-50, 200, x_position+50, 400],
                    'center': [x_position, 300],
                    'distance': distance,
                    'valid_distance': True
                }
            ]
        }
        
        msg = String()
        msg.data = json.dumps(test_data)
        self.test_person_pub.publish(msg)
        self.get_logger().info(f'📤 发布测试人体数据: 距离={distance}m, 位置=({x_position}, 300)')
    
    def run_tests(self):
        """运行测试序列"""
        self.get_logger().info('🧪 开始运行系统测试')
        
        # 等待系统启动
        self.get_logger().info('⏳ 等待系统启动...')
        time.sleep(5)
        
        # 测试1: 话题连接测试
        self.test_topic_connections()
        
        # 测试2: 模拟人体检测测试
        self.test_simulated_person_detection()
        
        # 测试3: 跟随控制测试
        self.test_following_control()
        
        # 测试4: 安全性测试
        self.test_safety_features()
        
        # 输出测试结果
        self.print_test_results()
    
    def test_topic_connections(self):
        """测试话题连接"""
        self.get_logger().info('🧪 测试话题连接...')
        
        # 等待消息接收
        start_time = time.time()
        timeout = 10.0
        
        while time.time() - start_time < timeout:
            if len(self.received_messages) >= 2:  # 至少接收2种消息
                break
            time.sleep(0.1)
        
        # 检查结果
        connected_topics = list(self.received_messages.keys())
        self.test_results['topic_connections'] = {
            'passed': len(connected_topics) >= 2,
            'connected_topics': connected_topics
        }
        
        if self.test_results['topic_connections']['passed']:
            self.get_logger().info(f'✅ 话题连接测试通过: {connected_topics}')
        else:
            self.get_logger().error(f'❌ 话题连接测试失败: 只连接了 {connected_topics}')
    
    def test_simulated_person_detection(self):
        """测试模拟人体检测"""
        self.get_logger().info('🧪 测试模拟人体检测...')
        
        # 发送测试数据
        self.publish_test_person_data(distance=1.5, x_position=320)
        
        # 等待响应
        time.sleep(2)
        
        # 检查是否收到跟随状态更新
        has_following_status = 'following_status' in self.received_messages
        has_cmd_vel = 'cmd_vel' in self.received_messages
        
        self.test_results['simulated_detection'] = {
            'passed': has_following_status and has_cmd_vel,
            'has_following_status': has_following_status,
            'has_cmd_vel': has_cmd_vel
        }
        
        if self.test_results['simulated_detection']['passed']:
            self.get_logger().info('✅ 模拟人体检测测试通过')
        else:
            self.get_logger().error('❌ 模拟人体检测测试失败')
    
    def test_following_control(self):
        """测试跟随控制逻辑"""
        self.get_logger().info('🧪 测试跟随控制逻辑...')
        
        test_cases = [
            {'distance': 0.8, 'x_pos': 320, 'expected': 'backward'},  # 太近，应该后退
            {'distance': 2.0, 'x_pos': 320, 'expected': 'forward'},   # 太远，应该前进
            {'distance': 1.0, 'x_pos': 200, 'expected': 'turn_right'}, # 偏左，应该右转
            {'distance': 1.0, 'x_pos': 440, 'expected': 'turn_left'},  # 偏右，应该左转
        ]
        
        control_results = []
        
        for case in test_cases:
            self.get_logger().info(f'🧪 测试案例: 距离={case["distance"]}m, 位置=({case["x_pos"]}, 300)')
            
            # 发送测试数据
            self.publish_test_person_data(case['distance'], case['x_pos'])
            
            # 等待控制响应
            time.sleep(1)
            
            # 检查控制指令
            if 'cmd_vel' in self.received_messages:
                cmd_data = self.received_messages['cmd_vel']
                linear_x = cmd_data['linear_x']
                angular_z = cmd_data['angular_z']
                
                # 判断控制行为
                actual_behavior = self.classify_control_behavior(linear_x, angular_z)
                
                result = {
                    'case': case,
                    'linear_x': linear_x,
                    'angular_z': angular_z,
                    'expected': case['expected'],
                    'actual': actual_behavior,
                    'passed': actual_behavior == case['expected']
                }
                
                control_results.append(result)
                
                if result['passed']:
                    self.get_logger().info(f'✅ 控制测试通过: {case["expected"]} -> {actual_behavior}')
                else:
                    self.get_logger().error(f'❌ 控制测试失败: 期望{case["expected"]}, 实际{actual_behavior}')
            
            time.sleep(0.5)
        
        # 计算总体通过率
        passed_count = sum(1 for r in control_results if r['passed'])
        total_count = len(control_results)
        
        self.test_results['following_control'] = {
            'passed': passed_count == total_count,
            'passed_count': passed_count,
            'total_count': total_count,
            'results': control_results
        }
        
        if self.test_results['following_control']['passed']:
            self.get_logger().info(f'✅ 跟随控制测试通过: {passed_count}/{total_count}')
        else:
            self.get_logger().error(f'❌ 跟随控制测试失败: {passed_count}/{total_count}')
    
    def classify_control_behavior(self, linear_x, angular_z):
        """分类控制行为"""
        if abs(linear_x) < 0.01 and abs(angular_z) < 0.01:
            return 'stop'
        elif linear_x > 0.05:
            return 'forward'
        elif linear_x < -0.05:
            return 'backward'
        elif angular_z > 0.1:
            return 'turn_left'
        elif angular_z < -0.1:
            return 'turn_right'
        else:
            return 'unknown'
    
    def test_safety_features(self):
        """测试安全特性"""
        self.get_logger().info('🧪 测试安全特性...')
        
        # 测试超时停止
        self.get_logger().info('🧪 测试超时停止功能...')
        
        # 停止发送人体数据，等待超时
        time.sleep(3)
        
        # 检查是否停止运动
        if 'cmd_vel' in self.received_messages:
            cmd_data = self.received_messages['cmd_vel']
            is_stopped = abs(cmd_data['linear_x']) < 0.01 and abs(cmd_data['angular_z']) < 0.01
            
            self.test_results['safety_timeout'] = {
                'passed': is_stopped,
                'is_stopped': is_stopped
            }
            
            if is_stopped:
                self.get_logger().info('✅ 超时停止测试通过')
            else:
                self.get_logger().error('❌ 超时停止测试失败')
        else:
            self.test_results['safety_timeout'] = {'passed': False, 'error': 'No cmd_vel received'}
    
    def print_test_results(self):
        """打印测试结果"""
        self.get_logger().info('📊 测试结果总结:')
        self.get_logger().info('=' * 50)
        
        total_tests = len(self.test_results)
        passed_tests = sum(1 for result in self.test_results.values() if result.get('passed', False))
        
        for test_name, result in self.test_results.items():
            status = '✅ PASSED' if result.get('passed', False) else '❌ FAILED'
            self.get_logger().info(f'{test_name}: {status}')
        
        self.get_logger().info('=' * 50)
        self.get_logger().info(f'总体结果: {passed_tests}/{total_tests} 测试通过')
        
        if passed_tests == total_tests:
            self.get_logger().info('🎉 所有测试通过! 系统运行正常.')
        else:
            self.get_logger().warn(f'⚠️ 有 {total_tests - passed_tests} 个测试失败.')


def main():
    """主函数"""
    print("🚀 启动人体跟随系统测试")
    
    rclpy.init()
    
    try:
        tester = PersonFollowingSystemTester()
        
        # 在单独线程中运行测试
        test_thread = threading.Thread(target=tester.run_tests)
        test_thread.daemon = True
        test_thread.start()
        
        # 运行ROS2节点
        rclpy.spin(tester)
        
    except KeyboardInterrupt:
        print("🛑 测试被用户中断")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()