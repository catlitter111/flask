#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
小车控制测试脚本
===============
用于测试小车控制节点的各种功能
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
import time
import sys
import threading

# ROS消息类型
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool
from turn_on_dlrobot_robot.msg import Position


class RobotControlTest(Node):
    """小车控制测试类"""
    
    def __init__(self):
        super().__init__('robot_control_test')
        
        qos = QoSProfile(depth=10)
        
        # 发布者
        self.manual_cmd_pub = self.create_publisher(
            Twist, '/robot_control/manual_cmd', qos)
        self.mode_pub = self.create_publisher(
            String, '/robot_control/set_mode', qos)
        self.target_person_pub = self.create_publisher(
            String, '/robot_control/target_person', qos)
        self.person_position_pub = self.create_publisher(
            Position, '/robot_control/person_position', qos)
        self.emergency_stop_pub = self.create_publisher(
            Bool, '/robot_control/emergency_stop', qos)
        
        # 订阅者
        self.status_sub = self.create_subscription(
            String, '/robot_control/status', self.status_callback, qos)
        self.mode_sub = self.create_subscription(
            String, '/robot_control/mode', self.mode_callback, qos)
        
        self.get_logger().info("🧪 小车控制测试节点启动")

    def status_callback(self, msg):
        """状态回调"""
        self.get_logger().info(f"📊 状态更新: {msg.data}")

    def mode_callback(self, msg):
        """模式回调"""
        self.get_logger().info(f"🎮 模式变更: {msg.data}")

    def set_mode(self, mode):
        """设置控制模式"""
        msg = String()
        msg.data = mode
        self.mode_pub.publish(msg)
        self.get_logger().info(f"设置模式: {mode}")

    def set_target_person(self, person_name):
        """设置目标人物"""
        msg = String()
        msg.data = person_name
        self.target_person_pub.publish(msg)
        self.get_logger().info(f"设置目标人物: {person_name}")

    def send_manual_command(self, linear_x, angular_z):
        """发送手动控制命令"""
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.manual_cmd_pub.publish(msg)
        self.get_logger().info(f"手动控制: linear={linear_x:.2f}, angular={angular_z:.2f}")

    def simulate_person_position(self, distance, angle_x, angle_y=0.0):
        """模拟人体位置"""
        msg = Position()
        msg.distance = distance
        msg.angle_x = angle_x
        msg.angle_y = angle_y
        self.person_position_pub.publish(msg)
        self.get_logger().info(f"模拟人体位置: 距离={distance:.2f}m, 角度={angle_x:.2f}rad")

    def emergency_stop(self, stop=True):
        """紧急停止"""
        msg = Bool()
        msg.data = stop
        self.emergency_stop_pub.publish(msg)
        self.get_logger().info(f"紧急停止: {'激活' if stop else '解除'}")


def test_manual_control(test_node):
    """测试手动控制"""
    print("\n🎮 测试手动控制模式")
    test_node.set_mode("manual")
    time.sleep(1)
    
    # 前进
    print("前进...")
    test_node.send_manual_command(0.2, 0.0)
    time.sleep(2)
    
    # 左转
    print("左转...")
    test_node.send_manual_command(0.0, 0.5)
    time.sleep(2)
    
    # 右转
    print("右转...")
    test_node.send_manual_command(0.0, -0.5)
    time.sleep(2)
    
    # 停止
    print("停止...")
    test_node.send_manual_command(0.0, 0.0)
    time.sleep(1)


def test_following_mode(test_node):
    """测试跟随模式"""
    print("\n🎯 测试跟随模式")
    test_node.set_target_person("test_person")
    test_node.set_mode("following")
    time.sleep(1)
    
    # 模拟人在远处
    print("模拟人在远处...")
    test_node.simulate_person_position(4.0, 0.0)
    time.sleep(3)
    
    # 模拟人在左侧
    print("模拟人在左侧...")
    test_node.simulate_person_position(2.0, 0.5)
    time.sleep(3)
    
    # 模拟人在右侧
    print("模拟人在右侧...")
    test_node.simulate_person_position(2.0, -0.5)
    time.sleep(3)
    
    # 模拟人太近
    print("模拟人太近...")
    test_node.simulate_person_position(0.5, 0.0)
    time.sleep(3)


def test_emergency_stop(test_node):
    """测试紧急停止"""
    print("\n🚨 测试紧急停止")
    test_node.set_mode("manual")
    time.sleep(1)
    
    # 开始移动
    print("开始移动...")
    test_node.send_manual_command(0.3, 0.0)
    time.sleep(1)
    
    # 紧急停止
    print("紧急停止!")
    test_node.emergency_stop(True)
    time.sleep(2)
    
    # 尝试移动（应该无效）
    print("尝试移动（应该无效）...")
    test_node.send_manual_command(0.3, 0.0)
    time.sleep(2)
    
    # 解除紧急停止
    print("解除紧急停止...")
    test_node.emergency_stop(False)
    time.sleep(1)


def interactive_mode(test_node):
    """交互模式"""
    print("\n🎮 进入交互模式")
    print("命令:")
    print("  m - 手动控制模式")
    print("  f - 跟随模式")
    print("  s - 停止模式")
    print("  e - 紧急停止")
    print("  r - 解除紧急停止")
    print("  w/a/s/d - 手动移动（前/左/后/右）")
    print("  p - 模拟人体位置")
    print("  q - 退出")
    
    while True:
        try:
            cmd = input("\n输入命令: ").strip().lower()
            
            if cmd == 'q':
                break
            elif cmd == 'm':
                test_node.set_mode("manual")
            elif cmd == 'f':
                test_node.set_mode("following")
            elif cmd == 's':
                test_node.set_mode("stop")
            elif cmd == 'e':
                test_node.emergency_stop(True)
            elif cmd == 'r':
                test_node.emergency_stop(False)
            elif cmd == 'w':
                test_node.send_manual_command(0.2, 0.0)
            elif cmd == 's':
                test_node.send_manual_command(-0.2, 0.0)
            elif cmd == 'a':
                test_node.send_manual_command(0.0, 0.5)
            elif cmd == 'd':
                test_node.send_manual_command(0.0, -0.5)
            elif cmd == 'p':
                try:
                    distance = float(input("输入距离(m): "))
                    angle = float(input("输入角度(rad): "))
                    test_node.simulate_person_position(distance, angle)
                except ValueError:
                    print("输入格式错误")
            else:
                print("未知命令")
                
        except KeyboardInterrupt:
            break
        except Exception as e:
            print(f"错误: {e}")


def main():
    """主函数"""
    if len(sys.argv) < 2:
        print("用法:")
        print("  python3 robot_control_test.py manual    # 测试手动控制")
        print("  python3 robot_control_test.py following # 测试跟随模式")
        print("  python3 robot_control_test.py emergency # 测试紧急停止")
        print("  python3 robot_control_test.py interactive # 交互模式")
        print("  python3 robot_control_test.py all       # 运行所有测试")
        return
    
    rclpy.init()
    
    try:
        test_node = RobotControlTest()
        
        # 启动ROS节点在后台
        import threading
        ros_thread = threading.Thread(target=lambda: rclpy.spin(test_node))
        ros_thread.daemon = True
        ros_thread.start()
        
        time.sleep(1)  # 等待节点启动
        
        test_mode = sys.argv[1].lower()
        
        if test_mode == "manual":
            test_manual_control(test_node)
        elif test_mode == "following":
            test_following_mode(test_node)
        elif test_mode == "emergency":
            test_emergency_stop(test_node)
        elif test_mode == "interactive":
            interactive_mode(test_node)
        elif test_mode == "all":
            test_manual_control(test_node)
            time.sleep(2)
            test_following_mode(test_node)
            time.sleep(2)
            test_emergency_stop(test_node)
        else:
            print(f"未知测试模式: {test_mode}")
        
        # 确保停止
        test_node.set_mode("stop")
        print("\n✅ 测试完成")
        
    except KeyboardInterrupt:
        print("\n测试被中断")
    finally:
        test_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 