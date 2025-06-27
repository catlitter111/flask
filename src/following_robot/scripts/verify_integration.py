#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
功能集成验证脚本
===============
验证人体检测和姿态检测功能是否正确集成到ROS2系统中

作者: AI Assistant
"""

import sys
import os
import subprocess

def check_file_exists(file_path, description):
    """检查文件是否存在"""
    if os.path.exists(file_path):
        size = os.path.getsize(file_path)
        print(f"✅ {description}: {file_path} ({size/1024/1024:.1f}MB)")
        return True
    else:
        print(f"❌ {description}: {file_path} - 文件不存在")
        return False

def check_python_import(module_name, description):
    """检查Python模块是否可以导入"""
    try:
        __import__(module_name)
        print(f"✅ {description}: {module_name}")
        return True
    except ImportError as e:
        print(f"❌ {description}: {module_name} - {e}")
        return False

def check_ros2_package():
    """检查ROS2包是否正确编译"""
    try:
        result = subprocess.run(['ros2', 'pkg', 'list'], 
                              capture_output=True, text=True, timeout=10)
        if 'following_robot' in result.stdout:
            print("✅ ROS2包: following_robot 已正确安装")
            return True
        else:
            print("❌ ROS2包: following_robot 未找到")
            return False
    except Exception as e:
        print(f"❌ ROS2包检查失败: {e}")
        return False

def check_ros2_executable():
    """检查ROS2可执行文件"""
    try:
        result = subprocess.run(['ros2', 'pkg', 'executables', 'following_robot'], 
                              capture_output=True, text=True, timeout=10)
        if 'stereo_vision_node' in result.stdout:
            print("✅ ROS2可执行文件: stereo_vision_node 可用")
            return True
        else:
            print("❌ ROS2可执行文件: stereo_vision_node 不可用")
            return False
    except Exception as e:
        print(f"❌ ROS2可执行文件检查失败: {e}")
        return False

def main():
    """主函数"""
    print("🚀 开始验证人体检测和姿态检测功能集成...")
    print("=" * 60)
    
    success_count = 0
    total_checks = 0
    
    # 检查模型文件
    print("\n📁 检查模型文件...")
    total_checks += 1
    if check_file_exists("src/following_robot/data/best3.rknn", "人体检测模型"):
        success_count += 1
    
    total_checks += 1
    if check_file_exists("src/following_robot/data/yolov8_pose.rknn", "姿态检测模型"):
        success_count += 1
    
    # 检查源码文件
    print("\n📄 检查源码文件...")
    source_files = [
        ("src/following_robot/following_robot/stereo_vision_node.py", "主节点文件"),
        ("src/following_robot/following_robot/rknn_colour_detect.py", "人体检测模块"),
        ("src/following_robot/following_robot/yolov8_pose_detector.py", "姿态检测模块"),
        ("src/following_robot/setup.py", "安装配置文件"),
        ("src/following_robot/package.xml", "ROS2包配置文件"),
    ]
    
    for file_path, description in source_files:
        total_checks += 1
        if check_file_exists(file_path, description):
            success_count += 1
    
    # 检查测试脚本
    print("\n🧪 检查测试脚本...")
    test_scripts = [
        ("src/following_robot/scripts/test_human_detection.py", "人体检测测试"),
        ("src/following_robot/scripts/test_clothing_detection.py", "衣服检测测试"),
        ("src/following_robot/scripts/test_pose_detection.py", "姿态检测测试"),
    ]
    
    for file_path, description in test_scripts:
        total_checks += 1
        if check_file_exists(file_path, description):
            success_count += 1
    
    # 检查Python依赖
    print("\n🐍 检查Python依赖...")
    dependencies = [
        ("cv2", "OpenCV"),
        ("numpy", "NumPy"),
        ("rclpy", "ROS2 Python客户端"),
        ("rknnlite.api", "RKNN Lite"),
    ]
    
    for module, description in dependencies:
        total_checks += 1
        if check_python_import(module, description):
            success_count += 1
    
    # 检查ROS2集成
    print("\n🤖 检查ROS2集成...")
    total_checks += 1
    if check_ros2_package():
        success_count += 1
    
    total_checks += 1
    if check_ros2_executable():
        success_count += 1
    
    # 检查安装后的文件
    print("\n📦 检查安装文件...")
    install_files = [
        ("install/following_robot/share/following_robot/data/best3.rknn", "安装的人体检测模型"),
        ("install/following_robot/share/following_robot/data/yolov8_pose.rknn", "安装的姿态检测模型"),
    ]
    
    for file_path, description in install_files:
        total_checks += 1
        if check_file_exists(file_path, description):
            success_count += 1
    
    # 显示结果
    print("\n" + "=" * 60)
    print(f"📊 验证结果: {success_count}/{total_checks} 项检查通过")
    
    if success_count == total_checks:
        print("🎉 所有功能集成验证通过！")
        print("\n💡 下一步操作:")
        print("   1. 启动节点: ros2 run following_robot stereo_vision_node")
        print("   2. 按键控制:")
        print("      - 'h': 切换人体检测")
        print("      - 'p': 切换姿态检测")
        print("      - 's': 测试立体视觉")
        print("      - 'q': 退出")
        print("   3. 距离测量: ros2 service call /stereo/get_distance custom_msgs/srv/GetDistance \"{x: 320, y: 240}\"")
        return True
    else:
        print("❌ 部分功能集成存在问题")
        failed_count = total_checks - success_count
        print(f"   {failed_count} 项检查失败，请检查上述错误信息")
        return False

if __name__ == '__main__':
    success = main()
    sys.exit(0 if success else 1) 