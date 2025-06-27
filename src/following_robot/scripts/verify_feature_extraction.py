#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
特征提取功能验证脚本
==================
验证特征提取节点的完整功能
"""

import os
import sys
import time
import subprocess
from pathlib import Path


def check_dependencies():
    """检查依赖项"""
    print("🔍 检查依赖项...")
    
    dependencies = [
        'rclpy',
        'cv2',
        'numpy',
        'openpyxl',
        'PIL',
        'cv_bridge'
    ]
    
    missing_deps = []
    for dep in dependencies:
        try:
            if dep == 'cv2':
                import cv2
            elif dep == 'PIL':
                from PIL import Image
            elif dep == 'cv_bridge':
                from cv_bridge import CvBridge
            else:
                __import__(dep)
            print(f"  ✅ {dep}")
        except ImportError:
            print(f"  ❌ {dep}")
            missing_deps.append(dep)
    
    if missing_deps:
        print(f"\n❌ 缺少依赖项: {', '.join(missing_deps)}")
        return False
    else:
        print("✅ 所有依赖项都已安装")
        return True


def check_ros2_setup():
    """检查ROS2环境设置"""
    print("\n🔍 检查ROS2环境...")
    
    # 检查ROS2环境变量
    if 'ROS_DISTRO' not in os.environ:
        print("❌ ROS_DISTRO环境变量未设置")
        return False
    
    print(f"  ✅ ROS_DISTRO: {os.environ['ROS_DISTRO']}")
    
    # 检查工作空间
    workspace_path = Path.cwd()
    if not (workspace_path / 'src').exists():
        print("❌ 当前不在ROS2工作空间中")
        return False
    
    print(f"  ✅ 工作空间: {workspace_path}")
    
    return True


def check_custom_messages():
    """检查自定义消息"""
    print("\n🔍 检查自定义消息...")
    
    try:
        # 检查服务定义文件
        srv_file = Path('src/custom_msgs/srv/FeatureExtraction.srv')
        if not srv_file.exists():
            print("❌ FeatureExtraction.srv文件不存在")
            return False
        
        print("  ✅ FeatureExtraction.srv文件存在")
        
        # 尝试导入服务
        try:
            from custom_msgs.srv import FeatureExtraction
            print("  ✅ FeatureExtraction服务可以导入")
        except ImportError as e:
            print(f"  ❌ 无法导入FeatureExtraction服务: {e}")
            return False
        
        return True
        
    except Exception as e:
        print(f"❌ 检查自定义消息时出错: {e}")
        return False


def check_feature_extraction_node():
    """检查特征提取节点"""
    print("\n🔍 检查特征提取节点...")
    
    try:
        # 检查节点文件
        node_file = Path('src/following_robot/following_robot/feature_extraction_node.py')
        if not node_file.exists():
            print("❌ feature_extraction_node.py文件不存在")
            return False
        
        print("  ✅ feature_extraction_node.py文件存在")
        
        # 检查节点是否可以导入
        sys.path.insert(0, str(Path('src/following_robot').absolute()))
        try:
            from following_robot.feature_extraction_node import FeatureExtractionNode
            print("  ✅ FeatureExtractionNode类可以导入")
        except ImportError as e:
            print(f"  ❌ 无法导入FeatureExtractionNode: {e}")
            return False
        
        return True
        
    except Exception as e:
        print(f"❌ 检查特征提取节点时出错: {e}")
        return False


def check_detection_modules():
    """检查检测模块"""
    print("\n🔍 检查检测模块...")
    
    try:
        sys.path.insert(0, str(Path('src/following_robot').absolute()))
        
        # 检查服装检测模块
        try:
            from following_robot.rknn_colour_detect import Obtain_the_target_color
            print("  ✅ 服装检测模块可用")
            clothing_available = True
        except ImportError as e:
            print(f"  ⚠️ 服装检测模块不可用: {e}")
            clothing_available = False
        
        # 检查姿态检测模块
        try:
            from following_robot.yolov8_pose_detector import detect_human_pose, draw_human_pose
            print("  ✅ 姿态检测模块可用")
            pose_available = True
        except ImportError as e:
            print(f"  ⚠️ 姿态检测模块不可用: {e}")
            pose_available = False
        
        if clothing_available or pose_available:
            print("  ✅ 至少一个检测模块可用")
            return True
        else:
            print("  ❌ 没有可用的检测模块")
            return False
        
    except Exception as e:
        print(f"❌ 检查检测模块时出错: {e}")
        return False


def check_model_files():
    """检查模型文件"""
    print("\n🔍 检查模型文件...")
    
    data_dir = Path('src/following_robot/data')
    if not data_dir.exists():
        print("❌ data目录不存在")
        return False
    
    print(f"  ✅ data目录存在: {data_dir}")
    
    # 检查模型文件
    model_files = {
        'best3.rknn': '服装检测模型',
        'yolov8_pose.rknn': '姿态检测模型'
    }
    
    available_models = 0
    for model_file, description in model_files.items():
        model_path = data_dir / model_file
        if model_path.exists():
            size_mb = model_path.stat().st_size / (1024 * 1024)
            print(f"  ✅ {description}: {model_file} ({size_mb:.1f}MB)")
            available_models += 1
        else:
            print(f"  ❌ {description}: {model_file} 不存在")
    
    if available_models > 0:
        print(f"  ✅ 找到 {available_models}/{len(model_files)} 个模型文件")
        return True
    else:
        print("  ❌ 没有找到模型文件")
        return False


def check_build_status():
    """检查编译状态"""
    print("\n🔍 检查编译状态...")
    
    try:
        # 检查install目录
        install_dir = Path('install')
        if not install_dir.exists():
            print("❌ install目录不存在，请先编译项目")
            return False
        
        # 检查包是否已安装
        packages = ['custom_msgs', 'following_robot']
        for package in packages:
            package_dir = install_dir / package
            if package_dir.exists():
                print(f"  ✅ {package} 已编译安装")
            else:
                print(f"  ❌ {package} 未编译安装")
                return False
        
        return True
        
    except Exception as e:
        print(f"❌ 检查编译状态时出错: {e}")
        return False


def run_verification():
    """运行完整验证"""
    print("🚀 开始特征提取功能验证")
    print("=" * 60)
    
    checks = [
        ("依赖项检查", check_dependencies),
        ("ROS2环境检查", check_ros2_setup),
        ("自定义消息检查", check_custom_messages),
        ("特征提取节点检查", check_feature_extraction_node),
        ("检测模块检查", check_detection_modules),
        ("模型文件检查", check_model_files),
        ("编译状态检查", check_build_status),
    ]
    
    passed = 0
    total = len(checks)
    
    for check_name, check_func in checks:
        print(f"\n📋 {check_name}")
        print("-" * 40)
        
        try:
            if check_func():
                passed += 1
                print(f"✅ {check_name} 通过")
            else:
                print(f"❌ {check_name} 失败")
        except Exception as e:
            print(f"💥 {check_name} 出错: {e}")
    
    print("\n" + "=" * 60)
    print(f"验证结果: {passed}/{total} 项检查通过")
    
    if passed == total:
        print("🎉 所有检查都通过！特征提取功能已准备就绪")
        print("\n📋 使用说明:")
        print("1. 启动特征提取节点:")
        print("   ros2 run following_robot feature_extraction_node")
        print("   或者使用启动文件:")
        print("   ros2 launch following_robot feature_extraction_launch.py")
        print("\n2. 测试特征提取功能:")
        print("   python3 src/following_robot/scripts/test_feature_extraction.py --camera")
        print("   或者:")
        print("   python3 src/following_robot/scripts/test_feature_extraction.py <image_path>")
        print("\n3. 服务接口:")
        print("   服务名称: /features/extract_features")
        print("   服务类型: custom_msgs/srv/FeatureExtraction")
        return True
    else:
        print(f"⚠️ 有 {total - passed} 项检查未通过，请解决相关问题")
        return False


def main():
    """主函数"""
    try:
        # 确保在正确的目录
        if not Path('src').exists():
            print("❌ 请在ROS2工作空间根目录运行此脚本")
            return False
        
        return run_verification()
        
    except KeyboardInterrupt:
        print("\n⏹️ 验证被用户中断")
        return False
    except Exception as e:
        print(f"💥 验证过程出错: {e}")
        return False


if __name__ == '__main__':
    success = main()
    sys.exit(0 if success else 1) 