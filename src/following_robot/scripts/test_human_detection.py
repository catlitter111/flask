#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
人体检测功能测试脚本
===================
用于测试RKNN人体检测功能是否正确集成到ROS2项目中

使用方法:
python3 test_human_detection.py
"""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'following_robot'))

def test_import():
    """测试模块导入"""
    print("🔍 测试模块导入...")
    
    try:
        from following_robot.rknn_colour_detect import detect_picture_with_confidence, Determine_the_position_of_the_entire_body
        print("✅ 人体检测模块导入成功")
        return True
    except ImportError as e:
        print(f"❌ 人体检测模块导入失败: {e}")
        return False

def test_model_path():
    """测试模型文件路径"""
    print("🔍 测试模型文件路径...")
    
    data_path = os.path.join(os.path.dirname(__file__), '..', 'data', 'best3.rknn')
    if os.path.exists(data_path):
        file_size = os.path.getsize(data_path) / (1024 * 1024)  # MB
        print(f"✅ 模型文件存在: {data_path} ({file_size:.1f}MB)")
        return True
    else:
        print(f"❌ 模型文件不存在: {data_path}")
        return False

def test_dependencies():
    """测试依赖库"""
    print("🔍 测试依赖库...")
    
    dependencies = [
        ('cv2', 'OpenCV'),
        ('numpy', 'NumPy'),
        ('pathlib', 'PathLib'),
        ('rknnlite.api', 'RKNN Lite'),
    ]
    
    all_good = True
    for module, name in dependencies:
        try:
            __import__(module)
            print(f"✅ {name} 可用")
        except ImportError:
            print(f"❌ {name} 不可用")
            all_good = False
    
    return all_good

def main():
    """主测试函数"""
    print("🚀 开始人体检测功能集成测试")
    print("=" * 50)
    
    tests = [
        ("模块导入", test_import),
        ("模型文件", test_model_path),
        ("依赖库", test_dependencies),
    ]
    
    results = []
    for test_name, test_func in tests:
        print(f"\n📋 {test_name}测试:")
        result = test_func()
        results.append((test_name, result))
        print(f"{'✅ 通过' if result else '❌ 失败'}")
    
    print("\n" + "=" * 50)
    print("📊 测试结果汇总:")
    
    passed = sum(1 for _, result in results if result)
    total = len(results)
    
    for test_name, result in results:
        status = "✅ 通过" if result else "❌ 失败"
        print(f"  {test_name}: {status}")
    
    print(f"\n🎯 总体结果: {passed}/{total} 测试通过")
    
    if passed == total:
        print("🎉 所有测试通过！人体检测功能已成功集成到ROS2项目中")
        print("\n📝 使用说明:")
        print("1. 编译ROS2项目: colcon build --packages-select following_robot")
        print("2. 启动节点: ros2 run following_robot stereo_vision_node")
        print("3. 按键控制:")
        print("   - 'q': 退出程序")
        print("   - 'h': 切换人体检测开关")
        print("   - 's': 测试立体视觉处理")
    else:
        print("⚠️ 部分测试失败，请检查相关问题")
        
    return passed == total

if __name__ == '__main__':
    success = main()
    sys.exit(0 if success else 1) 