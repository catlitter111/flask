#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
姿态检测测试脚本
===============
测试YOLOv8姿态检测功能

作者: AI Assistant
"""

import sys
import os
import cv2
import numpy as np

# 添加模块路径
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'following_robot'))

def test_pose_detection():
    """测试姿态检测功能"""
    print("🧪 开始测试姿态检测功能...")
    
    try:
        # 导入姿态检测模块
        from yolov8_pose_detector import YOLOv8PoseDetector, detect_human_pose, draw_human_pose
        print("✅ 成功导入姿态检测模块")
        
        # 创建检测器
        detector = YOLOv8PoseDetector()
        if detector.model_loaded:
            print("✅ 姿态检测模型加载成功")
        else:
            print("❌ 姿态检测模型加载失败")
            return False
        
        # 创建测试图像（模拟一个简单的人体图像）
        test_image = np.zeros((480, 640, 3), dtype=np.uint8)
        
        # 绘制一个简单的人体轮廓用于测试
        # 头部
        cv2.circle(test_image, (320, 100), 30, (255, 255, 255), -1)
        # 身体
        cv2.rectangle(test_image, (280, 130), (360, 300), (128, 128, 128), -1)
        # 手臂
        cv2.rectangle(test_image, (240, 150), (280, 250), (128, 128, 128), -1)
        cv2.rectangle(test_image, (360, 150), (400, 250), (128, 128, 128), -1)
        # 腿部
        cv2.rectangle(test_image, (290, 300), (330, 450), (128, 128, 128), -1)
        cv2.rectangle(test_image, (330, 300), (370, 450), (128, 128, 128), -1)
        
        print("📸 创建了测试图像")
        
        # 进行检测
        print("🔍 开始姿态检测...")
        pose_results = detector.detect_pose(test_image)
        
        if pose_results:
            print(f"✅ 检测成功！发现 {len(pose_results)} 个人体姿态")
            
            for i, pose in enumerate(pose_results):
                print(f"\n🦴 姿态 {i + 1}:")
                print(f"   边界框: {pose.bbox}")
                print(f"   置信度: {pose.score:.3f}")
                print(f"   关键点数量: {len(pose.keypoints)}")
                print(f"   类别ID: {pose.class_id}")
                
                # 统计有效关键点
                valid_keypoints = 0
                for kp in pose.keypoints:
                    if kp[0] > 0 and kp[1] > 0 and kp[2] > 0.5:
                        valid_keypoints += 1
                print(f"   有效关键点: {valid_keypoints}/17")
            
            # 绘制结果
            result_image = detector.draw_pose(test_image, pose_results)
            
            # 保存结果
            output_path = "/tmp/pose_detection_test.jpg"
            cv2.imwrite(output_path, result_image)
            print(f"   可视化结果已保存到: {output_path}")
            
        else:
            print("⚠️ 未检测到任何人体姿态")
            
        # 测试便捷函数
        print("\n🔧 测试便捷函数...")
        pose_results_2 = detect_human_pose(test_image)
        print(f"   便捷函数检测到 {len(pose_results_2)} 个姿态")
        
        if pose_results_2:
            result_image_2 = draw_human_pose(test_image, pose_results_2)
            output_path_2 = "/tmp/pose_detection_convenience_test.jpg"
            cv2.imwrite(output_path_2, result_image_2)
            print(f"   便捷函数结果已保存到: {output_path_2}")
            
    except ImportError as e:
        print(f"❌ 模块导入失败: {e}")
        return False
    except Exception as e:
        print(f"❌ 测试失败: {e}")
        import traceback
        traceback.print_exc()
        return False
    
    print("\n✅ 姿态检测功能测试完成!")
    return True

def test_model_loading():
    """测试模型加载"""
    print("\n🧪 测试模型加载...")
    
    try:
        from yolov8_pose_detector import YOLOv8PoseDetector
        
        # 测试自动路径查找
        detector = YOLOv8PoseDetector()
        if detector.model_loaded:
            print("✅ 自动路径查找成功")
        else:
            print("❌ 自动路径查找失败")
        
        # 测试指定路径
        model_paths = [
            '/userdata/try_again/SelfFollowingROS2/src/following_robot/data/yolov8_pose.rknn',
            '../data/yolov8_pose.rknn',
        ]
        
        for path in model_paths:
            if os.path.exists(path):
                print(f"✅ 找到模型文件: {path}")
                detector_manual = YOLOv8PoseDetector(path)
                if detector_manual.model_loaded:
                    print(f"✅ 手动加载成功: {path}")
                else:
                    print(f"❌ 手动加载失败: {path}")
                break
        else:
            print("⚠️ 未找到可用的模型文件")
            
    except Exception as e:
        print(f"❌ 模型加载测试失败: {e}")
        import traceback
        traceback.print_exc()
        return False
    
    return True

if __name__ == '__main__':
    print("🚀 启动姿态检测测试...")
    
    # 测试模型加载
    success1 = test_model_loading()
    
    # 测试姿态检测功能
    success2 = test_pose_detection()
    
    if success1 and success2:
        print("\n🎉 所有测试通过！")
        print("💡 现在可以运行ROS2节点来查看实际的姿态检测效果")
        print("   命令: ros2 run following_robot stereo_vision_node")
        print("   按键: 'p' 切换姿态检测, 'h' 切换人体检测, 'q' 退出")
    else:
        print("\n❌ 部分测试失败")
        sys.exit(1) 