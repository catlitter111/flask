#!/usr/bin/env python3
"""
相机测试脚本
用于诊断相机访问问题
"""

import cv2
import numpy as np
import sys

def test_camera(camera_id=0):
    """测试指定ID的相机"""
    print(f"🔍 测试相机 ID: {camera_id}")
    
    # 尝试打开相机
    cap = cv2.VideoCapture(camera_id, cv2.CAP_V4L2)
    
    if not cap.isOpened():
        print(f"❌ 无法打开相机 {camera_id}")
        return False
    
    print(f"✅ 相机 {camera_id} 打开成功")
    
    # 获取相机属性
    width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
    fps = cap.get(cv2.CAP_PROP_FPS)
    
    print(f"📷 相机属性: {width}x{height}, FPS: {fps}")
    
    # 尝试读取几帧
    success_count = 0
    for i in range(5):
        ret, frame = cap.read()
        if ret:
            success_count += 1
            print(f"✅ 第 {i+1} 帧读取成功, 形状: {frame.shape}")
        else:
            print(f"❌ 第 {i+1} 帧读取失败")
    
    cap.release()
    
    if success_count > 0:
        print(f"🎉 相机 {camera_id} 测试成功! ({success_count}/5 帧成功)")
        return True
    else:
        print(f"❌ 相机 {camera_id} 测试失败!")
        return False

def test_all_cameras():
    """测试所有可能的相机"""
    print("🔍 开始测试所有相机...")
    
    working_cameras = []
    
    for camera_id in range(10):  # 测试0-9号相机
        try:
            if test_camera(camera_id):
                working_cameras.append(camera_id)
        except Exception as e:
            print(f"❌ 测试相机 {camera_id} 时出错: {e}")
        print("-" * 50)
    
    if working_cameras:
        print(f"🎉 发现可用相机: {working_cameras}")
        print(f"建议使用相机ID: {working_cameras[0]}")
    else:
        print("❌ 未发现任何可用相机")
    
    return working_cameras

if __name__ == "__main__":
    print("🎬 相机诊断工具")
    print("=" * 50)
    
    if len(sys.argv) > 1:
        # 测试指定相机
        camera_id = int(sys.argv[1])
        test_camera(camera_id)
    else:
        # 测试所有相机
        test_all_cameras() 