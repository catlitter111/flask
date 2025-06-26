#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
简单的cv2.imshow测试脚本
=======================
用于测试OpenCV显示功能

作者: AI Assistant
"""

import cv2
import numpy as np
import time

def test_display():
    """测试cv2.imshow显示功能"""
    print("测试cv2.imshow显示功能...")
    print("按 'q' 键退出测试")
    
    frame_counter = 0
    start_time = time.time()
    fps_counter = 0
    current_fps = 0.0
    
    try:
        while True:
            # 创建一个测试图像
            test_image = np.zeros((480, 640, 3), dtype=np.uint8)
            test_image[:] = (50, 100, 150)  # 填充颜色
            
            frame_counter += 1
            fps_counter += 1
            
            # 计算FPS
            current_time = time.time()
            elapsed_time = current_time - start_time
            if elapsed_time >= 1.0:
                current_fps = fps_counter / elapsed_time
                fps_counter = 0
                start_time = current_time
            
            # 添加文本信息
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 0.7
            color = (0, 255, 0)
            thickness = 2
            
            # 显示信息
            cv2.putText(test_image, f"Frame: {frame_counter}", (10, 30), 
                       font, font_scale, color, thickness)
            cv2.putText(test_image, f"FPS: {current_fps:.1f}", (10, 60), 
                       font, font_scale, color, thickness)
            cv2.putText(test_image, "Test Display - Press 'q' to quit", (10, 90), 
                       font, font_scale, (255, 255, 255), thickness)
            
            # 显示图像
            cv2.imshow('Test Display', test_image)
            
            # 处理键盘事件
            key = cv2.waitKey(30) & 0xFF
            if key == ord('q'):
                print("用户按下'q'键，退出测试")
                break
            
    except Exception as e:
        print(f"测试出错: {e}")
    finally:
        cv2.destroyAllWindows()
        print("测试完成")

if __name__ == '__main__':
    test_display() 