#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
衣服检测测试脚本
===============
测试衣服和裤子分别检测和框定功能

作者: AI Assistant
"""

import sys
import os
import cv2
import numpy as np

# 添加模块路径
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'following_robot'))

def test_clothing_detection():
    """测试衣服检测功能"""
    print("🧪 开始测试衣服检测功能...")
    
    try:
        # 导入检测模块
        from rknn_colour_detect import detect_picture_with_confidence, Determine_the_position_of_the_entire_body
        print("✅ 成功导入检测模块")
        
        # 创建测试图像（模拟一个简单的人体图像）
        test_image = np.zeros((480, 640, 3), dtype=np.uint8)
        
        # 绘制一个简单的人体轮廓用于测试
        # 头部
        cv2.circle(test_image, (320, 100), 30, (255, 255, 255), -1)
        # 上身（衣服区域）
        cv2.rectangle(test_image, (280, 130), (360, 250), (0, 255, 0), -1)
        # 下身（裤子区域）
        cv2.rectangle(test_image, (290, 250), (350, 400), (0, 0, 255), -1)
        
        print("📸 创建了测试图像")
        
        # 进行检测
        print("🔍 开始检测...")
        detection_results = detect_picture_with_confidence(test_image)
        
        if detection_results:
            print(f"✅ 检测成功！发现 {len(detection_results)} 个人")
            
            for person_idx, result in enumerate(detection_results):
                print(f"\n👤 人员 {person_idx + 1}:")
                print(f"   检测结果长度: {len(result)}")
                
                if len(result) >= 2:
                    upper_info = result[0] if result[0] else None
                    lower_info = result[1] if result[1] else None
                    
                    print(f"   上装信息: {upper_info}")
                    print(f"   下装信息: {lower_info}")
                    
                    # 测试整体身体框计算
                    try:
                        body_box = Determine_the_position_of_the_entire_body(
                            upper_info, lower_info, test_image
                        )
                        print(f"   整体框: {body_box}")
                        
                        # 创建可视化图像
                        vis_image = test_image.copy()
                        
                        # 绘制上装框（蓝色）
                        if upper_info and len(upper_info) >= 4:
                            x1, y1, x2, y2 = upper_info[:4]
                            cv2.rectangle(vis_image, (int(x1), int(y1)), (int(x2), int(y2)), (255, 0, 0), 2)
                            cv2.putText(vis_image, f"Upper {person_idx+1}", (int(x1), int(y1)-10), 
                                      cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                        
                        # 绘制下装框（红色）
                        if lower_info and len(lower_info) >= 4:
                            x1, y1, x2, y2 = lower_info[:4]
                            cv2.rectangle(vis_image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 0, 255), 2)
                            cv2.putText(vis_image, f"Lower {person_idx+1}", (int(x1), int(y1)-10), 
                                      cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                        
                        # 绘制整体框（绿色）
                        if body_box and len(body_box) >= 4:
                            x1, y1, x2, y2 = body_box[:4]
                            cv2.rectangle(vis_image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                            cv2.putText(vis_image, f"Person {person_idx+1}", (int(x1), int(y1)-10), 
                                      cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                        
                        # 保存可视化结果
                        output_path = f"/tmp/clothing_detection_test_person_{person_idx+1}.jpg"
                        cv2.imwrite(output_path, vis_image)
                        print(f"   可视化结果已保存到: {output_path}")
                        
                    except Exception as e:
                        print(f"   ❌ 身体框计算失败: {e}")
                else:
                    print("   ⚠️ 检测结果格式不正确")
        else:
            print("⚠️ 未检测到任何人体")
            
    except ImportError as e:
        print(f"❌ 模块导入失败: {e}")
        return False
    except Exception as e:
        print(f"❌ 测试失败: {e}")
        import traceback
        traceback.print_exc()
        return False
    
    print("\n✅ 衣服检测功能测试完成!")
    return True

def test_detection_data_structure():
    """测试检测数据结构"""
    print("\n🧪 测试检测数据结构...")
    
    # 模拟检测结果数据
    mock_detection_results = [
        [
            [100, 50, 200, 150],    # 上装框
            [110, 150, 190, 300],   # 下装框
            "red",                   # 上装颜色
            "blue"                   # 下装颜色
        ],
        [
            [300, 60, 400, 160],    # 上装框
            [310, 160, 390, 320],   # 下装框
            "green",                # 上装颜色
            "black"                 # 下装颜色
        ]
    ]
    
    print("📊 处理模拟检测数据...")
    clothing_detections = []
    
    for person_idx, result in enumerate(mock_detection_results):
        if len(result) >= 2:
            upper_info = result[0] if result[0] else None
            lower_info = result[1] if result[1] else None
            
            # 保存衣服和裤子的详细检测信息
            person_clothes = {
                'person_id': person_idx,
                'upper': upper_info,
                'lower': lower_info,
                'upper_color': result[2] if len(result) > 2 else None,
                'lower_color': result[3] if len(result) > 3 else None
            }
            clothing_detections.append(person_clothes)
            
            print(f"👤 人员 {person_idx + 1}:")
            print(f"   上装: {upper_info} ({person_clothes['upper_color']})")
            print(f"   下装: {lower_info} ({person_clothes['lower_color']})")
    
    print(f"\n✅ 处理了 {len(clothing_detections)} 套衣服检测数据")
    return True

if __name__ == '__main__':
    print("🚀 启动衣服检测测试...")
    
    # 测试检测功能
    success1 = test_clothing_detection()
    
    # 测试数据结构
    success2 = test_detection_data_structure()
    
    if success1 and success2:
        print("\n🎉 所有测试通过！")
        print("💡 现在可以运行ROS2节点来查看实际的衣服检测效果")
        print("   命令: ros2 run following_robot stereo_vision_node")
        print("   按键: 'h' 切换检测开关, 'q' 退出")
    else:
        print("\n❌ 部分测试失败")
        sys.exit(1) 