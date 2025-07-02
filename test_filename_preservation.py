#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
文件名保存验证测试脚本
===================

测试用户自定义文件名是否能正确保存到机器人端

使用方法：
    python test_filename_preservation.py

前置条件：
    1. 启动服务器：python 服务端/server.py
    2. 启动ROS2节点：ros2 run following_robot websocket_bridge_node
    3. 运行此测试脚本

作者：AI Assistant
日期：2025
"""

import asyncio
import websockets
import json
import time
import base64
import requests
from pathlib import Path

# 测试配置
SERVER_HOST = '172.20.39.181'
WEBSOCKET_PORT = 1234
HTTP_PORT = 1235
CLIENT_ID = 'filename_test_client'
ROBOT_ID = 'companion_robot_001'

# 测试用例
TEST_CASES = [
    {
        'user_filename': '我的特征照片.jpg',
        'description': '中文文件名测试'
    },
    {
        'user_filename': 'PersonFeatures2025.jpg', 
        'description': '英文文件名测试'
    },
    {
        'user_filename': '张三的识别视频.mp4',
        'description': '中文视频文件名测试'
    },
    {
        'user_filename': 'test_image_特殊字符@#$.jpg',
        'description': '特殊字符文件名测试'
    }
]

def create_test_image():
    """创建测试图片"""
    try:
        import cv2
        import numpy as np
        
        # 创建一个简单的测试图片
        img = np.zeros((200, 300, 3), dtype=np.uint8)
        cv2.putText(img, 'Test Image', (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.rectangle(img, (20, 20), (280, 180), (0, 255, 0), 2)
        
        # 保存为临时文件
        temp_path = Path('temp_test_image.jpg')
        cv2.imwrite(str(temp_path), img)
        
        return temp_path
        
    except ImportError:
        print("警告: OpenCV未安装，使用空白文件代替")
        temp_path = Path('temp_test_image.jpg')
        with open(temp_path, 'wb') as f:
            # 创建一个最小的JPEG文件头
            f.write(b'\xff\xd8\xff\xe0\x00\x10JFIF\x00\x01\x01\x01\x00H\x00H\x00\x00\xff\xdb\x00C\x00')
            f.write(b'\x00' * 100)  # 简单的占位数据
        return temp_path

async def test_websocket_client(test_case):
    """测试WebSocket客户端"""
    client_id = f"{CLIENT_ID}_{int(time.time())}"
    robot_id = ROBOT_ID
    
    print(f"\n🧪 测试用例: {test_case['description']}")
    print(f"📝 用户输入文件名: {test_case['user_filename']}")
    
    try:
        # 连接WebSocket服务器
        ws_url = f"ws://{SERVER_HOST}:{WEBSOCKET_PORT}/ws/companion/{client_id}"
        
        async with websockets.connect(ws_url) as websocket:
            print(f"🔗 WebSocket连接成功: {ws_url}")
            
            # 发送客户端初始化
            init_message = {
                'type': 'client_init',
                'client_id': client_id,
                'robot_id': robot_id,
                'capabilities': {
                    'video_receive': True,
                    'file_upload': True
                },
                'timestamp': int(time.time() * 1000)
            }
            await websocket.send(json.dumps(init_message))
            print("📤 发送客户端初始化消息")
            
            # 等待服务器响应
            response = await asyncio.wait_for(websocket.recv(), timeout=5.0)
            data = json.loads(response)
            print(f"📥 收到服务器响应: {data.get('type', 'unknown')}")
            
            # 创建测试文件
            test_file = create_test_image()
            
            try:
                # 模拟HTTP上传
                print(f"📤 开始上传文件...")
                
                with open(test_file, 'rb') as f:
                    file_data = f.read()
                
                # 构建multipart form data
                files = {
                    'file': (test_case['user_filename'], file_data, 'image/jpeg')
                }
                
                upload_url = f"http://{SERVER_HOST}:{HTTP_PORT}/api/upload/{client_id}"
                response = requests.post(upload_url, files=files, timeout=10)
                
                if response.status_code == 200:
                    upload_result = response.json()
                    print(f"✅ 文件上传成功")
                    print(f"   - 文件ID: {upload_result.get('file_id', 'unknown')}")
                    print(f"   - 服务器记录文件名: {upload_result.get('file_name', 'unknown')}")
                    
                    # 等待机器人保存结果
                    print("⏳ 等待机器人保存结果...")
                    
                    timeout_count = 0
                    while timeout_count < 15:  # 最多等待15秒
                        try:
                            response = await asyncio.wait_for(websocket.recv(), timeout=1.0)
                            data = json.loads(response)
                            message_type = data.get('type', 'unknown')
                            
                            if message_type == 'file_save_result':
                                status = data.get('status', 'unknown')
                                if status == 'success':
                                    original_name = data.get('original_name', 'unknown')
                                    final_name = data.get('final_name', 'unknown')
                                    saved_path = data.get('saved_path', 'unknown')
                                    
                                    print(f"✅ 文件保存成功!")
                                    print(f"   - 用户输入名称: {original_name}")
                                    print(f"   - 机器人保存名称: {final_name}")
                                    print(f"   - 保存路径: {saved_path}")
                                    
                                    # 验证文件名是否正确保存
                                    if final_name == test_case['user_filename']:
                                        print("🎉 文件名保存正确！用户输入的文件名被完整保留")
                                        return True
                                    elif final_name.startswith(original_name.split('.')[0]):
                                        print("⚠️ 文件名部分保留（可能有重复文件处理）")
                                        return True
                                    else:
                                        print("❌ 文件名保存错误！与用户输入不符")
                                        return False
                                else:
                                    print(f"❌ 文件保存失败: {data.get('error', 'unknown')}")
                                    return False
                                    
                        except asyncio.TimeoutError:
                            timeout_count += 1
                            if timeout_count % 5 == 0:
                                print(f"⏱️ 等待文件保存结果... ({timeout_count}s)")
                    
                    print("⏰ 等待超时，未收到文件保存结果")
                    return False
                    
                else:
                    print(f"❌ 文件上传失败: {response.status_code} - {response.text}")
                    return False
                    
            finally:
                # 清理测试文件
                if test_file.exists():
                    test_file.unlink()
                    
    except Exception as e:
        print(f"❌ 测试失败: {e}")
        return False

async def main():
    """主函数"""
    print("🚀 开始文件名保存验证测试")
    print("=" * 50)
    
    success_count = 0
    total_count = len(TEST_CASES)
    
    for i, test_case in enumerate(TEST_CASES):
        print(f"\n📋 测试进度: {i+1}/{total_count}")
        
        try:
            result = await test_websocket_client(test_case)
            if result:
                success_count += 1
                print(f"✅ 测试通过: {test_case['description']}")
            else:
                print(f"❌ 测试失败: {test_case['description']}")
                
        except Exception as e:
            print(f"❌ 测试异常: {test_case['description']} - {e}")
        
        # 测试间隔
        if i < total_count - 1:
            print("⏳ 等待2秒后进行下一个测试...")
            await asyncio.sleep(2)
    
    print("\n" + "=" * 50)
    print(f"📊 测试总结:")
    print(f"   - 总测试数: {total_count}")
    print(f"   - 通过数量: {success_count}")
    print(f"   - 失败数量: {total_count - success_count}")
    print(f"   - 成功率: {success_count/total_count*100:.1f}%")
    
    if success_count == total_count:
        print("🎉 所有测试通过！文件名保存功能正常")
    else:
        print("⚠️ 部分测试失败，需要进一步检查")

if __name__ == '__main__':
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n🛑 测试被用户中断")
    except Exception as e:
        print(f"\n❌ 测试运行失败: {e}") 