#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
文件转发功能测试脚本
==================

测试服务器将文件转发给ROS2节点并保存的功能

使用方法：
    python test_file_forward.py

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
CLIENT_ID = 'test_client_001'
ROBOT_ID = 'companion_robot_001'

# 创建测试图片
def create_test_image():
    """创建一个简单的测试图片"""
    try:
        import cv2
        import numpy as np
        
        # 创建一个简单的测试图片
        img = np.zeros((200, 300, 3), dtype=np.uint8)
        cv2.putText(img, 'Test Image', (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.rectangle(img, (50, 50), (250, 150), (0, 255, 0), 2)
        
        # 保存图片
        test_file = Path('test_image.jpg')
        cv2.imwrite(str(test_file), img)
        
        return test_file
    except ImportError:
        # 如果没有OpenCV，创建一个简单的文本文件作为测试
        test_file = Path('test_file.txt')
        with open(test_file, 'w', encoding='utf-8') as f:
            f.write('这是一个测试文件\nTest file for upload\n时间戳: ' + str(time.time()))
        return test_file

async def test_websocket_client():
    """测试WebSocket客户端连接"""
    uri = f"ws://{SERVER_HOST}:{WEBSOCKET_PORT}/ws/companion/{CLIENT_ID}"
    
    try:
        async with websockets.connect(uri) as websocket:
            print(f"✅ WebSocket客户端已连接: {uri}")
            
            # 发送客户端初始化消息
            init_message = {
                'type': 'client_init',
                'robot_id': ROBOT_ID,
                'capabilities': {
                    'video_receive': True,
                    'file_upload': True
                },
                'timestamp': int(time.time() * 1000)
            }
            
            await websocket.send(json.dumps(init_message))
            print("📤 发送客户端初始化消息")
            
            # 等待服务器响应
            response = await websocket.recv()
            data = json.loads(response)
            print(f"📥 收到服务器响应: {data.get('type', 'unknown')}")
            
            # 监听消息
            print("👂 开始监听消息...")
            timeout_count = 0
            while timeout_count < 30:  # 最多等待30秒
                try:
                    response = await asyncio.wait_for(websocket.recv(), timeout=1.0)
                    data = json.loads(response)
                    message_type = data.get('type', 'unknown')
                    print(f"📥 收到消息: {message_type}")
                    
                    if message_type == 'file_save_result':
                        status = data.get('status', 'unknown')
                        if status == 'success':
                            print(f"✅ 文件保存成功: {data.get('saved_path', 'unknown')}")
                        else:
                            print(f"❌ 文件保存失败: {data.get('error', 'unknown')}")
                        break
                        
                except asyncio.TimeoutError:
                    timeout_count += 1
                    if timeout_count % 5 == 0:
                        print(f"⏱️ 等待文件保存结果... ({timeout_count}s)")
            
            if timeout_count >= 30:
                print("⏰ 等待超时，未收到文件保存结果")
                
    except Exception as e:
        print(f"❌ WebSocket连接失败: {e}")

def test_file_upload():
    """测试文件上传"""
    # 创建测试文件
    test_file = create_test_image()
    print(f"📁 创建测试文件: {test_file}")
    
    try:
        # 上传文件
        url = f"http://{SERVER_HOST}:{HTTP_PORT}/api/upload/{CLIENT_ID}"
        
        with open(test_file, 'rb') as f:
            files = {'file': (test_file.name, f, 'image/jpeg')}
            response = requests.post(url, files=files, timeout=30)
        
        if response.status_code == 200:
            result = response.json()
            print(f"✅ 文件上传成功: {result}")
            return True
        else:
            print(f"❌ 文件上传失败: {response.status_code} - {response.text}")
            return False
            
    except Exception as e:
        print(f"❌ 文件上传异常: {e}")
        return False
    finally:
        # 清理测试文件
        if test_file.exists():
            test_file.unlink()

async def main():
    """主测试函数"""
    print("🚀 开始文件转发功能测试")
    print(f"🔗 服务器地址: {SERVER_HOST}")
    print(f"📡 WebSocket端口: {WEBSOCKET_PORT}")
    print(f"🌐 HTTP端口: {HTTP_PORT}")
    print(f"👤 客户端ID: {CLIENT_ID}")
    print(f"🤖 机器人ID: {ROBOT_ID}")
    print("-" * 50)
    
    # 启动WebSocket客户端（在后台运行）
    client_task = asyncio.create_task(test_websocket_client())
    
    # 等待WebSocket连接建立
    await asyncio.sleep(2)
    
    # 测试文件上传
    upload_success = test_file_upload()
    
    if upload_success:
        print("📤 文件已上传，等待ROS2节点处理...")
        # 等待WebSocket客户端接收到文件保存结果
        await client_task
    else:
        print("❌ 文件上传失败，取消等待")
        client_task.cancel()
    
    print("🏁 测试完成")

if __name__ == '__main__':
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n🛑 测试被用户中断")
    except Exception as e:
        print(f"❌ 测试失败: {e}") 