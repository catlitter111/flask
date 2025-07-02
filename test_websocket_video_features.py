#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
WebSocket视频特征提取测试脚本
==========================
测试通过WebSocket桥接节点上传视频文件并进行特征提取

使用方法:
    python test_websocket_video_features.py <video_path>
"""

import websocket
import json
import base64
import sys
import time
from pathlib import Path
import threading


class VideoFeatureTestClient:
    """视频特征提取测试客户端"""
    
    def __init__(self, server_host='101.201.150.96', server_port=1234):
        self.server_host = server_host
        self.server_port = server_port
        self.client_id = f'test_client_{int(time.time())}'
        self.ws = None
        self.connected = False
        self.response_received = False
        self.test_result = None
        
    def connect(self):
        """连接到WebSocket服务器"""
        try:
            ws_url = f"ws://{self.server_host}:{self.server_port}/ws/client/{self.client_id}"
            print(f'🔗 正在连接服务器: {ws_url}')
            
            self.ws = websocket.WebSocketApp(
                ws_url,
                on_open=self.on_open,
                on_message=self.on_message,
                on_error=self.on_error,
                on_close=self.on_close
            )
            
            # 在新线程中运行WebSocket
            ws_thread = threading.Thread(
                target=self.ws.run_forever,
                kwargs={'ping_interval': 30, 'ping_timeout': 10}
            )
            ws_thread.daemon = True
            ws_thread.start()
            
            # 等待连接建立
            max_wait = 10
            wait_time = 0
            while not self.connected and wait_time < max_wait:
                time.sleep(0.1)
                wait_time += 0.1
                
            if not self.connected:
                print('❌ 连接超时')
                return False
                
            print('✅ 连接成功')
            return True
            
        except Exception as e:
            print(f'❌ 连接失败: {e}')
            return False
            
    def on_open(self, ws):
        """WebSocket连接打开"""
        self.connected = True
        print('📡 WebSocket连接已建立')
        
    def on_message(self, ws, message):
        """处理接收到的消息"""
        try:
            data = json.loads(message)
            message_type = data.get('type', '')
            
            print(f'📨 收到消息: {message_type}')
            
            if message_type == 'file_save_result':
                print(f'📁 文件保存结果: {data.get("status", "unknown")}')
                if data.get('status') == 'success':
                    print(f'  - 原始文件名: {data.get("original_name", "")}')
                    print(f'  - 保存文件名: {data.get("final_name", "")}')
                    print(f'  - 保存路径: {data.get("saved_path", "")}')
                else:
                    print(f'  - 错误信息: {data.get("error", "")}')
                    
            elif message_type == 'feature_extraction_result':
                print('🎉 收到特征提取结果!')
                self.test_result = data
                self.response_received = True
                self.print_feature_results(data)
                
            elif message_type == 'feature_extraction_error':
                print(f'❌ 特征提取失败: {data.get("error", "未知错误")}')
                self.test_result = {'error': data.get("error", "未知错误")}
                self.response_received = True
                
        except json.JSONDecodeError as e:
            print(f'❌ 消息解析失败: {e}')
        except Exception as e:
            print(f'❌ 消息处理失败: {e}')
            
    def on_error(self, ws, error):
        """WebSocket错误处理"""
        print(f'❌ WebSocket错误: {error}')
        
    def on_close(self, ws, close_status_code, close_msg):
        """WebSocket连接关闭"""
        self.connected = False
        print(f'🔌 WebSocket连接已关闭: {close_status_code} - {close_msg}')
        
    def send_message(self, message):
        """发送消息"""
        if self.connected and self.ws:
            try:
                if isinstance(message, dict):
                    message_str = json.dumps(message, ensure_ascii=False)
                else:
                    message_str = str(message)
                self.ws.send(message_str)
                return True
            except Exception as e:
                print(f'❌ 发送消息失败: {e}')
                return False
        return False
        
    def upload_video_file(self, video_path):
        """上传视频文件"""
        try:
            video_file = Path(video_path)
            
            if not video_file.exists():
                print(f'❌ 视频文件不存在: {video_path}')
                return False
                
            if not video_file.suffix.lower() in ['.mp4', '.avi', '.mov', '.mkv', '.flv']:
                print(f'❌ 不支持的视频格式: {video_file.suffix}')
                return False
                
            print(f'📹 开始上传视频: {video_file.name}')
            
            # 读取文件内容
            with open(video_file, 'rb') as f:
                file_data = f.read()
                
            file_size = len(file_data)
            print(f'📦 文件大小: {file_size / 1024 / 1024:.2f} MB')
            
            # 检查文件大小限制（50MB）
            max_size = 50 * 1024 * 1024
            if file_size > max_size:
                print(f'❌ 文件过大: {file_size / 1024 / 1024:.2f} MB > 50 MB')
                return False
                
            # 编码为base64
            file_data_b64 = base64.b64encode(file_data).decode('utf-8')
            
            # 构建上传消息
            upload_message = {
                'type': 'file_upload',
                'file_id': f'video_test_{int(time.time())}',
                'file_name': video_file.name,
                'file_type': f'video/{video_file.suffix[1:]}',  # 移除点号
                'file_size': file_size,
                'file_data_base64': file_data_b64,
                'client_id': self.client_id,
                'timestamp': int(time.time() * 1000),
                'extract_features': True  # 请求特征提取
            }
            
            print('📤 发送文件上传请求...')
            if self.send_message(upload_message):
                print('✅ 文件上传请求已发送')
                return True
            else:
                print('❌ 文件上传请求发送失败')
                return False
                
        except Exception as e:
            print(f'❌ 文件上传失败: {e}')
            return False
            
    def wait_for_result(self, timeout=300):
        """等待特征提取结果"""
        print(f'⏳ 等待特征提取结果（超时: {timeout}秒）...')
        
        start_time = time.time()
        while not self.response_received and (time.time() - start_time) < timeout:
            time.sleep(0.1)
            
        if self.response_received:
            return self.test_result
        else:
            print('❌ 等待结果超时')
            return None
            
    def print_feature_results(self, result_data):
        """打印特征提取结果"""
        data = result_data.get('data', {})
        
        print('\n📊 === 视频特征提取结果 ===')
        print(f'✅ 状态: {data.get("status", "unknown")}')
        print(f'📝 消息: {data.get("message", "")}')
        print(f'👥 检测到人数: {data.get("person_count", 0)}')
        print(f'📏 身体比例数据: {len(data.get("body_ratios", []))} 项')
        
        # 显示身体比例
        body_ratios = data.get("body_ratios", [])
        if body_ratios:
            print('\n📐 身体比例详情:')
            ratio_names = [
                '上肢下肢比例', '躯干身高比例', '肩宽身高比例', '臀宽肩宽比例',
                '头部躯干比例', '手臂身高比例', '腿长身高比例', '上臂下臂比例', 
                '大腿小腿比例', '躯干腿长比例', '手臂腿长比例', '肩宽髋宽比例',
                '头围身高比例', '脚长身高比例', '脚踝宽度比例', '腰围身高比例'
            ]
            
            for i, (name, ratio) in enumerate(zip(ratio_names, body_ratios)):
                print(f'  {i+1:2d}. {name}: {ratio:.4f}')
        
        # 显示服装颜色
        shirt_color = data.get("shirt_color", [0, 0, 0])
        pants_color = data.get("pants_color", [0, 0, 0])
        print(f'\n👕 上装颜色 (RGB): {shirt_color}')
        print(f'👖 下装颜色 (RGB): {pants_color}')
        
        # 显示文件路径
        result_image = data.get("result_image_path", "")
        feature_data = data.get("feature_data_path", "")
        if result_image:
            print(f'🖼️ 结果图像: {result_image}')
        if feature_data:
            print(f'📊 特征数据: {feature_data}')
            
        print(f'📂 文件类型: {data.get("file_type", "unknown")}')
        print('=' * 50)
        
    def test_video_feature_extraction(self, video_path):
        """测试视频特征提取"""
        print(f'🎬 开始视频特征提取测试')
        print(f'📹 视频文件: {video_path}')
        
        # 连接服务器
        if not self.connect():
            return False
            
        # 上传视频文件
        if not self.upload_video_file(video_path):
            return False
            
        # 等待结果
        result = self.wait_for_result()
        
        if result:
            if 'error' in result:
                print(f'❌ 测试失败: {result["error"]}')
                return False
            else:
                print('🎉 测试成功完成!')
                return True
        else:
            print('❌ 测试失败：未收到结果')
            return False
            
    def close(self):
        """关闭连接"""
        if self.ws:
            self.ws.close()


def main():
    if len(sys.argv) != 2:
        print("使用方法: python test_websocket_video_features.py <video_path>")
        print("示例: python test_websocket_video_features.py /path/to/video.mp4")
        sys.exit(1)
    
    video_path = sys.argv[1]
    
    try:
        # 创建测试客户端
        client = VideoFeatureTestClient()
        
        # 执行测试
        success = client.test_video_feature_extraction(video_path)
        
        # 关闭连接
        client.close()
        
        if success:
            print("\n🎉 视频特征提取测试成功!")
            sys.exit(0)
        else:
            print("\n💥 视频特征提取测试失败!")
            sys.exit(1)
        
    except KeyboardInterrupt:
        print("\n⛔ 测试被用户中断")
        sys.exit(1)
    except Exception as e:
        print(f"\n❌ 测试过程中发生错误: {e}")
        sys.exit(1)


if __name__ == '__main__':
    main() 