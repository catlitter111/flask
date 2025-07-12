#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
RFID集成测试脚本
=================

测试RFID系统的完整工作流程：
1. 启动RFID节点
2. 启动WebSocket桥接节点
3. 模拟微信小程序客户端
4. 测试开始/停止盘存命令
5. 验证消息传递和数据显示

使用方法：
    python3 test_rfid_integration.py
"""

import asyncio
import json
import websocket
import threading
import time
import logging
from typing import Dict, Any

# 配置日志
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger('rfid_test')

class MockWeChatClient:
    """模拟微信小程序客户端"""
    
    def __init__(self, server_url: str = "ws://101.201.150.96:1234"):
        self.server_url = server_url
        self.robot_id = "companion_robot_001"
        self.client_id = "test_wechat_client"
        self.ws = None
        self.connected = False
        self.received_messages = []
        
    def connect(self):
        """连接到服务器"""
        try:
            websocket_url = f"{self.server_url}/ws/companion/{self.client_id}"
            logger.info(f"🔗 连接到服务器: {websocket_url}")
            
            self.ws = websocket.WebSocketApp(
                websocket_url,
                on_open=self.on_open,
                on_message=self.on_message,
                on_error=self.on_error,
                on_close=self.on_close
            )
            
            # 在新线程中运行WebSocket
            self.ws_thread = threading.Thread(target=self.ws.run_forever)
            self.ws_thread.daemon = True
            self.ws_thread.start()
            
            # 等待连接建立
            time.sleep(2)
            return self.connected
            
        except Exception as e:
            logger.error(f"❌ 连接失败: {e}")
            return False
    
    def on_open(self, ws):
        """WebSocket连接打开"""
        self.connected = True
        logger.info("✅ WebSocket连接已建立")
        
        # 发送客户端初始化消息
        init_msg = {
            'type': 'client_init',
            'robot_id': self.robot_id,
            'capabilities': {
                'rfid_monitoring': True,
                'rfid_control': True
            },
            'timestamp': int(time.time() * 1000)
        }
        
        self.send_message(init_msg)
        
    def on_message(self, ws, message):
        """处理接收到的消息"""
        try:
            data = json.loads(message)
            message_type = data.get('type', '')
            
            # 记录收到的消息
            self.received_messages.append(data)
            
            logger.info(f"📨 收到消息: {message_type}")
            
            if message_type == 'server_welcome':
                logger.info(f"🎉 服务器欢迎: {data.get('server_version')}")
                
            elif message_type == 'robot_connection_status':
                connected = data.get('connected', False)
                logger.info(f"🤖 机器人连接状态: {'已连接' if connected else '未连接'}")
                
            elif message_type == 'rfid_tags_data':
                # RFID标签数据
                rfid_data = data.get('data', {})
                total_tags = rfid_data.get('total_tags', 0)
                total_reads = rfid_data.get('total_reads', 0)
                tags = rfid_data.get('tags', [])
                
                logger.info(f"📡 RFID标签数据 - 总标签: {total_tags}, 总读取: {total_reads}")
                for i, tag in enumerate(tags[:3]):  # 只显示前3个标签
                    epc = tag.get('epc', 'unknown')
                    rssi = tag.get('rssi_dbm', 0)
                    quality = tag.get('signal_quality', 'unknown')
                    logger.info(f"  标签{i+1}: EPC={epc}, RSSI={rssi}dBm, 质量={quality}")
                    
            elif message_type == 'rfid_status_update':
                # RFID状态更新
                status_data = data.get('data', {})
                connected = status_data.get('connected', False)
                inventory_active = status_data.get('inventory_active', False)
                reader_info = status_data.get('reader_info', {})
                
                logger.info(f"📊 RFID状态更新 - 连接: {'是' if connected else '否'}, 盘存: {'活跃' if inventory_active else '停止'}")
                logger.info(f"  读写器: {reader_info.get('ip', 'unknown')}:{reader_info.get('port', 0)}")
                
            elif message_type == 'rfid_tag_detected':
                # 新标签检测
                tag_data = data.get('data', {})
                epc = tag_data.get('epc', 'unknown')
                rssi = tag_data.get('rssi_dbm', 0)
                quality = tag_data.get('signal_quality', 'unknown')
                
                logger.info(f"🆕 新标签检测 - EPC: {epc}, RSSI: {rssi}dBm, 质量: {quality}")
                
            elif message_type == 'rfid_command_response':
                # RFID命令响应
                command = data.get('command', 'unknown')
                status = data.get('status', 'unknown')
                message_text = data.get('message', '')
                
                if status == 'success':
                    logger.info(f"✅ RFID命令成功 - 命令: {command}, 消息: {message_text}")
                else:
                    error = data.get('error', 'unknown error')
                    logger.error(f"❌ RFID命令失败 - 命令: {command}, 错误: {error}")
                    
            else:
                logger.debug(f"🔍 其他消息类型: {message_type}")
                
        except json.JSONDecodeError:
            logger.error(f"❌ 消息解析失败: {message}")
        except Exception as e:
            logger.error(f"❌ 消息处理错误: {e}")
    
    def on_error(self, ws, error):
        """WebSocket错误处理"""
        logger.error(f"❌ WebSocket错误: {error}")
        
    def on_close(self, ws, close_status_code, close_msg):
        """WebSocket连接关闭"""
        self.connected = False
        logger.info(f"🔌 WebSocket连接已关闭: {close_status_code} - {close_msg}")
    
    def send_message(self, message: Dict[str, Any]):
        """发送消息"""
        if not self.connected or not self.ws:
            logger.error("❌ WebSocket未连接")
            return False
            
        try:
            message_str = json.dumps(message)
            self.ws.send(message_str)
            logger.debug(f"📤 发送消息: {message.get('type', 'unknown')}")
            return True
        except Exception as e:
            logger.error(f"❌ 发送消息失败: {e}")
            return False
    
    def send_rfid_command(self, command: str, params: Dict[str, Any] = None):
        """发送RFID控制命令"""
        if params is None:
            params = {}
            
        rfid_command = {
            'type': 'rfid_command',
            'robot_id': self.robot_id,
            'command': command,
            'params': params,
            'timestamp': int(time.time() * 1000)
        }
        
        logger.info(f"📡🔥 发送RFID命令: {command}, 参数: {params}")
        return self.send_message(rfid_command)
    
    def start_inventory(self):
        """开始RFID盘存"""
        return self.send_rfid_command('start_inventory')
    
    def stop_inventory(self):
        """停止RFID盘存"""
        return self.send_rfid_command('stop_inventory')
    
    def set_antenna(self, antenna_id: int):
        """设置RFID天线"""
        return self.send_rfid_command('set_antenna', {'antenna_id': antenna_id})
    
    def get_rfid_status(self):
        """获取RFID状态"""
        return self.send_rfid_command('get_rfid_status')
    
    def close(self):
        """关闭连接"""
        if self.ws:
            self.ws.close()
            
def main():
    """主测试函数"""
    logger.info("🚀 开始RFID系统集成测试")
    
    # 创建模拟客户端
    client = MockWeChatClient()
    
    try:
        # 连接到服务器
        if not client.connect():
            logger.error("❌ 无法连接到服务器")
            return
        
        logger.info("⏳ 等待系统稳定...")
        time.sleep(3)
        
        # 测试序列
        logger.info("🧪 开始测试序列")
        
        # 1. 获取初始状态
        logger.info("📊 1. 获取初始RFID状态")
        client.get_rfid_status()
        time.sleep(2)
        
        # 2. 开始盘存
        logger.info("▶️ 2. 开始RFID盘存")
        client.start_inventory()
        time.sleep(5)  # 让盘存运行一段时间
        
        # 3. 设置天线
        logger.info("📡 3. 设置RFID天线为2号")
        client.set_antenna(2)
        time.sleep(2)
        
        # 4. 继续监控
        logger.info("👀 4. 监控RFID数据 (10秒)")
        time.sleep(10)
        
        # 5. 停止盘存
        logger.info("⏹️ 5. 停止RFID盘存")
        client.stop_inventory()
        time.sleep(2)
        
        # 6. 最终状态检查
        logger.info("🔍 6. 最终状态检查")
        client.get_rfid_status()
        time.sleep(2)
        
        # 统计结果
        logger.info("📈 测试结果统计")
        logger.info(f"总接收消息数: {len(client.received_messages)}")
        
        # 统计各类消息数量
        message_counts = {}
        for msg in client.received_messages:
            msg_type = msg.get('type', 'unknown')
            message_counts[msg_type] = message_counts.get(msg_type, 0) + 1
        
        for msg_type, count in message_counts.items():
            logger.info(f"  {msg_type}: {count}次")
        
        # 检查是否收到了期望的消息类型
        expected_types = [
            'rfid_command_response',
            'rfid_status_update', 
            'rfid_tags_data'
        ]
        
        missing_types = []
        for expected_type in expected_types:
            if expected_type not in message_counts:
                missing_types.append(expected_type)
        
        if missing_types:
            logger.warning(f"⚠️ 缺少期望的消息类型: {missing_types}")
        else:
            logger.info("✅ 所有期望的消息类型都已收到")
        
        logger.info("🎉 RFID系统集成测试完成")
        
    except KeyboardInterrupt:
        logger.info("🛑 用户中断测试")
    except Exception as e:
        logger.error(f"❌ 测试过程中发生错误: {e}")
    finally:
        # 清理资源
        client.close()
        logger.info("👋 测试结束")

if __name__ == '__main__':
    main()