#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
WebSocket桥接节点 - 连接ROS2系统与WebSocket服务器
==============================================

功能说明：
    - 订阅bytetracker节点发布的可视化图像
    - 建立与WebSocket服务器的连接
    - 将图像数据编码并发送给服务器
    - 处理服务器返回的控制命令
    - 发布机器人状态信息

调用关系：
    - 订阅：/bytetracker/visualization（sensor_msgs/Image）
    - 订阅：/bytetracker/tracking_result（custom_msgs/TrackingResult）
    - 发布：/robot_control/command（std_msgs/String）
    - WebSocket连接：与server.py通信

作者：AI Assistant
日期：2024
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float32, Bool, Int32
from geometry_msgs.msg import Point, Twist
from custom_msgs.msg import TrackingResult, RobotStatus
import cv2
from cv_bridge import CvBridge
import json
import base64
import threading
import time
import websocket
import numpy as np
from typing import Optional, Dict, Any
import logging

# 配置日志
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class WebSocketBridgeNode(Node):
    """
    WebSocket桥接节点 - ROS2与WebSocket服务器的通信桥梁
    ==================================================
    
    功能说明：
        作为ROS2系统和WebSocket服务器之间的桥梁，负责：
        1. 图像数据的实时传输
        2. 跟踪结果的状态同步
        3. 远程控制命令的接收和执行
        4. 机器人状态的实时上报
        
    设计目的：
        实现远程监控和控制功能，支持微信小程序等客户端应用
    """
    
    def __init__(self):
        super().__init__('websocket_bridge_node')
        
        try:
            # 直接声明参数，不调用单独的方法
            # WebSocket服务器配置
            self.declare_parameter('websocket_host', '101.201.150.96')
            self.declare_parameter('websocket_port', 1235)
            self.declare_parameter('robot_id', 'companion_robot_001')
            self.declare_parameter('reconnect_interval', 5.0)
            
            # 图像处理配置
            self.declare_parameter('image_quality', 80)
            self.declare_parameter('image_width', 640)
            self.declare_parameter('image_height', 480)
            self.declare_parameter('frame_rate', 15)
            
            # 功能开关
            self.declare_parameter('enable_image_stream', True)
            self.declare_parameter('enable_status_report', True)
            self.declare_parameter('enable_command_receive', True)
            
            self.get_logger().info('✅ 参数声明完成')
            
            # 初始化变量
            self.setup_variables()
            
            # 初始化ROS2组件
            self.setup_ros_components()
            
            # 启动WebSocket连接
            self.setup_websocket()
            
            # 启动心跳和状态监控
            self.setup_timers()
            
            self.get_logger().info('🌉 WebSocket桥接节点已启动')
            
        except Exception as e:
            self.get_logger().error(f'❌ 节点初始化失败: {e}')
            raise
        

        
    def setup_variables(self):
        """初始化变量"""
        # WebSocket配置
        self.ws_host = str(self.get_parameter('websocket_host').value)
        ws_port_param = self.get_parameter('websocket_port').value
        self.ws_port = int(ws_port_param) if ws_port_param is not None else 1235
        self.robot_id = str(self.get_parameter('robot_id').value)
        reconnect_param = self.get_parameter('reconnect_interval').value
        self.reconnect_interval = float(reconnect_param) if reconnect_param is not None else 5.0
        
        # 图像配置
        quality_param = self.get_parameter('image_quality').value
        self.image_quality = int(quality_param) if quality_param is not None else 80
        width_param = self.get_parameter('image_width').value
        self.image_width = int(width_param) if width_param is not None else 640
        height_param = self.get_parameter('image_height').value
        self.image_height = int(height_param) if height_param is not None else 480
        rate_param = self.get_parameter('frame_rate').value
        self.frame_rate = int(rate_param) if rate_param is not None else 15
        
        # 功能开关
        stream_param = self.get_parameter('enable_image_stream').value
        self.enable_image_stream = bool(stream_param) if stream_param is not None else True
        status_param = self.get_parameter('enable_status_report').value
        self.enable_status_report = bool(status_param) if status_param is not None else True
        command_param = self.get_parameter('enable_command_receive').value
        self.enable_command_receive = bool(command_param) if command_param is not None else True
        
        # WebSocket相关
        self.ws = None
        self.ws_connected = False
        self.ws_thread = None
        self.reconnect_timer = None
        
        # 图像处理
        self.bridge = CvBridge()
        self.last_frame_time = 0
        self.frame_interval = 1.0 / self.frame_rate
        self.current_frame = None
        self.frame_sequence = 0
        
        # 状态管理
        self.robot_status = {
            'connected': True,
            'battery_level': 100,
            'signal_strength': 100,
            'mode': 'idle',
            'following_target': None,
            'position': {'x': 0, 'y': 0, 'orientation': 0},
            'last_update': time.time()
        }
        
        # 跟踪结果
        self.tracking_result = None
        self.target_track = None
        
        # 命令处理
        self.pending_commands = []
        self.command_lock = threading.Lock()
        
        self.get_logger().info(f'🔧 配置完成 - 服务器: {self.ws_host}:{self.ws_port}')
        
    def setup_ros_components(self):
        """设置ROS2组件"""
        # 订阅者
        self.image_subscription = self.create_subscription(
            Image,
            '/bytetracker/visualization',
            self.image_callback,
            10
        )
        
        self.tracking_subscription = self.create_subscription(
            TrackingResult,
            '/bytetracker/tracking_result',
            self.tracking_callback,
            10
        )
        
        # 机器人状态订阅（如果有其他节点发布状态）
        self.status_subscription = self.create_subscription(
            RobotStatus,
            '/robot/status',
            self.status_callback,
            10
        )
        
        # 发布者
        self.command_publisher = self.create_publisher(
            String,
            '/robot_control/command',
            10
        )
        
        self.mode_publisher = self.create_publisher(
            String,
            '/bytetracker/mode',
            10
        )
        
        self.target_publisher = self.create_publisher(
            Point,
            '/bytetracker/target_position',
            10
        )
        
        self.get_logger().info('📡 ROS2组件初始化完成')
        
    def setup_websocket(self):
        """设置WebSocket连接"""
        self.connect_websocket()
        
    def connect_websocket(self):
        """连接到WebSocket服务器"""
        try:
            ws_url = f"ws://{self.ws_host}:{self.ws_port}/ws/companion_robot/{self.robot_id}"
            self.get_logger().info(f'🔗 正在连接WebSocket服务器: {ws_url}')
            
            self.ws = websocket.WebSocketApp(
                ws_url,
                on_open=self.on_ws_open,
                on_message=self.on_ws_message,
                on_error=self.on_ws_error,
                on_close=self.on_ws_close
            )
            
            # 在新线程中运行WebSocket
            self.ws_thread = threading.Thread(target=self.ws.run_forever)
            self.ws_thread.daemon = True
            self.ws_thread.start()
            
        except Exception as e:
            self.get_logger().error(f'❌ WebSocket连接失败: {e}')
            self.schedule_reconnect()
            
    def on_ws_open(self, ws):
        """WebSocket连接打开"""
        self.ws_connected = True
        self.get_logger().info('✅ WebSocket连接已建立')
        
        # 发送初始化消息
        init_msg = {
            'type': 'robot_init',
            'robot_id': self.robot_id,
            'capabilities': {
                'video_stream': self.enable_image_stream,
                'status_report': self.enable_status_report,
                'remote_control': self.enable_command_receive
            },
            'timestamp': int(time.time() * 1000)
        }
        
        self.send_ws_message(init_msg)
        
    def on_ws_message(self, ws, message):
        """处理WebSocket消息"""
        try:
            data = json.loads(message)
            message_type = data.get('type', '')
            
            self.get_logger().debug(f'📨 收到消息: {message_type}')
            
            if message_type == 'companion_command':
                self.handle_command(data)
            elif message_type == 'quality_adjustment':
                self.handle_quality_adjustment(data)
            elif message_type == 'heartbeat_ack':
                self.handle_heartbeat_ack(data)
            else:
                self.get_logger().debug(f'🔍 未知消息类型: {message_type}')
                
        except json.JSONDecodeError:
            self.get_logger().error(f'❌ 消息解析失败: {message}')
        except Exception as e:
            self.get_logger().error(f'❌ 消息处理失败: {e}')
            
    def on_ws_error(self, ws, error):
        """WebSocket错误处理"""
        self.get_logger().error(f'❌ WebSocket错误: {error}')
        self.ws_connected = False
        
    def on_ws_close(self, ws, close_status_code, close_msg):
        """WebSocket连接关闭"""
        self.ws_connected = False
        self.get_logger().warn(f'🔌 WebSocket连接已关闭: {close_status_code} - {close_msg}')
        self.schedule_reconnect()
        
    def schedule_reconnect(self):
        """安排重新连接"""
        if self.reconnect_timer is not None:
            self.reconnect_timer.cancel()
            
        self.get_logger().info(f'🔄 {self.reconnect_interval}秒后尝试重新连接')
        self.reconnect_timer = threading.Timer(self.reconnect_interval, self.connect_websocket)
        self.reconnect_timer.start()
        
    def send_ws_message(self, message):
        """发送WebSocket消息"""
        if not self.ws_connected or self.ws is None:
            return False
            
        try:
            if isinstance(message, dict):
                message_str = json.dumps(message)
            else:
                message_str = str(message)
                
            self.ws.send(message_str)
            return True
            
        except Exception as e:
            self.get_logger().error(f'❌ 发送消息失败: {e}')
            self.ws_connected = False
            return False
            
    def image_callback(self, msg):
        """处理接收到的图像"""
        if not self.enable_image_stream or not self.ws_connected:
            return
            
        current_time = time.time()
        
        # 控制帧率
        if current_time - self.last_frame_time < self.frame_interval:
            return
            
        try:
            # 转换为OpenCV图像
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # 调整图像大小
            if cv_image.shape[1] != self.image_width or cv_image.shape[0] != self.image_height:
                cv_image = cv2.resize(cv_image, (self.image_width, self.image_height))
            
            # 编码为JPEG
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.image_quality]
            result, encimg = cv2.imencode('.jpg', cv_image, encode_param)
            
            if result:
                # 转换为base64
                img_base64 = base64.b64encode(encimg).decode('utf-8')
                
                # 准备消息
                self.frame_sequence += 1
                frame_message = {
                    'type': 'video_frame',
                    'robot_id': self.robot_id,
                    'frame_data': img_base64,
                    'sequence': self.frame_sequence,
                    'timestamp': int(current_time * 1000),
                    'width': self.image_width,
                    'height': self.image_height,
                    'quality': self.image_quality
                }
                
                # 发送图像
                if self.send_ws_message(frame_message):
                    self.last_frame_time = current_time
                    self.get_logger().debug(f'📹 发送视频帧 #{self.frame_sequence}')
                    
        except Exception as e:
            self.get_logger().error(f'❌ 图像处理失败: {e}')
            
    def tracking_callback(self, msg):
        """处理跟踪结果"""
        self.tracking_result = msg
        
        # 提取目标跟踪信息
        if hasattr(msg, 'target_detected') and msg.target_detected:
            self.target_track = {
                'id': getattr(msg, 'target_id', -1),
                'position': {
                    'x': getattr(msg, 'target_x', 0),
                    'y': getattr(msg, 'target_y', 0)
                },
                'confidence': getattr(msg, 'confidence', 0.0),
                'distance': getattr(msg, 'distance', 0.0)
            }
        else:
            self.target_track = None
            
        # 更新机器人状态
        self.robot_status['following_target'] = self.target_track
        self.robot_status['mode'] = 'tracking' if self.target_track else 'idle'
        self.robot_status['last_update'] = time.time()
        
    def status_callback(self, msg):
        """处理机器人状态更新"""
        if hasattr(msg, 'battery_level'):
            self.robot_status['battery_level'] = msg.battery_level
        if hasattr(msg, 'signal_strength'):
            self.robot_status['signal_strength'] = msg.signal_strength
        if hasattr(msg, 'mode'):
            self.robot_status['mode'] = msg.mode
            
        self.robot_status['last_update'] = time.time()
        
    def handle_command(self, data):
        """处理远程控制命令"""
        if not self.enable_command_receive:
            return
            
        command = data.get('command', '')
        params = data.get('params', {})
        
        self.get_logger().info(f'📋 收到控制命令: {command}')
        
        try:
            if command == 'start_tracking':
                # 开始跟踪
                mode_msg = String()
                mode_msg.data = 'single_target'
                self.mode_publisher.publish(mode_msg)
                
            elif command == 'stop_tracking':
                # 停止跟踪
                mode_msg = String()
                mode_msg.data = 'multi_target'
                self.mode_publisher.publish(mode_msg)
                
            elif command == 'set_target_position':
                # 设置目标位置
                x = params.get('x', 0)
                y = params.get('y', 0)
                
                target_msg = Point()
                target_msg.x = float(x)
                target_msg.y = float(y)
                target_msg.z = 0.0
                self.target_publisher.publish(target_msg)
                
            elif command == 'emergency_stop':
                # 紧急停止
                cmd_msg = String()
                cmd_msg.data = 'emergency_stop'
                self.command_publisher.publish(cmd_msg)
                
            # 发送命令响应
            response = {
                'type': 'command_response',
                'command': command,
                'status': 'success',
                'timestamp': int(time.time() * 1000)
            }
            self.send_ws_message(response)
            
        except Exception as e:
            self.get_logger().error(f'❌ 命令执行失败: {e}')
            
            # 发送错误响应
            response = {
                'type': 'command_response',
                'command': command,
                'status': 'error',
                'error': str(e),
                'timestamp': int(time.time() * 1000)
            }
            self.send_ws_message(response)
            
    def handle_quality_adjustment(self, data):
        """处理视频质量调整"""
        preset = data.get('preset', 'medium')
        
        # 质量预设映射
        quality_mapping = {
            'high': {'width': 640, 'height': 480, 'quality': 90, 'fps': 20},
            'medium': {'width': 480, 'height': 360, 'quality': 80, 'fps': 15},
            'low': {'width': 320, 'height': 240, 'quality': 70, 'fps': 10},
            'very_low': {'width': 240, 'height': 180, 'quality': 60, 'fps': 8},
            'minimum': {'width': 160, 'height': 120, 'quality': 50, 'fps': 5}
        }
        
        if preset in quality_mapping:
            config = quality_mapping[preset]
            
            # 更新参数
            self.image_width = config['width']
            self.image_height = config['height']
            self.image_quality = config['quality']
            self.frame_rate = config['fps']
            self.frame_interval = 1.0 / self.frame_rate
            
            self.get_logger().info(f'📹 视频质量已调整为: {preset}')
            
            # 发送调整结果
            response = {
                'type': 'quality_adjustment_result',
                'success': True,
                'preset': preset,
                'actual_resolution': f"{self.image_width}x{self.image_height}",
                'actual_fps': self.frame_rate,
                'actual_quality': self.image_quality,
                'timestamp': int(time.time() * 1000)
            }
            self.send_ws_message(response)
            
    def handle_heartbeat_ack(self, data):
        """处理心跳确认"""
        pass  # 目前不需要特殊处理
        
    def setup_timers(self):
        """设置定时器"""
        # 心跳定时器
        self.heartbeat_timer = self.create_timer(15.0, self.send_heartbeat)
        
        # 状态上报定时器
        if self.enable_status_report:
            self.status_timer = self.create_timer(5.0, self.send_status_update)
            
        self.get_logger().info('⏰ 定时器设置完成')
        
    def send_heartbeat(self):
        """发送心跳"""
        if self.ws_connected:
            heartbeat = {
                'type': 'heartbeat',
                'robot_id': self.robot_id,
                'timestamp': int(time.time() * 1000)
            }
            self.send_ws_message(heartbeat)
            
    def send_status_update(self):
        """发送状态更新"""
        if not self.ws_connected:
            return
            
        # 更新连接状态
        self.robot_status['connected'] = self.ws_connected
        self.robot_status['last_update'] = time.time()
        
        status_message = {
            'type': 'robot_status_update',
            'robot_id': self.robot_id,
            'data': self.robot_status.copy(),
            'timestamp': int(time.time() * 1000)
        }
        
        # 添加跟踪特定信息
        if self.tracking_result:
            status_message['data']['tracking_active'] = True
            status_message['data']['target_detected'] = self.target_track is not None
            if self.target_track:
                status_message['data']['target_info'] = self.target_track
        else:
            status_message['data']['tracking_active'] = False
            status_message['data']['target_detected'] = False
            
        self.send_ws_message(status_message)
        
    def destroy_node(self):
        """节点销毁时的清理工作"""
        self.get_logger().info('🔄 正在关闭WebSocket桥接节点...')
        
        # 关闭WebSocket连接
        if self.ws:
            self.ws.close()
            
        # 等待WebSocket线程结束
        if self.ws_thread and self.ws_thread.is_alive():
            self.ws_thread.join(timeout=2.0)
            
        # 取消重连定时器
        if self.reconnect_timer:
            self.reconnect_timer.cancel()
            
        super().destroy_node()
        self.get_logger().info('✅ WebSocket桥接节点已关闭')


def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    try:
        node = WebSocketBridgeNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        logger.info('🛑 收到中断信号，正在关闭...')
    except Exception as e:
        logger.error(f'❌ 节点运行失败: {e}')
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()
        logger.info('👋 WebSocket桥接节点已退出')


if __name__ == '__main__':
    main() 