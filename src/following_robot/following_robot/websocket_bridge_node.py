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
from custom_msgs.srv import FeatureExtraction
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
from pathlib import Path
import traceback

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
            self.declare_parameter('websocket_port', 1234)
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
        self.ws_port = int(ws_port_param) if ws_port_param is not None else 1234
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
        
        # 图像压缩优化
        self.network_quality = 'good'  # good, fair, poor
        self.adaptive_quality = True
        self.compression_stats = {
            'total_frames': 0,
            'total_size': 0,
            'avg_compression_ratio': 1.0,
            'last_update': time.time()
        }
        
        # 跳帧控制
        self.frame_skip_counter = 0
        self.current_skip_rate = 1  # 1=不跳帧, 2=每2帧发送1帧
        
        # 文件保存配置
        self.file_save_dir = Path('received_files')
        self.file_save_dir.mkdir(exist_ok=True)
        self.max_file_size = 50 * 1024 * 1024  # 50MB
        
        self.get_logger().info(f'🔧 配置完成 - 服务器: {self.ws_host}:{self.ws_port}')
        self.get_logger().info(f'📁 文件保存目录: {self.file_save_dir.absolute()}')
        
        # 特征提取服务状态
        self.feature_service_available = False
        
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
        
        # 特征提取结果订阅
        self.feature_result_subscription = self.create_subscription(
            String,
            '/features/processing_result',
            self.feature_result_callback,
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
        
        # 服务客户端
        self.feature_extraction_client = self.create_client(
            FeatureExtraction,
            '/features/extract_features'
        )
        
        self.get_logger().info('📡 ROS2组件初始化完成')
        self.get_logger().info('🔧 等待特征提取服务...')
        
        # 等待特征提取服务可用（非阻塞）
        self.check_feature_service_timer = self.create_timer(2.0, self.check_feature_service)
        
    def setup_websocket(self):
        """设置WebSocket连接"""
        self.connect_websocket()
        
    def connect_websocket(self):
        """连接到WebSocket服务器"""
        try:
            ws_url = f"ws://{self.ws_host}:{self.ws_port}/ws/ros2_bridge/{self.robot_id}"
            self.get_logger().info(f'🔗 正在连接WebSocket服务器: {ws_url}')
            
            self.ws = websocket.WebSocketApp(
                ws_url,
                on_open=self.on_ws_open,
                on_message=self.on_ws_message,
                on_error=self.on_ws_error,
                on_close=self.on_ws_close
            )
            
            # 在新线程中运行WebSocket，配置心跳参数与服务端匹配
            self.ws_thread = threading.Thread(
                target=self.ws.run_forever,
                kwargs={
                    'ping_interval': 18,  # 比服务端的20秒稍小，确保客户端主动发送ping
                    'ping_timeout': 8,    # 比服务端的10秒稍小
                    'ping_payload': b'keepalive'  # 添加心跳负载
                }
            )
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
            elif message_type == 'file_upload_forward':
                self.handle_file_upload(data)
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
            
        # 跳帧控制
        self.frame_skip_counter += 1
        if self.frame_skip_counter % self.current_skip_rate != 0:
            return
            
        try:
            # 转换为OpenCV图像
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # 动态调整图像大小（基于网络状况）
            target_width, target_height = self.get_adaptive_resolution()
            if cv_image.shape[1] != target_width or cv_image.shape[0] != target_height:
                cv_image = cv2.resize(cv_image, (target_width, target_height), interpolation=cv2.INTER_AREA)
            
            # 应用预处理优化
            cv_image = self.preprocess_image(cv_image)
            
            # 选择最佳编码格式和参数
            encoded_data, format_type = self.encode_image_optimized(cv_image)
            
            if encoded_data is not None:
                # 转换为base64
                img_base64 = base64.b64encode(encoded_data).decode('utf-8')
                
                # 计算压缩率
                original_size = cv_image.shape[0] * cv_image.shape[1] * cv_image.shape[2]
                compressed_size = len(encoded_data)
                compression_ratio = original_size / compressed_size
                
                # 准备消息
                self.frame_sequence += 1
                frame_message = {
                    'type': 'video_frame',
                    'robot_id': self.robot_id,
                    'frame_data': img_base64,
                    'sequence': self.frame_sequence,
                    'timestamp': int(current_time * 1000),
                    'width': target_width,
                    'height': target_height,
                    'quality': self.image_quality,
                    'format': format_type,
                    'compressed_size': compressed_size,
                    'compression_ratio': round(compression_ratio, 2)
                }
                
                # 发送图像
                if self.send_ws_message(frame_message):
                    self.last_frame_time = current_time
                    self.get_logger().debug(f'📹 发送视频帧 #{self.frame_sequence} '
                                          f'({target_width}x{target_height}, {compressed_size}B, '
                                          f'压缩率:{compression_ratio:.1f}x)')
                    
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
        
    def feature_result_callback(self, msg):
        """处理特征提取结果"""
        try:
            # 解析JSON消息
            result_data = json.loads(msg.data)
            
            self.get_logger().info(f"📊 收到特征提取结果: {result_data.get('extraction_id', 'unknown')}")
            
            # 获取文件路径
            files = result_data.get('files', {})
            result_image_path = files.get('result_image', '')
            
            if result_image_path and Path(result_image_path).exists():
                # 读取并转发处理后的图片
                self.forward_processed_image(result_data, result_image_path)
            else:
                self.get_logger().warning(f"⚠️ 结果图片文件不存在: {result_image_path}")
                
        except json.JSONDecodeError as e:
            self.get_logger().error(f"❌ 解析特征提取结果失败: {e}")
        except Exception as e:
            self.get_logger().error(f"❌ 处理特征提取结果失败: {e}")
    
    def forward_processed_image(self, result_data, image_path):
        """转发处理后的图片给客户端"""
        try:
            # 读取图片文件
            image_path_obj = Path(image_path)
            if not image_path_obj.exists():
                self.get_logger().error(f"❌ 图片文件不存在: {image_path}")
                return
            
            # 读取图片数据
            with open(image_path, 'rb') as f:
                image_data = f.read()
            
            # 编码为base64
            image_base64 = base64.b64encode(image_data).decode('utf-8')
            
            # 获取图片信息
            file_size = len(image_data)
            file_name = image_path_obj.name
            
            # 提取特征数据
            features = result_data.get('features', {})
            body_ratios = features.get('body_ratios', [0.0] * 16)
            shirt_color = features.get('shirt_color', [0, 0, 0])
            pants_color = features.get('pants_color', [0, 0, 0])
            
            # 构建格式化的特征数据
            formatted_features = {
                'body_ratios': body_ratios,
                'clothing_colors': {
                    'top': {
                        'name': self.get_color_name(shirt_color),
                        'color': self.rgb_to_hex(shirt_color),
                        'confidence': 85  # 默认置信度
                    },
                    'bottom': {
                        'name': self.get_color_name(pants_color),
                        'color': self.rgb_to_hex(pants_color),
                        'confidence': 85  # 默认置信度
                    }
                },
                'body_proportions': self.format_body_proportions(body_ratios),
                'detailed_proportions': self.format_detailed_proportions(body_ratios)
            }
            
            # 构建转发消息（格式与小程序期望的一致）
            forward_message = {
                'type': 'processed_image_notification',
                'extraction_id': result_data.get('extraction_id', 'unknown'),
                'person_name': result_data.get('person_name', ''),
                'timestamp': result_data.get('timestamp', int(time.time() * 1000)),
                'robot_id': self.robot_id,
                # 图片数据（小程序期望的格式）
                'original_image': f'data:image/jpeg;base64,{image_base64}',  # 暂时用处理后的图片
                'processed_image': f'data:image/jpeg;base64,{image_base64}',
                'result_image': f'data:image/jpeg;base64,{image_base64}',
                # 特征数据
                'features': formatted_features,
                'colors': formatted_features['clothing_colors'],
                'proportions': formatted_features['body_proportions'],
                # 兼容字段
                'topColor': self.rgb_to_hex(shirt_color),
                'bottomColor': self.rgb_to_hex(pants_color),
                'topColorName': self.get_color_name(shirt_color),
                'bottomColorName': self.get_color_name(pants_color),
                'body_proportions': formatted_features['body_proportions'],
                'detailed_proportions': formatted_features['detailed_proportions'],
                # 文件信息
                'files': result_data.get('files', {}),
                'processing_info': {
                    'has_result_image': True,
                    'has_feature_data': bool(result_data.get('files', {}).get('feature_data')),
                    'has_result_video': bool(result_data.get('files', {}).get('result_video')),
                    'image_size_bytes': file_size,
                    'compression_info': f'原始图片大小: {file_size}字节',
                    'feature_count': len(body_ratios),
                    'has_valid_features': features.get('has_valid_data', False)
                }
            }
            
            # 发送给WebSocket服务器
            if self.send_ws_message(forward_message):
                self.get_logger().info(f"📤 已转发处理后图片: {file_name} (大小: {file_size}字节)")
                self.get_logger().info(f"🎯 特征数据: 身体比例{len(body_ratios)}个, "
                                     f"有效数据: {features.get('has_valid_data', False)}")
                self.get_logger().info(f"🎨 颜色数据: 上衣{shirt_color}, 下装{pants_color}")
            else:
                self.get_logger().warning(f"⚠️ 转发处理后图片失败: WebSocket未连接")
                
        except Exception as e:
            self.get_logger().error(f"❌ 转发处理后图片失败: {e}")
            traceback.print_exc()
    
    def rgb_to_hex(self, rgb):
        """将RGB值转换为十六进制颜色"""
        try:
            if isinstance(rgb, (list, tuple)) and len(rgb) >= 3:
                r, g, b = int(rgb[0]), int(rgb[1]), int(rgb[2])
                return f"#{r:02x}{g:02x}{b:02x}"
            else:
                return "#000000"
        except:
            return "#000000"
    
    def get_color_name(self, rgb):
        """根据RGB值获取颜色名称"""
        try:
            if not isinstance(rgb, (list, tuple)) or len(rgb) < 3:
                return "黑色"
            
            r, g, b = int(rgb[0]), int(rgb[1]), int(rgb[2])
            
            # 简单的颜色识别逻辑
            if r > 200 and g > 200 and b > 200:
                return "白色"
            elif r < 50 and g < 50 and b < 50:
                return "黑色"
            elif r > g and r > b:
                if g > b:
                    return "橙色" if g > 100 else "红色"
                else:
                    return "红色"
            elif g > r and g > b:
                if r > b:
                    return "黄色"
                elif b > r:
                    return "青色"
                else:
                    return "绿色"
            elif b > r and b > g:
                if r > g:
                    return "紫色"
                else:
                    return "蓝色"
            else:
                return "灰色"
        except:
            return "未知"
    
    def format_body_proportions(self, body_ratios):
        """格式化身体比例数据"""
        try:
            if not body_ratios or len(body_ratios) < 16:
                body_ratios = [0.0] * 16
            
            return {
                'height': f"{body_ratios[0]:.3f}",
                'shoulderWidth': f"{body_ratios[3]:.3f}",
                'chest': f"{body_ratios[5]:.3f}",
                'waist': f"{body_ratios[7]:.3f}",
                'hip': f"{body_ratios[9]:.3f}",
                'armLength': f"{body_ratios[10]:.3f}",
                'legLength': f"{body_ratios[12]:.3f}",
                'headHeight': f"{body_ratios[1]:.3f}",
                'neckHeight': f"{body_ratios[2]:.3f}",
                'torsoLength': f"{(body_ratios[5] + body_ratios[7]):.3f}",
                'thighLength': f"{body_ratios[13]:.3f}",
                'calfLength': f"{body_ratios[14]:.3f}",
                'footLength': f"{body_ratios[15]:.3f}",
                'handLength': f"{(body_ratios[10] * 0.15):.3f}",  # 估算
                'forearmLength': f"{body_ratios[11]:.3f}",
                'upperArmLength': f"{(body_ratios[10] - body_ratios[11]):.3f}"
            }
        except:
            return {
                'height': "0.000",
                'shoulderWidth': "0.000",
                'chest': "0.000",
                'waist': "0.000",
                'hip': "0.000",
                'armLength': "0.000",
                'legLength': "0.000",
                'headHeight': "0.000",
                'neckHeight': "0.000",
                'torsoLength': "0.000",
                'thighLength': "0.000",
                'calfLength': "0.000",
                'footLength': "0.000",
                'handLength': "0.000",
                'forearmLength': "0.000",
                'upperArmLength': "0.000"
            }
    
    def format_detailed_proportions(self, body_ratios):
        """格式化详细比例数据"""
        try:
            if not body_ratios or len(body_ratios) < 16:
                body_ratios = [0.0] * 16
            
            labels = [
                '身高', '头部高度', '颈部高度', '肩膀宽度',
                '胸部宽度', '胸围', '腰部宽度', '腰围',
                '臀部宽度', '臀围', '手臂长度', '前臂长度',
                '腿部长度', '大腿长度', '小腿长度', '脚部长度'
            ]
            
            return [
                {
                    'label': labels[i] if i < len(labels) else f'特征{i+1}',
                    'value': f"{body_ratios[i]:.3f}" if i < len(body_ratios) else "0.000"
                }
                for i in range(16)
            ]
        except:
            labels = [
                '身高', '头部高度', '颈部高度', '肩膀宽度',
                '胸部宽度', '胸围', '腰部宽度', '腰围',
                '臀部宽度', '臀围', '手臂长度', '前臂长度',
                '腿部长度', '大腿长度', '小腿长度', '脚部长度'
            ]
            return [
                {
                    'label': labels[i],
                    'value': "0.000"
                }
                for i in range(16)
            ]
    
    def handle_command(self, data):
        """处理远程控制命令"""
        if not self.enable_command_receive:
            return
            
        command = data.get('command', '')
        params = data.get('params', {})
        
        # 专门打印运动和模式切换指令
        if command in ['forward', 'backward', 'left', 'right', 'stop', 'start_auto', 'pause_auto', 'startInteraction', 'stopInteraction']:
            self.get_logger().info(f'🎮🔥 [运动/模式指令] 收到命令: {command}, 参数: {params}')
        else:
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
                
            elif command == 'request_video_stream':
                # 请求视频流 - 直接开启视频流传输
                self.enable_image_stream = True
                self.get_logger().info('📹 视频流已开启')
                
            elif command == 'quality_adjustment':
                # 质量调整命令
                self.handle_quality_adjustment(data)
                
            # 添加手动控制命令处理
            elif command in ['forward', 'backward', 'left', 'right', 'stop']:
                # 手动控制命令
                self.handle_manual_control(command, params)
                
            elif command == 'start_auto':
                # 开始自动模式
                cmd_msg = String()
                cmd_msg.data = 'start_auto_mode'
                self.command_publisher.publish(cmd_msg)
                self.get_logger().info('🤖🔥 [模式切换] start_auto -> ROS命令: "start_auto_mode"')
                
            elif command == 'pause_auto':
                # 暂停自动模式
                cmd_msg = String()
                cmd_msg.data = 'pause_auto_mode'
                self.command_publisher.publish(cmd_msg)
                self.get_logger().info('⏸️🔥 [模式切换] pause_auto -> ROS命令: "pause_auto_mode"')
                
            elif command == 'startInteraction':
                # 开始交互模式
                cmd_msg = String()
                cmd_msg.data = 'start_interaction'
                self.command_publisher.publish(cmd_msg)
                self.get_logger().info('🤝🔥 [模式切换] startInteraction -> ROS命令: "start_interaction"')
                
            elif command == 'stopInteraction':
                # 停止交互模式
                cmd_msg = String()
                cmd_msg.data = 'stop_interaction'
                self.command_publisher.publish(cmd_msg)
                self.get_logger().info('🛑🔥 [模式切换] stopInteraction -> ROS命令: "stop_interaction"')
                
            elif command == 'set_motor_speed':
                # 设置电机速度
                speed = params.get('speed', 50)
                cmd_msg = String()
                cmd_msg.data = f'set_motor_speed:{speed}'
                self.command_publisher.publish(cmd_msg)
                self.get_logger().info(f'⚡ 设置电机速度: {speed}%')
                
            elif command == 'switch_control_type':
                # 切换控制类型
                control_type = params.get('control_type', 'motor')
                cmd_msg = String()
                cmd_msg.data = f'switch_control_type:{control_type}'
                self.command_publisher.publish(cmd_msg)
                self.get_logger().info(f'🔄 切换控制类型: {control_type}')
                
            else:
                # 未知命令，记录日志但不报错
                self.get_logger().warning(f'⚠️ 未知命令: {command}')
                
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
            

    def handle_manual_control(self, command, params):
        """处理手动控制命令"""
        try:
            # 获取控制参数
            speed = params.get('speed', 50)
            control_type = params.get('control_type', 'motor')
            
            # 根据控制类型和命令构建ROS命令
            if control_type == 'companion':
                # 伴侣交互控制模式
                companion_command_map = {
                    'forward': 'companion_look_up',
                    'backward': 'companion_look_down', 
                    'left': 'companion_turn_left',
                    'right': 'companion_turn_right',
                    'stop': 'companion_stop'
                }
                ros_command = companion_command_map.get(command, f'companion_{command}')
            else:
                # 电机控制模式
                if command == 'stop':
                    ros_command = 'motor_stop'
                else:
                    ros_command = f'motor_{command}:{speed}'
            
            # 发布ROS命令
            cmd_msg = String()
            cmd_msg.data = ros_command
            self.command_publisher.publish(cmd_msg)
            
            self.get_logger().info(f'🎮🔥 [手动控制执行] {command} -> ROS命令: "{ros_command}" (类型: {control_type}, 速度: {speed}%)')
            
        except Exception as e:
            self.get_logger().error(f'❌ 手动控制命令处理失败: {e}')
            raise
    
    def handle_heartbeat_ack(self, data):
        """处理心跳确认"""
        pass  # 目前不需要特殊处理
        
    def check_feature_service(self):
        """检查特征提取服务是否可用"""
        if self.feature_extraction_client.service_is_ready():
            if not self.feature_service_available:
                self.feature_service_available = True
                self.get_logger().info('✅ 特征提取服务已就绪')
                # 取消定时检查
                self.check_feature_service_timer.cancel()
        else:
            if self.feature_service_available:
                self.feature_service_available = False
                self.get_logger().warn('⚠️ 特征提取服务不可用')

    def setup_timers(self):
        """设置定时器"""
        # 心跳定时器 - 调整为25秒，与WebSocket ping机制协调
        self.heartbeat_timer = self.create_timer(25.0, self.send_heartbeat)
        
        # 状态上报定时器
        if self.enable_status_report:
            self.status_timer = self.create_timer(5.0, self.send_status_update)
            
        self.get_logger().info('⏰ 定时器设置完成 (心跳: 25s, 状态: 5s)')
        
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
        
    def get_adaptive_resolution(self):
        """根据网络状况动态调整分辨率"""
        if self.network_quality == 'poor':
            return min(160, self.image_width), min(120, self.image_height)
        elif self.network_quality == 'fair':
            return min(240, self.image_width), min(180, self.image_height)
        else:  # good
            return min(480, self.image_width), min(360, self.image_height)
    
    def preprocess_image(self, image):
        """图像预处理优化"""
        try:
            # 如果网络质量差，应用更激进的优化
            if self.network_quality == 'poor':
                # 降噪和锐化
                image = cv2.bilateralFilter(image, 5, 50, 50)
                # 减少颜色深度
                image = (image // 8) * 8
            elif self.network_quality == 'fair':
                # 轻微降噪
                image = cv2.bilateralFilter(image, 3, 30, 30)
            
            return image
        except Exception as e:
            self.get_logger().warning(f'图像预处理失败: {e}')
            return image
    
    def encode_image_optimized(self, image):
        """优化的图像编码"""
        try:
            # 根据网络质量选择编码参数
            if self.network_quality == 'poor':
                # 极低质量，最大压缩
                quality = max(20, self.image_quality // 3)
                encode_param = [
                    int(cv2.IMWRITE_JPEG_QUALITY), quality,
                    int(cv2.IMWRITE_JPEG_OPTIMIZE), 1,
                    int(cv2.IMWRITE_JPEG_PROGRESSIVE), 1
                ]
            elif self.network_quality == 'fair':
                # 中等质量
                quality = max(40, self.image_quality // 2)
                encode_param = [
                    int(cv2.IMWRITE_JPEG_QUALITY), quality,
                    int(cv2.IMWRITE_JPEG_OPTIMIZE), 1
                ]
            else:
                # 标准质量
                quality = self.image_quality
                encode_param = [
                    int(cv2.IMWRITE_JPEG_QUALITY), quality,
                    int(cv2.IMWRITE_JPEG_OPTIMIZE), 1
                ]
            
            # 尝试JPEG编码
            success, encoded_data = cv2.imencode('.jpg', image, encode_param)
            
            if success:
                # 更新压缩统计
                self.update_compression_stats(len(encoded_data))
                return encoded_data.tobytes(), 'jpeg'
            else:
                self.get_logger().error('JPEG编码失败')
                return None, None
                
        except Exception as e:
            self.get_logger().error(f'图像编码失败: {e}')
            return None, None
    
    def update_compression_stats(self, compressed_size):
        """更新压缩统计信息"""
        try:
            self.compression_stats['total_frames'] += 1
            self.compression_stats['total_size'] += compressed_size
            
            # 每10帧更新一次网络质量评估
            if self.compression_stats['total_frames'] % 10 == 0:
                avg_size = self.compression_stats['total_size'] / self.compression_stats['total_frames']
                self.evaluate_network_quality(avg_size)
                
        except Exception as e:
            self.get_logger().warning(f'更新压缩统计失败: {e}')
    
    def evaluate_network_quality(self, avg_frame_size):
        """评估网络质量并调整参数"""
        try:
            # 基于平均帧大小和延迟评估网络质量
            if avg_frame_size > 50000:  # 50KB
                new_quality = 'poor'
                new_skip_rate = 3  # 每3帧发送1帧
            elif avg_frame_size > 25000:  # 25KB
                new_quality = 'fair'
                new_skip_rate = 2  # 每2帧发送1帧
            else:
                new_quality = 'good'
                new_skip_rate = 1  # 不跳帧
            
            # 更新网络质量
            if new_quality != self.network_quality:
                self.network_quality = new_quality
                self.current_skip_rate = new_skip_rate
                self.get_logger().info(f'📊 网络质量调整: {new_quality}, 跳帧率: {new_skip_rate}')
                
        except Exception as e:
            self.get_logger().warning(f'网络质量评估失败: {e}')
    
    def handle_quality_adjustment(self, data):
        """处理质量调整命令"""
        try:
            preset = data.get('preset', 'medium')
            immediate = data.get('immediate', False)
            
            # 质量预设映射 - 优化压缩和延迟
            quality_mapping = {
                'ultra_high': {'width': 800, 'height': 600, 'quality': 85, 'fps': 20, 'skip': 1},
                'high': {'width': 640, 'height': 480, 'quality': 75, 'fps': 15, 'skip': 1},
                'medium': {'width': 480, 'height': 360, 'quality': 65, 'fps': 12, 'skip': 1},
                'low': {'width': 320, 'height': 240, 'quality': 55, 'fps': 10, 'skip': 1},
                'very_low': {'width': 240, 'height': 180, 'quality': 45, 'fps': 8, 'skip': 2},
                'minimum': {'width': 160, 'height': 120, 'quality': 35, 'fps': 5, 'skip': 2},
                'ultra_low': {'width': 120, 'height': 90, 'quality': 25, 'fps': 3, 'skip': 3}
            }
            
            if preset in quality_mapping:
                settings = quality_mapping[preset]
                
                # 更新图像参数
                self.image_width = settings['width']
                self.image_height = settings['height']
                self.image_quality = settings['quality']
                self.current_skip_rate = settings['skip']
                
                # 更新帧率
                target_fps = settings['fps']
                self.frame_interval = 1.0 / target_fps if target_fps > 0 else 1.0 / 10
                
                # 根据预设调整网络质量评估
                if preset in ['ultra_low', 'minimum']:
                    self.network_quality = 'poor'
                elif preset in ['very_low', 'low']:
                    self.network_quality = 'fair'
                else:
                    self.network_quality = 'good'
                
                self.get_logger().info(f'🎛️ 质量调整完成: {preset} '
                                     f'({self.image_width}x{self.image_height}, '
                                     f'质量:{self.image_quality}, '
                                     f'帧率:{target_fps}fps, '
                                     f'跳帧:{self.current_skip_rate})')
                
                if immediate:
                    # 立即重置帧计数器以立即生效
                    self.frame_skip_counter = 0
                    self.last_frame_time = 0
                    
            else:
                self.get_logger().warning(f'未知的质量预设: {preset}')
                
        except Exception as e:
            self.get_logger().error(f'质量调整失败: {e}')
    
    def handle_file_upload(self, data):
        """处理文件上传命令"""
        try:
            file_id = data.get('file_id', '')
            file_name = data.get('file_name', '')
            file_data_b64 = data.get('file_data_base64', '')
            file_type = data.get('file_type', '')
            file_size = data.get('file_size', 0)
            client_id = data.get('client_id', '')
            
            self.get_logger().info(f'📂 收到文件上传 - 文件: {file_name}, 大小: {file_size}字节, 来源: {client_id}')
            
            if not file_id or not file_name or not file_data_b64:
                self.get_logger().error('❌ 文件数据不完整')
                return
                
            # 检查文件大小
            if file_size > self.max_file_size:
                self.get_logger().error(f'❌ 文件过大: {file_size} > {self.max_file_size}')
                return
                
            # 解码base64数据
            try:
                file_data = base64.b64decode(file_data_b64)
                if len(file_data) != file_size:
                    self.get_logger().warning(f'⚠️ 文件大小不匹配: 期望{file_size}, 实际{len(file_data)}')
            except Exception as e:
                self.get_logger().error(f'❌ base64解码失败: {e}')
                return
                
            # 生成保存路径
            # 直接使用用户输入的文件名，保持原始性
            safe_filename = self.sanitize_filename(file_name)
            
            # 如果文件名已存在，则添加数字后缀避免冲突
            save_path = self.file_save_dir / safe_filename
            counter = 1
            while save_path.exists():
                name_part, ext_part = safe_filename.rsplit('.', 1) if '.' in safe_filename else (safe_filename, '')
                new_filename = f"{name_part}_{counter}.{ext_part}" if ext_part else f"{safe_filename}_{counter}"
                save_path = self.file_save_dir / new_filename
                counter += 1
                
            # 最终使用的文件名
            final_filename = save_path.name
            
            # 保存文件
            try:
                with open(save_path, 'wb') as f:
                    f.write(file_data)
                    
                self.get_logger().info(f'✅ 文件保存成功 - 用户名称: {file_name} → 保存为: {final_filename} (路径: {save_path})')
                
                # 发送保存成功通知
                response = {
                    'type': 'file_save_result',
                    'status': 'success',
                    'file_id': file_id,
                    'original_name': file_name,
                    'final_name': final_filename,
                    'saved_path': str(save_path),
                    'saved_size': len(file_data),
                    'client_id': client_id,
                    'robot_id': self.robot_id,
                    'timestamp': int(time.time() * 1000)
                }
                self.send_ws_message(response)
                
                # 如果是图片或视频，触发特征提取
                if file_type.startswith('image/') or file_type.startswith('video/'):
                    self.trigger_feature_extraction(save_path, file_id, client_id)
                    
            except Exception as e:
                self.get_logger().error(f'❌ 文件保存失败: {e}')
                
                # 发送保存失败通知
                response = {
                    'type': 'file_save_result',
                    'status': 'error',
                    'file_id': file_id,
                    'error': str(e),
                    'client_id': client_id,
                    'robot_id': self.robot_id,
                    'timestamp': int(time.time() * 1000)
                }
                self.send_ws_message(response)
                
        except Exception as e:
            self.get_logger().error(f'❌ 文件处理失败: {e}')
            
    def sanitize_filename(self, filename):
        """清理文件名，移除不安全字符"""
        import re
        # 保留字母、数字、点、下划线和连字符
        safe_name = re.sub(r'[^\w\.-]', '_', filename)
        # 限制长度
        if len(safe_name) > 100:
            name_part, ext_part = safe_name.rsplit('.', 1) if '.' in safe_name else (safe_name, '')
            safe_name = name_part[:90] + ('.' + ext_part if ext_part else '')
        return safe_name
        
    def trigger_feature_extraction(self, file_path, file_id, client_id):
        """触发特征提取"""
        try:
            self.get_logger().info(f'🔍 开始特征提取: {file_path}')
            
            # 检查特征提取服务是否可用
            if not self.feature_service_available:
                self.get_logger().warn('⚠️ 特征提取服务不可用，跳过特征提取')
                # 通知客户端服务不可用
                self.send_feature_extraction_error(client_id, file_id, '特征提取服务不可用')
                return
            
            # 获取文件信息
            file_path_obj = Path(file_path)
            file_name = file_path_obj.name
            file_ext = file_path_obj.suffix.lower()
            
            # 判断文件类型
            if file_ext in ['.jpg', '.jpeg', '.png', '.bmp', '.tiff']:
                # 图片文件 - 调用现有服务
                self.extract_features_from_image(file_path, file_name, client_id, file_id)
            elif file_ext in ['.mp4', '.avi', '.mov', '.mkv', '.flv']:
                # 视频文件 - 预留接口
                self.extract_features_from_video(file_path, file_name, client_id, file_id)
            else:
                self.get_logger().warn(f'⚠️ 不支持的文件类型: {file_ext}')
                self.send_feature_extraction_error(client_id, file_id, f'不支持的文件类型: {file_ext}')
            
        except Exception as e:
            self.get_logger().error(f'❌ 特征提取触发失败: {e}')
            self.send_feature_extraction_error(client_id, file_id, f'特征提取失败: {str(e)}')
    
    def extract_features_from_image(self, file_path, file_name, client_id, file_id):
        """从图片提取特征"""
        try:
            # 读取图像文件
            cv_image = cv2.imread(str(file_path))
            if cv_image is None:
                self.get_logger().error(f'❌ 无法读取图像文件: {file_path}')
                self.send_feature_extraction_error(client_id, file_id, '无法读取图像文件')
                return
            
            # 转换为ROS图像消息
            try:
                image_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            except Exception as e:
                self.get_logger().error(f'❌ 图像转换失败: {e}')
                self.send_feature_extraction_error(client_id, file_id, '图像格式转换失败')
                return
            
            # 创建服务请求
            request = FeatureExtraction.Request()
            request.image = image_msg
            request.person_name = Path(file_name).stem  # 使用文件名（不含扩展名）作为人物名称
            request.save_to_file = True
            request.output_path = str(self.file_save_dir)
            
            self.get_logger().info(f'📤 发送特征提取服务请求 - 文件: {file_name}')
            
            # 异步调用服务
            future = self.feature_extraction_client.call_async(request)
            future.add_done_callback(
                lambda fut: self.handle_feature_extraction_response(fut, client_id, file_id, file_name, is_video=False)
            )
            
        except Exception as e:
            self.get_logger().error(f'❌ 图片特征提取失败: {e}')
            self.send_feature_extraction_error(client_id, file_id, f'图片处理失败: {str(e)}')
    
    def extract_features_from_video(self, file_path, file_name, client_id, file_id):
        """从视频提取特征"""
        try:
            self.get_logger().info(f'🎥 开始视频特征提取 - 文件: {file_name}')
            
            # 检查视频文件是否存在
            if not Path(file_path).exists():
                self.get_logger().error(f'❌ 视频文件不存在: {file_path}')
                self.send_feature_extraction_error(client_id, file_id, '视频文件不存在')
                return
            
            # 创建服务请求 - 使用特殊前缀标识视频处理
            request = FeatureExtraction.Request()
            request.person_name = f"VIDEO:{file_path}"  # 使用VIDEO:前缀标识视频处理
            request.save_to_file = True
            request.output_path = str(self.file_save_dir)
            
            # 创建一个空的图像消息（视频处理时不使用）
            request.image = Image()
            
            self.get_logger().info(f'📤 发送视频特征提取服务请求 - 文件: {file_name}，路径: {file_path}')
            
            # 异步调用服务
            future = self.feature_extraction_client.call_async(request)
            future.add_done_callback(
                lambda fut: self.handle_feature_extraction_response(fut, client_id, file_id, file_name, is_video=True)
            )
            
        except Exception as e:
            self.get_logger().error(f'❌ 视频特征提取失败: {e}')
            self.send_feature_extraction_error(client_id, file_id, f'视频处理失败: {str(e)}')
    
    def handle_feature_extraction_response(self, future, client_id, file_id, file_name, is_video=False):
        """处理特征提取服务响应"""
        try:
            response = future.result()
            
            if response.success:
                file_type = "视频" if is_video else "图片"
                self.get_logger().info(f'✅ {file_type}特征提取成功 - 文件: {file_name}')
                
                # 构建返回数据
                result_data = {
                    'status': 'success',
                    'message': response.message,
                    'person_count': response.person_count,
                    'body_ratios': list(response.body_ratios),
                    'shirt_color': list(response.shirt_color),
                    'pants_color': list(response.pants_color),
                    'result_image_path': response.result_image_path,
                    'feature_data_path': response.feature_data_path,
                    'file_name': file_name,
                    'file_type': 'video' if is_video else 'image'
                }
                
                # 对于视频处理，尝试添加视频文件路径
                if is_video:
                    # 尝试从响应中获取视频路径，如果没有，则尝试根据命名模式推断
                    if hasattr(response, 'result_video_path') and response.result_video_path:
                        result_data['result_video_path'] = response.result_video_path
                        self.get_logger().info(f'📹 包含视频路径: {response.result_video_path}')
                    else:
                        # 尝试根据图像路径推断视频路径
                        if response.result_image_path:
                            image_path = Path(response.result_image_path)
                            # 将 _video_result.jpg 替换为 _video_result.mp4
                            video_name = image_path.stem.replace('_video_result', '_video_result') + '.mp4'
                            potential_video_path = image_path.parent / video_name
                            
                            if potential_video_path.exists():
                                result_data['result_video_path'] = str(potential_video_path)
                                video_size = potential_video_path.stat().st_size / (1024 * 1024)  # MB
                                self.get_logger().info(f'📹 推断视频路径: {potential_video_path} (大小: {video_size:.2f}MB)')
                            else:
                                self.get_logger().warn(f'⚠️ 未找到期望的视频文件: {potential_video_path}')
                                
                        # 添加视频处理的额外信息
                        result_data['processing_info'] = {
                            'type': 'video_processing',
                            'has_annotated_video': 'result_video_path' in result_data,
                            'has_result_image': bool(response.result_image_path),
                            'has_feature_data': bool(response.feature_data_path)
                        }
                
                # 发送成功结果给客户端
                self.send_feature_extraction_result(client_id, file_id, result_data)
                
            else:
                file_type = "视频" if is_video else "图片"
                self.get_logger().error(f'❌ {file_type}特征提取失败 - 文件: {file_name}, 错误: {response.message}')
                self.send_feature_extraction_error(client_id, file_id, response.message)
                
        except Exception as e:
            file_type = "视频" if is_video else "图片"
            self.get_logger().error(f'❌ {file_type}特征提取服务调用失败: {e}')
            self.send_feature_extraction_error(client_id, file_id, f'服务调用失败: {str(e)}')
    
    def send_feature_extraction_result(self, client_id, file_id, result_data):
        """发送特征提取结果给客户端"""
        message = {
            'type': 'feature_extraction_result',
            'file_id': file_id,
            'client_id': client_id,
            'data': result_data,
            'timestamp': int(time.time() * 1000)
        }
        
        # 发送给WebSocket服务器，由服务器转发给客户端
        if self.send_ws_message(message):
            self.get_logger().info(f'📊 特征提取结果已发送 - 客户端: {client_id}, 文件: {file_id}')
        else:
            self.get_logger().warning(f'⚠️ 特征提取结果发送失败 - 客户端: {client_id}')
    
    def send_feature_extraction_error(self, client_id, file_id, error_message):
        """发送特征提取错误给客户端"""
        message = {
            'type': 'feature_extraction_error',
            'file_id': file_id,
            'client_id': client_id,
            'error': error_message,
            'timestamp': int(time.time() * 1000)
        }
        
        # 发送给WebSocket服务器，由服务器转发给客户端
        if self.send_ws_message(message):
            self.get_logger().error(f'❌ 特征提取错误已发送 - 客户端: {client_id}, 错误: {error_message}')
        else:
            self.get_logger().error(f'❌ 特征提取错误发送失败 - 客户端: {client_id}, 错误: {error_message}')
    
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
            
        # 取消特征服务检查定时器
        if hasattr(self, 'check_feature_service_timer'):
            self.check_feature_service_timer.cancel()
            
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