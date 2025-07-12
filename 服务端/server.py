#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
机器人伴侣WebSocket服务端
=======================

功能说明：
    - 提供WebSocket服务，连接微信小程序客户端和ROS2机器人节点
    - 实现消息路由，在客户端和机器人之间转发消息
    - 管理连接状态，处理心跳和重连
    - 转发视频流、控制命令、状态更新等
    - 提供HTTP文件上传接口

架构设计：
    - 客户端连接：/ws/companion/{client_id}
    - 机器人连接：/ws/ros2_bridge/{robot_id}
    - 文件上传：/api/upload/{client_id}
    - 消息队列：处理并发消息
    - 连接池：管理多个客户端和机器人

作者：AI Assistant
日期：2025
"""

import asyncio
import json
import logging
import time
import uuid
import os
import base64
from datetime import datetime
from typing import Dict, Set, Optional, Any, Union
from dataclasses import dataclass, field
from collections import defaultdict
from pathlib import Path

# 兼容不同版本的 websockets 库
try:
    # 尝试新版本的导入方式
    from websockets.server import WebSocketServerProtocol
    from websockets.legacy.server import WebSocketServerProtocol as LegacyWebSocketServerProtocol

    WebSocketType = Union[WebSocketServerProtocol, LegacyWebSocketServerProtocol]
except ImportError:
    try:
        # 尝试旧版本的导入方式
        from websockets import WebSocketServerProtocol

        WebSocketType = WebSocketServerProtocol
    except ImportError:
        # 使用通用类型
        import websockets

        WebSocketType = Any

import websockets
from websockets.exceptions import ConnectionClosed

# HTTP服务器相关
try:
    from aiohttp import web, MultipartReader
    from aiohttp.web_runner import GracefulExit
    AIOHTTP_AVAILABLE = True
except ImportError:
    print("警告: aiohttp未安装，文件上传功能将不可用")
    print("请安装: pip install aiohttp")
    AIOHTTP_AVAILABLE = False

# 配置日志
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger('companion_server')


@dataclass
class ClientConnection:
    """客户端连接信息"""
    websocket: WebSocketType  # 使用兼容的类型
    client_id: str
    client_type: str  # 'companion' or 'ros2_bridge'
    robot_id: Optional[str] = None
    connected_at: float = field(default_factory=time.time)
    last_heartbeat: float = field(default_factory=time.time)
    capabilities: Dict[str, Any] = field(default_factory=dict)


@dataclass
class RobotInfo:
    """机器人信息"""
    robot_id: str
    bridge_connection: Optional[ClientConnection] = None
    companion_clients: Set[str] = field(default_factory=set)
    status: Dict[str, Any] = field(default_factory=lambda: {
        'connected': False,
        'battery_level': 0,
        'signal_strength': 'unknown',
        'mode': 'idle',
        'last_update': 0
    })
    video_streaming: bool = False
    last_video_frame: Optional[Dict] = None


class CompanionServer:
    """机器人伴侣WebSocket服务器"""

    def __init__(self, host: str = '0.0.0.0', port: int = 1234, http_port: int = 8080):
        self.host = host
        self.port = port
        self.http_port = http_port

        # 连接管理
        self.connections: Dict[str, ClientConnection] = {}
        self.robots: Dict[str, RobotInfo] = {}

        # 消息队列
        self.message_queues: Dict[str, asyncio.Queue] = defaultdict(lambda: asyncio.Queue(maxsize=1000))

        # 文件上传管理
        self.upload_dir = Path("uploads")
        self.upload_dir.mkdir(exist_ok=True)
        self.max_file_size = 10 * 1024 * 1024  # 10MB
        
        # HTTP应用
        self.http_app = None
        self.http_runner = None

        # 统计信息
        self.stats = {
            'total_connections': 0,
            'messages_sent': 0,
            'messages_received': 0,
            'video_frames_forwarded': 0,
            'commands_forwarded': 0,
            'files_uploaded': 0,
            'errors': 0,
            'start_time': time.time()
        }

        # 心跳配置
        self.heartbeat_interval = 30  # 秒
        self.heartbeat_timeout = 60  # 秒

        logger.info(f"🚀 服务器初始化 - WebSocket: {host}:{port}, HTTP: {host}:{http_port}")

    def get_websockets_version(self):
        """获取websockets库版本"""
        try:
            import websockets
            version_str = getattr(websockets, '__version__', '0.0')
            version_parts = version_str.split('.')
            major = int(version_parts[0]) if len(version_parts) > 0 else 0
            minor = int(version_parts[1]) if len(version_parts) > 1 else 0
            return (major, minor)
        except:
            return (0, 0)

    async def start_http_server(self):
        """启动HTTP服务器"""
        if not AIOHTTP_AVAILABLE:
            return

        self.http_app = web.Application(client_max_size=self.max_file_size)
        
        # 添加路由
        self.http_app.router.add_post('/api/upload/{client_id}', self.handle_file_upload)
        self.http_app.router.add_get('/api/health', self.handle_health_check)
        self.http_app.router.add_options('/api/upload/{client_id}', self.handle_cors_preflight)
        
        # 添加CORS中间件
        self.http_app.middlewares.append(self.cors_middleware)
        
        # 启动HTTP服务器
        self.http_runner = web.AppRunner(self.http_app)
        await self.http_runner.setup()
        
        site = web.TCPSite(self.http_runner, self.host, self.http_port)
        await site.start()
        
        logger.info(f"✅ HTTP服务器已启动 - http://{self.host}:{self.http_port}")

    @web.middleware
    async def cors_middleware(self, request, handler):
        """CORS中间件"""
        # 处理预检请求
        if request.method == 'OPTIONS':
            return web.Response(
                headers={
                    'Access-Control-Allow-Origin': '*',
                    'Access-Control-Allow-Methods': 'GET, POST, OPTIONS',
                    'Access-Control-Allow-Headers': 'Content-Type',
                    'Access-Control-Max-Age': '86400'
                }
            )
        
        # 处理实际请求
        response = await handler(request)
        response.headers['Access-Control-Allow-Origin'] = '*'
        response.headers['Access-Control-Allow-Methods'] = 'GET, POST, OPTIONS'
        response.headers['Access-Control-Allow-Headers'] = 'Content-Type'
        
        return response

    async def handle_cors_preflight(self, request):
        """处理CORS预检请求"""
        return web.Response(
            headers={
                'Access-Control-Allow-Origin': '*',
                'Access-Control-Allow-Methods': 'GET, POST, OPTIONS',
                'Access-Control-Allow-Headers': 'Content-Type',
                'Access-Control-Max-Age': '86400'
            }
        )

    async def handle_health_check(self, request):
        """健康检查接口"""
        return web.json_response({
            'status': 'ok',
            'timestamp': int(time.time() * 1000),
            'connections': len(self.connections),
            'robots': len(self.robots)
        })

    async def handle_file_upload(self, request):
        """处理文件上传"""
        client_id = request.match_info['client_id']
        
        try:
            # 检查客户端是否连接
            if client_id not in self.connections:
                return web.json_response(
                    {'error': 'Client not connected', 'code': 'CLIENT_NOT_CONNECTED'},
                    status=400
                )

            # 检查Content-Type
            if not request.content_type.startswith('multipart/form-data'):
                return web.json_response(
                    {'error': 'Invalid content type', 'code': 'INVALID_CONTENT_TYPE'},
                    status=400
                )

            # 读取multipart数据
            reader = await request.multipart()
            file_data = None
            file_name = None
            file_type = None
            custom_filename = None

            async for part in reader:
                if part.name == 'file':
                    file_name = part.filename or f"upload_{int(time.time())}"
                    file_type = part.headers.get('Content-Type', 'application/octet-stream')
                    file_data = await part.read()
                    
                    # 检查文件大小
                    if len(file_data) > self.max_file_size:
                        return web.json_response(
                            {'error': 'File too large', 'code': 'FILE_TOO_LARGE'},
                            status=413
                        )
                    
                elif part.name == 'custom_filename':
                    # 读取自定义文件名
                    custom_filename_data = await part.read()
                    custom_filename = custom_filename_data.decode('utf-8').strip()
                    if custom_filename:
                        logger.info(f"📝 收到自定义文件名: {custom_filename}")

            # 如果有自定义文件名，使用自定义文件名
            if custom_filename:
                display_file_name = custom_filename
            else:
                display_file_name = file_name

            if not file_data:
                return web.json_response(
                    {'error': 'No file uploaded', 'code': 'NO_FILE'},
                    status=400
                )

            # 保存文件（使用自定义文件名）
            file_id = f"{client_id}_{int(time.time() * 1000)}_{display_file_name}"
            file_path = self.upload_dir / file_id
            
            with open(file_path, 'wb') as f:
                f.write(file_data)

            # 更新统计
            self.stats['files_uploaded'] += 1

            logger.info(f"📁 文件上传成功 - 客户端: {client_id}, 原文件名: {file_name}, 显示名称: {display_file_name}, 大小: {len(file_data)}字节")

            # 通知WebSocket客户端上传完成
            if client_id in self.connections:
                await self.send_message(self.connections[client_id].websocket, {
                    'type': 'file_upload_success',
                    'file_id': file_id,
                    'file_name': display_file_name,
                    'file_size': len(file_data),
                    'file_type': file_type,
                    'upload_time': int(time.time() * 1000)
                })

            # 转发文件给关联的ROS2机器人节点
            client_connection = self.connections.get(client_id)
            if client_connection and client_connection.robot_id:
                await self.forward_file_to_robot(client_connection.robot_id, {
                    'file_id': file_id,
                    'file_name': display_file_name,
                    'file_data': file_data,
                    'file_type': file_type,
                    'file_size': len(file_data),
                    'upload_time': int(time.time() * 1000),
                    'client_id': client_id
                })

            return web.json_response({
                'success': True,
                'file_id': file_id,
                'file_name': display_file_name,
                'file_size': len(file_data),
                'file_type': file_type,
                'upload_time': int(time.time() * 1000)
            })

        except Exception as e:
            logger.error(f"❌ 文件上传失败: {e}", exc_info=True)
            self.stats['errors'] += 1
            return web.json_response(
                {'error': 'Upload failed', 'code': 'UPLOAD_FAILED', 'message': str(e)},
                status=500
            )

    async def start(self):
        """启动服务器"""
        logger.info("🌟 正在启动机器人伴侣服务器...")

        # 启动心跳检查任务
        asyncio.create_task(self.heartbeat_checker())

        # 启动统计信息记录任务
        asyncio.create_task(self.stats_logger())

        # 启动HTTP服务器
        if AIOHTTP_AVAILABLE:
            await self.start_http_server()
        else:
            logger.warning("⚠️ HTTP服务器未启动，文件上传功能不可用")

        # 检测 websockets 版本并使用相应的启动方式
        version = self.get_websockets_version()
        logger.info(f"检测到 websockets 版本: {version[0]}.{version[1]}")

        if version >= (11, 0):
            # 新版本需要使用不同的方式
            logger.info("使用新版本 websockets 服务器启动方式")
            await self._start_new_version()
        else:
            # 旧版本的启动方式
            logger.info("使用旧版本 websockets 服务器启动方式")
            await self._start_legacy_version()

    async def _start_new_version(self):
        """新版本 websockets 的启动方式"""
        import websockets

        # 对于新版本，使用process_request来获取路径信息
        async def process_request(connection, request):
            """处理请求并提取路径信息"""
            # 将路径信息存储在连接对象上
            connection._custom_path = request.path
            return None  # 继续正常的握手过程

        # 创建处理函数包装器
        async def handler(websocket):
            # 从连接对象获取路径
            path = getattr(websocket, '_custom_path', getattr(websocket, 'path', '/'))
            await self.handle_connection(websocket, path)

        # 使用新版本的服务器，优化心跳参数与客户端协调
        async with websockets.serve(
                handler,
                self.host,
                self.port,
                process_request=process_request,
                ping_interval=22,  # 稍大于客户端的18秒，避免冲突
                ping_timeout=12    # 给予更多时间处理
        ):
            logger.info(f"✅ 服务器已启动 - ws://{self.host}:{self.port}")
            await asyncio.Future()  # 永久运行

    async def _start_legacy_version(self):
        """旧版本 websockets 的启动方式"""
        # 旧版本的启动方式，优化心跳参数与客户端协调
        async with websockets.serve(
                self.handle_connection,
                self.host,
                self.port,
                ping_interval=22,  # 稍大于客户端的18秒，避免冲突
                ping_timeout=12    # 给予更多时间处理
        ):
            logger.info(f"✅ 服务器已启动 - ws://{self.host}:{self.port}")
            await asyncio.Future()  # 永久运行

    async def handle_connection(self, websocket, path: str = None):
        """处理新的WebSocket连接"""
        client_id = None
        connection_type = None

        try:
            # 兼容新旧版本的 websockets 库获取路径
            if path is None:
                # 尝试多种方式获取路径
                path = getattr(websocket, 'path', None)
                if path is None:
                    path = getattr(websocket, '_custom_path', None)
                if path is None:
                    # 如果还是获取不到，尝试从request_headers获取
                    try:
                        if hasattr(websocket, 'request_headers'):
                            # 从请求头获取路径信息（备用方案）
                            path = '/'
                        else:
                            path = '/'
                    except:
                        path = '/'

            # 解析连接路径
            parts = path.strip('/').split('/')

            if len(parts) >= 3:
                if parts[0] == 'ws':
                    if parts[1] == 'companion' and len(parts) == 3:
                        # 小程序客户端连接
                        connection_type = 'companion'
                        client_id = parts[2]
                    elif parts[1] == 'ros2_bridge' and len(parts) == 3:
                        # ROS2节点连接
                        connection_type = 'ros2_bridge'
                        robot_id = parts[2]
                        client_id = f"bridge_{robot_id}"

            if not client_id or not connection_type:
                logger.warning(f"❌ 无效的连接路径: {path}")
                await websocket.close(1002, "Invalid path")
                return

            # 创建连接记录
            connection = ClientConnection(
                websocket=websocket,
                client_id=client_id,
                client_type=connection_type
            )

            # 添加到连接池
            self.connections[client_id] = connection
            self.stats['total_connections'] += 1

            # 获取远程地址（兼容不同版本）
            try:
                remote_address = getattr(websocket, 'remote_address', 'unknown')
                if remote_address == 'unknown':
                    # 尝试其他方式获取远程地址
                    if hasattr(websocket, 'transport') and hasattr(websocket.transport, 'get_extra_info'):
                        remote_address = websocket.transport.get_extra_info('peername', 'unknown')
            except:
                remote_address = 'unknown'

            logger.info(f"🔗 新连接 - 类型: {connection_type}, ID: {client_id}, 地址: {remote_address}")

            # 处理消息
            if connection_type == 'companion':
                await self.handle_companion_client(connection)
            else:  # ros2_bridge
                await self.handle_ros2_bridge(connection, robot_id)

        except ConnectionClosed:
            logger.info(f"🔌 连接关闭 - ID: {client_id}")
        except Exception as e:
            logger.error(f"❌ 连接处理错误: {e}", exc_info=True)
            self.stats['errors'] += 1
        finally:
            # 清理连接
            if client_id and client_id in self.connections:
                await self.cleanup_connection(client_id)

    async def handle_companion_client(self, connection: ClientConnection):
        """处理小程序客户端连接"""
        try:
            # 发送欢迎消息
            await self.send_message(connection.websocket, {
                'type': 'server_welcome',
                'server_version': '2.0.0',
                'timestamp': int(time.time() * 1000)
            })

            # 消息处理循环
            async for message in connection.websocket:
                try:
                    data = json.loads(message)
                    self.stats['messages_received'] += 1

                    await self.handle_companion_message(connection, data)

                except json.JSONDecodeError:
                    logger.error(f"❌ 消息解析失败: {message}")
                except Exception as e:
                    logger.error(f"❌ 消息处理错误: {e}")

        except ConnectionClosed:
            pass

    async def handle_companion_message(self, connection: ClientConnection, data: Dict):
        """处理小程序客户端消息"""
        message_type = data.get('type', '')

        logger.debug(f"📱 小程序消息 - 类型: {message_type}, 客户端: {connection.client_id}")

        if message_type == 'client_init':
            # 客户端初始化
            robot_id = data.get('robot_id')
            if robot_id:
                connection.robot_id = robot_id

                # 获取或创建机器人信息
                if robot_id not in self.robots:
                    self.robots[robot_id] = RobotInfo(robot_id=robot_id)

                # 添加客户端到机器人的客户端列表
                self.robots[robot_id].companion_clients.add(connection.client_id)

                # 更新客户端能力
                connection.capabilities = data.get('capabilities', {})

                logger.info(f"👤 客户端初始化 - 客户端: {connection.client_id}, 机器人: {robot_id}")

                # 发送机器人连接状态
                await self.send_robot_connection_status(connection, robot_id)

                # 如果机器人已连接，请求初始状态
                if self.robots[robot_id].bridge_connection:
                    await self.request_robot_status(robot_id)

        elif message_type == 'companion_command':
            # 转发控制命令到机器人
            robot_id = data.get('robot_id') or connection.robot_id
            if robot_id:
                await self.forward_to_robot(robot_id, data)

        elif message_type == 'client_quality_request':
            # 转发视频质量调整请求
            robot_id = data.get('robot_id') or connection.robot_id
            if robot_id:
                # 先确认收到请求
                await self.send_message(connection.websocket, {
                    'type': 'quality_request_received',
                    'preset': data.get('preset', 'medium'),
                    'timestamp': int(time.time() * 1000)
                })

                # 转发到机器人
                await self.forward_to_robot(robot_id, {
                    'type': 'quality_adjustment',
                    'preset': data.get('preset', 'medium'),
                    'immediate': True,
                    'timestamp': data.get('timestamp')
                })

        elif message_type == 'get_robot_status':
            # 请求机器人状态
            robot_id = data.get('robot_id') or connection.robot_id
            if robot_id:
                await self.request_robot_status(robot_id)

        elif message_type == 'feature_extraction_request':
            # 特征提取请求
            robot_id = data.get('robot_id') or connection.robot_id
            if robot_id:
                await self.handle_feature_extraction_request(connection, data)

        elif message_type == 'ping':
            # 心跳消息
            connection.last_heartbeat = time.time()
            await self.send_message(connection.websocket, {
                'type': 'pong',
                'echo_timestamp': data.get('timestamp'),
                'server_timestamp': int(time.time() * 1000)
            })

        elif message_type == 'client_network_status':
            # 客户端网络状态报告
            logger.info(f"📊 客户端网络状态 - {connection.client_id}: {data.get('status', {})}")

        elif message_type == 'processed_image_result':
            # 处理后图片结果
            await self.handle_processed_image_result(connection, data)
            
        elif message_type == 'rfid_command':
            # RFID控制命令（开始/停止盘存等）
            robot_id = data.get('robot_id') or connection.robot_id
            if robot_id:
                # 转发RFID命令到机器人
                command = data.get('command', '')
                params = data.get('params', {})
                logger.info(f"📡🔥 收到RFID命令 - 客户端: {connection.client_id}, 机器人: {robot_id}, 命令: {command}")
                
                # 构建转发给机器人的命令消息
                robot_command = {
                    'type': 'companion_command',
                    'command': command,
                    'params': params,
                    'client_id': connection.client_id,
                    'timestamp': data.get('timestamp', int(time.time() * 1000))
                }
                
                await self.forward_to_robot(robot_id, robot_command)

    async def handle_ros2_bridge(self, connection: ClientConnection, robot_id: str):
        """处理ROS2桥接节点连接"""
        try:
            # 记录机器人连接
            connection.robot_id = robot_id

            if robot_id not in self.robots:
                self.robots[robot_id] = RobotInfo(robot_id=robot_id)

            self.robots[robot_id].bridge_connection = connection
            self.robots[robot_id].status['connected'] = True

            logger.info(f"🤖 机器人连接 - ID: {robot_id}")

            # 通知所有关联的客户端
            await self.notify_robot_connection_change(robot_id, True)

            # 消息处理循环
            async for message in connection.websocket:
                try:
                    data = json.loads(message)
                    self.stats['messages_received'] += 1

                    await self.handle_robot_message(connection, data)

                except json.JSONDecodeError:
                    logger.error(f"❌ 消息解析失败: {message}")
                except Exception as e:
                    logger.error(f"❌ 消息处理错误: {e}")

        except ConnectionClosed:
            pass

    async def handle_robot_message(self, connection: ClientConnection, data: Dict):
        """处理机器人消息"""
        message_type = data.get('type', '')
        robot_id = data.get('robot_id') or connection.robot_id

        logger.debug(f"🤖 机器人消息 - 类型: {message_type}, 机器人: {robot_id}")

        if message_type == 'robot_init':
            # 机器人初始化
            connection.capabilities = data.get('capabilities', {})
            logger.info(f"🔧 机器人初始化 - ID: {robot_id}, 能力: {connection.capabilities}")

        elif message_type == 'video_frame':
            # 转发视频帧到所有客户端
            await self.forward_video_frame(robot_id, data)
            self.stats['video_frames_forwarded'] += 1

        elif message_type == 'robot_status_update':
            # 更新机器人状态
            if robot_id in self.robots:
                status_data = data.get('data', {})
                self.robots[robot_id].status.update(status_data)
                self.robots[robot_id].status['last_update'] = time.time()

            # 转发状态到所有客户端
            await self.forward_to_companions(robot_id, data)

        elif message_type == 'command_response':
            # 转发命令响应到客户端
            await self.forward_to_companions(robot_id, data)
            self.stats['commands_forwarded'] += 1

        elif message_type == 'quality_adjustment_result':
            # 转发质量调整结果
            actual_resolution = data.get('actual_resolution', '480x360')
            actual_fps = data.get('actual_fps', 10)
            preset = data.get('preset', 'medium')

            quality_update = {
                'type': 'video_quality_update',
                'preset': preset,
                'resolution': actual_resolution,
                'fps': actual_fps,
                'timestamp': data.get('timestamp')
            }

            await self.forward_to_companions(robot_id, quality_update)

        elif message_type == 'feature_extraction_result':
            # 特征提取结果
            await self.handle_feature_extraction_result(connection, data)

        elif message_type == 'file_save_result':
            # 文件保存结果
            await self.handle_file_save_result(connection, data)

        elif message_type == 'feature_extraction_error':
            # 特征提取错误
            await self.handle_feature_extraction_error(connection, data)

        elif message_type == 'processed_image_result':
            # 处理后图片结果
            await self.handle_processed_image_result(connection, data)

        elif message_type == 'tracking_data':
            # 转发跟踪数据到所有客户端
            await self.forward_to_companions(robot_id, data)
            logger.debug(f"📊 转发跟踪数据 - 机器人: {robot_id}, 目标检测: {data.get('data', {}).get('target_detected', False)}")
            
        elif message_type == 'detailed_tracking_data':
            # 转发详细跟踪数据到所有客户端
            await self.forward_to_companions(robot_id, data)
            logger.info(f"📈 转发详细跟踪数据 - 机器人: {robot_id}, 数据: {json.dumps(data, indent=2)}")
            logger.debug(f"📊 详细跟踪数据 - 机器人: {robot_id}, 轨迹数: {data.get('data', {}).get('total_tracks', 0)}, 目标检测: {data.get('data', {}).get('target_detected', False)}")
            
        elif message_type == 'rfid_tags_data':
            # 转发RFID标签数据到所有客户端
            await self.forward_to_companions(robot_id, data)
            tags_count = data.get('data', {}).get('total_tags', 0)
            reads_count = data.get('data', {}).get('total_reads', 0)
            logger.info(f"📡 转发RFID标签数据 - 机器人: {robot_id}, 标签数: {tags_count}, 读取次数: {reads_count}")
            
        elif message_type == 'rfid_status_update':
            # 转发RFID状态更新到所有客户端
            await self.forward_to_companions(robot_id, data)
            status_data = data.get('data', {})
            connected = status_data.get('connected', False)
            inventory_active = status_data.get('inventory_active', False)
            logger.info(f"📊 转发RFID状态更新 - 机器人: {robot_id}, 连接: {connected}, 盘存活跃: {inventory_active}")
            
        elif message_type == 'rfid_tag_detected':
            # 转发单个RFID标签检测到所有客户端
            await self.forward_to_companions(robot_id, data)
            tag_data = data.get('data', {})
            epc = tag_data.get('epc', 'unknown')
            rssi = tag_data.get('rssi_dbm', 0)
            logger.info(f"🆕 转发新RFID标签检测 - 机器人: {robot_id}, EPC: {epc}, RSSI: {rssi}dBm")
            
        elif message_type == 'rfid_command_response':
            # 转发RFID命令响应到所有客户端
            await self.forward_to_companions(robot_id, data)
            command = data.get('command', 'unknown')
            status = data.get('status', 'unknown')
            logger.info(f"⚡ 转发RFID命令响应 - 机器人: {robot_id}, 命令: {command}, 状态: {status}")
            
        elif message_type == 'heartbeat':
            # 机器人心跳
            connection.last_heartbeat = time.time()
            await self.send_message(connection.websocket, {
                'type': 'heartbeat_ack',
                'timestamp': int(time.time() * 1000)
            })

    async def forward_video_frame(self, robot_id: str, frame_data: Dict):
        """转发视频帧到客户端"""
        if robot_id not in self.robots:
            return

        robot = self.robots[robot_id]
        robot.last_video_frame = frame_data
        robot.video_streaming = True

        # 添加服务器时间戳
        frame_data['server_timestamp'] = int(time.time() * 1000)

        # 转发到所有连接的客户端
        tasks = []
        for client_id in robot.companion_clients.copy():
            if client_id in self.connections:
                client = self.connections[client_id]
                if client.capabilities.get('video_receive', True):
                    tasks.append(self.send_message(client.websocket, frame_data))

        if tasks:
            await asyncio.gather(*tasks, return_exceptions=True)

    async def forward_to_robot(self, robot_id: str, message: Dict):
        """转发消息到机器人"""
        if robot_id not in self.robots:
            logger.warning(f"⚠️ 机器人未连接: {robot_id}")
            return

        robot = self.robots[robot_id]
        if robot.bridge_connection and robot.bridge_connection.client_id in self.connections:
            bridge = robot.bridge_connection
            await self.send_message(bridge.websocket, message)
            logger.info(f"➡️ 转发到机器人 - 类型: {message.get('type')}, 机器人: {robot_id}")
        else:
            logger.warning(f"⚠️ 机器人桥接未连接: {robot_id}")

    async def forward_to_companions(self, robot_id: str, message: Dict):
        """转发消息到所有关联的客户端"""
        if robot_id not in self.robots:
            return

        robot = self.robots[robot_id]
        tasks = []

        for client_id in robot.companion_clients.copy():
            if client_id in self.connections:
                tasks.append(self.send_message(self.connections[client_id].websocket, message))

        if tasks:
            await asyncio.gather(*tasks, return_exceptions=True)

    async def forward_file_to_robot(self, robot_id: str, file_info: Dict):
        """转发文件到机器人节点"""
        if robot_id not in self.robots:
            logger.warning(f"⚠️ 机器人未连接，无法转发文件: {robot_id}")
            return

        robot = self.robots[robot_id]
        if not robot.bridge_connection or robot.bridge_connection.client_id not in self.connections:
            logger.warning(f"⚠️ 机器人桥接未连接，无法转发文件: {robot_id}")
            return

        try:
            # 将文件数据编码为base64以便通过WebSocket传输
            file_data_b64 = base64.b64encode(file_info['file_data']).decode('utf-8')
            
            # 准备转发消息
            forward_message = {
                'type': 'file_upload_forward',
                'robot_id': robot_id,
                'file_id': file_info['file_id'],
                'file_name': file_info['file_name'],
                'file_data_base64': file_data_b64,
                'file_type': file_info['file_type'],
                'file_size': file_info['file_size'],
                'upload_time': file_info['upload_time'],
                'client_id': file_info['client_id'],
                'timestamp': int(time.time() * 1000)
            }
            
            # 发送给机器人节点
            bridge = robot.bridge_connection
            await self.send_message(bridge.websocket, forward_message)
            
            logger.info(f"📤 文件转发成功 - 机器人: {robot_id}, 文件: {file_info['file_name']}, 大小: {file_info['file_size']}字节")
            
        except Exception as e:
            logger.error(f"❌ 文件转发失败: {e}")
            # 通知客户端转发失败
            if file_info['client_id'] in self.connections:
                await self.send_message(self.connections[file_info['client_id']].websocket, {
                    'type': 'file_forward_error',
                    'file_id': file_info['file_id'],
                    'error': f'转发到机器人失败: {str(e)}',
                    'timestamp': int(time.time() * 1000)
                })

    async def notify_robot_connection_change(self, robot_id: str, connected: bool):
        """通知客户端机器人连接状态变化"""
        notification = {
            'type': 'robot_connection_status',
            'robot_id': robot_id,
            'connected': connected,
            'timestamp': int(time.time() * 1000)
        }

        await self.forward_to_companions(robot_id, notification)

    async def send_robot_connection_status(self, connection: ClientConnection, robot_id: str):
        """发送机器人连接状态到客户端"""
        connected = False
        if robot_id in self.robots and self.robots[robot_id].bridge_connection:
            connected = True

        await self.send_message(connection.websocket, {
            'type': 'robot_connection_status',
            'robot_id': robot_id,
            'connected': connected,
            'timestamp': int(time.time() * 1000)
        })

    async def request_robot_status(self, robot_id: str):
        """请求机器人状态"""
        await self.forward_to_robot(robot_id, {
            'type': 'request_status',
            'timestamp': int(time.time() * 1000)
        })

    async def handle_feature_extraction_request(self, connection: ClientConnection, data: Dict):
        """处理特征提取请求"""
        robot_id = data.get('robot_id')
        file_id = data.get('file_id')
        
        if not robot_id or not file_id:
            await self.send_message(connection.websocket, {
                'type': 'feature_extraction_error',
                'error': 'Missing robot_id or file_id',
                'timestamp': int(time.time() * 1000)
            })
            return

        # 检查文件是否存在
        file_path = self.upload_dir / file_id
        if not file_path.exists():
            await self.send_message(connection.websocket, {
                'type': 'feature_extraction_error',
                'error': 'File not found',
                'timestamp': int(time.time() * 1000)
            })
            return

        # 转发到机器人进行特征提取
        await self.forward_to_robot(robot_id, {
            'type': 'feature_extraction_request',
            'file_id': file_id,
            'file_path': str(file_path),
            'client_id': connection.client_id,
            'extract_clothing_colors': data.get('extract_clothing_colors', True),
            'extract_body_proportions': data.get('extract_body_proportions', True),
            'timestamp': data.get('timestamp', int(time.time() * 1000))
        })

        logger.info(f"🔍 转发特征提取请求 - 客户端: {connection.client_id}, 机器人: {robot_id}, 文件: {file_id}")

    async def handle_feature_extraction_result(self, connection: ClientConnection, data: Dict):
        """处理特征提取结果"""
        client_id = data.get('client_id')
        robot_id = connection.robot_id
        
        if not client_id or client_id not in self.connections:
            logger.warning(f"⚠️ 特征提取结果无法转发：客户端不存在 - {client_id}")
            return

        # 转发完整的结果数据到对应的客户端（保持所有兼容字段）
        client_connection = self.connections[client_id]
        
        # 从ROS2节点发送的数据结构中提取实际数据
        # ROS2节点发送格式：{type: 'feature_extraction_result', data: {...实际数据}}
        ros_data = data.get('data', {})
        
        # 构建完整的转发消息，包含所有兼容字段
        forward_message = {
            'type': 'feature_extraction_result',
            'status': ros_data.get('status', 'success'),
            'confidence': ros_data.get('confidence', 0),
            'features': ros_data.get('features', {}),
            'error': ros_data.get('error'),
            'file_id': data.get('file_id'),
            'timestamp': data.get('timestamp', int(time.time() * 1000)),
            'message': ros_data.get('message', ''),
            'person_count': ros_data.get('person_count', 1),
            'file_name': ros_data.get('file_name', ''),
            'file_type': ros_data.get('file_type', 'image'),
            
            # 兼容字段 - 从WebSocket桥接节点传来的数据
            'body_ratios': ros_data.get('body_ratios', []),
            'bodyRatios': ros_data.get('body_ratios', []),  # 兼容字段
            'shirt_color': ros_data.get('shirt_color', [0, 0, 0]),
            'pants_color': ros_data.get('pants_color', [0, 0, 0]),
            'shirtColor': ros_data.get('shirt_color', [0, 0, 0]),  # 兼容字段
            'pantsColor': ros_data.get('pants_color', [0, 0, 0]),  # 兼容字段
            'body_proportions': ros_data.get('body_proportions', {}),
            'detailed_proportions': ros_data.get('detailed_proportions', []),
            'clothing_colors': ros_data.get('clothing_colors', {}),
            'result_image_path': ros_data.get('result_image_path', ''),
            'feature_data_path': ros_data.get('feature_data_path', ''),
            'resultImagePath': ros_data.get('result_image_path', ''),  # 兼容字段
            'featureDataPath': ros_data.get('feature_data_path', ''),  # 兼容字段
            'processing_info': ros_data.get('processing_info', {}),
            'result_video_path': ros_data.get('result_video_path', ''),
            # 新增：结果图片base64编码
            'result_image_base64': ros_data.get('result_image_base64', ''),
            'result_image_size_kb': ros_data.get('result_image_size_kb', 0),
            'resultImageBase64': ros_data.get('result_image_base64', ''),  # 兼容字段
        }
        
        # 调试日志
        logger.info(f"🔍 [服务端转发调试] 原始数据结构: {list(data.keys())}")
        logger.info(f"🔍 [服务端转发调试] ROS数据结构: {list(ros_data.keys())}")
        logger.info(f"🔍 [服务端转发调试] body_ratios长度: {len(forward_message['body_ratios'])}")
        logger.info(f"🔍 [服务端转发调试] shirt_color: {forward_message['shirt_color']}")
        logger.info(f"🔍 [服务端转发调试] pants_color: {forward_message['pants_color']}")
        
        await self.send_message(client_connection.websocket, forward_message)

        logger.info(f"📊 转发特征提取结果 - 机器人: {robot_id}, 客户端: {client_id}, 状态: {ros_data.get('status')}")

    async def handle_file_save_result(self, connection: ClientConnection, data: Dict):
        """处理文件保存结果"""
        client_id = data.get('client_id')
        robot_id = connection.robot_id
        
        if not client_id or client_id not in self.connections:
            logger.warning(f"⚠️ 文件保存结果无法转发：客户端不存在 - {client_id}")
            return

        # 转发结果到对应的客户端
        client_connection = self.connections[client_id]
        await self.send_message(client_connection.websocket, {
            'type': 'file_save_result',
            'status': data.get('status', 'success'),
            'error': data.get('error'),
            'file_id': data.get('file_id'),
            'timestamp': data.get('timestamp', int(time.time() * 1000))
        })

        logger.info(f"📁 转发文件保存结果 - 机器人: {robot_id}, 客户端: {client_id}, 状态: {data.get('status')}")

    async def handle_feature_extraction_error(self, connection: ClientConnection, data: Dict):
        """处理特征提取错误"""
        client_id = data.get('client_id')
        robot_id = connection.robot_id
        
        if not client_id or client_id not in self.connections:
            logger.warning(f"⚠️ 特征提取错误无法转发：客户端不存在 - {client_id}")
            return

        # 转发错误到对应的客户端
        client_connection = self.connections[client_id]
        
        # 格式化特征提取错误消息
        error_message = {
            'type': 'feature_extraction_complete',
            'file_id': data.get('file_id'),
            'status': 'error',
            'error': data.get('error', '未知错误'),
            'robot_id': robot_id,
            'timestamp': data.get('timestamp', int(time.time() * 1000))
        }
        
        await self.send_message(client_connection.websocket, error_message)
        
        logger.error(f"❌ 转发特征提取错误 - 机器人: {robot_id}, 客户端: {client_id}, 错误: {data.get('error')}")

    async def handle_processed_image_result(self, connection: ClientConnection, data: Dict):
        """处理处理后图片结果"""
        robot_id = connection.robot_id
        extraction_id = data.get('extraction_id', 'unknown')
        
        logger.info(f"📷 收到处理后图片结果 - 机器人: {robot_id}, 提取ID: {extraction_id}")
        
        # 广播给所有连接到此机器人的客户端
        if robot_id in self.robots:
            robot = self.robots[robot_id]
            
            # 构建转发消息
            forward_message = {
                'type': 'processed_image_notification',
                'extraction_id': extraction_id,
                'person_name': data.get('person_name', ''),
                'timestamp': data.get('timestamp', int(time.time() * 1000)),
                'robot_id': robot_id,
                'image_data': data.get('image_data', {}),
                'features': data.get('features', {}),
                'files': data.get('files', {}),
                'processing_info': data.get('processing_info', {})
            }
            
            # 发送给所有关联的客户端
            tasks = []
            for client_id in robot.companion_clients.copy():
                if client_id in self.connections:
                    tasks.append(self.send_message(self.connections[client_id].websocket, forward_message))
            
            if tasks:
                await asyncio.gather(*tasks, return_exceptions=True)
                logger.info(f"✅ 处理后图片结果已转发给 {len(tasks)} 个客户端")
            else:
                logger.info(f"ℹ️ 没有客户端连接到机器人 {robot_id}")
        else:
            logger.warning(f"⚠️ 机器人 {robot_id} 不存在于机器人列表中")

    async def send_message(self, websocket, message: Dict):
        """发送消息到WebSocket"""
        try:
            await websocket.send(json.dumps(message))
            self.stats['messages_sent'] += 1
        except ConnectionClosed:
            logger.debug("连接已关闭，无法发送消息")
        except Exception as e:
            logger.error(f"❌ 发送消息失败: {e}")
            self.stats['errors'] += 1

    async def cleanup_connection(self, client_id: str):
        """清理断开的连接"""
        if client_id not in self.connections:
            return

        connection = self.connections[client_id]
        logger.info(f"🧹 清理连接 - 类型: {connection.client_type}, ID: {client_id}")

        if connection.client_type == 'companion':
            # 清理小程序客户端
            if connection.robot_id and connection.robot_id in self.robots:
                self.robots[connection.robot_id].companion_clients.discard(client_id)

        elif connection.client_type == 'ros2_bridge':
            # 清理机器人连接
            if connection.robot_id and connection.robot_id in self.robots:
                robot = self.robots[connection.robot_id]
                robot.bridge_connection = None
                robot.status['connected'] = False
                robot.video_streaming = False

                # 通知所有客户端
                await self.notify_robot_connection_change(connection.robot_id, False)

        # 从连接池移除
        del self.connections[client_id]

    async def heartbeat_checker(self):
        """定期检查心跳超时的连接"""
        while True:
            try:
                current_time = time.time()
                timeout_connections = []

                for client_id, connection in self.connections.items():
                    if current_time - connection.last_heartbeat > self.heartbeat_timeout:
                        timeout_connections.append(client_id)

                for client_id in timeout_connections:
                    logger.warning(f"💔 心跳超时 - ID: {client_id}")
                    if client_id in self.connections:
                        await self.connections[client_id].websocket.close(1001, "Heartbeat timeout")

            except Exception as e:
                logger.error(f"❌ 心跳检查错误: {e}")

            await asyncio.sleep(self.heartbeat_interval)

    async def stats_logger(self):
        """定期记录统计信息"""
        while True:
            try:
                uptime = int(time.time() - self.stats['start_time'])
                hours, remainder = divmod(uptime, 3600)
                minutes, seconds = divmod(remainder, 60)

                logger.info(f"""
📊 服务器统计信息
================
运行时间: {hours}小时 {minutes}分钟 {seconds}秒
当前连接数: {len(self.connections)}
机器人数量: {len(self.robots)}
总连接数: {self.stats['total_connections']}
接收消息: {self.stats['messages_received']}
发送消息: {self.stats['messages_sent']}
视频帧转发: {self.stats['video_frames_forwarded']}
命令转发: {self.stats['commands_forwarded']}
文件上传: {self.stats['files_uploaded']}
错误数: {self.stats['errors']}
                """)

                # 详细的机器人状态
                for robot_id, robot in self.robots.items():
                    connected = "已连接" if robot.bridge_connection else "未连接"
                    clients = len(robot.companion_clients)
                    logger.info(f"🤖 机器人 {robot_id}: {connected}, 客户端数: {clients}")

            except Exception as e:
                logger.error(f"❌ 统计记录错误: {e}")

            await asyncio.sleep(300)  # 每5分钟记录一次


async def main():
    """主函数"""
    server = CompanionServer(host='172.20.39.181', port=1234, http_port=1235)
    await server.start()


if __name__ == '__main__':
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        logger.info("🛑 服务器停止")
    except Exception as e:
        logger.error(f"❌ 服务器错误: {e}", exc_info=True)