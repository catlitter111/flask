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

架构设计：
    - 客户端连接：/ws/companion/{client_id}
    - 机器人连接：/ws/ros2_bridge/{robot_id}
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
from datetime import datetime
from typing import Dict, Set, Optional, Any, Union
from dataclasses import dataclass, field
from collections import defaultdict

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

    def __init__(self, host: str = '0.0.0.0', port: int = 1234):
        self.host = host
        self.port = port

        # 连接管理
        self.connections: Dict[str, ClientConnection] = {}
        self.robots: Dict[str, RobotInfo] = {}

        # 消息队列
        self.message_queues: Dict[str, asyncio.Queue] = defaultdict(lambda: asyncio.Queue(maxsize=1000))

        # 统计信息
        self.stats = {
            'total_connections': 0,
            'messages_sent': 0,
            'messages_received': 0,
            'video_frames_forwarded': 0,
            'commands_forwarded': 0,
            'errors': 0,
            'start_time': time.time()
        }

        # 心跳配置
        self.heartbeat_interval = 30  # 秒
        self.heartbeat_timeout = 60  # 秒

        logger.info(f"🚀 服务器初始化 - {host}:{port}")

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

    async def start(self):
        """启动服务器"""
        logger.info("🌟 正在启动机器人伴侣服务器...")

        # 启动心跳检查任务
        asyncio.create_task(self.heartbeat_checker())

        # 启动统计信息记录任务
        asyncio.create_task(self.stats_logger())

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

        # 使用新版本的服务器
        async with websockets.serve(
                handler,
                self.host,
                self.port,
                process_request=process_request,
                ping_interval=20,
                ping_timeout=10
        ):
            logger.info(f"✅ 服务器已启动 - ws://{self.host}:{self.port}")
            await asyncio.Future()  # 永久运行

    async def _start_legacy_version(self):
        """旧版本 websockets 的启动方式"""
        # 旧版本的启动方式
        async with websockets.serve(
                self.handle_connection,
                self.host,
                self.port,
                ping_interval=20,
                ping_timeout=10
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
    server = CompanionServer(host='172.20.39.181', port=1234)
    await server.start()


if __name__ == '__main__':
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        logger.info("🛑 服务器停止")
    except Exception as e:
        logger.error(f"❌ 服务器错误: {e}", exc_info=True)