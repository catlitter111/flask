# server.py - 机器人伴侣服务端 (去除华为IoT)
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
import uvicorn
import json
import asyncio
import logging
import datetime
import time
import random
import base64
import os
from pathlib import Path

# 导入伴侣自适应视频管理器
from adaptive_video_manager import CompanionAdaptiveVideoManager

app = FastAPI(title="机器人伴侣服务端", version="2.0.0")

# 允许跨域请求
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# 日志配置
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger("CompanionServer")

# 全局变量
companions = {}  # 存储所有连接的伴侣机器人
clients = {}  # 存储所有连接的微信客户端
companion_to_clients = {}  # 伴侣ID到客户端ID的映射
client_to_companion = {}  # 客户端ID到伴侣ID的映射
lock = asyncio.Lock()  # 异步锁
adaptive_video_manager = None  # 自适应视频管理器实例

# 历史数据存储
daily_companion_data = {}  # 格式: {companion_id: {date: {data}}}
monthly_records = {}  # 格式: {companion_id: {year-month: [记录列表]}}

# AI聊天相关
pending_ai_requests = {}  # 格式: {request_id: {client_id, timestamp, ...}}

# 伴侣交互历史记录
interaction_history = {}  # 格式: {companion_id: [interaction_records...]}

# 伴侣跟踪历史
tracking_history = {}  # 格式: {companion_id: [tracking_records...]}


# 微信客户端WebSocket连接处理
@app.websocket("/ws/companion/{client_id}")
async def companion_websocket_endpoint(websocket: WebSocket, client_id: str):
    await websocket.accept()
    logger.info(f"微信客户端 {client_id} 已连接")

    # 注册客户端
    async with lock:
        clients[client_id] = {
            "websocket": websocket,
            "last_active": datetime.datetime.now(),
            "connection_id": f"{client_id}_{datetime.datetime.now().timestamp()}"
        }

    # 注册到自适应视频管理器
    adaptive_video_manager.register_client(client_id, websocket)

    try:
        while True:
            data = await websocket.receive_text()
            try:
                message = json.loads(data)
                message_type = message.get("type", "")

                # 更新客户端活跃时间
                async with lock:
                    if client_id in clients:
                        clients[client_id]["last_active"] = datetime.datetime.now()

                if message_type == "ping":
                    # 处理ping消息
                    timestamp = message.get("timestamp", 0)
                    companion_id = None
                    companion_online = False

                    # 查找连接的伴侣
                    async with lock:
                        if client_id in client_to_companion:
                            companion_id = client_to_companion[client_id]
                            if companion_id in companions:
                                companion_online = True

                    # 发送pong响应
                    await websocket.send_json({
                        "type": "pong",
                        "timestamp": timestamp,
                        "echo_timestamp": timestamp,
                        "companion_id": companion_id,
                        "companion_online": companion_online
                    })

                elif message_type == "client_init":
                    # 客户端初始化
                    companion_id = message.get("robot_id") or message.get("companion_id", "companion_robot_001")
                    
                    # 关联客户端和伴侣
                    success = await connect_client_to_companion(client_id, companion_id)

                    # 在自适应视频管理器中连接客户端和伴侣
                    if success:
                        adaptive_video_manager.connect_client_to_companion(client_id, companion_id)

                    # 获取伴侣在线状态
                    companion_online = companion_id in companions

                    # 如果伴侣在线，发送初始状态
                    if companion_online:
                        companion_data = companions[companion_id].get("data", {})
                        await websocket.send_json({
                            "type": "robot_status_update",
                            "data": companion_data
                        })

                    # 告知客户端伴侣连接状态
                    await websocket.send_json({
                        "type": "robot_connection_status",
                        "robot_id": companion_id,
                        "connected": companion_online,
                        "timestamp": int(time.time() * 1000)
                    })

                elif message_type == "companion_command":
                    # 处理伴侣控制命令
                    companion_id = message.get("robot_id")
                    command = message.get("command")
                    params = message.get("params", {})

                    logger.info(f"收到伴侣控制命令: {command}, 参数: {params}")

                    if companion_id in companions:
                        # 转发命令到伴侣
                        await forward_command_to_companion(companion_id, command, params)
                        
                        # 发送命令响应
                        await websocket.send_json({
                            "type": "command_response",
                            "command": command,
                            "status": "success",
                            "message": "命令已发送"
                        })
                    else:
                        await websocket.send_json({
                            "type": "command_response",
                            "command": command,
                            "status": "error",
                            "message": "伴侣机器人不在线"
                        })

                elif message_type == "get_robot_status":
                    # 处理获取伴侣状态请求
                    companion_id = message.get("robot_id")
                    if companion_id in companions:
                        await websocket.send_json({
                            "type": "robot_connection_status",
                            "robot_id": companion_id,
                            "connected": True,
                            "timestamp": int(time.time() * 1000)
                        })
                    else:
                        await websocket.send_json({
                            "type": "robot_connection_status",
                            "robot_id": companion_id,
                            "connected": False,
                            "timestamp": int(time.time() * 1000)
                        })

                elif message_type == "request_video_stream":
                    # 处理请求视频流
                    companion_id = message.get("robot_id")
                    if companion_id in companions:
                        await forward_command_to_companion(companion_id, "request_video_stream", {})
                        await websocket.send_json({
                            "type": "video_stream_request_sent",
                            "robot_id": companion_id,
                            "timestamp": int(time.time() * 1000)
                        })
                    else:
                        await websocket.send_json({
                            "type": "error",
                            "message": "伴侣机器人不在线，无法请求视频流"
                        })

                elif message_type == "client_network_status":
                    # 处理客户端网络状态更新
                    status_data = message.get("status", {})
                    adaptive_video_manager.update_client_status(client_id, status_data)

                elif message_type == "client_quality_request":
                    # 处理客户端质量调整请求
                    companion_id = message.get("robot_id")
                    preset = message.get("preset")

                    if companion_id in companions:
                        # 通知客户端请求已收到
                        await websocket.send_json({
                            "type": "quality_request_received",
                            "preset": preset
                        })
                        logger.info(f"客户端 {client_id} 请求调整质量为 {preset}")

            except json.JSONDecodeError:
                logger.error(f"收到无效JSON: {data}")
            except Exception as e:
                logger.error(f"处理微信客户端消息出错: {e}")

    except WebSocketDisconnect:
        logger.info(f"微信客户端 {client_id} 已断开连接")
        await handle_client_disconnect(client_id)


# 伴侣机器人WebSocket连接处理
@app.websocket("/ws/companion_robot/{companion_id}")
async def companion_robot_websocket_endpoint(websocket: WebSocket, companion_id: str):
    await websocket.accept()
    logger.info(f"伴侣机器人 {companion_id} 已连接")

    # 注册伴侣机器人
    async with lock:
        # 关闭旧连接
        if companion_id in companions and "websocket" in companions[companion_id]:
            try:
                old_ws = companions[companion_id]["websocket"]
                await old_ws.close(code=1000, reason="新连接替代")
            except Exception as e:
                logger.error(f"关闭旧连接失败: {e}")

        companions[companion_id] = {
            "websocket": websocket,
            "last_active": datetime.datetime.now(),
            "data": {},
            "connection_id": f"{companion_id}_{datetime.datetime.now().timestamp()}"
        }
        
        # 初始化伴侣到客户端的映射
        if companion_id not in companion_to_clients:
            companion_to_clients[companion_id] = []

    # 注册到自适应视频管理器
    adaptive_video_manager.register_companion(companion_id, websocket)

    # 通知所有关联的客户端伴侣已连接
    await broadcast_companion_status(companion_id, True)

    try:
        while True:
            data = await websocket.receive_text()
            try:
                message = json.loads(data)
                message_type = message.get("type", "")

                # 更新伴侣活跃时间
                async with lock:
                    if companion_id in companions:
                        companions[companion_id]["last_active"] = datetime.datetime.now()

                if message_type == "heartbeat":
                    # 处理心跳消息
                    await websocket.send_json({
                        "type": "heartbeat_ack",
                        "timestamp": message.get("timestamp", int(time.time() * 1000))
                    })

                elif message_type == "video_frame":
                    # 转发视频帧
                    message["server_timestamp"] = int(time.time() * 1000)
                    await forward_video_frame_optimized(companion_id, message)

                elif message_type == "robot_status_update":
                    # 更新伴侣状态
                    status_data = message.get("data", {})

                    # 更新本地存储
                    async with lock:
                        if companion_id in companions:
                            companions[companion_id]["data"].update(status_data)

                    # 更新自适应视频管理器
                    adaptive_video_manager.update_companion_status(companion_id, status_data)

                    # 保存状态到本地历史记录
                    await save_companion_status_to_history(companion_id, status_data)

                    # 广播给客户端
                    await broadcast_companion_update(companion_id)

                elif message_type == "command_response":
                    # 处理命令响应
                    await handle_command_response(companion_id, message)

                elif message_type == "quality_adjustment_result":
                    # 处理质量调整结果
                    await handle_quality_adjustment_result(companion_id, message)

                elif message_type == "interaction_event":
                    # 处理交互事件
                    await handle_interaction_event(companion_id, message)

            except json.JSONDecodeError:
                logger.error(f"收到无效JSON: {data}")
            except Exception as e:
                logger.error(f"处理伴侣机器人消息出错: {e}")

    except WebSocketDisconnect:
        logger.info(f"伴侣机器人 {companion_id} 已断开连接")
        await handle_companion_disconnect(companion_id)


# 保存伴侣状态到历史记录
async def save_companion_status_to_history(companion_id, status_data):
    """保存伴侣状态到本地历史记录"""
    try:
        current_time = datetime.datetime.now()
        date_str = current_time.strftime("%Y-%m-%d")
        
        async with lock:
            if companion_id not in daily_companion_data:
                daily_companion_data[companion_id] = {}
            
            if date_str not in daily_companion_data[companion_id]:
                daily_companion_data[companion_id][date_str] = {
                    "status_updates": [],
                    "summary": {}
                }
            
            # 添加状态更新记录
            status_record = status_data.copy()
            status_record["timestamp"] = int(current_time.timestamp() * 1000)
            status_record["time"] = current_time.strftime("%H:%M:%S")
            
            daily_companion_data[companion_id][date_str]["status_updates"].append(status_record)
            
            # 更新摘要信息
            daily_companion_data[companion_id][date_str]["summary"].update(status_data)
            
            # 保持最近30天的记录
            dates = list(daily_companion_data[companion_id].keys())
            if len(dates) > 30:
                dates.sort()
                old_dates = dates[:-30]
                for old_date in old_dates:
                    del daily_companion_data[companion_id][old_date]
                    
        logger.debug(f"已保存伴侣 {companion_id} 状态到历史记录")
        
    except Exception as e:
        logger.error(f"保存伴侣状态历史记录出错: {e}")


# 处理命令响应
async def handle_command_response(companion_id, message):
    """处理来自伴侣的命令响应"""
    command = message.get("command")
    status = message.get("status")
    error = message.get("error")
    
    logger.info(f"伴侣 {companion_id} 命令响应: {command} - {status}")
    
    # 转发给所有关联的客户端
    async with lock:
        if companion_id in companion_to_clients:
            for client_id in companion_to_clients[companion_id]:
                if client_id in clients and "websocket" in clients[client_id]:
                    try:
                        await clients[client_id]["websocket"].send_json({
                            "type": "command_response",
                            "command": command,
                            "status": status,
                            "error": error,
                            "companion_id": companion_id
                        })
                    except Exception as e:
                        logger.error(f"向客户端 {client_id} 发送命令响应失败: {e}")


# 处理质量调整结果
async def handle_quality_adjustment_result(companion_id, message):
    """处理质量调整结果"""
    success = message.get("success", False)
    preset = message.get("preset")
    actual_resolution = message.get("actual_resolution")
    actual_fps = message.get("actual_fps")

    logger.info(f"伴侣 {companion_id} 质量调整结果: {preset} - {success}")

    if success:
        async with lock:
            if companion_id in companion_to_clients:
                for client_id in companion_to_clients[companion_id]:
                    if client_id in clients and "websocket" in clients[client_id]:
                        try:
                            await clients[client_id]["websocket"].send_json({
                                "type": "video_quality_update",
                                "preset": preset,
                                "resolution": actual_resolution,
                                "fps": actual_fps
                            })
                        except Exception as e:
                            logger.error(f"通知客户端 {client_id} 质量更新失败: {e}")


# 处理交互事件
async def handle_interaction_event(companion_id, message):
    """处理交互事件"""
    event_type = message.get("event_type")
    event_data = message.get("data", {})
    timestamp = message.get("timestamp", int(time.time() * 1000))

    logger.info(f"伴侣 {companion_id} 交互事件: {event_type}")

    # 保存到交互历史
    await save_interaction_to_history(companion_id, {
        "event_type": event_type,
        "data": event_data,
        "timestamp": timestamp
    })

    # 转发给客户端
    async with lock:
        if companion_id in companion_to_clients:
            for client_id in companion_to_clients[companion_id]:
                if client_id in clients and "websocket" in clients[client_id]:
                    try:
                        await clients[client_id]["websocket"].send_json({
                            "type": "interaction_event",
                            "event_type": event_type,
                            "data": event_data,
                            "companion_id": companion_id,
                            "timestamp": timestamp
                        })
                    except Exception as e:
                        logger.error(f"向客户端 {client_id} 发送交互事件失败: {e}")


# 保存交互事件到历史
async def save_interaction_to_history(companion_id, interaction_data):
    """保存交互事件到历史记录"""
    try:
        async with lock:
            if companion_id not in interaction_history:
                interaction_history[companion_id] = []

            current_time = datetime.datetime.now()
            interaction_record = interaction_data.copy()
            interaction_record["saved_timestamp"] = int(current_time.timestamp() * 1000)
            interaction_record["saved_date"] = current_time.strftime("%Y-%m-%d")
            interaction_record["saved_time"] = current_time.strftime("%H:%M:%S")

            interaction_history[companion_id].append(interaction_record)

            # 保持最近200条记录
            if len(interaction_history[companion_id]) > 200:
                interaction_history[companion_id] = interaction_history[companion_id][-200:]

            logger.debug(f"已保存交互记录到历史 - 伴侣: {companion_id}")

    except Exception as e:
        logger.error(f"保存交互历史记录出错: {e}")


# 转发控制命令到伴侣
async def forward_command_to_companion(companion_id, command, params):
    """转发控制命令到伴侣机器人"""
    async with lock:
        if companion_id in companions and "websocket" in companions[companion_id]:
            try:
                await companions[companion_id]["websocket"].send_json({
                    "type": "companion_command",
                    "command": command,
                    "params": params,
                    "timestamp": int(time.time() * 1000)
                })
                logger.info(f"向伴侣 {companion_id} 转发命令: {command}")
                return True
            except Exception as e:
                logger.error(f"向伴侣 {companion_id} 转发命令失败: {e}")
                return False
        else:
            logger.warning(f"找不到伴侣 {companion_id}")
            return False


# 优化后的视频帧转发
async def forward_video_frame_optimized(companion_id, message):
    """优化的视频帧转发"""
    async with lock:
        if companion_id in companion_to_clients:
            for client_id in companion_to_clients[companion_id]:
                if client_id in clients and "websocket" in clients[client_id]:
                    try:
                        await clients[client_id]["websocket"].send_json(message)
                    except Exception as e:
                        logger.error(f"向客户端 {client_id} 发送视频帧失败: {e}")


# 连接客户端到伴侣
async def connect_client_to_companion(client_id, companion_id):
    """连接客户端到伴侣"""
    async with lock:
        # 如果客户端已关联其他伴侣，先取消关联
        if client_id in client_to_companion:
            old_companion_id = client_to_companion[client_id]
            if old_companion_id in companion_to_clients and client_id in companion_to_clients[old_companion_id]:
                companion_to_clients[old_companion_id].remove(client_id)

        # 建立新的关联
        client_to_companion[client_id] = companion_id
        if companion_id not in companion_to_clients:
            companion_to_clients[companion_id] = []
        if client_id not in companion_to_clients[companion_id]:
            companion_to_clients[companion_id].append(client_id)

        logger.info(f"客户端 {client_id} 已连接到伴侣 {companion_id}")
        return companion_id in companions


# 广播伴侣状态
async def broadcast_companion_status(companion_id, is_connected):
    """向所有关联客户端广播伴侣状态"""
    async with lock:
        if companion_id in companion_to_clients:
            for client_id in companion_to_clients[companion_id]:
                if client_id in clients and "websocket" in clients[client_id]:
                    try:
                        await clients[client_id]["websocket"].send_json({
                            "type": "robot_connection_status",
                            "robot_id": companion_id,
                            "connected": is_connected,
                            "timestamp": int(time.time() * 1000)
                        })
                    except Exception as e:
                        logger.error(f"通知客户端 {client_id} 伴侣状态变更失败: {e}")


# 广播伴侣更新
async def broadcast_companion_update(companion_id):
    """广播伴侣数据更新"""
    async with lock:
        if companion_id in companion_to_clients and companion_id in companions and "data" in companions[companion_id]:
            companion_data = companions[companion_id]["data"]

            for client_id in companion_to_clients[companion_id]:
                if client_id in clients and "websocket" in clients[client_id]:
                    try:
                        await clients[client_id]["websocket"].send_json({
                            "type": "robot_status_update",
                            "data": companion_data
                        })
                    except Exception as e:
                        logger.error(f"向客户端 {client_id} 发送状态更新失败: {e}")


# 处理客户端断开连接
async def handle_client_disconnect(client_id):
    """处理客户端断开连接"""
    async with lock:
        if client_id in clients:
            del clients[client_id]
        if client_id in client_to_companion:
            companion_id = client_to_companion[client_id]
            if companion_id in companion_to_clients and client_id in companion_to_clients[companion_id]:
                companion_to_clients[companion_id].remove(client_id)
            del client_to_companion[client_id]

    # 从自适应视频管理器中断开客户端
    adaptive_video_manager.disconnect_client(client_id)


# 处理伴侣断开连接
async def handle_companion_disconnect(companion_id):
    """处理伴侣断开连接"""
    async with lock:
        if companion_id in companions:
            del companions[companion_id]

    # 通知所有关联的客户端伴侣已断开
    await broadcast_companion_status(companion_id, False)

    # 从自适应视频管理器中断开伴侣
    adaptive_video_manager.disconnect_companion(companion_id)


# 连接监控
async def connection_monitor():
    """定期检查连接状态，清理过期连接"""
    while True:
        try:
            current_time = datetime.datetime.now()

            # 检查伴侣连接状态
            async with lock:
                expired_companions = []
                for companion_id, companion in companions.items():
                    if (current_time - companion["last_active"]).total_seconds() > 30:
                        expired_companions.append(companion_id)

                for companion_id in expired_companions:
                    logger.warning(f"伴侣 {companion_id} 连接超时，清理连接")
                    await handle_companion_disconnect(companion_id)

                # 检查客户端连接状态
                expired_clients = []
                for client_id, client in clients.items():
                    if (current_time - client["last_active"]).total_seconds() > 60:
                        expired_clients.append(client_id)

                for client_id in expired_clients:
                    logger.warning(f"客户端 {client_id} 连接超时，清理连接")
                    await handle_client_disconnect(client_id)

        except Exception as e:
            logger.error(f"连接监控错误: {e}")

        await asyncio.sleep(10)


# 启动事件
@app.on_event("startup")
async def startup_event():
    """服务器启动时的初始化工作"""
    global adaptive_video_manager

    # 初始化自适应视频管理器
    adaptive_video_manager = CompanionAdaptiveVideoManager()
    logger.info("伴侣自适应视频管理器已初始化")

    # 启动连接监控
    asyncio.create_task(connection_monitor())
    
    logger.info("机器人伴侣服务端已启动 (不含IoT功能)")


# 关闭事件
@app.on_event("shutdown")
def shutdown_event():
    """服务器关闭时的清理工作"""
    if adaptive_video_manager:
        adaptive_video_manager.shutdown()

    logger.info("伴侣服务器已关闭")


# 健康检查路由
@app.get("/health")
def health_check():
    return {"status": "healthy", "service": "companion_robot"}


# 获取伴侣状态API
@app.get("/api/companion/{companion_id}/status")
def get_companion_status(companion_id: str):
    """获取伴侣状态"""
    if companion_id in companions:
        return {
            "status": "online",
            "data": companions[companion_id].get("data", {}),
            "connection_info": {
                "connected_at": companions[companion_id]["last_active"].isoformat(),
                "connection_id": companions[companion_id]["connection_id"]
            }
        }
    else:
        return {"status": "offline", "data": {}}


# 获取伴侣历史数据API
@app.get("/api/companion/{companion_id}/history/{date}")
def get_companion_history(companion_id: str, date: str):
    """获取伴侣指定日期的历史数据"""
    if companion_id in daily_companion_data and date in daily_companion_data[companion_id]:
        return {
            "status": "success",
            "date": date,
            "data": daily_companion_data[companion_id][date]
        }
    else:
        return {"status": "not_found", "message": "指定日期的数据不存在"}


# 获取伴侣交互历史API
@app.get("/api/companion/{companion_id}/interactions")
def get_companion_interactions(companion_id: str, limit: int = 50):
    """获取伴侣交互历史"""
    if companion_id in interaction_history:
        interactions = interaction_history[companion_id][-limit:]
        return {
            "status": "success",
            "companion_id": companion_id,
            "interactions": interactions,
            "total_count": len(interaction_history[companion_id])
        }
    else:
        return {
            "status": "success",
            "companion_id": companion_id,
            "interactions": [],
            "total_count": 0
        }


# 启动服务器
if __name__ == "__main__":
    uvicorn.run(app, host="172.20.39.181", port=1234)