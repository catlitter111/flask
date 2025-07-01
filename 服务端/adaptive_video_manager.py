# adaptive_video_manager.py - 机器人伴侣自适应视频管理器
import time
import threading
import json
import logging
from collections import defaultdict

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger("CompanionAdaptiveVideoManager")


class CompanionAdaptiveVideoManager:
    """管理多个伴侣设备的视频质量自适应"""

    # 预设质量配置 - 优化了延迟和压缩
    QUALITY_PRESETS = {
        "ultra_high": {
            "resolution": (800, 600),
            "fps": 20,
            "bitrate": 1200,  # Kbps
            "quality": 85,  # JPEG质量(1-100)
            "min_bandwidth": 1500  # 需要的最小带宽(Kbps)
        },
        "high": {
            "resolution": (640, 480),
            "fps": 15,
            "bitrate": 600,  # 降低码率
            "quality": 75,  # 降低质量以减少延迟
            "min_bandwidth": 800
        },
        "medium": {
            "resolution": (480, 360),
            "fps": 12,  # 稍微提高帧率
            "bitrate": 350,  # 降低码率
            "quality": 65,
            "min_bandwidth": 450
        },
        "low": {
            "resolution": (320, 240),
            "fps": 10,
            "bitrate": 200,  # 大幅降低码率
            "quality": 55,
            "min_bandwidth": 250
        },
        "very_low": {
            "resolution": (240, 180),
            "fps": 8,
            "bitrate": 120,
            "quality": 45,
            "min_bandwidth": 150
        },
        "minimum": {
            "resolution": (160, 120),
            "fps": 5,
            "bitrate": 60,  # 最小码率
            "quality": 35,
            "min_bandwidth": 80
        },
        "ultra_low": {
            "resolution": (120, 90),
            "fps": 3,
            "bitrate": 30,  # 极低码率
            "quality": 25,
            "min_bandwidth": 40
        }
    }

    def __init__(self):
        self.clients = {}  # 存储各个客户端的连接信息
        self.companions = {}  # 存储各个伴侣机器人的状态信息
        self.companion_to_clients = defaultdict(set)  # 映射伴侣到观看它的客户端
        self.lock = threading.RLock()  # 用于线程安全访问共享数据

        # 质量命令回调函数
        self.quality_command_callback = None
        
        # 启动监控线程
        self.running = True
        self.monitor_thread = threading.Thread(target=self._monitor_loop)
        self.monitor_thread.daemon = True
        self.monitor_thread.start()

    def set_quality_command_callback(self, callback):
        """设置质量命令回调函数"""
        self.quality_command_callback = callback
        logger.info("质量命令回调函数已设置")

    def register_client(self, client_id, websocket):
        """注册一个微信客户端"""
        with self.lock:
            connection_time = time.time()

            self.clients[client_id] = {
                "websocket": websocket,
                "connected_at": connection_time,
                "last_seen": connection_time,
                "buffer_health": 100,  # 初始缓冲健康度(0-100)
                "network_quality": "unknown",  # 网络质量评估
                "current_preset": "medium",  # 默认使用中等质量
                "companion_id": None,  # 尚未连接到任何伴侣
                "latency": 0,  # 客户端报告的延迟(ms)
                "jitter": 0,  # 客户端报告的抖动(ms)
                "fps": 0,  # 客户端报告的fps
                "dropped_frames": 0,  # 客户端报告的丢帧数
                "last_buffer_health": 100,  # 上次缓冲健康度
                "last_latency": 0,  # 上次延迟
                "connection_status": "connected",  # 连接状态
                "quality_change_time": connection_time,  # 上次质量变更时间
                "quality_stable_period": 10,  # 质量稳定期(秒)
                "interaction_mode": "normal"  # 交互模式
            }
            logger.info(f"客户端 {client_id} 已注册")

    def register_companion(self, companion_id, websocket):
        """注册一个伴侣机器人"""
        with self.lock:
            connection_time = time.time()

            self.companions[companion_id] = {
                "websocket": websocket,
                "connected_at": connection_time,
                "last_seen": connection_time,
                "upload_bandwidth": 1000,  # 初始假设上传带宽1000Kbps
                "current_preset": "medium",  # 默认使用中等质量
                "cpu_usage": 0,  # CPU使用率(0-100)
                "battery_level": 100,  # 电池电量(0-100)
                "signal_strength": 70,  # 信号强度(0-100)
                "frames_sent": 0,  # 已发送的帧数
                "bytes_sent": 0,  # 已发送的字节数
                "connection_status": "connected",  # 连接状态
                "reconnect_count": 0,  # 重连次数
                "last_quality_change": connection_time,  # 上次质量变更时间
                "quality_stable_period": 15,  # 质量稳定期(秒)
                "companion_mode": "idle",  # 伴侣模式: idle, following, interacting
                "emotion_state": "neutral",  # 情绪状态
                "interaction_level": "normal"  # 交互级别
            }
            logger.info(f"伴侣机器人 {companion_id} 已注册")

    def connect_client_to_companion(self, client_id, companion_id):
        """将客户端连接到特定伴侣"""
        with self.lock:
            if client_id in self.clients and companion_id in self.companions:
                self.clients[client_id]["companion_id"] = companion_id
                self.companion_to_clients[companion_id].add(client_id)

                connection_time = time.time()
                self.clients[client_id]["connected_to_companion_at"] = connection_time

                # 立即发送当前质量设置
                current_preset = self.companions[companion_id]["current_preset"]
                self._send_quality_info_to_client(client_id, current_preset)
                logger.info(f"客户端 {client_id} 已连接到伴侣 {companion_id}")
                return True
            return False

    def disconnect_client(self, client_id):
        """断开客户端连接"""
        with self.lock:
            if client_id in self.clients:
                companion_id = self.clients[client_id].get("companion_id")
                if companion_id and companion_id in self.companion_to_clients:
                    self.companion_to_clients[companion_id].discard(client_id)
                del self.clients[client_id]
                logger.info(f"客户端 {client_id} 已断开连接")

    def disconnect_companion(self, companion_id):
        """断开伴侣连接"""
        with self.lock:
            if companion_id in self.companions:
                # 通知所有连接到此伴侣的客户端
                for client_id in list(self.companion_to_clients[companion_id]):
                    if client_id in self.clients:
                        self._send_to_client(client_id, {
                            "type": "companion_disconnected",
                            "companion_id": companion_id,
                            "timestamp": int(time.time() * 1000)
                        })
                        self.clients[client_id]["companion_id"] = None
                        self.clients[client_id]["connection_status"] = "companion_disconnected"

                del self.companions[companion_id]
                self.companion_to_clients[companion_id].clear()
                logger.info(f"伴侣机器人 {companion_id} 已断开连接")

    def update_client_status(self, client_id, status_data):
        """更新客户端状态"""
        with self.lock:
            if client_id in self.clients:
                client = self.clients[client_id]
                
                # 更新最后活跃时间
                client["last_seen"] = time.time()

                # 处理来自server.py的连接活跃信号
                if "last_message_time" in status_data:
                    client["last_seen"] = status_data["last_message_time"]
                if "connection_active" in status_data:
                    client["connection_status"] = "connected" if status_data["connection_active"] else "disconnected"

                # 更新客户端报告的状态
                if "buffer_health" in status_data:
                    client["buffer_health"] = status_data["buffer_health"]
                if "network_quality" in status_data:
                    client["network_quality"] = status_data["network_quality"]
                if "latency" in status_data:
                    client["latency"] = status_data["latency"]
                if "jitter" in status_data:
                    client["jitter"] = status_data["jitter"]
                if "fps" in status_data:
                    client["fps"] = status_data["fps"]
                if "dropped_frames" in status_data:
                    client["dropped_frames"] = status_data["dropped_frames"]

                # 检查是否需要调整质量
                should_adjust = False
                buffer_change = abs(client.get("last_buffer_health", 100) - client["buffer_health"])
                if buffer_change > 20:
                    should_adjust = True
                    client["last_buffer_health"] = client["buffer_health"]

                latency_change = abs(client.get("last_latency", 0) - client["latency"])
                if latency_change > 100:
                    should_adjust = True
                    client["last_latency"] = client["latency"]

                current_time = time.time()
                time_since_last_change = current_time - client.get("quality_change_time", 0)
                quality_stable_period = client.get("quality_stable_period", 10)

                if should_adjust and time_since_last_change >= quality_stable_period:
                    self._check_client_quality_adjustment(client_id)
                    client["quality_change_time"] = current_time

    def update_companion_status(self, companion_id, status_data):
        """更新伴侣状态"""
        with self.lock:
            if companion_id in self.companions:
                companion = self.companions[companion_id]
                
                # 更新最后活跃时间
                companion["last_seen"] = time.time()

                # 处理来自server.py的连接活跃信号
                if "last_message_time" in status_data:
                    companion["last_seen"] = status_data["last_message_time"]
                if "connection_active" in status_data:
                    companion["connection_status"] = "connected" if status_data["connection_active"] else "disconnected"

                # 更新伴侣报告的状态
                if "upload_bandwidth" in status_data:
                    companion["upload_bandwidth"] = status_data["upload_bandwidth"]
                if "cpu_usage" in status_data:
                    companion["cpu_usage"] = status_data["cpu_usage"]
                if "battery_level" in status_data:
                    companion["battery_level"] = status_data["battery_level"]
                if "signal_strength" in status_data:
                    companion["signal_strength"] = status_data["signal_strength"]
                if "frames_sent" in status_data:
                    companion["frames_sent"] = status_data["frames_sent"]
                if "bytes_sent" in status_data:
                    companion["bytes_sent"] = status_data["bytes_sent"]
                if "companion_mode" in status_data:
                    companion["companion_mode"] = status_data["companion_mode"]
                if "emotion_state" in status_data:
                    companion["emotion_state"] = status_data["emotion_state"]

                current_time = time.time()
                time_since_last_change = current_time - companion.get("last_quality_change", 0)
                quality_stable_period = companion.get("quality_stable_period", 15)

                if time_since_last_change >= quality_stable_period:
                    self._check_companion_quality_adjustment(companion_id)
                    companion["last_quality_change"] = current_time

    def _check_client_quality_adjustment(self, client_id):
        """检查是否需要根据客户端状态调整质量 - 激进策略"""
        client = self.clients[client_id]
        companion_id = client.get("companion_id")

        if not companion_id or companion_id not in self.companions:
            return

        current_preset = client["current_preset"]
        buffer_health = client["buffer_health"]
        latency = client["latency"]
        jitter = client["jitter"]

        # 更激进的质量调整策略
        new_preset = None
        reason = ""

        # 极高延迟 - 立即降到最低
        if latency > 2000:
            new_preset = "ultra_low"
            reason = f"极高延迟({latency}ms)"
        elif latency > 1000:
            new_preset = self._get_lower_preset(current_preset, 3)
            reason = f"高延迟({latency}ms)"
        elif latency > 500:
            new_preset = self._get_lower_preset(current_preset, 2)
            reason = f"延迟过高({latency}ms)"
        
        # 缓冲健康度检查
        elif buffer_health < 15:
            new_preset = self._get_lower_preset(current_preset, 3)
            reason = f"缓冲极差({buffer_health}%)"
        elif buffer_health < 30:
            new_preset = self._get_lower_preset(current_preset, 2)
            reason = f"缓冲较差({buffer_health}%)"
        elif buffer_health < 50:
            new_preset = self._get_lower_preset(current_preset, 1)
            reason = f"缓冲偏低({buffer_health}%)"
        
        # 抖动检查
        elif jitter > 200:
            new_preset = self._get_lower_preset(current_preset, 2)
            reason = f"抖动严重({jitter}ms)"
        elif jitter > 100:
            new_preset = self._get_lower_preset(current_preset, 1)
            reason = f"抖动偏高({jitter}ms)"
        
        # 网络状况良好时适度提升
        elif (buffer_health > 85 and latency < 100 and jitter < 30 and 
              current_preset in ["ultra_low", "minimum", "very_low"]):
            new_preset = self._get_higher_preset(current_preset, 1)
            reason = f"网络状况改善"
        elif (buffer_health > 95 and latency < 50 and jitter < 20):
            new_preset = self._get_higher_preset(current_preset, 1)
            reason = f"网络状况优秀"

        if new_preset and new_preset != current_preset:
            logger.info(f"客户端 {client_id} 质量调整: {current_preset} -> {new_preset}")
            logger.info(f"调整依据: {reason} (缓冲={buffer_health}%, 延迟={latency}ms, 抖动={jitter}ms)")
            client["current_preset"] = new_preset
            client["quality_change_time"] = time.time()
            self._send_quality_info_to_client(client_id, new_preset)
            
            # 同时通知伴侣调整编码质量
            self._send_quality_command_to_companion(companion_id, new_preset)
            
    def _send_quality_command_to_companion(self, companion_id, preset):
        """向伴侣发送质量调整命令"""
        try:
            if companion_id in self.companions:
                companion_ws = self.companions[companion_id]
                message = {
                    "type": "quality_adjustment",
                    "preset": preset,
                    "immediate": True,
                    "timestamp": int(time.time() * 1000)
                }
                
                # 通过回调函数发送消息（避免异步问题）
                if self.quality_command_callback:
                    try:
                        import asyncio
                        import inspect
                        
                        # 检查回调函数是否是协程函数
                        if inspect.iscoroutinefunction(self.quality_command_callback):
                            # 如果有活动的事件循环，创建任务；否则用线程执行
                            try:
                                loop = asyncio.get_running_loop()
                                asyncio.create_task(self.quality_command_callback(companion_id, message))
                            except RuntimeError:
                                # 没有活动的事件循环，使用线程调用
                                def run_async_callback():
                                    try:
                                        asyncio.run(self.quality_command_callback(companion_id, message))
                                    except Exception as e:
                                        logger.error(f"异步回调执行失败: {e}")
                                
                                import threading
                                thread = threading.Thread(target=run_async_callback, daemon=True)
                                thread.start()
                        else:
                            # 同步回调函数，直接调用
                            self.quality_command_callback(companion_id, message)
                            
                        logger.info(f"已向伴侣 {companion_id} 发送质量调整命令: {preset}")
                    except Exception as e:
                        logger.error(f"向伴侣 {companion_id} 发送质量调整命令失败: {e}")
                else:
                    logger.warning(f"无质量命令回调函数，无法向伴侣 {companion_id} 发送命令")
                        
        except Exception as e:
            logger.error(f"发送质量命令到伴侣失败: {e}")

    def _check_companion_quality_adjustment(self, companion_id):
        """检查是否需要根据伴侣状态调整质量"""
        companion = self.companions[companion_id]
        current_preset = companion["current_preset"]
        upload_bandwidth = companion["upload_bandwidth"]
        signal_strength = companion["signal_strength"]
        cpu_usage = companion["cpu_usage"]

        client_count = len(self.companion_to_clients[companion_id])
        if client_count == 0:
            return

        # 计算每个客户端可用的带宽
        available_bandwidth_per_client = upload_bandwidth * 0.8 / client_count

        # 根据CPU使用率和信号强度调整
        if cpu_usage > 80:
            available_bandwidth_per_client *= 0.8
        if signal_strength < 40:
            available_bandwidth_per_client *= 0.7

        # 找到适合的质量预设
        best_preset = "minimum"
        for preset, config in sorted(self.QUALITY_PRESETS.items(),
                                     key=lambda x: x[1]["min_bandwidth"],
                                     reverse=True):
            if config["min_bandwidth"] <= available_bandwidth_per_client:
                best_preset = preset
                break

        if best_preset != current_preset:
            logger.info(f"伴侣 {companion_id} 质量调整: {current_preset} -> {best_preset}")
            logger.info(f"带宽={upload_bandwidth}Kbps, CPU={cpu_usage}%, 信号={signal_strength}%, 客户端={client_count}")
            companion["current_preset"] = best_preset

            # 通知所有连接的客户端
            for client_id in self.companion_to_clients[companion_id]:
                if client_id in self.clients:
                    self.clients[client_id]["current_preset"] = best_preset
                    self._send_quality_info_to_client(client_id, best_preset)

            # 通知伴侣调整质量
            self._send_quality_command_to_companion(companion_id, best_preset)



    def _send_quality_info_to_client(self, client_id, preset):
        """向客户端发送质量信息"""
        if client_id in self.clients:
            client = self.clients[client_id]
            ws = client["websocket"]

            quality_config = self.QUALITY_PRESETS[preset]
            info = {
                "type": "video_quality_update",
                "preset": preset,
                "resolution": f"{quality_config['resolution'][0]}x{quality_config['resolution'][1]}",
                "fps": quality_config["fps"]
            }

            try:
                # 异步发送消息 - 安全处理
                import asyncio
                import inspect
                
                if hasattr(ws, 'send_json'):
                    # FastAPI WebSocket - 需要异步处理
                    try:
                        loop = asyncio.get_running_loop()
                        asyncio.create_task(ws.send_json(info))
                    except RuntimeError:
                        # 没有活动的事件循环，记录错误
                        logger.error(f"无活动事件循环，无法向客户端 {client_id} 发送质量信息")
                        return
                else:
                    # 兼容同步WebSocket
                    ws.send(json.dumps(info))
                logger.info(f"已向客户端 {client_id} 发送质量信息: {preset}")
            except Exception as e:
                logger.error(f"向客户端 {client_id} 发送质量信息失败: {e}")
                client["connection_status"] = "connection_error"

    def _send_to_client(self, client_id, message):
        """向客户端发送通用消息"""
        if client_id in self.clients:
            client = self.clients[client_id]
            ws = client["websocket"]

            try:
                # 异步发送消息 - 安全处理
                import asyncio
                
                if hasattr(ws, 'send_json'):
                    # FastAPI WebSocket - 需要异步处理
                    try:
                        loop = asyncio.get_running_loop()
                        asyncio.create_task(ws.send_json(message))
                    except RuntimeError:
                        # 没有活动的事件循环，记录错误
                        logger.error(f"无活动事件循环，无法向客户端 {client_id} 发送消息")
                        return
                else:
                    # 兼容同步WebSocket
                    ws.send(json.dumps(message))
            except Exception as e:
                logger.error(f"向客户端 {client_id} 发送消息失败: {e}")
                client["connection_status"] = "connection_error"

    def _get_lower_preset(self, current_preset, steps=1):
        """获取比当前更低的质量预设"""
        # 按质量从高到低排序
        presets = ["ultra_high", "high", "medium", "low", "very_low", "minimum", "ultra_low"]
        try:
            current_index = presets.index(current_preset)
            target_index = min(len(presets) - 1, current_index + steps)
            return presets[target_index]
        except ValueError:
            return "low"  # 默认返回较低质量

    def _get_higher_preset(self, current_preset, steps=1):
        """获取比当前更高的质量预设"""
        # 按质量从高到低排序
        presets = ["ultra_high", "high", "medium", "low", "very_low", "minimum", "ultra_low"]
        try:
            current_index = presets.index(current_preset)
            target_index = max(0, current_index - steps)
            return presets[target_index]
        except ValueError:
            return "medium"  # 默认返回中等质量

    def _monitor_loop(self):
        """监控循环，定期检查所有连接的状态"""
        last_full_check_time = 0
        full_check_interval = 10

        while self.running:
            try:
                current_time = time.time()
                with self.lock:
                    self._check_connection_status()

                    if current_time - last_full_check_time >= full_check_interval:
                        # 检查离线客户端 - 延长超时时间
                        offline_clients = []
                        for client_id, client in self.clients.items():
                            if current_time - client["last_seen"] > 60:  # 60秒超时
                                offline_clients.append(client_id)
                                logger.warning(f"客户端 {client_id} 已超过60秒无响应")

                        for client_id in offline_clients:
                            self.disconnect_client(client_id)

                        # 检查离线伴侣 - 延长超时时间
                        offline_companions = []
                        for companion_id, companion in self.companions.items():
                            if current_time - companion["last_seen"] > 60:  # 60秒超时
                                offline_companions.append(companion_id)
                                logger.warning(f"伴侣 {companion_id} 已超过60秒无响应")

                        for companion_id in offline_companions:
                            self.disconnect_companion(companion_id)

                        last_full_check_time = current_time

            except Exception as e:
                logger.error(f"监控循环出错: {e}")

            time.sleep(1)

    def _check_connection_status(self):
        """检查所有连接状态"""
        current_time = time.time()

        # 检查客户端连接
        for client_id, client in self.clients.items():
            if current_time - client["last_seen"] > 30:  # 30秒警告
                client["connection_status"] = "warning"
            elif client["buffer_health"] < 30 or client["latency"] > 800:
                client["connection_status"] = "poor_network"
            else:
                client["connection_status"] = "connected"

        # 检查伴侣连接
        for companion_id, companion in self.companions.items():
            if current_time - companion["last_seen"] > 30:  # 30秒警告
                companion["connection_status"] = "warning"
            elif companion.get("signal_strength", 100) < 30:
                companion["connection_status"] = "poor_signal"
            else:
                companion["connection_status"] = "connected"

    def get_companion_status(self, companion_id):
        """获取伴侣状态"""
        with self.lock:
            if companion_id in self.companions:
                return self.companions[companion_id].copy()
            return None

    def get_client_status(self, client_id):
        """获取客户端状态"""
        with self.lock:
            if client_id in self.clients:
                return self.clients[client_id].copy()
            return None

    async def handle_video_frame(self, companion_id, message):
        """处理来自ROS2桥接节点的视频帧"""
        try:
            with self.lock:
                if companion_id not in self.companion_to_clients:
                    logger.debug(f"伴侣 {companion_id} 没有连接的客户端")
                    return
                
                # 获取连接到此伴侣的所有客户端
                client_ids = list(self.companion_to_clients[companion_id])
                
            if not client_ids:
                return
                
            # 为每个客户端转发视频帧
            import asyncio
            tasks = []
            
            for client_id in client_ids:
                if client_id in self.clients:
                    client = self.clients[client_id]
                    ws = client["websocket"]
                    
                    # 根据客户端质量设置调整消息
                    current_preset = client.get("current_preset", "medium")
                    
                    # 准备发送给客户端的消息
                    client_message = {
                        "type": "video_frame",
                        "robot_id": companion_id,
                        "frame_data": message.get("frame_data"),
                        "sequence": message.get("sequence", 0),
                        "timestamp": message.get("timestamp", 0),
                        "width": message.get("width", 640),
                        "height": message.get("height", 480),
                        "quality": message.get("quality", 80),
                        "server_timestamp": int(time.time() * 1000),
                        "quality_preset": current_preset
                    }
                    
                    # 异步发送消息
                    task = self._send_frame_to_client(client_id, ws, client_message)
                    tasks.append(task)
            
            # 并发发送给所有客户端
            if tasks:
                await asyncio.gather(*tasks, return_exceptions=True)
                logger.debug(f"视频帧已转发给 {len(tasks)} 个客户端")
                
        except Exception as e:
            logger.error(f"处理视频帧失败: {e}")
    
    async def _send_frame_to_client(self, client_id, websocket, message):
        """向单个客户端发送视频帧"""
        try:
            if hasattr(websocket, 'send_json'):
                await websocket.send_json(message)
            else:
                # 兼容同步WebSocket
                import json
                websocket.send(json.dumps(message))
                
        except Exception as e:
            logger.error(f"向客户端 {client_id} 发送视频帧失败: {e}")
            # 标记客户端连接有问题
            with self.lock:
                if client_id in self.clients:
                    self.clients[client_id]["connection_status"] = "connection_error"

    def shutdown(self):
        """关闭管理器"""
        self.running = False
        if self.monitor_thread.is_alive():
            self.monitor_thread.join(timeout=5)
        logger.info("伴侣自适应视频管理器已关闭")