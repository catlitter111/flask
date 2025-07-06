# WebRTC视频传输配置说明

## 📡 概述

本项目已升级为**优先使用WebRTC**进行视频传输的系统，WebSocket作为备用传输方式，确保在各种网络环境下都能正常工作。

## 🔧 系统架构

### 传输优先级
1. **WebRTC传输**（主要方式）- 低延迟，高质量
2. **WebSocket传输**（备用方式）- 兼容性好，稳定性高
3. **自动降级机制** - 当WebRTC不可用时自动切换

### 三端修改概述

#### 1. 🤖 ROS2端（websocket_bridge_node.py）
- **优先WebRTC传输**：默认使用WebRTC推送视频帧
- **增强编码质量**：WebRTC模式使用90%质量编码
- **智能降级**：WebRTC失败时自动使用WebSocket备用
- **传输状态监控**：实时监控传输成功率

#### 2. 🖥️ 服务端（server.py）
- **WebRTC信令处理**：优化信令服务器功能
- **视频轨道管理**：为每个机器人创建独立的视频轨道
- **多路复用**：支持多个客户端同时观看同一机器人
- **性能优化**：使用uvloop提升异步性能

#### 3. 📱 微信小程序端（control.js）
- **默认WebRTC模式**：启动时自动尝试WebRTC连接
- **live-player组件**：使用微信原生WebRTC播放器
- **智能降级**：连接失败时自动切换到image组件
- **状态监控**：实时显示传输模式和连接状态

## 🚀 快速开始

### 1. 安装依赖
```bash
# 安装基础依赖
pip install -r requirements_webrtc.txt

# 如果某些包安装失败，可以单独安装核心依赖
pip install websockets aiohttp pillow uvloop aiofiles
```

### 2. 启动服务
```bash
# 启动服务端
cd 服务端
python server.py

# 启动ROS2桥接节点
cd src/following_robot
ros2 run following_robot websocket_bridge_node
```

### 3. 微信小程序配置
微信小程序会自动尝试WebRTC连接，无需额外配置。

## 📊 传输模式说明

### WebRTC模式特点
- ✅ **低延迟**：通常延迟 < 100ms
- ✅ **高质量**：90%JPEG质量编码
- ✅ **实时性**：P2P直连，减少服务器负载
- ❌ **兼容性**：依赖平台WebRTC支持

### WebSocket模式特点
- ✅ **兼容性好**：支持所有平台
- ✅ **稳定性高**：经过充分测试
- ✅ **灵活性**：可动态调整质量和帧率
- ❌ **延迟较高**：通常延迟 200-500ms

## 🔧 配置选项

### ROS2端配置
```yaml
# 在launch文件中设置
enable_webrtc: true          # 启用WebRTC功能
webrtc_priority: true        # 优先使用WebRTC
websocket_fallback: true     # 允许WebSocket降级
webrtc_quality: 90          # WebRTC编码质量
```

### 微信小程序配置
```javascript
// 在control.js中可以修改
data: {
  webrtcEnabled: true,       // 启用WebRTC功能
  useWebRTC: true,          // 默认使用WebRTC
  fallbackToWebSocket: true, // 允许降级到WebSocket
  webrtcStreamUrl: ''       // WebRTC流地址
}
```

## 🛠️ 故障排除

### 常见问题

#### 1. WebRTC连接失败
**现象**：小程序显示"已切换到备用传输"
**解决**：
- 检查服务器防火墙设置
- 确认WebRTC端口（1236）已开放
- 检查网络环境是否支持WebRTC

#### 2. 视频质量较低
**现象**：图像模糊或帧率低
**解决**：
- 检查网络带宽
- 调整编码质量参数
- 确认使用的传输模式

#### 3. 频繁切换传输模式
**现象**：WebRTC和WebSocket之间频繁切换
**解决**：
- 检查网络稳定性
- 调整降级触发条件
- 考虑固定使用WebSocket模式

### 调试方法

#### 1. 查看ROS2日志
```bash
ros2 run following_robot websocket_bridge_node --ros-args --log-level debug
```

#### 2. 查看服务端日志
```bash
# 服务端会输出详细的WebRTC连接信息
python server.py
```

#### 3. 查看小程序日志
在微信开发者工具中查看Console输出，包含详细的连接状态信息。

## 📈 性能优化

### WebRTC优化建议
1. **网络环境**：确保低延迟、高带宽网络
2. **服务器配置**：使用SSD硬盘，充足内存
3. **编码参数**：根据网络情况调整质量
4. **连接池**：复用WebRTC连接

### 备用传输优化
1. **自适应质量**：根据网络情况动态调整
2. **帧率控制**：避免过高的帧率导致卡顿
3. **压缩优化**：使用合适的JPEG压缩参数

## 🔮 未来计划

1. **H.264编码**：支持硬件加速编码
2. **多路推流**：支持多个机器人同时推流
3. **录制功能**：支持视频录制和回放
4. **移动端优化**：针对移动设备进行优化

## 📞 技术支持

如果遇到问题，请：
1. 查看本文档的故障排除部分
2. 检查系统日志输出
3. 确认网络环境配置
4. 联系技术支持团队

---

**更新日期**：2025年1月  
**版本**：v2.0 WebRTC版本 