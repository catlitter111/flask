# ROS2 WebSocket桥接系统使用指南

## 系统概述

这个WebSocket桥接系统连接ROS2跟踪节点与微信小程序，实现实时图像传输和跟踪数据共享。

```
┌─────────────────┐    ROS2话题    ┌──────────────────┐    WebSocket    ┌─────────────────┐    WebSocket    ┌─────────────────┐
│   ByteTracker   │────────────────→│ WebSocket桥接节点 │─────────────────→│   Flask服务器    │─────────────────→│   微信小程序     │
│      节点       │                │                  │                 │                 │                 │                 │
└─────────────────┘                └──────────────────┘                 └─────────────────┘                 └─────────────────┘
```

## 文件结构

```
flask/
├── src/
│   ├── custom_msgs/
│   │   └── msg/
│   │       ├── TrackingResult.msg     # 跟踪结果消息类型
│   │       └── RobotStatus.msg        # 机器人状态消息类型
│   └── following_robot/
│       ├── following_robot/
│       │   ├── bytetracker_node.py           # 修改后的跟踪节点
│       │   └── websocket_bridge_node.py      # WebSocket桥接节点
│       ├── launch/
│       │   ├── websocket_bridge.launch.py   # 桥接节点启动文件
│       │   └── full_system.launch.py        # 完整系统启动文件
│       └── config/
│           └── websocket_bridge_config.yaml # 配置文件
├── 服务端/
│   ├── server.py                    # 修改后的Flask服务器
│   └── adaptive_video_manager.py    # 视频管理器
└── self_following_miniproam/        # 修改后的微信小程序
    └── app.js                       # 小程序主文件
```

## 快速开始

### 1. 编译ROS2包

```bash
# 进入工作空间
cd flask/src

# 编译自定义消息
colcon build --packages-select custom_msgs

# 编译跟踪包
colcon build --packages-select following_robot

# 加载环境
source install/setup.bash
```

### 2. 启动系统

#### 方式一：完整系统启动（推荐）

```bash
# 启动完整系统（包括bytetracker和websocket桥接）
ros2 launch following_robot full_system.launch.py

# 可选参数：
ros2 launch following_robot full_system.launch.py \
    websocket_host:=192.168.1.100 \
    robot_id:=robot_001 \
    image_quality:=medium
```

#### 方式二：分别启动

```bash
# 1. 启动ByteTracker节点
ros2 run following_robot bytetracker_node

# 2. 启动WebSocket桥接节点
ros2 run following_robot websocket_bridge_node

# 或使用launch文件
ros2 launch following_robot websocket_bridge.launch.py
```

### 3. 启动服务器

```bash
cd flask/服务端
python server.py
```

### 4. 使用微信小程序

在微信小程序中，系统会自动连接到服务器并接收图像和跟踪数据。

## 配置参数

### WebSocket桥接节点参数

在 `config/websocket_bridge_config.yaml` 中配置：

```yaml
server:
  host: "172.20.39.181"      # 服务器地址
  port: 1234                 # 服务器端口

robot:
  id: "robot_001"            # 机器人ID

image:
  quality: 80                # 图像质量 (1-100)
  max_width: 640             # 最大宽度
  max_height: 480            # 最大高度
```

### Launch文件参数

```bash
# 可用参数
ros2 launch following_robot websocket_bridge.launch.py --show-args

# 示例使用：
ros2 launch following_robot websocket_bridge.launch.py \
    websocket_host:=192.168.1.100 \
    websocket_port:=8080 \
    robot_id:=my_robot \
    image_quality:=high \
    frame_rate:=15
```

## 数据流说明

### 1. 跟踪结果数据流

```
ByteTracker节点 → /bytetracker/tracking_result → WebSocket桥接节点 → 服务器 → 微信小程序
```

**数据格式：**
```json
{
  "type": "tracking_result",
  "robot_id": "robot_001",
  "timestamp": 1640995200000,
  "mode": "single_target",
  "total_tracks": 3,
  "target_detected": true,
  "target_x": 320.5,
  "target_y": 240.0,
  "target_width": 80.0,
  "target_height": 120.0,
  "confidence": 0.85,
  "distance": 2.5,
  "tracking_status": "tracking",
  "fps": 25.0
}
```

### 2. 视频数据流

```
ByteTracker节点 → /bytetracker/visualization → WebSocket桥接节点 → 服务器 → 微信小程序
```

**数据格式：**
```json
{
  "type": "video_frame",
  "robot_id": "robot_001",
  "timestamp": 1640995200000,
  "image_data": "base64编码的图像数据",
  "width": 640,
  "height": 480,
  "encoding": "jpg",
  "quality": 80
}
```

### 3. 控制命令流

```
微信小程序 → 服务器 → WebSocket桥接节点 → /robot_control/command → 机器人控制节点
```

## 故障排除

### 常见问题

#### 1. WebSocket连接失败

**症状：** 桥接节点报告连接失败
**解决方案：**
```bash
# 检查服务器是否运行
curl http://172.20.39.181:1234/health

# 检查网络连接
ping 172.20.39.181

# 检查防火墙设置
sudo ufw status
```

#### 2. 图像传输延迟

**症状：** 小程序中视频延迟较大
**解决方案：**
- 降低图像质量：修改 `image.quality` 为较低值
- 增加跳帧：设置 `image.skip_frames` 为 2 或更高
- 检查网络带宽

#### 3. 跟踪数据丢失

**症状：** 小程序收不到跟踪数据
**解决方案：**
```bash
# 检查ROS2话题
ros2 topic list | grep bytetracker
ros2 topic echo /bytetracker/tracking_result

# 检查节点状态
ros2 node list
ros2 node info /websocket_bridge_node
```

#### 4. 消息类型错误

**症状：** 编译或运行时报告消息类型未找到
**解决方案：**
```bash
# 重新编译自定义消息
cd flask/src
colcon build --packages-select custom_msgs --force-cmake-configure
source install/setup.bash

# 检查消息类型
ros2 interface list | grep custom_msgs
```

### 日志查看

```bash
# 查看节点日志
ros2 run following_robot websocket_bridge_node --ros-args --log-level DEBUG

# 查看系统日志
journalctl -f -u your_service_name

# 查看launch文件日志
ros2 launch following_robot full_system.launch.py --ros-args --log-level DEBUG
```

## 性能优化

### 1. 图像传输优化

```yaml
# 在配置文件中调整
image:
  quality: 60                # 降低质量以减少带宽
  max_width: 480             # 降低分辨率
  skip_frames: 2             # 跳帧传输
  
transmission:
  enable_compression: true   # 启用压缩
  compression_level: 8       # 高压缩级别
```

### 2. 网络优化

```yaml
transmission:
  batch_size: 3              # 批量传输
  max_queue_size: 50         # 较小队列避免积压
  drop_policy: "drop_oldest" # 丢弃旧数据
```

### 3. 系统监控

```bash
# 监控网络使用
iftop -i eth0

# 监控CPU和内存
htop

# 监控ROS2性能
ros2 topic hz /bytetracker/visualization
ros2 topic bw /bytetracker/tracking_result
```

## API参考

### WebSocket桥接节点

**话题订阅：**
- `/bytetracker/tracking_result` (TrackingResult)
- `/bytetracker/visualization` (Image)
- `/robot_status` (RobotStatus)

**话题发布：**
- `/robot_control/command` (String)

**服务：**
- 无

**参数：**
- `websocket_host`: 服务器地址
- `websocket_port`: 服务器端口
- `robot_id`: 机器人标识
- `config_file`: 配置文件路径

### 服务器端点

**WebSocket端点：**
- `/ws/ros2_bridge/{robot_id}`: ROS2桥接连接
- `/ws/companion/{client_id}`: 微信小程序连接

**HTTP端点：**
- `/health`: 健康检查
- `/api/companion/{robot_id}/status`: 机器人状态
- `/api/companion/{robot_id}/history/{date}`: 历史数据

## 开发指南

### 扩展功能

1. **添加新的消息类型：**
   - 在 `custom_msgs/msg/` 中定义新消息
   - 更新 `CMakeLists.txt`
   - 重新编译包

2. **添加新的控制命令：**
   - 在WebSocket桥接节点中添加处理逻辑
   - 在服务器端添加转发逻辑
   - 在小程序中添加UI控制

3. **优化图像处理：**
   - 实现自适应质量调整
   - 添加图像预处理功能
   - 支持多种编码格式

### 调试工具

```bash
# WebSocket连接测试
wscat -c ws://172.20.39.181:1234/ws/ros2_bridge/test_robot

# 图像数据测试
ros2 topic pub /test_image sensor_msgs/Image ...

# 性能测试
ros2 run following_robot websocket_bridge_node --ros-args -p enable_profiling:=true
```

## 部署指南

### 生产环境部署

1. **系统要求：**
   - Ubuntu 22.04 LTS
   - ROS2 Humble
   - Python 3.8+
   - 至少 4GB RAM

2. **网络配置：**
   - 确保端口1234开放
   - 配置防火墙规则
   - 设置静态IP地址

3. **服务化部署：**
```bash
# 创建systemd服务文件
sudo nano /etc/systemd/system/websocket-bridge.service

# 启用服务
sudo systemctl enable websocket-bridge.service
sudo systemctl start websocket-bridge.service
```

### 容器化部署

```dockerfile
# Dockerfile示例
FROM ros:humble-ros-base

# 安装依赖
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-websockets

# 复制代码
COPY src/ /ros2_ws/src/

# 编译
RUN cd /ros2_ws && colcon build

# 启动脚本
CMD ["ros2", "launch", "following_robot", "full_system.launch.py"]
```

## 版本更新

### v1.0.0 (当前版本)
- 基础WebSocket桥接功能
- 图像和跟踪数据传输
- 微信小程序集成

### 计划功能
- [ ] 双向音频传输
- [ ] 多机器人支持
- [ ] 云端数据存储
- [ ] AI语音交互

## 联系支持

如遇到问题，请：
1. 查看本文档的故障排除部分
2. 检查系统日志和错误信息
3. 提交Issue并附上详细的错误日志 