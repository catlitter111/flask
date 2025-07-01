# WebSocket桥接节点与机器人控制节点集成指南

## 概述

本文档详细说明了WebSocket桥接节点(`websocket_bridge_node`)和机器人控制节点(`robot_control_node`)之间的集成，实现了从微信小程序到机器人的完整控制链路。

## 系统架构

```
微信小程序 → 服务器(server.py) → WebSocket桥接节点 → 机器人控制节点 → 底层驱动节点 → 小车硬件
```

### 通信流程

1. **微信小程序** 发送控制命令到服务器
2. **服务器** 通过WebSocket转发命令到桥接节点
3. **WebSocket桥接节点** 解析命令并发布到ROS话题
4. **机器人控制节点** 接收命令并转换为速度控制
5. **底层驱动节点** (turn_on_dlrobot_robot) 通过串口控制硬件

### 节点架构

| 节点名称 | 功能 | 输入 | 输出 |
|---------|------|------|------|
| `websocket_bridge_node` | WebSocket通信桥接 | WebSocket消息 | `/robot_control/command` |
| `robot_control_node` | 高级控制逻辑 | `/robot_control/command` | `/cmd_vel` |
| `dlrobot_robot_node` | 底层硬件驱动 | `/cmd_vel` | 串口通信 |
| `bytetracker_node` | 人体跟踪 | 摄像头数据 | 跟踪结果 |

## 话题通信

### 主要话题

| 话题名称 | 消息类型 | 方向 | 说明 |
|---------|---------|------|------|
| `/robot_control/command` | `std_msgs/String` | WebSocket桥接节点 → 机器人控制节点 | 控制命令传递 |
| `/robot_control/status` | `std_msgs/String` | 机器人控制节点 → 外部 | 机器人状态信息 |
| `/robot_control/mode` | `std_msgs/String` | 机器人控制节点 → 外部 | 控制模式状态 |
| `/cmd_vel` | `geometry_msgs/Twist` | 机器人控制节点 → 底层驱动 | 速度控制命令 |
| `/ackermann_cmd` | `ackermann_msgs/AckermannDriveStamped` | 转换节点 → 底层驱动 | 阿克曼控制命令 |
| `/imu` | `sensor_msgs/Imu` | 底层驱动 → 外部 | IMU传感器数据 |
| `/odom` | `nav_msgs/Odometry` | 底层驱动 → 外部 | 里程计数据 |

## 支持的控制命令

### 电机控制命令

| 命令 | 格式 | 说明 | 示例 |
|------|------|------|------|
| 前进 | `motor_forward[:speed]` | 电机前进，可选速度参数(0-100) | `motor_forward:50` |
| 后退 | `motor_backward[:speed]` | 电机后退，可选速度参数(0-100) | `motor_backward:30` |
| 左转 | `motor_left[:speed]` | 电机左转，可选速度参数(0-100) | `motor_left:40` |
| 右转 | `motor_right[:speed]` | 电机右转，可选速度参数(0-100) | `motor_right:60` |
| 停止 | `motor_stop` | 电机停止 | `motor_stop` |

### 模式切换命令

| 命令 | 说明 | 模式变化 |
|------|------|----------|
| `start_auto_mode` | 启动自动跟随模式 | → FOLLOWING |
| `pause_auto_mode` | 暂停自动模式 | → STOP |
| `start_interaction` | 启动交互模式 | → INTERACTION |
| `stop_interaction` | 停止交互模式 | → MANUAL |

### 伴侣交互命令

| 命令 | 说明 |
|------|------|
| `companion_look_up` | 伴侣向上看 |
| `companion_look_down` | 伴侣向下看 |
| `companion_turn_left` | 伴侣左转 |
| `companion_turn_right` | 伴侣右转 |
| `companion_stop` | 伴侣停止 |

### 配置命令

| 命令 | 格式 | 说明 | 示例 |
|------|------|------|------|
| 设置电机速度 | `set_motor_speed:value` | 设置默认电机速度(0-100) | `set_motor_speed:75` |
| 切换控制类型 | `switch_control_type:type` | 切换控制类型(motor/companion) | `switch_control_type:companion` |
| 紧急停止 | `emergency_stop` | 激活紧急停止 | `emergency_stop` |

## 控制模式

机器人控制节点支持以下控制模式：

### 1. MANUAL (手动控制)
- 接收WebSocket的手动控制命令
- 直接控制电机运动
- 默认模式

### 2. FOLLOWING (跟随模式)
- 自动跟随检测到的人体目标
- 基于人体位置调整运动
- 支持距离和角度控制

### 3. INTERACTION (交互模式)
- 伴侣机器人交互功能
- 支持头部/相机控制
- 自然交互响应

### 4. STOP (停止模式)
- 所有运动停止
- 安全模式
- 紧急情况使用

## 启动系统

### 使用launch文件启动完整系统

```bash
# 启动完整系统（包含所有节点）
ros2 launch following_robot full_system.launch.py

# 自定义WebSocket服务器地址
ros2 launch following_robot full_system.launch.py websocket_host:=192.168.1.100

# 启用阿克曼转向控制
ros2 launch following_robot full_system.launch.py use_ackermann:=true

# 自定义串口设备
ros2 launch following_robot full_system.launch.py serial_port:=/dev/ttyUSB0

# 完整自定义启动
ros2 launch following_robot full_system.launch.py \
    websocket_host:=192.168.1.100 \
    websocket_port:=8080 \
    robot_id:=my_robot_001 \
    image_quality:=60 \
    use_ackermann:=true \
    serial_port:=/dev/ttyUSB0 \
    serial_baud:=115200
```

### 单独启动节点（用于调试）

```bash
# 启动机器人控制节点
ros2 run following_robot robot_control_node

# 启动WebSocket桥接节点
ros2 run following_robot websocket_bridge_node

# 启动ByteTracker节点
ros2 run following_robot bytetracker_node

# 启动底层驱动节点（普通模式）
ros2 run turn_on_dlrobot_robot dlrobot_robot_node

# 启动底层驱动节点（阿克曼模式）
ros2 run turn_on_dlrobot_robot dlrobot_robot_node --ros-args -p akm_cmd_vel:=ackermann_cmd

# 启动Twist到Ackermann转换节点
ros2 run turn_on_dlrobot_robot cmd_vel_to_ackermann_drive.py

# 启动tank模式（使用原始launch文件）
ros2 launch turn_on_dlrobot_robot tank.launch.py
ros2 launch turn_on_dlrobot_robot tank.launch.py akmcar:=true
```

## 测试系统

### 运行集成测试

```bash
# 运行集成测试脚本
python3 test_websocket_robot_integration.py
```

### 手动测试命令

```bash
# 发送测试命令
ros2 topic pub /robot_control/command std_msgs/String "data: 'motor_forward:50'"

# 监听状态信息
ros2 topic echo /robot_control/status

# 监听模式变化
ros2 topic echo /robot_control/mode
```

## 配置参数

### WebSocket桥接节点参数

```yaml
websocket_host: "101.201.150.96"    # WebSocket服务器地址
websocket_port: 1234                # WebSocket服务器端口
robot_id: "companion_robot_001"     # 机器人唯一标识
image_quality: 80                   # 图像质量(1-100)
frame_rate: 15                      # 视频帧率
enable_image_stream: true           # 启用图像流
enable_status_report: true          # 启用状态上报
enable_command_receive: true        # 启用命令接收
```

### 机器人控制节点参数

```yaml
max_linear_speed: 0.5               # 最大线速度(m/s)
max_angular_speed: 1.0              # 最大角速度(rad/s)
min_follow_distance: 1.0            # 最小跟随距离(m)
max_follow_distance: 3.0            # 最大跟随距离(m)
follow_speed_factor: 0.3            # 跟随速度因子
wheelbase: 0.143                    # 轴距(m)
use_ackermann: false                # 是否使用阿克曼转向
safety_enabled: true                # 启用安全模式
```

### 底层驱动节点参数

```yaml
usart_port_name: "/dev/dlrobot_controller"  # 串口设备路径
serial_baud_rate: 115200            # 串口波特率
robot_frame_id: "base_footprint"    # 机器人坐标系
odom_frame_id: "odom_combined"      # 里程计坐标系
cmd_vel: "cmd_vel"                  # 速度话题名称
akm_cmd_vel: "none"                 # 阿克曼话题名称 (none/ackermann_cmd)
product_number: 0                   # 产品编号
```

### 启动参数

| 参数名称 | 默认值 | 说明 |
|---------|-------|------|
| `websocket_host` | `101.201.150.96` | WebSocket服务器地址 |
| `websocket_port` | `1234` | WebSocket服务器端口 |
| `robot_id` | `companion_robot_001` | 机器人唯一标识 |
| `use_ackermann` | `false` | 是否使用阿克曼转向控制 |
| `serial_port` | `/dev/dlrobot_controller` | 串口设备路径 |
| `serial_baud` | `115200` | 串口波特率 |
| `image_quality` | `80` | 图像质量(1-100) |
| `frame_rate` | `30` | 视频帧率 |

## 故障排除

### 常见问题

1. **WebSocket连接失败**
   - 检查服务器地址和端口
   - 确认服务器正在运行
   - 检查网络连接

2. **控制命令无响应**
   - 检查话题通信：`ros2 topic list`
   - 监听命令话题：`ros2 topic echo /robot_control/command`
   - 检查节点状态：`ros2 node list`

3. **电机不动作**
   - 检查硬件连接
   - 确认速度参数不为0
   - 检查紧急停止状态

4. **模式切换失败**
   - 监听模式话题：`ros2 topic echo /robot_control/mode`
   - 检查命令格式是否正确
   - 查看节点日志

5. **串口通信失败**
   - 检查串口设备是否存在：`ls -l /dev/dlrobot_controller`
   - 检查串口权限：`sudo chmod 666 /dev/dlrobot_controller`
   - 运行udev规则：`sudo bash dlrobot_udev.sh`
   - 检查串口波特率设置

6. **底层驱动节点异常**
   - 检查硬件连接
   - 查看驱动节点日志：`ros2 node info /dlrobot_robot_node`
   - 监听里程计数据：`ros2 topic echo /odom`
   - 监听IMU数据：`ros2 topic echo /imu`

### 调试方法

```bash
# 查看节点日志
ros2 run following_robot robot_control_node --ros-args --log-level DEBUG

# 监听所有相关话题
ros2 topic echo /robot_control/command
ros2 topic echo /robot_control/status
ros2 topic echo /robot_control/mode
ros2 topic echo /cmd_vel
ros2 topic echo /odom
ros2 topic echo /imu

# 检查节点信息
ros2 node info /robot_control_node
ros2 node info /websocket_bridge_node
ros2 node info /dlrobot_robot_node
```

## 扩展功能

### 添加新的控制命令

1. 在`websocket_bridge_node.py`的`handle_command`方法中添加命令处理
2. 在`robot_control_node.py`的`command_callback`方法中添加相应的执行逻辑
3. 更新本文档的命令表

### 自定义控制模式

1. 在`ControlMode`枚举中添加新模式
2. 实现模式切换逻辑
3. 添加模式特定的控制逻辑

## 性能优化

### 延迟优化
- 调整消息队列深度
- 优化命令处理频率
- 减少不必要的日志输出

### 带宽优化
- 调整图像质量参数
- 启用自适应质量控制
- 优化状态上报频率

## 安全考虑

- 紧急停止功能始终可用
- 速度限制确保安全运行
- 命令验证防止无效操作
- 超时保护避免失控

---

**最后更新：** 2024年
**维护者：** AI Assistant 