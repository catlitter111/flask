# RFID Reader ROS2 Package

基于原有RFID_test.py程序开发的ROS2 Humble RFID读写器功能包，提供完整的RFID标签检测、监控和数据发布功能。

## 功能特性

- **实时标签检测**: 持续检测RFID标签并发布详细信息
- **多标签管理**: 支持同时跟踪多个RFID标签
- **ROS2集成**: 完整的ROS2话题、服务和参数支持
- **状态监控**: 实时发布读写器连接状态和统计信息
- **配置灵活**: 支持参数配置和launch文件启动

## 消息类型

### RfidTag
单个RFID标签信息
```
std_msgs/Header header
string epc               # 电子产品码
uint8[] pc               # 协议控制位
int8 rssi_dbm           # 信号强度 (dBm)
uint8 rssi_raw          # 原始RSSI值
uint8 antenna_id        # 天线ID
uint8 freq_param        # 频点参数
builtin_interfaces/Time first_seen    # 首次检测时间
builtin_interfaces/Time last_seen     # 最后检测时间
uint32 read_count       # 读取次数
```

### RfidTagArray
标签数组信息
```
std_msgs/Header header
RfidTag[] tags          # 检测到的标签列表
uint32 total_tags       # 总标签数量
uint32 total_reads      # 总读取次数
```

### RfidReaderStatus
读写器状态信息
```
std_msgs/Header header
bool connected          # 连接状态
string reader_ip        # 读写器IP地址
uint16 reader_port      # 读写器端口
uint8 antenna_id        # 当前工作天线ID
uint16 read_rate        # 读取速率 (次/秒)
uint32 total_read       # 总读取次数
builtin_interfaces/Time session_start_time  # 会话开始时间
float64 session_duration # 会话持续时间 (秒)
```

## 服务

### RfidCommand
控制读写器操作
```
# 请求
string command          # 命令: "start_inventory", "stop_inventory", "set_antenna", "get_status"
uint8 antenna_id        # 天线ID (用于set_antenna命令)
string ip_address       # IP地址
uint16 port             # 端口

# 响应
bool success            # 命令执行结果
string message          # 结果消息
RfidReaderStatus status # 当前状态
```

## 话题

### 发布话题
- `/rfid/tag_detected` (RfidTag) - 新检测到的标签
- `/rfid/tags` (RfidTagArray) - 所有当前标签
- `/rfid/status` (RfidReaderStatus) - 读写器状态

### 服务
- `/rfid/command` (RfidCommand) - 控制命令服务

## 构建和安装

1. 确保在ROS2 Humble环境中
```bash
source /opt/ros/humble/setup.bash
```

2. 构建功能包
```bash
cd /userdata/try_again/SelfFollowingROS2
colcon build --packages-select rfid_reader
source install/setup.bash
```

## 使用方法

### 基本启动
```bash
# 使用默认参数启动
ros2 launch rfid_reader rfid_reader.launch.py

# 指定读写器IP地址
ros2 launch rfid_reader rfid_reader.launch.py reader_ip:=192.168.1.100

# 指定其他参数
ros2 launch rfid_reader rfid_reader.launch.py reader_ip:=192.168.1.100 antenna_id:=2 auto_start:=false
```

### 调试模式
```bash
# 启动调试模式（手动控制）
ros2 launch rfid_reader rfid_debug.launch.py
```

### 使用配置文件
```bash
# 使用预定义配置
ros2 run rfid_reader rfid_reader_node.py --ros-args --params-file /path/to/config/rfid_reader_params.yaml
```

### 测试客户端
```bash
# 启动测试客户端进行交互测试
ros2 run rfid_reader rfid_test_client.py
```

## 命令行控制

### 服务调用示例
```bash
# 开始盘存
ros2 service call /rfid/command rfid_reader/srv/RfidCommand "{command: 'start_inventory'}"

# 停止盘存
ros2 service call /rfid/command rfid_reader/srv/RfidCommand "{command: 'stop_inventory'}"

# 获取状态
ros2 service call /rfid/command rfid_reader/srv/RfidCommand "{command: 'get_status'}"

# 设置天线
ros2 service call /rfid/command rfid_reader/srv/RfidCommand "{command: 'set_antenna', antenna_id: 2}"
```

### 话题监控
```bash
# 查看所有标签
ros2 topic echo /rfid/tags

# 查看新检测标签
ros2 topic echo /rfid/tag_detected

# 查看状态信息
ros2 topic echo /rfid/status

# 查看话题列表
ros2 topic list | grep rfid
```

## 参数配置

### 主要参数
- `reader_ip`: RFID读写器IP地址 (默认: "192.168.0.178")
- `reader_port`: 读写器端口 (默认: 4001)
- `antenna_id`: 工作天线ID (默认: 1)
- `publish_rate`: 发布频率 Hz (默认: 1.0)
- `auto_start`: 自动开始盘存 (默认: true)

### 配置文件
配置文件位于 `config/` 目录:
- `rfid_reader_params.yaml` - 生产环境配置
- `rfid_debug_params.yaml` - 调试环境配置

## 故障排除

### 常见问题

1. **连接失败**
   - 检查读写器IP地址和端口
   - 确认网络连通性
   - 验证读写器是否开启并监听指定端口

2. **无法检测标签**
   - 确认天线连接正常
   - 检查标签是否在读取范围内
   - 验证读写器工作频率

3. **构建失败**
   - 确保ROS2环境正确source
   - 检查依赖包是否安装完整
   - 验证Python3环境

### 调试技巧
```bash
# 查看节点日志
ros2 run rfid_reader rfid_reader_node.py --ros-args --log-level DEBUG

# 检查节点状态
ros2 node info /rfid_reader

# 监控网络连接
ping 192.168.0.178
telnet 192.168.0.178 4001
```

## 与其他系统集成

本功能包可以轻松集成到现有的ROS2系统中，例如：
- 机器人导航系统
- 仓储管理系统
- 人员跟踪系统

通过订阅 `/rfid/tags` 话题，其他节点可以获取实时的RFID标签信息并做出相应决策。

## 技术规格

- **ROS2版本**: Humble
- **Python版本**: 3.8+
- **支持的读写器**: 基于TCP Socket通信的RFID读写器
- **通信协议**: 自定义二进制协议（基于原RFID_test.py）
- **最大标签数**: 无限制（受内存限制）
- **支持频率**: 915MHz等（取决于硬件）

## 许可证

MIT License