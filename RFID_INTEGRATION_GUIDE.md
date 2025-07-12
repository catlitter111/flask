# RFID监控系统集成指南

## 功能概述

已成功为你的微信小程序添加了RFID监控功能，支持以下特性：

### 📡 RFID功能特性
- **实时标签检测**: 检测范围内的RFID标签
- **盘存控制**: 远程开启/关闭RFID盘存
- **状态监控**: 实时显示读写器连接状态和读取统计
- **信号质量**: 显示标签信号强度和质量等级
- **天线控制**: 切换不同的RFID天线

## 🏗️ 系统架构

```
微信小程序 ↔ 服务端 ↔ WebSocket桥接节点 ↔ RFID节点
```

### 消息流转
1. **小程序 → 服务端**: 发送RFID控制命令
2. **服务端 → WebSocket桥接**: 转发命令到ROS2系统
3. **WebSocket桥接 → RFID节点**: 调用RFID服务接口
4. **RFID节点 → WebSocket桥接**: 发布标签数据和状态
5. **WebSocket桥接 → 服务端**: 转发RFID数据
6. **服务端 → 小程序**: 显示实时数据

## 📋 RFID消息类型

### 小程序发送的命令
```javascript
// 开始盘存
{
  type: 'rfid_command',
  robot_id: 'companion_robot_001',
  command: 'start_inventory',
  params: {},
  timestamp: 1234567890
}

// 停止盘存  
{
  type: 'rfid_command',
  robot_id: 'companion_robot_001',
  command: 'stop_inventory',
  params: {},
  timestamp: 1234567890
}

// 设置天线
{
  type: 'rfid_command',
  robot_id: 'companion_robot_001', 
  command: 'set_antenna',
  params: { antenna_id: 2 },
  timestamp: 1234567890
}
```

### 小程序接收的数据

#### 1. RFID标签数据
```javascript
{
  type: 'rfid_tags_data',
  robot_id: 'companion_robot_001',
  timestamp: 1234567890,
  data: {
    total_tags: 5,
    total_reads: 123,
    status: '检测中',
    tags: [
      {
        epc: '3000123456789ABC',
        rssi_dbm: -45,
        rssi_raw: 180,
        antenna_id: 1,
        read_count: 12,
        signal_quality: '强',
        first_seen: { sec: 1234567890, nanosec: 0 },
        last_seen: { sec: 1234567890, nanosec: 0 }
      }
    ]
  }
}
```

#### 2. RFID状态更新
```javascript
{
  type: 'rfid_status_update',
  robot_id: 'companion_robot_001',
  timestamp: 1234567890,
  data: {
    connected: true,
    inventory_active: true,
    reader_info: {
      ip: '192.168.0.178',
      port: 4001,
      antenna_id: 1
    },
    session_info: {
      duration: 120.5,
      start_time: { sec: 1234567890, nanosec: 0 }
    },
    statistics: {
      read_rate: 15,
      total_reads: 1500
    },
    status_text: '盘存中'
  }
}
```

#### 3. 新标签检测
```javascript
{
  type: 'rfid_tag_detected',
  robot_id: 'companion_robot_001',
  timestamp: 1234567890,
  data: {
    epc: '3000123456789ABC',
    rssi_dbm: -45,
    signal_quality: '强',
    antenna_id: 1,
    read_count: 1,
    detection_time: { sec: 1234567890, nanosec: 0 }
  }
}
```

#### 4. 命令响应
```javascript
{
  type: 'rfid_command_response',
  command: 'start_inventory',
  status: 'success',
  message: '盘存已启动',
  rfid_status: {
    connected: true,
    reader_ip: '192.168.0.178',
    reader_port: 4001,
    antenna_id: 1,
    session_duration: 0.0
  },
  timestamp: 1234567890
}
```

## 🔧 部署和配置

### 1. 构建消息包
```bash
# 构建RFID消息包
colcon build --packages-select rfid_reader

# 构建跟踪系统（包含WebSocket桥接）
colcon build --packages-select following_robot

# 加载环境
source install/setup.bash
```

### 2. 启动RFID节点
```bash
# 启动RFID读写器节点
ros2 run rfid_reader rfid_reader_node

# 或使用launch文件
ros2 launch rfid_reader rfid_reader.launch.py
```

### 3. 启动WebSocket桥接
```bash
# 启动WebSocket桥接节点（包含RFID支持）
ros2 run following_robot websocket_bridge_node
```

### 4. 启动服务端
```bash
# 启动WebSocket服务器
cd 服务端
python3 server.py
```

## 🧪 测试验证

### 自动化测试
```bash
# 运行RFID集成测试
python3 test_rfid_integration.py
```

### 手动测试步骤
1. **连接验证**: 确认小程序能连接到服务端
2. **状态查询**: 发送`get_rfid_status`命令检查初始状态
3. **开始盘存**: 发送`start_inventory`命令
4. **监控数据**: 观察是否收到标签数据和状态更新
5. **停止盘存**: 发送`stop_inventory`命令
6. **天线切换**: 测试`set_antenna`命令

## 📱 微信小程序集成

### 页面结构建议
```
RFID监控页面
├── 状态显示区
│   ├── 连接状态指示器
│   ├── 盘存状态开关
│   └── 读写器信息
├── 统计信息区
│   ├── 总标签数
│   ├── 总读取次数
│   └── 会话时长
├── 标签列表区
│   ├── EPC编码
│   ├── 信号强度
│   ├── 信号质量
│   └── 读取次数
└── 控制按钮区
    ├── 开始/停止盘存
    ├── 天线选择
    └── 刷新状态
```

### 关键代码示例
```javascript
// WebSocket消息处理
onMessage(event) {
  const data = JSON.parse(event.data);
  
  switch(data.type) {
    case 'rfid_tags_data':
      this.updateTagsList(data.data.tags);
      this.updateStatistics(data.data);
      break;
      
    case 'rfid_status_update':
      this.updateStatus(data.data);
      break;
      
    case 'rfid_tag_detected':
      this.addNewTag(data.data);
      this.showNewTagNotification(data.data);
      break;
      
    case 'rfid_command_response':
      this.handleCommandResponse(data);
      break;
  }
}

// 发送RFID命令
sendRfidCommand(command, params = {}) {
  const message = {
    type: 'rfid_command',
    robot_id: this.robotId,
    command: command,
    params: params,
    timestamp: Date.now()
  };
  
  this.websocket.send(JSON.stringify(message));
}
```

## 🔍 信号质量等级

| RSSI范围 (dBm) | 质量等级 | 说明 |
|---------------|----------|------|
| ≥ -40 | 极强 | 标签非常接近读写器 |
| -40 ~ -55 | 强 | 信号强度良好 |
| -55 ~ -70 | 中等 | 正常读取范围 |
| -70 ~ -85 | 弱 | 可能出现读取不稳定 |
| < -85 | 极弱 | 标签接近读取边界 |

## ⚠️ 注意事项

1. **网络延迟**: RFID数据更新频率较高，注意处理网络延迟
2. **消息量控制**: 可根据需要调整数据发送频率
3. **错误处理**: 实现RFID设备断连重连逻辑
4. **用户体验**: 添加加载状态和错误提示
5. **数据持久化**: 考虑重要标签数据的本地存储

## 🐛 故障排除

### 常见问题
1. **RFID节点无法启动**: 检查设备连接和IP配置
2. **无法收到标签数据**: 确认盘存已启动且有标签在范围内
3. **WebSocket连接失败**: 检查服务端状态和网络连通性
4. **命令无响应**: 确认RFID服务可用性

### 调试方法
```bash
# 查看RFID节点日志
ros2 topic echo /rfid/tags
ros2 topic echo /rfid/status

# 查看WebSocket桥接日志  
ros2 run following_robot websocket_bridge_node --ros-args --log-level DEBUG

# 测试RFID服务
ros2 service call /rfid/command rfid_reader/srv/RfidCommand "{command: 'get_status'}"
```

## 📞 技术支持

如遇到问题，请检查：
1. ROS2环境是否正确配置
2. 所有依赖包是否已安装
3. RFID硬件设备是否正常连接
4. 网络连接是否稳定

现在你的微信小程序已经具备完整的RFID监控功能！