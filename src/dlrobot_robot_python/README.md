# DLRobot Python机器人底盘驱动包

## 📋 概述

这是DLRobot机器人底盘驱动的Python实现版本，基于原有的C++版本`turn_on_dlrobot_robot`重新开发。该包提供了完整的ROS2 Humble兼容的机器人底盘控制功能。

## ✨ 主要特性

- 🚗 **多模式驱动支持**: 普通驱动模式和阿克曼转向模式
- 📡 **完整传感器数据**: IMU、里程计、电压监测
- 🔄 **实时四元数解算**: 基于Madgwick算法的姿态估计
- 📱 **串口通信**: 可靠的BBC校验协议
- 🎛️ **灵活配置**: 丰富的ROS2参数支持
- 🚀 **即插即用**: 提供多种启动配置

## 📦 包结构

```
dlrobot_robot_python/
├── dlrobot_robot_python/          # Python源代码
│   ├── __init__.py
│   ├── dlrobot_robot_node.py      # 主控制节点
│   ├── quaternion_solution.py     # 四元数解算模块
│   ├── cmd_vel_to_ackermann.py    # Twist到Ackermann转换
│   └── parameter_node.py          # 参数演示节点
├── launch/                        # 启动文件
│   ├── dlrobot_python.launch.py   # 完整启动文件
│   └── dlrobot_python_mini.launch.py # 简化启动文件
├── config/                        # 配置文件
├── package.xml                    # ROS2包配置
├── setup.py                      # Python包配置
└── README.md                      # 本文件
```

## 🛠️ 安装和编译

### 依赖项

确保安装了以下依赖：

```bash
# ROS2 Humble基础包
sudo apt install ros-humble-desktop

# Python串口库
sudo apt install python3-serial python3-numpy

# 自定义消息包（如果需要）
# 确保workspace中有以下包：
# - custom_msgs
# - dlrobot_robot_msg
# - ackermann_msgs
```

### 编译

```bash
# 进入工作空间
cd /path/to/your/workspace

# 编译包
colcon build --packages-select dlrobot_robot_python

# 加载环境
source install/setup.bash
```

## 🚀 使用方法

### 1. 硬件连接

- 将DLRobot底盘通过USB串口连接到计算机
- 确保设备出现在`/dev/dlrobot_controller`（可能需要配置udev规则）

### 2. 基本启动

**普通模式（差分驱动）:**
```bash
ros2 launch dlrobot_robot_python dlrobot_python_mini.launch.py
```

**阿克曼模式:**
```bash
ros2 launch dlrobot_robot_python dlrobot_python.launch.py akmcar:=true
```

### 3. 高级配置

**自定义串口和参数:**
```bash
ros2 launch dlrobot_robot_python dlrobot_python.launch.py \
    usart_port_name:=/dev/ttyUSB0 \
    serial_baud_rate:=115200 \
    wheelbase:=0.143
```

## 📡 话题接口

### 订阅话题

- `cmd_vel` (geometry_msgs/Twist): 机器人速度控制命令
- `ackermann_cmd` (ackermann_msgs/AckermannDriveStamped): 阿克曼驱动命令

### 发布话题

- `odom_combined` (nav_msgs/Odometry): 机器人里程计数据
- `mobile_base/sensors/imu_data` (sensor_msgs/Imu): IMU传感器数据
- `PowerVoltage` (std_msgs/Float32): 电源电压
- `robotpose` (dlrobot_robot_msg/Data): 机器人位置
- `robotvel` (dlrobot_robot_msg/Data): 机器人速度

## ⚙️ 配置参数

| 参数名 | 类型 | 默认值 | 描述 |
|--------|------|--------|------|
| `usart_port_name` | string | `/dev/dlrobot_controller` | 串口设备名 |
| `serial_baud_rate` | int | 115200 | 串口波特率 |
| `robot_frame_id` | string | `base_footprint` | 机器人坐标系 |
| `odom_frame_id` | string | `odom_combined` | 里程计坐标系 |
| `gyro_frame_id` | string | `gyro_link` | IMU坐标系 |
| `cmd_vel` | string | `cmd_vel` | 速度控制话题名 |
| `akm_cmd_vel` | string | `none` | 阿克曼控制话题名 |

## 🔧 机器人配置

支持多种DLRobot机器人型号：

| 型号 | 轮距 (m) | 适用场景 |
|------|----------|----------|
| mini_akm | 0.143 | 小型室内机器人 |
| senior_akm | 0.320 | 中型服务机器人 |
| top_akm_bs | 0.503 | 大型移动平台 |
| top_akm_dl | 0.549 | 重载运输机器人 |

## 🧪 测试和调试

### 查看话题

```bash
# 查看所有话题
ros2 topic list

# 监控里程计数据
ros2 topic echo /odom_combined

# 监控IMU数据
ros2 topic echo /mobile_base/sensors/imu_data
```

### 手动控制

```bash
# 发送速度命令（前进）
ros2 topic pub /cmd_vel geometry_msgs/Twist "linear: {x: 0.2}" --once

# 发送转向命令
ros2 topic pub /cmd_vel geometry_msgs/Twist "angular: {z: 0.5}" --once

# 停止机器人
ros2 topic pub /cmd_vel geometry_msgs/Twist "{}" --once
```

### 参数调试

```bash
# 查看当前参数
ros2 param list

# 修改参数
ros2 param set /dlrobot_robot_node serial_baud_rate 9600
```

## 🔍 故障排除

### 常见问题

1. **串口连接失败**
   - 检查设备是否正确连接：`ls /dev/ttyUSB* /dev/dlrobot_controller`
   - 检查权限：`sudo chmod 666 /dev/ttyUSB0`
   - 配置udev规则（参考原包的`dlrobot_udev.sh`）

2. **没有数据发布**
   - 检查串口参数是否正确
   - 确认下位机正常工作
   - 查看节点日志：`ros2 log list`

3. **IMU数据异常**
   - 检查IMU传感器连接
   - 验证数据转换系数
   - 校准加速度计和陀螺仪

## 🆚 与C++版本对比

| 功能 | C++版本 | Python版本 | 备注 |
|------|---------|-------------|------|
| 基本功能 | ✅ | ✅ | 完全兼容 |
| 串口通信 | ✅ | ✅ | 相同协议 |
| IMU解算 | ✅ | ✅ | Madgwick算法 |
| 阿克曼模式 | ✅ | ✅ | 完全支持 |
| 性能 | 高 | 中等 | Python解释型语言 |
| 调试便利性 | 中等 | 高 | Python更易调试 |
| 内存占用 | 低 | 中等 | Python运行时开销 |

## 🤝 贡献

欢迎提交Issue和Pull Request来改进这个包！

## 📄 许可证

MIT License

## 👥 作者

- AI Assistant
- 基于原始C++版本: DLRobot团队

---

**注意**: 这是原C++包`turn_on_dlrobot_robot`的Python重新实现版本，保持了相同的功能和接口兼容性。 