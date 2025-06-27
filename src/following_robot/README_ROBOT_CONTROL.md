# 小车控制节点使用说明

## 📋 功能概述

小车控制节点整合了`turn_on_dlrobot_robot`包的底层控制功能，提供了高级的运动控制接口，支持多种控制模式：

- **手动控制模式**：通过话题接收控制命令
- **人体跟随模式**：自动跟随检测到的目标人物
- **导航模式**：预留接口，可与导航系统集成
- **停止模式**：安全停止状态

## 🏗️ 系统架构

```
┌─────────────────────┐    ┌─────────────────────┐    ┌─────────────────────┐
│   特征提取节点      │    │   小车控制节点      │    │   底层驱动节点      │
│ feature_extraction  │    │ robot_control_node  │    │ dlrobot_robot_node  │
│                     │    │                     │    │                     │
│ - 人体检测          │───▶│ - 跟随控制          │───▶│ - 串口通信          │
│ - 特征提取          │    │ - 手动控制          │    │ - 电机控制          │
│ - 位置计算          │    │ - 安全监控          │    │ - 传感器数据        │
└─────────────────────┘    └─────────────────────┘    └─────────────────────┘
```

## 🚀 快速开始

### 1. 编译项目

```bash
cd /userdata/try_again/SelfFollowingROS2
colcon build --packages-select following_robot turn_on_dlrobot_robot custom_msgs
source install/setup.bash
```

### 2. 启动小车控制系统

#### 标准模式（差速驱动）
```bash
ros2 launch following_robot robot_control_launch.py
```

#### 阿克曼模式（转向驱动）
```bash
ros2 launch following_robot robot_control_launch.py use_ackermann:=true
```

#### 自定义参数启动
```bash
ros2 launch following_robot robot_control_launch.py \
    max_linear_speed:=0.8 \
    max_angular_speed:=1.5 \
    min_follow_distance:=0.8 \
    max_follow_distance:=2.5 \
    wheelbase:=0.143
```

### 3. 测试控制功能

#### 交互式测试
```bash
python3 src/following_robot/scripts/robot_control_test.py interactive
```

#### 自动化测试
```bash
# 测试手动控制
python3 src/following_robot/scripts/robot_control_test.py manual

# 测试跟随模式
python3 src/following_robot/scripts/robot_control_test.py following

# 测试紧急停止
python3 src/following_robot/scripts/robot_control_test.py emergency

# 运行所有测试
python3 src/following_robot/scripts/robot_control_test.py all
```

## 🎮 控制接口

### 话题接口

#### 发布话题（输出）
- `/cmd_vel` (geometry_msgs/Twist): 标准速度控制命令
- `/ackermann_cmd` (ackermann_msgs/AckermannDriveStamped): 阿克曼控制命令
- `/robot_control/status` (std_msgs/String): 系统状态信息
- `/robot_control/mode` (std_msgs/String): 当前控制模式

#### 订阅话题（输入）
- `/robot_control/manual_cmd` (geometry_msgs/Twist): 手动控制命令
- `/robot_control/person_position` (turn_on_dlrobot_robot/Position): 人体位置信息
- `/robot_control/set_mode` (std_msgs/String): 模式切换命令
- `/robot_control/target_person` (std_msgs/String): 目标人物设置
- `/robot_control/emergency_stop` (std_msgs/Bool): 紧急停止命令
- `/odom` (nav_msgs/Odometry): 里程计数据

### 控制模式

#### 1. 手动控制模式 (`manual`)
```bash
# 设置为手动控制模式
ros2 topic pub /robot_control/set_mode std_msgs/String "data: 'manual'"

# 发送控制命令
ros2 topic pub /robot_control/manual_cmd geometry_msgs/Twist "
linear:
  x: 0.2
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.5"
```

#### 2. 跟随模式 (`following`)
```bash
# 设置目标人物
ros2 topic pub /robot_control/target_person std_msgs/String "data: 'person_1'"

# 设置为跟随模式
ros2 topic pub /robot_control/set_mode std_msgs/String "data: 'following'"

# 发送人体位置信息
ros2 topic pub /robot_control/person_position turn_on_dlrobot_robot/Position "
distance: 2.0
angle_x: 0.3
angle_y: 0.0"
```

#### 3. 停止模式 (`stop`)
```bash
ros2 topic pub /robot_control/set_mode std_msgs/String "data: 'stop'"
```

#### 4. 紧急停止
```bash
# 激活紧急停止
ros2 topic pub /robot_control/emergency_stop std_msgs/Bool "data: true"

# 解除紧急停止
ros2 topic pub /robot_control/emergency_stop std_msgs/Bool "data: false"
```

## ⚙️ 参数配置

### 运动参数
- `max_linear_speed`: 最大线速度 (m/s，默认: 0.5)
- `max_angular_speed`: 最大角速度 (rad/s，默认: 1.0)
- `min_follow_distance`: 最小跟随距离 (m，默认: 1.0)
- `max_follow_distance`: 最大跟随距离 (m，默认: 3.0)
- `follow_speed_factor`: 跟随速度因子 (默认: 0.3)

### 车辆参数
- `wheelbase`: 车辆轴距 (m，默认: 0.143)
- `use_ackermann`: 是否使用阿克曼转向 (默认: false)

### 安全参数
- `safety_enabled`: 是否启用安全功能 (默认: true)

### 底层驱动参数
- `usart_port_name`: 串口设备名称 (默认: /dev/dlrobot_controller)
- `serial_baud_rate`: 串口波特率 (默认: 115200)
- `robot_frame_id`: 机器人坐标系ID (默认: base_footprint)
- `odom_frame_id`: 里程计坐标系ID (默认: odom_combined)

## 🔧 集成示例

### 与特征提取节点集成

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from turn_on_dlrobot_robot.msg import Position

class FollowingDemo(Node):
    def __init__(self):
        super().__init__('following_demo')
        
        # 发布者
        self.mode_pub = self.create_publisher(String, '/robot_control/set_mode', 10)
        self.person_pub = self.create_publisher(Position, '/robot_control/person_position', 10)
        
        # 启动跟随模式
        self.start_following()
    
    def start_following(self):
        # 设置跟随模式
        mode_msg = String()
        mode_msg.data = 'following'
        self.mode_pub.publish(mode_msg)
        
        # 模拟人体位置
        pos_msg = Position()
        pos_msg.distance = 2.0
        pos_msg.angle_x = 0.0
        pos_msg.angle_y = 0.0
        self.person_pub.publish(pos_msg)

def main():
    rclpy.init()
    node = FollowingDemo()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 🛡️ 安全特性

### 1. 紧急停止系统
- 支持软件紧急停止
- 自动检测通信超时
- 安全状态监控

### 2. 速度限制
- 可配置的最大速度限制
- 平滑的加减速控制
- 防止危险操作

### 3. 跟随安全
- 最小/最大跟随距离限制
- 人体检测超时保护
- 自动停止机制

## 📊 监控和调试

### 查看系统状态
```bash
ros2 topic echo /robot_control/status
ros2 topic echo /robot_control/mode
```

### 查看控制命令
```bash
ros2 topic echo /cmd_vel
ros2 topic echo /ackermann_cmd
```

### 查看里程计数据
```bash
ros2 topic echo /odom
```

### 节点信息
```bash
ros2 node info /robot_control_node
```

## 🔍 故障排除

### 常见问题

#### 1. 小车不响应控制命令
- 检查串口连接：`ls -la /dev/dlrobot_controller`
- 检查权限：`sudo chmod 666 /dev/dlrobot_controller`
- 检查底层驱动节点是否运行：`ros2 node list | grep dlrobot`

#### 2. 跟随模式不工作
- 检查人体位置话题：`ros2 topic echo /robot_control/person_position`
- 检查控制模式：`ros2 topic echo /robot_control/mode`
- 检查特征提取节点是否运行

#### 3. 阿克曼模式问题
- 确认启动时使用了`use_ackermann:=true`
- 检查转换节点：`ros2 node list | grep cmd_vel_to_ackermann`
- 检查轴距参数是否正确

### 日志分析
```bash
ros2 run rqt_console rqt_console
```

## 📝 开发说明

### 扩展控制模式
要添加新的控制模式，需要：

1. 在`ControlMode`枚举中添加新模式
2. 在`control_loop`方法中添加处理逻辑
3. 实现相应的回调函数
4. 更新文档

### 自定义安全检查
在`safety_check`方法中添加自定义安全逻辑：

```python
def safety_check(self):
    # 自定义安全检查
    if self.check_battery_level() < 0.2:
        self.emergency_stop = True
        self.get_logger().warn("电池电量低，紧急停止")
```

## 📞 技术支持

如有问题，请检查：
1. ROS2环境是否正确设置
2. 所有依赖包是否正确安装
3. 串口设备是否正确连接
4. 参数配置是否合理

更多信息请参考：
- [ROS2官方文档](https://docs.ros.org/en/humble/)
- [following_robot包文档](README.md)
- [特征提取功能文档](README_FEATURE_EXTRACTION.md) 