# Robot Control Node 轴控制修改说明

## 🎯 修改目标

将 `robot_control_node.py` 程序修改为只改变 X 轴和 Z 轴的速度，Y 轴强制设为 0，以适配差分驱动机器人的运动特性。

## 📋 修改内容

### 1. 文档说明增强

#### 文件头部文档
```python
"""
🎯 运动控制设计：
- X轴速度：前进(+)/后退(-)线速度控制 (m/s)
- Z轴速度：左转(+)/右转(-)角速度控制 (rad/s)  
- Y轴速度：强制设为0.0 (差分驱动机器人不支持侧向运动)

适用于差分驱动机器人，如DLRobot系列底盘
"""
```

#### 类文档说明
```python
class RobotControlNode(Node):
    """
    小车控制节点
    
    🎯 运动控制说明：
    - X轴速度：前进(+)/后退(-)线速度控制
    - Z轴速度：左转(+)/右转(-)角速度控制  
    - Y轴速度：强制设为0.0（差分驱动机器人不支持侧向运动）
    """
```

### 2. 核心方法修改

#### `send_velocity_command` 方法
- **增强文档说明**：明确参数含义和Y轴处理策略
- **明确Y轴处理**：显式声明 `linear_y = 0.0`
- **详细注释**：为Twist消息的每个轴添加详细注释
- **日志增强**：输出信息明确显示各轴的速度值

**修改前：**
```python
def send_velocity_command(self, linear_x, angular_z):
    """发送速度控制命令"""
    msg.linear.y = 0.0
```

**修改后：**
```python
def send_velocity_command(self, linear_x, angular_z):
    """
    发送速度控制命令
    
    参数说明：
    - linear_x: X轴线速度 (m/s) - 前进(+)/后退(-)
    - angular_z: Z轴角速度 (rad/s) - 左转(+)/右转(-)
    - Y轴速度: 强制设为0.0 (差分驱动机器人不支持侧向运动)
    """
    # ⚠️ Y轴速度强制设为0（差分驱动不支持侧向运动）
    linear_y = 0.0
    
    # 🎯 机器人运动控制：只使用X轴和Z轴，Y轴强制设为0
    msg.linear.x = linear_x    # X轴：前进(+)/后退(-)线速度
    msg.linear.y = 0.0         # Y轴：侧向速度，强制设为0（差分驱动不支持）
    msg.linear.z = 0.0         # Z轴：垂直速度，强制设为0（2D平面运动）
    msg.angular.x = 0.0        # X轴旋转：俯仰角速度，强制设为0（2D平面运动）
    msg.angular.y = 0.0        # Y轴旋转：翻滚角速度，强制设为0（2D平面运动）
    msg.angular.z = angular_z  # Z轴：偏航角速度，左转(+)/右转(-)
```

#### `manual_cmd_callback` 方法
- **明确轴处理**：显式提取和处理各轴速度
- **注释说明**：明确说明只使用X轴和Z轴，忽略Y轴
- **日志增强**：输出信息显示各轴状态

**修改前：**
```python
def manual_cmd_callback(self, msg):
    if self.control_mode == ControlMode.MANUAL:
        self.send_velocity_command(msg.linear.x, msg.angular.z)
```

**修改后：**
```python
def manual_cmd_callback(self, msg):
    if self.control_mode == ControlMode.MANUAL:
        # 🎯 只使用X轴（前进/后退）和Z轴（旋转）速度，忽略Y轴（侧向）速度
        linear_x = msg.linear.x  # 前进/后退速度
        angular_z = msg.angular.z  # 旋转角速度
        # 明确忽略 msg.linear.y（侧向速度）
        
        self.send_velocity_command(linear_x, angular_z)
        self.get_logger().debug(f"手动控制: X轴(前进)={linear_x:.2f}, Z轴(旋转)={angular_z:.2f}, Y轴(侧向)=0.0(忽略)")
```

#### `handle_motor_command` 方法
- **文档增强**：明确说明电机控制的轴配置
- **注释详化**：为每个运动命令添加轴说明
- **日志优化**：显示具体的轴速度值

**修改前：**
```python
def handle_motor_command(self, cmd_type, cmd_value):
    """处理电机控制命令"""
    if cmd_type == 'motor_forward':
        self.send_velocity_command(linear_vel, 0.0)
        self.get_logger().info(f"🔺 电机前进: {speed}%")
```

**修改后：**
```python
def handle_motor_command(self, cmd_type, cmd_value):
    """
    处理电机控制命令
    
    🎯 电机控制说明：
    - 前进/后退：使用X轴线速度控制
    - 左转/右转：使用Z轴角速度控制
    - 侧向运动：不支持（Y轴速度强制为0）
    """
    if cmd_type == 'motor_forward':
        # X轴正向线速度，Z轴角速度为0
        self.send_velocity_command(linear_vel, 0.0)
        self.get_logger().info(f"🔺 电机前进: {speed}% (X轴={linear_vel:.3f})")
```

### 3. 初始化日志增强

在节点初始化完成时添加运动控制配置说明：

```python
self.get_logger().info("📐 运动控制配置: X轴(前进/后退) + Z轴(左转/右转), Y轴强制为0(差分驱动)")
```

## 🔧 技术实现细节

### 坐标系定义
- **X轴**：机器人前方方向（Forward）
  - 正值：前进
  - 负值：后退
  
- **Y轴**：机器人左侧方向（Left）
  - **强制设为 0.0**（差分驱动不支持侧向运动）
  
- **Z轴**：机器人上方方向（Up），用于旋转控制
  - 正值：逆时针旋转（左转）
  - 负值：顺时针旋转（右转）

### WebSocket信息增强

在发送给WebSocket的控制信息中明确包含Y轴状态：

```python
command_info = {
    "linear_x": round(linear_x, 3),
    "linear_y": round(linear_y, 3),  # 明确记录Y轴为0
    "angular_z": round(angular_z, 3),
}
```

## 🎯 修改效果

### 1. 代码清晰性
- 明确的文档说明轴控制策略
- 详细的注释说明每个轴的用途
- 清晰的日志输出显示轴状态

### 2. 功能安全性
- 强制Y轴为0，避免差分驱动机器人的无效指令
- 明确的参数验证和限制
- 完整的错误处理机制

### 3. 调试便利性
- 详细的日志输出显示各轴速度值
- 清晰的运动状态追踪
- 便于故障诊断的信息输出

## 📋 兼容性说明

### 保持兼容
- 所有现有的API接口保持不变
- 现有的调用方式继续有效
- 向后兼容所有控制模式

### 行为变化
- Y轴速度现在明确强制为0（之前也是0但不够明确）
- 日志输出更加详细和清晰
- 错误处理更加完善

## 🚀 使用示例

### 手动控制
```bash
# 前进
ros2 topic pub /robot_control/manual_cmd geometry_msgs/Twist "linear: {x: 0.5}" --once

# 左转
ros2 topic pub /robot_control/manual_cmd geometry_msgs/Twist "angular: {z: 0.5}" --once

# 侧向移动（会被忽略，Y轴强制为0）
ros2 topic pub /robot_control/manual_cmd geometry_msgs/Twist "linear: {y: 0.5}" --once
```

### 电机控制
```bash
# 通过WebSocket发送电机命令
echo "motor_forward:50" | nc localhost 1234
echo "motor_left:30" | nc localhost 1234
```

## 📝 总结

通过这次修改，`robot_control_node.py` 现在更加明确地实现了"只使用X轴和Z轴速度，Y轴设为0"的设计要求。修改保持了完全的向后兼容性，同时大大提高了代码的可读性、可维护性和调试便利性。

这个设计完全符合差分驱动机器人的运动特性，确保了控制指令的有效性和安全性。 