# Following Robot Package

基于ROS2 Humble的双目立体视觉跟随机器人包

## 📋 功能特性

- 🎥 **双目立体视觉**：支持双目相机的立体视觉处理
- 📏 **距离测量**：提供精确的距离测量服务
- 🖼️ **实时显示**：使用cv2.imshow直接显示相机图像
- 📊 **状态显示**：实时显示帧数、FPS、分辨率等信息
- ⌨️ **键盘控制**：支持键盘交互控制
- 🔧 **参数可配置**：支持多种相机参数配置

## 🛠️ 依赖项

### ROS2依赖
- `rclpy`
- `custom_msgs`

### Python依赖
- `opencv-python`
- `numpy`

### 系统依赖
- `libopencv-dev`
- `python3-opencv`
- `python3-numpy`

## 📦 安装

1. 确保您的ROS2环境已正确设置
2. 将此包放在您的ROS2工作空间的`src`目录中
3. 构建包：
   ```bash
   cd ~/your_ros2_ws
   colcon build --packages-select following_robot
   source install/setup.bash
   ```

## 🚀 使用方法

### 单独启动双目视觉节点

```bash
ros2 run following_robot stereo_vision_node
```

### 使用Launch文件启动

```bash
# 启动双目立体视觉节点（默认参数）
ros2 launch following_robot stereo_vision_launch.py

# 使用自定义参数启动
ros2 launch following_robot stereo_vision_launch.py camera_id:=0 enable_stereo:=false fps_limit:=15
```

### 测试显示功能

```bash
ros2 run following_robot test_display
```

## ⌨️ 键盘控制

- **'q'键**: 退出程序
- **'s'键**: 切换立体视觉处理开关

## 📡 服务

### 提供的服务
- `/stereo/get_distance` (custom_msgs/GetDistance): 距离测量服务

## ⚙️ 配置参数

### Launch参数
- `camera_id`: 相机设备ID (默认: 1)
- `enable_stereo`: 是否启用立体视觉处理 (默认: true)
- `fps_limit`: 帧率限制 (默认: 30)

### 立体视觉参数
- `baseline`: 基线距离 (mm)
- `focal_length`: 焦距 (像素)
- `min_distance`: 最小测量距离 (mm)
- `max_distance`: 最大测量距离 (mm)

## 🖼️ 显示信息

程序运行时会在窗口中显示：
- **Frame**: 当前帧数
- **FPS**: 实时帧率
- **Size**: 图像分辨率
- **Stereo**: 立体视觉处理状态

## 🧪 测试

运行单元测试：
```bash
colcon test --packages-select following_robot
```

## 📝 文件结构

```
following_robot/
├── following_robot/
│   ├── __init__.py
│   └── stereo_vision_node.py      # 主要的双目视觉节点
├── scripts/
│   └── test_display.py            # 显示功能测试脚本
├── launch/
│   └── stereo_vision_launch.py   # 双目视觉启动文件
├── data/
│   └── yolov8_pose.rknn          # 姿态检测模型
├── package.xml                    # 包依赖声明
├── setup.py                      # Python包配置
├── setup.cfg                     # 包配置
└── README.md                     # 本文件
```

## 🔧 故障排除

### 常见问题

1. **相机无法打开**
   - 检查相机是否正确连接
   - 确认相机设备ID是否正确
   - 检查相机权限

2. **显示窗口无法打开**
   - 确保系统支持X11转发（如果使用SSH）
   - 检查OpenCV是否正确安装
   - 尝试运行测试脚本：`ros2 run following_robot test_display`

3. **依赖项错误**
   - 确保所有依赖项已正确安装
   - 运行 `rosdep install --from-paths src --ignore-src -r -y`

4. **OpenCV错误**
   - 确保OpenCV版本兼容
   - 检查系统是否有多个OpenCV版本冲突

### 调试技巧

- 使用 `ros2 service list` 查看可用服务
- 检查节点日志：`ros2 log view`
- 测试相机：先运行 `ros2 run following_robot test_display`

## 📄 许可证

TODO: 添加许可证信息

## 🤝 贡献

欢迎提交Issue和Pull Request来改进此包。

## 📞 联系方式

维护者：monster
邮箱：monster@todo.todo 