# Following Robot Package

基于ROS2 Humble的双目立体视觉跟随机器人包

## 📋 功能特性

- 🎥 **双目立体视觉**：支持双目相机的立体视觉处理
- 📏 **距离测量**：提供精确的距离测量服务
- 🚀 **实时处理**：优化的算法确保实时性能
- 🔧 **参数可配置**：支持多种相机参数配置

## 🛠️ 依赖项

### ROS2依赖
- `rclpy`
- `sensor_msgs`
- `geometry_msgs`
- `cv_bridge`
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
# 启动双目立体视觉节点
ros2 launch following_robot stereo_vision_launch.py

# 启动演示系统（包含测试节点）
ros2 launch following_robot demo_launch.py
```

### 运行演示测试

```bash
ros2 run following_robot demo_test
```

## 📡 话题和服务

### 发布的话题
- `/stereo/left_image` (sensor_msgs/Image): 左相机图像

### 提供的服务
- `/stereo/get_distance` (custom_msgs/GetDistance): 距离测量服务

## ⚙️ 配置参数

### 相机参数
- `camera_id`: 相机设备ID (默认: 1)
- `frame_width`: 图像宽度 (默认: 1280)
- `frame_height`: 图像高度 (默认: 480)

### 立体视觉参数
- `baseline`: 基线距离 (mm)
- `focal_length`: 焦距 (像素)
- `min_distance`: 最小测量距离 (mm)
- `max_distance`: 最大测量距离 (mm)

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
│   └── demo_test.py               # 演示测试脚本
├── launch/
│   ├── stereo_vision_launch.py   # 双目视觉启动文件
│   └── demo_launch.py            # 演示系统启动文件
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

2. **依赖项错误**
   - 确保所有依赖项已正确安装
   - 运行 `rosdep install --from-paths src --ignore-src -r -y`

3. **OpenCV错误**
   - 确保OpenCV版本兼容
   - 检查系统是否有多个OpenCV版本冲突

### 调试技巧

- 使用 `ros2 topic list` 查看可用话题
- 使用 `ros2 service list` 查看可用服务
- 检查节点日志：`ros2 log view`

## 📄 许可证

TODO: 添加许可证信息

## 🤝 贡献

欢迎提交Issue和Pull Request来改进此包。

## 📞 联系方式

维护者：monster
邮箱：monster@todo.todo 