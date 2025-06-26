# 变更日志 (CHANGELOG)

## [v2.0.0] - 2024-12-28

### 🔄 重大变更
- **删除ROS2图像发布功能**: 不再通过`/stereo/left_image`话题发布图像
- **采用cv2.imshow直接显示**: 改为使用OpenCV窗口直接显示相机图像
- **移除demo_test**: 删除了演示测试相关的文件和功能

### ✨ 新增功能
- **实时状态显示**: 在图像窗口中显示帧数、FPS、分辨率等信息
- **键盘交互控制**: 
  - 按`'q'`键退出程序
  - 按`'s'`键切换立体视觉处理开关
- **简化的launch文件**: 新增`simple_launch.py`快速启动
- **显示测试工具**: 新增`test_display.py`用于测试显示功能

### 🗑️ 删除内容
- `demo_launch.py` - 演示系统启动文件
- `demo_test.py` - 演示测试脚本
- `sensor_msgs`, `geometry_msgs`, `cv_bridge` 依赖项
- 图像发布相关的代码和配置

### 🔧 优化改进
- **简化依赖**: 移除不必要的ROS2依赖项
- **提升性能**: 去除图像转换和发布的开销
- **更直观的显示**: 实时显示系统状态信息
- **更好的用户体验**: 键盘控制更加直观

### 📋 使用方法更新

#### 旧版本启动方式
```bash
ros2 launch following_robot demo_launch.py
ros2 run following_robot demo_test
```

#### 新版本启动方式
```bash
# 简单启动
ros2 run following_robot stereo_vision_node

# 使用launch文件
ros2 launch following_robot stereo_vision_launch.py
ros2 launch following_robot simple_launch.py

# 测试显示功能
ros2 run following_robot test_display
```

### ⚠️ 注意事项
- 需要系统支持图形界面显示（X11）
- SSH连接时需要启用X11转发
- 程序运行期间会显示OpenCV窗口
- 按键控制只在OpenCV窗口处于焦点时有效

### 🛠️ 兼容性
- ROS2 Humble
- OpenCV 4.x
- Python 3.8+
- Ubuntu 20.04+ 