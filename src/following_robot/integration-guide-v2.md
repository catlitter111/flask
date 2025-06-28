# ByteTracker ROS2集成指南

## 一、系统架构概述

ByteTracker已完整移植到ROS2 Humble系统中，保留了所有原始功能并与现有节点无缝集成。

### 核心功能
1. **多目标跟踪**：同时跟踪多个人物
2. **单目标跟踪**：锁定并跟踪特定目标
3. **特征融合**：颜色特征 + 身体比例特征
4. **卡尔曼滤波**：预测目标运动轨迹
5. **小车控制**：自动跟随目标人物

### 节点架构
```
ByteTracker节点
├── 服装检测（使用rknn_colour_detect）
├── 姿态检测（使用yolov8_pose_detector）
├── ByteTracker算法
│   ├── KalmanFilter（卡尔曼滤波）
│   ├── ColorByteTracker（多目标跟踪）
│   └── SingleTargetTracker（单目标跟踪）
└── ROS2接口
    ├── 订阅相机图像
    ├── 发布跟踪结果
    └── 与其他节点通信
```

## 二、文件配置

### 1. 添加新文件到ROS2包

将以下文件添加到您的ROS2工作空间：

```bash
# 在following_robot包中添加
src/following_robot/following_robot/
├── bytetracker_node.py          # ByteTracker主节点
└── ...其他现有文件...

# 在custom_msgs包中添加消息定义
src/custom_msgs/msg/
├── TrackedPerson.msg            # 被跟踪人员信息
├── TrackedPersonArray.msg       # 跟踪人员数组
├── TrackingMode.msg            # 跟踪模式信息
└── Position.msg                # 位置信息

# 添加启动文件
src/following_robot/launch/
├── bytetracker_launch.py        # ByteTracker启动文件
└── full_tracking_system_launch.py  # 完整系统启动文件
```

### 2. 更新CMakeLists.txt

在`custom_msgs/CMakeLists.txt`中添加新消息：

```cmake
rosidl_generate_interfaces(${PROJECT_NAME}
  # 现有消息...
  "msg/TrackedPerson.msg"
  "msg/TrackedPersonArray.msg"
  "msg/TrackingMode.msg"
  "msg/Position.msg"
  DEPENDENCIES std_msgs sensor_msgs geometry_msgs
)
```

### 3. 更新setup.py

在`following_robot/setup.py`中添加新节点：

```python
entry_points={
    'console_scripts': [
        # 现有节点...
        'bytetracker_node = following_robot.bytetracker_node:main',
    ],
},
```

## 三、构建和安装

```bash
# 进入工作空间
cd ~/ros2_ws

# 构建包
colcon build --packages-select custom_msgs following_robot

# 源环境
source install/setup.bash
```

## 四、使用方法

### 1. 启动多目标跟踪

```bash
# 基础多目标跟踪
ros2 launch following_robot bytetracker_launch.py tracking_mode:=multi

# 带小车控制的多目标跟踪
ros2 launch following_robot bytetracker_launch.py \
    tracking_mode:=multi \
    enable_car_control:=true
```

### 2. 启动单目标跟踪

```bash
# 单目标跟踪（需要目标特征文件）
ros2 launch following_robot full_tracking_system_launch.py \
    tracking_mode:=single \
    target_features_file:=/path/to/person.xlsx \
    enable_car_control:=true \
    enable_stereo:=true
```

### 3. 生成目标特征文件

```bash
# 使用特征提取服务生成目标特征
ros2 service call /features/extract_features custom_msgs/srv/FeatureExtraction \
    "{image: <image_msg>, person_name: 'target_person', save_to_file: true}"
```

## 五、ROS2接口

### 发布话题

- `/bytetracker/tracked_persons` (TrackedPersonArray): 所有被跟踪的人员
- `/bytetracker/visualization` (Image): 可视化图像
- `/bytetracker/status` (String): 跟踪器状态信息
- `/robot_control/person_position` (Position): 目标位置（用于小车控制）

### 订阅话题

- `/camera/image_raw` (Image): 输入相机图像
- `/bytetracker/set_mode` (String): 设置跟踪模式
- `/bytetracker/set_target` (String): 设置目标人物

### 参数

- `tracking_mode`: 跟踪模式 ("multi" 或 "single")
- `track_thresh`: 跟踪阈值 (默认: 0.5)
- `track_buffer`: 轨迹缓冲 (默认: 100)
- `match_thresh`: 匹配阈值 (默认: 0.8)
- `color_weight`: 颜色权重 (默认: 0.5)
- `target_features_file`: 目标特征文件路径
- `enable_car_control`: 是否启用小车控制
- `enable_distance_measure`: 是否启用距离测量

## 六、示例应用

### 1. 单目标跟随机器人

```bash
# 启动完整系统
ros2 launch following_robot full_tracking_system_launch.py \
    tracking_mode:=single \
    target_features_file:=$(ros2 pkg prefix following_robot)/share/following_robot/features-data/person.xlsx \
    enable_car_control:=true \
    enable_stereo:=true

# 系统将自动：
# 1. 根据特征文件寻找目标人物
# 2. 锁定目标后持续跟踪
# 3. 控制小车跟随目标
# 4. 使用双目视觉保持安全距离
```

### 2. 多人监控系统

```bash
# 启动多目标跟踪
ros2 launch following_robot bytetracker_launch.py \
    tracking_mode:=multi \
    enable_visualization:=true

# 监听跟踪结果
ros2 topic echo /bytetracker/tracked_persons
```

### 3. 实时切换跟踪模式

```bash
# 从多目标切换到单目标
ros2 topic pub -1 /bytetracker/set_mode std_msgs/String "data: single"

# 设置目标人物
ros2 topic pub -1 /bytetracker/set_target std_msgs/String "data: person_001"
```

## 七、调试工具

### 1. 查看跟踪状态

```bash
# 实时查看状态
ros2 topic echo /bytetracker/status

# 查看跟踪结果
ros2 topic echo /bytetracker/tracked_persons
```

### 2. 可视化

```bash
# 查看可视化图像
ros2 run rqt_image_view rqt_image_view /bytetracker/visualization

# 使用RViz2查看
rviz2 -d $(ros2 pkg prefix following_robot)/share/following_robot/rviz/bytetracker.rviz
```

### 3. 性能监控

```bash
# 查看节点CPU使用
ros2 run rqt_top rqt_top

# 查看话题频率
ros2 topic hz /bytetracker/tracked_persons
```

## 八、常见问题

### Q1: 单目标跟踪无法锁定目标
- 检查目标特征文件是否正确
- 确保目标人物的服装颜色与特征文件匹配
- 调整`track_thresh`参数降低匹配阈值

### Q2: 小车控制不响应
- 确认`enable_car_control`参数为true
- 检查`robot_control_node`是否正常运行
- 验证串口连接是否正常

### Q3: 跟踪性能较差
- 调整`color_weight`参数平衡颜色和运动特征
- 增加`track_buffer`提高轨迹稳定性
- 检查相机帧率是否足够

## 九、高级配置

### 1. 自定义跟踪参数

创建参数文件 `config/bytetracker_params.yaml`:

```yaml
bytetracker_node:
  ros__parameters:
    tracking_mode: "single"
    track_thresh: 0.4
    track_buffer: 150
    match_thresh: 0.7
    color_weight: 0.6
    # 卡尔曼滤波参数
    std_weight_position: 0.05
    std_weight_velocity: 0.00625
```

### 2. 集成自定义检测器

如需使用其他检测器，修改`detect_and_extract_features`方法：

```python
def detect_and_extract_features(self, frame):
    # 使用您的自定义检测器
    detections = your_custom_detector(frame)
    # 转换为ByteTracker格式
    return convert_to_bytetracker_format(detections)
```

## 十、性能优化建议

1. **降低检测频率**：在多目标模式下，可以每N帧进行一次身体比例计算
2. **使用GPU加速**：确保RKNN模型使用NPU加速
3. **调整图像分辨率**：降低输入分辨率可以提高处理速度
4. **优化参数**：根据实际场景调整跟踪参数

## 完整性验证

ByteTracker的所有核心功能已完整移植：
- ✅ 卡尔曼滤波预测
- ✅ 多特征融合（颜色+身体比例）
- ✅ 多目标跟踪算法
- ✅ 单目标锁定跟踪
- ✅ 与ROS2系统集成
- ✅ 小车控制接口
- ✅ 双目视觉测距