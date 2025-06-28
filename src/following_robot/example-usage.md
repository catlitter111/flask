# ByteTracker ROS2使用示例

## 快速开始

### 1. 编译和安装

```bash
# 进入工作空间
cd ~/ros2_ws

# 清理之前的构建（可选）
rm -rf build install log

# 构建所需包
colcon build --packages-select custom_msgs following_robot

# 源环境
source install/setup.bash
```

### 2. 基础多目标跟踪

```bash
# 终端1：启动相机节点（如果需要）
ros2 run v4l2_camera v4l2_camera_node

# 终端2：启动ByteTracker多目标跟踪
ros2 launch following_robot bytetracker_launch.py tracking_mode:=multi

# 终端3：查看可视化结果
ros2 run rqt_image_view rqt_image_view
# 选择话题：/bytetracker/visualization
```

### 3. 单目标跟踪与小车控制

```bash
# 步骤1：生成目标特征文件（只需执行一次）
# 拍摄目标人物照片并提取特征
ros2 run following_robot feature_extraction_node &
# 调用服务提取特征（需要提供包含目标人物的图像）
# 这会生成一个Excel文件包含目标特征

# 步骤2：启动完整的单目标跟踪系统
ros2 launch following_robot full_tracking_system_launch.py \
    tracking_mode:=single \
    target_features_file:=/home/user/features-data/person.xlsx \
    enable_car_control:=true \
    enable_stereo:=true
```

### 4. 使用参数文件启动

```bash
# 使用自定义参数文件
ros2 launch following_robot bytetracker_launch.py \
    --params-file src/following_robot/config/bytetracker_params.yaml
```

## 高级用法

### 1. 动态切换跟踪模式

```bash
# 启动时使用多目标模式
ros2 launch following_robot bytetracker_launch.py tracking_mode:=multi

# 运行时切换到单目标模式
ros2 topic pub -1 /bytetracker/set_mode std_msgs/String "data: single"

# 设置要跟踪的目标
ros2 topic pub -1 /bytetracker/set_target std_msgs/String "data: person_001"
```

### 2. 监控跟踪状态

```bash
# 查看实时跟踪状态
ros2 topic echo /bytetracker/status

# 查看跟踪到的所有人员
ros2 topic echo /bytetracker/tracked_persons

# 监控性能（帧率）
ros2 topic hz /bytetracker/visualization
```

### 3. 调试和测试

```bash
# 录制测试数据
ros2 bag record /camera/image_raw /bytetracker/tracked_persons

# 回放测试
ros2 bag play your_bag_file.db3

# 查看节点图
ros2 run rqt_graph rqt_graph
```

### 4. 与小车控制集成

```bash
# 确保小车控制节点运行
ros2 run following_robot robot_control_node

# 查看小车控制命令
ros2 topic echo /robot_control/person_position

# 手动控制小车（测试用）
ros2 topic pub /robot_control/manual_cmd geometry_msgs/Twist \
    "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

## 常用命令速查

### 服务调用

```bash
# 提取特征
ros2 service call /features/extract_features custom_msgs/srv/FeatureExtraction \
    "{image: ..., person_name: 'john', save_to_file: true, output_path: '/tmp'}"

# 获取距离
ros2 service call /stereo/get_distance custom_msgs/srv/GetDistance \
    "{x: 320, y: 240}"
```

### 参数设置

```bash
# 运行时修改参数
ros2 param set /bytetracker_node track_thresh 0.4
ros2 param set /bytetracker_node color_weight 0.6

# 查看所有参数
ros2 param list /bytetracker_node

# 保存当前参数
ros2 param dump /bytetracker_node
```

### 故障排除

```bash
# 检查节点状态
ros2 node list
ros2 node info /bytetracker_node

# 查看日志
ros2 run following_robot bytetracker_node --ros-args --log-level debug

# 检查话题连接
ros2 topic info /camera/image_raw
ros2 topic info /bytetracker/tracked_persons
```

## 典型应用场景

### 场景1：展会人流跟踪

```bash
# 使用多目标模式，启用可视化
ros2 launch following_robot bytetracker_launch.py \
    tracking_mode:=multi \
    enable_visualization:=true \
    track_buffer:=150
```

### 场景2：个人助理机器人

```bash
# 单目标跟踪，启用所有功能
ros2 launch following_robot full_tracking_system_launch.py \
    tracking_mode:=single \
    target_features_file:=/path/to/owner.xlsx \
    enable_car_control:=true \
    enable_stereo:=true \
    min_follow_distance:=1.2 \
    max_follow_distance:=2.5
```

### 场景3：安防监控

```bash
# 多目标跟踪，保存跟踪数据
ros2 launch following_robot bytetracker_launch.py \
    tracking_mode:=multi &

# 录制跟踪数据
ros2 bag record -o security_log \
    /bytetracker/tracked_persons \
    /bytetracker/visualization \
    /bytetracker/status
```

## 性能优化技巧

1. **降低处理负载**
   ```bash
   # 减少身体比例计算频率
   ros2 param set /bytetracker_node body_ratio_calc_interval 20
   ```

2. **调整跟踪灵敏度**
   ```bash
   # 提高跟踪稳定性
   ros2 param set /bytetracker_node track_buffer 150
   ros2 param set /bytetracker_node match_thresh 0.7
   ```

3. **优化单目标跟踪**
   ```bash
   # 调整特征权重
   ros2 param set /bytetracker_node upper_color_weight 0.5
   ros2 param set /bytetracker_node body_ratio_weight 0.2
   ```