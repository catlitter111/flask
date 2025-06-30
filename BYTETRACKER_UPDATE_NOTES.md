# ByteTracker ROS2节点 - 更新说明

## 🔥 新增功能

### 1. ByteTracker跟踪算法
- ✅ 完整移植ByteTracker多目标跟踪算法到ROS2 Humble
- ✅ 融合颜色特征和身体比例特征的匹配算法
- ✅ 卡尔曼滤波预测目标运动
- ✅ 支持多目标和单目标跟踪模式

### 2. 新增文件

#### 核心文件
- `src/following_robot/following_robot/bytetracker_node.py` - ByteTracker主节点
- `src/following_robot/launch/bytetracker_launch.py` - 启动文件

#### 自定义消息 (custom_msgs)
```
msg/
├── Position.msg           # 目标位置信息
├── TrackedPerson.msg      # 被跟踪人员信息  
├── TrackedPersonArray.msg # 跟踪人员数组
└── TrackingMode.msg       # 跟踪模式

srv/
├── FeatureExtraction.srv  # 特征提取服务
├── GetDistance.srv        # 距离测量服务
└── PoseDetection.srv      # 姿态检测服务
```

## 🛠️ 配置文件更新

### 1. custom_msgs包
- ✅ **移动消息文件**: 将.msg文件从`srv/`移动到`msg/`目录
- ✅ **更新CMakeLists.txt**: 修正消息文件路径
- ✅ **package.xml**: 依赖配置已完善

### 2. following_robot包
- ✅ **setup.py**: 
  - 添加`bytetracker_node`入口点
  - 新增依赖: `scipy`, `lap`
- ✅ **package.xml**: 
  - 添加`python3-openpyxl`依赖

## 📋 安装说明

### 自动安装（推荐）
```bash
cd /userdata/try_again/SelfFollowingROS2
./install_dependencies.sh
```

### 手动安装
```bash
# 1. 安装系统依赖
sudo apt update
sudo apt install -y python3-pip python3-scipy python3-openpyxl

# 2. 安装Python依赖
pip3 install --user scipy lap openpyxl

# 3. 编译项目
cd /userdata/try_again/SelfFollowingROS2
colcon build
source install/setup.bash
```

## 🚀 使用方法

### 启动ByteTracker节点
```bash
# 多目标跟踪模式（默认）
ros2 launch following_robot bytetracker_launch.py

# 单目标跟踪模式  
ros2 launch following_robot bytetracker_launch.py tracking_mode:=single target_features_file:=/path/to/features.xlsx

# 启用小车控制
ros2 launch following_robot bytetracker_launch.py enable_car_control:=true

# 启用距离测量
ros2 launch following_robot bytetracker_launch.py enable_distance_measure:=true
```

### 运行时控制
```bash
# 切换跟踪模式
ros2 topic pub /bytetracker/set_mode std_msgs/String "data: 'single'"
ros2 topic pub /bytetracker/set_mode std_msgs/String "data: 'multi'"

# 设置目标人物  
ros2 topic pub /bytetracker/set_target std_msgs/String "data: 'person_001'"
```

### 查看跟踪结果
```bash
# 跟踪结果
ros2 topic echo /bytetracker/tracked_persons

# 状态信息
ros2 topic echo /bytetracker/status

# 可视化图像
ros2 run rqt_image_view rqt_image_view /bytetracker/visualization
```

## 🔧 参数配置

### 启动参数
- `tracking_mode`: 跟踪模式 (multi/single)
- `target_features_file`: 目标特征文件路径
- `camera_topic`: 相机话题名称
- `enable_car_control`: 是否启用小车控制
- `enable_distance_measure`: 是否启用距离测量
- `track_thresh`: 跟踪阈值 (默认0.5)
- `track_buffer`: 轨迹缓冲 (默认100)
- `match_thresh`: 匹配阈值 (默认0.8)  
- `color_weight`: 颜色权重 (默认0.5)

## 📊 话题接口

### 发布话题
- `/bytetracker/tracked_persons` - 跟踪结果
- `/bytetracker/visualization` - 可视化图像  
- `/bytetracker/status` - 状态信息
- `/robot_control/person_position` - 目标位置(仅启用控制时)

### 订阅话题
- `/camera/image_raw` - 输入图像
- `/bytetracker/set_mode` - 模式控制
- `/bytetracker/set_target` - 目标设置

### 服务客户端
- `/features/extract_features` - 特征提取服务
- `/stereo/get_distance` - 距离测量服务

## ⚠️ 注意事项

1. **依赖要求**: 确保安装了所有必需的Python库
2. **特征文件**: 单目标模式需要提供Excel格式的特征文件
3. **相机话题**: 确保相机节点正常工作
4. **性能**: ByteTracker算法计算密集，建议在性能较好的设备上运行

## 🐛 故障排除

### 常见问题
1. **导入错误**: 确保已正确编译custom_msgs包
2. **依赖缺失**: 运行install_dependencies.sh脚本
3. **相机无图像**: 检查相机话题是否正确
4. **跟踪不稳定**: 调整跟踪参数或提高图像质量

### 调试命令
```bash
# 检查节点状态
ros2 node list | grep bytetracker

# 检查话题
ros2 topic list | grep bytetracker  

# 查看日志
ros2 run following_robot bytetracker_node --ros-args --log-level debug
```

---
**更新完成时间**: $(date)  
**版本**: v1.0.0 