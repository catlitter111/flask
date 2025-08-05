# ByteTracker 可视化跟踪功能指南

## 概述

本指南介绍了新增的 ByteTracker 可视化跟踪功能，该功能可以：
- 订阅 Astra 相机的原始图像  
- 实时显示跟踪结果
- 用不同颜色框标识不同类型的目标
- 提供直观的交互界面

## 功能特性

### 🎯 可视化标识
- **红色框**: 目标跟踪对象（在单目标模式下）
- **绿色框**: 其他检测到的人员
- **中心点**: 每个跟踪对象的中心位置
- **颜色块**: 显示检测到的上衣(U)和下装(L)颜色

### 📊 信息显示
- 实时帧数和跟踪统计
- 跟踪模式状态
- FPS 性能指标
- 目标信息（ID 和置信度）

### ⌨️ 交互控制
- 按 `q` 或 `ESC` 键退出程序
- 通过 ROS2 话题动态切换跟踪模式

## 启动方法

### 1. 完整系统启动

使用新的 launch 文件启动完整的检测和跟踪系统：

```bash
cd /userdata/SelfFollowingROS2
ros2 launch following_robot bytetracker_integrated_detection.launch.py
```

### 2. 自定义参数启动

```bash
# 多目标跟踪模式（默认）
ros2 launch following_robot bytetracker_integrated_detection.launch.py \
    tracking_mode:=multi \
    enable_display:=true

# 单目标跟踪模式  
ros2 launch following_robot bytetracker_integrated_detection.launch.py \
    tracking_mode:=single \
    target_features_file:=/path/to/target_features.xlsx \
    enable_display:=true

# 关闭可视化显示
ros2 launch following_robot bytetracker_integrated_detection.launch.py \
    enable_display:=false
```

## 启动参数说明

### 基础参数
- `camera_name`: 相机名称前缀（默认: 'camera'）
- `input_topic`: 输入图像话题（默认: '/camera/color/image_raw'）
- `confidence_threshold`: 检测置信度阈值（默认: 0.3）

### 跟踪参数
- `tracking_mode`: 跟踪模式 - 'multi' 或 'single'（默认: 'multi'）
- `track_thresh`: 跟踪置信度阈值（默认: 0.5）
- `color_weight`: 颜色匹配权重（默认: 0.5）
- `target_features_file`: 目标特征文件路径（单目标模式使用）

### 显示参数
- `enable_display`: 启用 ByteTracker 图像显示（默认: true）
- `enable_debug_display`: 启用集成检测节点调试显示（默认: false）

## 节点架构

```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────────────┐
│  astra_camera   │───▶│  depth_service   │    │  integrated_person      │
│     _node       │    │      _node       │    │   _detection_node       │
└─────────────────┘    └──────────────────┘    └─────────────────────────┘
         │                                                   │
         │ /camera/color/image_raw                          │ /person_detection/
         │                                                   │ person_positions
         ▼                                                   ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                         bytetracker_node                                │
│  • 订阅图像和检测数据                                                  │
│  • 执行多目标/单目标跟踪                                               │
│  • 绘制跟踪结果并显示图像                                              │
│  • 发布跟踪结果消息                                                    │
└─────────────────────────────────────────────────────────────────────────┘
```

## 话题说明

### 输入话题
- `/camera/color/image_raw`: 原始图像数据
- `/person_detection/person_positions`: 人员检测结果（JSON格式）

### 输出话题
- `/bytetracker/tracked_persons`: 跟踪人员数组
- `/bytetracker/tracking_result`: 跟踪结果总结
- `/bytetracker/detailed_tracking_data`: 详细跟踪数据（JSON）
- `/bytetracker/status`: 节点状态信息

### 控制话题
- `/bytetracker/set_mode`: 切换跟踪模式（发送 "multi" 或 "single"）
- `/bytetracker/set_target`: 设置目标人物名称

## 运行时控制

### 切换到多目标模式
```bash
ros2 topic pub --once /bytetracker/set_mode std_msgs/String "data: 'multi'"
```

### 切换到单目标模式
```bash
ros2 topic pub --once /bytetracker/set_mode std_msgs/String "data: 'single'"
```

### 设置目标人物
```bash
ros2 topic pub --once /bytetracker/set_target std_msgs/String "data: '目标人物名称'"
```

## 可视化界面说明

### 主窗口: "ByteTracker - Person Tracking"
- 显示实时相机图像
- 叠加跟踪结果
- 显示系统状态信息

### 信息显示区域
- 左上角: 帧数、跟踪数量、模式、FPS
- 左下角: 目标信息（如果存在）
- 跟踪框: 每个人员的边界框和标签
- 颜色块: 服装颜色信息（U=上衣，L=下装）

## 故障排除

### 1. 图像不显示
- 检查相机连接
- 确认 `enable_display` 参数为 true
- 查看图像话题是否正确发布

### 2. 跟踪效果不佳
- 调整 `confidence_threshold` 和 `track_thresh` 参数
- 调整 `color_weight` 影响颜色匹配权重
- 检查光照条件

### 3. 程序崩溃
- 检查依赖包安装（opencv、cv_bridge）
- 查看控制台错误信息
- 确认模型文件路径正确

## 性能优化建议

1. **降低图像分辨率**: 可适当降低相机分辨率提升性能
2. **调整跟踪参数**: 根据场景调整跟踪阈值
3. **关闭调试显示**: 在性能要求高的场景下关闭可视化
4. **优化颜色权重**: 根据环境光照调整颜色匹配权重

## 扩展功能

该系统支持进一步扩展：
- 添加轨迹记录和回放
- 集成目标预测和行为分析
- 支持多相机融合跟踪
- 添加远程监控界面

---

**注意**: 确保系统具有足够的计算资源来同时运行 YOLO11 检测和 ByteTracker 跟踪算法。建议在 GPU 加速环境下运行以获得最佳性能。 