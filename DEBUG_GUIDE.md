# ByteTracker Node 调试指南

本指南将帮助你在 Cursor 中调试 `bytetracker_node.py` 文件。

## 📋 前置条件

1. **安装必要的扩展**：
   - Python 扩展
   - ROS 扩展（可选）

2. **系统要求**：
   - ROS2 Humble
   - Python 3.10+
   - OpenCV
   - 必要的 Python 包

## 🚀 快速开始

### 1. 环境设置

```bash
# 运行调试环境设置脚本
source debug_setup.sh
```

### 2. 构建项目

```bash
# 构建 ROS2 包
colcon build --packages-select following_robot custom_msgs --symlink-install
```

### 3. 启动调试

1. 在 Cursor 中打开调试面板 (`Ctrl+Shift+D`)
2. 选择以下调试配置之一：
   - **调试 ByteTracker 节点 - 单目标跟踪** - 单目标跟踪模式（推荐）
   - **调试 ByteTracker 节点 - 多目标跟踪** - 多目标跟踪模式  
   - **调试 ByteTracker 节点 (无相机) - 单目标跟踪** - 无摄像头的单目标调试
   - **调试 ByteTracker 节点 (单步调试) - 单目标跟踪** - 从程序开始逐行调试
   - **调试 ByteTracker 节点 - 视频文件单目标跟踪** - 使用视频文件调试（推荐用于算法调试）
   - **调试 ByteTracker 节点 - 视频文件单目标跟踪 (单步调试)** - 视频文件单步调试
3. 按 `F5` 开始调试

## 🔧 调试配置详解

### 配置1: 调试 ByteTracker 节点 - 单目标跟踪 (推荐)
- **用途**: 单目标跟踪调试，需要摄像头
- **特征**: 自动加载 `features-data/person.xlsx` 特征文件
- **参数**: `tracking_mode=single`, `camera_id=1`
- **适用场景**: 调试单人跟踪功能，有硬件设备时使用

### 配置2: 调试 ByteTracker 节点 - 多目标跟踪
- **用途**: 多目标跟踪调试模式
- **参数**: `tracking_mode=multi`, `camera_id=1`
- **适用场景**: 调试多人跟踪功能

### 配置3: 调试 ByteTracker 节点 (无相机) - 单目标跟踪
- **用途**: 无摄像头的单目标跟踪调试
- **环境**: 设置了 `BYTETRACKER_NO_CAMERA=1`
- **特征**: 加载特征文件但不使用相机
- **适用场景**: 没有摄像头设备时的逻辑调试

### 配置4: 调试 ByteTracker 节点 (单步调试) - 单目标跟踪
- **用途**: 从程序入口开始单步调试单目标跟踪
- **特点**: `stopOnEntry=true`, `justMyCode=false`
- **参数**: 包含单目标跟踪模式和特征文件路径
- **适用场景**: 详细分析单目标跟踪的程序执行流程

### 配置5: 调试 ByteTracker 节点 - 视频文件单目标跟踪 (推荐用于算法调试)
- **用途**: 使用视频文件进行单目标跟踪调试
- **特征**: 
  - 自动禁用距离测量和立体视觉功能
  - 可重复播放同一视频进行测试
  - 支持多种视频格式（mp4、avi、mov等）
- **参数**: `video_file_path=${workspaceFolder}/test_video.mp4`
- **适用场景**: 算法开发和测试，无需硬件设备

### 配置6: 调试 ByteTracker 节点 - 视频文件单目标跟踪 (单步调试)
- **用途**: 使用视频文件进行单步调试
- **特点**: `stopOnEntry=true`, `justMyCode=false`
- **参数**: 包含视频文件路径和单目标跟踪配置
- **适用场景**: 详细分析视频文件处理流程和算法逻辑

## 🎯 断点设置建议

### 关键调试点：

1. **初始化阶段**：
   ```python
   # 第 2641 行 - ByteTrackerNode.__init__()
   # 检查节点初始化过程
   ```

2. **帧处理核心**：
   ```python
   # 第 2747 行 - process_frame()
   # 核心处理逻辑，建议重点调试
   ```

3. **特征提取**：
   ```python
   # 第 2898 行 - detect_and_extract_features()
   # 检查检测和特征提取过程
   ```

4. **跟踪更新**：
   ```python
   # 第 1736 行 - ColorByteTracker.update()
   # 跟踪算法核心逻辑
   ```

5. **单目标跟踪**：
   ```python
   # 第 2353 行 - SingleTargetTracker.update()
   # 单目标跟踪逻辑
   ```

## 🐛 常见调试场景

### 场景1: 相机初始化失败
```python
# 在第 3685 行设置断点
def setup_camera(self):
    # 检查相机设备和参数配置
```

### 场景2: 特征提取异常
```python
# 在第 2958 行设置断点
def extract_body_ratios_from_detections(self, frame, detection_results, calculate_ratios=True):
    # 检查身体比例计算过程
```

### 场景3: 跟踪匹配问题
```python
# 在第 1197 行设置断点
def iou_distance(atracks, btracks):
    # 检查 IOU 距离计算
```

### 场景4: ROS2 消息发布问题
```python
# 在第 3272 行设置断点
def publish_tracking_results(self, tracks, target_track):
    # 检查消息发布过程
```

## 🔍 调试技巧

### 变量监视
在调试过程中重点关注以下变量：
- `self.current_frame` - 当前处理的图像帧
- `detection_results` - 检测结果
- `tracks` - 当前跟踪的轨迹列表
- `self.tracker` - 跟踪器状态
- `self.single_target_tracker` - 单目标跟踪器状态

### 条件断点
```python
# 只在特定条件下停止
track.track_id == 1  # 只调试 ID 为 1 的轨迹
len(tracks) > 5      # 只在检测到多个目标时停止
```

### 日志断点
```python
# 在断点处执行表达式而不停止
print(f"处理第 {self.frame_count} 帧，检测到 {len(detection_results)} 个目标")
```

## 📊 性能调试

### 时间分析
```python
# 在关键函数入口和出口设置断点
import time
start_time = time.time()
# ... 函数执行 ...
end_time = time.time()
print(f"函数执行时间: {end_time - start_time:.3f}s")
```

### 内存监控
```python
import psutil
import os
process = psutil.Process(os.getpid())
print(f"内存使用: {process.memory_info().rss / 1024 / 1024:.2f} MB")
```

## 🚨 故障排除

### 问题1: ROS2 环境变量未设置
**解决方案**: 运行 `source debug_setup.sh`

### 问题2: Python 模块导入失败
**解决方案**: 检查 PYTHONPATH 和包安装

### 问题3: 相机设备权限问题
**解决方案**: 
```bash
sudo chmod 666 /dev/video*
# 或者使用无相机模式调试
```

### 问题4: 特征文件路径问题
**解决方案**: 检查 `features-data` 目录是否存在

### 问题5: 视频文件无法打开
**解决方案**: 
```bash
# 检查视频文件格式和编码
ffmpeg -i test_video.mp4
# 转换为兼容格式
ffmpeg -i input.mp4 -c:v libx264 -c:a aac test_video.mp4
```

## 💡 高级调试技巧

### 1. 视频文件调试 (推荐用于算法开发)

**优势**:
- 可重复测试：同一视频可以反复播放，确保测试的一致性
- 无硬件依赖：不需要相机设备，便于算法开发和测试
- 自动优化：系统会自动禁用距离测量，避免不必要的计算

**使用步骤**:
1. 准备测试视频文件（建议包含目标人物的单目标跟踪场景）
2. 将视频文件放在项目根目录，命名为 `test_video.mp4`
3. 选择 "调试 ByteTracker 节点 - 视频文件单目标跟踪" 配置
4. 设置断点进行调试

**视频要求**:
- 格式：mp4、avi、mov 等 OpenCV 支持的格式
- 编码：H.264 (推荐)
- 分辨率：建议 1280x720 或更高
- 帧率：15-30 FPS

### 2. 远程调试
如果需要在远程设备上调试，可以使用 `debugpy`：
```python
import debugpy
debugpy.listen(5678)
debugpy.wait_for_client()
```

### 3. 多进程调试
ROS2 节点可能涉及多进程，注意设置：
```json
"subProcess": true
```

### 3. 实时数据可视化
在调试控制台中运行：
```python
import matplotlib.pyplot as plt
plt.imshow(frame)
plt.show()
```

## 📝 调试日志

建议在调试过程中记录：
- 断点位置和触发条件
- 关键变量的值变化
- 异常和错误信息
- 性能瓶颈点

---

**祝调试顺利！** 🎉

如果遇到问题，可以查看调试控制台的输出信息，或者检查 ROS2 日志。 