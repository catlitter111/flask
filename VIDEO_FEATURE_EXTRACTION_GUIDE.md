# 视频特征提取功能指南

## 概述

特征提取节点现在支持从视频文件中提取人体特征。系统会自动在视频中每隔几帧进行检测，收集有效的特征数据，并计算平均值。

## 新增功能

### 🎬 视频处理能力
- **帧间隔检测**: 每隔指定帧数检测一次，提高处理效率
- **有效性过滤**: 自动排除无效检测结果，只计算有效数据的平均值
- **平均值计算**: 对多帧检测结果进行平均，提高特征准确性
- **详细数据保存**: 同时保存每帧数据和平均值到Excel文件

### 📊 检测结果
- **16个身体比例**: 上肢下肢比例、躯干身高比例、肩宽身高比例等
- **服装颜色检测**: 上装和下装颜色信息
- **统计信息**: 有效检测帧数、总检测帧数等

## 配置参数

在启动节点时可以设置以下参数：

```bash
ros2 run following_robot feature_extraction_node \
  --ros-args \
  -p video_frame_interval:=10 \
  -p video_detection_limit:=20 \
  -p output_dir:=features-data
```

### 参数说明
- `video_frame_interval`: 视频帧检测间隔（默认：10帧）
- `video_detection_limit`: 最大检测帧数（默认：20帧）  
- `output_dir`: 输出目录（默认：features-data）

## 使用方法

### 方法1: 服务调用

使用特殊的`person_name`前缀来标识视频处理：

```python
import rclpy
from custom_msgs.srv import FeatureExtraction
from sensor_msgs.msg import Image

# 创建请求
request = FeatureExtraction.Request()
request.person_name = "VIDEO:/path/to/video.mp4"  # 使用VIDEO:前缀
request.save_to_file = True
request.output_path = ""  # 使用默认输出路径
request.image = Image()  # 空的图像消息

# 发送请求
client = node.create_client(FeatureExtraction, '/features/extract_features')
future = client.call_async(request)
```

### 方法2: 测试脚本

使用提供的测试脚本：

```bash
# 启动特征提取节点
ros2 run following_robot feature_extraction_node

# 在另一个终端运行测试
python test_video_feature_extraction.py /path/to/video.mp4
```

## 处理流程

1. **视频加载**: 打开视频文件并获取基本信息（帧数、FPS等）
2. **帧间隔检测**: 每隔指定帧数提取一帧进行分析
3. **特征提取**: 对每帧进行人体姿态检测和服装颜色识别
4. **有效性验证**: 检查检测结果是否有效（关键点置信度、颜色信息等）
5. **数据收集**: 收集所有有效的检测结果
6. **平均值计算**: 计算身体比例和颜色的平均值
7. **结果保存**: 保存平均特征和详细数据到文件

## 有效性判断标准

系统会自动过滤无效检测，判断标准包括：

- **身体比例有效性**: 至少30%的比例值不为0
- **颜色有效性**: 检测到的颜色不是纯黑色(0,0,0)
- **综合判断**: 身体比例有效 OR 颜色检测有效

## 输出文件

### 结果图像
- 文件名: `{name}_video_result.jpg`
- 内容: 带有人体关键点和服装检测标注的最后一帧
- 统计信息: 显示有效帧数/总检测帧数

### Excel数据文件
- 文件名: `{name}_video_features.xlsx`
- 工作表1: `平均特征_{name}` - 包含平均的16个身体比例和颜色
- 工作表2: `详细数据_{name}` - 包含每帧的详细检测数据

## 示例输出

```
[INFO] 视频信息: 总帧数=300, FPS=30.00, 时长=10.00秒
[INFO] 检测设置: 每10帧检测一次, 最多检测20帧
[INFO] 第 0 帧检测有效
[INFO] 第 10 帧检测有效  
[INFO] 第 20 帧检测无效，跳过
...
[INFO] 共收集到 15 个有效检测结果，开始计算平均值
[INFO] 视频处理完成，耗时: 45.67秒，有效检测: 15/20
```

## 性能建议

1. **视频长度**: 建议处理10-30秒的视频片段
2. **帧间隔**: 根据视频FPS调整，一般10-15帧间隔较合适
3. **检测限制**: 限制最大检测帧数可控制处理时间
4. **视频质量**: 清晰度越高，检测准确性越好

## 故障排除

### 常见问题

1. **视频文件无法打开**
   - 检查文件路径是否正确
   - 确认视频格式被OpenCV支持

2. **检测结果全部无效**
   - 视频中人体不够清晰
   - 调整检测参数或使用质量更好的视频

3. **处理时间过长**
   - 增大帧间隔`video_frame_interval`
   - 减少检测限制`video_detection_limit`

### 日志级别
- `INFO`: 正常处理信息
- `DEBUG`: 详细的帧处理信息
- `WARN`: 检测失败但不影响整体处理
- `ERROR`: 严重错误，处理中断

## 集成建议

视频特征提取功能可以集成到以下场景：

1. **离线特征库建立**: 批量处理训练视频建立特征数据库
2. **跟踪目标优化**: 使用视频片段获得更准确的目标特征  
3. **系统测试验证**: 使用标准测试视频验证检测精度
4. **特征对比分析**: 比较不同视频中的人体特征差异 