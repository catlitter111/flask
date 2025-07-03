# 视频特征提取完整指南

## 功能概述

本系统已升级支持视频特征提取的完整功能，包括：

1. **完整标注视频保存** - 不再只保存一张图片，而是保存完整的处理后视频
2. **实时进度显示** - 在每帧上显示处理进度和状态
3. **详细统计信息** - 包含帧计数、检测状态、有效检测数量等
4. **多种输出格式** - 同时生成标注视频、结果图像和特征数据Excel文件

## 主要改进

### 1. 视频处理增强
- **保存完整视频**：每帧都被处理并保存到输出视频中
- **实时标注**：在视频帧上显示检测结果、进度信息
- **状态可视化**：区分有效检测、无效检测、跳过帧等状态

### 2. 输出文件结构
```
output_directory/
├── {name}_video_result.mp4      # 完整标注视频（新增）
├── {name}_video_result.jpg      # 最后一帧结果图像
├── {name}_video_features.xlsx   # 特征数据Excel文件
└── ...
```

### 3. 处理流程优化
- **帧级别处理**：每帧都会被处理和保存
- **检测间隔控制**：可配置检测间隔和最大检测数量
- **进度追踪**：实时显示处理进度和预计剩余时间

## 使用方法

### 1. 启动完整系统
```bash
# 启动完整系统（包含特征提取节点）
ros2 launch following_robot full_system.launch.py

# 或者单独启动特征提取节点
ros2 launch following_robot feature_extraction_launch.py
```

### 2. 通过WebSocket上传视频
- 支持格式：`.mp4`, `.avi`, `.mov`, `.mkv`, `.flv`
- 通过微信小程序或Web客户端上传视频文件
- 系统会自动检测文件类型并触发视频处理

### 3. 直接调用ROS服务
```python
# 创建服务请求
request = FeatureExtraction.Request()
request.person_name = "VIDEO:/path/to/video.mp4"  # 使用VIDEO:前缀
request.save_to_file = True
request.output_path = "output_directory"

# 调用服务
future = client.call_async(request)
```

### 4. 使用测试脚本
```bash
# 运行测试脚本（会自动创建测试视频）
python test_video_feature_extraction.py
```

## 配置参数

### 节点参数
```yaml
# 特征提取节点参数
feature_extraction_node:
  ros__parameters:
    output_dir: "features-data"         # 输出目录
    video_frame_interval: 10            # 视频帧检测间隔（每N帧检测一次）
    video_detection_limit: 20           # 最大检测帧数
```

### 在launch文件中设置
```python
# 在launch文件中修改参数
declare_frame_interval = DeclareLaunchArgument(
    'video_frame_interval',
    default_value='10',
    description='视频帧检测间隔'
)

declare_detection_limit = DeclareLaunchArgument(
    'video_detection_limit', 
    default_value='20',
    description='最大检测帧数'
)
```

## 输出文件详解

### 1. 标注视频 (`*_video_result.mp4`)
- **完整处理视频**：包含所有帧的处理结果
- **实时标注**：
  - 帧号和进度百分比
  - 检测状态（有效/无效/跳过/失败）
  - 有效检测计数
  - 人体关键点和服装检测框
- **视频规格**：保持原始分辨率和帧率

### 2. 结果图像 (`*_video_result.jpg`)
- **最后有效帧**：最后一个成功检测的帧
- **统计信息**：总帧数、有效检测数、平均特征

### 3. 特征数据 (`*_video_features.xlsx`)
- **平均特征工作表**：
  - 16个身体比例的平均值
  - 平均上装和下装颜色
  - 统计信息（有效帧数、总检测帧数）
- **详细数据工作表**：
  - 每个有效检测帧的详细数据
  - 每帧的身体比例和颜色信息

## 视频帧标注说明

### 帧状态标识
- **VALID DETECTION** (绿色)：检测有效，数据已记录
- **INVALID DETECTION** (红色)：检测无效，数据已丢弃
- **DETECTION FAILED** (红色)：检测过程失败
- **SKIPPED FRAME** (灰色)：跳过的帧（根据间隔设置）

### 显示信息
- **帧信息**：`Frame: X/Y (Z.Z%)`
- **统计信息**：`Valid detections: N`
- **检测结果**：人体关键点、服装检测框、颜色信息

## 性能优化建议

### 1. 参数调整
```yaml
# 高精度模式（慢但准确）
video_frame_interval: 5      # 每5帧检测一次
video_detection_limit: 50    # 最多50次检测

# 快速模式（快但精度可能降低）
video_frame_interval: 20     # 每20帧检测一次
video_detection_limit: 10    # 最多10次检测
```

### 2. 硬件要求
- **CPU**：多核处理器（推荐8核以上）
- **内存**：至少4GB可用内存
- **存储**：SSD存储（提高视频写入性能）

### 3. 视频格式建议
- **分辨率**：640x480或1280x720（避免过高分辨率）
- **帧率**：30fps或以下
- **编码**：H.264编码的MP4格式

## 故障排除

### 1. 视频文件无法打开
   - 检查文件路径是否正确
- 确认视频格式是否支持
- 检查文件权限

### 2. 视频写入失败
- 检查输出目录权限
- 确认磁盘空间充足
- 检查视频编解码器是否可用

### 3. 检测结果不准确
- 调整检测间隔和限制参数
- 检查视频质量和光照条件
- 确认人体在视频中清晰可见

## API接口

### 服务接口
```
Service: /features/extract_features
Type: custom_msgs/srv/FeatureExtraction

Request:
- Image image                    # 输入图像（视频处理时可为空）
- string person_name            # 人物名称（视频处理时使用"VIDEO:路径"格式）
- bool save_to_file            # 是否保存文件
- string output_path           # 输出路径

Response:
- bool success                 # 处理是否成功
- string message               # 状态消息
- int32 person_count          # 检测到的人数
- float32[] body_ratios       # 身体比例数据（16个）
- int32[] shirt_color         # 上装颜色 [R, G, B]
- int32[] pants_color         # 下装颜色 [R, G, B]
- string result_image_path    # 结果图像路径
- string feature_data_path    # 特征数据路径
- string result_video_path    # 结果视频路径（新增）
```

### WebSocket接口
```javascript
// 文件上传响应
{
  type: 'feature_extraction_result',
  file_id: 'xxx',
  client_id: 'xxx',
  data: {
    status: 'success',
    message: '...',
    person_count: 1,
    body_ratios: [...],
    shirt_color: [r, g, b],
    pants_color: [r, g, b],
    result_image_path: '/path/to/image.jpg',
    result_video_path: '/path/to/video.mp4',  // 新增
    feature_data_path: '/path/to/data.xlsx',
    file_name: 'original_name.mp4',
    file_type: 'video',
    processing_info: {
      type: 'video_processing',
      has_annotated_video: true,
      has_result_image: true,
      has_feature_data: true
    }
  }
}
```

## 日志和调试

### 关键日志信息
```
[INFO] 🎬 检测到视频处理请求: /path/to/video.mp4
[INFO] 视频信息: 总帧数=1800, FPS=30.00, 分辨率=640x480, 时长=60.00秒
[INFO] 📹 视频写入器已创建: /path/to/output_video.mp4
[INFO] 进度: 300/1800 (16.7%), 预计剩余: 45.2秒
[INFO] ✅ 视频写入完成: /path/to/output_video.mp4
[INFO] 完整标注视频已保存到: /path/to/output_video.mp4 (大小: 25.67MB)
```

### 调试命令
```bash
# 查看节点状态
ros2 node info /feature_extraction_node

# 查看服务列表
ros2 service list | grep features

# 测试服务调用
ros2 service call /features/extract_features custom_msgs/srv/FeatureExtraction "{person_name: 'VIDEO:/path/to/test.mp4', save_to_file: true}"
```

---

## 总结

通过这次升级，视频特征提取系统现在能够：

1. **保存完整的标注视频**，而不仅仅是一张图片
2. **提供实时的处理进度和状态信息**
3. **支持灵活的配置和优化**
4. **提供完整的API接口和WebSocket支持**

这使得系统更适合用于视频分析、人体特征研究和实时监控等应用场景。 