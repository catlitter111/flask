# WebSocket视频特征提取功能指南

## 概述

WebSocket桥接节点现在支持完整的视频特征提取功能。用户可以通过WebSocket连接上传视频文件，系统会自动进行特征提取并返回与图片处理相同格式的结果数据。

## 🎬 新增功能

### 视频处理能力
- **视频文件上传**: 支持MP4、AVI、MOV、MKV、FLV等格式
- **自动特征提取**: 视频上传后自动触发特征提取
- **统一数据格式**: 视频和图片返回相同的特征数据结构
- **帧间隔检测**: 每隔指定帧数检测，取有效结果的平均值

### 支持的视频格式
- `.mp4` - MP4视频
- `.avi` - AVI视频  
- `.mov` - QuickTime视频
- `.mkv` - Matroska视频
- `.flv` - Flash视频

## 🔧 实现原理

### WebSocket桥接节点更新
1. **文件类型识别**: 根据文件扩展名自动识别视频文件
2. **特殊标识传递**: 使用`VIDEO:`前缀标识视频处理请求
3. **统一响应处理**: 视频和图片使用相同的响应处理逻辑

### 特征提取节点集成
1. **前缀识别**: 检测到`VIDEO:`前缀后切换到视频处理模式
2. **视频解析**: 自动解析视频帧并进行特征检测
3. **结果聚合**: 计算多帧检测结果的平均值

## 📊 数据格式

### 请求格式（WebSocket消息）
```json
{
  "type": "file_upload",
  "file_id": "video_test_1234567890",
  "file_name": "sample_video.mp4",
  "file_type": "video/mp4",
  "file_size": 12345678,
  "file_data_base64": "base64编码的视频数据...",
  "client_id": "client_001",
  "timestamp": 1640995200000,
  "extract_features": true
}
```

### 响应格式（WebSocket消息）
```json
{
  "type": "feature_extraction_result",
  "file_id": "video_test_1234567890",
  "client_id": "client_001",
  "data": {
    "status": "success",
    "message": "特征提取成功，耗时: 45.67秒",
    "person_count": 1,
    "body_ratios": [0.123, 0.456, ...],  // 16个身体比例
    "shirt_color": [128, 64, 200],        // 上装RGB颜色
    "pants_color": [50, 100, 150],        // 下装RGB颜色
    "result_image_path": "/path/to/result.jpg",
    "feature_data_path": "/path/to/features.xlsx",
    "file_name": "sample_video.mp4",
    "file_type": "video"
  },
  "timestamp": 1640995200000
}
```

## 🚀 使用方法

### 1. 启动系统
```bash
# 启动特征提取节点
ros2 run following_robot feature_extraction_node

# 启动WebSocket桥接节点
ros2 run following_robot websocket_bridge_node

# 启动WebSocket服务器
python 服务端/server.py
```

### 2. 客户端上传视频
```python
import websocket
import json
import base64

# 读取视频文件
with open('video.mp4', 'rb') as f:
    video_data = f.read()

# 构建上传消息
message = {
    'type': 'file_upload',
    'file_name': 'video.mp4',
    'file_type': 'video/mp4',
    'file_size': len(video_data),
    'file_data_base64': base64.b64encode(video_data).decode(),
    'client_id': 'test_client',
    'extract_features': True
}

# 发送到WebSocket服务器
ws.send(json.dumps(message))
```

### 3. 测试脚本
```bash
# 使用提供的测试脚本
python test_websocket_video_features.py /path/to/video.mp4
```

## 📈 性能特点

### 处理效率
- **帧间隔采样**: 默认每10帧检测一次，可配置
- **检测限制**: 默认最多检测20帧，可配置
- **有效性过滤**: 自动排除无效检测，只计算有效数据

### 准确性提升
- **多帧平均**: 对多个有效检测结果求平均值
- **异常值过滤**: 自动排除置信度低的检测结果
- **颜色聚合**: 智能计算服装颜色的平均值

### 配置参数
```bash
# 启动时可配置视频处理参数
ros2 run following_robot feature_extraction_node \
  --ros-args \
  -p video_frame_interval:=10 \    # 帧间隔
  -p video_detection_limit:=20     # 最大检测帧数
```

## 🔍 检测质量

### 有效性判断标准
- **身体比例**: 至少30%的比例值 > 0
- **颜色检测**: 检测到非纯黑色(0,0,0)
- **综合评估**: 身体比例有效 OR 颜色检测有效

### 结果可靠性
- **多帧验证**: 基于多帧检测结果的一致性
- **置信度阈值**: 关键点置信度 > 0.5
- **统计过滤**: 异常值自动剔除

## 📁 输出文件

### 结果图像
- **文件名**: `{name}_video_result.jpg`
- **内容**: 最后一个有效检测帧的标注结果
- **标注信息**: 人体关键点 + 服装区域 + 统计信息

### 特征数据
- **文件名**: `{name}_video_features.xlsx`
- **工作表1**: 平均特征数据（16个比例 + 颜色）
- **工作表2**: 每帧详细数据（可选）

## 🛠️ 故障排除

### 常见问题

1. **视频格式不支持**
   - 确认文件扩展名是否在支持列表中
   - 检查视频编码格式是否被OpenCV支持

2. **文件大小限制**
   - 当前限制: 50MB
   - 建议: 使用较短的视频片段（10-30秒）

3. **检测结果全部无效**
   - 检查视频清晰度和人体可见性
   - 调整检测参数或使用更高质量的视频

4. **处理时间过长**
   - 增大`video_frame_interval`参数
   - 减少`video_detection_limit`参数
   - 使用较短的视频

### 日志分析
```bash
# 查看详细日志
ros2 topic echo /rosout

# 关键日志消息
[INFO] 🎥 开始视频特征提取 - 文件: sample.mp4
[INFO] 📤 发送视频特征提取服务请求
[INFO] ✅ 视频特征提取成功 - 文件: sample.mp4
```

## 🔮 未来扩展

### 计划功能
1. **实时视频流处理**: 支持摄像头实时视频
2. **批量视频处理**: 一次上传多个视频文件
3. **视频质量优化**: 自动调整视频质量以适应网络
4. **高级分析**: 动作识别、情感分析等

### 性能优化
1. **GPU加速**: 支持CUDA加速处理
2. **分布式处理**: 多节点并行视频处理
3. **缓存机制**: 相似视频的特征缓存
4. **增量更新**: 仅处理视频的新增部分

## 📞 技术支持

如果在使用过程中遇到问题，可以：

1. 查看系统日志进行问题定位
2. 使用测试脚本验证功能正常性
3. 检查网络连接和服务状态
4. 确认视频文件格式和大小符合要求

---

WebSocket视频特征提取功能现已完全集成到系统中，为用户提供了统一、高效的多媒体特征提取解决方案！🎉 