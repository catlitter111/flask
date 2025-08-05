# ByteTracker与集成人员检测节点集成指南

## 📋 概述

本文档详细介绍了如何将`bytetracker_node.py`与`integrated_person_detection_node.cpp`集成，实现完整的人员检测与跟踪系统。

## 🔄 数据流架构

```
📷 相机 → 🔍 集成人员检测节点 → 🎯 ByteTracker节点 → 📊 跟踪结果
  |                |                    |              |
  |                |                    |              └─ 跟踪轨迹
  |                |                    └─ 多目标/单目标跟踪
  |                └─ 服装检测、颜色识别、身体比例
  └─ 图像输入
```

## 🗂️ 数据格式转换

### 集成检测节点输出格式 (JSON)
```json
{
    "timestamp": 1705123456789,
    "person_count": 2,
    "detection_mode": "FULL",
    "persons": [
        {
            "id": "Person_0",
            "bbox": [100, 50, 300, 400],
            "center": [200, 225],
            "clothing": {
                "upper": {
                    "bbox": [110, 60, 290, 180],
                    "color": "blue",
                    "color_rgb": [0, 100, 200],
                    "confidence": 0.85
                },
                "lower": {
                    "bbox": [120, 180, 280, 380],
                    "color": "black", 
                    "color_rgb": [20, 20, 20],
                    "confidence": 0.78
                }
            },
            "body_ratios": [1.2, 0.8, 0.3, ...],
            "has_body_ratios": true,
            "distance": 1.85,
            "valid_distance": true
        }
    ]
}
```

### ByteTracker期望输入格式
```python
detection_results = [
    (
        upper_bbox,      # [x1, y1, x2, y2] 上衣边界框
        lower_bbox,      # [x1, y1, x2, y2] 下装边界框  
        upper_color,     # (R, G, B) 上衣颜色
        lower_color,     # (R, G, B) 下装颜色
        upper_confidence, # float 上衣检测置信度
        lower_confidence, # float 下装检测置信度
        body_ratios      # [ratio1, ratio2, ...] 16个身体比例
    )
]
```

## ⚙️ 关键修改内容

### 1. integrated_person_detection_node.cpp 修改

在`publishPersonPositions`函数中添加了服装边界框和颜色RGB值输出：

```cpp
// 添加上衣边界框
Json::Value upper_bbox_array(Json::arrayValue);
upper_bbox_array.append(person.clothing.upper.bbox.x);
upper_bbox_array.append(person.clothing.upper.bbox.y);
upper_bbox_array.append(person.clothing.upper.bbox.x + person.clothing.upper.bbox.width);
upper_bbox_array.append(person.clothing.upper.bbox.y + person.clothing.upper.bbox.height);
upper_data["bbox"] = upper_bbox_array;

// 添加上衣颜色RGB值
Json::Value upper_color_rgb(Json::arrayValue);
upper_color_rgb.append(static_cast<int>(person.clothing.upper.color_rgb[2])); // R
upper_color_rgb.append(static_cast<int>(person.clothing.upper.color_rgb[1])); // G 
upper_color_rgb.append(static_cast<int>(person.clothing.upper.color_rgb[0])); // B
upper_data["color_rgb"] = upper_color_rgb;
```

### 2. bytetracker_node.py 修改

#### a) 移除DetectionFeatures导入，修改订阅话题：
```python
# 人员位置数据订阅者
self.person_positions_sub = self.create_subscription(
    String, self.person_positions_topic, 
    self.person_positions_callback, qos)
```

#### b) 添加JSON数据解析逻辑：
```python
def parse_person_positions_message(self, msg):
    """解析集成检测节点的JSON数据"""
    data = json.loads(msg.data)
    persons = data.get('persons', [])
    
    for person in persons:
        clothing = person.get('clothing', {})
        upper_info = clothing.get('upper', {})
        lower_info = clothing.get('lower', {})
        
        # 提取边界框和颜色信息
        upper_bbox = upper_info.get('bbox', [])
        upper_color = tuple(upper_info.get('color_rgb', [0, 0, 0])[:3])
        upper_confidence = float(upper_info.get('confidence', 0.0))
        
        # ... 类似处理下装信息
```

## 🚀 启动方式

### 1. 完整系统启动
```bash
# 启动集成系统（包含相机、检测、跟踪）
ros2 launch following_robot bytetracker_with_integrated_detection.launch.py
```

### 2. 自定义参数启动
```bash
# 多目标跟踪模式
ros2 launch following_robot bytetracker_with_integrated_detection.launch.py \
    tracking_mode:=multi \
    confidence_threshold:=0.3

# 单目标跟踪模式（需要特征文件）
ros2 launch following_robot bytetracker_with_integrated_detection.launch.py \
    tracking_mode:=single \
    target_features_file:=/path/to/target_features.xlsx
```

### 3. 分别启动节点
```bash
# 终端1: 启动集成检测节点
ros2 launch rknn_yolo11_ros2 integrated_person_detection.launch.py

# 终端2: 启动ByteTracker节点
ros2 run following_robot bytetracker_node \
    --ros-args \
    -p tracking_mode:=multi \
    -p person_positions_topic:=/person_detection/person_positions
```

## 📊 话题接口

### 输入话题
| 话题名 | 消息类型 | 描述 |
|--------|----------|------|
| `/person_detection/person_positions` | `std_msgs/String` | 检测节点的JSON数据 |
| `/camera/color/image_raw` | `sensor_msgs/Image` | 可选的图像输入 |
| `/bytetracker/set_mode` | `std_msgs/String` | 模式切换控制 |
| `/bytetracker/set_target` | `std_msgs/String` | 目标人物设置 |

### 输出话题  
| 话题名 | 消息类型 | 描述 |
|--------|----------|------|
| `/bytetracker/tracked_persons` | `TrackedPersonArray` | 跟踪结果数组 |
| `/bytetracker/tracking_result` | `TrackingResult` | 跟踪状态摘要 |
| `/bytetracker/detailed_tracking_data` | `std_msgs/String` | 详细跟踪数据(JSON) |
| `/bytetracker/status` | `std_msgs/String` | 节点状态信息 |

## 🧪 测试和验证

### 1. 运行集成测试
```bash
# 启动测试脚本
python3 test_bytetracker_integration.py
```

### 2. 手动验证数据流
```bash
# 检查检测数据
ros2 topic echo /person_detection/person_positions

# 检查跟踪结果  
ros2 topic echo /bytetracker/tracked_persons

# 检查跟踪状态
ros2 topic echo /bytetracker/tracking_result
```

### 3. 性能监控
```bash
# 监控话题频率
ros2 topic hz /person_detection/person_positions
ros2 topic hz /bytetracker/tracked_persons

# 监控详细性能数据
ros2 topic echo /bytetracker/detailed_tracking_data
```

## 🔧 参数配置

### ByteTracker参数
```yaml
# ByteTracker核心参数
track_thresh: 0.5          # 跟踪置信度阈值
track_buffer: 100          # 跟踪缓存帧数
match_thresh: 0.8          # 匹配阈值
color_weight: 0.5          # 颜色特征权重

# 话题配置
person_positions_topic: "/person_detection/person_positions"
input_image_topic: "/camera/color/image_raw"

# 模式配置  
tracking_mode: "multi"     # "multi" 或 "single"
target_features_file: ""   # 单目标模式的特征文件路径
```

### 集成检测节点参数
```yaml
# 检测参数
confidence_threshold: 0.3
nms_threshold: 0.5
enable_debug_display: true

# 模型路径
model_path: "/userdata/rknn_yolo11_ros2/model/cloth.rknn"
pose_model_path: "/userdata/rknn_yolo11_ros2/model/yolov8_pose.rknn"
```

## 🐛 故障排除

### 常见问题

1. **无跟踪结果输出**
   - 检查检测节点是否正常运行
   - 调低`track_thresh`参数
   - 确认检测数据格式正确

2. **跟踪不稳定**
   - 增加`track_buffer`参数
   - 调整`match_thresh`参数
   - 检查颜色特征是否有效

3. **数据格式错误**
   - 确认集成检测节点版本正确
   - 检查JSON数据结构
   - 验证边界框格式

4. **性能问题**
   - 调整置信度阈值
   - 减少跟踪目标数量
   - 检查系统资源使用

### 调试命令
```bash
# 检查节点状态
ros2 node list | grep -E "(integrated|bytetracker)"

# 检查话题连接
ros2 topic info /person_detection/person_positions
ros2 topic info /bytetracker/tracked_persons

# 查看详细日志
ros2 run following_robot bytetracker_node --ros-args --log-level debug
```

## 📈 性能优化建议

1. **检测节点优化**
   - 使用适当的置信度阈值
   - 根据需要选择FULL或PARTIAL检测模式
   - 调整图像分辨率

2. **跟踪节点优化**
   - 合理设置跟踪参数
   - 使用颜色特征加速匹配
   - 限制最大跟踪目标数

3. **系统优化**
   - 使用高效的QoS设置
   - 避免不必要的数据拷贝
   - 监控内存使用

## 📚 扩展功能

1. **添加距离信息**
   - 集成深度相机数据
   - 实现3D跟踪

2. **增强特征匹配**
   - 添加更多身体特征
   - 实现面部识别

3. **改进跟踪算法**
   - 集成更先进的跟踪算法
   - 添加预测轨迹

4. **可视化增强**
   - 实时轨迹显示
   - 多视角支持

## 🔗 相关文档

- [YOLO11集成检测使用指南](../rknn_yolo11_ros2/INTEGRATED_PERSON_DETECTION_USAGE.md)
- [ByteTracker算法文档](BYTETRACKER_ALGORITHM.md)
- [ROS2消息接口定义](src/custom_msgs/)
- [性能测试报告](PERFORMANCE_TEST_REPORT.md) 