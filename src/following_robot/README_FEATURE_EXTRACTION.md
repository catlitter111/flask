# 人体特征提取功能使用说明

## 概述

人体特征提取节点是一个ROS2服务节点，基于RKNN模型进行人体服装检测和姿态估计，提取16个身体比例特征和服装颜色信息，并保存为Excel文件。

## 功能特点

- 🤖 **服装检测**: 检测上装和下装，提取颜色信息
- 🦴 **姿态估计**: 检测17个人体关键点
- 📊 **特征计算**: 计算16个身体比例特征
- 📁 **数据保存**: 自动保存Excel特征文件和结果图像
- 🔧 **ROS2集成**: 提供标准ROS2服务接口

## 安装和编译

1. **确保依赖项已安装**:
```bash
# Python依赖
pip3 install opencv-python numpy openpyxl pillow

# ROS2依赖
sudo apt install ros-humble-cv-bridge
```

2. **编译项目**:
```bash
cd /path/to/your/workspace
colcon build --packages-select custom_msgs following_robot
source install/setup.bash
```

## 使用方法

### 1. 启动特征提取节点

**方法一: 直接运行节点**
```bash
ros2 run following_robot feature_extraction_node
```

**方法二: 使用启动文件**
```bash
ros2 launch following_robot feature_extraction_launch.py
```

### 2. 测试功能

**快速测试**:
```bash
# 使用图像文件测试
python3 src/following_robot/scripts/quick_test.py <图像路径> [人物名称]

# 示例
python3 src/following_robot/scripts/quick_test.py test.jpg person1
```

**详细测试**:
```bash
# 图像文件测试
python3 src/following_robot/scripts/simple_feature_test.py <图像路径> --name <人物名称> --output <输出路径>

# 示例
python3 src/following_robot/scripts/simple_feature_test.py /path/to/image.jpg --name "张三" --output "./results"
```

**交互式测试**:
```bash
# 摄像头测试
python3 src/following_robot/scripts/test_feature_extraction.py --camera [摄像头ID] [人物名称]

# 示例
python3 src/following_robot/scripts/test_feature_extraction.py --camera 1 person1
```

## 服务接口

### 服务名称
`/features/extract_features`

### 服务类型
`custom_msgs/srv/FeatureExtraction`

### 请求参数
```
sensor_msgs/Image image         # 输入图像
string person_name             # 人物标识名称
bool save_to_file              # 是否保存到文件
string output_path             # 输出路径（可选）
```

### 响应参数
```
bool success                   # 提取是否成功
string message                 # 错误或状态消息
int32 person_count            # 检测到的人数
float32[] body_ratios         # 身体比例数据（16个）
int32[] shirt_color           # 上装颜色 [R, G, B]
int32[] pants_color           # 下装颜色 [R, G, B]
string result_image_path      # 结果图像保存路径
string feature_data_path      # 特征数据保存路径
```

## 身体比例特征说明

提取的16个身体比例特征包括：

1. **上肢/下肢比例**: 手臂长度与腿部长度的比例
2. **躯干/身高比例**: 躯干高度与总身高的比例
3. **肩宽/身高比例**: 肩膀宽度与身高的比例
4. **臀宽/肩宽比例**: 髋部宽度与肩膀宽度的比例
5. **头部/躯干比例**: 头部高度与躯干高度的比例
6. **手臂/身高比例**: 手臂长度与身高的比例
7. **腿长/身高比例**: 腿部长度与身高的比例
8. **上臂/下臂比例**: 上臂与下臂长度的比例
9. **大腿/小腿比例**: 大腿与小腿长度的比例
10. **躯干/腿长比例**: 躯干与腿部长度的比例
11. **手臂/腿长比例**: 手臂与腿部长度的比例
12. **肩宽/髋宽比例**: 肩膀与髋部宽度的比例
13. **头围/身高比例**: 估算头围与身高的比例
14. **脚长/身高比例**: 估算脚长与身高的比例
15. **脚踝宽/身高比例**: 脚踝宽度与身高的比例
16. **腰围/身高比例**: 估算腰围与身高的比例

## 输出文件

### Excel特征文件格式
- **文件名**: `{person_name}_features.xlsx`
- **A1-A16**: 16个身体比例特征值
- **A17**: 上装颜色 (字符串格式: "(R,G,B)")
- **A18**: 下装颜色 (字符串格式: "(R,G,B)")

### 结果图像
- **文件名**: `{person_name}_result.jpg`
- **内容**: 原图像 + 检测框 + 关键点 + 骨架

## 命令行工具

### 1. 快速测试工具
```bash
python3 src/following_robot/scripts/quick_test.py <图像路径> [人物名称]
```

### 2. 详细测试工具
```bash
python3 src/following_robot/scripts/simple_feature_test.py <图像路径> [选项]
```

选项:
- `--name, -n`: 人物名称
- `--output, -o`: 输出路径

### 3. 验证工具
```bash
python3 src/following_robot/scripts/verify_feature_extraction.py
```

## 编程接口示例

### Python客户端示例
```python
import rclpy
from rclpy.node import Node
from custom_msgs.srv import FeatureExtraction
from cv_bridge import CvBridge
import cv2

class FeatureClient(Node):
    def __init__(self):
        super().__init__('feature_client')
        self.client = self.create_client(FeatureExtraction, '/features/extract_features')
        self.bridge = CvBridge()
    
    def extract_features(self, image_path, person_name):
        # 读取图像
        cv_image = cv2.imread(image_path)
        ros_image = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
        
        # 创建请求
        request = FeatureExtraction.Request()
        request.image = ros_image
        request.person_name = person_name
        request.save_to_file = True
        
        # 发送请求
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        return future.result()

# 使用示例
rclpy.init()
client = FeatureClient()
response = client.extract_features("test.jpg", "person1")
print(f"成功: {response.success}")
print(f"特征数量: {len(response.body_ratios)}")
```

## 故障排除

### 1. 服务不可用
```
❌ 特征提取服务在 10 秒内未响应
```
**解决方案**: 确保特征提取节点正在运行
```bash
ros2 run following_robot feature_extraction_node
```

### 2. 模型文件缺失
```
⚠️ 服装检测模块导入失败
⚠️ 姿态检测模块导入失败
```
**解决方案**: 确保模型文件存在于 `src/following_robot/data/` 目录
- `best3.rknn` (服装检测模型)
- `yolov8_pose.rknn` (姿态检测模型)

### 3. 依赖项缺失
```
ImportError: No module named 'cv_bridge'
```
**解决方案**: 安装缺失的依赖项
```bash
sudo apt install ros-humble-cv-bridge
pip3 install opencv-python openpyxl
```

### 4. 权限问题
```
❌ 无法保存文件到目录
```
**解决方案**: 确保有写入权限或指定可写目录
```bash
chmod 755 features-data/
```

## 性能优化

1. **图像尺寸**: 建议输入图像尺寸不超过1920x1080以获得最佳性能
2. **模型加载**: 首次运行时模型加载需要几秒钟时间
3. **内存使用**: 处理大图像时可能需要较多内存

## 更新日志

- **v1.0.0**: 初始版本，支持基本特征提取功能
- 整合服装检测和姿态估计
- 提供ROS2服务接口
- 支持Excel文件输出

## 技术支持

如有问题，请检查：
1. ROS2环境是否正确设置
2. 所有依赖项是否已安装
3. 模型文件是否存在
4. 节点是否正常运行

更多信息请参考项目文档或联系开发团队。 