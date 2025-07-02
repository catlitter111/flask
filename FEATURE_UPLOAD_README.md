# Feature页面文件上传功能说明

## 🚨 最新修复（2025-01-27）

### 🔧 问题修复
1. **端口配置统一**：服务端HTTP端口已调整为1235，与小程序端保持一致
2. **文件信息显示修复**：
   - 增加了详细的调试日志，便于排查问题
   - 改进了文件名提取逻辑，支持不同路径分隔符
   - 上传失败时保留文件信息显示，让用户知道哪个文件上传失败
   - 增加了文件大小的实时显示（MB格式）

### 🐛 调试改进
- 添加了完整的文件选择、拍照、录制的调试日志
- 上传过程的详细状态跟踪
- 错误处理时保留用户选择的文件信息

### 📋 显示改进
- 文件信息区域现在会显示：
  - 支持格式：JPG, PNG, MP4
  - 文件大小限制：≤ 10MB
  - 当前文件名（如果已选择）
  - 当前文件大小（如果已选择，显示为MB）

---

## 功能概述

已为feature页面实现了完整的照片上传功能，包括：

1. **HTTP文件上传服务器**：在服务端添加了专门的文件上传接口
2. **真实文件上传**：小程序端使用wx.uploadFile真实上传文件到服务器
3. **特征提取请求**：上传完成后自动发送特征提取请求
4. **结果处理**：通过WebSocket接收特征提取结果并显示

## 实现的功能

### 服务端修改
- ✅ 添加了HTTP服务器(aiohttp)，监听1235端口
- ✅ 实现文件上传接口：`/api/upload/{client_id}`
- ✅ 文件大小限制：10MB
- ✅ 支持的格式：JPG, PNG, MP4
- ✅ CORS跨域支持
- ✅ 文件存储到uploads目录
- ✅ 上传完成后通知WebSocket客户端
- ✅ 特征提取请求转发到机器人

### 小程序端修改
- ✅ 实现真实文件上传功能
- ✅ 上传进度显示
- ✅ 错误处理和用户反馈
- ✅ 上传完成后自动开始特征提取
- ✅ 通过WebSocket接收特征提取结果
- ✅ 结果数据格式化和显示

## 使用方法

### 1. 安装依赖
```bash
chmod +x install_http_dependencies.sh
./install_http_dependencies.sh
```

### 2. 启动服务端
```bash
cd 服务端
python3 server.py
```

服务端将启动：
- **WebSocket服务**：`ws://172.20.39.181:1234`
- **HTTP文件上传**：`http://172.20.39.181:1235`

### 3. 使用小程序
1. 打开feature页面
2. 点击"选择文件"、"拍照"或"录制"按钮
3. 选择或拍摄照片/视频
4. 自动开始上传，显示进度条
5. 上传完成后自动开始特征提取
6. 显示识别结果

## 技术架构

```
小程序客户端 --> HTTP上传 --> 服务端文件存储
     ↓                          ↓
WebSocket连接 <-- 特征提取结果 <-- 机器人处理
```

### 上传流程
1. **文件选择**：用户选择文件
2. **HTTP上传**：wx.uploadFile上传到服务端
3. **文件存储**：服务端保存文件并返回file_id
4. **WebSocket通知**：服务端通过WebSocket通知上传成功
5. **特征提取请求**：小程序发送特征提取WebSocket消息
6. **转发处理**：服务端转发请求到机器人节点
7. **结果返回**：机器人处理完成后返回结果
8. **显示结果**：小程序接收并显示特征识别结果

## 文件结构

```
服务端/
├── server.py              # 主服务器文件（已修改）
└── uploads/               # 上传文件存储目录（自动创建）

self_following_miniproam/
└── pages/feature/
    ├── feature.js         # 页面逻辑（已修改）
    ├── feature.wxml       # 页面结构
    └── feature.wxss       # 页面样式
```

## API接口

### 文件上传接口
- **URL**: `POST /api/upload/{client_id}`
- **Content-Type**: `multipart/form-data`
- **参数**: 
  - `file`: 上传的文件
  - `client_id`: 客户端ID
- **返回**: 
  ```json
  {
    "success": true,
    "file_id": "client123_1640995200000_image.jpg",
    "file_name": "image.jpg",
    "file_size": 1024000,
    "file_type": "image/jpeg",
    "upload_time": 1640995200000
  }
  ```

### WebSocket消息

#### 特征提取请求
```json
{
  "type": "feature_extraction_request",
  "robot_id": "companion_robot_001",
  "file_id": "client123_1640995200000_image.jpg",
  "extract_clothing_colors": true,
  "extract_body_proportions": true,
  "timestamp": 1640995200000
}
```

#### 特征提取结果
```json
{
  "type": "feature_extraction_result",
  "status": "success",
  "confidence": 0.89,
  "features": {
    "clothing_colors": {
      "top": {
        "name": "绿色",
        "hex_color": "#4CAF50",
        "confidence": 0.85
      },
      "bottom": {
        "name": "蓝色", 
        "hex_color": "#2196F3",
        "confidence": 0.92
      }
    },
    "body_proportions": {
      "height": 175.2,
      "shoulder_width": 42.3,
      "chest_circumference": 95.8
    }
  }
}
```

## 注意事项

1. **网络配置**：确保小程序能访问服务器的1235端口
2. **文件大小**：上传文件不能超过10MB
3. **存储空间**：uploads目录会存储所有上传的文件，注意磁盘空间
4. **机器人连接**：需要机器人节点连接到WebSocket才能进行特征提取
5. **权限设置**：小程序需要相机、相册访问权限

## 故障排除

### 上传失败
- 检查网络连接
- 确认服务端HTTP服务是否启动
- 检查文件大小是否超限

### 特征提取无响应
- 检查WebSocket连接状态
- 确认机器人节点是否连接
- 查看服务端日志

### 依赖安装问题
```bash
# 如果pip3不存在，使用pip
pip install aiohttp

# 或者使用conda
conda install aiohttp
```

## 扩展功能

后续可以添加的功能：
- [ ] 文件压缩和优化
- [ ] 批量上传
- [ ] 上传历史管理
- [ ] 文件预览和编辑
- [ ] 多种特征提取算法选择 