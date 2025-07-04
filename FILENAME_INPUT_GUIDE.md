# 文件名输入功能使用指南

## 功能概述

在微信小程序的特征识别页面，现在用户可以为上传的照片或视频自定义名称，增强了文件管理的灵活性。

## 功能特点

### 🎯 主要功能
- **自定义文件名**: 用户可以为上传的文件设置个性化名称
- **智能默认名**: 根据文件类型自动生成合理的默认文件名
- **类型识别**: 支持图片（JPG、PNG）和视频（MP4）文件
- **长度限制**: 文件名最大支持50个字符
- **友好界面**: 美观的弹窗式输入界面

### ⚡ 交互流程
1. 用户选择文件/拍照/录制视频
2. 自动显示文件名输入对话框
3. 系统根据文件类型生成默认名称
4. 用户可以修改文件名或使用默认名称
5. 点击"确认上传"开始文件传输
6. 点击"取消"可以重新选择文件

## 实现细节

### 📱 前端实现

#### 数据结构
```javascript
data: {
  // 文件名输入相关
  customFileName: '',        // 用户输入的文件名
  showFileNameInput: false,  // 是否显示输入对话框
  currentFilePath: '',       // 当前文件的路径
}
```

#### 主要函数
- `showFileNameInputDialog()`: 显示文件名输入对话框
- `onFileNameInput()`: 处理用户输入
- `confirmUpload()`: 确认上传并验证文件名
- `cancelUpload()`: 取消上传并清理状态

#### 默认文件名生成规则
- **图片文件**: `特征识别_时间戳.jpg`
- **视频文件**: `特征识别_时间戳.mp4`
- **其他文件**: 保持原始文件名

### 🎨 界面设计

#### 对话框样式
- 模态弹窗设计，背景半透明遮罩
- 圆角卡片样式，符合微信小程序设计规范
- 清晰的标题和说明文字
- 响应式布局，适配不同屏幕尺寸

#### 输入框特性
- 自动聚焦，方便用户直接输入
- 实时输入验证
- 50字符长度限制
- 友好的占位符提示

## 使用示例

### 场景1：上传照片
```
用户点击"选择文件" → 选择照片
↓
弹出对话框，默认名称："特征识别_1640995200000.jpg"
↓
用户修改为："张三的特征照片.jpg"
↓
点击"确认上传" → 开始上传
```

### 场景2：拍照
```
用户点击"拍照" → 拍摄照片
↓
弹出对话框，默认名称："特征识别_1640995300000.jpg"
↓
用户保持默认名称
↓
点击"确认上传" → 开始上传
```

### 场景3：录制视频
```
用户点击"录制" → 录制视频
↓
弹出对话框，默认名称："特征识别_1640995400000.mp4"
↓
用户修改为："动态特征视频.mp4"
↓
点击"确认上传" → 开始上传
```

## 技术要点

### 🔒 输入验证
- 非空验证：确保用户输入有效文件名
- 长度限制：最大50字符
- 特殊字符处理：过滤或转换不安全字符

### 📂 文件管理
- 文件名与原始文件路径分离管理
- 支持中文文件名
- 自动文件扩展名处理

### 🔄 状态管理
- 清晰的状态切换逻辑
- 错误状态处理和恢复
- 用户操作的即时反馈

## 优势

### 👤 用户体验
- **个性化**: 用户可以设置有意义的文件名
- **便于管理**: 清晰的文件命名有助于后续查找
- **操作简单**: 直观的界面，无学习成本

### 🛡️ 系统可靠性
- **输入验证**: 防止无效或危险的文件名
- **错误处理**: 完善的异常情况处理
- **状态一致**: 确保界面状态与数据状态同步

### 🔧 维护性
- **模块化设计**: 功能独立，便于维护和扩展
- **清晰接口**: 函数职责明确，便于调试
- **文档完善**: 详细的代码注释和使用说明

## 后续扩展

### 可能的改进方向
1. **文件名模板**: 提供多种文件名模板供用户选择
2. **历史记录**: 记住用户常用的文件名模式
3. **批量操作**: 支持多文件上传时的批量命名
4. **智能建议**: 基于内容分析提供文件名建议
5. **文件夹分类**: 支持按类别组织文件

### 集成考虑
- 与现有的文件转发功能无缝集成
- 保持与服务端接口的兼容性
- 考虑后续特征识别流程的需求

## 总结

文件名输入功能增强了用户对文件管理的控制力，提供了更加个性化和友好的用户体验。通过合理的默认值和灵活的自定义选项，既照顾了快速操作的需求，也满足了精细管理的要求。

该功能的实现充分考虑了微信小程序的设计规范和用户习惯，确保了功能的易用性和可靠性。 