# 微信小程序 WebRTC 配置说明

## 问题描述

在微信小程序中使用 `live-player` 组件时，可能遇到以下错误：
```
operateLivePlayer:fail no permission, appId=wxXXXXXXXXXXXXXXXX
```

## 解决方案

### 1. 小程序类目要求

`live-player` 组件需要特定的小程序类目才能使用。请确保您的小程序类目包含以下之一：

- **社交** > 直播
- **教育** > 在线教育
- **医疗** > 互联网医院，公立医院
- **政务民生** > 所有二级类目
- **金融** > 基金、信托、保险、银行、证券/期货、非金融机构自营小额贷款、征信业务、消费金融
- **汽车** > 汽车预售服务
- **工具** > 视频播放器

### 2. app.json 配置

在 `app.json` 中添加必要的配置：

```json
{
  "plugins": {
    "live-player-plugin": {
      "version": "1.3.0",
      "provider": "wx2b03c6e691cd7370"
    }
  },
  "requiredPrivateInfos": [
    "getLocation"
  ],
  "permission": {
    "scope.userLocation": {
      "desc": "需要获取您的地理位置以记录机器人伴侣的陪伴轨迹"
    }
  }
}
```

### 3. 页面配置 (control.json)

在使用 `live-player` 的页面配置文件中添加：

```json
{
  "usingComponents": {},
  "disableScroll": true
}
```

### 4. 降级机制

本项目已实现智能降级机制：

1. **权限检查**: 启动时自动检查 `live-player` 权限
2. **重试机制**: WebRTC 失败时最多重试 3 次
3. **自动降级**: 权限不足或重试失败后自动切换到 WebSocket 模式
4. **避免循环**: 降级完成后不会重复尝试 WebRTC

### 5. 日志说明

正常的日志流程：
```
📡 正在初始化WebRTC连接...
✅ live-player权限检查通过
📡 WebRTC初始化完成
📡 WebRTC live-player开始播放
✅ WebRTC连接成功
```

权限不足时的日志流程：
```
📡 正在初始化WebRTC连接...
⚠️ 没有live-player权限，直接使用WebSocket模式
📡 降级到WebSocket视频传输模式
📡 WebSocket降级完成，等待接收图像数据...
```

## 开发建议

### 1. 类目申请

如果您需要使用 WebRTC 功能：
1. 登录微信公众平台
2. 进入"设置" > "基本设置" > "服务类目"
3. 添加符合要求的类目
4. 等待审核通过

### 2. 测试环境

在开发环境中，可以：
1. 使用微信开发者工具的"真机调试"功能
2. 在项目设置中开启"不校验合法域名"
3. 测试 WebSocket 降级功能

### 3. 生产环境

在生产环境中：
1. 确保服务器域名已添加到"合法域名"列表
2. 使用 HTTPS 协议
3. 配置正确的 WebRTC 流地址

## 常见问题

### Q: 为什么总是降级到 WebSocket？
A: 可能的原因：
- 小程序类目不符合要求
- 没有配置 live-player 插件
- 网络环境不支持 WebRTC

### Q: WebSocket 模式性能如何？
A: WebSocket 模式：
- 延迟稍高（50-200ms）
- 支持动态质量调整
- 兼容性更好
- 适合大多数使用场景

### Q: 如何强制使用 WebRTC？
A: 不建议强制使用，但可以在 control.js 中设置：
```javascript
data: {
  webrtcEnabled: true,
  fallbackToWebSocket: false
}
```

## 技术支持

如遇到其他问题，请检查：
1. 微信开发者工具控制台日志
2. 网络连接状态
3. 服务器运行状态
4. 小程序版本是否最新 