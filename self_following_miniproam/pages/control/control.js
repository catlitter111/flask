// pages/control/control.js - 机器人伴侣控制页面
Page({
    data: {
      operationMode: 'auto', // 'manual' 或 'auto' - 修改默认模式为自动
      connected: false,        // 小程序与服务器的连接状态
      connecting: false,       // 正在连接中
      robotConnected: false,   // 机器人与服务器的连接状态
      batteryLevel: 0,
      signalStrength: '未连接',
      autoProgress: 0,
      autoStatus: '未启动',
      robotId: 'companion_robot_001',
      clientId: '',
      videoBase64: '',         // 视频帧base64数据
      reconnectCount: 0,
      maxReconnectAttempts: 5,
      reconnectInterval: 3000, // 3秒
      lastCommandTime: 0,
      commandCooldown: 500,    // 命令冷却时间，防止过于频繁发送指令
      
      // 视频质量相关数据
      currentQuality: 'medium', // 当前视频质量预设
      qualityPresets: ['ultra_high', 'high', 'medium', 'low', 'very_low', 'minimum', 'ultra_low'],
      qualityLabels: {
        'ultra_high': '超高清 (800×600)',
        'high': '高清 (640×480)',
        'medium': '标准 (480×360)',
        'low': '流畅 (320×240)', 
        'very_low': '省流 (240×180)',
        'minimum': '最小 (160×120)',
        'ultra_low': '极简 (120×90)'
      },
      bufferHealth: 100,       // 缓冲健康度(0-100)
      frameLatency: 0,         // 帧延迟(ms)
      frameJitter: 0,          // 帧抖动(ms)
      lastFrameTimestamp: 0,   // 最后一帧的时间戳
      lastFrameReceived: 0,    // 最后收到帧的本地时间戳
      frameTimestamps: [],     // 最近10帧的时间戳，用于计算抖动
      videoResolution: '480×360', // 当前视频分辨率
      videoFps: 10,            // 当前帧率
      showQualityPanel: false, // 是否显示质量设置面板
      autoQuality: true,       // 是否自动调整质量
      
      // 视频分析指标
      receivedFrames: 0,       // 已接收的帧数
      framesPerSecond: 0,      // 实际显示的每秒帧数
      fpsCounterTime: 0,       // FPS计数器时间点
      fpsCounter: 0,           // FPS计数器
      
      // 视频过期状态
      videoExpired: false,     // 视频是否已过期（长时间未收到新帧）
      videoExpireTimeout: 5000, // 视频过期时间（毫秒）
      statusUpdateInterval: 5000, // 状态更新间隔（毫秒）
      lastStatusUpdateTime: 0, // 上次状态更新时间
      
      // 连接状态显示
      connectionStatusText: '未连接',
      connectionChecking: false, // 是否正在检查连接状态
      connectionCheckingText: '检查连接中...',
      
      // 新增状态变量
      frameSequence: 0,            // 最后收到的帧序号
      expectedSequence: 0,         // 期望的下一帧序号
      droppedFrames: 0,            // 丢帧计数
      networkDelay: 0,             // 网络延迟
      serverToClientDelay: 0,      // 服务器到客户端延迟
      reconnectingRobot: false,    // 机器人是否正在重连
      lastRobotStatusTime: 0,      // 最后收到机器人状态的时间
      robotReconnectAttempts: 0,   // 机器人重连尝试次数
      lastConnectionAttempt: 0,    // 上次连接尝试时间
      
      // 电机速度控制
      motorSpeed: 50,              // 电机速度，默认值50%
      
      // 控制类型：'motor' 电机控制，'companion' 伴侣交互控制
      controlType: 'motor',        // 默认为电机控制
      
      // 新增：图像更新控制
      lastImageUpdateTime: 0,
      minImageUpdateInterval: 100,  // 最小更新间隔(毫秒)
      imageUpdatePending: false,
      pendingImageData: null,
      
      // 系统信息
      statusBarHeight: 20,
      navBarHeight: 44,
      availableHeight: 0,
      windowHeight: 0,
      
      // 新增：拖拽调整大小相关
      videoHeight: 55, // 视频容器高度百分比（vh）
      minVideoHeight: 25, // 最小视频高度（vh）
      maxVideoHeight: 75, // 最大视频高度（vh）
      isDragging: false, // 是否正在拖拽
      dragStartY: 0, // 拖拽开始时的Y坐标
      dragStartHeight: 0, // 拖拽开始时的高度
      showDragHint: true, // 是否显示拖拽提示
      
      // 新增：Tab切换相关
      currentTab: 'control', // 当前tab：'control' 或 'data'
      
      // 新增：跟踪数据展示
      trackingData: {
        tracks: [
          // 模拟数据，后续会从服务器接收
          {
            id: 1,
            confidence: 0.85,
            status: 'tracking',
            clothingColors: {
              top: { name: '蓝色', color: '#2196F3' },
              bottom: { name: '黑色', color: '#424242' }
            },
            position: { x: 320, y: 240 },
            lastUpdateTime: Date.now()
          },
          {
            id: 2,
            confidence: 0.72,
            status: 'lost',
            clothingColors: {
              top: { name: '红色', color: '#F44336' },
              bottom: { name: '白色', color: '#FFFFFF' }
            },
            position: { x: 180, y: 160 },
            lastUpdateTime: Date.now() - 5000
          }
        ],
        totalTracks: 2,
        activeTracks: 1,
        lostTracks: 1,
        targetTrackId: 1, // 当前跟踪目标ID
        lastUpdateTime: Date.now()
      },
      
      // 新增：真实特征数据
      realFeatureData: {
        bodyRatios: [],           // 身体比例数组
        shirtColor: [0, 0, 0],    // 上衣颜色RGB
        pantsColor: [0, 0, 0],    // 下装颜色RGB
        personCount: 0,           // 人数统计
        resultImagePath: '',      // 结果图片路径
        lastUpdate: 0             // 最后更新时间
      }
    },
  
    onLoad: function(options) {
      // 生成唯一的客户端ID
      const app = getApp();
      
      this.setData({
        clientId: app.globalData.clientId || `user_for_${this.data.robotId}_${Date.now()}`
      });
      
      // 注册到全局应用以接收消息
      app.globalData.controlPage = this;
      
      // 获取系统信息，适配不同屏幕 - 使用新的API
      const windowInfo = wx.getWindowInfo();
      const systemSetting = wx.getSystemSetting();
      const statusBarHeight = windowInfo.statusBarHeight || 20;
      const navBarHeight = 44; // 导航栏标准高度
      
      // 计算可用高度
      const windowHeight = windowInfo.windowHeight;
      const availableHeight = windowHeight - statusBarHeight - navBarHeight;
      
      this.setData({
        statusBarHeight: statusBarHeight,
        navBarHeight: navBarHeight,
        availableHeight: availableHeight,
        windowHeight: windowHeight
      });
      
      console.log('系统信息:', {
        windowHeight: windowHeight,
        statusBarHeight: statusBarHeight,
        navBarHeight: navBarHeight,
        availableHeight: availableHeight
      });
      
      // 启动帧率计算定时器
      this.startFpsCalculator();
      
      // 启动连接状态检查定时器
      this.startConnectionChecker();
      
      // 更新连接状态
      this.checkGlobalConnectionState();
      
          // 新增：3秒后隐藏拖拽提示
    setTimeout(() => {
      this.setData({
        showDragHint: false
      });
    }, 3000);
    
    // 启动跟踪数据更新定时器
    this.startTrackingDataUpdater();
    
    // 启动定期内存清理
    this.startMemoryCleanup();
    
    // 启用性能监控（仅在开发模式）
    if (wx.getAccountInfoSync().miniProgram.envVersion === 'develop') {
      this.addPerformanceMonitor();
    }
    },
  
    onShow: function() {
      // 页面显示时重新注册
      const app = getApp();
      app.globalData.controlPage = this;
      
      // 更新连接状态
      this.checkGlobalConnectionState();
      
      // 如果之前收到过视频帧但现在已经过期，请求重新开始视频流
      // 视频流将自动恢复
      
      // 新增：恢复用户保存的视频高度偏好
      const savedHeight = wx.getStorageSync('userVideoHeight');
      if (savedHeight) {
        this.setData({
          videoHeight: savedHeight
        });
      }
    },
    
    onHide: function() {
      // 页面隐藏时暂时不断开，保持后台连接
    },
    
    onUnload: function() {
      // 页面卸载时，取消注册并清除所有定时器
      const app = getApp();
      app.globalData.controlPage = null;
      
      // 清除所有定时器
      if (this.fpsTimer) {
        clearInterval(this.fpsTimer);
      }
      
      if (this.connectionChecker) {
        clearInterval(this.connectionChecker);
      }
      
      if (this.videoExpireChecker) {
        clearInterval(this.videoExpireChecker);
      }
      
      if (this.trackingDataTimer) {
        clearInterval(this.trackingDataTimer);
      }
    },
    
    // 页面卸载时清理资源
    onUnload: function() {
      // 清理所有定时器
      if (this.fpsTimer) {
        clearInterval(this.fpsTimer);
        this.fpsTimer = null;
      }
      
      if (this.trackingDataTimer) {
        clearInterval(this.trackingDataTimer);
        this.trackingDataTimer = null;
      }
      
      if (this.connectionChecker) {
        clearInterval(this.connectionChecker);
        this.connectionChecker = null;
      }
      
      if (this.videoExpireChecker) {
        clearInterval(this.videoExpireChecker);
        this.videoExpireChecker = null;
      }
      
      if (this.memoryCleanupTimer) {
        clearInterval(this.memoryCleanupTimer);
        this.memoryCleanupTimer = null;
      }
      
      // 清理数据数组
      this.setData({
        frameTimestamps: [],
        'trackingData.tracks': []
      });
      
      // 关闭WebSocket连接
      if (this.data.websocket) {
        this.data.websocket.close();
      }
      
      console.log('🧹 页面卸载，资源已清理');
    },
    
    // 页面隐藏时暂停不必要的定时器
    onHide: function() {
      // 暂停跟踪数据更新（如果在数据页面）
      if (this.trackingDataTimer) {
        clearInterval(this.trackingDataTimer);
        this.trackingDataTimer = null;
      }
      
      // 暂停FPS计算
      if (this.fpsTimer) {
        clearInterval(this.fpsTimer);
        this.fpsTimer = null;
      }
      
      console.log('⏸️ 页面隐藏，定时器已暂停');
    },
    
    // 页面显示时恢复定时器
    onShow: function() {
      // 恢复FPS计算
      if (this.data.websocket && this.data.websocket.readyState === WebSocket.OPEN) {
        this.startFpsCalculator();
      }
      
      // 如果在数据页面，恢复数据更新
      if (this.data.currentTab === 'data') {
        this.startTrackingDataUpdater();
      }
      
      console.log('▶️ 页面显示，定时器已恢复');
    },
    
    // 添加性能监控方法
    addPerformanceMonitor: function() {
      // 监控长时间运行的setTimeout
      const originalSetTimeout = setTimeout;
      setTimeout = function(callback, delay) {
        return originalSetTimeout(function() {
          const startTime = Date.now();
          const result = callback.apply(this, arguments);
          const executionTime = Date.now() - startTime;
          
          if (executionTime > 50) {
            console.warn(`⚠️ setTimeout执行时间过长: ${executionTime}ms`);
          }
          
          return result;
        }, delay);
      };
      
      console.log('🔍 性能监控已启用');
    },
    
    // 启动定期内存清理
    startMemoryCleanup: function() {
      // 清除之前的定时器
      if (this.memoryCleanupTimer) {
        clearInterval(this.memoryCleanupTimer);
      }
      
      // 动态调整内存清理频率（根据性能情况）
      this.memoryCleanupInterval = 60000; // 初始60秒
      this.memoryCleanupTimer = setInterval(() => {
        const startTime = Date.now();
        this.performMemoryCleanup();
        const executionTime = Date.now() - startTime;
        
        // 根据执行时间动态调整间隔
        if (executionTime > 30) {
          this.memoryCleanupInterval = Math.min(120000, this.memoryCleanupInterval * 1.2);
        } else if (executionTime < 10) {
          this.memoryCleanupInterval = Math.max(30000, this.memoryCleanupInterval * 0.9);
        }
        
        // 重新设置定时器
        clearInterval(this.memoryCleanupTimer);
        this.memoryCleanupTimer = setInterval(() => {
          this.performMemoryCleanup();
        }, this.memoryCleanupInterval);
      }, this.memoryCleanupInterval);
    },
    
    // 执行内存清理（异步分片处理）
    performMemoryCleanup: function() {
      const now = Date.now();
      const updates = {};
      let cleanupCount = 0;
      
      // 分片处理，避免阻塞主线程
      const processCleanup = () => {
        // 快速检查并清理帧时间戳
        const frameTimestamps = this.data.frameTimestamps;
        if (frameTimestamps.length > 5) {
          updates.frameTimestamps = frameTimestamps.slice(-3);
          cleanupCount++;
        }
        
        // 快速检查跟踪数据数量
        const tracks = this.data.trackingData.tracks;
        if (tracks.length > 10) {
          updates['trackingData.tracks'] = tracks.slice(0, 8);
          updates['trackingData.totalTracks'] = 8;
          cleanupCount++;
        }
        
        // 重置大数值计数器
        if (this.data.receivedFrames > 10000) {
          updates.receivedFrames = 0;
          updates.droppedFrames = 0;
          updates.fpsCounter = 0;
          cleanupCount++;
        }
        
        // 清理临时数据
        if (this.data.pendingImageData) {
          this.data.pendingImageData = null;
          cleanupCount++;
        }
        
        // 批量更新，减少渲染次数
        if (Object.keys(updates).length > 0) {
          this.setData(updates);
        }
        
        if (cleanupCount > 0) {
          console.log(`🧹 内存清理完成，清理了 ${cleanupCount} 项数据`);
        }
      };
      
      // 使用setTimeout异步执行，避免阻塞
      setTimeout(processCleanup, 0);
    },
    
    // Tab切换方法
    switchTab: function(e) {
      const tab = e.currentTarget.dataset.tab;
      this.setData({
        currentTab: tab
      });
      
      // 根据Tab状态管理定时器，节省内存
      if (tab === 'data') {
        // 切换到数据页面时，启动数据更新定时器
        this.startTrackingDataUpdater();
      } else if (tab === 'control') {
        // 切换到控制页面时，暂停数据更新定时器
        if (this.trackingDataTimer) {
          clearInterval(this.trackingDataTimer);
          this.trackingDataTimer = null;
        }
      }
    },
    
    // 启动跟踪数据更新定时器
    startTrackingDataUpdater: function() {
      // 清除之前的定时器，防止重复创建
      if (this.trackingDataTimer) {
        clearInterval(this.trackingDataTimer);
      }
      
      // 智能调整跟踪数据更新频率
      this.trackingUpdateInterval = 3000; // 初始3秒
      
      this.trackingDataTimer = setInterval(() => {
        if (this.data.currentTab === 'data') {
          const startTime = Date.now();
          this.updateTrackingData();
          const executionTime = Date.now() - startTime;
          
          // 根据执行时间调整更新频率
          if (executionTime > 50) {
            this.trackingUpdateInterval = Math.min(5000, this.trackingUpdateInterval + 500);
            console.log(`⚡ 跟踪数据更新耗时${executionTime}ms，调整频率至${this.trackingUpdateInterval}ms`);
          } else if (executionTime < 20) {
            this.trackingUpdateInterval = Math.max(2000, this.trackingUpdateInterval - 200);
          }
          
          // 重新设置定时器
          clearInterval(this.trackingDataTimer);
          this.trackingDataTimer = setInterval(() => {
            if (this.data.currentTab === 'data') {
              this.updateTrackingData();
            }
          }, this.trackingUpdateInterval);
        }
      }, this.trackingUpdateInterval);
    },
    
    // 更新跟踪数据（优化性能）
    updateTrackingData: function() {
      const now = Date.now();
      const tracks = this.data.trackingData.tracks;
      
      // 快速限制轨迹数量
      if (tracks.length > 8) {
        tracks.length = 6; // 直接截断，避免splice操作
      }
      
      // 分片处理，避免长时间循环
      const processChunk = (startIndex) => {
        const chunkSize = 3; // 每次处理3个轨迹
        const endIndex = Math.min(startIndex + chunkSize, tracks.length);
        let hasChanges = false;
        let activeTracks = 0;
        let lostTracks = 0;
        
        for (let i = startIndex; i < endIndex; i++) {
          const track = tracks[i];
          
          // 简化状态检查，减少计算
          const shouldUpdate = Math.random() > 0.7; // 只有30%的概率更新
          if (!shouldUpdate) {
            // 只统计状态
            if (track.status === 'tracking') activeTracks++;
            else if (track.status === 'lost') lostTracks++;
            continue;
          }
          
          // 简化置信度计算
          const confidenceChange = (Math.random() - 0.5) * 0.03;
          const newConfidence = Math.max(0, Math.min(1, track.confidence + confidenceChange));
          
          // 直接修改对象属性，避免创建新对象
          track.confidence = parseFloat(newConfidence.toFixed(2));
          
          // 简化位置更新
          track.position.x += (Math.random() - 0.5) * 5;
          track.position.y += (Math.random() - 0.5) * 5;
          
          // 随机更新时间
          if (Math.random() > 0.8) {
            track.lastUpdateTime = now;
          }
          
          // 统计状态
          if (track.status === 'tracking') activeTracks++;
          else if (track.status === 'lost') lostTracks++;
          
          hasChanges = true;
        }
        
        // 继续处理下一批
        if (endIndex < tracks.length) {
          setTimeout(() => processChunk(endIndex), 0);
        } else if (hasChanges) {
          // 所有轨迹处理完成，更新界面
          this.setData({
            'trackingData.tracks': tracks,
            'trackingData.activeTracks': activeTracks,
            'trackingData.lostTracks': lostTracks,
            'trackingData.totalTracks': tracks.length,
            'trackingData.lastUpdateTime': now
          });
        }
      };
      
      // 开始分片处理
      if (tracks.length > 0) {
        processChunk(0);
      }
    },
    
    // 设置目标跟踪ID
    setTargetTrack: function(e) {
      const trackId = e.currentTarget.dataset.trackId;
      this.setData({
        'trackingData.targetTrackId': parseInt(trackId)
      });
      
      wx.showToast({
        title: `已设置目标轨迹 ID: ${trackId}`,
        icon: 'success',
        duration: 1500
      });
    },
    

    
    // 新增：处理拖拽开始
    handleDragStart: function(e) {
      this.setData({
        isDragging: true,
        dragStartY: e.touches[0].clientY,
        dragStartHeight: this.data.videoHeight,
        showDragHint: false
      });
      
      // 震动反馈
      wx.vibrateShort({
        type: 'light'
      });
    },
    
    // 新增：处理拖拽移动
    handleDragMove: function(e) {
      if (!this.data.isDragging) return;
      
      const currentY = e.touches[0].clientY;
      const deltaY = currentY - this.data.dragStartY;
      
      // 将像素差值转换为vh单位（windowHeight对应100vh）
      const deltaVh = (deltaY / this.data.windowHeight) * 100;
      
      // 计算新的高度
      let newHeight = this.data.dragStartHeight + deltaVh;
      
      // 限制高度范围
      newHeight = Math.max(this.data.minVideoHeight, Math.min(this.data.maxVideoHeight, newHeight));
      
      this.setData({
        videoHeight: newHeight
      });
    },
    
    // 新增：处理拖拽结束
    handleDragEnd: function(e) {
      if (!this.data.isDragging) return;
      
      this.setData({
        isDragging: false
      });
      
      // 震动反馈
      wx.vibrateShort({
        type: 'light'
      });
      
      // 保存用户偏好（可选）
      wx.setStorageSync('userVideoHeight', this.data.videoHeight);
    },
    
    // 检查全局连接状态
    checkGlobalConnectionState: function() {
      const app = getApp();
      
      this.setData({
        connected: app.globalData.connected,
        connecting: app.globalData.connecting
      });
      
      // 如果未连接，通过全局方法重连
      if (!app.globalData.connected && !app.globalData.connecting) {
        app.connectWebSocket();
      }
    },
  
    // 启动连接状态检查定时器
    startConnectionChecker: function() {
      const that = this;
      
      // 每2秒检查一次连接状态
      this.connectionChecker = setInterval(function() {
        // 检查视频帧是否已过期
        const now = Date.now();
        const lastFrameAge = now - that.data.lastFrameReceived;
        const lastStatusAge = now - that.data.lastRobotStatusTime;
        
        // 更新全局连接状态
        const app = getApp();
        that.setData({
          connected: app.globalData.connected,
          connecting: app.globalData.connecting
        });
        
        // 如果超过指定时间未收到新帧，且机器人连接状态为已连接，则认为视频已过期
        // 但是给机器人重连后一定的缓冲时间来恢复视频流
        const robotConnectAge = now - that.data.lastRobotStatusTime;
        const allowVideoRecoveryTime = 10000; // 给机器人10秒时间来恢复视频流
        
        if (lastFrameAge > that.data.videoExpireTimeout && 
            that.data.robotConnected && 
            robotConnectAge > allowVideoRecoveryTime) {
          // 如果之前视频未过期，则发出警告
          if (!that.data.videoExpired) {
            console.warn('🎥 视频帧已过期，最后一帧接收时间:', new Date(that.data.lastFrameReceived));
            console.warn('🎥 机器人连接时长:', robotConnectAge, 'ms，视频帧年龄:', lastFrameAge, 'ms');
            that.setData({
              videoExpired: true
            });
          }
        }
        
        // 如果机器人状态长时间未更新，认为机器人可能离线
        if (lastStatusAge > 15000 && that.data.robotConnected) {  // 15秒
          that.setData({
            robotConnected: false,
            signalStrength: '未响应',
            connectionStatusText: '等待伴侣响应'
          });
        }
        
        // 更新连接状态文本
        let statusText = '未连接';
        if (that.data.connecting) {
          statusText = '连接中...';
        } else if (that.data.connected) {
          if (that.data.robotConnected) {
            if (that.data.videoExpired) {
              // 检查是否刚重连，给一些恢复时间
              const robotConnectAge = now - that.data.lastRobotStatusTime;
              if (robotConnectAge < 15000) { // 15秒内
                statusText = '视频恢复中...';
              } else {
                statusText = '视频超时';
              }
            } else {
              statusText = '伴侣在线';
            }
          } else if (that.data.reconnectingRobot) {
            statusText = '伴侣重连中...';
          } else {
            statusText = '伴侣离线';
          }
        } else {
          statusText = '未连接';
        }
        
        that.setData({
          connectionStatusText: statusText
        });
        
        // 如果已连接服务器，则发送心跳检测机器人状态
        if (that.data.connected && !that.data.connecting) {
          that.sendSocketMessage({
            type: 'ping',
            timestamp: now
          });
          
          // 定期请求机器人状态
          if (now - that.data.lastRobotStatusTime > 10000) {  // 10秒
            that.requestRobotStatus();
          }
        }
      }, 2000);  // 缩短为2秒，提高响应速度
      
      // 启动视频过期检查定时器
      this.videoExpireChecker = setInterval(function() {
        // 当视频过期或机器人离线时，清除视频画面
        if ((that.data.videoExpired || !that.data.robotConnected) && that.data.videoBase64) {
          that.setData({
            videoBase64: ''  // 清除视频画面
          });
        }
      }, 1000);
    },
  
    // 启动FPS计算器
    startFpsCalculator: function() {
      const that = this;
      
      // 清除之前的定时器
      if (this.fpsTimer) {
        clearInterval(this.fpsTimer);
      }
      
      // 优化FPS计算，减少处理时间
      this.fpsTimer = setInterval(function() {
        const fps = Math.round(that.data.fpsCounter / 3); // 3秒周期
        
        // 批量更新，减少setData调用
        const updates = { fpsCounter: 0 };
        if (fps !== that.data.framesPerSecond) {
          updates.framesPerSecond = fps;
        }
        
        // 异步更新，避免阻塞
        setTimeout(() => {
          that.setData(updates);
        }, 0);
      }, 3000); // 降低到3秒更新一次
    },
    
    // 处理视频帧
    handleVideoFrame: function(data) {
      const now = Date.now();
      
      // 更新最后收到帧的时间
      this.setData({
        lastFrameReceived: now,
        videoExpired: false
      });
      
      // 控制更新频率，避免闪烁
      const timeSinceLastUpdate = now - this.data.lastImageUpdateTime;
      
      if (timeSinceLastUpdate >= this.data.minImageUpdateInterval) {
        // 立即更新
        this.updateVideoFrame(data);
      } else {
        // 缓存最新帧，稍后更新
        this.data.pendingImageData = data;
        
        if (!this.data.imageUpdatePending) {
          this.data.imageUpdatePending = true;
          
          // 延迟更新（限制执行时间）
          const delayTime = Math.min(50, this.data.minImageUpdateInterval - timeSinceLastUpdate);
          setTimeout(() => {
            const startTime = Date.now();
            if (this.data.pendingImageData && Date.now() - startTime < 30) {
              this.updateVideoFrame(this.data.pendingImageData);
              this.data.pendingImageData = null;
            }
            this.data.imageUpdatePending = false;
          }, delayTime);
        }
      }
    },
    
    // 实际更新视频帧的方法（优化性能）
    updateVideoFrame: function(data) {
      const now = Date.now();
      
      // 批量更新数据，减少渲染次数
      const updateData = {
        videoBase64: `data:image/jpeg;base64,${data.frame_data || data.data}`,
        receivedFrames: this.data.receivedFrames + 1,
        lastImageUpdateTime: now,
        fpsCounter: this.data.fpsCounter + 1
      };
      
      // 服务器延迟计算（简化）
      if (data.server_timestamp) {
        updateData.serverToClientDelay = now - data.server_timestamp;
      }
      
      // 序号检查（简化）
      if (data.sequence) {
        if (this.data.expectedSequence > 0 && data.sequence > this.data.expectedSequence) {
          updateData.droppedFrames = this.data.droppedFrames + (data.sequence - this.data.expectedSequence);
        }
        updateData.frameSequence = data.sequence;
        updateData.expectedSequence = data.sequence + 1;
      }
      
      // 视频质量信息
      if (data.resolution) {
        updateData.videoResolution = data.resolution;
      }
      
      // 帧延迟计算（简化，减少复杂数学运算）
      if (data.timestamp) {
        updateData.frameLatency = now - data.timestamp;
        
        // 简化时间戳管理，只保留最近3帧
        const timestamps = this.data.frameTimestamps.slice(-2);
        timestamps.push(data.timestamp);
        updateData.frameTimestamps = timestamps;
        updateData.lastFrameTimestamp = data.timestamp;
        
        // 简化抖动计算 - 只计算最近两帧的间隔差异
        if (timestamps.length >= 2) {
          const interval1 = timestamps[1] - timestamps[0];
          const interval2 = timestamps.length > 2 ? timestamps[2] - timestamps[1] : interval1;
          updateData.frameJitter = Math.abs(interval2 - interval1);
        }
      }
      
      // 简化缓冲健康度计算
      const health = Math.min(100, Math.max(0, 100 - (updateData.frameLatency || 0) / 10));
      updateData.bufferHealth = Math.round(health);
      
      // 连接状态更新
      if (!this.data.robotConnected) {
        updateData.robotConnected = true;
        updateData.reconnectingRobot = false;
        updateData.signalStrength = '已连接';
        updateData.connectionStatusText = '伴侣在线';
        updateData.lastRobotStatusTime = now;
      }
      
      // 一次性更新所有数据
      this.setData(updateData);
      
      // 异步处理状态更新，避免阻塞
      if (now - this.data.lastStatusUpdateTime > this.data.statusUpdateInterval) {
        setTimeout(() => {
          this.sendStatusUpdate(health);
          this.setData({
            lastStatusUpdateTime: now
          });
        }, 0);
      }
    },
    
    // 处理机器人连接状态
    handleRobotConnectionStatus: function(data) {
      const connected = data.connected;
      const now = Date.now();
      
      this.setData({
        lastRobotStatusTime: now
      });
      
      if (connected) {
        // 机器人已连接
        this.setData({
          robotConnected: true,
          reconnectingRobot: false,
          robotReconnectAttempts: 0,
          signalStrength: '已连接',
          connectionStatusText: '伴侣在线',
          // 重置视频相关状态，给视频流恢复时间
          videoExpired: false,
          lastFrameReceived: now  // 重置最后帧接收时间，避免立即触发视频过期
        });
        
        console.log('🔗 机器人重新连接，等待视频流恢复...');
      } else {
        // 机器人断开连接
        this.setData({
          robotConnected: false,
          reconnectingRobot: true,
          signalStrength: '未连接',
          connectionStatusText: '伴侣离线'
        });
        
        // 清除视频
        if (this.data.videoBase64) {
          this.setData({
            videoBase64: ''
          });
        }
        
        // 尝试恢复连接(最多尝试3次)
        if (this.data.robotReconnectAttempts < 3) {
          this.setData({
            robotReconnectAttempts: this.data.robotReconnectAttempts + 1
          });
          
          // 每次等待时间翻倍(最多20秒)
          const waitTime = Math.min(20000, 2000 * Math.pow(2, this.data.robotReconnectAttempts)); 
          
          setTimeout(() => {
            if (this.data.connected && !this.data.robotConnected) {
              this.requestRobotStatus();
            }
          }, waitTime);
        }
      }
    },
    
    // 处理质量更新
    handleQualityUpdate: function(data) {
      console.log('收到质量更新:', data);
      
      this.setData({
        currentQuality: data.preset,
        videoResolution: data.resolution,
        videoFps: data.fps
      });
      
      // 显示提示
      wx.showToast({
        title: `视频质量已调整: ${this.data.qualityLabels[data.preset]}`,
        icon: 'none',
        duration: 2000
      });
    },
    
    // 关闭WebSocket连接（现使用全局连接，此方法弃用）
    closeConnection: function() {
      // 继续保留此方法以兼容原代码
      console.log("使用全局WebSocket连接，关闭操作已忽略");
    },
    
    // 发送WebSocket消息
    sendSocketMessage: function(msg) {
      const app = getApp();
      app.sendSocketMessage(msg);
    },
    
    // 更新连接状态
    updateConnectionStatus: function(isConnected, robotId) {
      if (robotId === this.data.robotId) {
        // 检查状态是否真的发生了变化
        const currentConnectionState = this.data.robotConnected;
        
        if (currentConnectionState !== isConnected) {
          // 状态改变时才输出日志
          console.log(`🔗 机器人状态变更: ${isConnected ? '已连接' : '已断开'}`);
        }
        
        this.setData({
          robotConnected: isConnected,
          reconnectingRobot: false
        });
        
        if (isConnected) {
          const now = Date.now();
          this.setData({
            signalStrength: '良好',
            connectionStatusText: '已连接',
            // 重置视频相关状态，给视频流恢复时间
            videoExpired: false,
            lastFrameReceived: now,
            lastRobotStatusTime: now
          });
          
          console.log('🔗 机器人重新连接 (来自updateConnectionStatus)，等待视频流恢复...');
          
          // 如果机器人重新连接，请求初始状态
          this.requestRobotStatus();
        } else {
          this.setData({
            signalStrength: '未连接',
            batteryLevel: 0,
            videoExpired: true,
            connectionStatusText: '未连接'
          });
        }
        
        // 更新全局连接状态检查
        this.checkGlobalConnectionState();
      }
    },

    // 显示质量调整请求已接收
    showQualityRequestReceived: function(preset) {
      wx.showToast({
        title: `质量调整为${preset}`,
        icon: 'none',
        duration: 1500
      });
    },



    // 更新视频质量
    updateVideoQuality: function(data) {
      this.setData({
        currentQuality: data.preset || this.data.currentQuality,
        videoResolution: data.resolution || this.data.videoResolution,
        videoFps: data.fps || this.data.videoFps
      });
      
      wx.showToast({
        title: `视频质量: ${data.preset}`,
        icon: 'none',
        duration: 1500
      });
    },

    // 处理伴侣状态更新
    handleCompanionStatusUpdate: function(data) {
      // 更新伴侣相关状态
      if (data.following_mode) {
        this.setData({
          autoStatus: data.following_mode
        });
      }
      
      if (data.emotion_state) {
        // 可以添加情绪状态显示
        console.log('情绪状态:', data.emotion_state);
      }
      
      if (data.interaction_mode !== undefined) {
        // 更新交互模式
        console.log('交互模式:', data.interaction_mode);
      }
    },

    // 处理交互事件
    handleInteractionEvent: function(data) {
      switch (data.event_type) {
        case 'gesture_detected':
          wx.showToast({
            title: '检测到手势',
            icon: 'none'
          });
          break;
        case 'voice_command':
          wx.showToast({
            title: '收到语音命令',
            icon: 'none'
          });
          break;
        case 'emotion_change':
          wx.showToast({
            title: `情绪变化: ${data.emotion}`,
            icon: 'none'
          });
          break;
      }
    },

    // 处理伴侣断开连接
    handleCompanionDisconnected: function(data) {
      console.log('💔 伴侣断开连接');
      
      this.setData({
        robotConnected: false,
        signalStrength: '未连接',
        batteryLevel: 0,
        videoExpired: true,
        connectionStatusText: '伴侣已断开'
      });
      
      // 更新全局连接状态
      this.checkGlobalConnectionState();
    },

    // 更新机器人状态
    updateRobotStatus: function(statusData) {
      if (!statusData) return;
      
      const now = Date.now();
      
      // 更新最后收到机器人状态的时间
      this.setData({
        lastRobotStatusTime: now
      });
      
      // 更新状态数据
      this.setData({
        batteryLevel: statusData.battery_level !== undefined ? statusData.battery_level : this.data.batteryLevel,
        autoProgress: statusData.progress !== undefined ? statusData.progress : this.data.autoProgress,
        autoStatus: statusData.status || this.data.autoStatus,
        robotConnected: true,
        reconnectingRobot: false
      });
      
      // 更新信号强度
      if (statusData.signal_strength) {
        let signalText = '未知';
        if (typeof statusData.signal_strength === 'number') {
          // 如果是数值，根据范围判断
          const strength = statusData.signal_strength;
          if (strength > 80) signalText = '极好';
          else if (strength > 60) signalText = '良好';
          else if (strength > 40) signalText = '一般';
          else if (strength > 20) signalText = '较弱';
          else signalText = '很差';
        } else {
          // 如果是字符串，直接判断
          switch(statusData.signal_strength) {
            case 'strong':
              signalText = '极好';
              break;
            case 'good':
              signalText = '良好';
              break;
            case 'medium':
              signalText = '一般';
              break;
            case 'weak':
              signalText = '较弱';
              break;
            case 'poor':
              signalText = '很差';
              break;
          }
        }
        this.setData({
          signalStrength: signalText
        });
      }
      
      // 视频流将自动恢复
    },
    
    // 计算缓冲健康度
    calculateBufferHealth: function() {
      // 基于延迟和抖动计算缓冲健康度
      const latency = this.data.frameLatency;
      const jitter = this.data.frameJitter;
      
      let health = 100;
      
      // 延迟超过500ms开始降低健康度
      if (latency > 500) {
        health -= Math.min(50, (latency - 500) / 20);
      }
      
      // 抖动超过100ms开始降低健康度
      if (jitter > 100) {
        health -= Math.min(50, (jitter - 100) / 4);
      }
      
      // 确保健康度在0-100之间
      health = Math.max(0, Math.min(100, health));
      
      this.setData({
        bufferHealth: Math.round(health)
      });
      
      return health;
    },
    
    // 发送状态更新
    sendStatusUpdate: function(bufferHealth) {
      this.sendSocketMessage({
        type: 'client_network_status',
        status: {
          buffer_health: bufferHealth,
          network_quality: this.getNetworkQuality(),
          latency: this.data.frameLatency,
          jitter: this.data.frameJitter,
          fps: this.data.framesPerSecond,
          dropped_frames: this.data.droppedFrames
        }
      });
    },
    
    // 获取网络质量评估
    getNetworkQuality: function() {
      const latency = this.data.frameLatency;
      const jitter = this.data.frameJitter;
      const health = this.data.bufferHealth;
      
      if (health > 90 && latency < 200 && jitter < 50) {
        return 'excellent';
      } else if (health > 70 && latency < 500 && jitter < 100) {
        return 'good';
      } else if (health > 40 && latency < 1000 && jitter < 200) {
        return 'fair';
      } else if (health > 20) {
        return 'poor';
      } else {
        return 'very_poor';
      }
    },
  
    // 刷新连接
    refreshConnection: function() {
      const that = this;
      
      // 如果正在检查连接状态，直接返回
      if (this.data.connectionChecking) return;
      
      this.setData({
        connectionChecking: true,
        connectionStatusText: this.data.connectionCheckingText,
        lastConnectionAttempt: Date.now()
      });
      
      // 通过全局方法重新连接
      const app = getApp();
      app.connectWebSocket();
      
      // 请求机器人状态
      setTimeout(() => {
        if (app.globalData.connected) {
          that.requestRobotStatus();
        }
      }, 1000);
      
      // 5秒后重置检查状态
      setTimeout(function() {
        that.setData({
          connectionChecking: false
        });
      }, 5000);
    },
    

    
    // 请求机器人状态
    requestRobotStatus: function() {
      if (!this.data.connected) return;
      
      this.sendSocketMessage({
        type: 'get_robot_status',
        robot_id: this.data.robotId,
        timestamp: Date.now()
      });
    },
  
    // 切换操作模式
    switchMode: function(e) {
      const mode = e.detail.value ? 'auto' : 'manual';
      this.setData({
        operationMode: mode
      });
      
      // 发送模式切换命令
      this.sendCommand(mode === 'auto' ? 'switch_to_auto' : 'switch_to_manual');
      
      wx.showToast({
        title: mode === 'auto' ? '已切换至自动模式' : '已切换至手动模式',
        icon: 'none'
      });
    },
  
    // 切换视频质量设置面板
    toggleQualityPanel: function() {
      this.setData({
        showQualityPanel: !this.data.showQualityPanel
      });
    },
    
    // 切换自动质量调整
    toggleAutoQuality: function(e) {
      const autoQuality = e.detail.value;
      this.setData({
        autoQuality: autoQuality
      });
      
      wx.showToast({
        title: autoQuality ? '自动调整视频质量已开启' : '自动调整视频质量已关闭',
        icon: 'none'
      });
      
      // 如果关闭了自动调整，发送当前质量作为手动设置
      if (!autoQuality) {
        this.setQualityPreset(this.data.currentQuality);
      }
    },
    
    // 设置质量预设
    setQualityPreset: function(preset) {
      if (this.data.qualityPresets.includes(preset)) {
        this.sendSocketMessage({
          type: 'client_quality_request',
          robot_id: this.data.robotId,
          preset: preset
        });
        
        // 暂时更新UI，实际值会在收到服务器确认后更新
        this.setData({
          currentQuality: preset
        });
      }
    },
    
    // 手动选择质量预设
    selectQualityPreset: function(e) {
      const preset = e.currentTarget.dataset.preset;
      this.setQualityPreset(preset);
      
      // 关闭质量面板
      this.setData({
        showQualityPanel: false
      });
    },
    
    // 处理电机速度变化
    changeMotorSpeed: function(e) {
      const speed = e.detail.value;
      this.setData({
        motorSpeed: speed
      });
      
      // 通知用户速度已更改
      wx.vibrateShort({
        type: 'light'
      });
      
      // 发送速度设置命令
      this.sendCommand('set_motor_speed', {
        speed: speed
      });
      
      console.log('移动速度已设置为:', speed);
    },

    // 切换控制类型：电机控制 / 伴侣交互控制
    switchControlType: function(e) {
      const newType = e.detail.value ? 'companion' : 'motor';
      
      this.setData({
        controlType: newType
      });
      
      // 发送控制类型切换命令
      this.sendCommand('switch_control_type', {
        control_type: newType
      });
      
      wx.showToast({
        title: newType === 'companion' ? '已切换至伴侣交互控制' : '已切换至移动控制',
        icon: 'none',
        duration: 1500
      });
      
      console.log('控制类型已切换为:', newType === 'companion' ? '伴侣交互控制' : '移动控制');
    },
  
    // 发送控制命令（带冷却时间限制）
    sendCommand: function(command, params = {}) {
      const now = Date.now();
      
      // 检查命令冷却时间
      if (now - this.data.lastCommandTime < this.data.commandCooldown) {
        console.log('命令发送过于频繁，已忽略:', command);
        return;
      }
      
      // 更新最后命令发送时间
      this.setData({
        lastCommandTime: now
      });
      
      // 为移动相关命令添加速度参数和控制类型
      if (['forward', 'backward', 'left', 'right'].includes(command)) {
        params.speed = this.data.motorSpeed;
        params.control_type = this.data.controlType;
        
        // 根据控制类型修改命令名称
        if (this.data.controlType === 'companion') {
          // 伴侣交互控制命令映射
          const companionCommandMap = {
            'forward': 'companion_look_up',       // 向上看
            'backward': 'companion_look_down',    // 向下看
            'left': 'companion_turn_left',        // 向左转头
            'right': 'companion_turn_right'       // 向右转头
          };
          command = companionCommandMap[command] || command;
        }
      }
      
          // 通过WebSocket发送命令
    this.sendSocketMessage({
      type: 'companion_command',
      robot_id: this.data.robotId,
      command: command,
      params: params
    });
    
    console.log('发送命令:', command, params);
    },
  
    // 手动控制方向
    moveForward: function() {
      this.sendCommand('forward');
      wx.vibrateShort({
        type: 'medium'
      });
    },
  
    moveBackward: function() {
      this.sendCommand('backward');
      wx.vibrateShort({
        type: 'medium'
      });
    },
  
    moveLeft: function() {
      this.sendCommand('left');
      wx.vibrateShort({
        type: 'medium'
      });
    },
  
    moveRight: function() {
      this.sendCommand('right');
      wx.vibrateShort({
        type: 'medium'
      });
    },
  
    stopMovement: function() {
      this.sendCommand('stop');
      wx.vibrateShort({
        type: 'heavy'
      });
    },
  
    startInteraction: function() {
      this.sendCommand('startInteraction');
      wx.showToast({
        title: '开始互动',
        icon: 'success'
      });
    },
  
    pauseInteraction: function() {
      this.sendCommand('stopInteraction');
      wx.showToast({
        title: '暂停互动',
        icon: 'none'
      });
    },
  
    // 自动模式控制
    startAutoMode: function() {
      this.sendCommand('start_auto');
      wx.showToast({
        title: '开始自动陪伴',
        icon: 'success'
      });
      this.setData({
        autoStatus: '自动陪伴中'
      });
    },
  
    pauseAutoMode: function() {
      this.sendCommand('pause_auto');
      wx.showToast({
        title: '已暂停',
        icon: 'none'
      });
      this.setData({
        autoStatus: '已暂停'
      });
    },
  
      // 紧急停止
  emergencyStop: function() {
    this.sendCommand('emergencyStop');
    wx.vibrateLong();
    wx.showModal({
      title: '紧急停止',
      content: '已发送紧急停止命令，伴侣已停止所有动作',
      showCancel: false
    });
    this.setData({
      autoStatus: '已停止'
    });
  },

  // 处理真实特征数据
  handleRealFeatureData: function(data) {
    console.log('🎯 收到真实特征数据:', data);
    
    try {
      // 检查当前是否在数据页面
      if (this.data.currentTab !== 'data') {
        console.log('⚠️ 当前不在数据页面，跳过特征数据处理');
        return;
      }
      
      // 处理身体比例数据
      if (data.body_ratios && data.body_ratios.length > 0) {
        console.log('📊 身体比例数据:', data.body_ratios);
        
        // 更新身体比例显示
        this.setData({
          'realFeatureData.bodyRatios': data.body_ratios,
          'realFeatureData.lastUpdate': Date.now()
        });
      }
      
      // 处理颜色数据
      if (data.shirt_color && data.pants_color) {
        console.log('🎨 颜色数据:', {
          shirt: data.shirt_color,
          pants: data.pants_color
        });
        
        // 更新颜色显示
        this.setData({
          'realFeatureData.shirtColor': data.shirt_color,
          'realFeatureData.pantsColor': data.pants_color,
          'realFeatureData.lastUpdate': Date.now()
        });
      }
      
      // 处理其他特征数据
      if (data.person_count !== undefined) {
        console.log('👥 人数统计:', data.person_count);
        
        this.setData({
          'realFeatureData.personCount': data.person_count,
          'realFeatureData.lastUpdate': Date.now()
        });
      }
      
      // 处理文件路径
      if (data.result_image_path) {
        console.log('📸 结果图片路径:', data.result_image_path);
        
        this.setData({
          'realFeatureData.resultImagePath': data.result_image_path,
          'realFeatureData.lastUpdate': Date.now()
        });
      }
      
      // 如果有完整的特征数据，显示成功提示
      if (data.status === 'success') {
        wx.showToast({
          title: '特征数据已更新',
          icon: 'success',
          duration: 1500
        });
      }
      
    } catch (error) {
      console.error('❌ 处理真实特征数据失败:', error);
      wx.showToast({
        title: '特征数据处理失败',
        icon: 'none',
        duration: 2000
      });
    }
  }
});