// app.js - 机器人伴侣(智能陪伴)微信小程序
App({
    globalData: {
      userInfo: null,
      socketTask: null,
      connected: false,
      connecting: false,
      serverUrl: "ws://101.201.150.96:1234/ws/companion/", // 机器人伴侣服务器地址
      clientId: '',
      robotId: 'companion_robot_001',
      
      // 页面引用，用于消息分发
      controlPage: null,
      historyPage: null,
      featurePage: null,
      aiAssistantPage: null,
      
      // 连接管理
      lastPongTime: 0,
      reconnectDelay: 3000,
      maxReconnectDelay: 30000,
      reconnectAttempts: 0,
      heartbeatTimer: null,
      reconnectTimer: null,
      closedByUser: false,
      lastMessageTime: 0,
      clientVersion: '2.0.1', // 更新版本号
      networkLatency: 0,
      
      // 性能优化配置
      debugMode: false, // 调试模式，控制日志输出
      lastVideoFrameTime: 0, // 视频帧处理时间戳
      performanceMonitor: true, // 性能监控开关
      deviceInfo: null, // 设备信息缓存
      appBaseInfo: null, // 应用基础信息缓存
      
      // 机器人伴侣状态
      followingMode: 'idle', // 'idle', 'following', 'waiting', 'lost'
      companionMode: 'manual', // 'manual', 'auto', 'interactive'
      targetPersonId: null,
      batteryLevel: 0,
      signalStrength: '未连接',
      followDistance: 1.5, // 跟随距离(米)
      followSpeed: 0.8, // 跟随速度(m/s)
      
      // 视频流相关
      videoStreamActive: false,
      videoQuality: 'medium',
      videoResolution: { width: 480, height: 360 }, // 默认分辨率
      frameRate: 10,
      lastVideoFrame: null,
      videoStats: {
        receivedFrames: 0,
        droppedFrames: 0,
        latency: 0,
        jitter: 0
      },
      
      // 命令队列和状态
      commandQueue: [],
      commandSending: false,
      lastCommandTime: 0,
      commandCooldown: 300, // 减少命令冷却时间
      
      // 特征数据
      extractedFeatures: [],
      currentTarget: null,
      
      // 位置和导航
      robotPosition: { x: 0, y: 0, orientation: 0 },
      targetPosition: { x: 0, y: 0 },
      trackingHistory: [],
      
      // 交互状态
      interactionMode: false,
      voiceActive: false,
      emotionState: 'neutral' // 'happy', 'sad', 'excited', 'calm', 'neutral'
    },
    
    onLaunch: function () {
      console.log('🤖 机器人伴侣小程序启动 v2.0.1');
      
      // 初始化性能优化设置
      this.initPerformanceSettings();
      
      // 生成唯一客户端ID
      this.globalData.clientId = `companion_${Date.now()}_${Math.floor(Math.random() * 10000)}`;
      
      // 连接WebSocket服务器
      this.connectWebSocket();
      
      // 获取用户权限
      this.getUserPermissions();
      
      // 初始化本地存储
      this.initLocalStorage();
      
      // 初始化命令处理器
      this.initCommandProcessor();
      
      // 初始化性能优化定时器
      this.initPerformanceTimers();
    },

    // 应用隐藏时的处理
    onHide: function() {
      if (this.globalData.debugMode) {
        console.log('🔄 应用已隐藏，暂停部分功能');
      }
      
      // 暂停视频流处理
      this.globalData.videoStreamActive = false;
      
      // 清理性能定时器，释放资源
      this.clearPerformanceTimers();
      
      // 刷新所有缓冲数据
      this.flushAllBuffers();
    },

    // 应用显示时的处理
    onShow: function() {
      if (this.globalData.debugMode) {
        console.log('🔄 应用已显示，恢复功能');
      }
      
      // 检查连接状态
      if (!this.globalData.connected && !this.globalData.connecting) {
        this.connectWebSocket();
      }
      
      // 重新启动性能定时器
      this.initPerformanceTimers();
    },

    // 应用卸载时的处理
    onUnload: function() {
      if (this.globalData.debugMode) {
        console.log('🔄 应用正在卸载，清理所有资源');
      }
      
      // 清理所有定时器
      this.clearPerformanceTimers();
      this.stopHeartbeat();
      
      // 关闭WebSocket连接
      if (this.globalData.socketTask) {
        this.globalData.socketTask.close();
      }
    },

    // 刷新所有缓冲数据
    flushAllBuffers: function() {
      try {
        if (this._featureDataBuffer && this._featureDataBuffer.length > 0) {
          this._flushFeatureData();
        }
        if (this._trackingDataBuffer && this._trackingDataBuffer.length > 0) {
          this._flushTrackingData();
        }
      } catch (error) {
        if (this.globalData.debugMode) {
          console.error('刷新缓冲数据失败:', error);
        }
      }
    },
    
    // 初始化性能优化设置
    initPerformanceSettings: function() {
      try {
        // 检查是否为调试模式
        const launchOptions = wx.getLaunchOptionsSync();
        if (launchOptions.scene === 1001 || launchOptions.query.debug) {
          this.globalData.debugMode = true;
          console.log('🔧 调试模式已启用');
        }
        
        // 获取设备信息进行性能优化 - 使用新的API
        const deviceInfo = wx.getDeviceInfo();
        const appBaseInfo = wx.getAppBaseInfo();
        
        // 根据设备性能调整配置
        if (deviceInfo.benchmarkLevel < 20) {
          // 低性能设备优化
          this.globalData.commandCooldown = 500;
          this.globalData.frameRate = 8;
          this.globalData.videoQuality = 'low';
          if (this.globalData.debugMode) {
            console.log('📱 检测到低性能设备，已启用性能优化');
          }
        } else if (deviceInfo.benchmarkLevel > 50) {
          // 高性能设备
          this.globalData.commandCooldown = 200;
          this.globalData.frameRate = 15;
          if (this.globalData.debugMode) {
            console.log('📱 检测到高性能设备，已启用高性能模式');
          }
        }
        
        // 存储设备信息供后续使用
        this.globalData.deviceInfo = deviceInfo;
        this.globalData.appBaseInfo = appBaseInfo;
        
        // 清理已存在的定时器
        if (this._trackingDataSaveTimer) {
          clearTimeout(this._trackingDataSaveTimer);
        }
        if (this._featureDataSaveTimer) {
          clearTimeout(this._featureDataSaveTimer);
        }
        
      } catch (error) {
        console.error('初始化性能设置失败:', error);
      }
    },

    // 获取用户权限（相机、位置、录音等）
    getUserPermissions: function() {
      const permissions = [
        'scope.camera',
        'scope.userLocation', 
        'scope.record',
        'scope.writePhotosAlbum'
      ];
      
      permissions.forEach(permission => {
        wx.getSetting({
          success: (res) => {
            if (!res.authSetting[permission]) {
              wx.authorize({
                scope: permission,
                success: () => {
                  if (this.globalData.debugMode) {
                    console.log(`✅ ${permission} 权限获取成功`);
                  }
                },
                fail: (error) => {
                  console.error(`❌ ${permission} 权限获取失败`, error);
                }
              });
            }
          }
        });
      });
    },
    
    // 初始化本地存储
    initLocalStorage: function() {
      // 初始化陪伴历史
      const trackingHistory = wx.getStorageSync('companionHistory') || [];
      this.globalData.trackingHistory = trackingHistory;
      
      // 初始化特征数据
      const extractedFeatures = wx.getStorageSync('extractedFeatures') || [];
      this.globalData.extractedFeatures = extractedFeatures;
      
      // 初始化用户偏好设置
      const userPreferences = wx.getStorageSync('userPreferences') || {
        videoQuality: 'medium',
        followDistance: 1.5,
        interactionMode: true,
        voiceEnabled: true
      };
      
      this.globalData.videoQuality = userPreferences.videoQuality;
      this.globalData.followDistance = userPreferences.followDistance;
      this.globalData.interactionMode = userPreferences.interactionMode;
      
      console.log('💾 本地存储初始化完成');
    },
    
    // 初始化命令处理器
    initCommandProcessor: function() {
      // 启动命令队列处理器
      this.startCommandProcessor();
    },
    
    // 连接WebSocket服务器
    connectWebSocket: function() {
      const that = this;
      
      if (this.globalData.connecting || this.globalData.connected) {
        return;
      }
      
      this.globalData.connecting = true;
      
      const wsUrl = `${this.globalData.serverUrl}${this.globalData.clientId}`;
      
      console.log('🔗 正在连接机器人伴侣服务器...', wsUrl);
      
      this.globalData.socketTask = wx.connectSocket({
        url: wsUrl,
        success: function() {
          console.log('📡 WebSocket连接请求已发送');
        },
        fail: function(error) {
          console.error('❌ WebSocket连接失败', error);
          that.globalData.connecting = false;
          that.scheduleReconnect();
        }
      });
      
      // 监听连接打开
      this.globalData.socketTask.onOpen(function() {
        console.log('✅ 机器人伴侣连接已建立');
        that.globalData.connected = true;
        that.globalData.connecting = false;
        that.globalData.reconnectAttempts = 0;
        that.globalData.lastPongTime = Date.now();
        
        // 发送初始化消息
        that.sendSocketMessage({
          type: 'client_init',
          robot_id: that.globalData.robotId,
          client_id: that.globalData.clientId,
          client_version: that.globalData.clientVersion,
          connection_time: Date.now(),
          device_type: 'companion_client',
          capabilities: {
            video_receive: true,
            command_send: true,
            feature_extraction: true,
            voice_interaction: true
          }
        });
        
        // 启动心跳检测
        that.startHeartbeat();
        
        // 请求初始状态
        that.requestInitialStatus();
      });
      
      // 监听接收消息
      this.globalData.socketTask.onMessage(function(res) {
        try {
          const data = JSON.parse(res.data);
          const now = Date.now();
          that.globalData.lastMessageTime = now;
          
          // 处理心跳响应 - 直接返回，不进入分发流程
          if (data.type === 'pong') {
            that.globalData.lastPongTime = now;
            if (data.echo_timestamp) {
              that.globalData.networkLatency = now - data.echo_timestamp;
            }
            return;
          }
          
          // 处理ping消息 - 直接返回
          if (data.type === 'ping') {
            return;
          }
          
          // 过滤频繁但不重要的消息类型，减少处理负担
          const frequentTypes = ['heartbeat', 'status_ping', 'alive_check'];
          if (frequentTypes.includes(data.type)) {
            return;
          }
          
          // 根据消息类型分发到对应页面
          that.distributeMessage(data);
          
        } catch (error) {
          console.error('❌ 解析WebSocket消息失败:', error);
        }
      });
      
      // 监听连接错误
      this.globalData.socketTask.onError(function(error) {
        console.error('❌ WebSocket发生错误:', error);
        that.globalData.connected = false;
        that.globalData.connecting = false;
        that.globalData.videoStreamActive = false;
        that.stopHeartbeat();
        that.scheduleReconnect();
      });
      
      // 监听连接关闭
      this.globalData.socketTask.onClose(function(event) {
        console.log('🔌 WebSocket连接已关闭, 代码:', event.code, '原因:', event.reason);
        that.globalData.connected = false;
        that.globalData.connecting = false;
        that.globalData.videoStreamActive = false;
        that.stopHeartbeat();
        
        if (!that.globalData.closedByUser) {
          that.scheduleReconnect();
        }
      });
    },
    
    // 请求初始状态
    requestInitialStatus: function() {
      // 请求机器人状态
      this.sendSocketMessage({
        type: 'get_robot_status',
        robot_id: this.globalData.robotId
      });
      
      // 视频流将自动开始传输
    },
    
    // 消息分发到对应页面 - 高度优化版本
    distributeMessage: function(data) {
      // 直接处理，避免双重异步嵌套
      this._distributeMessageSync(data);
    },
    
    // 异步消息分发处理 - 高度优化版本
    _distributeMessageSync: function(data) {
      // 对于频繁的消息类型，使用setTimeout进行批处理
      const isFrequentMessage = data.type === 'video_frame' || data.type === 'robot_status_update' || data.type === 'tracking_data';
      
      if (isFrequentMessage) {
        // 频繁消息使用setTimeout异步处理
        setTimeout(() => {
          this._handleMessage(data);
        }, 0);
      } else {
        // 非频繁消息直接处理
        this._handleMessage(data);
      }
    },
    
    // 核心消息处理方法
    _handleMessage: function(data) {
        switch (data.type) {
          case 'robot_status_update':
            // 机器人状态更新
            this.handleRobotStatusUpdate(data);
            break;
            
          case 'video_frame':
            // 视频帧数据 - 增加节流机制
            this.handleVideoFrameThrottled(data);
            break;
            
          case 'command_response':
            // 命令执行响应
            this.handleCommandResponseOptimized(data);
            break;
            
          case 'companion_status_update':
            // 伴侣状态更新
            this.handleCompanionStatusUpdate(data);
            break;
            
          case 'tracking_data':
            // 跟踪数据更新 - 异步处理
            this.handleTrackingDataAsync(data);
            break;
            
          case 'detailed_tracking_data':
            // 详细跟踪数据更新 - 异步处理
            this.handleDetailedTrackingDataAsync(data);
            break;
            
          case 'file_upload_success':
            // 文件上传成功
            if (this.globalData.featurePage) {
              this.globalData.featurePage.handleFileUploadSuccess(data);
            }
            break;

          case 'file_save_result':
            // 文件保存结果
            if (this.globalData.featurePage) {
              this.globalData.featurePage.handleFileSaveResult(data);
            }
            break;

          case 'feature_extraction_result':
            // 特征提取结果（旧格式，保留兼容性）
            this.handleFeatureExtractionAsync(data);
            break;

          case 'feature_extraction_complete':
            // 特征提取完成（新格式）
            this.handleFeatureExtractionCompleteAsync(data);
            break;

          case 'feature_extraction_error':
            // 特征提取错误
            if (this.globalData.featurePage) {
              this.globalData.featurePage.handleFeatureError(data);
            }
            break;

          case 'processed_image_notification':
            // 处理后图片通知 - 异步处理
            this.handleProcessedImageAsync(data);
            break;
            
          case 'ai_response':
            // AI助手回复
            if (this.globalData.aiAssistantPage) {
              this.globalData.aiAssistantPage.handleAIResponse(data);
            }
            break;
            
          case 'position_update':
            // 位置更新
            this.handlePositionUpdate(data);
            break;
            
          case 'interaction_event':
            // 交互事件
            this.handleInteractionEvent(data);
            break;
            
          case 'robot_connection_status':
            // 机器人连接状态更新
            this.handleRobotConnectionStatus(data);
            break;
            
          case 'video_quality_update':
            // 视频质量更新
            this.handleVideoQualityUpdate(data);
            break;
            
          case 'quality_request_received':
            // 质量调整请求已接收
            this.handleQualityRequestReceived(data);
            break;
            
          case 'companion_disconnected':
            // 伴侣机器人断开连接
            this.handleCompanionDisconnected(data);
            break;
            
          case 'error':
            // 错误消息
            this.handleError(data);
            break;
            
          default:
            // 减少日志输出 - 只在调试模式下输出
            if (data.type !== 'ping' && data.type !== 'pong' && this.globalData.debugMode) {
              console.log('🔍 未知消息类型:', data.type);
            }
        }
    },
    
    // 处理机器人状态更新 - 优化版本
    handleRobotStatusUpdate: function(data) {
      try {
        // 使用节流，避免过于频繁的状态更新
        const now = Date.now();
        if (!this.globalData.lastRobotStatusUpdate || now - this.globalData.lastRobotStatusUpdate > 200) {
          this.globalData.lastRobotStatusUpdate = now;
          
          // 快速更新全局数据
          this.globalData.batteryLevel = data.battery_level || this.globalData.batteryLevel;
          this.globalData.signalStrength = data.signal_strength || this.globalData.signalStrength;  
          this.globalData.companionMode = data.mode || this.globalData.companionMode;
          
          // 异步更新控制页面，避免阻塞
          if (this.globalData.controlPage) {
            setTimeout(() => {
              this.globalData.controlPage.updateRobotStatus(data);
            }, 0);
          }
        }
      } catch (error) {
        // 静默处理错误
        if (this.globalData.debugMode) {
          console.error('Robot status update error:', error);
        }
      }
    },
    
    // 处理视频帧 - 优化版本
    handleVideoFrame: function(data) {
      this.globalData.videoStreamActive = true;
      this.globalData.lastVideoFrame = data;
      
      // 节流：只每5帧更新一次统计
      if (data.sequence % 5 === 0) {
        this.globalData.videoStats.receivedFrames++;
        if (data.sequence && this.globalData.lastFrameSequence) {
          const expectedSeq = this.globalData.lastFrameSequence + 1;
          if (data.sequence > expectedSeq) {
            this.globalData.videoStats.droppedFrames += (data.sequence - expectedSeq);
          }
        }
        this.globalData.lastFrameSequence = data.sequence;
      }
      
      // 分发到控制页面 - 使用异步防止阻塞
      if (this.globalData.controlPage) {
        wx.nextTick(() => {
          this.globalData.controlPage.handleVideoFrame(data);
        });
      }
    },

    // 视频帧节流处理 - 新增优化方法
    handleVideoFrameThrottled: function(data) {
      // 使用时间戳节流，避免过于频繁的视频帧处理
      const now = Date.now();
      if (!this.globalData.lastVideoFrameTime || now - this.globalData.lastVideoFrameTime > 100) {
        this.globalData.lastVideoFrameTime = now;
        this.handleVideoFrame(data);
      }
    },

    // 命令响应优化处理
    handleCommandResponseOptimized: function(data) {
      // 标记命令发送完成
      this.globalData.commandSending = false;
      
      // 异步处理下一个命令
      setTimeout(() => {
        this.processNextCommand();
      }, 0);
      
      // 分发到控制页面 - 简化处理
      if (this.globalData.controlPage && data.status) {
        if (data.status === 'success') {
          // 成功时只在控制台简单记录
          if (this.globalData.debugMode) {
            console.log(`✅ ${data.command} 成功`);
          }
        } else {
          // 失败时使用异步显示错误
          wx.nextTick(() => {
            console.error('❌ 命令执行失败:', data.command, data.error);
            wx.showToast({
              title: `命令执行失败: ${data.error || '未知错误'}`,
              icon: 'none',
              duration: 2000
            });
          });
        }
      }
    },

    // 跟踪数据异步处理
    handleTrackingDataAsync: function(data) {
      // 异步分发到控制页面（新增）
      if (this.globalData.controlPage) {
        wx.nextTick(() => {
          this.globalData.controlPage.handleTrackingData(data);
        });
      }
      
      // 异步分发到历史页面
      if (this.globalData.historyPage) {
        wx.nextTick(() => {
          this.globalData.historyPage.handleTrackingData(data);
        });
      }
      
      // 异步保存跟踪数据
      setTimeout(() => {
        this.saveTrackingData(data);
      }, 0);
    },
    
    // 详细跟踪数据异步处理
    handleDetailedTrackingDataAsync: function(data) {
      console.log('📈 收到详细跟踪数据:', data);
      
      // 异步分发到控制页面
      if (this.globalData.controlPage) {
        wx.nextTick(() => {
          this.globalData.controlPage.handleDetailedTrackingData(data);
        });
      }
      
      // 异步分发到历史页面
      if (this.globalData.historyPage) {
        wx.nextTick(() => {
          this.globalData.historyPage.handleDetailedTrackingData(data);
        });
      }
      
      // 异步保存详细跟踪数据
      setTimeout(() => {
        this.saveDetailedTrackingData(data);
      }, 0);
    },

    // 特征提取异步处理
    handleFeatureExtractionAsync: function(data) {
      if (this.globalData.featurePage) {
        wx.nextTick(() => {
          this.globalData.featurePage.handleFeatureResult(data);
        });
      }
      
      // 分发到控制页面显示真实特征数据
      if (this.globalData.controlPage && this.globalData.controlPage.handleRealFeatureData) {
        wx.nextTick(() => {
          this.globalData.controlPage.handleRealFeatureData(data);
        });
      }
      
      // 异步保存特征数据
      setTimeout(() => {
        this.saveFeatureData(data);
      }, 0);
    },

    // 特征提取完成异步处理
    handleFeatureExtractionCompleteAsync: function(data) {
      if (this.globalData.featurePage) {
        wx.nextTick(() => {
          this.globalData.featurePage.handleFeatureResult(data);
        });
      }
      
      // 分发到控制页面显示真实特征数据
      if (this.globalData.controlPage && this.globalData.controlPage.handleRealFeatureData) {
        wx.nextTick(() => {
          this.globalData.controlPage.handleRealFeatureData(data);
        });
      }
      
      if (data.status === 'success') {
        // 异步保存特征数据
        setTimeout(() => {
          this.saveFeatureData(data);
        }, 0);
      }
    },

    // 处理后图片异步处理
    handleProcessedImageAsync: function(data) {
      if (this.globalData.featurePage) {
        wx.nextTick(() => {
          this.globalData.featurePage.handleProcessedImageNotification(data);
        });
      }
      
      // 异步保存处理后的图片数据
      setTimeout(() => {
        this.saveProcessedImageData(data);
      }, 0);
    },
    
    // 处理命令响应
    handleCommandResponse: function(data) {
      // 标记命令发送完成
      this.globalData.commandSending = false;
      
      // 处理下一个命令
      this.processNextCommand();
      
      // 分发到控制页面
      if (this.globalData.controlPage && data.status) {
        if (data.status === 'success') {
          // 成功时只在控制台简单记录
          console.log(`✅ ${data.command} 成功`);
        } else {
          // 失败时详细记录
          console.error('❌ 命令执行失败:', data.command, data.error);
          wx.showToast({
            title: `命令执行失败: ${data.error || '未知错误'}`,
            icon: 'none',
            duration: 2000
          });
        }
      }
    },
    
    // 处理伴侣状态更新
    handleCompanionStatusUpdate: function(data) {
      this.globalData.followingMode = data.following_mode || this.globalData.followingMode;
      this.globalData.emotionState = data.emotion_state || this.globalData.emotionState;
      this.globalData.interactionMode = data.interaction_mode !== undefined ? 
        data.interaction_mode : this.globalData.interactionMode;
      
      if (this.globalData.controlPage) {
        this.globalData.controlPage.handleCompanionStatusUpdate(data);
      }
    },
    
    // 处理位置更新
    handlePositionUpdate: function(data) {
      this.globalData.robotPosition = data.robot_position || this.globalData.robotPosition;
      this.globalData.targetPosition = data.target_position || this.globalData.targetPosition;
      
      if (this.globalData.historyPage) {
        this.globalData.historyPage.handlePositionUpdate(data);
      }
    },
    
    // 处理交互事件
    handleInteractionEvent: function(data) {
      // 根据交互类型处理
      switch (data.event_type) {
        case 'gesture_detected':
          console.log('🤝 检测到手势');
          break;
        case 'voice_command':
          console.log('🤝 收到语音命令');
          break;
        case 'emotion_change':
          console.log('🤝 情绪变化:', data.emotion);
          this.globalData.emotionState = data.emotion;
          break;
        default:
          console.log('🤝 交互事件:', data.event_type);
      }
      
      if (this.globalData.controlPage) {
        this.globalData.controlPage.handleInteractionEvent(data);
      }
    },
    
    // 处理错误
    handleError: function(data) {
      console.error('🚨 服务器错误:', data);
      wx.showToast({
        title: data.message || '发生错误',
        icon: 'none',
        duration: 3000
      });
    },
    
        // 保存跟踪数据 - 高性能优化版本
    saveTrackingData: function(data) {
      // 节流：只每15条记录保存一次，减少处理频率
      if (!this._trackingDataBuffer) {
        this._trackingDataBuffer = [];
      }
      
      const trackingEntry = {
        timestamp: Date.now(),
        robot_position: data.robot_position,
        target_position: data.target_position,
        following_mode: data.following_mode,
        distance: data.distance
      };
      
      this._trackingDataBuffer.push(trackingEntry);
      
      // 批量处理：每15条数据处理一次，减少IO操作
      if (this._trackingDataBuffer.length >= 15) {
        this._flushTrackingData();
      }
    },
    
    // 保存详细跟踪数据 - 高性能优化版本
    saveDetailedTrackingData: function(data) {
      // 节流：只每10条记录保存一次，减少处理频率
      if (!this._detailedTrackingDataBuffer) {
        this._detailedTrackingDataBuffer = [];
      }
      
      const detailedData = data.data || {};
      const detailedTrackingEntry = {
        timestamp: Date.now(),
        frame_id: detailedData.frame_id,
        tracking_mode: detailedData.tracking_mode,
        target_detected: detailedData.target_detected,
        total_tracks: detailedData.total_tracks,
        target_track: detailedData.target_track,
        tracks_count: detailedData.tracks ? detailedData.tracks.length : 0,
        system_info: detailedData.system_info
      };
      
      this._detailedTrackingDataBuffer.push(detailedTrackingEntry);
      
      // 批量处理：每10条数据处理一次，减少IO操作
      if (this._detailedTrackingDataBuffer.length >= 10) {
        this._flushDetailedTrackingData();
      }
    },

    // 批量刷新跟踪数据 - 优化版本
    _flushTrackingData: function() {
      if (!this._trackingDataBuffer || this._trackingDataBuffer.length === 0) {
        return;
      }
      
      try {
        this.globalData.trackingHistory.push(...this._trackingDataBuffer);
        this._trackingDataBuffer = [];
        
        // 限制历史记录数量
        if (this.globalData.trackingHistory.length > 1000) {
          this.globalData.trackingHistory = this.globalData.trackingHistory.slice(-800);
        }
        
        // 异步保存到本地存储，使用防抖机制
        if (this._trackingDataSaveTimer) {
          clearTimeout(this._trackingDataSaveTimer);
        }
        
        this._trackingDataSaveTimer = setTimeout(() => {
          try {
            wx.setStorageSync('companionHistory', this.globalData.trackingHistory);
          } catch (error) {
            if (this.globalData.debugMode) {
              console.error('保存跟踪数据失败:', error);
            }
          }
        }, 1000);
      } catch (error) {
        if (this.globalData.debugMode) {
          console.error('刷新跟踪数据失败:', error);
        }
      }
    },
    
    // 批量刷新详细跟踪数据 - 优化版本
    _flushDetailedTrackingData: function() {
      if (!this._detailedTrackingDataBuffer || this._detailedTrackingDataBuffer.length === 0) {
        return;
      }
      
      try {
        // 添加到全局数据
        if (!this.globalData.detailedTrackingHistory) {
          this.globalData.detailedTrackingHistory = [];
        }
        
        this.globalData.detailedTrackingHistory.push(...this._detailedTrackingDataBuffer);
        this._detailedTrackingDataBuffer = [];
        
        // 限制历史记录数量（详细数据通常更大，保留较少记录）
        if (this.globalData.detailedTrackingHistory.length > 500) {
          this.globalData.detailedTrackingHistory = this.globalData.detailedTrackingHistory.slice(-300);
        }
        
        // 异步保存到本地存储，使用防抖机制
        if (this._detailedTrackingDataSaveTimer) {
          clearTimeout(this._detailedTrackingDataSaveTimer);
        }
        
        this._detailedTrackingDataSaveTimer = setTimeout(() => {
          try {
            wx.setStorageSync('companionDetailedHistory', this.globalData.detailedTrackingHistory);
          } catch (error) {
            if (this.globalData.debugMode) {
              console.error('保存详细跟踪数据失败:', error);
            }
          }
        }, 1000);
      } catch (error) {
        if (this.globalData.debugMode) {
          console.error('刷新详细跟踪数据失败:', error);
        }
      }
    },

    // 保存特征数据 - 高性能优化版本
    saveFeatureData: function(data) {
      // 节流：批量处理特征数据，减少处理频率
      if (!this._featureDataBuffer) {
        this._featureDataBuffer = [];
      }
      
      const featureEntry = {
        id: data.feature_id || Date.now(),
        timestamp: Date.now(),
        features: data.features,
        person_id: data.person_id,
        confidence: data.confidence,
        image_data: data.image_data
      };
      
      this._featureDataBuffer.push(featureEntry);
      
      // 批量处理：每8条数据处理一次，优化处理频率
      if (this._featureDataBuffer.length >= 8) {
        this._flushFeatureData();
      }
    },

    // 批量刷新特征数据 - 优化版本
    _flushFeatureData: function() {
      if (!this._featureDataBuffer || this._featureDataBuffer.length === 0) {
        return;
      }
      
      try {
        this.globalData.extractedFeatures.push(...this._featureDataBuffer);
        this._featureDataBuffer = [];
        
        // 限制特征数据数量
        if (this.globalData.extractedFeatures.length > 100) {
          this.globalData.extractedFeatures = this.globalData.extractedFeatures.slice(-80);
        }
        
        // 异步保存到本地存储，使用防抖机制
        if (this._featureDataSaveTimer) {
          clearTimeout(this._featureDataSaveTimer);
        }
        
        this._featureDataSaveTimer = setTimeout(() => {
          try {
            wx.setStorageSync('extractedFeatures', this.globalData.extractedFeatures);
          } catch (error) {
            if (this.globalData.debugMode) {
              console.error('保存特征数据失败:', error);
            }
          }
        }, 1000);
      } catch (error) {
        if (this.globalData.debugMode) {
          console.error('刷新特征数据失败:', error);
        }
      }
    },

    // 保存处理后图片数据 - 优化版本
    saveProcessedImageData: function(data) {
      // 移除大部分调试日志，只保留关键信息
      console.log('📷 接收到处理后图片数据');
      
      // 提取图片数据
      const originalImage = data.original_image || data.image_data?.data_base64 || '';
      const processedImage = data.processed_image || data.image_data?.data_base64 || '';
      const resultImage = data.result_image || data.image_data?.data_base64 || '';
      
      // 确保图片数据格式正确
      const formatImage = (imageData) => {
        if (!imageData) return '';
        if (imageData.startsWith('data:image/')) return imageData;
        return `data:image/jpeg;base64,${imageData}`;
      };
      
      // 提取特征数据
      const features = data.features || {};
      const bodyRatios = features.body_ratios || new Array(16).fill(0.0);
      const clothingColors = features.clothing_colors || data.colors || {};
      const bodyProportions = features.body_proportions || data.proportions || {};
      const detailedProportions = features.detailed_proportions || data.detailed_proportions || [];
      
      // 移除详细调试日志以提升性能
      
      // 格式化时间戳
      const formatTimestamp = (ts) => {
        try {
          const date = new Date(ts);
          if (isNaN(date.getTime())) {
            return new Date().toLocaleString();
          }
          const year = date.getFullYear();
          const month = String(date.getMonth() + 1).padStart(2, '0');
          const day = String(date.getDate()).padStart(2, '0');
          const hour = String(date.getHours()).padStart(2, '0');
          const minute = String(date.getMinutes()).padStart(2, '0');
          return `${year}-${month}-${day} ${hour}:${minute}`;
        } catch (e) {
          return new Date().toLocaleString();
        }
      };

      const processedImageEntry = {
        id: data.extraction_id || Date.now(),
        timestamp: formatTimestamp(data.timestamp || Date.now()),
        name: data.person_name || `处理结果_${new Date().getTime()}`,
        
        // 图片数据
        image_data: formatImage(originalImage),
        processed_image: formatImage(processedImage),
        result_image: formatImage(resultImage),
        previewImage: formatImage(resultImage), // 兼容字段
        
        // 特征数据
        features: {
          body_ratios: bodyRatios,
          clothing_colors: clothingColors,
          body_proportions: bodyProportions,
          detailed_proportions: detailedProportions
        },
        
        // 兼容的顶层字段
        body_proportions: bodyProportions,
        detailed_proportions: detailedProportions,
        clothing_colors: clothingColors,
        
        // 颜色信息（兼容现有格式）
        topColor: data.topColor || clothingColors.top?.color || '#000000',
        bottomColor: data.bottomColor || clothingColors.bottom?.color || '#000000',
        topColorName: data.topColorName || clothingColors.top?.name || '黑色',
        bottomColorName: data.bottomColorName || clothingColors.bottom?.name || '黑色',
        
        // 状态信息
        status: 'success',
        extraction_type: 'processed_image',
        confidence: 95, // 默认置信度
        
        // 处理信息
        processing_info: data.processing_info || {}
      };
      
      // 添加到全局特征数据中（复用现有的存储）
      this.globalData.extractedFeatures = this.globalData.extractedFeatures || [];
      this.globalData.extractedFeatures.push(processedImageEntry);
      
      // 限制特征数据数量
      if (this.globalData.extractedFeatures.length > 100) {
        this.globalData.extractedFeatures = this.globalData.extractedFeatures.slice(-80);
      }
      
      // 保存到本地存储
      wx.setStorageSync('extractedFeatures', this.globalData.extractedFeatures);
      
      // 简化日志输出
      console.log('💾 保存处理后图片数据完成:', processedImageEntry.id);
    },
    
    // 启动命令处理器
    startCommandProcessor: function() {
      const that = this;
      
      // 每200ms检查一次命令队列
      setInterval(() => {
        if (!that.globalData.commandSending && that.globalData.commandQueue.length > 0) {
          that.processNextCommand();
        }
      }, 200);
    },
    
    // 处理下一个命令
    processNextCommand: function() {
      if (this.globalData.commandQueue.length === 0 || this.globalData.commandSending) {
        return;
      }
      
      const command = this.globalData.commandQueue.shift();
      const now = Date.now();
      
      // 检查命令冷却时间
      if (now - this.globalData.lastCommandTime < this.globalData.commandCooldown) {
        // 重新加入队列
        this.globalData.commandQueue.unshift(command);
        return;
      }
      
      this.globalData.commandSending = true;
      this.globalData.lastCommandTime = now;
      
      // 发送命令
      this.sendSocketMessage(command);
    },
    
    // 发送控制命令
    sendCompanionCommand: function(command, params = {}) {
      const commandMessage = {
        type: 'companion_command',
        robot_id: this.globalData.robotId,
        command: command,
        params: params,
        timestamp: Date.now(),
        client_id: this.globalData.clientId
      };
      
      // 加入命令队列
      this.globalData.commandQueue.push(commandMessage);
    },
    
    // 请求视频流质量调整
    requestVideoQuality: function(quality) {
      this.sendSocketMessage({
        type: 'client_quality_request',
        robot_id: this.globalData.robotId,
        preset: quality,
        timestamp: Date.now()
      });
      
      this.globalData.videoQuality = quality;
    },
    
    // 启动心跳机制
    startHeartbeat: function() {
      const that = this;
      
      if (this.globalData.heartbeatTimer) {
        clearInterval(this.globalData.heartbeatTimer);
      }
      
      this.globalData.lastPongTime = Date.now();
      
      this.globalData.heartbeatTimer = setInterval(() => {
        if (that.globalData.connected) {
          const now = Date.now();
          if (now - that.globalData.lastPongTime > 30000) {
            console.warn('💔 心跳超时，连接可能已断开');
            that.globalData.connected = false;
            that.globalData.videoStreamActive = false;
            
            if (that.globalData.socketTask) {
              try {
                that.globalData.socketTask.close({
                  code: 1000,
                  reason: '心跳超时'
                });
              } catch (e) {
                console.error('❌ 关闭超时连接失败:', e);
              }
            }
            
            that.stopHeartbeat();
            that.scheduleReconnect();
            return;
          }
          
          that.sendSocketMessage({
            type: 'ping',
            timestamp: now,
            client_stats: {
              video_frames_received: that.globalData.videoStats.receivedFrames,
              video_frames_dropped: that.globalData.videoStats.droppedFrames,
              network_latency: that.globalData.networkLatency
            }
          });
        }
      }, 15000);
    },
    
    // 停止心跳
    stopHeartbeat: function() {
      if (this.globalData.heartbeatTimer) {
        clearInterval(this.globalData.heartbeatTimer);
        this.globalData.heartbeatTimer = null;
      }
    },
    
    // 安排重连
    scheduleReconnect: function() {
      const that = this;
      
      if (this.globalData.reconnectTimer) {
        clearTimeout(this.globalData.reconnectTimer);
      }
      
      const attempts = this.globalData.reconnectAttempts;
      const delay = Math.min(
        this.globalData.maxReconnectDelay,
        this.globalData.reconnectDelay * Math.pow(1.5, attempts)
      );
      
      console.log(`🔄 安排第 ${attempts + 1} 次重连，延迟 ${delay}ms`);
      
      this.globalData.reconnectTimer = setTimeout(function() {
        that.globalData.reconnectAttempts++;
        that.connectWebSocket();
      }, delay);
    },
    
    // 发送WebSocket消息
    sendSocketMessage: function(msg) {
      if (this.globalData.socketTask && this.globalData.connected) {
        if (typeof msg === 'object') {
          msg.client_timestamp = Date.now();
        }
        
        this.globalData.socketTask.send({
          data: JSON.stringify(msg),
          success: function() {
            // 成功发送
          },
          fail: function(error) {
            console.error('❌ 消息发送失败:', error);
            // 如果是命令消息，标记发送完成以便处理下一个
            if (msg.type === 'companion_command') {
              this.globalData.commandSending = false;
            }
          }
        });
      } else {
        console.warn('⚠️ WebSocket未连接，无法发送消息');
        if (!this.globalData.connected && !this.globalData.connecting) {
          this.connectWebSocket();
        }
      }
    },
    
    // 处理机器人连接状态更新
    handleRobotConnectionStatus: function(data) {
      const isConnected = data.connected;
      const robotId = data.robot_id;
      
      // 更新全局状态
      if (robotId === this.globalData.robotId) {
        // 检查状态是否真的发生了变化
        const currentConnectionState = this.globalData.robotConnected || false;
        
        if (currentConnectionState !== isConnected) {
          // 状态改变时才输出日志
          console.log(`🔗 机器人连接状态变更: ${isConnected ? '已连接' : '已断开'}`);
          
          this.globalData.robotConnected = isConnected;
        }
        
        // 更新连接状态相关的UI显示
        if (isConnected) {
          this.globalData.signalStrength = '良好';
          
          // 如果之前断开连接，现在重新连接了，请求初始状态
          if (!this.globalData.videoStreamActive) {
            this.requestInitialStatus();
          }
        } else {
          this.globalData.signalStrength = '未连接';
          this.globalData.videoStreamActive = false;
          this.globalData.batteryLevel = 0;
        }
        
        // 通知相关页面更新状态
        if (this.globalData.controlPage) {
          this.globalData.controlPage.updateConnectionStatus(isConnected, robotId);
        }
        
        if (this.globalData.historyPage) {
          this.globalData.historyPage.updateConnectionStatus(isConnected, robotId);
        }
        
        if (this.globalData.featurePage) {
          this.globalData.featurePage.updateConnectionStatus(isConnected, robotId);
        }
      }
    },
    
    // 处理视频质量更新
    handleVideoQualityUpdate: function(data) {
      this.globalData.videoQuality = data.preset || this.globalData.videoQuality;
      
      // 解析分辨率字符串 (如 "640x480")
      if (data.resolution) {
        const resParts = data.resolution.split('x');
        if (resParts.length === 2) {
          this.globalData.videoResolution = {
            width: parseInt(resParts[0]),
            height: parseInt(resParts[1])
          };
        }
      }
      
      if (data.fps) {
        this.globalData.frameRate = data.fps;
      }
      
      // 通知控制页面更新视频质量显示
      if (this.globalData.controlPage) {
        this.globalData.controlPage.updateVideoQuality(data);
      }
      
      console.log('📹 视频质量调整为:', this.globalData.videoQuality);
    },
    
    // 处理质量调整请求已接收
    handleQualityRequestReceived: function(data) {
      // 显示请求已接收的提示
      if (this.globalData.controlPage) {
        this.globalData.controlPage.showQualityRequestReceived(data.preset);
      }
    },
    

    
    // 处理伴侣机器人断开连接
    handleCompanionDisconnected: function(data) {
      console.log('💔 伴侣机器人断开连接:', data.companion_id);
      
      // 重置相关状态
      this.globalData.videoStreamActive = false;
      this.globalData.signalStrength = '未连接';
      this.globalData.batteryLevel = 0;
      this.globalData.followingMode = 'idle';
      
      // 显示断开提示
      wx.showToast({
        title: '机器人连接已断开',
        icon: 'none',
        duration: 2000
      });
      
      // 通知所有页面更新状态
      if (this.globalData.controlPage) {
        this.globalData.controlPage.handleCompanionDisconnected(data);
      }
      
      if (this.globalData.historyPage) {
        this.globalData.historyPage.handleCompanionDisconnected(data);
      }
      
      if (this.globalData.featurePage) {
        this.globalData.featurePage.handleCompanionDisconnected(data);
      }
    },

    // 主动关闭连接
    closeConnection: function() {
      this.globalData.closedByUser = true;
      this.globalData.videoStreamActive = false;
      this.stopHeartbeat();
      
      if (this.globalData.reconnectTimer) {
        clearTimeout(this.globalData.reconnectTimer);
        this.globalData.reconnectTimer = null;
      }
      
      if (this.globalData.socketTask && this.globalData.connected) {
        this.globalData.socketTask.close({
          code: 1000,
          reason: '用户主动关闭'
        });
        this.globalData.connected = false;
      }
      
      console.log('🔌 机器人伴侣连接已主动关闭');
      
      // 3秒后重置状态
      setTimeout(() => {
        this.globalData.closedByUser = false;
      }, 3000);
    },
    
    // 获取连接状态
    getConnectionStatus: function() {
      return {
        connected: this.globalData.connected,
        connecting: this.globalData.connecting,
        videoActive: this.globalData.videoStreamActive,
        networkLatency: this.globalData.networkLatency,
        videoStats: this.globalData.videoStats
      };
    },
    
    // 获取机器人状态
    getRobotStatus: function() {
      return {
        batteryLevel: this.globalData.batteryLevel,
        signalStrength: this.globalData.signalStrength,
        companionMode: this.globalData.companionMode,
        followingMode: this.globalData.followingMode,
        emotionState: this.globalData.emotionState,
        interactionMode: this.globalData.interactionMode,
        position: this.globalData.robotPosition
      };
    },
    
    // 初始化性能优化定时器
    initPerformanceTimers: function() {
      // 数据缓冲处理定时器 - 进一步优化
      this.globalData.dataProcessingTimer = setInterval(() => {
        // 限制执行时间不超过10ms
        const maxExecutionTime = 10;
        const startTime = Date.now();
        
        try {
          // 轻量级检查，避免复杂操作
          if (this._featureDataBuffer && this._featureDataBuffer.length > 0) {
            // 延迟处理，避免阻塞
            setTimeout(() => {
              try {
                this._flushFeatureData && this._flushFeatureData();
              } catch (e) {
                // 静默处理错误
              }
            }, 0);
          }
          
          if (this._trackingDataBuffer && this._trackingDataBuffer.length > 0 && (Date.now() - startTime) < maxExecutionTime) {
            // 延迟处理，避免阻塞
            setTimeout(() => {
              try {
                this._flushTrackingData && this._flushTrackingData();
              } catch (e) {
                // 静默处理错误
              }
            }, 0);
          }
        } catch (error) {
          // 静默处理，避免日志过多
        }
      }, 5000); // 延长到5秒，进一步减少CPU占用
      
      // 性能监控定时器 - 大幅简化
      if (this.globalData.performanceMonitor) {
        this.globalData.performanceTimer = setInterval(() => {
          // 异步执行性能检查，避免阻塞主线程
          setTimeout(() => {
            try {
              this.performanceCheckLightweight();
            } catch (error) {
              // 静默处理错误
            }
          }, 0);
        }, 60000); // 延长到60秒，减少频率
      }
    },

    // 轻量级性能检查方法
    performanceCheckLightweight: function() {
      try {
        const videoStats = this.globalData.videoStats || {};
        
        // 只记录核心性能指标，避免复杂计算
        if (this.globalData.debugMode) {
          console.log('📊 轻量性能监控:', {
            connected: this.globalData.connected,
            videoFrames: videoStats.receivedFrames || 0,
            droppedFrames: videoStats.droppedFrames || 0
          });
        }
        
        // 简化的视频质量自动调整
        if (videoStats.receivedFrames > 50) {
          const dropRate = (videoStats.droppedFrames || 0) / videoStats.receivedFrames;
          if (dropRate > 0.3 && this.globalData.videoQuality !== 'low') {
            // 异步调整，避免阻塞
            setTimeout(() => {
              this.adjustVideoQuality('low');
            }, 100);
          }
        }
      } catch (error) {
        // 完全静默处理错误
      }
    },
    
    // 清理性能定时器
    clearPerformanceTimers: function() {
      if (this.globalData.dataProcessingTimer) {
        clearInterval(this.globalData.dataProcessingTimer);
        this.globalData.dataProcessingTimer = null;
      }
      
      if (this.globalData.performanceTimer) {
        clearInterval(this.globalData.performanceTimer);
        this.globalData.performanceTimer = null;
      }
    },

    // 性能检查方法（保留用于兼容性）
    performanceCheck: function() {
      // 重定向到轻量级版本
      this.performanceCheckLightweight();
    },

    // 自动调整视频质量
    adjustVideoQuality: function(quality) {
      try {
        if (this.globalData.videoQuality !== quality) {
          this.requestVideoQuality(quality);
          if (this.globalData.debugMode) {
            console.log('🎥 自动调整视频质量为:', quality);
          }
        }
      } catch (error) {
        console.error('❌ 自动调整视频质量失败:', error);
      }
    }
  });