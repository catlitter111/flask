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
      clientVersion: '2.0.0',
      networkLatency: 0,
      
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
      
      // WebRTC相关
      webrtcSupported: false,
      webrtcEnabled: false,
      webrtcConnected: false,
      useWebRTC: false,  // 是否使用WebRTC传输视频
      webrtcStreamUrl: '', // WebRTC视频流地址
      
      // 命令队列和状态
      commandQueue: [],
      commandSending: false,
      lastCommandTime: 0,
      commandCooldown: 500,
      
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
      console.log('🤖 机器人伴侣小程序启动');
      
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
                  console.log(`✅ ${permission} 权限获取成功`);
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
      
      // 检测WebRTC支持
      this.detectWebRTCSupport();
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
        
        // 发送初始化消息 - 修复 webrtcSupported 上下文问题
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
            voice_interaction: true,
            webrtc_support: that.globalData.webrtcSupported // 修复：使用 that 而不是 this
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
    
    // 消息分发到对应页面 - 优化版本，防止阻塞
    distributeMessage: function(data) {
      // 使用异步处理防止阻塞主线程，并增加超时处理
      setTimeout(() => {
        try {
          this._distributeMessageSync(data);
        } catch (error) {
          console.error('❌ 消息分发处理错误:', error);
        }
      }, 0);
    },
    
    // 同步消息分发处理 - 优化版本
    _distributeMessageSync: function(data) {
      if (!data || !data.type) {
        console.warn('⚠️ 收到无效消息数据');
        return;
      }

      const messageType = data.type;
      const startTime = Date.now();
      
      try {
        switch (messageType) {
          case 'robot_status_update':
            this.handleRobotStatusUpdate(data);
            break;
            
          case 'video_frame':
            // 视频帧数据 - 最高优先级，快速处理
            this.handleVideoFrame(data);
            break;
            
          case 'command_response':
            this.handleCommandResponse(data);
            break;
            
          case 'companion_status_update':
            this.handleCompanionStatusUpdate(data);
            break;
            
          case 'tracking_data':
            // 跟踪数据更新 - 异步处理以避免阻塞
            setTimeout(() => {
              if (this.globalData.historyPage) {
                this.globalData.historyPage.handleTrackingData(data);
              }
              this.saveTrackingData(data);
            }, 0);
            break;
            
          case 'file_upload_success':
            if (this.globalData.featurePage) {
              this.globalData.featurePage.handleFileUploadSuccess(data);
            }
            break;

          case 'file_save_result':
            if (this.globalData.featurePage) {
              this.globalData.featurePage.handleFileSaveResult(data);
            }
            break;

          case 'feature_extraction_result':
            // 特征提取结果（旧格式，保留兼容性）
            if (this.globalData.featurePage) {
              this.globalData.featurePage.handleFeatureResult(data);
            }
            // 异步保存数据以避免阻塞
            setTimeout(() => {
              this.saveFeatureData(data);
            }, 0);
            break;

          case 'feature_extraction_complete':
            // 特征提取完成（新格式）
            if (this.globalData.featurePage) {
              this.globalData.featurePage.handleFeatureResult(data);
            }
            if (data.status === 'success') {
              setTimeout(() => {
                this.saveFeatureData(data);
              }, 0);
            }
            break;

          case 'feature_extraction_error':
            if (this.globalData.featurePage) {
              this.globalData.featurePage.handleFeatureError(data);
            }
            break;

          case 'processed_image_notification':
            // 处理后图片通知
            if (this.globalData.featurePage) {
              this.globalData.featurePage.handleProcessedImageNotification(data);
            }
            // 异步保存数据
            setTimeout(() => {
              this.saveProcessedImageData(data);
            }, 0);
            break;
            
          case 'ai_response':
            if (this.globalData.aiAssistantPage) {
              this.globalData.aiAssistantPage.handleAIResponse(data);
            }
            break;
            
          case 'position_update':
            this.handlePositionUpdate(data);
            break;
            
          case 'interaction_event':
            this.handleInteractionEvent(data);
            break;
            
          case 'robot_connection_status':
            this.handleRobotConnectionStatus(data);
            break;
            
          case 'webrtc_available':
            this.handleWebRTCAvailable(data);
            break;
            
          case 'webrtc_answer':
            this.handleWebRTCAnswer(data);
            break;
            
          case 'webrtc_error':
            this.handleWebRTCError(data);
            break;
            
          case 'video_quality_update':
            this.handleVideoQualityUpdate(data);
            break;
            
          case 'quality_request_received':
            this.handleQualityRequestReceived(data);
            break;
            
          case 'server_welcome':
            console.log('🎉 服务器欢迎消息，版本:', data.server_version);
            break;
            
          case 'companion_connected':
            this.handleCompanionConnected(data);
            break;
            
          case 'companion_disconnected':
            this.handleCompanionDisconnected(data);
            break;
            
          case 'error':
            this.handleError(data);
            break;
            
          default:
            // 减少日志输出
            if (data.type !== 'ping' && data.type !== 'pong') {
              console.log('🔍 未知消息类型:', data.type);
            }
        }
      } catch (error) {
        console.error('❌ 消息处理错误:', error);
      }
      
      // 记录处理时间，警告耗时过长的操作
      const processingTime = Date.now() - startTime;
      if (processingTime > 50) {
        console.warn(`⚠️ 消息处理耗时过长: ${processingTime}ms, 类型: ${messageType}`);
      }
    },
    
    // 处理机器人状态更新
    handleRobotStatusUpdate: function(data) {
      this.globalData.batteryLevel = data.battery_level || this.globalData.batteryLevel;
      this.globalData.signalStrength = data.signal_strength || this.globalData.signalStrength;
      this.globalData.companionMode = data.mode || this.globalData.companionMode;
      
      if (this.globalData.controlPage) {
        this.globalData.controlPage.updateRobotStatus(data);
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
    
    // WebRTC功能检测
    detectWebRTCSupport: function() {
      // 微信小程序环境中，WebRTC支持有限
      // 这里我们检测是否有相关API可用
      try {
        if (typeof wx.createLivePlayerContext !== 'undefined') {
          this.globalData.webrtcSupported = true;
          console.log('📡 检测到WebRTC相关功能支持');
        } else {
          this.globalData.webrtcSupported = false;
          console.log('⚠️ 当前环境不支持WebRTC功能');
        }
      } catch (e) {
        this.globalData.webrtcSupported = false;
        console.log('⚠️ WebRTC功能检测失败:', e);
      }
    },
    
    // 启动命令队列处理器
    startCommandProcessor: function() {
      setInterval(() => {
        this.processNextCommand();
      }, 100);
    },
    
    // 处理下一个命令
    processNextCommand: function() {
      if (this.globalData.commandSending || this.globalData.commandQueue.length === 0) {
        return;
      }
      
      const now = Date.now();
      if (now - this.globalData.lastCommandTime < this.globalData.commandCooldown) {
        return;
      }
      
      const command = this.globalData.commandQueue.shift();
      this.globalData.commandSending = true;
      this.globalData.lastCommandTime = now;
      
      this.sendSocketMessage(command);
    },
    
    // 发送WebSocket消息
    sendSocketMessage: function(message) {
      if (!this.globalData.connected || !this.globalData.socketTask) {
        console.warn('⚠️ WebSocket未连接，消息发送失败');
        return false;
      }
      
      try {
        this.globalData.socketTask.send({
          data: JSON.stringify(message)
        });
        return true;
      } catch (error) {
        console.error('❌ 消息发送失败:', error);
        return false;
      }
    },
    
    // 启动心跳
    startHeartbeat: function() {
      this.stopHeartbeat(); // 先停止现有的心跳
      
      this.globalData.heartbeatTimer = setInterval(() => {
        if (this.globalData.connected) {
          this.sendSocketMessage({
            type: 'ping',
            timestamp: Date.now()
          });
        }
      }, 18000); // 18秒发送一次心跳，与服务器协调
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
      if (this.globalData.closedByUser) {
        return;
      }
      
      this.globalData.reconnectAttempts++;
      const delay = Math.min(
        this.globalData.reconnectDelay * Math.pow(2, this.globalData.reconnectAttempts - 1),
        this.globalData.maxReconnectDelay
      );
      
      console.log(`🔄 将在 ${delay}ms 后尝试重连 (第${this.globalData.reconnectAttempts}次)`);
      
      this.globalData.reconnectTimer = setTimeout(() => {
        this.connectWebSocket();
      }, delay);
    },
    
    // 初始化性能优化定时器
    initPerformanceTimers: function() {
      // 定期刷新缓冲数据（每5秒，减少频率）
      setInterval(() => {
        // 检查是否有数据需要刷新，避免无意义的调用
        if (this._featureDataBuffer && this._featureDataBuffer.length > 0) {
          this._flushFeatureData();
        }
        if (this._trackingDataBuffer && this._trackingDataBuffer.length > 0) {
          this._flushTrackingData();
        }
      }, 5000);
    },
    
    // 保存跟踪数据 - 优化版本
    saveTrackingData: function(data) {
      // 节流：只每10条记录保存一次
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
      
      // 批量处理：每10条数据处理一次
      if (this._trackingDataBuffer.length >= 10) {
        this._flushTrackingData();
      }
    },
    
    // 批量刷新跟踪数据
    _flushTrackingData: function() {
      if (!this._trackingDataBuffer || this._trackingDataBuffer.length === 0) {
        return;
      }
      
      this.globalData.trackingHistory.push(...this._trackingDataBuffer);
      this._trackingDataBuffer = [];
      
      // 限制历史记录数量
      if (this.globalData.trackingHistory.length > 1000) {
        this.globalData.trackingHistory = this.globalData.trackingHistory.slice(-800);
      }
      
      // 异步保存到本地存储
      wx.nextTick(() => {
        wx.setStorageSync('companionHistory', this.globalData.trackingHistory);
      });
    },
    
    // 保存特征数据 - 优化版本
    saveFeatureData: function(data) {
      // 节流：批量处理特征数据
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
      
      // 批量处理：每5条数据处理一次
      if (this._featureDataBuffer.length >= 5) {
        this._flushFeatureData();
      }
    },
    
    // 批量刷新特征数据
    _flushFeatureData: function() {
      if (!this._featureDataBuffer || this._featureDataBuffer.length === 0) {
        return;
      }
      
      this.globalData.extractedFeatures.push(...this._featureDataBuffer);
      this._featureDataBuffer = [];
      
      // 限制数据量
      if (this.globalData.extractedFeatures.length > 500) {
        this.globalData.extractedFeatures = this.globalData.extractedFeatures.slice(-400);
      }
      
      // 异步保存到本地存储
      wx.nextTick(() => {
        wx.setStorageSync('extractedFeatures', this.globalData.extractedFeatures);
      });
    },
    
    // 添加命令到队列
    addCommand: function(command) {
      this.globalData.commandQueue.push(command);
    },
    
    // 处理其他消息类型的空方法（避免报错）
    handleCompanionStatusUpdate: function(data) {
      this.globalData.followingMode = data.following_mode || this.globalData.followingMode;
      this.globalData.emotionState = data.emotion_state || this.globalData.emotionState;
      this.globalData.interactionMode = data.interaction_mode !== undefined ? 
        data.interaction_mode : this.globalData.interactionMode;
      
      if (this.globalData.controlPage) {
        this.globalData.controlPage.handleCompanionStatusUpdate(data);
      }
    },
    
    handlePositionUpdate: function(data) {
      this.globalData.robotPosition = data.robot_position || this.globalData.robotPosition;
      this.globalData.targetPosition = data.target_position || this.globalData.targetPosition;
      
      if (this.globalData.historyPage) {
        this.globalData.historyPage.handlePositionUpdate(data);
      }
    },
    
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
    
    handleRobotConnectionStatus: function(data) {
      console.log('🔗 机器人连接状态:', data.connected ? '已连接' : '已断开');
      
      if (this.globalData.controlPage) {
        this.globalData.controlPage.handleRobotConnectionStatus(data);
      }
    },
    
    handleWebRTCAvailable: function(data) {
      console.log('📡 收到WebRTC可用通知:', data.robot_id);
      console.log('📡 视频流地址:', data.video_stream_url);
      
      if (this.globalData.webrtcSupported && !this.globalData.webrtcConnected) {
        // 尝试建立WebRTC连接
        this.initWebRTCConnection(data.robot_id, data.video_stream_url);
      }
    },
    
    initWebRTCConnection: function(robotId, videoStreamUrl) {
      console.log('📡 正在初始化WebRTC连接...');
      console.log('📡 使用视频流地址:', videoStreamUrl);
      
      // 存储视频流地址
      this.globalData.webrtcStreamUrl = videoStreamUrl;
      
      // 在微信小程序中，我们直接使用视频流地址
      // 标记WebRTC为已连接状态
      this.globalData.webrtcConnected = true;
      this.globalData.useWebRTC = true;
      
      // 通知控制页面切换到WebRTC模式
      if (this.globalData.controlPage) {
        this.globalData.controlPage.switchToWebRTC({
          video_stream_url: videoStreamUrl,
          robot_id: robotId
        });
      }
      
      console.log('✅ WebRTC连接已建立');
    },
    
    handleWebRTCAnswer: function(data) {
      console.log('📡 收到WebRTC答案');
      
      // 在微信小程序中，我们标记WebRTC为可用状态
      this.globalData.webrtcConnected = true;
      this.globalData.useWebRTC = true;
      
      // 通知控制页面切换到WebRTC模式
      if (this.globalData.controlPage) {
        this.globalData.controlPage.switchToWebRTC(data);
      }
      
      console.log('✅ WebRTC连接已建立');
    },
    
    handleWebRTCError: function(data) {
      console.error('❌ WebRTC错误:', data.error);
      
      this.globalData.webrtcConnected = false;
      this.globalData.useWebRTC = false;
      
      // 降级到WebSocket视频传输
      if (this.globalData.controlPage) {
        this.globalData.controlPage.fallbackToWebSocket();
      }
    },
    
    handleVideoQualityUpdate: function(data) {
      this.globalData.videoQuality = data.preset || this.globalData.videoQuality;
      this.globalData.videoResolution = {
        width: parseInt(data.resolution.split('x')[0]) || this.globalData.videoResolution.width,
        height: parseInt(data.resolution.split('x')[1]) || this.globalData.videoResolution.height
      };
      this.globalData.frameRate = data.fps || this.globalData.frameRate;
      
      console.log('📹 视频质量调整为:', this.globalData.videoQuality);
    },
    
    handleQualityRequestReceived: function(data) {
      // 显示请求已接收的提示
      if (this.globalData.controlPage) {
        this.globalData.controlPage.showQualityRequestReceived(data.preset);
      }
    },
    
    handleCompanionConnected: function(data) {
      console.log('💝 伴侣机器人已连接:', data.companion_id);
      
      // 通知所有页面更新状态
      if (this.globalData.controlPage) {
        this.globalData.controlPage.handleCompanionConnected(data);
      }
    },
    
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
    },
    
    handleError: function(data) {
      console.error('🚨 服务器错误:', data);
      wx.showToast({
        title: data.message || '发生错误',
        icon: 'none',
        duration: 3000
      });
    },
    
    saveProcessedImageData: function(data) {
      // 保存处理后的图片数据
      console.log('📷 保存处理后图片数据:', data.extraction_id);
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
    }
  }); 