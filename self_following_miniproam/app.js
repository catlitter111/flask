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
          
          // 处理心跳响应
          if (data.type === 'pong') {
            that.globalData.lastPongTime = now;
            if (data.echo_timestamp) {
              that.globalData.networkLatency = now - data.echo_timestamp;
            }
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
    
    // 消息分发到对应页面
    distributeMessage: function(data) {
      switch (data.type) {
        case 'robot_status_update':
          // 机器人状态更新
          this.handleRobotStatusUpdate(data);
          break;
          
        case 'video_frame':
          // 视频帧数据
          this.handleVideoFrame(data);
          break;
          
        case 'command_response':
          // 命令执行响应
          this.handleCommandResponse(data);
          break;
          
        case 'companion_status_update':
          // 伴侣状态更新
          this.handleCompanionStatusUpdate(data);
          break;
          
        case 'tracking_data':
          // 跟踪数据更新
          if (this.globalData.historyPage) {
            this.globalData.historyPage.handleTrackingData(data);
          }
          this.saveTrackingData(data);
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
          if (this.globalData.featurePage) {
            this.globalData.featurePage.handleFeatureResult(data);
          }
          this.saveFeatureData(data);
          break;

        case 'feature_extraction_complete':
          // 特征提取完成（新格式）
          if (this.globalData.featurePage) {
            this.globalData.featurePage.handleFeatureResult(data);
          }
          if (data.status === 'success') {
            this.saveFeatureData(data);
          }
          break;

        case 'feature_extraction_error':
          // 特征提取错误
          if (this.globalData.featurePage) {
            this.globalData.featurePage.handleFeatureError(data);
          }
          break;

        case 'processed_image_notification':
          // 处理后图片通知
          if (this.globalData.featurePage) {
            this.globalData.featurePage.handleProcessedImageNotification(data);
          }
          this.saveProcessedImageData(data);
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
          console.log('🔍 未知消息类型:', data.type);
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
    
    // 处理视频帧
    handleVideoFrame: function(data) {
      this.globalData.videoStreamActive = true;
      this.globalData.lastVideoFrame = data;
      
      // 更新视频统计
      this.globalData.videoStats.receivedFrames++;
      if (data.sequence && this.globalData.lastFrameSequence) {
        const expectedSeq = this.globalData.lastFrameSequence + 1;
        if (data.sequence > expectedSeq) {
          this.globalData.videoStats.droppedFrames += (data.sequence - expectedSeq);
        }
      }
      this.globalData.lastFrameSequence = data.sequence;
      
      // 分发到控制页面
      if (this.globalData.controlPage) {
        this.globalData.controlPage.handleVideoFrame(data);
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
    
    // 保存跟踪数据
    saveTrackingData: function(data) {
      const trackingEntry = {
        timestamp: Date.now(),
        robot_position: data.robot_position,
        target_position: data.target_position,
        following_mode: data.following_mode,
        distance: data.distance
      };
      
      this.globalData.trackingHistory.push(trackingEntry);
      
      // 限制历史记录数量
      if (this.globalData.trackingHistory.length > 1000) {
        this.globalData.trackingHistory = this.globalData.trackingHistory.slice(-800);
      }
      
      // 保存到本地存储
      wx.setStorageSync('companionHistory', this.globalData.trackingHistory);
    },
    
    // 保存特征数据
    saveFeatureData: function(data) {
      const featureEntry = {
        id: data.feature_id || Date.now(),
        timestamp: Date.now(),
        features: data.features,
        person_id: data.person_id,
        confidence: data.confidence,
        image_data: data.image_data
      };
      
      this.globalData.extractedFeatures.push(featureEntry);
      
      // 限制特征数据数量
      if (this.globalData.extractedFeatures.length > 100) {
        this.globalData.extractedFeatures = this.globalData.extractedFeatures.slice(-80);
      }
      
      // 保存到本地存储
      wx.setStorageSync('extractedFeatures', this.globalData.extractedFeatures);
    },

    // 保存处理后图片数据
    saveProcessedImageData: function(data) {
      const processedImageEntry = {
        id: data.extraction_id || Date.now(),
        timestamp: Date.now(),
        original_image: data.original_image,
        processed_image: data.processed_image,
        result_image: data.result_image,
        features: data.features,
        colors: data.colors,
        proportions: data.proportions,
        status: 'success',
        extraction_type: 'processed_image'
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
      
      console.log('💾 保存处理后图片数据:', data.extraction_id);
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
      const timestamp = data.timestamp;
      
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
    }
  });