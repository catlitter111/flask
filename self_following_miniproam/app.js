// app.js - 机器人伴侣(自跟随小车)微信小程序
App({
    globalData: {
      userInfo: null,
      socketTask: null,
      connected: false,
      connecting: false,
      serverUrl: "ws://192.168.1.100:1234/ws/companion/", // 机器人伴侣服务器地址
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
      targetPersonId: null,
      batteryLevel: 0,
      signalStrength: '未连接',
      followDistance: 1.5, // 跟随距离(米)
      followSpeed: 0.8, // 跟随速度(m/s)
      
      // 特征数据
      extractedFeatures: [],
      currentTarget: null,
      
      // 位置和导航
      robotPosition: { x: 0, y: 0 },
      targetPosition: { x: 0, y: 0 },
      trackingHistory: []
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
    },
    
    // 获取用户权限（相机、位置等）
    getUserPermissions: function() {
      // 获取相机权限（用于特征提取）
      wx.getSetting({
        success: (res) => {
          if (!res.authSetting['scope.camera']) {
            wx.authorize({
              scope: 'scope.camera',
              success: () => {
                console.log('📷 相机权限获取成功');
              },
              fail: (error) => {
                console.error('📷 相机权限获取失败', error);
              }
            });
          }
        }
      });

      // 获取位置权限（用于跟踪历史）
      wx.getSetting({
        success: (res) => {
          if (!res.authSetting['scope.userLocation']) {
            wx.authorize({
              scope: 'scope.userLocation',
              success: () => {
                console.log('📍 位置权限获取成功');
              },
              fail: (error) => {
                console.error('📍 位置权限获取失败', error);
              }
            });
          }
        }
      });
    },
    
    // 初始化本地存储
    initLocalStorage: function() {
      // 初始化跟踪历史
      const trackingHistory = wx.getStorageSync('trackingHistory') || [];
      this.globalData.trackingHistory = trackingHistory;
      
      // 初始化特征数据
      const extractedFeatures = wx.getStorageSync('extractedFeatures') || [];
      this.globalData.extractedFeatures = extractedFeatures;
      
      console.log('💾 本地存储初始化完成');
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
          type: 'init',
          robot_id: that.globalData.robotId,
          client_version: that.globalData.clientVersion,
          connection_time: Date.now(),
          device_type: 'companion_client'
        });
        
        // 启动心跳检测
        that.startHeartbeat();
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
        that.stopHeartbeat();
        that.scheduleReconnect();
      });
      
      // 监听连接关闭
      this.globalData.socketTask.onClose(function(event) {
        console.log('🔌 WebSocket连接已关闭, 代码:', event.code, '原因:', event.reason);
        that.globalData.connected = false;
        that.globalData.connecting = false;
        that.stopHeartbeat();
        
        if (!that.globalData.closedByUser) {
          that.scheduleReconnect();
        }
      });
    },
    
    // 消息分发到对应页面
    distributeMessage: function(data) {
      switch (data.type) {
        case 'robot_status_update':
          // 机器人状态更新
          if (this.globalData.controlPage) {
            this.globalData.controlPage.handleRobotStatus(data);
          }
          break;
          
        case 'following_status_update':
          // 跟随状态更新
          this.globalData.followingMode = data.mode || 'idle';
          if (this.globalData.controlPage) {
            this.globalData.controlPage.handleFollowingStatus(data);
          }
          break;
          
        case 'tracking_data':
          // 跟踪数据更新
          if (this.globalData.historyPage) {
            this.globalData.historyPage.handleTrackingData(data);
          }
          break;
          
        case 'feature_extraction_result':
          // 特征提取结果
          if (this.globalData.featurePage) {
            this.globalData.featurePage.handleFeatureResult(data);
          }
          break;
          
        case 'ai_response':
          // AI助手回复
          if (this.globalData.aiAssistantPage) {
            this.globalData.aiAssistantPage.handleAIResponse(data);
          }
          break;
          
        case 'position_update':
          // 位置更新
          this.globalData.robotPosition = data.robot_position || this.globalData.robotPosition;
          this.globalData.targetPosition = data.target_position || this.globalData.targetPosition;
          
          if (this.globalData.historyPage) {
            this.globalData.historyPage.handlePositionUpdate(data);
          }
          break;
          
        default:
          console.log('🔍 未知消息类型:', data.type);
      }
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
            timestamp: now
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
          }
        });
      } else {
        console.warn('⚠️ WebSocket未连接，无法发送消息');
        if (!this.globalData.connected && !this.globalData.connecting) {
          this.connectWebSocket();
        }
      }
    },
    
    // 主动关闭连接
    closeConnection: function() {
      this.globalData.closedByUser = true;
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
    }
  });
