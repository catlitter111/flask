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
      robotId: 'companion_robot_123',
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
    },
  
    onLoad: function(options) {
      // 生成唯一的客户端ID
      const app = getApp();
      
      this.setData({
        clientId: app.globalData.clientId || `user_for_${this.data.robotId}_${Date.now()}`
      });
      
      // 注册到全局应用以接收消息
      app.globalData.controlPage = this;
      
      // 获取系统信息，适配不同屏幕
      const systemInfo = wx.getSystemInfoSync();
      const statusBarHeight = systemInfo.statusBarHeight || 20;
      const navBarHeight = 44; // 导航栏标准高度
      
      // 计算可用高度
      const windowHeight = systemInfo.windowHeight;
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
        if (lastFrameAge > that.data.videoExpireTimeout && that.data.robotConnected) {
          // 如果之前视频未过期，则发出警告
          if (!that.data.videoExpired) {
            console.warn('视频帧已过期，最后一帧接收时间:', new Date(that.data.lastFrameReceived));
            that.setData({
              videoExpired: true
            });
          }
          
          // 视频流将自动恢复
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
              statusText = '视频超时';
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
      
      // 每秒计算一次FPS
      this.fpsTimer = setInterval(function() {
        const fps = that.data.fpsCounter;
        that.setData({
          framesPerSecond: fps,
          fpsCounter: 0
        });
      }, 1000);
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
          
          // 延迟更新
          setTimeout(() => {
            if (this.data.pendingImageData) {
              this.updateVideoFrame(this.data.pendingImageData);
              this.data.pendingImageData = null;
            }
            this.data.imageUpdatePending = false;
          }, this.data.minImageUpdateInterval - timeSinceLastUpdate);
        }
      }
    },
    
    // 实际更新视频帧的方法
    updateVideoFrame: function(data) {
      const now = Date.now();
      
      // 计算服务器到客户端延迟
      if (data.server_timestamp) {
        const serverToClientDelay = now - data.server_timestamp;
        this.setData({
          serverToClientDelay: serverToClientDelay
        });
      }
      
      // 检查帧序号，计算丢帧
      if (data.sequence) {
        if (this.data.expectedSequence > 0 && data.sequence > this.data.expectedSequence) {
          const dropped = data.sequence - this.data.expectedSequence;
          this.setData({
            droppedFrames: this.data.droppedFrames + dropped
          });
        }
        this.setData({
          frameSequence: data.sequence,
          expectedSequence: data.sequence + 1
        });
      }
      
      // 更新视频质量信息
      if (data.resolution) {
        this.setData({
          videoResolution: data.resolution
        });
      }
      
      // 批量更新数据，减少渲染次数
      const updateData = {
        videoBase64: `data:image/jpeg;base64,${data.frame_data || data.data}`,
        receivedFrames: this.data.receivedFrames + 1,
        lastImageUpdateTime: now
      };
      
      // 计算帧延迟
      if (data.timestamp) {
        const frameLatency = now - data.timestamp;
        
        // 保存最近10帧的时间戳
        const timestamps = this.data.frameTimestamps.slice(-9);
        timestamps.push(data.timestamp);
        
        // 计算帧抖动
        let jitter = 0;
        if (timestamps.length > 1) {
          const intervals = [];
          for (let i = 1; i < timestamps.length; i++) {
            intervals.push(timestamps[i] - timestamps[i-1]);
          }
          
          const mean = intervals.reduce((a, b) => a + b, 0) / intervals.length;
          const squaredDiffs = intervals.map(x => Math.pow(x - mean, 2));
          jitter = Math.sqrt(squaredDiffs.reduce((a, b) => a + b, 0) / intervals.length);
        }
        
        const networkDelay = this.data.serverToClientDelay + (frameLatency - this.data.serverToClientDelay);
        
        updateData.frameLatency = frameLatency;
        updateData.networkDelay = networkDelay;
        updateData.frameJitter = Math.round(jitter);
        updateData.lastFrameTimestamp = data.timestamp;
        updateData.frameTimestamps = timestamps;
      }
      
      // 计算FPS
      updateData.fpsCounter = this.data.fpsCounter + 1;
      
      // 计算缓冲健康度
      const health = this.calculateBufferHealth();
      updateData.bufferHealth = Math.round(health);
      
      // 一次性更新所有数据
      this.setData(updateData);
      
      // 每5秒发送一次状态更新
      if (now - this.data.lastStatusUpdateTime > this.data.statusUpdateInterval) {
        this.sendStatusUpdate(health);
        this.setData({
          lastStatusUpdateTime: now
        });
      }
      
      // 接收到视频帧表示机器人已连接
      if (!this.data.robotConnected) {
        this.setData({
          robotConnected: true,
          reconnectingRobot: false,
          signalStrength: '已连接',
          connectionStatusText: '伴侣在线',
          lastRobotStatusTime: now
        });
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
          connectionStatusText: '伴侣在线'
        });
        
        // 视频流将自动恢复
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
          this.setData({
            signalStrength: '良好',
            connectionStatusText: '已连接'
          });
          
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
    }
  });