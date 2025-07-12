// pages/rfid/rfid.js - RFID实时监控页面
const app = getApp();

Page({
    data: {
      // 连接状态
      connected: false,
      robotConnected: false,
      readerAddress: '192.168.0.178:4001',
      
      // 扫描状态
      isScanning: false,
      runningTime: '00:00:00',
      readRate: 0,
      
      // 天线配置
      selectedAntenna: 1,
      
      // 统计数据
      totalTags: 0,
      newTagsCount: 0,
      totalReads: 0,
      avgReadsPerTag: '0.0',
      avgRSSI: -70,
      
      // 标签列表
      tagList: [],
      
      // 定时器
      runningTimer: null,
      startTime: null,
      
      // RFID状态
      rfidStatus: {
        connected: false,
        inventory_active: false,
        session_duration: 0.0,
        last_update: 0
      },
      
      // 消息处理
      lastMessageTime: 0,
      messageCount: 0
    },
  
    onLoad: function(options) {
      console.log('📡 RFID页面加载');
      
      // 注册页面到全局app，用于接收WebSocket消息
      app.globalData.rfidPage = this;
      
      // 初始化WebSocket连接
      this.initRFIDConnection();
      
      // 恢复上次的配置
      this.loadSettings();
      
      // 检查连接状态
      this.checkConnectionStatus();
    },
  
    onUnload: function() {
      console.log('📡 RFID页面卸载');
      
      // 清理定时器
      if (this.data.runningTimer) {
        clearInterval(this.data.runningTimer);
      }
      
      // 停止盘存（如果正在运行）
      if (this.data.isScanning) {
        this.sendRfidCommand('stop_inventory');
      }
      
      // 从全局app中移除页面引用
      if (app.globalData.rfidPage === this) {
        app.globalData.rfidPage = null;
      }
    },
  
    // 检查连接状态
    checkConnectionStatus: function() {
      const connectionStatus = app.getConnectionStatus();
      const robotStatus = app.getRobotStatus();
      
      this.setData({
        connected: connectionStatus.connected,
        robotConnected: connectionStatus.connected
      });
      
      console.log('📡 RFID页面连接状态:', {
        websocket: connectionStatus.connected,
        robot: connectionStatus.connected
      });
    },
    
    // 初始化RFID连接
    initRFIDConnection: function() {
      console.log('📡 初始化RFID WebSocket连接...');
      
      // 检查WebSocket连接状态
      if (!app.globalData.connected) {
        console.log('⚠️ WebSocket未连接，尝试重新连接...');
        app.connectWebSocket();
        
        // 等待连接建立后再检查状态
        setTimeout(() => {
          this.checkConnectionStatus();
        }, 2000);
      } else {
        // 已连接，更新状态
        this.setData({
          connected: true,
          robotConnected: true
        });
        
        // 请求RFID初始状态
        this.requestRfidStatus();
      }
    },
  
    // 请求RFID状态
    requestRfidStatus: function() {
      this.sendRfidCommand('get_rfid_status');
    },
    
    // 发送RFID命令
    sendRfidCommand: function(command, params = {}) {
      if (!app.globalData.connected) {
        wx.showToast({
          title: '请先连接到机器人',
          icon: 'none',
          duration: 2000
        });
        return;
      }
      
      const rfidCommand = {
        type: 'rfid_command',
        robot_id: app.globalData.robotId,
        command: command,
        params: params,
        timestamp: Date.now()
      };
      
      console.log('📡🔥 发送RFID命令:', command, params);
      app.sendSocketMessage(rfidCommand);
    },
  
    // 加载设置
    loadSettings: function() {
      const settings = wx.getStorageSync('rfidSettings') || {};
      if (settings.selectedAntenna) {
        this.setData({
          selectedAntenna: settings.selectedAntenna
        });
      }
      console.log('📡 加载RFID设置:', settings);
    },
  
    // 切换扫描状态
    toggleScanning: function() {
      if (this.data.isScanning) {
        this.stopScanning();
      } else {
        this.startScanning();
      }
    },
  
    // 开始扫描
    startScanning: function() {
      if (!this.data.connected) {
        wx.showToast({
          title: '请先连接到机器人',
          icon: 'none'
        });
        return;
      }
  
      console.log('📡🔥 开始RFID盘存');
      
      // 发送开始盘存命令
      this.sendRfidCommand('start_inventory');
      
      this.setData({
        isScanning: true,
        startTime: Date.now(),
        newTagsCount: 0  // 重置新标签计数
      });
  
      // 启动计时器
      this.startRunningTimer();
  
      wx.showToast({
        title: '开始RFID盘存',
        icon: 'success'
      });
    },
  
    // 停止扫描
    stopScanning: function() {
      console.log('📡🔥 停止RFID盘存');
      
      // 发送停止盘存命令
      this.sendRfidCommand('stop_inventory');
      
      this.setData({
        isScanning: false
      });
  
      // 停止计时器
      if (this.data.runningTimer) {
        clearInterval(this.data.runningTimer);
        this.data.runningTimer = null;
      }
  
      wx.showToast({
        title: '已停止RFID盘存',
        icon: 'none'
      });
    },
  
    // 启动运行时间计时器
    startRunningTimer: function() {
      this.data.runningTimer = setInterval(() => {
        const elapsed = Date.now() - this.data.startTime;
        const hours = Math.floor(elapsed / 3600000);
        const minutes = Math.floor((elapsed % 3600000) / 60000);
        const seconds = Math.floor((elapsed % 60000) / 1000);
        
        this.setData({
          runningTime: `${String(hours).padStart(2, '0')}:${String(minutes).padStart(2, '0')}:${String(seconds).padStart(2, '0')}`
        });
      }, 1000);
    },
  
    // ==== WebSocket消息处理方法 ====
    
    // 处理RFID标签数据
    handleRfidTagsData: function(data) {
      console.log('📡 收到RFID标签数据:', data);
      
      const rfidData = data.data || {};
      const tags = rfidData.tags || [];
      
      // 更新统计数据
      this.setData({
        totalTags: rfidData.total_tags || 0,
        totalReads: rfidData.total_reads || 0,
        avgReadsPerTag: rfidData.total_tags > 0 ? 
          (rfidData.total_reads / rfidData.total_tags).toFixed(1) : '0.0'
      });
      
      // 处理标签列表
      const formattedTags = tags.map(tag => ({
        epc: tag.epc || 'Unknown',
        rssi: tag.rssi_dbm || -99,
        readCount: tag.read_count || 0,
        antenna: tag.antenna_id || 1,
        lastSeen: this.formatTime(tag.last_seen),
        isActive: true,
        signalQuality: tag.signal_quality || '未知'
      }));
      
      // 检查新标签
      const currentEpcs = this.data.tagList.map(tag => tag.epc);
      const newTags = formattedTags.filter(tag => !currentEpcs.includes(tag.epc));
      
      if (newTags.length > 0) {
        this.setData({
          newTagsCount: this.data.newTagsCount + newTags.length
        });
      }
      
      // 更新标签列表
      this.setData({
        tagList: formattedTags
      });
      
      // 计算平均RSSI
      if (formattedTags.length > 0) {
        const avgRSSI = Math.round(
          formattedTags.reduce((sum, tag) => sum + tag.rssi, 0) / formattedTags.length
        );
        this.setData({
          avgRSSI: avgRSSI
        });
      }
    },
    
    // 处理RFID状态更新
    handleRfidStatusUpdate: function(data) {
      console.log('📊 收到RFID状态更新:', data);
      
      const statusData = data.data || {};
      const readerInfo = statusData.reader_info || {};
      const statistics = statusData.statistics || {};
      
      this.setData({
        rfidStatus: {
          connected: statusData.connected || false,
          inventory_active: statusData.inventory_active || false,
          session_duration: statusData.session_info?.duration || 0,
          last_update: Date.now()
        },
        readRate: statistics.read_rate || 0,
        readerAddress: `${readerInfo.ip || '192.168.0.178'}:${readerInfo.port || 4001}`,
        selectedAntenna: readerInfo.antenna_id || this.data.selectedAntenna
      });
      
      // 如果盘存状态发生变化，同步UI状态
      if (statusData.inventory_active !== this.data.isScanning) {
        this.setData({
          isScanning: statusData.inventory_active
        });
        
        if (statusData.inventory_active && !this.data.runningTimer) {
          this.setData({ startTime: Date.now() });
          this.startRunningTimer();
        } else if (!statusData.inventory_active && this.data.runningTimer) {
          clearInterval(this.data.runningTimer);
          this.data.runningTimer = null;
        }
      }
    },
    
    // 处理单个RFID标签检测
    handleRfidTagDetected: function(data) {
      console.log('🆕 收到新标签检测:', data);
      
      const tagData = data.data || {};
      const newTag = {
        epc: tagData.epc || 'Unknown',
        rssi: tagData.rssi_dbm || -99,
        readCount: tagData.read_count || 1,
        antenna: tagData.antenna_id || 1,
        lastSeen: this.formatTime(tagData.detection_time),
        isActive: true,
        signalQuality: tagData.signal_quality || '未知'
      };
      
      // 检查是否是新标签
      const existingTag = this.data.tagList.find(tag => tag.epc === newTag.epc);
      if (!existingTag) {
        // 新标签，添加到列表顶部
        this.setData({
          tagList: [newTag, ...this.data.tagList],
          totalTags: this.data.totalTags + 1,
          newTagsCount: this.data.newTagsCount + 1
        });
        
        // 显示新标签提示
        wx.showToast({
          title: `检测到新标签`,
          icon: 'none',
          duration: 1500
        });
      } else {
        // 更新现有标签
        const updatedList = this.data.tagList.map(tag => 
          tag.epc === newTag.epc ? newTag : tag
        );
        this.setData({
          tagList: updatedList
        });
      }
    },
    
    // 处理RFID命令响应
    handleRfidCommandResponse: function(data) {
      console.log('⚡ 收到RFID命令响应:', data);
      
      const command = data.command || 'unknown';
      const status = data.status || 'unknown';
      const message = data.message || '';
      
      if (status === 'success') {
        console.log(`✅ RFID命令成功: ${command} - ${message}`);
        
        // 根据命令类型处理响应
        switch (command) {
          case 'start_inventory':
            wx.showToast({
              title: 'RFID盘存已开始',
              icon: 'success'
            });
            break;
          case 'stop_inventory':
            wx.showToast({
              title: 'RFID盘存已停止',
              icon: 'none'
            });
            break;
          case 'set_antenna':
            // 天线切换成功，已在selectAntenna中显示提示
            break;
          case 'get_rfid_status':
            // 状态获取成功，不需要特殊处理
            break;
        }
        
        // 更新RFID状态
        if (data.rfid_status) {
          this.setData({
            rfidStatus: {
              connected: data.rfid_status.connected || false,
              inventory_active: command === 'start_inventory',
              session_duration: data.rfid_status.session_duration || 0,
              last_update: Date.now()
            }
          });
        }
      } else {
        const error = data.error || '未知错误';
        console.error(`❌ RFID命令失败: ${command} - ${error}`);
        
        wx.showToast({
          title: `${command}失败: ${error}`,
          icon: 'none',
          duration: 2000
        });
      }
    },
    
    // 更新连接状态
    updateConnectionStatus: function(connected, robotId) {
      console.log('📡 RFID页面连接状态更新:', connected, robotId);
      
      this.setData({
        connected: connected,
        robotConnected: connected
      });
      
      if (connected) {
        // 重新连接后，请求RFID状态
        setTimeout(() => {
          this.requestRfidStatus();
        }, 1000);
      } else {
        // 断开连接，重置状态
        this.setData({
          rfidStatus: {
            connected: false,
            inventory_active: false,
            session_duration: 0,
            last_update: 0
          },
          isScanning: false,
          readRate: 0
        });
        
        // 停止计时器
        if (this.data.runningTimer) {
          clearInterval(this.data.runningTimer);
          this.data.runningTimer = null;
        }
      }
    },
    
    // 格式化时间
    formatTime: function(timeData) {
      if (!timeData) {
        return new Date().toLocaleTimeString('zh-CN', { hour12: false });
      }
      
      if (typeof timeData === 'object' && timeData.sec) {
        // ROS时间格式
        const date = new Date(timeData.sec * 1000);
        return date.toLocaleTimeString('zh-CN', { hour12: false });
      }
      
      if (typeof timeData === 'number') {
        // 时间戳
        const date = new Date(timeData);
        return date.toLocaleTimeString('zh-CN', { hour12: false });
      }
      
      return timeData.toString();
    },
  
    // 选择天线
    selectAntenna: function(e) {
      const antenna = parseInt(e.currentTarget.dataset.antenna);
      this.setData({
        selectedAntenna: antenna
      });
  
      // 发送天线切换指令到机器人
      this.sendRfidCommand('set_antenna', { antenna_id: antenna });
      console.log('📡🔥 切换到天线', antenna);
  
      // 保存设置
      wx.setStorageSync('rfidSettings', {
        selectedAntenna: antenna
      });
  
      wx.showToast({
        title: `已切换到天线${antenna}`,
        icon: 'none',
        duration: 1000
      });
    },
  
    // 清空数据
    clearData: function() {
      wx.showModal({
        title: '确认清空',
        content: '是否清空所有标签数据？此操作不可恢复。',
        success: (res) => {
          if (res.confirm) {
            this.setData({
              tagList: [],
              totalTags: 0,
              newTagsCount: 0,
              totalReads: 0,
              avgReadsPerTag: '0.0',
              avgRSSI: -70
            });
  
            wx.showToast({
              title: '数据已清空',
              icon: 'success'
            });
          }
        }
      });
    },
  
    // 导出数据
    exportData: function() {
      if (this.data.tagList.length === 0) {
        wx.showToast({
          title: '暂无数据可导出',
          icon: 'none'
        });
        return;
      }
  
      // TODO: 实现数据导出功能
      wx.showModal({
        title: '导出数据',
        content: '功能开发中，敬请期待',
        showCancel: false
      });
    },
  
    // 查看历史记录
    viewHistory: function() {
      // TODO: 跳转到历史记录页面
      wx.showModal({
        title: '历史记录',
        content: '功能开发中，敬请期待',
        showCancel: false
      });
    },
  
    // 打开高级设置
    openSettings: function() {
      // TODO: 跳转到设置页面
      wx.showModal({
        title: '高级设置',
        content: '功能开发中，敬请期待',
        showCancel: false
      });
    },
  });