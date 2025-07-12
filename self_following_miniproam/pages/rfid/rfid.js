// pages/rfid/rfid.js - RFID实时监控页面
Page({
    data: {
      // 连接状态
      connected: false,
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
      startTime: null
    },
  
    onLoad: function(options) {
      console.log('RFID页面加载');
      
      // TODO: 初始化RFID连接
      this.initRFIDConnection();
      
      // TODO: 恢复上次的配置
      this.loadSettings();
    },
  
    onUnload: function() {
      // 清理定时器
      if (this.data.runningTimer) {
        clearInterval(this.data.runningTimer);
      }
      
      // TODO: 断开RFID连接
      this.disconnectRFID();
    },
  
    // 初始化RFID连接
    initRFIDConnection: function() {
      // TODO: 实现RFID读写器连接逻辑
      console.log('初始化RFID连接...');
      
      // 模拟连接成功
      setTimeout(() => {
        this.setData({
          connected: true
        });
        wx.showToast({
          title: '读写器连接成功',
          icon: 'success'
        });
      }, 1000);
    },
  
    // 断开RFID连接
    disconnectRFID: function() {
      // TODO: 实现断开连接逻辑
      console.log('断开RFID连接');
    },
  
    // 加载设置
    loadSettings: function() {
      // TODO: 从本地存储加载用户设置
      const settings = wx.getStorageSync('rfidSettings') || {};
      if (settings.selectedAntenna) {
        this.setData({
          selectedAntenna: settings.selectedAntenna
        });
      }
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
          title: '请先连接读写器',
          icon: 'none'
        });
        return;
      }
  
      console.log('开始RFID扫描');
      
      // TODO: 发送开始扫描指令到读写器
      
      this.setData({
        isScanning: true,
        startTime: Date.now()
      });
  
      // 启动计时器
      this.startRunningTimer();
  
      // TODO: 模拟数据更新（实际应该接收读写器数据）
      this.simulateDataUpdate();
  
      wx.showToast({
        title: '开始扫描',
        icon: 'success'
      });
    },
  
    // 停止扫描
    stopScanning: function() {
      console.log('停止RFID扫描');
      
      // TODO: 发送停止扫描指令到读写器
      
      this.setData({
        isScanning: false
      });
  
      // 停止计时器
      if (this.data.runningTimer) {
        clearInterval(this.data.runningTimer);
        this.data.runningTimer = null;
      }
  
      wx.showToast({
        title: '已停止扫描',
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
  
    // 模拟数据更新（实际项目中应该接收真实数据）
    simulateDataUpdate: function() {
      // TODO: 这里应该接收来自读写器的实时数据
      
      // 模拟添加标签
      const mockTags = [
        {
          epc: 'E200 3412 0123 4567 8901 2345',
          rssi: -45,
          readCount: 12,
          antenna: 1,
          lastSeen: '10:23:45',
          isActive: true
        },
        {
          epc: 'E200 3412 0123 4567 8901 2346',
          rssi: -52,
          readCount: 8,
          antenna: 1,
          lastSeen: '10:23:44',
          isActive: true
        }
      ];
  
      // 模拟数据更新
      let readCount = 0;
      const updateInterval = setInterval(() => {
        if (!this.data.isScanning) {
          clearInterval(updateInterval);
          return;
        }
  
        readCount++;
        this.setData({
          readRate: Math.floor(Math.random() * 50) + 75,
          totalReads: this.data.totalReads + 1,
          avgReadsPerTag: ((this.data.totalReads + 1) / Math.max(1, this.data.totalTags)).toFixed(1)
        });
  
        // 随机添加新标签
        if (Math.random() > 0.7 && this.data.tagList.length < 10) {
          const newTag = {
            epc: `E200 3412 0123 4567 8901 234${this.data.tagList.length}`,
            rssi: -Math.floor(Math.random() * 40 + 40),
            readCount: 1,
            antenna: this.data.selectedAntenna,
            lastSeen: new Date().toLocaleTimeString('zh-CN', { hour12: false }),
            isActive: true
          };
  
          this.setData({
            tagList: [newTag, ...this.data.tagList],
            totalTags: this.data.totalTags + 1,
            newTagsCount: this.data.newTagsCount + 1
          });
        }
      }, 1000);
    },
  
    // 选择天线
    selectAntenna: function(e) {
      const antenna = parseInt(e.currentTarget.dataset.antenna);
      this.setData({
        selectedAntenna: antenna
      });
  
      // TODO: 发送天线切换指令到读写器
      console.log('切换到天线', antenna);
  
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
  
    // 处理RFID数据（预留接口）
    handleRFIDData: function(data) {
      // TODO: 处理从读写器接收的数据
      console.log('收到RFID数据:', data);
    },
  
    // 发送指令到读写器（预留接口）
    sendCommand: function(command, params) {
      // TODO: 实现发送指令逻辑
      console.log('发送指令:', command, params);
    }
  });