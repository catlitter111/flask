// pages/history/history.js - 机器人伴侣跟踪历史
Page({
    data: {
      // === 历史数据 ===
      trackingHistory: [],         // 跟踪历史记录
      currentTrack: null,          // 当前查看的轨迹
      filteredHistory: [],         // 过滤后的历史记录
      
      // === 统计数据 ===
      totalTracks: 0,              // 总跟踪次数
      totalDistance: 0,            // 总跟踪距离
      totalDuration: 0,            // 总跟踪时长
      averageSpeed: 0,             // 平均速度
      
      // === 筛选设置 ===
      filterMode: 'all',           // 'all', 'today', 'week', 'month'
      sortMode: 'time_desc',       // 'time_desc', 'time_asc', 'distance_desc', 'duration_desc'
      selectedDate: '',            // 选择的日期
      
      // === 地图显示 ===
      mapLatitude: 39.916,         // 地图中心纬度
      mapLongitude: 116.397,       // 地图中心经度
      mapScale: 16,                // 地图缩放级别
      mapMarkers: [],              // 地图标记点
      mapPolylines: [],            // 地图轨迹线
      showMap: false,              // 是否显示地图
      
      // === UI状态 ===
      selectedTrackId: null,       // 选中的轨迹ID
      showTrackDetail: false,      // 显示轨迹详情
      showFilterModal: false,      // 显示筛选弹窗
      showStatistics: true,        // 显示统计信息
      loading: false,              // 加载状态
      refreshing: false,           // 刷新状态
      
      // === 实时跟踪 ===
      realtimeTracking: false,     // 实时跟踪模式
      currentPosition: null,       // 当前位置
      liveTrackData: [],           // 实时轨迹数据
      
      // === 导出选项 ===
      exportFormats: ['GPX', 'KML', 'JSON', 'CSV'],
      selectedExportFormat: 'GPX'
    },
  
    onLoad: function(options) {
      console.log('📍 跟踪历史页面加载');
      
      // 注册到全局应用
      const app = getApp();
      app.globalData.historyPage = this;
      
      // 加载历史数据
      this.loadTrackingHistory();
      
      // 计算统计数据
      this.calculateStatistics();
      
      // 获取当前位置
      this.getCurrentLocation();
      
      // 检查是否有指定轨迹
      if (options.trackId) {
        this.viewTrackDetail(options.trackId);
      }
    },
    
    onShow: function() {
      console.log('📍 跟踪历史页面显示');
      
      // 重新注册
      const app = getApp();
      app.globalData.historyPage = this;
      
      // 刷新数据
      this.refreshData();
    },
    
    onUnload: function() {
      // 页面卸载时取消注册
      const app = getApp();
      app.globalData.historyPage = null;
      
      // 停止实时跟踪
      if (this.data.realtimeTracking) {
        this.stopRealtimeTracking();
      }
    },

    // === 连接状态处理 ===
    updateConnectionStatus: function(isConnected, robotId) {
      console.log('历史页面连接状态更新:', isConnected, robotId);
      // 历史页面可以根据连接状态更新实时跟踪功能的可用性
      if (!isConnected && this.data.realtimeTracking) {
        this.stopRealtimeTracking();
      }
    },

    handleCompanionDisconnected: function(data) {
      console.log('历史页面处理伴侣断开连接:', data);
      if (this.data.realtimeTracking) {
        this.stopRealtimeTracking();
        wx.showToast({
          title: '机器人断开，实时跟踪已停止',
          icon: 'none'
        });
      }
    },

    handleTrackingData: function(data) {
      console.log('收到跟踪数据:', data);
      // 处理实时跟踪数据
      if (this.data.realtimeTracking) {
        this.updateLiveTrackData(data);
      }
    },

    handlePositionUpdate: function(data) {
      console.log('收到位置更新:', data);
      // 更新当前位置显示
      if (data.robot_position) {
        this.setData({
          currentPosition: data.robot_position
        });
      }
    },

    // 处理详细跟踪数据
    handleDetailedTrackingData: function(data) {
      console.log('📍 收到跟踪数据更新:', data);
      
      try {
        // 检查是否正在实时跟踪
        if (this.data.realtimeTracking) {
          this.updateLiveTrackData(data);
        }
        
        // 解析详细跟踪数据
        const detailedData = data.data || {};
        
        // 更新实时统计信息
        if (detailedData.statistics) {
          console.log('📊 更新实时统计:', detailedData.statistics);
          // 可以在这里更新页面上的实时统计显示
        }
        
        // 处理轨迹数据
        if (detailedData.tracks && Array.isArray(detailedData.tracks)) {
          console.log('📈 处理轨迹数据:', detailedData.tracks.length, '条轨迹');
          // 如果需要，可以在这里更新历史记录
        }
        
        // 处理目标轨迹
        if (detailedData.target_track) {
          console.log('🎯 目标轨迹更新:', detailedData.target_track);
          // 更新目标轨迹显示
        }
        
        // 记录详细跟踪数据到历史记录（可选）
        if (this.data.realtimeTracking) {
          this.recordDetailedTrackingData(detailedData);
        }
        
      } catch (error) {
        console.error('❌ 处理详细跟踪数据失败:', error);
      }
    },
    
    // 记录详细跟踪数据到历史记录
    recordDetailedTrackingData: function(detailedData) {
      try {
        // 只在实时跟踪时记录数据
        if (!this.data.realtimeTracking) {
          return;
        }
        
        // 创建历史记录条目
        const historyEntry = {
          id: 'live_' + Date.now(),
          timestamp: Date.now(),
          type: 'detailed_tracking',
          data: {
            total_tracks: detailedData.total_tracks || 0,
            target_detected: detailedData.target_detected || false,
            tracking_mode: detailedData.tracking_mode || 'unknown',
            frame_id: detailedData.frame_id || 0,
            statistics: detailedData.statistics || {},
            target_track: detailedData.target_track || null,
            system_info: detailedData.system_info || {}
          }
        };
        
        // 添加到实时跟踪数据
        const currentLiveData = this.data.liveTrackData || [];
        currentLiveData.push(historyEntry);
        
        // 限制实时数据数量，避免内存过多占用
        if (currentLiveData.length > 100) {
          currentLiveData.splice(0, currentLiveData.length - 100);
        }
        
        this.setData({
          liveTrackData: currentLiveData
        });
        
      } catch (error) {
        console.error('❌ 记录详细跟踪数据失败:', error);
      }
    },
    
    onPullDownRefresh: function() {
      this.refreshData(() => {
        wx.stopPullDownRefresh();
      });
    },
    
    // === 数据加载 ===
    loadTrackingHistory: function() {
      this.setData({ loading: true });
      
      try {
        // 从本地存储加载
        const history = wx.getStorageSync('trackingHistory') || [];
        
        // 从全局数据同步
        const app = getApp();
        const globalHistory = app.globalData.trackingHistory || [];
        
        // 合并数据并去重
        const mergedHistory = this.mergeHistoryData(history, globalHistory);
        
        this.setData({
          trackingHistory: mergedHistory,
          filteredHistory: mergedHistory,
          loading: false
        });
        
        // 应用当前筛选
        this.applyFilter();
        
        console.log(`📊 已加载 ${mergedHistory.length} 条跟踪记录`);
        
      } catch (error) {
        console.error('❌ 加载跟踪历史失败:', error);
        wx.showToast({
          title: '加载历史失败',
          icon: 'error'
        });
        this.setData({ loading: false });
      }
    },
    
    mergeHistoryData: function(localData, globalData) {
      const merged = [...localData];
      const existingIds = new Set(localData.map(item => item.id));
      
      globalData.forEach(item => {
        if (!existingIds.has(item.id)) {
          merged.push(item);
        }
      });
      
      // 按时间排序
      return merged.sort((a, b) => new Date(b.start_time) - new Date(a.start_time));
    },
    
    refreshData: function(callback) {
      this.setData({ refreshing: true });
      
      // 重新加载数据
      this.loadTrackingHistory();
      
      // 重新计算统计
      this.calculateStatistics();
      
      setTimeout(() => {
        this.setData({ refreshing: false });
        if (callback) callback();
      }, 1000);
    },
    
    // === 统计计算 ===
    calculateStatistics: function() {
      const history = this.data.filteredHistory;
      
      let totalDistance = 0;
      let totalDuration = 0;
      let totalTracks = history.length;
      
      history.forEach(track => {
        totalDistance += track.total_distance || 0;
        totalDuration += track.duration || 0;
      });
      
      const averageSpeed = totalDuration > 0 ? (totalDistance / totalDuration * 3.6) : 0; // km/h
      
      this.setData({
        totalTracks,
        totalDistance: totalDistance.toFixed(2),
        totalDuration: Math.round(totalDuration / 60), // 转换为分钟
        averageSpeed: averageSpeed.toFixed(1)
      });
    },
    
    // === 筛选和排序 ===
    applyFilter: function() {
      let filtered = [...this.data.trackingHistory];
      const now = new Date();
      
      // 按时间筛选
      switch (this.data.filterMode) {
        case 'today':
          const today = new Date(now.getFullYear(), now.getMonth(), now.getDate());
          filtered = filtered.filter(track => new Date(track.start_time) >= today);
          break;
          
        case 'week':
          const weekAgo = new Date(now.getTime() - 7 * 24 * 60 * 60 * 1000);
          filtered = filtered.filter(track => new Date(track.start_time) >= weekAgo);
          break;
          
        case 'month':
          const monthAgo = new Date(now.getTime() - 30 * 24 * 60 * 60 * 1000);
          filtered = filtered.filter(track => new Date(track.start_time) >= monthAgo);
          break;
      }
      
      // 排序
      switch (this.data.sortMode) {
        case 'time_asc':
          filtered.sort((a, b) => new Date(a.start_time) - new Date(b.start_time));
          break;
        case 'distance_desc':
          filtered.sort((a, b) => (b.total_distance || 0) - (a.total_distance || 0));
          break;
        case 'duration_desc':
          filtered.sort((a, b) => (b.duration || 0) - (a.duration || 0));
          break;
        default: // time_desc
          filtered.sort((a, b) => new Date(b.start_time) - new Date(a.start_time));
      }
      
      this.setData({
        filteredHistory: filtered
      });
      
      // 重新计算统计
      this.calculateStatistics();
    },
    
    changeFilter: function(e) {
      const filterMode = e.currentTarget.dataset.filter;
      
      this.setData({
        filterMode: filterMode
      });
      
      this.applyFilter();
    },
    
    changeSortMode: function(e) {
      const sortMode = e.detail.value;
      
      this.setData({
        sortMode: sortMode
      });
      
      this.applyFilter();
    },
    
    // === 轨迹详情 ===
    viewTrackDetail: function(trackId) {
      const track = this.data.filteredHistory.find(t => t.id === trackId);
      
      if (!track) {
        wx.showToast({
          title: '轨迹不存在',
          icon: 'error'
        });
        return;
      }
      
      this.setData({
        currentTrack: track,
        selectedTrackId: trackId,
        showTrackDetail: true
      });
      
      // 在地图上显示轨迹
      this.showTrackOnMap(track);
      
      console.log('📊 查看轨迹详情:', track.id);
    },
    
    onTrackItemTap: function(e) {
      const trackId = e.currentTarget.dataset.trackId;
      this.viewTrackDetail(trackId);
    },
    
    closeTrackDetail: function() {
      this.setData({
        showTrackDetail: false,
        currentTrack: null,
        selectedTrackId: null,
        showMap: false
      });
    },
    
    // === 地图显示 ===
    showTrackOnMap: function(track) {
      if (!track.path_data || track.path_data.length === 0) {
        wx.showToast({
          title: '无轨迹数据',
          icon: 'none'
        });
        return;
      }
      
      const pathData = track.path_data;
      const firstPoint = pathData[0];
      const lastPoint = pathData[pathData.length - 1];
      
      // 设置地图中心为轨迹中心
      const centerLat = (firstPoint.latitude + lastPoint.latitude) / 2;
      const centerLng = (firstPoint.longitude + lastPoint.longitude) / 2;
      
      // 创建标记点
      const markers = [
        {
          id: 1,
          latitude: firstPoint.latitude,
          longitude: firstPoint.longitude,
          iconPath: '/images/start-marker.png',
          width: 30,
          height: 30,
          title: '起点'
        },
        {
          id: 2,
          latitude: lastPoint.latitude,
          longitude: lastPoint.longitude,
          iconPath: '/images/end-marker.png',
          width: 30,
          height: 30,
          title: '终点'
        }
      ];
      
      // 创建轨迹线
      const polylines = [{
        points: pathData.map(point => ({
          latitude: point.latitude,
          longitude: point.longitude
        })),
        color: '#2196F3',
        width: 4,
        dottedLine: false,
        arrowLine: true
      }];
      
      this.setData({
        mapLatitude: centerLat,
        mapLongitude: centerLng,
        mapMarkers: markers,
        mapPolylines: polylines,
        showMap: true
      });
    },
    
    getCurrentLocation: function() {
      wx.getLocation({
        type: 'gcj02',
        success: (res) => {
          this.setData({
            mapLatitude: res.latitude,
            mapLongitude: res.longitude,
            currentPosition: {
              latitude: res.latitude,
              longitude: res.longitude
            }
          });
          console.log('📍 获取当前位置成功');
        },
        fail: (error) => {
          console.error('📍 获取位置失败:', error);
        }
      });
    },
    
    // === 实时跟踪 ===
    startRealtimeTracking: function() {
      this.setData({
        realtimeTracking: true,
        liveTrackData: []
      });
      
      // 开始记录实时轨迹
      this.realtimeTimer = setInterval(() => {
        this.recordCurrentPosition();
      }, 2000); // 每2秒记录一次位置
      
      wx.showToast({
        title: '开始实时跟踪',
        icon: 'success'
      });
      
      console.log('🔴 开始实时跟踪');
    },
    
    stopRealtimeTracking: function() {
      this.setData({
        realtimeTracking: false
      });
      
      if (this.realtimeTimer) {
        clearInterval(this.realtimeTimer);
        this.realtimeTimer = null;
      }
      
      // 保存实时轨迹数据
      if (this.data.liveTrackData.length > 0) {
        this.saveRealtimeTrack();
      }
      
      wx.showToast({
        title: '停止实时跟踪',
        icon: 'success'
      });
      
            console.log('⏹️ 停止实时跟踪');
    },

    updateLiveTrackData: function(data) {
      if (!this.data.realtimeTracking) return;
      
      const trackPoint = {
        latitude: data.robot_position?.y || 0,
        longitude: data.robot_position?.x || 0,
        timestamp: Date.now(),
        robot_position: data.robot_position,
        target_position: data.target_position,
        distance: data.distance || 0,
        following_mode: data.following_mode
      };
      
      const liveData = [...this.data.liveTrackData, trackPoint];
      
      this.setData({
        liveTrackData: liveData
      });
      
      // 限制数据量
      if (liveData.length > 1000) {
        this.setData({
          liveTrackData: liveData.slice(-800)
        });
      }
      
      console.log('📍 更新实时跟踪数据');
    },

    recordCurrentPosition: function() {
      wx.getLocation({
        type: 'gcj02',
        success: (res) => {
          const positionData = {
            latitude: res.latitude,
            longitude: res.longitude,
            timestamp: Date.now(),
            accuracy: res.accuracy || 0,
            speed: res.speed || 0
          };
          
          const liveData = [...this.data.liveTrackData, positionData];
          
          this.setData({
            liveTrackData: liveData,
            currentPosition: {
              latitude: res.latitude,
              longitude: res.longitude
            }
          });
        }
      });
    },
    
    saveRealtimeTrack: function() {
      const liveData = this.data.liveTrackData;
      
      if (liveData.length < 2) {
        return; // 数据太少，不保存
      }
      
      // 计算总距离和时长
      let totalDistance = 0;
      for (let i = 1; i < liveData.length; i++) {
        const dist = this.calculateDistance(
          liveData[i-1].latitude, liveData[i-1].longitude,
          liveData[i].latitude, liveData[i].longitude
        );
        totalDistance += dist;
      }
      
      const duration = (liveData[liveData.length - 1].timestamp - liveData[0].timestamp) / 1000;
      
      const trackRecord = {
        id: `track_${Date.now()}`,
        start_time: new Date(liveData[0].timestamp).toISOString(),
        end_time: new Date(liveData[liveData.length - 1].timestamp).toISOString(),
        duration: duration,
        total_distance: totalDistance,
        average_speed: duration > 0 ? totalDistance / duration : 0,
        path_data: liveData,
        track_type: 'realtime',
        target_id: null
      };
      
      // 添加到历史记录
      const history = [...this.data.trackingHistory, trackRecord];
      
      this.setData({
        trackingHistory: history,
        liveTrackData: []
      });
      
      // 保存到本地存储
      wx.setStorageSync('trackingHistory', history);
      
      // 刷新显示
      this.applyFilter();
      
      wx.showToast({
        title: '轨迹已保存',
        icon: 'success'
      });
      
      console.log('💾 实时轨迹已保存:', trackRecord.id);
    },
    
    // === 数据管理 ===
    deleteTrack: function(e) {
      const trackId = e.currentTarget.dataset.trackId;
      
      wx.showModal({
        title: '删除确认',
        content: '确定要删除这条跟踪记录吗？',
        confirmColor: '#FF5722',
        success: (res) => {
          if (res.confirm) {
            this.performDeleteTrack(trackId);
          }
        }
      });
    },
    
    performDeleteTrack: function(trackId) {
      const history = this.data.trackingHistory.filter(track => track.id !== trackId);
      
      this.setData({
        trackingHistory: history
      });
      
      // 更新本地存储
      wx.setStorageSync('trackingHistory', history);
      
      // 刷新显示
      this.applyFilter();
      
      // 如果删除的是当前查看的轨迹，关闭详情
      if (this.data.selectedTrackId === trackId) {
        this.closeTrackDetail();
      }
      
      wx.showToast({
        title: '删除成功',
        icon: 'success'
      });
      
      console.log('🗑️ 删除轨迹:', trackId);
    },
    
    clearAllHistory: function() {
      wx.showModal({
        title: '清空确认',
        content: '确定要清空所有跟踪历史吗？此操作不可恢复！',
        confirmColor: '#FF5722',
        success: (res) => {
          if (res.confirm) {
            this.performClearHistory();
          }
        }
      });
    },
    
    performClearHistory: function() {
      this.setData({
        trackingHistory: [],
        filteredHistory: [],
        currentTrack: null,
        showTrackDetail: false
      });
      
      // 清空本地存储
      wx.removeStorageSync('trackingHistory');
      
      // 重新计算统计
      this.calculateStatistics();
      
      wx.showToast({
        title: '历史已清空',
        icon: 'success'
      });
      
      console.log('🧹 跟踪历史已清空');
    },
    
    // === 数据导出 ===
    exportTrackData: function(e) {
      const trackId = e.currentTarget.dataset.trackId;
      const format = this.data.selectedExportFormat;
      
      const track = this.data.filteredHistory.find(t => t.id === trackId);
      
      if (!track) {
        wx.showToast({
          title: '轨迹不存在',
          icon: 'error'
        });
        return;
      }
      
      this.performExport(track, format);
    },
    
    performExport: function(track, format) {
      // 这里应该调用导出服务
      // 目前只是模拟导出
      
      wx.showLoading({
        title: '导出中...'
      });
      
      setTimeout(() => {
        wx.hideLoading();
        wx.showToast({
          title: `${format}导出成功`,
          icon: 'success'
        });
        
        console.log(`📤 导出轨迹 ${track.id} 为 ${format} 格式`);
      }, 2000);
    },
    
    // === WebSocket消息处理 ===
    handleTrackingData: function(data) {
      console.log('📍 收到跟踪数据更新:', data);
      
      // 处理新的跟踪数据
      if (data.track_point) {
        this.addTrackPoint(data.track_point);
      }
      
      if (data.track_complete) {
        this.addCompleteTrack(data.track_complete);
      }
    },
    
    handlePositionUpdate: function(data) {
      console.log('📡 收到位置更新:', data);
      
      // 更新当前位置
      if (data.robot_position) {
        this.setData({
          currentPosition: data.robot_position
        });
      }
    },
    
    addTrackPoint: function(point) {
      if (this.data.realtimeTracking) {
        const liveData = [...this.data.liveTrackData, point];
        this.setData({
          liveTrackData: liveData
        });
      }
    },
    
    addCompleteTrack: function(track) {
      const history = [...this.data.trackingHistory, track];
      
      this.setData({
        trackingHistory: history
      });
      
      // 保存到本地存储
      wx.setStorageSync('trackingHistory', history);
      
      // 刷新显示
      this.applyFilter();
      
      wx.showToast({
        title: '新轨迹已记录',
        icon: 'success'
      });
    },
    
    // === 工具方法 ===
    calculateDistance: function(lat1, lng1, lat2, lng2) {
      const R = 6371; // 地球半径 km
      const dLat = (lat2 - lat1) * Math.PI / 180;
      const dLng = (lng2 - lng1) * Math.PI / 180;
      const a = Math.sin(dLat/2) * Math.sin(dLat/2) +
                Math.cos(lat1 * Math.PI / 180) * Math.cos(lat2 * Math.PI / 180) *
                Math.sin(dLng/2) * Math.sin(dLng/2);
      const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1-a));
      return R * c * 1000; // 返回米
    },
    
    formatDuration: function(seconds) {
      const minutes = Math.floor(seconds / 60);
      const remainingSeconds = seconds % 60;
      return `${minutes}分${remainingSeconds}秒`;
    },
    
    formatDate: function(dateString) {
      const date = new Date(dateString);
      return `${date.getFullYear()}-${String(date.getMonth() + 1).padStart(2, '0')}-${String(date.getDate()).padStart(2, '0')} ${String(date.getHours()).padStart(2, '0')}:${String(date.getMinutes()).padStart(2, '0')}`;
    },
    
    // === UI交互 ===
    toggleStatistics: function() {
      this.setData({
        showStatistics: !this.data.showStatistics
      });
    },
    
    showFilterModal: function() {
      this.setData({
        showFilterModal: true
      });
    },
    
    hideFilterModal: function() {
      this.setData({
        showFilterModal: false
      });
    }
  }); 