// pages/rfid/history/history.js - RFID历史记录页面
Page({
  data: {
    // 历史记录数据
    historyList: [],
    filteredHistory: [],
    
    // 统计数据
    totalRecords: 0,
    detectedCount: 0,
    lostCount: 0,
    uniqueTagsCount: 0,
    
    // 筛选和排序
    filterType: 'all', // all, detected, lost
    sortBy: 'time_desc', // time_desc, time_asc, epc_asc, epc_desc
    dateFilter: 'today', // today, week, month, all
    
    // 搜索
    searchKeyword: '',
    showSearch: false,
    
    // 分页
    currentPage: 1,
    pageSize: 20,
    hasMore: true,
    
    // 选择模式
    selectionMode: false,
    selectedItems: [],
    
    // 日期范围
    startDate: '',
    endDate: '',
    
    // 加载状态
    loading: false,
    refreshing: false
  },

  onLoad: function(options) {
    console.log('📚 RFID历史记录页面加载');
    
    // 设置导航栏标题
    wx.setNavigationBarTitle({
      title: 'RFID历史记录'
    });
    
    // 初始化日期
    this.initDateFilter();
    
    // 加载历史数据
    this.loadHistoryData();
    
    // 注册页面到全局app，用于实时更新
    const app = getApp();
    app.globalData.rfidHistoryPage = this;
    
      // 注意：下拉刷新功能已在页面配置文件 history.json 中启用
  // 无需在代码中手动启用
  },

  onUnload: function() {
    console.log('📚 RFID历史记录页面卸载');
    
    // 从全局app中移除页面引用
    const app = getApp();
    if (app.globalData.rfidHistoryPage === this) {
      app.globalData.rfidHistoryPage = null;
    }
  },

  onShow: function() {
    console.log('📚 RFID历史记录页面显示');
    
    // 重新注册页面到全局app
    const app = getApp();
    app.globalData.rfidHistoryPage = this;
    
    // 重新加载数据（可能有新的历史记录）
    this.loadHistoryData();
  },

  // 外部调用：通知历史记录更新
  notifyHistoryUpdate: function() {
    console.log('📚 收到历史记录更新通知');
    this.loadHistoryData();
  },

  // 初始化日期筛选
  initDateFilter: function() {
    const now = new Date();
    const today = this.formatDate(now);
    const weekAgo = this.formatDate(new Date(now.getTime() - 7 * 24 * 60 * 60 * 1000));
    
    this.setData({
      endDate: today,
      startDate: weekAgo
    });
  },

  // 加载历史数据
  loadHistoryData: function() {
    this.setData({ loading: true });
    
    try {
      // 从本地存储加载
      const historyData = wx.getStorageSync('rfidHistory') || [];
      console.log(`📚 加载历史数据: ${historyData.length} 条记录`);
      
      // 处理和统计数据
      this.processHistoryData(historyData);
      
      // 应用筛选
      this.applyFilters();
      
    } catch (e) {
      console.error('加载历史数据失败:', e);
      wx.showToast({
        title: '加载失败',
        icon: 'none'
      });
    } finally {
      this.setData({ loading: false });
    }
  },

  // 处理历史数据
  processHistoryData: function(rawData) {
    const processedData = rawData.map(item => {
      // 如果数据已经包含了显示字段，使用已有的；否则生成新的
      return {
        ...item,
        timeString: item.timeString || new Date(item.timestamp).toLocaleString('zh-CN'),
        dateString: item.dateString || new Date(item.timestamp).toLocaleDateString('zh-CN'),
        timeOnly: item.timeOnly || new Date(item.timestamp).toLocaleTimeString('zh-CN'),
        shortEpc: item.shortEpc || (item.epc && item.epc.length > 16 ? `${item.epc.substr(0, 8)}...${item.epc.substr(-8)}` : item.epc),
        actionText: item.actionText || (item.action === 'detected' ? '检测到' : '离线'),
        actionIcon: item.actionIcon || (item.action === 'detected' ? '🟢' : '🔴'),
        durationText: item.durationText || (item.duration ? this.formatDuration(item.duration) : '-'),
        // 确保必要字段存在
        rssi: item.rssi || -99,
        antenna: item.antenna || 1,
        readCount: item.readCount || 0,
        signalQuality: item.signalQuality || '未知'
      };
    });

    // 统计数据
    const detectedCount = processedData.filter(item => item.action === 'detected').length;
    const lostCount = processedData.filter(item => item.action === 'lost').length;
    const uniqueTags = new Set(processedData.map(item => item.epc));

    console.log(`📊 历史数据统计: 总记录${processedData.length}, 检测${detectedCount}, 离线${lostCount}, 唯一标签${uniqueTags.size}`);

    this.setData({
      historyList: processedData,
      totalRecords: processedData.length,
      detectedCount: detectedCount,
      lostCount: lostCount,
      uniqueTagsCount: uniqueTags.size
    });
  },

  // 应用筛选条件
  applyFilters: function() {
    let filtered = [...this.data.historyList];
    
    // 按类型筛选
    if (this.data.filterType !== 'all') {
      filtered = filtered.filter(item => item.action === this.data.filterType);
    }
    
    // 按日期筛选
    filtered = this.filterByDate(filtered);
    
    // 按关键词搜索
    if (this.data.searchKeyword) {
      const keyword = this.data.searchKeyword.toLowerCase();
      filtered = filtered.filter(item => 
        item.epc.toLowerCase().includes(keyword) ||
        item.actionText.includes(keyword)
      );
    }
    
    // 排序
    filtered = this.sortData(filtered);
    
    // 分页
    const paginatedData = this.paginateData(filtered);
    
    this.setData({
      filteredHistory: paginatedData,
      hasMore: paginatedData.length < filtered.length
    });
  },

  // 按日期筛选
  filterByDate: function(data) {
    const now = new Date();
    const today = new Date(now.getFullYear(), now.getMonth(), now.getDate());
    
    switch (this.data.dateFilter) {
      case 'today':
        return data.filter(item => new Date(item.timestamp) >= today);
      case 'week':
        const weekAgo = new Date(now.getTime() - 7 * 24 * 60 * 60 * 1000);
        return data.filter(item => new Date(item.timestamp) >= weekAgo);
      case 'month':
        const monthAgo = new Date(now.getTime() - 30 * 24 * 60 * 60 * 1000);
        return data.filter(item => new Date(item.timestamp) >= monthAgo);
      default:
        return data;
    }
  },

  // 排序数据
  sortData: function(data) {
    return data.sort((a, b) => {
      switch (this.data.sortBy) {
        case 'time_desc':
          return b.timestamp - a.timestamp;
        case 'time_asc':
          return a.timestamp - b.timestamp;
        case 'epc_asc':
          return a.epc.localeCompare(b.epc);
        case 'epc_desc':
          return b.epc.localeCompare(a.epc);
        default:
          return b.timestamp - a.timestamp;
      }
    });
  },

  // 分页数据
  paginateData: function(data) {
    const start = 0;
    const end = this.data.currentPage * this.data.pageSize;
    return data.slice(start, end);
  },

  // 筛选类型改变
  onFilterTypeChange: function(e) {
    const filterOptions = ['all', 'detected', 'lost'];
    const selectedFilter = filterOptions[e.detail.value];
    
    this.setData({
      filterType: selectedFilter,
      currentPage: 1
    }, () => {
      this.applyFilters();
    });
  },

  // 日期筛选改变
  onDateFilterChange: function(e) {
    const dateOptions = ['today', 'week', 'month', 'all'];
    const selectedDate = dateOptions[e.detail.value];
    
    this.setData({
      dateFilter: selectedDate,
      currentPage: 1
    }, () => {
      this.applyFilters();
    });
  },

  // 排序方式改变
  onSortChange: function(e) {
    const sortOptions = ['time_desc', 'time_asc', 'epc_asc', 'epc_desc'];
    const selectedSort = sortOptions[e.detail.value];
    
    this.setData({
      sortBy: selectedSort,
      currentPage: 1
    }, () => {
      this.applyFilters();
    });
  },

  // 搜索输入
  onSearchInput: function(e) {
    this.setData({
      searchKeyword: e.detail.value,
      currentPage: 1
    }, () => {
      this.applyFilters();
    });
  },

  // 切换搜索显示
  toggleSearch: function() {
    this.setData({
      showSearch: !this.data.showSearch,
      searchKeyword: this.data.showSearch ? '' : this.data.searchKeyword
    }, () => {
      if (!this.data.showSearch) {
        this.applyFilters();
      }
    });
  },

  // 加载更多
  loadMore: function() {
    if (!this.data.hasMore || this.data.loading) return;
    
    this.setData({
      currentPage: this.data.currentPage + 1
    }, () => {
      this.applyFilters();
    });
  },

  // 下拉刷新
  onPullDownRefresh: function() {
    console.log('📚 开始刷新RFID历史记录');
    this.setData({
      refreshing: true,
      currentPage: 1
    });
    
    // 从存储中重新加载历史数据
    this.loadHistoryData();
    
    setTimeout(() => {
      wx.stopPullDownRefresh();
      this.setData({ refreshing: false });
      console.log('📚 历史记录刷新完成');
    }, 1000);
  },

  // 下拉刷新功能说明：
  // 下拉刷新功能已在页面配置文件 history.json 中启用，
  // 微信小程序会自动处理下拉手势，并调用 onPullDownRefresh 生命周期函数

  // 切换选择模式
  toggleSelectionMode: function() {
    this.setData({
      selectionMode: !this.data.selectionMode,
      selectedItems: []
    });
  },

  // 选择/取消选择项目
  toggleItemSelection: function(e) {
    if (!this.data.selectionMode) return;
    
    const id = e.currentTarget.dataset.id;
    const selectedItems = [...this.data.selectedItems];
    const index = selectedItems.indexOf(id);
    
    if (index > -1) {
      selectedItems.splice(index, 1);
    } else {
      selectedItems.push(id);
    }
    
    this.setData({ selectedItems });
  },

  // 全选/取消全选
  toggleSelectAll: function() {
    const isAllSelected = this.data.selectedItems.length === this.data.filteredHistory.length;
    
    this.setData({
      selectedItems: isAllSelected ? [] : this.data.filteredHistory.map(item => item.id)
    });
  },

  // 删除选中项
  deleteSelected: function() {
    if (this.data.selectedItems.length === 0) {
      wx.showToast({
        title: '请先选择要删除的记录',
        icon: 'none'
      });
      return;
    }
    
    wx.showModal({
      title: '确认删除',
      content: `确定要删除选中的 ${this.data.selectedItems.length} 条记录吗？此操作不可恢复。`,
      success: (res) => {
        if (res.confirm) {
          this.performDelete();
        }
      }
    });
  },

  // 执行删除
  performDelete: function() {
    try {
      // 从历史记录中移除选中的项
      const updatedHistory = this.data.historyList.filter(
        item => !this.data.selectedItems.includes(item.id)
      );
      
      // 保存到本地存储
      wx.setStorageSync('rfidHistory', updatedHistory);
      
      // 重新加载数据
      this.loadHistoryData();
      
      // 退出选择模式
      this.setData({
        selectionMode: false,
        selectedItems: []
      });
      
      wx.showToast({
        title: '删除成功',
        icon: 'success'
      });
      
    } catch (e) {
      console.error('删除记录失败:', e);
      wx.showToast({
        title: '删除失败',
        icon: 'none'
      });
    }
  },

  // 导出数据
  exportData: function() {
    if (this.data.filteredHistory.length === 0) {
      wx.showToast({
        title: '暂无数据可导出',
        icon: 'none'
      });
      return;
    }
    
    try {
      // 生成CSV格式数据
      let csvContent = '时间,日期,EPC编码,动作,信号强度,天线,读取次数,持续时间\n';
      
      this.data.filteredHistory.forEach(item => {
        csvContent += `${item.timeString},${item.dateString},${item.epc},${item.actionText},${item.rssi}dBm,${item.antenna},${item.readCount},${item.durationText}\n`;
      });
      
      // 复制到剪贴板
      wx.setClipboardData({
        data: csvContent,
        success: () => {
          wx.showModal({
            title: '导出成功',
            content: `已将 ${this.data.filteredHistory.length} 条记录复制到剪贴板，可粘贴到Excel或其他应用中。`,
            showCancel: false
          });
        }
      });
      
    } catch (e) {
      console.error('导出数据失败:', e);
      wx.showToast({
        title: '导出失败',
        icon: 'none'
      });
    }
  },

  // 清空所有历史
  clearAllHistory: function() {
    wx.showModal({
      title: '确认清空',
      content: '确定要清空所有历史记录吗？此操作不可恢复。',
      confirmColor: '#FF5722',
      success: (res) => {
        if (res.confirm) {
          try {
            wx.removeStorageSync('rfidHistory');
            this.loadHistoryData();
            wx.showToast({
              title: '已清空历史记录',
              icon: 'success'
            });
          } catch (e) {
            wx.showToast({
              title: '清空失败',
              icon: 'none'
            });
          }
        }
      }
    });
  },

  // 查看详细信息
  viewDetails: function(e) {
    const item = e.currentTarget.dataset.item;
    
    let content = `EPC: ${item.epc}\n`;
    content += `动作: ${item.actionText}\n`;
    content += `时间: ${item.timeString}\n`;
    content += `信号强度: ${item.rssi}dBm\n`;
    content += `天线: ${item.antenna}\n`;
    content += `读取次数: ${item.readCount}\n`;
    if (item.duration) {
      content += `持续时间: ${item.durationText}\n`;
    }
    
    wx.showModal({
      title: '记录详情',
      content: content,
      showCancel: false
    });
  },

  // 格式化持续时间
  formatDuration: function(duration) {
    if (!duration) return '-';
    
    const seconds = Math.floor(duration / 1000);
    if (seconds < 60) {
      return `${seconds}秒`;
    } else if (seconds < 3600) {
      const minutes = Math.floor(seconds / 60);
      const remainingSeconds = seconds % 60;
      return `${minutes}分${remainingSeconds}秒`;
    } else {
      const hours = Math.floor(seconds / 3600);
      const minutes = Math.floor((seconds % 3600) / 60);
      return `${hours}时${minutes}分`;
    }
  },

  // 格式化日期
  formatDate: function(date) {
    const year = date.getFullYear();
    const month = String(date.getMonth() + 1).padStart(2, '0');
    const day = String(date.getDate()).padStart(2, '0');
    return `${year}-${month}-${day}`;
  },

  // 分享历史统计
  shareStatistics: function() {
    const stats = {
      total: this.data.totalRecords,
      detected: this.data.detectedCount,
      lost: this.data.lostCount,
      uniqueTags: this.data.uniqueTagsCount
    };
    
    wx.showShareMenu({
      withShareTicket: true,
      menus: ['shareAppMessage', 'shareTimeline']
    });
  },

  // 分享
  onShareAppMessage: function() {
    return {
      title: `RFID历史记录统计: 总计${this.data.totalRecords}条记录`,
      path: '/pages/rfid/history/history',
      imageUrl: '/images/rfid-history-share.png'
    };
  }
});