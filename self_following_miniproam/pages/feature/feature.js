// pages/feature/feature.js - 机器人伴侣特征提取
Page({
    data: {
      // === 特征数据 ===
      extractedFeatures: [],       // 已提取的特征
      currentFeature: null,        // 当前选中的特征
      selectedPersonId: null,      // 选中的人员ID
      
      // === 相机控制 ===
      cameraAuthorized: false,     // 相机权限
      showCamera: false,           // 显示相机
      cameraPosition: 'back',      // 相机位置 front/back
      cameraFrameSize: 'medium',   // 相机帧大小
      
      // === 提取状态 ===
      extracting: false,           // 正在提取特征
      extractProgress: 0,          // 提取进度
      lastExtractTime: 0,          // 最后提取时间
      
      // === 特征管理 ===
      showFeatureDetail: false,    // 显示特征详情
      editingFeature: false,       // 编辑特征信息
      featureFilter: 'all',        // 特征筛选 all/recent/favorites
      
      // === 识别结果 ===
      recognitionResults: [],      // 识别结果
      recognitionAccuracy: 0,      // 识别精度
      processingTime: 0,           // 处理时间
      
      // === UI状态 ===
      loading: false,
      showManageModal: false,
      selectedFeatures: []         // 批量选择的特征
    },
  
    onLoad: function(options) {
      console.log('👤 特征提取页面加载');
      
      // 注册到全局应用
      const app = getApp();
      app.globalData.featurePage = this;
      
      // 检查相机权限
      this.checkCameraPermission();
      
      // 加载已保存的特征
      this.loadExtractedFeatures();
    },
    
    onShow: function() {
      const app = getApp();
      app.globalData.featurePage = this;
      this.refreshFeatureList();
    },
    
    onUnload: function() {
      const app = getApp();
      app.globalData.featurePage = null;
      this.stopCamera();
    },
    
    // === 相机管理 ===
    checkCameraPermission: function() {
      wx.getSetting({
        success: (res) => {
          if (res.authSetting['scope.camera']) {
            this.setData({ cameraAuthorized: true });
          } else {
            this.requestCameraPermission();
          }
        }
      });
    },
    
    requestCameraPermission: function() {
      wx.authorize({
        scope: 'scope.camera',
        success: () => {
          this.setData({ cameraAuthorized: true });
          console.log('📷 相机权限获取成功');
        },
        fail: () => {
          wx.showModal({
            title: '需要相机权限',
            content: '特征提取功能需要使用相机，请在设置中开启相机权限',
            confirmText: '去设置',
            success: (res) => {
              if (res.confirm) {
                wx.openSetting();
              }
            }
          });
        }
      });
    },
    
    startCamera: function() {
      if (!this.data.cameraAuthorized) {
        this.requestCameraPermission();
        return;
      }
      
      this.setData({ showCamera: true });
      console.log('📷 启动相机');
    },
    
    stopCamera: function() {
      this.setData({ showCamera: false });
      console.log('📷 关闭相机');
    },
    
    switchCamera: function() {
      const newPosition = this.data.cameraPosition === 'back' ? 'front' : 'back';
      this.setData({ cameraPosition: newPosition });
    },
    
    // === 特征提取 ===
    extractFeatures: function() {
      if (this.data.extracting) return;
      
      this.setData({ 
        extracting: true,
        extractProgress: 0
      });
      
      // 模拟提取进度
      const progressInterval = setInterval(() => {
        const progress = this.data.extractProgress + 10;
        this.setData({ extractProgress: progress });
        
        if (progress >= 100) {
          clearInterval(progressInterval);
          this.completeExtraction();
        }
      }, 200);
      
      // 发送提取命令到服务器
      this.sendExtractionCommand();
      
      console.log('🎯 开始特征提取');
    },
    
    sendExtractionCommand: function() {
      const app = getApp();
      
      app.sendSocketMessage({
        type: 'extract_features',
        timestamp: Date.now(),
        camera_position: this.data.cameraPosition,
        frame_size: this.data.cameraFrameSize
      });
    },
    
    completeExtraction: function() {
      this.setData({
        extracting: false,
        extractProgress: 0,
        lastExtractTime: Date.now()
      });
      
      wx.showToast({
        title: '特征提取完成',
        icon: 'success'
      });
      
      console.log('✅ 特征提取完成');
    },
    
    // === 特征管理 ===
    loadExtractedFeatures: function() {
      const app = getApp();
      const features = wx.getStorageSync('extractedFeatures') || app.globalData.extractedFeatures || [];
      
      this.setData({
        extractedFeatures: features
      });
      
      console.log(`📊 已加载 ${features.length} 个特征`);
    },
    
    saveFeature: function(featureData) {
      const features = [...this.data.extractedFeatures, featureData];
      
      this.setData({
        extractedFeatures: features
      });
      
      // 保存到本地存储
      wx.setStorageSync('extractedFeatures', features);
      
      // 同步到全局数据
      const app = getApp();
      app.globalData.extractedFeatures = features;
      
      console.log('💾 特征已保存:', featureData.id);
    },
    
    deleteFeature: function(e) {
      const featureId = e.currentTarget.dataset.featureId;
      
      wx.showModal({
        title: '删除确认',
        content: '确定要删除这个特征吗？',
        confirmColor: '#FF5722',
        success: (res) => {
          if (res.confirm) {
            this.performDeleteFeature(featureId);
          }
        }
      });
    },
    
    performDeleteFeature: function(featureId) {
      const features = this.data.extractedFeatures.filter(f => f.id !== featureId);
      
      this.setData({
        extractedFeatures: features
      });
      
      wx.setStorageSync('extractedFeatures', features);
      
      wx.showToast({
        title: '删除成功',
        icon: 'success'
      });
    },
    
    // === WebSocket消息处理 ===
    handleFeatureResult: function(data) {
      console.log('🎯 收到特征提取结果:', data);
      
      if (data.success) {
        const featureData = {
          id: `feature_${Date.now()}`,
          person_id: data.person_id,
          feature_vector: data.feature_vector,
          confidence: data.confidence,
          image_url: data.image_url,
          extract_time: new Date().toISOString(),
          description: data.description || '未命名特征'
        };
        
        this.saveFeature(featureData);
        
        this.setData({
          recognitionAccuracy: data.confidence,
          processingTime: data.processing_time
        });
      } else {
        wx.showToast({
          title: data.error || '特征提取失败',
          icon: 'error'
        });
      }
      
      this.completeExtraction();
    },
    
    // === UI交互 ===
    selectFeature: function(e) {
      const featureId = e.currentTarget.dataset.featureId;
      const feature = this.data.extractedFeatures.find(f => f.id === featureId);
      
      this.setData({
        currentFeature: feature,
        selectedPersonId: feature ? feature.person_id : null,
        showFeatureDetail: true
      });
    },
    
    closeFeatureDetail: function() {
      this.setData({
        showFeatureDetail: false,
        currentFeature: null,
        editingFeature: false
      });
    },
    
    setAsTarget: function() {
      if (!this.data.currentFeature) return;
      
      const app = getApp();
      app.globalData.targetPersonId = this.data.currentFeature.person_id;
      app.globalData.currentTarget = this.data.currentFeature;
      
      // 通知控制页面
      if (app.globalData.controlPage) {
        app.globalData.controlPage.setData({
          targetPersonId: this.data.currentFeature.person_id
        });
      }
      
      wx.showToast({
        title: '已设为跟随目标',
        icon: 'success'
      });
      
      console.log('🎯 设置跟随目标:', this.data.currentFeature.person_id);
    },
    
    refreshFeatureList: function() {
      this.loadExtractedFeatures();
    }
  }); 