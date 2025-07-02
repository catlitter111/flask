// pages/feature/feature.js - 特征识别页面
Page({
    data: {
      // 上传相关状态
      uploading: false,
      uploadProgress: 0,
      previewImage: '',
      currentFile: null,
      
      // 分析状态
      extracting: false,
      extracted: false,
      overallConfidence: 0,
      
      // 服装颜色识别结果
      clothingColors: {
        top: {
          name: '绿色',
          color: '#4CAF50',
          confidence: 85
        },
        bottom: {
          name: '蓝色', 
          color: '#2196F3',
          confidence: 92
        },
        shoes: {
          name: '棕色',
          color: '#795548',
          confidence: 78
        }
      },
      
      // 身体比例数据
      bodyProportions: {
        height: '175.2',
        shoulderWidth: '42.3',
        chest: '95.8',
        waist: '78.4',
        hip: '88.7'
      },
      
      // 详细比例数据（16项）
      detailedProportions: [
        { key: 'height', label: '身高', value: '175.2', unit: 'cm' },
        { key: 'head_height', label: '头部高度', value: '22.4', unit: 'cm' },
        { key: 'neck_height', label: '颈部高度', value: '12.8', unit: 'cm' },
        { key: 'shoulder_width', label: '肩膀宽度', value: '42.3', unit: 'cm' },
        { key: 'chest_width', label: '胸部宽度', value: '35.6', unit: 'cm' },
        { key: 'chest_circumference', label: '胸围', value: '95.8', unit: 'cm' },
        { key: 'waist_width', label: '腰部宽度', value: '28.9', unit: 'cm' },
        { key: 'waist_circumference', label: '腰围', value: '78.4', unit: 'cm' },
        { key: 'hip_width', label: '臀部宽度', value: '32.1', unit: 'cm' },
        { key: 'hip_circumference', label: '臀围', value: '88.7', unit: 'cm' },
        { key: 'arm_length', label: '手臂长度', value: '58.3', unit: 'cm' },
        { key: 'forearm_length', label: '前臂长度', value: '25.7', unit: 'cm' },
        { key: 'leg_length', label: '腿部长度', value: '87.5', unit: 'cm' },
        { key: 'thigh_length', label: '大腿长度', value: '45.2', unit: 'cm' },
        { key: 'calf_length', label: '小腿长度', value: '38.9', unit: 'cm' },
        { key: 'foot_length', label: '脚部长度', value: '26.4', unit: 'cm' }
      ],
      
      // 识别历史
      extractedFeatures: [],
      currentTargetId: null,
      
      // 弹窗状态
      showDetailModal: false
    },
  
    onLoad: function(options) {
      console.log('🧬 特征识别页面加载');
      
      // 注册到全局应用以接收消息
      const app = getApp();
      app.globalData.featurePage = this;
      
      // 加载历史数据
      this.loadHistoryData();
      
      // 模拟演示数据 - 实际使用时删除
      this.loadDemoData();
    },
  
    onShow: function() {
      // 页面显示时重新注册
      const app = getApp();
      app.globalData.featurePage = this;
      
      // 检查是否有新的特征数据
      this.loadHistoryData();
    },
  
    onHide: function() {
      // 页面隐藏时保存数据
      this.saveCurrentData();
    },
  
    onUnload: function() {
      // 页面卸载时取消注册
      const app = getApp();
      app.globalData.featurePage = null;
    },
  
    // 加载历史数据
    loadHistoryData: function() {
      const app = getApp();
      const features = app.globalData.extractedFeatures || [];
      
      // 格式化历史数据
      const formattedFeatures = features.map((item, index) => ({
        ...item,
        name: item.name || `目标${index + 1}`,
        topColorName: this.getColorName(item.features?.clothing_colors?.top?.color),
        bottomColorName: this.getColorName(item.features?.clothing_colors?.bottom?.color),
        topColor: item.features?.clothing_colors?.top?.color || '#4CAF50',
        bottomColor: item.features?.clothing_colors?.bottom?.color || '#2196F3',
        timestamp: this.formatTimestamp(item.timestamp),
        isFollowing: item.id === app.globalData.currentTarget?.id
      }));
  
      this.setData({
        extractedFeatures: formattedFeatures
      });
    },
  
    // 加载演示数据（实际使用时删除）
    loadDemoData: function() {
      // 模拟一些历史数据
      const demoHistory = [
        {
          id: 'demo_1',
          name: '目标A',
          topColorName: '绿色',
          bottomColorName: '蓝色',
          topColor: '#4CAF50',
          bottomColor: '#2196F3',
          timestamp: '2024-01-15 14:30',
          isFollowing: true
        },
        {
          id: 'demo_2',
          name: '目标B',
          topColorName: '蓝色',
          bottomColorName: '黑色',
          topColor: '#2196F3',
          bottomColor: '#424242',
          timestamp: '2024-01-15 13:15',
          isFollowing: false
        }
      ];
  
      this.setData({
        extractedFeatures: demoHistory,
        currentTargetId: 'demo_1'
      });
    },
  
    // 文件选择
    selectFile: function() {
      if (this.data.uploading || this.data.extracting) return;
  
      const that = this;
      
      wx.chooseMedia({
        count: 1,
        mediaType: ['image', 'video'],
        sourceType: ['album', 'camera'],
        maxDuration: 30,
        camera: 'back',
        success: function(res) {
          const file = res.tempFiles[0];
          console.log('选择的文件:', file);
          
          // 检查文件大小
          if (file.size > 10 * 1024 * 1024) {
            wx.showToast({
              title: '文件大小不能超过10MB',
              icon: 'none'
            });
            return;
          }
          
          that.setData({
            currentFile: {
              name: `${file.tempFilePath.split('/').pop()}`,
              size: file.size,
              type: file.fileType
            },
            previewImage: file.tempFilePath
          });
          
          // 开始上传和分析
          that.uploadAndAnalyze(file.tempFilePath);
        },
        fail: function(error) {
          console.error('文件选择失败:', error);
          wx.showToast({
            title: '文件选择失败',
            icon: 'none'
          });
        }
      });
    },
  
    // 拍照
    takePhoto: function() {
      if (this.data.uploading || this.data.extracting) return;
  
      const that = this;
      
      wx.chooseMedia({
        count: 1,
        mediaType: ['image'],
        sourceType: ['camera'],
        camera: 'back',
        success: function(res) {
          const file = res.tempFiles[0];
          
          that.setData({
            currentFile: {
              name: '拍摄照片.jpg',
              size: file.size,
              type: 'image'
            },
            previewImage: file.tempFilePath
          });
          
          that.uploadAndAnalyze(file.tempFilePath);
        }
      });
    },
  
    // 录制视频
    recordVideo: function() {
      if (this.data.uploading || this.data.extracting) return;
  
      const that = this;
      
      wx.chooseMedia({
        count: 1,
        mediaType: ['video'],
        sourceType: ['camera'],
        maxDuration: 30,
        camera: 'back',
        success: function(res) {
          const file = res.tempFiles[0];
          
          that.setData({
            currentFile: {
              name: '录制视频.mp4',
              size: file.size,
              type: 'video'
            },
            previewImage: file.thumbTempFilePath || file.tempFilePath
          });
          
          that.uploadAndAnalyze(file.tempFilePath);
        }
      });
    },
  
    // 移除图片
    removeImage: function() {
      this.setData({
        previewImage: '',
        currentFile: null,
        extracted: false,
        extracting: false
      });
    },
  
    // 上传并分析
    uploadAndAnalyze: function(filePath) {
      const that = this;
      
      // 开始上传
      this.setData({
        uploading: true,
        uploadProgress: 0,
        extracted: false,
        extracting: false
      });
  
      // 模拟上传进度
      const uploadTimer = setInterval(() => {
        const progress = that.data.uploadProgress + 10;
        that.setData({
          uploadProgress: progress
        });
        
        if (progress >= 100) {
          clearInterval(uploadTimer);
          that.setData({
            uploading: false,
            extracting: true
          });
          
          // 开始特征提取
          that.startFeatureExtraction(filePath);
        }
      }, 200);
  
      // 实际上传逻辑
      // this.uploadFile(filePath);
    },
  
    // 开始特征提取
    startFeatureExtraction: function(filePath) {
      const that = this;
      
      console.log('🔍 开始特征提取:', filePath);
      
      // 发送特征提取请求到服务器
      this.sendFeatureExtractionRequest(filePath);
      
      // 模拟分析进度（实际应从服务器接收）
      setTimeout(() => {
        that.setData({
          extracting: false,
          extracted: true,
          overallConfidence: 89,
          // 模拟返回的数据 - 实际应从服务器获取
          clothingColors: {
            top: {
              name: '绿色',
              color: '#4CAF50', 
              confidence: 85
            },
            bottom: {
              name: '蓝色',
              color: '#2196F3',
              confidence: 92
            },
            shoes: {
              name: '棕色',
              color: '#795548',
              confidence: 78
            }
          }
        });
        
        wx.showToast({
          title: '特征提取完成',
          icon: 'success'
        });
      }, 3000);
    },
  
    // 发送特征提取请求
    sendFeatureExtractionRequest: function(filePath) {
      const app = getApp();
      
      if (!app.globalData.connected) {
        wx.showToast({
          title: '服务器未连接',
          icon: 'none'
        });
        this.setData({
          extracting: false
        });
        return;
      }
  
      // 这里实现文件上传和特征提取请求
      // 实际实现时需要：
      // 1. 将文件上传到服务器
      // 2. 发送特征提取命令
      // 3. 等待服务器返回结果
      
      app.sendSocketMessage({
        type: 'feature_extraction_request',
        robot_id: app.globalData.robotId,
        file_path: filePath,
        extract_clothing_colors: true,
        extract_body_proportions: true,
        timestamp: Date.now()
      });
    },
  
    // 处理特征提取结果（由app.js调用）
    handleFeatureResult: function(data) {
      console.log('📊 收到特征提取结果:', data);
      
      if (data.status === 'success') {
        this.setData({
          extracting: false,
          extracted: true,
          overallConfidence: data.confidence || 89,
          clothingColors: this.formatClothingColors(data.features.clothing_colors),
          bodyProportions: this.formatBodyProportions(data.features.body_proportions),
          detailedProportions: this.formatDetailedProportions(data.features.detailed_proportions)
        });
        
        wx.showToast({
          title: '特征提取完成',
          icon: 'success'
        });
      } else {
        this.setData({
          extracting: false
        });
        
        wx.showToast({
          title: data.error || '特征提取失败',
          icon: 'none'
        });
      }
    },
  
    // 格式化服装颜色数据
    formatClothingColors: function(colors) {
      return {
        top: {
          name: colors.top.name || '未知',
          color: colors.top.hex_color || '#4CAF50',
          confidence: Math.round(colors.top.confidence * 100)
        },
        bottom: {
          name: colors.bottom.name || '未知',
          color: colors.bottom.hex_color || '#2196F3',
          confidence: Math.round(colors.bottom.confidence * 100)
        },
        shoes: colors.shoes ? {
          name: colors.shoes.name || '未知',
          color: colors.shoes.hex_color || '#795548',
          confidence: Math.round(colors.shoes.confidence * 100)
        } : null
      };
    },
  
    // 格式化身体比例数据
    formatBodyProportions: function(proportions) {
      return {
        height: proportions.height?.toFixed(1) || '0.0',
        shoulderWidth: proportions.shoulder_width?.toFixed(1) || '0.0',
        chest: proportions.chest_circumference?.toFixed(1) || '0.0',
        waist: proportions.waist_circumference?.toFixed(1) || '0.0',
        hip: proportions.hip_circumference?.toFixed(1) || '0.0'
      };
    },
  
    // 格式化详细比例数据
    formatDetailedProportions: function(detailed) {
      // 将16项详细数据转换为显示格式
      const keys = [
        'height', 'head_height', 'neck_height', 'shoulder_width',
        'chest_width', 'chest_circumference', 'waist_width', 'waist_circumference',
        'hip_width', 'hip_circumference', 'arm_length', 'forearm_length',
        'leg_length', 'thigh_length', 'calf_length', 'foot_length'
      ];
      
      const labels = {
        'height': '身高',
        'head_height': '头部高度',
        'neck_height': '颈部高度',
        'shoulder_width': '肩膀宽度',
        'chest_width': '胸部宽度',
        'chest_circumference': '胸围',
        'waist_width': '腰部宽度',
        'waist_circumference': '腰围',
        'hip_width': '臀部宽度',
        'hip_circumference': '臀围',
        'arm_length': '手臂长度',
        'forearm_length': '前臂长度',
        'leg_length': '腿部长度',
        'thigh_length': '大腿长度',
        'calf_length': '小腿长度',
        'foot_length': '脚部长度'
      };
      
      return keys.map(key => ({
        key: key,
        label: labels[key],
        value: detailed[key]?.toFixed(1) || '0.0',
        unit: 'cm'
      }));
    },
  
    // 显示详细数据
    showDetailedData: function() {
      this.setData({
        showDetailModal: true
      });
    },
  
    // 隐藏详细数据弹窗
    hideDetailModal: function() {
      this.setData({
        showDetailModal: false
      });
    },
  
    // 阻止事件冒泡
    stopPropagation: function() {
      // 阻止点击弹窗内容时关闭弹窗
    },
  
    // 选择历史目标
    selectHistoryTarget: function(e) {
      const target = e.currentTarget.dataset.target;
      
      this.setData({
        currentTargetId: target.id
      });
      
      // 加载历史目标的数据
      this.loadHistoryTarget(target);
      
      wx.showToast({
        title: `已选择${target.name}`,
        icon: 'success'
      });
    },
  
    // 加载历史目标数据
    loadHistoryTarget: function(target) {
      // 从历史数据中恢复特征信息
      if (target.features) {
        this.setData({
          extracted: true,
          clothingColors: this.formatClothingColors(target.features.clothing_colors),
          bodyProportions: this.formatBodyProportions(target.features.body_proportions),
          detailedProportions: this.formatDetailedProportions(target.features.detailed_proportions),
          overallConfidence: target.confidence || 89
        });
      }
    },
  
    // 保存特征
    saveFeatures: function() {
      if (!this.data.extracted) {
        wx.showToast({
          title: '请先完成特征提取',
          icon: 'none'
        });
        return;
      }
  
      const app = getApp();
      
      // 创建特征数据
      const featureData = {
        id: `feature_${Date.now()}`,
        timestamp: Date.now(),
        features: {
          clothing_colors: {
            top: {
              name: this.data.clothingColors.top.name,
              color: this.data.clothingColors.top.color,
              confidence: this.data.clothingColors.top.confidence / 100
            },
            bottom: {
              name: this.data.clothingColors.bottom.name,
              color: this.data.clothingColors.bottom.color,
              confidence: this.data.clothingColors.bottom.confidence / 100
            }
          },
          body_proportions: this.data.bodyProportions,
          detailed_proportions: this.data.detailedProportions
        },
        confidence: this.data.overallConfidence,
        image_data: this.data.previewImage
      };
  
      // 保存到全局数据
      app.globalData.extractedFeatures.push(featureData);
      
      // 保存到本地存储
      wx.setStorageSync('extractedFeatures', app.globalData.extractedFeatures);
      
      // 重新加载历史数据
      this.loadHistoryData();
      
      wx.showToast({
        title: '特征已保存',
        icon: 'success'
      });
    },
  
    // 设为目标
    setAsTarget: function() {
      if (!this.data.extracted) {
        wx.showToast({
          title: '请先完成特征提取',
          icon: 'none'
        });
        return;
      }
  
      const app = getApp();
      
      // 设置当前目标
      app.globalData.currentTarget = {
        id: `target_${Date.now()}`,
        features: {
          clothing_colors: this.data.clothingColors,
          body_proportions: this.data.bodyProportions
        },
        confidence: this.data.overallConfidence
      };
  
      // 发送设置目标命令
      app.sendSocketMessage({
        type: 'set_tracking_target',
        robot_id: app.globalData.robotId,
        target_features: app.globalData.currentTarget.features,
        timestamp: Date.now()
      });
  
      wx.showToast({
        title: '目标已设置',
        icon: 'success'
      });
    },
  
    // 开始跟随
    startFollowing: function() {
      if (!this.data.extracted) {
        wx.showToast({
          title: '请先完成特征提取',
          icon: 'none'
        });
        return;
      }
  
      const app = getApp();
      
      // 先设为目标
      this.setAsTarget();
      
      // 发送开始跟随命令
      setTimeout(() => {
        app.sendSocketMessage({
          type: 'start_following',
          robot_id: app.globalData.robotId,
          target_id: app.globalData.currentTarget.id,
          timestamp: Date.now()
        });
        
        wx.showToast({
          title: '开始跟随',
          icon: 'success'
        });
        
        // 跳转到控制页面
        wx.switchTab({
          url: '/pages/control/control'
        });
      }, 500);
    },
  
    // 保存当前数据
    saveCurrentData: function() {
      // 保存当前分析状态到本地存储
      if (this.data.extracted) {
        const currentData = {
          extracted: this.data.extracted,
          clothingColors: this.data.clothingColors,
          bodyProportions: this.data.bodyProportions,
          overallConfidence: this.data.overallConfidence,
          previewImage: this.data.previewImage
        };
        
        wx.setStorageSync('currentFeatureData', currentData);
      }
    },
  
    // 工具函数：获取颜色名称
    getColorName: function(hexColor) {
      const colorMap = {
        '#4CAF50': '绿色',
        '#2196F3': '蓝色', 
        '#795548': '棕色',
        '#F44336': '红色',
        '#FF9800': '橙色',
        '#9C27B0': '紫色',
        '#607D8B': '灰色',
        '#424242': '黑色',
        '#FFFFFF': '白色'
      };
      
      return colorMap[hexColor] || '未知';
    },
  
    // 工具函数：格式化时间戳
    formatTimestamp: function(timestamp) {
      const date = new Date(timestamp);
      const year = date.getFullYear();
      const month = String(date.getMonth() + 1).padStart(2, '0');
      const day = String(date.getDate()).padStart(2, '0');
      const hour = String(date.getHours()).padStart(2, '0');
      const minute = String(date.getMinutes()).padStart(2, '0');
      
      return `${year}-${month}-${day} ${hour}:${minute}`;
    },
  
    // 处理连接状态更新（由app.js调用）
    updateConnectionStatus: function(isConnected, robotId) {
      // 更新连接状态相关的UI
      console.log('🔗 特征识别页面 - 连接状态更新:', isConnected);
    },
  
    // 处理机器人断开连接（由app.js调用）
    handleCompanionDisconnected: function(data) {
      console.log('💔 特征识别页面 - 机器人断开连接');
      
      // 如果正在进行特征提取，停止操作
      if (this.data.extracting) {
        this.setData({
          extracting: false
        });
        
        wx.showToast({
          title: '连接断开，提取已停止',
          icon: 'none'
        });
      }
    }
  });