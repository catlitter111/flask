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
          
          // 提取文件名，处理可能的路径分隔符问题
          const fileName = file.tempFilePath.split('/').pop() || file.tempFilePath.split('\\').pop() || 'unknown_file';
          
          console.log('📁 设置文件信息:', {
            fileName: fileName,
            size: file.size,
            type: file.fileType
          });
          
          // 处理文件名显示 - 如果太长则截断
          const displayName = fileName.length > 25 ? 
            fileName.substring(0, 12) + '...' + fileName.substring(fileName.length - 8) : 
            fileName;
          
          that.setData({
            currentFile: {
              name: fileName,
              displayName: displayName,
              size: file.size,
              type: file.fileType || 'unknown',
              sizeText: (file.size / 1024 / 1024).toFixed(2) + 'MB'
            },
            previewImage: file.tempFilePath
          });
          
          console.log('📄 当前文件信息已设置:', that.data.currentFile);
          
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
          
          console.log('📷 拍照成功:', file);
          
          that.setData({
            currentFile: {
              name: '拍摄照片.jpg',
              displayName: '拍摄照片.jpg',
              size: file.size,
              type: 'image',
              sizeText: (file.size / 1024 / 1024).toFixed(2) + 'MB'
            },
            previewImage: file.tempFilePath
          });
          
          console.log('📄 拍照文件信息已设置:', that.data.currentFile);
          
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
          
          console.log('🎥 录制视频成功:', file);
          
          that.setData({
            currentFile: {
              name: '录制视频.mp4',
              displayName: '录制视频.mp4',
              size: file.size,
              type: 'video',
              sizeText: (file.size / 1024 / 1024).toFixed(2) + 'MB'
            },
            previewImage: file.thumbTempFilePath || file.tempFilePath
          });
          
          console.log('📄 录制视频文件信息已设置:', that.data.currentFile);
          
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
  
      // 实际上传文件
      this.uploadFile(filePath);
    },

    // 上传文件到服务器
    uploadFile: function(filePath) {
      const that = this;
      const app = getApp();
      
      if (!app.globalData.connected) {
        wx.showToast({
          title: '服务器未连接',
          icon: 'none'
        });
        this.setData({
          uploading: false
        });
        return;
      }

      // 构建上传URL
      const uploadUrl = `http://101.201.150.96:1235/api/upload/${app.globalData.clientId}`;
      
      console.log('🚀 开始上传文件:', filePath);
      console.log('📡 上传地址:', uploadUrl);

      // 上传文件
      const uploadTask = wx.uploadFile({
        url: uploadUrl,
        filePath: filePath,
        name: 'file',
        formData: {
          'client_id': app.globalData.clientId,
          'timestamp': Date.now()
        },
        success: function(res) {
          console.log('✅ 文件上传成功:', res);
          
          try {
            const response = JSON.parse(res.data);
            if (response.success) {
              that.setData({
                uploading: false,
                uploadProgress: 100,
                extracting: true
              });
              
              // 开始特征提取
              that.startFeatureExtraction(response.file_id);
            } else {
              throw new Error(response.error || '上传失败');
            }
          } catch (error) {
            console.error('❌ 上传响应解析失败:', error);
            that.handleUploadError('上传响应解析失败');
          }
        },
        fail: function(error) {
          console.error('❌ 文件上传失败:', error);
          that.handleUploadError(error.errMsg || '上传失败');
        }
      });

      // 监听上传进度
      uploadTask.onProgressUpdate(function(res) {
        that.setData({
          uploadProgress: res.progress
        });
        console.log('📊 上传进度:', res.progress + '%');
      });
    },

    // 处理上传错误
    handleUploadError: function(errorMsg) {
      console.error('❌ 上传错误:', errorMsg);
      console.log('📄 保留文件信息:', this.data.currentFile);
      
      this.setData({
        uploading: false,
        uploadProgress: 0,
        extracting: false
        // 不清除 currentFile 和 previewImage，让用户知道是哪个文件上传失败
      });
      
      wx.showToast({
        title: errorMsg,
        icon: 'none',
        duration: 3000
      });
    },
  
    // 开始特征提取
    startFeatureExtraction: function(fileId) {
      const that = this;
      
      console.log('🔍 开始特征提取:', fileId);
      
      // 发送特征提取请求到服务器
      this.sendFeatureExtractionRequest(fileId);
      
      // 等待服务器返回结果（不再使用模拟数据）
      // 实际结果会通过handleFeatureResult方法接收
    },
  
    // 发送特征提取请求
    sendFeatureExtractionRequest: function(fileId) {
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

      // 发送特征提取请求（文件已经上传到服务器）
      app.sendSocketMessage({
        type: 'feature_extraction_request',
        robot_id: app.globalData.robotId,
        file_id: fileId,
        extract_clothing_colors: true,
        extract_body_proportions: true,
        timestamp: Date.now()
      });

      console.log('📤 已发送特征提取请求 - 文件ID:', fileId);
    },
  
    // 处理文件上传成功（由app.js调用）
    handleFileUploadSuccess: function(data) {
      console.log('📁 收到文件上传成功消息:', data);
      // 这个消息通常已经在上传过程中处理了，这里只是备用
    },

    // 处理特征提取结果（由app.js调用）
    handleFeatureResult: function(data) {
      console.log('📊 收到特征提取结果:', data);
      
      if (data.status === 'success') {
        this.setData({
          extracting: false,
          extracted: true,
          overallConfidence: Math.round((data.confidence || 0.89) * 100),
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

    // 处理特征提取错误（由app.js调用）
    handleFeatureError: function(data) {
      console.error('❌ 特征提取错误:', data);
      
      this.setData({
        extracting: false,
        extracted: false
      });
      
      wx.showToast({
        title: data.error || '特征提取失败',
        icon: 'none',
        duration: 3000
      });
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
  
    // 处理文件保存结果（由app.js调用）
    handleFileSaveResult: function(data) {
      console.log('📁 收到文件保存结果:', data);
      
      const status = data.status;
      
      if (status === 'success') {
        console.log('✅ 文件已保存到ROS2节点:', data.saved_path);
        wx.showToast({
          title: '文件已转发到机器人',
          icon: 'success',
          duration: 2000
        });
        
        // 可以在这里触发特征提取或其他后续操作
        if (this.data.currentFile) {
          this.setData({
            'currentFile.robotSaved': true,
            'currentFile.savedPath': data.saved_path
          });
        }
      } else {
        console.error('❌ 文件保存失败:', data.error);
        wx.showToast({
          title: '文件转发失败',
          icon: 'none',
          duration: 2000
        });
        
        if (this.data.currentFile) {
          this.setData({
            'currentFile.robotSaved': false,
            'currentFile.error': data.error
          });
        }
      }
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