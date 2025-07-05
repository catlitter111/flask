// pages/feature/detail/detail.js - 特征识别详情页面
Page({
  data: {
    target: null,           // 当前查看的目标
    showImageModal: false,  // 是否显示图片模态框
    showColorDetails: false, // 是否显示颜色详情
    showProportionDetails: false, // 是否显示比例详情
    
    // 当前显示的图片
    currentImage: '',
    
    // 详细的身体比例标签
    proportionLabels: [
      { key: 'height', label: '身高', unit: 'cm' },
      { key: 'shoulderWidth', label: '肩宽', unit: 'cm' },
      { key: 'chest', label: '胸围', unit: 'cm' },
      { key: 'waist', label: '腰围', unit: 'cm' },
      { key: 'hip', label: '臀围', unit: 'cm' },
      { key: 'armLength', label: '臂长', unit: 'cm' },
      { key: 'legLength', label: '腿长', unit: 'cm' },
      { key: 'headHeight', label: '头高', unit: 'cm' },
      { key: 'neckHeight', label: '颈高', unit: 'cm' },
      { key: 'torsoLength', label: '躯干长', unit: 'cm' },
      { key: 'thighLength', label: '大腿长', unit: 'cm' },
      { key: 'calfLength', label: '小腿长', unit: 'cm' },
      { key: 'footLength', label: '脚长', unit: 'cm' },
      { key: 'handLength', label: '手长', unit: 'cm' },
      { key: 'forearmLength', label: '前臂长', unit: 'cm' },
      { key: 'upperArmLength', label: '上臂长', unit: 'cm' }
    ]
  },

  onLoad: function(options) {
    console.log('🔍 特征详情页面加载');
    
    // 从全局数据获取目标信息
    const app = getApp();
    const target = app.globalData.currentHistoryTarget;
    
    if (target) {
      // 处理和格式化数据
      const processedTarget = this.processTargetData(target);
      
      this.setData({
        target: processedTarget
      });
      console.log('📊 加载目标详情:', processedTarget);
    } else {
      wx.showToast({
        title: '目标数据不存在',
        icon: 'none'
      });
      setTimeout(() => {
        wx.navigateBack();
      }, 1500);
    }
  },

  // 处理目标数据，确保格式正确
  processTargetData: function(rawTarget) {
    console.log('🔄 [详情页] 处理原始数据:', rawTarget);
    
    // 详细调试：显示所有图片相关字段
    console.log('🔍 [详情页图片调试] 所有可能的图片字段:');
    console.log('🔍 [详情页图片调试] rawTarget.image_data存在:', !!rawTarget.image_data);
    console.log('🔍 [详情页图片调试] rawTarget.processed_image存在:', !!rawTarget.processed_image);
    console.log('🔍 [详情页图片调试] rawTarget.result_image存在:', !!rawTarget.result_image);
    console.log('🔍 [详情页图片调试] rawTarget.previewImage存在:', !!rawTarget.previewImage);
    console.log('🔍 [详情页图片调试] 所有keys:', Object.keys(rawTarget));
    
    // 提取身体比例数据
    const body_proportions = rawTarget.features?.body_proportions || rawTarget.body_proportions || {};
    const detailed_proportions = rawTarget.features?.detailed_proportions || rawTarget.detailed_proportions || [];
    
    console.log('📊 [详情页] 提取的比例数据:', {
      body_proportions: body_proportions,
      detailed_proportions: detailed_proportions,
      'body_proportions类型': typeof body_proportions,
      'body_proportions键数': Object.keys(body_proportions).length,
      'detailed_proportions类型': Array.isArray(detailed_proportions),
      'detailed_proportions长度': detailed_proportions.length
    });
    
    // 处理图片数据，确保有处理后的图片
    const image_data = rawTarget.image_data || rawTarget.previewImage || '';
    
    // 优先使用处理后的图片，但加强调试
    let processed_image = '';
    if (rawTarget.processed_image) {
      processed_image = rawTarget.processed_image;
      console.log('🖼️ [详情页] 使用processed_image字段');
    } else if (rawTarget.result_image) {
      processed_image = rawTarget.result_image;
      console.log('🖼️ [详情页] 使用result_image字段');
    } else {
      processed_image = image_data;
      console.log('🖼️ [详情页] 回退到image_data字段');
    }
    
    const result_image = rawTarget.result_image || processed_image || image_data;
    
    console.log('🖼️ [详情页] 图片数据处理结果:', {
      has_image_data: !!image_data,
      has_processed_image: !!processed_image,
      has_result_image: !!result_image,
      are_images_same: image_data === processed_image,
      processed_image_preview: processed_image ? processed_image.substring(0, 50) + '...' : 'none'
    });
    
    const processed = {
      ...rawTarget,
      // 确保时间戳格式正确
      timestamp: this.formatTimestamp(rawTarget.timestamp),
      // 确保颜色数据格式正确
      topColor: rawTarget.topColor || rawTarget.features?.clothing_colors?.top?.color || '#4CAF50',
      bottomColor: rawTarget.bottomColor || rawTarget.features?.clothing_colors?.bottom?.color || '#2196F3',
      topColorName: rawTarget.topColorName || rawTarget.features?.clothing_colors?.top?.name || '绿色',
      bottomColorName: rawTarget.bottomColorName || rawTarget.features?.clothing_colors?.bottom?.name || '蓝色',
      // 处理置信度（乘以100恢复百分比）
      features: {
        ...rawTarget.features,
        clothing_colors: {
          top: {
            ...rawTarget.features?.clothing_colors?.top,
            confidence: (rawTarget.features?.clothing_colors?.top?.confidence || 85)
          },
          bottom: {
            ...rawTarget.features?.clothing_colors?.bottom,
            confidence: (rawTarget.features?.clothing_colors?.bottom?.confidence || 85)
          }
        },
        body_proportions: body_proportions,
        detailed_proportions: detailed_proportions
      },
      // 确保有图片数据，优先使用处理后的图片
      image_data: image_data,
      processed_image: processed_image,
      result_image: result_image,
      // 确保有身体比例数据（顶层也放一份，兼容多种访问方式）
      body_proportions: body_proportions,
      detailed_proportions: detailed_proportions
    };
    
    console.log('✅ [详情页] 处理后的数据:', {
      id: processed.id,
      name: processed.name,
      has_processed_image: !!processed.processed_image,
      topColor: processed.topColor,
      bottomColor: processed.bottomColor,
      body_proportions_keys: Object.keys(processed.body_proportions || {}),
      detailed_proportions_length: (processed.detailed_proportions || []).length
    });
    return processed;
  },

  // 格式化时间戳
  formatTimestamp: function(timestamp) {
    if (!timestamp) return '未知时间';
    
    let date;
    
    // 如果timestamp是字符串格式，需要转换为iOS兼容格式
    if (typeof timestamp === 'string') {
      // 将 "yyyy-MM-dd HH:mm" 转换为 "yyyy/MM/dd HH:mm:ss" (iOS兼容)
      const isoString = timestamp.replace(/(\d{4})-(\d{2})-(\d{2}) (\d{2}):(\d{2})/, '$1/$2/$3 $4:$5:00');
      date = new Date(isoString);
      
      // 如果转换失败，尝试直接解析
      if (isNaN(date.getTime())) {
        date = new Date(timestamp);
      }
    } else {
      // 数字时间戳
      date = new Date(timestamp);
    }
    
    // 验证日期是否有效
    if (isNaN(date.getTime())) {
      console.warn('⚠️ 无效的时间戳:', timestamp);
      return '时间格式错误';
    }
    
    const year = date.getFullYear();
    const month = String(date.getMonth() + 1).padStart(2, '0');
    const day = String(date.getDate()).padStart(2, '0');
    const hour = String(date.getHours()).padStart(2, '0');
    const minute = String(date.getMinutes()).padStart(2, '0');
    
    return `${year}-${month}-${day} ${hour}:${minute}`;
  },

  onShow: function() {
    // 设置导航栏标题
    if (this.data.target) {
      wx.setNavigationBarTitle({
        title: this.data.target.name || '目标详情'
      });
    }
  },

  // 查看原图
  viewOriginalImage: function() {
    if (this.data.target && this.data.target.image_data) {
      this.setData({
        currentImage: this.data.target.image_data,
        showImageModal: true
      });
    } else {
      wx.showToast({
        title: '没有原图数据',
        icon: 'none'
      });
    }
  },

  // 查看处理后的图片
  viewProcessedImage: function() {
    console.log('🖼️ [查看处理图片] 点击处理结果按钮');
    console.log('🖼️ [查看处理图片] target存在:', !!this.data.target);
    console.log('🖼️ [查看处理图片] processed_image存在:', !!this.data.target?.processed_image);
    if (this.data.target?.processed_image) {
      console.log('🖼️ [查看处理图片] processed_image预览:', this.data.target.processed_image.substring(0, 80) + '...');
    }
    if (this.data.target?.image_data) {
      console.log('🖼️ [查看处理图片] image_data预览:', this.data.target.image_data.substring(0, 80) + '...');
    }
    console.log('🖼️ [查看处理图片] 两者是否相同:', this.data.target?.processed_image === this.data.target?.image_data);
    
    if (this.data.target && this.data.target.processed_image) {
      this.setData({
        currentImage: this.data.target.processed_image,
        showImageModal: true
      });
      console.log('🖼️ [查看处理图片] 显示处理后图片');
    } else {
      console.log('🖼️ [查看处理图片] 没有处理后图片数据');
      wx.showToast({
        title: '没有处理后的图片',
        icon: 'none'
      });
    }
  },

  // 关闭图片模态框
  closeImageModal: function() {
    this.setData({
      showImageModal: false,
      currentImage: ''
    });
  },

  // 预览图片
  previewImage: function() {
    if (this.data.currentImage) {
      wx.previewImage({
        current: this.data.currentImage,
        urls: [this.data.currentImage]
      });
    }
  },

  // 切换颜色详情显示
  toggleColorDetails: function() {
    this.setData({
      showColorDetails: !this.data.showColorDetails
    });
  },

  // 切换比例详情显示
  toggleProportionDetails: function() {
    this.setData({
      showProportionDetails: !this.data.showProportionDetails
    });
  },

  // 设为当前目标
  setAsCurrentTarget: function() {
    const app = getApp();
    const target = this.data.target;
    
    if (!target) {
      wx.showToast({
        title: '目标数据不存在',
        icon: 'none'
      });
      return;
    }

    // 设置为当前跟随目标
    app.globalData.currentTarget = {
      id: target.id,
      features: target.features,
      name: target.name
    };

    // 发送消息到机器人
    if (app.globalData.connected) {
      app.sendSocketMessage({
        type: 'set_tracking_target',
        robot_id: app.globalData.robotId,
        target_features: target.features,
        target_name: target.name,
        timestamp: Date.now()
      });
    }

    wx.showToast({
      title: '已设为当前目标',
      icon: 'success'
    });
  },

  // 开始跟随
  startFollowing: function() {
    const app = getApp();
    
    if (!app.globalData.connected) {
      wx.showToast({
        title: '机器人未连接',
        icon: 'none'
      });
      return;
    }

    // 先设为目标
    this.setAsCurrentTarget();

    // 延时发送跟随命令
    setTimeout(() => {
      app.sendSocketMessage({
        type: 'start_following',
        robot_id: app.globalData.robotId,
        target_id: this.data.target.id,
        timestamp: Date.now()
      });

      wx.showToast({
        title: '开始跟随',
        icon: 'success'
      });

      // 跳转到控制页面
      setTimeout(() => {
        wx.switchTab({
          url: '/pages/control/control'
        });
      }, 1000);
    }, 300);
  },

  // 删除历史记录
  deleteHistory: function() {
    const that = this;
    
    wx.showModal({
      title: '确认删除',
      content: '确定要删除这条历史记录吗？删除后无法恢复。',
      confirmText: '删除',
      confirmColor: '#ff4444',
      success: function(res) {
        if (res.confirm) {
          that.performDelete();
        }
      }
    });
  },

  // 执行删除操作
  performDelete: function() {
    const app = getApp();
    const targetId = this.data.target.id;
    
    // 从全局数据中删除
    const features = app.globalData.extractedFeatures || [];
    const updatedFeatures = features.filter(item => item.id !== targetId);
    app.globalData.extractedFeatures = updatedFeatures;
    
    // 更新本地存储
    wx.setStorageSync('extractedFeatures', updatedFeatures);
    
    wx.showToast({
      title: '已删除',
      icon: 'success'
    });
    
    // 返回上一页
    setTimeout(() => {
      wx.navigateBack();
    }, 1000);
  },

  // 分享功能
  shareTarget: function() {
    const target = this.data.target;
    
    if (!target) return;
    
    // 生成分享内容
    const shareContent = `特征目标：${target.name}\n颜色：${target.topColorName}衣${target.bottomColorName}裤\n时间：${target.timestamp}`;
    
    wx.setClipboardData({
      data: shareContent,
      success: function() {
        wx.showToast({
          title: '已复制到剪贴板',
          icon: 'success'
        });
      }
    });
  },

  // 分享目标信息
  shareTarget: function() {
    const target = this.data.target;
    if (!target) {
      wx.showToast({
        title: '目标数据不存在',
        icon: 'none'
      });
      return;
    }

    const shareText = `目标信息：${target.name || '未命名目标'}
服装颜色：${target.topColorName}衣${target.bottomColorName}裤
识别时间：${target.timestamp}
置信度：${target.confidence || '未知'}%`;

    // 复制到剪贴板
    wx.setClipboardData({
      data: shareText,
      success: function() {
        wx.showToast({
          title: '已复制到剪贴板',
          icon: 'success'
        });
      },
      fail: function() {
        wx.showToast({
          title: '复制失败',
          icon: 'none'
        });
      }
    });
  },

  // 返回上一页
  navigateBack: function() {
    wx.navigateBack();
  },

  // 阻止事件冒泡
  stopPropagation: function() {
    // 防止点击模态框内容时关闭对话框
  }
}); 