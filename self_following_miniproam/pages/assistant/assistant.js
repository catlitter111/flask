// pages/assistant/assistant.js - AI助手页面
Page({

  /**
   * 页面的初始数据
   */
  data: {
    // AI对话相关
    messages: [],
    inputText: '',
    sending: false,
    connecting: false
  },

  /**
   * 生命周期函数--监听页面加载
   */
  onLoad(options) {
    console.log('🤖 AI助手页面加载');
    
    // 注册到全局应用
    const app = getApp();
    app.globalData.aiAssistantPage = this;
  },

  /**
   * 生命周期函数--监听页面初次渲染完成
   */
  onReady() {

  },

  /**
   * 生命周期函数--监听页面显示
   */
  onShow() {
    // 重新注册
    const app = getApp();
    app.globalData.aiAssistantPage = this;
  },

  /**
   * 生命周期函数--监听页面隐藏
   */
  onHide() {

  },

  /**
   * 生命周期函数--监听页面卸载
   */
  onUnload() {
    // 取消注册
    const app = getApp();
    app.globalData.aiAssistantPage = null;
  },

  // 处理AI响应
  handleAIResponse: function(data) {
    console.log('收到AI响应:', data);
    
    // 添加AI回复到消息列表
    const newMessage = {
      id: Date.now(),
      type: 'ai',
      content: data.response || data.message,
      timestamp: Date.now()
    };
    
    this.setData({
      messages: [...this.data.messages, newMessage],
      sending: false
    });
  },

  /**
   * 页面相关事件处理函数--监听用户下拉动作
   */
  onPullDownRefresh() {

  },

  /**
   * 页面上拉触底事件的处理函数
   */
  onReachBottom() {

  },

  /**
   * 用户点击右上角分享
   */
  onShareAppMessage() {

  }
})