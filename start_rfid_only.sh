#!/bin/bash
# -*- coding: utf-8 -*-

# RFID监控系统独立启动脚本
# ============================
# 仅启动RFID相关组件进行独立测试

echo "📡 正在启动RFID监控系统..."
echo "================================="

# 检查ROS2环境
if ! command -v ros2 &> /dev/null; then
    echo "❌ 错误: 未找到ROS2命令，请确保已正确安装和配置ROS2环境"
    echo "请运行: source /opt/ros/humble/setup.bash"
    exit 1
fi

# 检查工作空间环境
if [ ! -f "install/setup.bash" ]; then
    echo "❌ 错误: 未找到工作空间安装文件，请先构建项目"
    echo "请运行: colcon build"
    exit 1
fi

# 设置默认参数
RFID_READER_IP=${RFID_READER_IP:-"192.168.0.178"}
RFID_READER_PORT=${RFID_READER_PORT:-"4001"}
RFID_AUTO_START=${RFID_AUTO_START:-"true"}
RFID_ANTENNA_ID=${RFID_ANTENNA_ID:-"1"}
RFID_PUBLISH_RATE=${RFID_PUBLISH_RATE:-"2.0"}

echo "📋 RFID配置:"
echo "  读写器IP: $RFID_READER_IP"
echo "  读写器端口: $RFID_READER_PORT"
echo "  自动开始盘存: $RFID_AUTO_START"
echo "  默认天线: $RFID_ANTENNA_ID"
echo "  发布频率: $RFID_PUBLISH_RATE Hz"
echo ""

# 检查网络连接
echo "🌐 检查RFID读写器连接..."
if ping -c 1 "$RFID_READER_IP" &> /dev/null; then
    echo "✅ RFID读写器 $RFID_READER_IP 可达"
else
    echo "⚠️ 警告: 无法连接到RFID读写器 $RFID_READER_IP"
    echo "  请检查网络连接和设备状态"
fi

echo ""

# 源化环境
echo "🔧 配置ROS2环境..."
source /opt/ros/humble/setup.bash
source install/setup.bash

# 启动RFID节点
echo "📡 启动RFID读写器节点..."
echo "按 Ctrl+C 停止系统"
echo ""

exec ros2 run rfid_reader rfid_reader_node.py \
    --ros-args \
    -p reader_ip:="$RFID_READER_IP" \
    -p reader_port:="$RFID_READER_PORT" \
    -p auto_start:="$RFID_AUTO_START" \
    -p antenna_id:="$RFID_ANTENNA_ID" \
    -p publish_rate:="$RFID_PUBLISH_RATE"