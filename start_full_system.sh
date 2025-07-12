#!/bin/bash
# -*- coding: utf-8 -*-

# 完整系统启动脚本
# ===================
# 启动包含RFID监控的完整人跟随机器人系统

echo "🚀 正在启动完整的人跟随机器人系统..."
echo "==================================================="

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
WEBSOCKET_HOST=${WEBSOCKET_HOST:-"101.201.150.96"}
ENABLE_RFID=${ENABLE_RFID:-"true"}
RFID_READER_IP=${RFID_READER_IP:-"192.168.0.178"}
RFID_AUTO_START=${RFID_AUTO_START:-"false"}
USE_ACKERMANN=${USE_ACKERMANN:-"false"}
SERIAL_PORT=${SERIAL_PORT:-"/dev/ttyS7"}
ROBOT_ID=${ROBOT_ID:-"companion_robot_001"}

echo "📋 启动配置:"
echo "  WebSocket服务器: $WEBSOCKET_HOST:1234"
echo "  启用RFID监控: $ENABLE_RFID"
if [ "$ENABLE_RFID" = "true" ]; then
    echo "  RFID读写器IP: $RFID_READER_IP:4001"
    echo "  RFID自动启动: $RFID_AUTO_START"
fi
echo "  机器人ID: $ROBOT_ID"
echo "  使用阿克曼转向: $USE_ACKERMANN"
echo "  串口设备: $SERIAL_PORT"
echo ""

# 检查串口设备
if [ ! -e "$SERIAL_PORT" ]; then
    echo "⚠️ 警告: 串口设备 $SERIAL_PORT 不存在"
    echo "  请检查硬件连接或修改 SERIAL_PORT 环境变量"
    echo "  可用串口设备:"
    ls /dev/tty* | grep -E "(ttyS|ttyUSB|ttyACM)" | head -5
    echo ""
fi

# 检查网络连接
echo "🌐 检查网络连接..."
if ping -c 1 "$WEBSOCKET_HOST" &> /dev/null; then
    echo "✅ WebSocket服务器 $WEBSOCKET_HOST 可达"
else
    echo "⚠️ 警告: 无法连接到WebSocket服务器 $WEBSOCKET_HOST"
fi

if [ "$ENABLE_RFID" = "true" ]; then
    if ping -c 1 "$RFID_READER_IP" &> /dev/null; then
        echo "✅ RFID读写器 $RFID_READER_IP 可达"
    else
        echo "⚠️ 警告: 无法连接到RFID读写器 $RFID_READER_IP"
    fi
fi

echo ""

# 源化环境
echo "🔧 配置ROS2环境..."
source /opt/ros/humble/setup.bash
source install/setup.bash

# 启动完整系统
echo "🚀 启动完整系统..."
echo "按 Ctrl+C 停止系统"
echo ""

exec ros2 launch following_robot full_system.launch.py \
    websocket_host:="$WEBSOCKET_HOST" \
    robot_id:="$ROBOT_ID" \
    enable_rfid:="$ENABLE_RFID" \
    rfid_reader_ip:="$RFID_READER_IP" \
    rfid_auto_start:="$RFID_AUTO_START" \
    use_ackermann:="$USE_ACKERMANN" \
    serial_port:="$SERIAL_PORT"