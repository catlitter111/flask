#!/bin/bash
# -*- coding: utf-8 -*-

# RFID系统简化测试脚本
# ======================
# 用于快速诊断RFID功能是否正常

echo "🔍 RFID系统诊断测试"
echo "==================="

# 源化环境
source /opt/ros/humble/setup.bash
source install/setup.bash

echo "📋 检查ROS2节点状态..."
echo "活跃节点列表:"
ros2 node list | grep -E "(rfid|websocket|bytetracker)" || echo "  未发现相关节点"

echo ""
echo "📡 检查RFID相关话题..."
echo "RFID话题列表:"
ros2 topic list | grep rfid || echo "  未发现RFID话题"

echo ""
echo "⚙️ 检查RFID服务..."
echo "RFID服务列表:"
ros2 service list | grep rfid || echo "  未发现RFID服务"

echo ""
echo "🔧 测试RFID服务可用性..."
if ros2 service list | grep -q "/rfid/command"; then
    echo "✅ RFID命令服务存在"
    
    echo "📞 测试服务调用..."
    timeout 3 ros2 service call /rfid/command rfid_reader/srv/RfidCommand \
        '{command: "get_status", antenna_id: 1, ip_address: "", port: 0}' 2>/dev/null
    
    if [ $? -eq 0 ]; then
        echo "✅ RFID服务调用成功"
    elif [ $? -eq 124 ]; then
        echo "⏰ RFID服务调用超时（可能RFID硬件未连接）"
    else
        echo "❌ RFID服务调用失败"
    fi
else
    echo "❌ RFID命令服务不存在"
fi

echo ""
echo "📊 检查WebSocket桥接节点状态..."
if ros2 node list | grep -q "websocket_bridge"; then
    echo "✅ WebSocket桥接节点运行中"
    
    echo "🔍 检查节点信息..."
    ros2 node info /websocket_bridge_node | grep -E "(Subscribers|Publishers|Service Servers|Service Clients)" -A 3
else
    echo "❌ WebSocket桥接节点未运行"
fi

echo ""
echo "🏁 诊断完成"
echo "=================="

echo ""
echo "💡 故障排除建议:"
echo "1. 如果RFID服务不存在，请启动RFID节点："
echo "   ros2 run rfid_reader rfid_reader_node.py"
echo ""
echo "2. 如果服务调用超时，请检查RFID硬件连接："
echo "   - 确保RFID读写器已连接到网络"
echo "   - 检查IP地址：192.168.0.178"
echo "   - 检查端口：4001"
echo ""
echo "3. 如果WebSocket桥接节点未运行，请重启系统："
echo "   ros2 launch following_robot full_system.launch.py"
echo ""
echo "4. 检查日志获取详细错误信息："
echo "   ros2 log level websocket_bridge_node debug"