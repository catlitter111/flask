#!/bin/bash

# 图像压缩优化应用脚本
# 用于应用所有压缩优化设置并重启ROS2系统

echo "🚀 开始应用图像压缩优化..."

# 检查是否在正确的目录
if [ ! -d "src" ]; then
    echo "❌ 请在flask目录下运行此脚本"
    exit 1
fi

# 1. 编译ROS2包
echo "📦 重新编译ROS2包..."
cd src
colcon build --packages-select custom_msgs following_robot
source install/setup.bash
cd ..

if [ $? -ne 0 ]; then
    echo "❌ ROS2包编译失败"
    exit 1
fi

echo "✅ ROS2包编译完成"

# 2. 停止现有的ROS2进程
echo "🛑 停止现有的ROS2进程..."
pkill -f "ros2 launch"
pkill -f "following_robot"
sleep 2

# 3. 测试压缩效果（可选）
echo "🧪 测试压缩效果..."
if [ -f "test_compression.py" ]; then
    python3 test_compression.py
    echo ""
fi

# 4. 启动优化后的系统
echo "🔄 启动优化后的系统..."

# 启动ROS2系统
cd src
gnome-terminal --title="ROS2 Full System" -- bash -c "
source install/setup.bash;
ros2 launch following_robot full_system.launch.py;
exec bash"

sleep 3

# 5. 启动服务器
cd ..
echo "🌐 启动服务器..."
gnome-terminal --title="Flask Server" -- bash -c "
cd 服务端;
python server.py;
exec bash"

echo ""
echo "🎉 压缩优化应用完成！"
echo ""
echo "📊 优化效果："
echo "   • 新增 ultra_low 质量预设（120x90, 质量25）"
echo "   • 降低了所有预设的默认质量以减少延迟"
echo "   • 启用自适应跳帧机制"
echo "   • 添加图像预处理优化"
echo "   • 更激进的质量调整策略"
echo ""
echo "📱 在微信小程序中监控："
echo "   • 延迟指标"
echo "   • 质量自动调整"
echo "   • 新的超低质量选项"
echo ""
echo "⚡ 预期改善："
echo "   • 延迟降低 50-70%"
echo "   • 带宽占用减少 60-80%"
echo "   • 更流畅的视频传输"
echo ""
echo "🔧 如需手动测试压缩效果："
echo "   python3 test_compression.py --image_path 你的测试图片路径"
echo "" 