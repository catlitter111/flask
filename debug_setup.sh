#!/bin/bash

# ByteTracker 调试环境设置脚本
# 使用方法: source debug_setup.sh

echo "🔧 正在设置 ByteTracker 调试环境..."

# 设置ROS2环境
source /opt/ros/humble/setup.bash 2>/dev/null || echo "⚠️ 请确保已安装ROS2 Humble"

# 设置工作空间环境
if [ -f "./install/setup.bash" ]; then
    source ./install/setup.bash
    echo "✅ 已加载工作空间环境"
else
    echo "⚠️ 未找到工作空间安装目录，请先运行 colcon build"
fi

# 设置调试环境变量
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export BYTETRACKER_DEBUG_MODE=1
export BYTETRACKER_VERBOSE=1
export PYTHONUNBUFFERED=1
export PYTHONDONTWRITEBYTECODE=1
export OPENCV_LOG_LEVEL=ERROR

# 检查Python依赖
echo "🔍 检查Python依赖..."
python3 -c "
import sys
required_modules = ['rclpy', 'cv2', 'numpy', 'openpyxl', 'threading']
missing_modules = []

for module in required_modules:
    try:
        __import__(module)
        print(f'✅ {module}')
    except ImportError:
        missing_modules.append(module)
        print(f'❌ {module} - 未安装')

if missing_modules:
    print(f'缺少模块: {missing_modules}')
    print('请安装缺少的模块')
else:
    print('所有依赖模块已安装')
"

# 检查相机设备
echo "📷 检查相机设备..."
if ls /dev/video* 1> /dev/null 2>&1; then
    echo "✅ 找到相机设备:"
    ls -la /dev/video*
else
    echo "⚠️ 未找到相机设备，调试时可能需要使用无相机模式"
    export BYTETRACKER_NO_CAMERA=1
fi

# 检查特征文件
echo "📊 检查特征数据文件..."
FEATURES_DIR="./features-data"
if [ -d "$FEATURES_DIR" ]; then
    echo "✅ 找到特征数据目录: $FEATURES_DIR"
    find "$FEATURES_DIR" -name "*.xlsx" -exec echo "  📄 {}" \;
else
    echo "⚠️ 未找到特征数据目录，单目标跟踪可能无法工作"
fi

echo ""
echo "🎯 调试配置完成！现在你可以："
echo "1. 在 Cursor 中打开调试面板 (Ctrl+Shift+D)"
echo "2. 选择调试配置："
echo "   - '调试 ByteTracker 节点' - 正常调试"
echo "   - '调试 ByteTracker 节点 (无相机)' - 无相机模式调试"
echo "   - '调试 ByteTracker 节点 (单步调试)' - 从第一行开始单步调试"
echo "3. 按 F5 开始调试"
echo ""
echo "💡 调试技巧："
echo "- 按 F9 设置/取消断点"
echo "- 按 F10 单步跳过"
echo "- 按 F11 单步进入"
echo "- 按 Shift+F11 单步跳出"
echo "- 按 F5 继续执行"
echo "- 在变量窗口中查看变量值"
echo "- 在调试控制台中执行Python代码" 