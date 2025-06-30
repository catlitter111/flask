#!/bin/bash
# ByteTracker依赖安装脚本
# 用于安装ByteTracker ROS2节点所需的Python依赖

echo "开始安装ByteTracker依赖..."

# 安装系统依赖
echo "安装系统依赖..."
sudo apt update
sudo apt install -y python3-pip python3-scipy python3-openpyxl

# 安装Python依赖
echo "安装Python依赖..."
pip3 install --user scipy lap openpyxl

# 编译自定义消息
echo "编译自定义消息..."
cd src/custom_msgs
colcon build --packages-select custom_msgs
source install/setup.bash

echo "依赖安装完成！"
echo "请运行以下命令来编译整个工作空间："
echo "cd /userdata/try_again/SelfFollowingROS2"
echo "colcon build"
echo "source install/setup.bash" 