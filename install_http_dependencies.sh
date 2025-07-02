#!/bin/bash

# 安装HTTP文件上传功能所需的依赖
echo "🚀 正在安装HTTP服务器依赖..."

# 安装aiohttp用于文件上传功能
pip3 install aiohttp

echo "✅ HTTP服务器依赖安装完成"
echo "现在可以运行服务端并使用文件上传功能了"
echo ""
echo "启动命令："
echo "cd 服务端"
echo "python3 server.py"
echo ""
echo "服务端将启动以下服务："
echo "- WebSocket服务: ws://172.20.39.181:1234"
echo "- HTTP文件上传: http://172.20.39.181:1235" 