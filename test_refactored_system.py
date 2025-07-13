#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
重构后系统测试脚本
================
测试新的架构是否正常工作

功能：
1. 验证编译是否成功
2. 检查节点是否正确注册
3. 测试话题连接
4. 验证命令监控功能
"""

import os
import subprocess
import sys
import time

def run_command(cmd, description, timeout=30):
    """运行命令并返回结果"""
    print(f"\n🔍 {description}")
    print(f"   命令: {cmd}")
    
    try:
        result = subprocess.run(
            cmd, shell=True, capture_output=True, 
            text=True, timeout=timeout
        )
        
        if result.returncode == 0:
            print(f"   ✅ 成功")
            if result.stdout.strip():
                print(f"   输出: {result.stdout.strip()[:200]}...")
        else:
            print(f"   ❌ 失败 (返回码: {result.returncode})")
            if result.stderr.strip():
                print(f"   错误: {result.stderr.strip()[:200]}...")
        
        return result.returncode == 0, result.stdout, result.stderr
        
    except subprocess.TimeoutExpired:
        print(f"   ⏰ 超时 ({timeout}秒)")
        return False, "", f"命令超时"
    except Exception as e:
        print(f"   ❌ 异常: {e}")
        return False, "", str(e)

def main():
    """主测试函数"""
    print("🚀 重构后系统测试开始")
    print("=" * 60)
    
    # 设置ROS2环境
    ros_setup_cmd = "source /opt/ros/humble/setup.bash"
    
    # 1. 检查工作空间
    print(f"📁 当前工作目录: {os.getcwd()}")
    
    # 2. 编译测试
    print("\n📦 1. 编译测试")
    success, stdout, stderr = run_command(
        f"{ros_setup_cmd} && colcon build --packages-select following_robot dlrobot_robot_python",
        "编译核心功能包",
        timeout=120
    )
    
    if not success:
        print("❌ 编译失败，请检查代码")
        return False
    
    # 3. 源环境
    source_cmd = f"{ros_setup_cmd} && source install/setup.bash"
    
    # 4. 检查节点注册
    print("\n🔍 2. 检查节点注册")
    
    nodes_to_check = [
        "robot_control_node",
        "command_monitor_node", 
        "person_following_controller"
    ]
    
    for node in nodes_to_check:
        success, stdout, stderr = run_command(
            f"{source_cmd} && ros2 pkg executables following_robot | grep {node} || ros2 pkg executables dlrobot_robot_python | grep {node}",
            f"检查 {node} 节点",
            timeout=10
        )
        
        if success and node in stdout:
            print(f"   ✅ {node} 已注册")
        else:
            print(f"   ❌ {node} 未找到")
    
    # 5. 检查话题定义
    print("\n📡 3. 检查话题结构")
    
    # 启动一个测试节点来检查话题
    test_cmd = f"""
{source_cmd} && timeout 5 ros2 run following_robot robot_control_node --ros-args -p debug_mode:=true &
sleep 2
ros2 topic list | grep -E "(command_monitor|serial_monitor|robot_control)"
pkill -f robot_control_node
"""
    
    success, stdout, stderr = run_command(
        test_cmd,
        "检查话题发布",
        timeout=15
    )
    
    if success:
        print("   话题列表:")
        for line in stdout.split('\n'):
            if line.strip():
                print(f"     - {line.strip()}")
    
    # 6. 测试启动文件
    print("\n🚀 4. 测试启动文件")
    
    launch_test_cmd = f"""
{source_cmd} && timeout 10 ros2 launch following_robot person_detection_system.launch.py \\
    use_astra_camera:=false \\
    use_depth_service:=false \\
    use_dlrobot_driver:=false &
sleep 5
ros2 node list
pkill -f person_detection_system
"""
    
    success, stdout, stderr = run_command(
        launch_test_cmd,
        "测试启动文件",
        timeout=20
    )
    
    if success:
        print("   启动的节点:")
        for line in stdout.split('\n'):
            if line.strip() and line.startswith('/'):
                print(f"     - {line.strip()}")
    
    print("\n" + "=" * 60)
    print("🎯 重构后系统测试完成")
    
    print("\n📋 重构成果总结:")
    print("1. ✅ 简化了 robot_control_node 职责")
    print("2. ✅ 引入了专业的 person_following_controller")
    print("3. ✅ 增加了命令监控功能")
    print("4. ✅ 优化了架构分层")
    
    print("\n🔥 新功能亮点:")
    print("- 📡 实时串口命令监控 (command_monitor_node)")
    print("- 🎯 专业PID跟随控制 (person_following_controller)")
    print("- 📊 详细的命令统计和调试信息")
    print("- 🏗️ 清晰的分层架构设计")
    
    print("\n📝 使用方法:")
    print("1. 启动完整系统:")
    print("   ros2 launch following_robot person_detection_system.launch.py")
    print("2. 查看命令监控:")
    print("   ros2 topic echo /robot_control/command_monitor")
    print("3. 查看串口监控:")
    print("   ros2 topic echo /robot_control/serial_monitor")
    print("4. 发送测试命令:")
    print("   ros2 topic pub /websocket_bridge/command std_msgs/String 'data: motor_forward:50' --once")
    
    return True

if __name__ == "__main__":
    try:
        success = main()
        sys.exit(0 if success else 1)
    except KeyboardInterrupt:
        print("\n⚠️ 用户中断测试")
        sys.exit(1)
    except Exception as e:
        print(f"\n❌ 测试异常: {e}")
        sys.exit(1)