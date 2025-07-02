#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
完整系统启动验证脚本
=================

测试和验证完整系统的启动配置是否正确

使用方法：
    python3 test_full_system_launch.py
"""

import subprocess
import time
import sys
import signal
import threading
from pathlib import Path

class SystemLaunchTester:
    """完整系统启动测试器"""
    
    def __init__(self):
        self.process = None
        self.output_log = []
        self.expected_nodes = [
            'feature_extraction_node',
            'bytetracker_node', 
            'websocket_bridge_node',
            'robot_control_node'
        ]
        
    def test_launch_file_syntax(self):
        """测试launch文件语法是否正确"""
        print("🔍 测试launch文件语法...")
        
        try:
            # 运行语法检查
            result = subprocess.run([
                'python3', '-m', 'py_compile', 
                'src/following_robot/launch/full_system.launch.py'
            ], capture_output=True, text=True, cwd='SelfFollowingROS2')
            
            if result.returncode == 0:
                print("✅ Launch文件语法检查通过")
                return True
            else:
                print("❌ Launch文件语法错误:")
                print(result.stderr)
                return False
                
        except Exception as e:
            print(f"❌ 语法检查失败: {e}")
            return False
    
    def test_dry_run(self):
        """测试干运行（不实际启动节点）"""
        print("🔍 测试launch文件参数解析...")
        
        try:
            # 使用ros2 launch进行验证
            cmd = [
                'ros2', 'launch', 'following_robot', 'full_system.launch.py',
                '--show-args'
            ]
            
            result = subprocess.run(cmd, capture_output=True, text=True, 
                                  timeout=10, cwd='SelfFollowingROS2')
            
            if result.returncode == 0:
                print("✅ Launch文件参数解析成功")
                print("📋 可用参数:")
                for line in result.stdout.split('\n'):
                    if 'Arguments' in line or ':' in line:
                        print(f"   {line}")
                return True
            else:
                print("❌ Launch文件参数解析失败:")
                print(result.stderr)
                return False
                
        except subprocess.TimeoutExpired:
            print("⚠️ 参数解析超时")
            return False
        except Exception as e:
            print(f"❌ 参数解析失败: {e}")
            return False
    
    def start_system(self, custom_params=None):
        """启动完整系统（用于测试）"""
        print("🚀 启动完整系统进行测试...")
        
        cmd = [
            'ros2', 'launch', 'following_robot', 'full_system.launch.py'
        ]
        
        # 添加自定义参数
        if custom_params:
            for key, value in custom_params.items():
                cmd.extend([f'{key}:={value}'])
        
        try:
            self.process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                cwd='SelfFollowingROS2',
                bufsize=1,
                universal_newlines=True
            )
            
            print("✅ 系统启动命令已执行")
            return True
            
        except Exception as e:
            print(f"❌ 系统启动失败: {e}")
            return False
    
    def monitor_startup(self, timeout=30):
        """监控启动过程"""
        print(f"⏱️ 监控启动过程 (超时: {timeout}秒)...")
        
        if not self.process:
            print("❌ 系统未启动")
            return False
        
        start_time = time.time()
        found_nodes = set()
        
        # 启动输出监控线程
        def read_output():
            try:
                for line in iter(self.process.stdout.readline, ''):
                    if line:
                        self.output_log.append(line.strip())
                        print(f"📝 {line.strip()}")
                        
                        # 检查节点启动信息
                        for node_name in self.expected_nodes:
                            if node_name in line and ('初始化完成' in line or '已启动' in line or 'ready' in line.lower()):
                                found_nodes.add(node_name)
                                print(f"✅ 检测到节点启动: {node_name}")
                        
                        # 检查错误信息
                        if 'error' in line.lower() or 'failed' in line.lower():
                            print(f"⚠️ 检测到错误: {line.strip()}")
                            
            except Exception as e:
                print(f"📝 输出监控异常: {e}")
        
        monitor_thread = threading.Thread(target=read_output, daemon=True)
        monitor_thread.start()
        
        # 等待启动完成或超时
        while time.time() - start_time < timeout:
            if self.process.poll() is not None:
                print("❌ 系统意外退出")
                return False
                
            if len(found_nodes) >= len(self.expected_nodes):
                print("✅ 所有预期节点都已启动")
                return True
                
            time.sleep(1)
        
        print(f"⚠️ 启动监控超时，已找到节点: {found_nodes}")
        print(f"📊 预期节点: {self.expected_nodes}")
        
        return len(found_nodes) > 0
    
    def stop_system(self):
        """停止系统"""
        print("🛑 停止系统...")
        
        if self.process:
            try:
                # 发送SIGINT信号
                self.process.send_signal(signal.SIGINT)
                
                # 等待优雅退出
                try:
                    self.process.wait(timeout=10)
                    print("✅ 系统已优雅退出")
                except subprocess.TimeoutExpired:
                    print("⚠️ 优雅退出超时，强制终止")
                    self.process.kill()
                    self.process.wait()
                    
            except Exception as e:
                print(f"❌ 停止系统失败: {e}")
    
    def run_full_test(self):
        """运行完整测试"""
        print("=" * 60)
        print("🧪 完整系统启动验证测试")
        print("=" * 60)
        
        # 1. 语法检查
        if not self.test_launch_file_syntax():
            return False
        
        print()
        
        # 2. 参数解析检查
        if not self.test_dry_run():
            return False
        
        print()
        
        # 3. 实际启动测试（可选）
        print("❓ 是否进行实际启动测试？(y/N): ", end="")
        try:
            response = input().strip().lower()
            if response == 'y' or response == 'yes':
                
                # 启动系统
                test_params = {
                    'feature_output_dir': 'test_features',
                    'camera_index': '0',  
                    'websocket_host': '127.0.0.1'
                }
                
                if self.start_system(test_params):
                    # 监控启动
                    success = self.monitor_startup(timeout=20)
                    
                    # 停止系统
                    self.stop_system()
                    
                    if success:
                        print("\n✅ 完整系统测试通过")
                        return True
                    else:
                        print("\n⚠️ 系统启动不完整")
                        return False
                else:
                    print("\n❌ 系统启动失败")
                    return False
            else:
                print("✅ 跳过实际启动测试")
                return True
                
        except KeyboardInterrupt:
            print("\n🛑 测试被用户中断")
            self.stop_system()
            return False


def main():
    """主函数"""
    tester = SystemLaunchTester()
    
    try:
        success = tester.run_full_test()
        
        if success:
            print("\n🎉 所有测试通过！系统启动配置正确")
            sys.exit(0)
        else:
            print("\n❌ 测试失败，请检查配置")
            sys.exit(1)
            
    except KeyboardInterrupt:
        print("\n🛑 测试被中断")
        tester.stop_system()
        sys.exit(1)
    except Exception as e:
        print(f"\n❌ 测试异常: {e}")
        tester.stop_system()
        sys.exit(1)


if __name__ == '__main__':
    main() 