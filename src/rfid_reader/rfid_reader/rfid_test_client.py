#!/usr/bin/env python3
"""
RFID读写器测试客户端
用于测试RFID功能包的基本功能
"""

import rclpy
from rclpy.node import Node
from rfid_reader.msg import RfidTag, RfidTagArray, RfidReaderStatus
from rfid_reader.srv import RfidCommand


class RFIDTestClient(Node):
    """RFID测试客户端节点"""
    
    def __init__(self):
        super().__init__('rfid_test_client')
        
        # 创建服务客户端
        self.command_client = self.create_client(RfidCommand, 'rfid/command')
        
        # 创建订阅者
        self.tags_subscription = self.create_subscription(
            RfidTagArray, 'rfid/tags', self.tags_callback, 10)
        self.status_subscription = self.create_subscription(
            RfidReaderStatus, 'rfid/status', self.status_callback, 10)
        self.tag_subscription = self.create_subscription(
            RfidTag, 'rfid/tag_detected', self.tag_detected_callback, 10)
        
        self.get_logger().info('RFID测试客户端已启动')
        self.get_logger().info('可用命令:')
        self.get_logger().info('  start - 开始盘存')
        self.get_logger().info('  stop  - 停止盘存')
        self.get_logger().info('  status - 获取状态')
        
    def tags_callback(self, msg):
        """标签数组回调"""
        if msg.total_tags > 0:
            self.get_logger().info(f'当前检测到 {msg.total_tags} 个标签，总读取次数: {msg.total_reads}')
            for i, tag in enumerate(msg.tags[:5]):  # 只显示前5个标签
                self.get_logger().info(f'  标签{i+1}: {tag.epc}, RSSI: {tag.rssi_dbm}dBm, 读取次数: {tag.read_count}')
    
    def status_callback(self, msg):
        """状态回调"""
        connection_status = "已连接" if msg.connected else "未连接"
        self.get_logger().info(f'状态更新: {connection_status}, 天线: {msg.antenna_id}, 读取速率: {msg.read_rate}/s')
    
    def tag_detected_callback(self, msg):
        """新标签检测回调"""
        self.get_logger().info(f'🏷️  新标签检测: {msg.epc}, RSSI: {msg.rssi_dbm}dBm')
    
    def send_command(self, command, antenna_id=1):
        """发送命令"""
        if not self.command_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('RFID服务不可用')
            return False
        
        request = RfidCommand.Request()
        request.command = command
        request.antenna_id = antenna_id
        
        self.get_logger().info(f'发送命令: {command}')
        
        future = self.command_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        
        if future.done():
            response = future.result()
            if response.success:
                self.get_logger().info(f'✅ 命令成功: {response.message}')
            else:
                self.get_logger().error(f'❌ 命令失败: {response.message}')
            return response.success
        else:
            self.get_logger().error('命令执行超时')
            return False


def main(args=None):
    rclpy.init(args=args)
    
    client = RFIDTestClient()
    
    try:
        print("\n=== RFID读写器测试客户端 ===")
        print("输入命令 (start/stop/status/quit):")
        
        while rclpy.ok():
            # 非阻塞地处理ROS回调
            rclpy.spin_once(client, timeout_sec=0.1)
            
            # 检查用户输入
            try:
                import select
                import sys
                
                if select.select([sys.stdin], [], [], 0):
                    command = input().strip().lower()
                    
                    if command == 'quit' or command == 'q':
                        break
                    elif command == 'start':
                        client.send_command('start_inventory')
                    elif command == 'stop':
                        client.send_command('stop_inventory')
                    elif command == 'status':
                        client.send_command('get_status')
                    elif command == 'help' or command == 'h':
                        print("可用命令: start, stop, status, quit")
                    else:
                        print(f"未知命令: {command}")
                        
            except (EOFError, KeyboardInterrupt):
                break
            except ImportError:
                # Windows系统或没有select模块
                print("请手动输入命令后按回车:")
                try:
                    command = input().strip().lower()
                    if command == 'quit':
                        break
                    elif command == 'start':
                        client.send_command('start_inventory')
                    elif command == 'stop':
                        client.send_command('stop_inventory')
                    elif command == 'status':
                        client.send_command('get_status')
                except (EOFError, KeyboardInterrupt):
                    break
    
    except KeyboardInterrupt:
        pass
    finally:
        print("\n停止测试客户端...")
        client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()