#!/usr/bin/env python3
"""
RFID读写器ROS2节点
基于原有RFID_test.py程序改写，提供ROS2接口用于RFID标签检测和监控
"""

import socket
import time
import threading
from datetime import datetime
from collections import defaultdict

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Header
from builtin_interfaces.msg import Time as ROSTime

from rfid_reader.msg import RfidTag, RfidTagArray, RfidReaderStatus
from rfid_reader.srv import RfidCommand


def calculate_checksum(data):
    """根据文档计算校验和：除校验和本身外所有字节的和，取反加一"""
    return (~sum(data) + 1) & 0xFF


def print_hex(data):
    """以十六进制格式打印字节数据，方便调试"""
    return " ".join(f"{b:02X}" for b in data)


def rssi_to_dbm(rssi_value):
    """根据协议文档将RSSI原始值转换为dBm"""
    if rssi_value >= 98:
        return -31
    elif rssi_value <= 31:
        return -99
    else:
        return -(129 - rssi_value)


def datetime_to_ros_time(dt):
    """将Python datetime转换为ROS Time"""
    timestamp = dt.timestamp()
    sec = int(timestamp)
    nanosec = int((timestamp - sec) * 1e9)
    ros_time = ROSTime()
    ros_time.sec = sec
    ros_time.nanosec = nanosec
    return ros_time


class TagInfo:
    """标签信息类"""
    def __init__(self, epc, pc, rssi, antenna_id, freq_param):
        self.epc = epc
        self.pc = pc
        self.rssi = rssi
        self.antenna_id = antenna_id
        self.freq_param = freq_param
        self.first_seen = datetime.now()
        self.last_seen = datetime.now()
        self.read_count = 1
        
    def update(self, rssi, antenna_id, freq_param):
        """更新标签信息"""
        self.rssi = rssi
        self.antenna_id = antenna_id
        self.freq_param = freq_param
        self.last_seen = datetime.now()
        self.read_count += 1

    def to_ros_msg(self, header):
        """转换为ROS消息"""
        msg = RfidTag()
        msg.header = header
        msg.epc = self.epc
        msg.pc = list(self.pc)
        msg.rssi_dbm = rssi_to_dbm(self.rssi)
        msg.rssi_raw = self.rssi
        msg.antenna_id = self.antenna_id
        msg.freq_param = self.freq_param
        msg.first_seen = datetime_to_ros_time(self.first_seen)
        msg.last_seen = datetime_to_ros_time(self.last_seen)
        msg.read_count = self.read_count
        return msg


class RFIDReaderNode(Node):
    """RFID读写器ROS2节点"""
    
    def __init__(self):
        super().__init__('rfid_reader_node')
        
        # 声明参数
        self.declare_parameter('reader_ip', '192.168.0.178')
        self.declare_parameter('reader_port', 4001)
        self.declare_parameter('reader_address', 0xFF)
        self.declare_parameter('publish_rate', 1.0)  # Hz
        self.declare_parameter('auto_start', True)
        self.declare_parameter('antenna_id', 1)
        self.declare_parameter('tag_timeout', 30.0)  # 标签超时时间(秒)
        
        # 获取参数
        self.reader_ip = self.get_parameter('reader_ip').value
        self.reader_port = self.get_parameter('reader_port').value
        self.reader_address = self.get_parameter('reader_address').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.auto_start = self.get_parameter('auto_start').value
        self.antenna_id = self.get_parameter('antenna_id').value
        self.tag_timeout = self.get_parameter('tag_timeout').value
        
        # 初始化变量
        self.sock = None
        self.is_running = False
        self.lock = threading.Lock()
        
        # 标签管理
        self.current_tags = {}  # EPC -> TagInfo
        self.total_reads = 0
        self.session_start_time = None
        self.last_inventory_stats = {}
        
        # 创建发布者
        self.tag_publisher = self.create_publisher(
            RfidTag, 'rfid/tag_detected', 10)
        self.tags_publisher = self.create_publisher(
            RfidTagArray, 'rfid/tags', 10)
        self.status_publisher = self.create_publisher(
            RfidReaderStatus, 'rfid/status', 10)
        
        # 创建服务
        self.command_service = self.create_service(
            RfidCommand, 'rfid/command', self.command_callback)
        
        # 创建定时器
        self.publish_timer = self.create_timer(
            1.0 / self.publish_rate, self.publish_callback)
        
        # 指令定义
        self.CMD_SET_ANTENNA_1 = bytearray([0xA0, 0x04, self.reader_address, 0x74, 0x00])
        self.CMD_REAL_TIME_INVENTORY = bytearray([0xA0, 0x04, self.reader_address, 0x89, 0xFF])
        
        self.get_logger().info(f'RFID读写器节点已启动，目标设备: {self.reader_ip}:{self.reader_port}')
        
        # 自动启动
        if self.auto_start:
            self.start_inventory()

    def connect(self):
        """连接到读写器"""
        try:
            self.get_logger().info(f"正在连接到读写器 {self.reader_ip}:{self.reader_port}...")
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(5)
            self.sock.connect((self.reader_ip, self.reader_port))
            self.get_logger().info("连接成功！")
            return True
        except socket.error as e:
            self.get_logger().error(f"连接失败: {e}")
            self.sock = None
            return False

    def disconnect(self):
        """断开与读写器的连接"""
        self.is_running = False
        if self.sock:
            self.sock.close()
            self.sock = None
            self.get_logger().info("已断开连接")

    def send_command(self, command_body):
        """发送指令到读写器（自动计算并添加校验和）"""
        if not self.sock:
            self.get_logger().error("错误：未连接到读写器")
            return False

        checksum = calculate_checksum(command_body)
        full_command = command_body + bytearray([checksum])

        try:
            with self.lock:
                self.sock.send(full_command)
            time.sleep(0.1)
            return True
        except Exception as e:
            self.get_logger().error(f"发送指令失败: {e}")
            return False

    def receive_data(self):
        """在一个独立的线程中持续接收和解析数据"""
        while self.is_running and self.sock:
            try:
                response = self.sock.recv(1024)
                if not response:
                    self.get_logger().warn("与服务器的连接已断开")
                    self.is_running = False
                    break

                self.parse_response(response)

            except socket.timeout:
                continue
            except Exception as e:
                self.get_logger().error(f"接收数据时发生错误: {e}")
                self.is_running = False
                break

    def parse_response(self, data):
        """解析读写器返回的数据"""
        i = 0
        while i < len(data):
            # 寻找包头 0xA0
            if data[i] != 0xA0:
                i += 1
                continue

            # 检查是否有足够的长度信息
            if i + 1 >= len(data):
                break

            # 获取数据包长度 (Len 字段)
            pkt_len = data[i + 1]

            # 检查整个包是否完整
            if i + pkt_len + 2 > len(data):
                break

            # 提取完整的数据包
            packet = data[i: i + pkt_len + 2]

            # 校验数据包
            received_checksum = packet[-1]
            calculated_checksum = calculate_checksum(packet[:-1])

            if received_checksum != calculated_checksum:
                self.get_logger().warn(f"校验和错误! 收到: {received_checksum:02X}, 计算: {calculated_checksum:02X}")
                i += 1
                continue

            # 解析数据
            cmd = packet[3]
            response_data = packet[4:-1]

            if cmd == 0x74:  # 设置天线响应
                if response_data[0] == 0x10:
                    self.get_logger().info("✓ 成功设置工作天线")
                else:
                    self.get_logger().error(f"✗ 设置天线失败，错误码: {response_data[0]:02X}")

            elif cmd == 0x89:  # 实时盘存响应
                if len(packet) > 7 and len(response_data) > 3:
                    self.process_tag_data(response_data)
                else:
                    self.process_inventory_stats(response_data)

            i += len(packet)

    def process_tag_data(self, response_data):
        """处理标签数据"""
        try:
            freq_ant = response_data[0]
            freq_param = (freq_ant >> 2) & 0x3F
            antenna_id = freq_ant & 0x03
            
            pc = response_data[1:3]
            epc = response_data[3:-1]
            rssi = response_data[-1]
            
            epc_hex = print_hex(epc)
            
            # 更新标签信息
            with self.lock:
                if epc_hex in self.current_tags:
                    self.current_tags[epc_hex].update(rssi, antenna_id, freq_param)
                else:
                    tag_info = TagInfo(epc_hex, pc, rssi, antenna_id, freq_param)
                    self.current_tags[epc_hex] = tag_info
                    
                    # 发布新检测到的标签
                    header = Header()
                    header.stamp = self.get_clock().now().to_msg()
                    header.frame_id = "rfid_reader"
                    
                    tag_msg = tag_info.to_ros_msg(header)
                    self.tag_publisher.publish(tag_msg)
                    
                    self.get_logger().info(f"检测到新标签: {epc_hex}, RSSI: {rssi_to_dbm(rssi)}dBm")
                
                self.total_reads += 1
                
        except Exception as e:
            self.get_logger().error(f"解析标签数据错误: {e}")

    def process_inventory_stats(self, response_data):
        """处理盘存统计信息"""
        try:
            if len(response_data) >= 7:
                antenna_id = int.from_bytes(response_data[0:2], 'big')
                read_rate = int.from_bytes(response_data[2:4], 'big')
                total_read = int.from_bytes(response_data[4:8], 'big')
                
                self.last_inventory_stats = {
                    'antenna_id': antenna_id,
                    'read_rate': read_rate,
                    'total_read': total_read,
                    'timestamp': datetime.now()
                }
        except Exception as e:
            self.get_logger().error(f"解析盘存统计错误: {e}")

    def start_inventory(self):
        """启动实时盘存流程"""
        if not self.connect():
            return False

        self.is_running = True
        self.session_start_time = datetime.now()

        # 启动接收线程
        receive_thread = threading.Thread(target=self.receive_data)
        receive_thread.daemon = True
        receive_thread.start()

        # 设置工作天线
        self.get_logger().info(f"设置工作天线为{self.antenna_id}号")
        self.send_command(self.CMD_SET_ANTENNA_1)
        time.sleep(1)

        # 启动盘存线程
        inventory_thread = threading.Thread(target=self.inventory_loop)
        inventory_thread.daemon = True
        inventory_thread.start()

        self.get_logger().info("开始实时盘存")
        return True

    def inventory_loop(self):
        """盘存循环"""
        while self.is_running:
            self.send_command(self.CMD_REAL_TIME_INVENTORY)
            time.sleep(0.5)

    def is_tag_active(self, tag_info):
        """检查标签是否仍然活跃（未超时）"""
        now = datetime.now()
        time_since_last_seen = (now - tag_info.last_seen).total_seconds()
        return time_since_last_seen <= self.tag_timeout

    def cleanup_expired_tags(self):
        """清理过期的标签"""
        now = datetime.now()
        expired_tags = []
        
        with self.lock:
            for epc, tag_info in list(self.current_tags.items()):
                time_since_last_seen = (now - tag_info.last_seen).total_seconds()
                if time_since_last_seen > self.tag_timeout:
                    expired_tags.append(epc)
                    del self.current_tags[epc]
        
        # 记录清理的过期标签
        if expired_tags:
            self.get_logger().info(f"清理了 {len(expired_tags)} 个过期标签: {expired_tags[:3]}{'...' if len(expired_tags) > 3 else ''}")

    def stop_inventory(self):
        """停止盘存"""
        self.is_running = False
        self.disconnect()
        
        if self.session_start_time:
            duration = datetime.now() - self.session_start_time
            self.get_logger().info(f"盘存会话结束，持续时间: {str(duration).split('.')[0]}")
            self.get_logger().info(f"检测到的标签总数: {len(self.current_tags)}")
            self.get_logger().info(f"总读取次数: {self.total_reads}")

    def publish_callback(self):
        """定时发布回调"""
        if not self.is_running:
            return

        # 清理过期标签
        self.cleanup_expired_tags()

        # 发布标签数组
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "rfid_reader"

        with self.lock:
            # 只发布活跃的标签
            active_tags = [tag_info for tag_info in self.current_tags.values() 
                          if self.is_tag_active(tag_info)]
            
            tags_msg = RfidTagArray()
            tags_msg.header = header
            tags_msg.tags = [tag_info.to_ros_msg(header) for tag_info in active_tags]
            tags_msg.total_tags = len(active_tags)
            tags_msg.total_reads = self.total_reads

        self.tags_publisher.publish(tags_msg)

        # 发布状态信息
        status_msg = RfidReaderStatus()
        status_msg.header = header
        status_msg.connected = self.sock is not None
        status_msg.reader_ip = self.reader_ip
        status_msg.reader_port = self.reader_port
        status_msg.antenna_id = self.antenna_id
        
        if self.last_inventory_stats:
            status_msg.read_rate = self.last_inventory_stats.get('read_rate', 0)
            status_msg.total_read = self.last_inventory_stats.get('total_read', 0)
        
        if self.session_start_time:
            status_msg.session_start_time = datetime_to_ros_time(self.session_start_time)
            duration = (datetime.now() - self.session_start_time).total_seconds()
            status_msg.session_duration = duration

        self.status_publisher.publish(status_msg)

    def command_callback(self, request, response):
        """处理服务请求"""
        command = request.command.lower()
        
        try:
            if command == "start_inventory":
                if self.start_inventory():
                    response.success = True
                    response.message = "盘存已启动"
                else:
                    response.success = False
                    response.message = "启动盘存失败"
                    
            elif command == "stop_inventory":
                self.stop_inventory()
                response.success = True
                response.message = "盘存已停止"
                
            elif command == "set_antenna":
                self.antenna_id = request.antenna_id
                if self.is_running:
                    self.send_command(self.CMD_SET_ANTENNA_1)
                response.success = True
                response.message = f"天线已设置为{self.antenna_id}号"
                
            elif command == "get_status":
                response.success = True
                response.message = "状态获取成功"
                
            else:
                response.success = False
                response.message = f"未知命令: {command}"
                
        except Exception as e:
            response.success = False
            response.message = f"命令执行失败: {str(e)}"
            self.get_logger().error(f"服务请求处理错误: {e}")

        # 填充状态信息
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "rfid_reader"
        
        response.status.header = header
        response.status.connected = self.sock is not None
        response.status.reader_ip = self.reader_ip
        response.status.reader_port = self.reader_port
        response.status.antenna_id = self.antenna_id
        
        if self.session_start_time:
            response.status.session_start_time = datetime_to_ros_time(self.session_start_time)
            duration = (datetime.now() - self.session_start_time).total_seconds()
            response.status.session_duration = duration

        return response

    def destroy_node(self):
        """节点销毁时的清理工作"""
        self.stop_inventory()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = RFIDReaderNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()