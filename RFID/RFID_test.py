import socket
import time
import threading
import os
from datetime import datetime
from collections import defaultdict

# --- 配置 ---
READER_IP = "192.168.0.178"
READER_PORT = 4001
READER_ADDRESS = 0xFF  # 读写器地址，FF为广播地址

# --- 指令定义 ---
# 设置工作天线为1号天线
CMD_SET_ANTENNA_1 = bytearray([0xA0, 0x04, READER_ADDRESS, 0x74, 0x00])
# 实时盘存指令 (重复次数0xFF表示以最快速度)
CMD_REAL_TIME_INVENTORY = bytearray([0xA0, 0x04, READER_ADDRESS, 0x89, 0xFF])


def calculate_checksum(data):
    """
    根据文档计算校验和：除校验和本身外所有字节的和，取反加一
    """
    return (~sum(data) + 1) & 0xFF


def print_hex(data):
    """
    以十六进制格式打印字节数据，方便调试
    """
    return " ".join(f"{b:02X}" for b in data)


def clear_screen():
    """
    清屏函数
    """
    os.system('cls' if os.name == 'nt' else 'clear')


def rssi_to_dbm(rssi_value):
    """
    根据协议文档将RSSI原始值转换为dBm
    """
    # 根据文档中的RSSI参数对应表
    if rssi_value >= 98:
        return -31
    elif rssi_value <= 31:
        return -99
    else:
        return -(129 - rssi_value)


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
        self.rssi = rssi  # 更新为最新的RSSI
        self.antenna_id = antenna_id
        self.freq_param = freq_param
        self.last_seen = datetime.now()
        self.read_count += 1


class RFIDReader:
    def __init__(self, ip, port):
        self.ip = ip
        self.port = port
        self.sock = None
        self.is_running = False
        self.lock = threading.Lock()
        
        # 标签管理
        self.current_tags = {}  # EPC -> TagInfo
        self.total_reads = 0
        self.session_start_time = None
        self.last_inventory_stats = {}
        
        # 显示控制
        self.display_enabled = True
        self.display_thread = None

    def connect(self):
        """
        连接到读写器
        """
        try:
            print(f"正在连接到读写器 {self.ip}:{self.port}...")
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(5)  # 设置超时
            self.sock.connect((self.ip, self.port))
            print("连接成功！")
            return True
        except socket.error as e:
            print(f"连接失败: {e}")
            self.sock = None
            return False

    def disconnect(self):
        """
        断开与读写器的连接
        """
        self.is_running = False
        if self.sock:
            self.sock.close()
            self.sock = None
            print("已断开连接。")

    def send_command(self, command_body):
        """
        发送指令到读写器（自动计算并添加校验和）
        """
        if not self.sock:
            print("错误：未连接到读写器。")
            return

        checksum = calculate_checksum(command_body)
        full_command = command_body + bytearray([checksum])

        with self.lock:
            self.sock.send(full_command)
        time.sleep(0.1)  # 发送后稍作等待

    def receive_data(self):
        """
        在一个独立的线程中持续接收和解析数据
        """
        while self.is_running and self.sock:
            try:
                # 每次最多读取1024字节
                response = self.sock.recv(1024)
                if not response:
                    print("与服务器的连接已断开。")
                    self.is_running = False
                    break

                self.parse_response(response)

            except socket.timeout:
                # 超时是正常的，因为我们不知道设备何时会发送数据
                continue
            except Exception as e:
                print(f"接收数据时发生错误: {e}")
                self.is_running = False
                break

    def parse_response(self, data):
        """
        解析读写器返回的数据
        """
        # 多个响应可能在一个TCP包中，需要循环处理
        i = 0
        while i < len(data):
            # 寻找包头 0xA0
            if data[i] != 0xA0:
                i += 1
                continue

            # 检查是否有足够的长度信息
            if i + 1 >= len(data):
                break  # 数据不完整

            # 获取数据包长度 (Len 字段)
            pkt_len = data[i + 1]

            # 检查整个包是否完整
            if i + pkt_len + 2 > len(data):
                # 数据包不完整，等待更多数据
                break

            # 提取完整的数据包
            packet = data[i: i + pkt_len + 2]

            # 校验数据包
            received_checksum = packet[-1]
            calculated_checksum = calculate_checksum(packet[:-1])

            if received_checksum != calculated_checksum:
                print(f"校验和错误! 收到: {received_checksum:02X}, 计算: {calculated_checksum:02X}")
                i += 1
                continue

            # 解析数据
            cmd = packet[3]
            response_data = packet[4:-1]

            if cmd == 0x74:  # 设置天线响应
                if response_data[0] == 0x10:  # 0x10 是 command_success
                    print("✓ 成功设置工作天线为1号")
                else:
                    print(f"✗ 设置天线失败，错误码: {response_data[0]:02X}")

            elif cmd == 0x89:  # 实时盘存响应
                # 检查是标签数据还是盘存结束报告
                if len(packet) > 7 and len(response_data) > 3:
                    # 这是标签数据
                    self.process_tag_data(response_data)
                else:
                    # 这是盘存结束报告
                    self.process_inventory_stats(response_data)

            # 移动到下一个数据包的起始位置
            i += len(packet)

    def process_tag_data(self, response_data):
        """
        处理标签数据
        """
        try:
            freq_ant = response_data[0]
            freq_param = (freq_ant >> 2) & 0x3F  # 高6位是频点参数
            antenna_id = freq_ant & 0x03  # 低2位是天线号
            
            pc = response_data[1:3]
            epc = response_data[3:-1]
            rssi = response_data[-1]
            
            epc_hex = print_hex(epc)
            
            # 更新标签信息
            with self.lock:
                if epc_hex in self.current_tags:
                    self.current_tags[epc_hex].update(rssi, antenna_id, freq_param)
                else:
                    self.current_tags[epc_hex] = TagInfo(epc_hex, pc, rssi, antenna_id, freq_param)
                
                self.total_reads += 1
                
        except Exception as e:
            print(f"解析标签数据错误: {e}")

    def process_inventory_stats(self, response_data):
        """
        处理盘存统计信息
        """
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
            print(f"解析盘存统计错误: {e}")

    def display_status(self):
        """
        实时显示状态信息
        """
        while self.is_running and self.display_enabled:
            try:
                clear_screen()
                
                # 标题
                print("=" * 80)
                print("               RFID 实时盘存监控系统")
                print("=" * 80)
                
                # 基本信息
                current_time = datetime.now()
                if self.session_start_time:
                    runtime = current_time - self.session_start_time
                    print(f"运行时间: {str(runtime).split('.')[0]}")
                
                print(f"连接状态: {'✓ 已连接' if self.sock else '✗ 未连接'}")
                print(f"当前时间: {current_time.strftime('%Y-%m-%d %H:%M:%S')}")
                print()
                
                # 标签统计
                with self.lock:
                    tag_count = len(self.current_tags)
                    total_reads = self.total_reads
                    
                    print("=" * 80)
                    print(f"当前检测到的标签数量: {tag_count}")
                    print(f"总读取次数: {total_reads}")
                    
                    if self.last_inventory_stats:
                        stats = self.last_inventory_stats
                        print(f"读取速率: {stats.get('read_rate', 0)} 次/秒")
                        print(f"天线号: {stats.get('antenna_id', 0) + 1}")
                    
                    print("=" * 80)
                    
                    # 标签详细信息
                    if tag_count > 0:
                        print("\n标签列表:")
                        print("-" * 80)
                        print(f"{'序号':<4} {'EPC':<30} {'RSSI':<8} {'读取次数':<8} {'最后更新':<19}")
                        print("-" * 80)
                        
                        # 按最后更新时间排序
                        sorted_tags = sorted(
                            self.current_tags.items(),
                            key=lambda x: x[1].last_seen,
                            reverse=True
                        )
                        
                        for idx, (epc, tag_info) in enumerate(sorted_tags[:20], 1):  # 最多显示20个标签
                            rssi_dbm = rssi_to_dbm(tag_info.rssi)
                            last_seen = tag_info.last_seen.strftime('%H:%M:%S')
                            
                            print(f"{idx:<4} {epc[:28]:<30} {rssi_dbm:>4}dBm {tag_info.read_count:<8} {last_seen:<19}")
                        
                        if tag_count > 20:
                            print(f"\n... 还有 {tag_count - 20} 个标签未显示")
                    else:
                        print("\n暂未检测到标签")
                
                print("\n" + "=" * 80)
                print("按 Ctrl+C 停止监控")
                
            except Exception as e:
                print(f"显示错误: {e}")
            
            time.sleep(1)  # 每秒更新一次显示

    def start_inventory(self):
        """
        启动实时盘存流程
        """
        if not self.connect():
            return

        self.is_running = True
        self.session_start_time = datetime.now()

        # 启动接收线程
        receive_thread = threading.Thread(target=self.receive_data)
        receive_thread.daemon = True
        receive_thread.start()

        # 启动显示线程
        self.display_thread = threading.Thread(target=self.display_status)
        self.display_thread.daemon = True
        self.display_thread.start()

        try:
            # 1. 设置工作天线
            print("步骤 1: 设置工作天线为1号")
            self.send_command(CMD_SET_ANTENNA_1)
            time.sleep(1)  # 等待设备响应和处理

            # 2. 循环进行实时盘存
            print("步骤 2: 开始实时盘存")
            time.sleep(2)  # 等待显示线程启动
            
            while self.is_running:
                self.send_command(CMD_REAL_TIME_INVENTORY)
                time.sleep(0.5)  # 每隔0.5秒发送一次盘存指令
                
        except KeyboardInterrupt:
            print("\n\n程序停止中...")
        finally:
            self.is_running = False
            self.display_enabled = False
            time.sleep(1)  # 等待线程结束
            self.disconnect()
            
            # 显示最终统计
            clear_screen()
            print("=" * 60)
            print("              盘存会话结束")
            print("=" * 60)
            print(f"会话时长: {str(datetime.now() - self.session_start_time).split('.')[0]}")
            print(f"检测到的标签总数: {len(self.current_tags)}")
            print(f"总读取次数: {self.total_reads}")
            print("\n谢谢使用！")


if __name__ == "__main__":
    reader = RFIDReader(READER_IP, READER_PORT)
    reader.start_inventory()