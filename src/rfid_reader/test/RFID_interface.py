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
ACCESS_PASSWORD = bytearray([0x00, 0x00, 0x00, 0x00])  # 访问密码，通常默认为4个0

# --- 指令定义 (基本部分) ---
CMD_SET_ANTENNA_1 = bytearray([0xA0, 0x04, READER_ADDRESS, 0x74, 0x00])
CMD_REAL_TIME_INVENTORY = bytearray([0xA0, 0x04, READER_ADDRESS, 0x89, 0xFF])

# --- 全局变量 ---
last_write_response = None
last_read_response = None
response_event = threading.Event()

# 新增：错误码字典，用于提供更友好的错误信息
ERROR_CODES = {
    0x10: "命令成功完成",
    0x11: "命令执行失败",
    0x22: "天线未连接",
    0x33: "写标签错误",
    0x34: "锁定标签错误",
    0x35: "灭活标签错误",
    0x36: "无可操作标签错误",
    0x37: "成功盘存但访问失败",
    0x40: "访问标签错误或访问密码错误",
    0x41: "无效的参数",
    0x53: "射频芯片无响应",
}


def calculate_checksum(data):
    """根据文档计算校验和"""
    return (~sum(data) + 1) & 0xFF


def print_hex(data):
    """以十六进制格式打印字节数据"""
    return " ".join(f"{b:02X}" for b in data)


def clear_screen():
    """清屏函数"""
    os.system('cls' if os.name == 'nt' else 'clear')


def rssi_to_dbm(rssi_value):
    """根据协议文档将RSSI原始值转换为dBm"""
    if rssi_value >= 98: return -31
    if rssi_value <= 31: return -99
    if 90 <= rssi_value <= 97: return -31 - (98 - rssi_value)
    if 65 <= rssi_value <= 89: return -40 - (89 - rssi_value)
    return -99


class TagInfo:
    """标签信息类"""
    def __init__(self, epc, pc, rssi, antenna_id):
        self.epc = epc
        self.pc = pc
        self.rssi = rssi
        self.antenna_id = antenna_id
        self.first_seen = datetime.now()
        self.last_seen = datetime.now()
        self.read_count = 1
        self.user_data = ""
        self.user_data_read_attempted = False

    def update(self, rssi, antenna_id):
        """更新标签信息"""
        self.rssi = rssi
        self.antenna_id = antenna_id
        self.last_seen = datetime.now()
        self.read_count += 1


class RFIDReader:
    def __init__(self, ip, port):
        self.ip = ip
        self.port = port
        self.sock = None
        self.is_running = False
        self.inventory_active = False
        self.lock = threading.Lock()
        self.current_tags = {}
        self.total_reads = 0
        self.session_start_time = None
        self.last_inventory_stats = {}
        self.display_thread = None

    def connect(self):
        try:
            print(f"正在连接到读写器 {self.ip}:{self.port}...")
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(5)
            self.sock.connect((self.ip, self.port))
            print("连接成功！")
            return True
        except socket.error as e:
            print(f"连接失败: {e}")
            self.sock = None
            return False

    def disconnect(self):
        self.is_running = False
        self.inventory_active = False
        if self.display_thread:
            self.display_thread.join()
        if self.sock:
            self.sock.close()
            self.sock = None
            print("已断开连接。")

    def send_command(self, command_body):
        if not self.sock:
            print("错误：未连接到读写器。")
            return
        checksum = calculate_checksum(command_body)
        full_command = command_body + bytearray([checksum])
        with self.lock:
            self.sock.send(full_command)
        time.sleep(0.05)

    def receive_data(self):
        buffer = bytearray()
        while self.is_running:
            try:
                data = self.sock.recv(1024)
                if not data:
                    print("与服务器的连接已断开。")
                    self.is_running = False
                    break
                buffer.extend(data)
                buffer = self.parse_response_stream(buffer)
            except socket.timeout:
                continue
            except Exception as e:
                print(f"接收数据时发生错误: {e}")
                self.is_running = False
                break

    def parse_response_stream(self, buffer):
        global last_write_response, last_read_response, response_event
        i = 0
        while i < len(buffer):
            if buffer[i] != 0xA0:
                i += 1
                continue
            if i + 1 >= len(buffer): break
            pkt_len = buffer[i + 1]
            if i + pkt_len + 2 > len(buffer): break
            packet = buffer[i : i + pkt_len + 2]
            received_checksum = packet[-1]
            calculated_checksum = calculate_checksum(packet[:-1])
            if received_checksum != calculated_checksum:
                i += len(packet)
                continue
            cmd = packet[3]
            response_data = packet[4:-1]
            if cmd == 0x74:
                if response_data and response_data[0] == 0x10:
                    print("✓ 成功设置工作天线为1号")
                else:
                    print(f"✗ 设置天线失败，错误码: {response_data[0]:02X}")
            elif cmd == 0x89:
                if len(packet) > 7 and len(response_data) > 3:
                    self.process_tag_data(response_data)
                else:
                    self.process_inventory_stats(response_data)
            elif cmd == 0x85:
                pass
            elif cmd == 0x82:
                last_write_response = packet
                response_event.set()
            elif cmd == 0x81:
                last_read_response = packet
                response_event.set()
            i += len(packet)
        return buffer[i:]

    def process_tag_data(self, response_data):
        try:
            freq_ant = response_data[0]
            antenna_id = freq_ant & 0x03
            pc = response_data[1:3]
            epc = response_data[3:-1]
            rssi = response_data[-1]
            epc_hex = print_hex(epc)
            with self.lock:
                if epc_hex in self.current_tags:
                    self.current_tags[epc_hex].update(rssi, antenna_id)
                else:
                    self.current_tags[epc_hex] = TagInfo(epc_hex, pc, rssi, antenna_id)
                self.total_reads += 1
        except Exception as e:
            print(f"解析标签数据错误: {e}")

    def process_inventory_stats(self, response_data):
        try:
            if len(response_data) >= 7:
                self.last_inventory_stats = {
                    'antenna_id': int.from_bytes(response_data[0:2], 'big'),
                    'read_rate': int.from_bytes(response_data[2:4], 'big'),
                    'total_read': int.from_bytes(response_data[4:8], 'big'),
                }
        except Exception as e:
            print(f"解析盘存统计错误: {e}")

    def display_status(self):
        while self.is_running:
            if not self.inventory_active:
                time.sleep(0.5)
                continue
            clear_screen()
            print("=" * 80)
            print("               RFID 实时盘存监控系统 (按 's' 停止盘存返回菜单)")
            print("=" * 80)
            with self.lock:
                tag_count = len(self.current_tags)
                print(f"当前检测到的标签数量: {tag_count} | 总读取次数: {self.total_reads}")
                if self.last_inventory_stats:
                    stats = self.last_inventory_stats
                    print(f"读取速率: {stats.get('read_rate', 0)} 次/秒 | 天线号: {stats.get('antenna_id', 0) + 1}")
                print("=" * 80)
                if tag_count > 0:
                    print(f"{'序号':<4} {'EPC':<36} {'RSSI':<8} {'次数':<6} {'物品信息':<20}")
                    print("-" * 80)
                    sorted_tags = sorted(self.current_tags.items(), key=lambda x: x[1].last_seen, reverse=True)
                    for idx, (epc, tag_info) in enumerate(sorted_tags, 1):
                        if not tag_info.user_data_read_attempted:
                            tag_info.user_data_read_attempted = True
                            threading.Thread(target=self.read_user_data, args=(epc,)).start()
                        rssi_dbm = rssi_to_dbm(tag_info.rssi)
                        print(f"{idx:<4} {epc:<36} {rssi_dbm:>4}dBm {tag_info.read_count:<6} {tag_info.user_data}")
                else:
                    print("\n暂未检测到标签...")
            time.sleep(1)

    def start_inventory_loop(self):
        self.inventory_active = True
        print("开始实时盘存...")
        while self.inventory_active:
            self.send_command(CMD_REAL_TIME_INVENTORY)
            time.sleep(0.5)

    def stop_inventory_loop(self):
        self.inventory_active = False
        print("已停止盘存。")

    def _set_epc_match(self, epc_hex):
        epc_bytes = bytearray.fromhex(epc_hex.replace(" ", ""))
        epc_len = len(epc_bytes)
        cmd_body = bytearray([0xA0, 5 + epc_len, READER_ADDRESS, 0x85, 0x00, epc_len]) + epc_bytes
        self.send_command(cmd_body)
        time.sleep(0.1)

    def write_user_data(self, epc_hex, data_string):
        global last_write_response, response_event
        print(f"\n准备向 EPC: {epc_hex} 写入数据: '{data_string}'")
        self._set_epc_match(epc_hex)
        data_bytes = data_string.encode('utf-8')
        if len(data_bytes) % 2 != 0:
            data_bytes += b'\x00'
        word_cnt = len(data_bytes) // 2
        if word_cnt > 32:
            print("错误：数据太长！")
            return False
        word_add = 0
        cmd_len = 12 + len(data_bytes)
        cmd_body = bytearray([0xA0, cmd_len, READER_ADDRESS, 0x82])
        cmd_body += ACCESS_PASSWORD
        cmd_body += bytearray([0x03])
        cmd_body += word_add.to_bytes(2, 'big')
        cmd_body += word_cnt.to_bytes(2, 'big')
        cmd_body += data_bytes
        response_event.clear()
        last_write_response = None
        self.send_command(cmd_body)
        if response_event.wait(timeout=3.0):
            packet = last_write_response
            if packet and packet[3] == 0x82:
                response_payload = packet[4:-1]
                if len(packet) == 6:
                    err_code = response_payload[0]
                    error_desc = ERROR_CODES.get(err_code, "未知错误")
                    print(f"✗ 写指令失败，读写器错误码: {err_code:02X} ({error_desc})")
                    return False
                if len(response_payload) > 4:
                    data_len = response_payload[2]
                    tag_err_code_index = 3 + data_len
                    if tag_err_code_index < len(response_payload):
                        tag_err_code = response_payload[tag_err_code_index]
                        if tag_err_code == 0x10 or tag_err_code == 0x00:
                            print("✓ 写入成功！")
                            with self.lock:
                                if epc_hex in self.current_tags:
                                    self.current_tags[epc_hex].user_data = data_string
                            return True
                        else:
                            error_desc = ERROR_CODES.get(tag_err_code, "未知错误")
                            print(f"✗ 写入标签失败，标签返回状态码: {tag_err_code:02X} ({error_desc})")
                            return False
                    else:
                        print(f"✗ 解析写响应失败：响应包结构异常。包: {print_hex(packet)}")
                        return False
                else:
                    print(f"✗ 收到未知的写响应格式: {print_hex(packet)}")
                    return False
        else:
            print("✗ 写入超时，未收到读写器响应。")
        return False

    def read_user_data(self, epc_hex, word_count=16, manual_read=False):
        """读取标签用户数据
        Args:
            epc_hex: 标签EPC
            word_count: 读取字数（默认16字，32字节）
            manual_read: 是否为手动读取（会显示详细信息）
        Returns:
            成功时返回读取的数据字符串，失败时返回None
        """
        global last_read_response, response_event
        
        if manual_read:
            print(f"\n准备从 EPC: {epc_hex} 读取用户数据...")
        
        self._set_epc_match(epc_hex)
        word_add = 0
        cmd_len = 12
        cmd_body = bytearray([0xA0, cmd_len, READER_ADDRESS, 0x81])
        cmd_body += bytearray([0x03])  # MemBank (USER区域)
        cmd_body += word_add.to_bytes(2, 'big')  # WordAdd
        cmd_body += word_count.to_bytes(2, 'big')  # WordCnt
        cmd_body += ACCESS_PASSWORD  # PassWord
        
        response_event.clear()
        last_read_response = None
        self.send_command(cmd_body)
        
        timeout = 3.0 if manual_read else 2.0
        if response_event.wait(timeout=timeout):
            packet = last_read_response
            if packet and packet[3] == 0x81:
                response_payload = packet[4:-1]
                
                # 检查是否为错误响应
                if len(packet) == 6:
                    err_code = response_payload[0]
                    error_desc = ERROR_CODES.get(err_code, "未知错误")
                    if manual_read:
                        print(f"✗ 读取失败，读写器错误码: {err_code:02X} ({error_desc})")
                    return None
                
                # 解析成功响应
                if len(response_payload) > 4:
                    # 获取读取的数据长度（字节数）
                    read_len_bytes = response_payload[-3] if len(response_payload) >= 3 else 0
                    
                    if read_len_bytes > 0:
                        # 计算数据开始位置
                        data_field_len = response_payload[2]
                        start_of_read_data = 3 + (data_field_len - read_len_bytes)
                        end_of_read_data = start_of_read_data + read_len_bytes
                        
                        if end_of_read_data <= len(response_payload) - 3:
                            user_data_bytes = response_payload[start_of_read_data:end_of_read_data]
                            
                            if manual_read:
                                print(f"✓ 读取成功！")
                                print(f"原始数据 (hex): {print_hex(user_data_bytes)}")
                            
                            try:
                                # 尝试解码为UTF-8字符串
                                user_data_str = user_data_bytes.strip(b'\x00').decode('utf-8', errors='ignore')
                                
                                if manual_read:
                                    print(f"解码数据: '{user_data_str}'")
                                    if not user_data_str.strip():
                                        print("注意：标签用户区域为空或只包含空字符")
                                
                                # 更新缓存
                                with self.lock:
                                    if epc_hex in self.current_tags:
                                        self.current_tags[epc_hex].user_data = user_data_str
                                
                                return user_data_str
                                
                            except Exception as e:
                                if manual_read:
                                    print(f"数据解码失败: {e}")
                                return None
                    else:
                        if manual_read:
                            print("✓ 读取成功，但标签用户区域为空")
                        return ""
                else:
                    if manual_read:
                        print(f"✗ 收到未知的读响应格式: {print_hex(packet)}")
                    return None
        else:
            if manual_read:
                print("✗ 读取超时，未收到读写器响应")
            return None

    def read_all_memory_banks(self, epc_hex):
        """读取标签的所有存储区域"""
        print(f"\n=== 读取标签 {epc_hex} 的所有存储区域 ===")
        
        memory_banks = {
            0x00: ("RESERVED", "保留区"),
            0x01: ("EPC", "EPC区"),
            0x02: ("TID", "TID区"),
            0x03: ("USER", "用户区")
        }
        
        for bank_id, (bank_name, bank_desc) in memory_banks.items():
            print(f"\n--- {bank_desc} ({bank_name}) ---")
            
            # 设置EPC匹配
            self._set_epc_match(epc_hex)
            
            # 构造读取命令
            word_add = 0
            word_count = 8 if bank_id == 0x03 else 4  # USER区读取更多数据
            cmd_len = 12
            cmd_body = bytearray([0xA0, cmd_len, READER_ADDRESS, 0x81])
            cmd_body += bytearray([bank_id])  # MemBank
            cmd_body += word_add.to_bytes(2, 'big')  # WordAdd
            cmd_body += word_count.to_bytes(2, 'big')  # WordCnt
            cmd_body += ACCESS_PASSWORD  # PassWord
            
            global last_read_response, response_event
            response_event.clear()
            last_read_response = None
            self.send_command(cmd_body)
            
            if response_event.wait(timeout=2.0):
                packet = last_read_response
                if packet and packet[3] == 0x81:
                    response_payload = packet[4:-1]
                    
                    if len(packet) == 6:
                        err_code = response_payload[0]
                        error_desc = ERROR_CODES.get(err_code, "未知错误")
                        print(f"读取失败: {err_code:02X} ({error_desc})")
                        continue
                    
                    if len(response_payload) > 4:
                        read_len_bytes = response_payload[-3] if len(response_payload) >= 3 else 0
                        if read_len_bytes > 0:
                            data_field_len = response_payload[2]
                            start_of_read_data = 3 + (data_field_len - read_len_bytes)
                            end_of_read_data = start_of_read_data + read_len_bytes
                            
                            if end_of_read_data <= len(response_payload) - 3:
                                data_bytes = response_payload[start_of_read_data:end_of_read_data]
                                print(f"十六进制数据: {print_hex(data_bytes)}")
                                
                                # 尝试解码为ASCII字符串
                                try:
                                    ascii_str = data_bytes.decode('ascii', errors='ignore').strip('\x00')
                                    if ascii_str:
                                        print(f"ASCII解码: '{ascii_str}'")
                                except:
                                    pass
                                
                                # 对于USER区域，尝试UTF-8解码
                                if bank_id == 0x03:
                                    try:
                                        utf8_str = data_bytes.strip(b'\x00').decode('utf-8', errors='ignore')
                                        if utf8_str:
                                            print(f"UTF-8解码: '{utf8_str}'")
                                    except:
                                        pass
                        else:
                            print("该区域为空")
                else:
                    print("读取失败：无效响应")
            else:
                print("读取超时")
            
            time.sleep(0.2)  # 给读写器一点休息时间

    def start(self):
        if not self.connect():
            return
        self.is_running = True
        self.session_start_time = datetime.now()
        threading.Thread(target=self.receive_data, daemon=True).start()
        self.display_thread = threading.Thread(target=self.display_status, daemon=True)
        self.display_thread.start()
        self.send_command(CMD_SET_ANTENNA_1)
        time.sleep(0.5)
        try:
            while self.is_running:
                clear_screen()
                print("=" * 40)
                print("          RFID 读写操作菜单")
                print("=" * 40)
                print("1. 开始实时盘存")
                print("2. 写入数据到标签")
                print("3. 读取标签数据")
                print("4. 读取标签所有存储区")
                print("q. 退出程序")
                print("-" * 40)
                choice = input("请输入选项: ").strip().lower()
                if choice == '1':
                    inventory_thread = threading.Thread(target=self.start_inventory_loop, daemon=True)
                    inventory_thread.start()
                    print("按 's' 停止盘存并返回菜单...")
                    while True:
                        try:
                           if input() == 's':
                               self.stop_inventory_loop()
                               break
                        except (EOFError, KeyboardInterrupt):
                           self.stop_inventory_loop()
                           break
                elif choice == '2':
                    self.handle_write_tag()
                elif choice == '3':
                    self.handle_read_tag()
                elif choice == '4':
                    self.handle_read_all_banks()
                elif choice == 'q':
                    self.is_running = False
                else:
                    print("无效输入，请重新选择。")
                    time.sleep(1)
        except KeyboardInterrupt:
            print("\n程序被中断。")
        finally:
            self.disconnect()
            print("\n程序已退出。")

    def handle_write_tag(self):
        with self.lock:
            if not self.current_tags:
                print("\n错误：当前没有盘存到任何标签，请先进行盘存。")
                time.sleep(2)
                return
            sorted_tags = sorted(self.current_tags.items(), key=lambda x: x[1].last_seen, reverse=True)
        clear_screen()
        print("--- 请选择要写入的标签 ---")
        for idx, (epc, tag_info) in enumerate(sorted_tags, 1):
            print(f"{idx}. EPC: {epc} (RSSI: {rssi_to_dbm(tag_info.rssi)}dBm)")
        print("--------------------------")
        try:
            tag_idx_str = input("请输入标签序号 (或按 Enter 取消): ")
            if not tag_idx_str:
                return
            tag_idx = int(tag_idx_str) - 1
            if not (0 <= tag_idx < len(sorted_tags)):
                print("无效序号。")
                time.sleep(1)
                return
            target_epc = sorted_tags[tag_idx][0]
            data_to_write = input(f"请输入要为 {target_epc[:18]}... 写入的物品名称: ")
            if not data_to_write:
                print("输入为空，操作取消。")
                time.sleep(1)
                return
            was_inventoring = self.inventory_active
            if was_inventoring:
                self.stop_inventory_loop()
                time.sleep(0.5)
            self.write_user_data(target_epc, data_to_write)
            if was_inventoring:
                threading.Thread(target=self.start_inventory_loop, daemon=True).start()
            input("\n按 Enter键 返回主菜单...")
        except (ValueError, IndexError):
            print("输入无效。")
            time.sleep(2)

    def handle_read_tag(self):
        """处理手动读取标签数据"""
        with self.lock:
            if not self.current_tags:
                print("\n错误：当前没有盘存到任何标签，请先进行盘存。")
                time.sleep(2)
                return
            sorted_tags = sorted(self.current_tags.items(), key=lambda x: x[1].last_seen, reverse=True)
        
        clear_screen()
        print("--- 请选择要读取的标签 ---")
        for idx, (epc, tag_info) in enumerate(sorted_tags, 1):
            rssi_dbm = rssi_to_dbm(tag_info.rssi)
            current_data = tag_info.user_data if tag_info.user_data else "未读取"
            print(f"{idx}. EPC: {epc} (RSSI: {rssi_dbm}dBm) 当前数据: {current_data}")
        print("--------------------------")
        
        try:
            tag_idx_str = input("请输入标签序号 (或按 Enter 取消): ")
            if not tag_idx_str:
                return
            
            tag_idx = int(tag_idx_str) - 1
            if not (0 <= tag_idx < len(sorted_tags)):
                print("无效序号。")
                time.sleep(2)
                return
            
            target_epc = sorted_tags[tag_idx][0]
            
            # 停止盘存以避免干扰
            was_inventoring = self.inventory_active
            if was_inventoring:
                self.stop_inventory_loop()
                time.sleep(0.5)
            
            # 执行读取
            result = self.read_user_data(target_epc, word_count=32, manual_read=True)
            
            # 恢复盘存
            if was_inventoring:
                threading.Thread(target=self.start_inventory_loop, daemon=True).start()
            
            input("\n按 Enter键 返回主菜单...")
            
        except (ValueError, IndexError):
            print("输入无效。")
            time.sleep(2)

    def handle_read_all_banks(self):
        """处理读取标签所有存储区域"""
        with self.lock:
            if not self.current_tags:
                print("\n错误：当前没有盘存到任何标签，请先进行盘存。")
                time.sleep(2)
                return
            sorted_tags = sorted(self.current_tags.items(), key=lambda x: x[1].last_seen, reverse=True)
        
        clear_screen()
        print("--- 请选择要全面读取的标签 ---")
        for idx, (epc, tag_info) in enumerate(sorted_tags, 1):
            rssi_dbm = rssi_to_dbm(tag_info.rssi)
            print(f"{idx}. EPC: {epc} (RSSI: {rssi_dbm}dBm)")
        print("--------------------------")
        
        try:
            tag_idx_str = input("请输入标签序号 (或按 Enter 取消): ")
            if not tag_idx_str:
                return
            
            tag_idx = int(tag_idx_str) - 1
            if not (0 <= tag_idx < len(sorted_tags)):
                print("无效序号。")
                time.sleep(2)
                return
            
            target_epc = sorted_tags[tag_idx][0]
            
            # 停止盘存以避免干扰
            was_inventoring = self.inventory_active
            if was_inventoring:
                self.stop_inventory_loop()
                time.sleep(0.5)
            
            # 执行全面读取
            self.read_all_memory_banks(target_epc)
            
            # 恢复盘存
            if was_inventoring:
                threading.Thread(target=self.start_inventory_loop, daemon=True).start()
            
            input("\n按 Enter键 返回主菜单...")
            
        except (ValueError, IndexError):
            print("输入无效。")
            time.sleep(2)


if __name__ == "__main__":
    reader = RFIDReader(READER_IP, READER_PORT)
    reader.start()