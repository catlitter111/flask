# RFID标签信息
std_msgs/Header header
string epc               # 电子产品码
uint8[] pc               # 协议控制位
int8 rssi_dbm           # 信号强度 (dBm)
uint8 rssi_raw          # 原始RSSI值
uint8 antenna_id        # 天线ID
uint8 freq_param        # 频点参数
builtin_interfaces/Time first_seen    # 首次检测时间
builtin_interfaces/Time last_seen     # 最后检测时间
uint32 read_count       # 读取次数