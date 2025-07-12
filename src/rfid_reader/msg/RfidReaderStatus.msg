# RFID读写器状态信息
std_msgs/Header header
bool connected          # 连接状态
string reader_ip        # 读写器IP地址
uint16 reader_port      # 读写器端口
uint8 antenna_id        # 当前工作天线ID
uint16 read_rate        # 读取速率 (次/秒)
uint32 total_read       # 总读取次数
builtin_interfaces/Time session_start_time  # 会话开始时间
float64 session_duration # 会话持续时间 (秒)