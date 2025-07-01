# RobotStatus.msg - 机器人状态消息
# 用于传递机器人的完整状态信息

# 基本信息
std_msgs/Header header
string robot_id                 # 机器人唯一标识

# 连接状态
bool connected                  # 是否连接
int32 signal_strength          # 信号强度 [0-100]
float32 network_latency        # 网络延迟（毫秒）

# 电源状态
int32 battery_level            # 电池电量 [0-100]
bool charging                  # 是否正在充电
float32 voltage                # 电压（伏特）
float32 current                # 电流（安培）

# 运行状态
string mode                    # 运行模式: "idle", "tracking", "manual", "emergency"
bool is_moving                 # 是否正在移动
float32 speed                  # 当前速度（米/秒）
geometry_msgs/Twist velocity   # 当前速度向量

# 位置信息
geometry_msgs/Point position   # 机器人位置（米）
float32 orientation           # 方向角度（弧度）
geometry_msgs/Point target_position  # 目标位置（如果在跟踪模式）

# 传感器状态
bool camera_active            # 摄像头是否激活
bool stereo_vision_active     # 立体视觉是否激活
bool lidar_active             # 激光雷达是否激活（如果有）
float32 cpu_usage             # CPU使用率 [0.0-1.0]
float32 memory_usage          # 内存使用率 [0.0-1.0]
float32 temperature           # 设备温度（摄氏度）

# 错误和警告
string[] errors               # 错误列表
string[] warnings             # 警告列表
int32 error_count            # 错误计数
int32 warning_count          # 警告计数

# 时间戳
int64 uptime                 # 运行时间（秒）
int64 last_update           # 最后更新时间戳（毫秒） 