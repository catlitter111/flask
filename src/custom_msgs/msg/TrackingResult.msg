# TrackingResult.msg - 目标跟踪结果消息
# 用于传递bytetracker节点的跟踪结果

# 基本信息
std_msgs/Header header
string mode                      # 跟踪模式: "multi_target" 或 "single_target"
int32 total_tracks              # 总跟踪目标数量
bool target_detected            # 是否检测到特定目标（单目标模式）

# 目标信息（单目标模式）
int32 target_id                 # 目标ID，-1表示无目标
float32 target_x                # 目标中心X坐标（像素）
float32 target_y                # 目标中心Y坐标（像素）
float32 target_width            # 目标宽度（像素）
float32 target_height           # 目标高度（像素）
float32 confidence              # 跟踪置信度 [0.0-1.0]
float32 distance                # 估计距离（米）

# 多目标信息
int32[] track_ids               # 所有跟踪目标的ID列表
float32[] track_confidences     # 对应的置信度列表
geometry_msgs/Point[] positions # 所有目标的位置列表

# 状态信息
string tracking_status          # 跟踪状态: "idle", "tracking", "lost", "searching"
float32 fps                     # 当前处理帧率
int64 frame_count              # 已处理帧数量 