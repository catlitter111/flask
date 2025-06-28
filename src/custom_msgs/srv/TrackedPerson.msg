# 被跟踪人员信息
int32 track_id                 # 跟踪ID
sensor_msgs/RegionOfInterest bbox  # 边界框
float32 confidence            # 置信度
int32[] upper_color           # 上装颜色 [B, G, R]
int32[] lower_color           # 下装颜色 [B, G, R]
float32[] body_ratios         # 身体比例特征（16个）
bool is_target                # 是否为目标人物