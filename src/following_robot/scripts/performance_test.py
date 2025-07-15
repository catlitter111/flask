#!/usr/bin/env python3
"""
性能测试脚本：监控通信优化效果
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from std_msgs.msg import String
import time
import json
import sys

class PerformanceMonitor(Node):
    def __init__(self):
        super().__init__('performance_monitor')
        
        # 性能统计
        self.stats = {
            'image_messages': 0,
            'detection_messages': 0,
            'position_messages': 0,
            'image_hz': 0.0,
            'detection_hz': 0.0,
            'position_hz': 0.0,
            'latency_image_to_detection': [],
            'latency_detection_to_position': [],
            'detection_processing_time': [],
            'total_persons_detected': 0,
        }
        
        # 时间戳记录
        self.last_image_time = 0
        self.last_detection_time = 0
        self.last_position_time = 0
        
        # 消息时间戳
        self.image_timestamps = []
        self.detection_timestamps = []
        self.position_timestamps = []
        
        # QoS配置
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # 订阅各种话题
        self.image_sub = self.create_subscription(
            Image, '/camera/color/image_raw', self.image_callback, sensor_qos)
        
        self.detection_sub = self.create_subscription(
            Detection2DArray, '/detections', self.detection_callback, 10)
        
        self.position_sub = self.create_subscription(
            String, '/person_detection/person_positions', self.position_callback, 10)
        
        # 定时器报告性能
        self.timer = self.create_timer(2.0, self.report_performance)
        
        self.get_logger().info('Performance Monitor started')
        print("\n=== 性能监控器已启动 ===")
        print("监控话题:")
        print("- /camera/color/image_raw")
        print("- /detections")
        print("- /person_detection/person_positions")
        print("每2秒报告一次性能统计\n")
    
    def image_callback(self, msg):
        current_time = time.time()
        self.stats['image_messages'] += 1
        self.last_image_time = current_time
        
        # 记录时间戳用于计算频率
        self.image_timestamps.append(current_time)
        if len(self.image_timestamps) > 100:
            self.image_timestamps.pop(0)
    
    def detection_callback(self, msg):
        current_time = time.time()
        self.stats['detection_messages'] += 1
        self.last_detection_time = current_time
        
        # 记录检测延迟
        if self.last_image_time > 0:
            latency = current_time - self.last_image_time
            self.stats['latency_image_to_detection'].append(latency)
            if len(self.stats['latency_image_to_detection']) > 100:
                self.stats['latency_image_to_detection'].pop(0)
        
        # 记录检测结果数量
        self.stats['total_persons_detected'] += len(msg.detections)
        
        # 记录时间戳
        self.detection_timestamps.append(current_time)
        if len(self.detection_timestamps) > 100:
            self.detection_timestamps.pop(0)
    
    def position_callback(self, msg):
        current_time = time.time()
        self.stats['position_messages'] += 1
        self.last_position_time = current_time
        
        # 记录处理延迟
        if self.last_detection_time > 0:
            latency = current_time - self.last_detection_time
            self.stats['latency_detection_to_position'].append(latency)
            if len(self.stats['latency_detection_to_position']) > 100:
                self.stats['latency_detection_to_position'].pop(0)
        
        # 解析位置数据
        try:
            data = json.loads(msg.data)
            person_count = data.get('person_count', 0)
            self.get_logger().debug(f'Position update: {person_count} persons')
        except:
            pass
        
        # 记录时间戳
        self.position_timestamps.append(current_time)
        if len(self.position_timestamps) > 100:
            self.position_timestamps.pop(0)
    
    def calculate_hz(self, timestamps):
        """计算消息频率"""
        if len(timestamps) < 2:
            return 0.0
        
        time_span = timestamps[-1] - timestamps[0]
        if time_span <= 0:
            return 0.0
        
        return (len(timestamps) - 1) / time_span
    
    def report_performance(self):
        """报告性能统计"""
        # 计算消息频率
        self.stats['image_hz'] = self.calculate_hz(self.image_timestamps)
        self.stats['detection_hz'] = self.calculate_hz(self.detection_timestamps)
        self.stats['position_hz'] = self.calculate_hz(self.position_timestamps)
        
        # 计算延迟统计
        avg_img_to_det = 0.0
        avg_det_to_pos = 0.0
        
        if self.stats['latency_image_to_detection']:
            avg_img_to_det = sum(self.stats['latency_image_to_detection']) / len(self.stats['latency_image_to_detection'])
        
        if self.stats['latency_detection_to_position']:
            avg_det_to_pos = sum(self.stats['latency_detection_to_position']) / len(self.stats['latency_detection_to_position'])
        
        # 打印性能报告
        print(f"\n{'='*60}")
        print(f"性能统计报告 - {time.strftime('%H:%M:%S')}")
        print(f"{'='*60}")
        print(f"消息频率:")
        print(f"  图像消息:     {self.stats['image_hz']:.1f} Hz")
        print(f"  检测消息:     {self.stats['detection_hz']:.1f} Hz")
        print(f"  位置消息:     {self.stats['position_hz']:.1f} Hz")
        print(f"")
        print(f"延迟统计:")
        print(f"  图像→检测:    {avg_img_to_det*1000:.1f} ms")
        print(f"  检测→位置:    {avg_det_to_pos*1000:.1f} ms")
        print(f"  总延迟:       {(avg_img_to_det + avg_det_to_pos)*1000:.1f} ms")
        print(f"")
        print(f"消息计数:")
        print(f"  图像消息:     {self.stats['image_messages']}")
        print(f"  检测消息:     {self.stats['detection_messages']}")
        print(f"  位置消息:     {self.stats['position_messages']}")
        print(f"")
        print(f"检测统计:")
        print(f"  总检测人数:   {self.stats['total_persons_detected']}")
        
        # 性能评估
        if self.stats['image_hz'] > 0 and self.stats['detection_hz'] > 0:
            efficiency = (self.stats['detection_hz'] / self.stats['image_hz']) * 100
            print(f"  处理效率:     {efficiency:.1f}%")
        
        # 延迟评估
        total_latency = (avg_img_to_det + avg_det_to_pos) * 1000
        if total_latency < 100:
            latency_grade = "优秀"
        elif total_latency < 200:
            latency_grade = "良好"
        elif total_latency < 300:
            latency_grade = "一般"
        else:
            latency_grade = "需要优化"
        
        print(f"  延迟评级:     {latency_grade}")
        print(f"{'='*60}")

def main():
    rclpy.init()
    
    try:
        monitor = PerformanceMonitor()
        print("按 Ctrl+C 停止监控")
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        print("\n性能监控已停止")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main() 