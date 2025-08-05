#!/usr/bin/env python3
"""
ByteTracker与集成人员检测节点集成测试脚本
========================================
测试ByteTracker节点订阅集成人员检测节点的数据，验证数据流和功能
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from custom_msgs.msg import TrackedPersonArray, TrackingResult
import json
import time
import sys

class ByteTrackerIntegrationTester(Node):
    
    def __init__(self):
        super().__init__('bytetracker_integration_tester')
        
        # 订阅人员位置话题（来自集成检测节点）
        self.person_positions_sub = self.create_subscription(
            String,
            '/person_detection/person_positions',
            self.person_positions_callback,
            10
        )
        
        # 订阅ByteTracker输出话题
        self.tracked_persons_sub = self.create_subscription(
            TrackedPersonArray,
            '/bytetracker/tracked_persons',
            self.tracked_persons_callback,
            10
        )
        
        self.tracking_result_sub = self.create_subscription(
            TrackingResult,
            '/bytetracker/tracking_result',
            self.tracking_result_callback,
            10
        )
        
        self.detailed_tracking_sub = self.create_subscription(
            String,
            '/bytetracker/detailed_tracking_data',
            self.detailed_tracking_callback,
            10
        )
        
        # 发布控制命令
        self.mode_pub = self.create_publisher(
            String,
            '/bytetracker/set_mode',
            10
        )
        
        # 统计数据
        self.person_positions_count = 0
        self.tracked_persons_count = 0
        self.tracking_result_count = 0
        self.detailed_tracking_count = 0
        self.last_log_time = time.time()
        
        # 数据验证
        self.last_person_data = None
        self.last_tracking_data = None
        
        self.get_logger().info("🧪 ByteTracker集成测试器启动")
        self.get_logger().info("📊 监控数据流...")
        self.get_logger().info("🔍 验证数据一致性...")
        
        # 定时器用于状态报告
        self.status_timer = self.create_timer(5.0, self.print_status)
        
        # 模式切换测试定时器（可选）
        # self.mode_test_timer = self.create_timer(15.0, self.test_mode_switching)
        
    def person_positions_callback(self, msg):
        """处理人员位置数据（检测节点输出）"""
        try:
            data = json.loads(msg.data)
            self.person_positions_count += 1
            self.last_person_data = data
            
            current_time = time.time()
            if current_time - self.last_log_time > 3.0:  # 每3秒记录一次
                person_count = data.get('person_count', 0)
                detection_mode = data.get('detection_mode', 'UNKNOWN')
                
                self.get_logger().info(f"📥 接收检测数据 #{self.person_positions_count}")
                self.get_logger().info(f"   检测到人数: {person_count}")
                self.get_logger().info(f"   检测模式: {detection_mode}")
                
                persons = data.get('persons', [])
                for i, person in enumerate(persons[:3]):  # 只显示前3个人
                    person_id = person.get('id', f'Unknown_{i}')
                    bbox = person.get('bbox', [])
                    clothing = person.get('clothing', {})
                    has_upper = 'upper' in clothing and clothing['upper']
                    has_lower = 'lower' in clothing and clothing['lower']
                    has_body_ratios = person.get('has_body_ratios', False)
                    
                    features = []
                    if has_upper:
                        features.append("上衣")
                    if has_lower:
                        features.append("下装")
                    if has_body_ratios:
                        features.append("身体比例")
                    
                    self.get_logger().info(f"   {person_id}: 边界框{bbox[:4] if len(bbox)>=4 else bbox}, 特征[{', '.join(features)}]")
                
        except json.JSONDecodeError as e:
            self.get_logger().error(f"❌ 解析人员位置JSON失败: {e}")
        except Exception as e:
            self.get_logger().error(f"❌ 处理人员位置数据时出错: {e}")
    
    def tracked_persons_callback(self, msg):
        """处理跟踪结果数据（ByteTracker输出）"""
        self.tracked_persons_count += 1
        self.last_tracking_data = msg
        
        if self.tracked_persons_count % 10 == 0:  # 每10次记录一次
            self.get_logger().info(f"📤 接收跟踪数据 #{self.tracked_persons_count}")
            self.get_logger().info(f"   跟踪目标数: {len(msg.persons)}")
            
            for person in msg.persons[:3]:  # 只显示前3个
                features = []
                if person.upper_color:
                    features.append(f"上衣{person.upper_color}")
                if person.lower_color:
                    features.append(f"下装{person.lower_color}")
                if person.body_ratios:
                    features.append(f"比例({len(person.body_ratios)})")
                if person.is_target:
                    features.append("目标")
                
                self.get_logger().info(f"   轨迹{person.track_id}: 置信度{person.confidence:.2f}, 特征[{', '.join(features)}]")
    
    def tracking_result_callback(self, msg):
        """处理跟踪结果摘要"""
        self.tracking_result_count += 1
        
        if self.tracking_result_count % 15 == 0:  # 每15次记录一次
            self.get_logger().info(f"🎯 跟踪状态 #{self.tracking_result_count}")
            self.get_logger().info(f"   模式: {msg.mode}")
            self.get_logger().info(f"   状态: {msg.tracking_status}")
            self.get_logger().info(f"   目标检测: {msg.target_detected}")
            self.get_logger().info(f"   总轨迹数: {msg.total_tracks}")
            self.get_logger().info(f"   FPS: {msg.fps:.1f}")
    
    def detailed_tracking_callback(self, msg):
        """处理详细跟踪数据"""
        try:
            data = json.loads(msg.data)
            self.detailed_tracking_count += 1
            
            if self.detailed_tracking_count % 20 == 0:  # 每20次记录一次
                tracking_mode = data.get('tracking_mode', 'unknown')
                target_detected = data.get('target_detected', False)
                total_tracks = data.get('total_tracks', 0)
                
                self.get_logger().info(f"📊 详细跟踪数据 #{self.detailed_tracking_count}")
                self.get_logger().info(f"   跟踪模式: {tracking_mode}")
                self.get_logger().info(f"   目标检测: {target_detected}")
                self.get_logger().info(f"   活跃轨迹: {total_tracks}")
                
                system_info = data.get('system_info', {})
                fps = system_info.get('fps', 0)
                processing_time = system_info.get('processing_time_ms', 0)
                memory_usage = system_info.get('memory_usage_mb', 0)
                
                self.get_logger().info(f"   系统性能: FPS={fps:.1f}, 处理时间={processing_time:.1f}ms, 内存={memory_usage:.1f}MB")
                
        except json.JSONDecodeError as e:
            self.get_logger().error(f"❌ 解析详细跟踪JSON失败: {e}")
        except Exception as e:
            self.get_logger().error(f"❌ 处理详细跟踪数据时出错: {e}")
    
    def print_status(self):
        """定期打印状态摘要"""
        self.get_logger().info("="*60)
        self.get_logger().info("📊 ByteTracker集成测试状态摘要")
        self.get_logger().info(f"   检测数据接收次数: {self.person_positions_count}")
        self.get_logger().info(f"   跟踪结果接收次数: {self.tracked_persons_count}")
        self.get_logger().info(f"   跟踪状态接收次数: {self.tracking_result_count}")
        self.get_logger().info(f"   详细数据接收次数: {self.detailed_tracking_count}")
        
        # 数据一致性检查
        if self.last_person_data and self.last_tracking_data:
            person_count = self.last_person_data.get('person_count', 0)
            tracking_count = len(self.last_tracking_data.persons)
            
            self.get_logger().info(f"   数据一致性: 检测{person_count}人 → 跟踪{tracking_count}人")
            
            if person_count > 0 and tracking_count == 0:
                self.get_logger().warn("⚠️  检测到人员但无跟踪结果，可能需要调整参数")
            elif person_count == 0 and tracking_count > 0:
                self.get_logger().info("ℹ️  无新检测但保持跟踪（正常情况）")
        
        self.get_logger().info("="*60)
    
    def test_mode_switching(self):
        """测试模式切换功能"""
        modes = ['multi', 'single']
        current_mode = getattr(self, 'current_test_mode', 0)
        mode = modes[current_mode % len(modes)]
        
        self.get_logger().info(f"🔄 测试模式切换: {mode}")
        
        mode_msg = String()
        mode_msg.data = mode
        self.mode_pub.publish(mode_msg)
        
        self.current_test_mode = current_mode + 1

def main(args=None):
    """主函数"""
    try:
        rclpy.init(args=args)
        
        print("🧪 ByteTracker集成测试器")
        print("=" * 50)
        print("📋 测试内容:")
        print("   1. 验证数据流: 检测节点 → ByteTracker")
        print("   2. 检查数据格式一致性")
        print("   3. 监控跟踪性能")
        print("   4. 验证特征传递")
        print("=" * 50)
        print("🚀 启动测试...")
        print()
        
        tester = ByteTrackerIntegrationTester()
        
        try:
            rclpy.spin(tester)
        except KeyboardInterrupt:
            print("\n🛑 用户中断测试")
        finally:
            print("\n📊 测试总结:")
            print(f"   检测数据接收: {tester.person_positions_count} 次")
            print(f"   跟踪结果接收: {tester.tracked_persons_count} 次")
            print(f"   系统状态接收: {tester.tracking_result_count} 次")
            print(f"   详细数据接收: {tester.detailed_tracking_count} 次")
            
            if tester.person_positions_count > 0 and tester.tracked_persons_count > 0:
                print("✅ 数据流验证成功")
            else:
                print("❌ 数据流验证失败 - 检查节点是否正常运行")
            
            tester.destroy_node()
            rclpy.shutdown()
            
    except Exception as e:
        print(f"❌ 测试错误: {e}")
        import traceback
        traceback.print_exc()

if __name__ == '__main__':
    main() 