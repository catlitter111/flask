#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import json
import time
import threading
import math

class PersonDetectionDistanceNode(Node):
    """
    人体检测和距离查询节点
    通过订阅YOLO11检测结果并进行服装配对来确定人体位置，并查询距离信息
    """
    
    def __init__(self):
        super().__init__('person_detection_distance_node')
        
        # 初始化CV bridge
        self.bridge = CvBridge()
        
        # 存储最新的图像和检测结果
        self.latest_image = None
        self.latest_detections = None
        
        # 服装类别定义
        self.CLOTHING_CATEGORIES = {
            'upper': [
                'short_sleeved_shirt',
                'long_sleeved_shirt', 
                'short_sleeved_outwear',
                'long_sleeved_outwear',
                'vest',
                'sling',
            ],
            'lower': [
                'shorts',
                'trousers',
                'skirt',
                'short_sleeved_dress',
                'long_sleeved_dress',
                'vest_dress',
                'sling_dress'
            ]
        }
        
        # QoS配置
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # 订阅相机原始图像
        self.image_sub = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            sensor_qos
        )
        
        # 订阅YOLO11检测结果
        self.detection_sub = self.create_subscription(
            Detection2DArray,
            '/detections',
            self.detection_callback,
            10
        )
        
        # 发布检测结果（包含框选和距离信息）
        self.detection_pub = self.create_publisher(
            Image,
            '/bytetracker/visualization',
            10
        )
        
        # 发布人体位置信息
        self.position_pub = self.create_publisher(
            String,
            '/person_detection/person_positions',
            10
        )
        
        # 订阅深度查询结果
        self.depth_result_sub = self.create_subscription(
            String,
            '/depth_reader/depth_value',
            self.depth_result_callback,
            10
        )
        
        # 发布深度查询请求
        self.depth_query_pub = self.create_publisher(
            String,
            '/depth_reader/get_depth_at',
            10
        )
        
        # 存储人体检测结果和距离信息
        self.person_positions = []
        self.distance_queries = {}  # 存储待查询的距离信息
        self.person_distances = {}  # 持久化存储人体距离信息
        
        # 图像显示相关
        self.display_image = None
        self.display_lock = threading.Lock()
        
        # 帧率计算
        self.frame_count = 0
        self.fps = 0.0
        self.last_fps_time = time.time()
        self.fps_frame_count = 0
        
        # 定时器处理图像 - 改为更高频率以获得真实FPS
        self.timer = self.create_timer(0.033, self.process_detections)  # 约30 FPS
        
        # 启动图像显示线程
        self.display_thread = threading.Thread(target=self.display_loop, daemon=True)
        self.display_thread.start()
        
        self.get_logger().info('Person Detection Distance Node initialized')
        self.get_logger().info('Subscribing to /detections topic for YOLO11 results')
        self.get_logger().info('Waiting for camera images and detection results...')
        self.get_logger().info('Press "q" in the image window to quit')
    
    def image_callback(self, msg):
        """处理接收到的图像"""
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}')
    
    def detection_callback(self, msg):
        """处理接收到的YOLO11检测结果"""
        try:
            self.latest_detections = msg
            self.get_logger().debug(f'Received {len(msg.detections)} detections')
        except Exception as e:
            self.get_logger().error(f'Error processing detection results: {e}')
    
    def depth_result_callback(self, msg):
        """处理深度查询结果"""
        try:
            result = json.loads(msg.data)
            query_id = f"{result['x']}_{result['y']}"
            
            # 更新对应人体的距离信息
            if query_id in self.distance_queries:
                person_id = self.distance_queries[query_id]['person_id']
                
                # 持久化存储距离信息
                if result.get('valid', False) and result.get('depth_m') is not None:
                    self.person_distances[person_id] = {
                        'distance': result.get('depth_m'),
                        'valid': True,
                        'timestamp': time.time(),
                        'x': result['x'],
                        'y': result['y']
                    }
                
                # 同时更新当前帧的人体信息
                for person in self.person_positions:
                    if person['id'] == person_id:
                        person['distance'] = result.get('depth_m', None)
                        person['valid_distance'] = result.get('valid', False)
                        break
                
                # 清理已处理的查询
                del self.distance_queries[query_id]
                
        except Exception as e:
            self.get_logger().error(f'Error processing depth result: {e}')
    
    def convert_detections_to_clothing_format(self, detections_msg):
        """将Detection2DArray消息转换为服装检测格式"""
        upper_clothing = []
        lower_clothing = []
        
        if detections_msg is None:
            return upper_clothing, lower_clothing
        
        for detection in detections_msg.detections:
            if not detection.results:
                continue
                
            # 获取类别和置信度
            class_id = detection.results[0].hypothesis.class_id
            confidence = detection.results[0].hypothesis.score
            
            # 获取边界框坐标
            center_x = detection.bbox.center.position.x
            center_y = detection.bbox.center.position.y
            size_x = detection.bbox.size_x
            size_y = detection.bbox.size_y
            
            # 计算边界框坐标
            xmin = int(center_x - size_x / 2)
            ymin = int(center_y - size_y / 2)
            xmax = int(center_x + size_x / 2)
            ymax = int(center_y + size_y / 2)
            
            # 分类到上衣或下装
            if class_id in self.CLOTHING_CATEGORIES['upper']:
                upper_clothing.append((xmin, ymin, xmax, ymax, confidence, 'upper'))
            elif class_id in self.CLOTHING_CATEGORIES['lower']:
                lower_clothing.append((xmin, ymin, xmax, ymax, confidence, 'lower'))
        
        return upper_clothing, lower_clothing
    
    def match_clothing_items(self, upper_items, lower_items, img):
        """匹配上衣和下装"""
        pairs = []
        
        if not upper_items and not lower_items:
            return pairs
            
        height, width = img.shape[:2]
        max_x_distance = width * 0.2  # 最大水平距离比例
        
        # 简化为坐标
        simplified_upper = [item[:4] for item in upper_items]
        simplified_lower = [item[:4] for item in lower_items]
        lower_items_copy = simplified_lower.copy()
        
        # 对每件上衣寻找最近的下装
        for upper in simplified_upper:
            closest_lower = None
            min_distance = float('inf')
            
            # 计算上衣中心点
            upper_center_x = (upper[0] + upper[2]) // 2
            upper_center_y = (upper[1] + upper[3]) // 2
            
            for lower in lower_items_copy:
                # 计算下装中心点
                lower_center_x = (lower[0] + lower[2]) // 2
                lower_center_y = (lower[1] + lower[3]) // 2
                
                # 检查水平距离约束
                x_distance = abs(upper_center_x - lower_center_x)
                if x_distance > max_x_distance:
                    continue
                
                # 计算中心点之间的欧氏距离
                distance = ((upper_center_x - lower_center_x) ** 2 +
                           (upper_center_y - lower_center_y) ** 2) ** 0.5
                
                # 更新最近匹配
                if distance < min_distance:
                    min_distance = distance
                    closest_lower = lower
            
            # 添加配对或未配对的上衣
            if closest_lower:
                pairs.append([upper, closest_lower])
                lower_items_copy.remove(closest_lower)
            else:
                pairs.append([upper, (-1,)])
        
        # 添加剩余未配对的下装
        for lower in lower_items_copy:
            pairs.append([(-1,), lower])
        
        return pairs
    
    def determine_person_position(self, upper_coords, lower_coords, img):
        """确定整个人体位置"""
        person_positions = []
        
        if img is None or img.size == 0:
            return person_positions
            
        img_height, img_width = img.shape[:2]
        
        try:
            if len(upper_coords) == 4 and len(lower_coords) == 4:
                # 两者都检测到 - 计算整个人体区域
                person_xmin = min(upper_coords[0], lower_coords[0])
                person_ymin = min(upper_coords[1], lower_coords[1])
                person_xmax = max(upper_coords[2], lower_coords[2])
                person_ymax = max(upper_coords[3], lower_coords[3])
                
                # 添加头部和脚部扩展
                upper_height = upper_coords[3] - upper_coords[1]
                lower_height = lower_coords[3] - lower_coords[1]
                upper_width = upper_coords[2] - upper_coords[0]
                
                head_extension = int(upper_height * 0.5)
                foot_extension = int(lower_height * 0.6)
                side_extension = int(upper_width * 0.3)
                
                # 应用扩展（带边界检查）
                person_xmin = max(0, person_xmin - side_extension)
                person_xmax = min(img_width, person_xmax + side_extension)
                person_ymin = max(0, person_ymin - head_extension)
                person_ymax = min(img_height, person_ymax + foot_extension)
                
            elif len(upper_coords) == 4 and len(lower_coords) == 1:
                # 只有上衣
                upper_height = upper_coords[3] - upper_coords[1]
                
                person_xmin = max(0, upper_coords[0] - 40)
                person_ymin = max(0, upper_coords[1] - int(upper_height * 0.4))
                person_xmax = min(img_width, upper_coords[2] + 40)
                person_ymax = min(img_height, upper_coords[3] + int(upper_height * 2.5))
                
            elif len(upper_coords) == 1 and len(lower_coords) == 4:
                # 只有下装
                lower_height = lower_coords[3] - lower_coords[1]
                
                person_xmin = max(0, lower_coords[0] - 40)
                person_ymin = max(0, lower_coords[1] - int(lower_height * 1.8))
                person_xmax = min(img_width, lower_coords[2] + 40)
                person_ymax = min(img_height, lower_coords[3] + int(lower_height * 0.7))
            else:
                # 无效输入
                return person_positions
            
            person_positions.append((int(person_xmin), int(person_ymin),
                                   int(person_xmax), int(person_ymax)))
                                   
        except Exception as e:
            self.get_logger().error(f'Error determining person position: {e}')
            
        return person_positions
    
    def process_detections(self):
        """处理检测结果并进行服装配对"""
        if self.latest_image is None or self.latest_detections is None:
            return
            
        try:
            # 计算帧率
            self.fps_frame_count += 1
            current_time = time.time()
            if current_time - self.last_fps_time >= 1.0:
                self.fps = self.fps_frame_count / (current_time - self.last_fps_time)
                self.fps_frame_count = 0
                self.last_fps_time = current_time
                
            # 复制图像用于处理
            img = self.latest_image.copy()
            
            # 转换检测结果格式
            upper_clothing, lower_clothing = self.convert_detections_to_clothing_format(self.latest_detections)
            
            # 匹配上衣和下装
            pairs = self.match_clothing_items(upper_clothing, lower_clothing, img)
            
            # 存储检测到的人体位置
            self.person_positions = []
            
            # 处理每个配对
            for i, pair in enumerate(pairs):
                upper_coords = pair[0] if len(pair[0]) > 1 else None
                lower_coords = pair[1] if len(pair[1]) > 1 else None
                
                # 确定整个身体的位置
                if upper_coords or lower_coords:
                    # 处理缺失的坐标
                    if upper_coords is None:
                        upper_coords = (-1,)
                    if lower_coords is None:
                        lower_coords = (-1,)
                        
                    body_positions = self.determine_person_position(upper_coords, lower_coords, img)
                    
                    # 处理每个检测到的身体位置
                    for j, body_pos in enumerate(body_positions):
                        if len(body_pos) == 4:
                            xmin, ymin, xmax, ymax = body_pos
                            
                            # 计算人体中心点
                            center_x = int((xmin + xmax) / 2)
                            center_y = int((ymin + ymax) / 2)
                            
                            # 在图像上绘制人体框
                            cv2.rectangle(img, (xmin, ymin), (xmax, ymax), (0, 255, 0), 2)
                            
                            # 添加人体编号标签
                            person_id = f"Person_{i}_{j}"
                            cv2.putText(img, person_id, (xmin, ymin - 10),
                                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                            
                            # 标记中心点
                            cv2.circle(img, (center_x, center_y), 5, (255, 0, 0), -1)
                            
                            # 存储人体位置信息
                            person_info = {
                                'id': person_id,
                                'bbox': [xmin, ymin, xmax, ymax],
                                'center': [center_x, center_y],
                                'distance': None,
                                'valid_distance': False
                            }
                            
                            # 如果有历史距离数据，先使用历史数据
                            if person_id in self.person_distances:
                                cached_data = self.person_distances[person_id]
                                # 如果缓存数据不超过2秒，使用缓存数据
                                if time.time() - cached_data['timestamp'] < 2.0:
                                    person_info['distance'] = cached_data['distance']
                                    person_info['valid_distance'] = cached_data['valid']
                                else:
                                    # 清理过期数据
                                    del self.person_distances[person_id]
                            
                            self.person_positions.append(person_info)
                            
                            # 查询距离信息
                            self.query_distance(center_x, center_y, person_id)
            
            # 在上衣和下装上绘制检测框
            for upper_item in upper_clothing:
                xmin, ymin, xmax, ymax = upper_item[:4]
                cv2.rectangle(img, (xmin, ymin), (xmax, ymax), (255, 0, 0), 2)  # 蓝色框
                cv2.putText(img, "Upper", (xmin, ymin - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
            
            for lower_item in lower_clothing:
                xmin, ymin, xmax, ymax = lower_item[:4]
                cv2.rectangle(img, (xmin, ymin), (xmax, ymax), (0, 255, 0), 2)  # 绿色框
                cv2.putText(img, "Lower", (xmin, ymin - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            # 发布人体位置信息
            self.publish_person_positions()
            
            # 发布带标注的图像
            self.publish_annotated_image(img)
            
        except Exception as e:
            self.get_logger().error(f'Error processing detections: {e}')
    
    def query_distance(self, x, y, person_id):
        """查询指定像素点的距离信息"""
        try:
            query_id = f"{x}_{y}"
            
            # 如果已经有相同坐标的查询正在进行，跳过
            if query_id in self.distance_queries:
                return
            
            # 存储查询信息
            self.distance_queries[query_id] = {
                'person_id': person_id,
                'x': x,
                'y': y,
                'distance': None,
                'valid': False,
                'timestamp': time.time()
            }
            
            # 发布距离查询请求
            query_msg = String()
            query_msg.data = json.dumps({'x': x, 'y': y})
            self.depth_query_pub.publish(query_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error querying distance: {e}')
    
    def publish_annotated_image(self, img):
        """发布带标注的图像"""
        try:
            # 在图像上添加系统信息
            height, width = img.shape[:2]
            
            # 添加帧率信息
            fps_text = f"FPS: {self.fps:.1f}"
            cv2.putText(img, fps_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # 添加检测到的人数
            person_count_text = f"Persons: {len(self.person_positions)}"
            cv2.putText(img, person_count_text, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # 在图像上添加每个人的距离信息
            for person in self.person_positions:
                center_x, center_y = person['center']
                
                # 显示人员ID
                person_id_text = f"ID: {person['id']}"
                cv2.putText(img, person_id_text, (center_x - 30, center_y - 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
                
                # 显示距离信息
                distance_text = "Distance: N/A"
                color = (0, 0, 255)  # 红色表示无效距离
                
                # 优先使用当前帧的距离数据
                if person.get('valid_distance', False) and person.get('distance') is not None:
                    distance_text = f"Distance: {person['distance']:.3f}m"
                    color = (0, 255, 0)  # 绿色表示有效距离
                # 如果当前帧没有有效数据，尝试使用缓存数据
                elif person['id'] in self.person_distances:
                    cached_data = self.person_distances[person['id']]
                    if time.time() - cached_data['timestamp'] < 2.0:
                        distance_text = f"Distance: {cached_data['distance']:.3f}m"
                        color = (0, 255, 255)  # 黄色表示缓存数据
                
                cv2.putText(img, distance_text, (center_x - 30, center_y - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
                
                # 显示坐标信息
                coord_text = f"({center_x}, {center_y})"
                cv2.putText(img, coord_text, (center_x - 30, center_y + 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
            
            # 添加操作提示
            cv2.putText(img, "Press 'q' to quit", (10, height - 20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
            # 更新显示图像
            with self.display_lock:
                self.display_image = img.copy()
            
            # 转换为ROS消息并发布
            img_msg = self.bridge.cv2_to_imgmsg(img, 'bgr8')
            self.detection_pub.publish(img_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error publishing annotated image: {e}')
    
    def publish_person_positions(self):
        """发布人体位置信息"""
        try:
            # 清理过期的距离查询
            current_time = time.time()
            expired_queries = [
                query_id for query_id, info in self.distance_queries.items()
                if current_time - info['timestamp'] > 2.0
            ]
            for query_id in expired_queries:
                del self.distance_queries[query_id]
                
            # 清理过期的缓存距离数据
            expired_distances = [
                person_id for person_id, info in self.person_distances.items()
                if current_time - info['timestamp'] > 5.0
            ]
            for person_id in expired_distances:
                del self.person_distances[person_id]
            
            # 准备发布的数据
            positions_data = {
                'timestamp': current_time,
                'person_count': len(self.person_positions),
                'persons': self.person_positions
            }
            
            # 发布人体位置信息
            pos_msg = String()
            pos_msg.data = json.dumps(positions_data)
            self.position_pub.publish(pos_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error publishing person positions: {e}')
    
    def display_loop(self):
        """图像显示循环"""
        try:
            cv2.namedWindow('Person Detection', cv2.WINDOW_AUTOSIZE)
            
            while True:
                try:
                    with self.display_lock:
                        if self.display_image is not None:
                            cv2.imshow('Person Detection', self.display_image)
                        else:
                            # 显示等待图像
                            waiting_img = np.zeros((480, 640, 3), dtype=np.uint8)
                            cv2.putText(waiting_img, "Waiting for camera images and detections...", 
                                       (50, 240), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
                            cv2.putText(waiting_img, "Press 'q' to quit", 
                                       (250, 280), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                            cv2.imshow('Person Detection', waiting_img)
                    
                    # 检查按键
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord('q') or key == 27:  # 'q' 或 ESC
                        self.get_logger().info("User pressed 'q', shutting down...")
                        break
                        
                    time.sleep(0.03)  # 约30fps显示
                    
                except Exception as e:
                    self.get_logger().error(f'Error in display loop: {e}')
                    time.sleep(0.1)
                    
        except Exception as e:
            self.get_logger().error(f'Failed to initialize display: {e}')
        finally:
            cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = PersonDetectionDistanceNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()