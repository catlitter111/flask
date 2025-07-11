#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import json
import time
import os
import sys
import threading

# 导入rknn_colour_detect模块的函数
from .rknn_colour_detect import detect_picture, Determine_the_position_of_the_entire_body

class PersonDetectionDistanceNode(Node):
    """
    人体检测和距离查询节点
    通过服装检测确定人体位置，并查询距离信息
    """
    
    def __init__(self):
        super().__init__('person_detection_distance_node')
        
        # 初始化CV bridge
        self.bridge = CvBridge()
        
        # 存储最新的图像
        self.latest_image = None
        
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
        
        # 发布检测结果（包含框选和距离信息）
        self.detection_pub = self.create_publisher(
            Image,
            '/person_detection/annotated_image',
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
        self.timer = self.create_timer(0.033, self.process_image)  # 约30 FPS
        
        # 启动图像显示线程
        self.display_thread = threading.Thread(target=self.display_loop, daemon=True)
        self.display_thread.start()
        
        self.get_logger().info('Person Detection Distance Node initialized')
        self.get_logger().info('Waiting for camera images...')
        self.get_logger().info('Press "q" in the image window to quit')
    
    def image_callback(self, msg):
        """处理接收到的图像"""
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}')
    
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
    
    def process_image(self):
        """处理图像并检测人体"""
        if self.latest_image is None:
            return
            
        try:
            # 计算帧率
            self.fps_frame_count += 1
            current_time = time.time()
            if current_time - self.last_fps_time >= 1.0:  # 每秒更新一次FPS
                self.fps = self.fps_frame_count / (current_time - self.last_fps_time)
                self.fps_frame_count = 0
                self.last_fps_time = current_time
            # 复制图像用于处理
            img = self.latest_image.copy()
            
            # 使用服装检测来检测人体
            pairs = detect_picture(img)
            
            # 存储检测到的人体位置
            self.person_positions = []
            
            # 处理每个检测到的服装组合
            for i, pair in enumerate(pairs):
                if len(pair) >= 2:  # 确保有上衣和下装信息
                    upper_clothes = pair[0] if len(pair[0]) > 1 else []
                    lower_clothes = pair[1] if len(pair[1]) > 1 else []
                    
                    # 确定整个身体的位置
                    body_positions = Determine_the_position_of_the_entire_body(
                        upper_clothes, lower_clothes, img
                    )
                    
                    # 处理每个检测到的身体位置
                    for j, body_pos in enumerate(body_positions):
                        if len(body_pos) == 4:  # 确保有完整的坐标信息
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
            
            # 发布人体位置信息
            self.publish_person_positions()
            
            # 发布带标注的图像（延后，让距离数据有时间更新）
            self.publish_annotated_image(img)
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')
    
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
                
                # 显示距离信息 - 改进逻辑，优先使用有效的距离数据
                distance_text = "Distance: N/A"
                color = (0, 0, 255)  # 红色表示无效距离
                
                # 优先使用当前帧的距离数据
                if person.get('valid_distance', False) and person.get('distance') is not None:
                    distance_text = f"Distance: {person['distance']:.3f}m"
                    color = (0, 255, 0)  # 绿色表示有效距离
                # 如果当前帧没有有效数据，尝试使用缓存数据
                elif person['id'] in self.person_distances:
                    cached_data = self.person_distances[person['id']]
                    if time.time() - cached_data['timestamp'] < 2.0:  # 2秒内的缓存数据
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
                if current_time - info['timestamp'] > 2.0  # 2秒超时
            ]
            for query_id in expired_queries:
                del self.distance_queries[query_id]
                
            # 清理过期的缓存距离数据
            expired_distances = [
                person_id for person_id, info in self.person_distances.items()
                if current_time - info['timestamp'] > 5.0  # 5秒超时
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
            
            # 日志输出
            if self.person_positions:
                self.get_logger().info(f'Detected {len(self.person_positions)} person(s)')
                for person in self.person_positions:
                    if person['valid_distance'] and person['distance'] is not None:
                        self.get_logger().info(
                            f"{person['id']}: center=({person['center'][0]}, {person['center'][1]}), "
                            f"distance={person['distance']:.2f}m"
                        )
            
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
                            cv2.putText(waiting_img, "Waiting for camera images...", 
                                       (150, 240), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
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