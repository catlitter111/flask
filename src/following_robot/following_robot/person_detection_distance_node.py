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
        
        # 图像显示相关
        self.display_image = None
        self.display_lock = threading.Lock()
        
        # 定时器处理图像
        self.timer = self.create_timer(0.1, self.process_image)  # 10 FPS
        
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
            
            if query_id in self.distance_queries:
                # 更新距离信息
                self.distance_queries[query_id]['distance'] = result.get('depth_m', None)
                self.distance_queries[query_id]['valid'] = result.get('valid', False)
                
                # 在图像上标注距离信息
                self.annotate_distance_on_image(query_id, result)
        except Exception as e:
            self.get_logger().error(f'Error processing depth result: {e}')
    
    def process_image(self):
        """处理图像并检测人体"""
        if self.latest_image is None:
            return
            
        try:
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
                            self.person_positions.append(person_info)
                            
                            # 查询距离信息
                            self.query_distance(center_x, center_y, person_id)
            
            # 发布带标注的图像
            self.publish_annotated_image(img)
            
            # 发布人体位置信息
            self.publish_person_positions()
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')
    
    def query_distance(self, x, y, person_id):
        """查询指定像素点的距离信息"""
        try:
            query_id = f"{x}_{y}"
            
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
    
    def annotate_distance_on_image(self, query_id, result):
        """在图像上标注距离信息"""
        try:
            if query_id in self.distance_queries:
                query_info = self.distance_queries[query_id]
                person_id = query_info['person_id']
                
                # 更新人体位置信息中的距离
                for person in self.person_positions:
                    if person['id'] == person_id:
                        person['distance'] = result.get('depth_m', None)
                        person['valid_distance'] = result.get('valid', False)
                        break
                
        except Exception as e:
            self.get_logger().error(f'Error annotating distance: {e}')
    
    def publish_annotated_image(self, img):
        """发布带标注的图像"""
        try:
            # 在图像上添加距离信息
            for person in self.person_positions:
                if person['valid_distance'] and person['distance'] is not None:
                    center_x, center_y = person['center']
                    distance_text = f"{person['distance']:.2f}m"
                    
                    # 在中心点附近显示距离
                    cv2.putText(img, distance_text, (center_x + 10, center_y),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
            
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
                            # 添加状态信息
                            display_img = self.display_image.copy()
                            
                            # 在图像上添加帧率和状态信息
                            cv2.putText(display_img, f"Persons: {len(self.person_positions)}", 
                                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                            
                            # 添加说明文字
                            cv2.putText(display_img, "Press 'q' to quit", 
                                       (10, display_img.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                            
                            cv2.imshow('Person Detection', display_img)
                        else:
                            # 显示等待图像
                            waiting_img = np.zeros((480, 640, 3), dtype=np.uint8)
                            cv2.putText(waiting_img, "Waiting for camera images...", 
                                       (50, 240), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
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