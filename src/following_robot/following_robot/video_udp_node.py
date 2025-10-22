#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS2 视频TCP传输节点
===================
订阅 /camera/color/image_raw 话题，通过TCP发送视频流
支持边界框监听和显示功能

功能特性：
- 订阅ROS2图像话题
- TCP视频流传输（可靠传输）
- 边界框UDP监听和过滤
- 实时显示跟踪结果
- 线程安全的边界框缓冲

作者: AI Assistant
版本: v1.0.0
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import cv2
import socket
import pickle
import struct
import time
import json
import threading
import queue
import numpy as np

# ROS2消息类型
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge

class TCPBidirectionalHandler:
    """TCP双向通信处理器 - 用于发送视频帧和接收边界框数据"""
    
    def __init__(self, connect_host: str, port: int, send_buf_size: int = 4 * 1024 * 1024, timeout: float = 2.0):
        """
        初始化TCP双向通信处理器
        
        参数:
            connect_host: 连接的目标主机地址
            port: 端口号
            send_buf_size: 发送缓冲区大小
            timeout: 超时时间
        """
        self.connect_host = connect_host
        self.port = port
        self.send_buf_size = send_buf_size
        self.timeout = timeout
        self.client_socket = None
        self.connected = False
        self.lock = threading.Lock()
        
        # 连接到服务器
        self._connect_to_server()
    
    def _connect_to_server(self):
        """连接到TCP服务器"""
        try:
            self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.client_socket.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, self.send_buf_size)
            self.client_socket.settimeout(self.timeout)
            self.client_socket.connect((self.connect_host, self.port))
            self.connected = True
            print(f"TCP双向通信已连接到 {self.connect_host}:{self.port}")
        except Exception as e:
            print(f"TCP连接失败: {e}")
            self.connected = False
            if self.client_socket:
                self.client_socket.close()
                self.client_socket = None
    
    def _recv_exact(self, sock, size):
        """确保接收到指定长度的数据"""
        data = b''
        while len(data) < size:
            try:
                chunk = sock.recv(size - len(data))
                if not chunk:
                    return None
                data += chunk
            except socket.timeout:
                return None
            except Exception:
                return None
        return data
    
    def send_frame(self, frame_data):
        """发送视频帧数据（消息类型1）"""
        if not self.connected or not self.client_socket:
            return False
        
        try:
            with self.lock:
                # 消息格式: !BI (消息类型1字节 + 数据长度4字节)
                message_type = 1  # 视频帧
                data_length = len(frame_data)
                header = struct.pack('!BI', message_type, data_length)
                
                # 发送头部和数据
                self.client_socket.sendall(header)
                self.client_socket.sendall(frame_data)
                return True
        except Exception as e:
            print(f"发送视频帧失败: {e}")
            self._reconnect()
            return False
    
    def receive_bbox(self):
        """接收边界框数据（消息类型2）"""
        if not self.connected or not self.client_socket:
            return None
        
        try:
            # 接收头部 (1字节消息类型 + 4字节数据长度)
            header = self._recv_exact(self.client_socket, 5)
            if not header:
                return None
            
            message_type, data_length = struct.unpack('!BI', header)
            
            # 检查消息类型
            if message_type != 2:  # 边界框数据
                print(f"收到未知消息类型: {message_type}")
                return None
            
            # 接收边界框数据
            bbox_data = self._recv_exact(self.client_socket, data_length)
            # print(bbox_data)
            if not bbox_data:
                return None
            
            # 解析JSON数据
            bbox_json = json.loads(bbox_data.decode('utf-8'))
            return bbox_json
            
        except socket.timeout:
            return None
        except Exception as e:
            print(f"接收边界框数据失败: {e}")
            self._reconnect()
            return None
    
    def _reconnect(self):
        """重新连接"""
        self.connected = False
        if self.client_socket:
            try:
                self.client_socket.close()
            except:
                pass
            self.client_socket = None
        
        # 尝试重新连接
        self._connect_to_server()
    
    def close(self):
        """关闭连接"""
        self.connected = False
        if self.client_socket:
            try:
                self.client_socket.close()
            except:
                pass
            self.client_socket = None
    
    def __del__(self):
        """析构函数"""
        self.close()

class VideoTCPNode(Node):
    """ROS2视频TCP传输节点"""
    
    def __init__(self):
        super().__init__('video_tcp_node')
        
        # 初始化参数
        self.setup_parameters()
        
        # 初始化组件
        self.bridge = CvBridge()
        self.latest_frame = None
        self.frame_lock = threading.Lock()
        
        # 单次使用的bbox缓冲（线程安全）
        self.latest_bbox_lock = threading.Lock()
        self.latest_bbox = {'bbox': None, 'raw': None, 'used': True}
        
        # 初始化双向TCP通信处理器
        self.tcp_handler = TCPBidirectionalHandler(self.tcp_host, self.tcp_port)
        
        # 设置订阅器
        self.setup_subscribers()
        
        # 设置发布器
        self.setup_publishers()
        
        # 启动线程
        self.start_threads()
        
        self.get_logger().info("🚀 视频TCP传输节点已启动")
        self.get_logger().info(f"📡 TCP双向通信连接到: {self.tcp_host}:{self.tcp_port}")
        
        # 统计变量
        self.frame_count = 0
        self.start_time = time.time()
    
    def setup_parameters(self):
        """设置ROS2参数"""
        # TCP双向通信参数
        self.declare_parameter('tcp_host', '192.168.137.1')
        self.declare_parameter('tcp_port', 5005)
        
        # 过滤参数
        self.declare_parameter('min_conf', 0.3)
        self.declare_parameter('min_size', 10)
        self.declare_parameter('ignore_tid0', False)
        self.declare_parameter('drop_zero_box', True)
        
        # 显示参数
        self.declare_parameter('enable_display', True)
        self.declare_parameter('jpeg_quality', 80)
        self.declare_parameter('max_packet_size', 60000)
        
        # 获取参数值
        self.tcp_host = self.get_parameter('tcp_host').get_parameter_value().string_value
        self.tcp_port = self.get_parameter('tcp_port').get_parameter_value().integer_value
        self.min_conf = self.get_parameter('min_conf').get_parameter_value().double_value
        self.min_size = self.get_parameter('min_size').get_parameter_value().integer_value
        self.ignore_tid0 = self.get_parameter('ignore_tid0').get_parameter_value().bool_value
        self.drop_zero_box = self.get_parameter('drop_zero_box').get_parameter_value().bool_value
        self.enable_display = self.get_parameter('enable_display').get_parameter_value().bool_value
        self.jpeg_quality = self.get_parameter('jpeg_quality').get_parameter_value().integer_value
        self.max_packet_size = self.get_parameter('max_packet_size').get_parameter_value().integer_value
    

    
    def setup_subscribers(self):
        """设置ROS2订阅器"""
        # 图像订阅器 - 使用可靠的QoS配置
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.image_subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            qos_profile
        )
    
    def setup_publishers(self):
        """设置ROS2发布器"""
        # 目标点发布器
        self.target_point_publisher = self.create_publisher(
            PointStamped,
            '/target_point',
            10
        )
    
    def start_threads(self):
        """启动后台线程"""
        # 边界框接收线程
        self.bbox_thread = threading.Thread(
            target=self.bbox_receiver_worker,
            daemon=True
        )
        self.bbox_thread.start()
        
        # 显示线程
        if self.enable_display:
            self.display_queue = queue.Queue(maxsize=1)
            self.display_thread = threading.Thread(
                target=self.display_worker,
                daemon=True
            )
            self.display_thread.start()
        else:
            self.display_queue = None
    
    def image_callback(self, msg):
        """ROS2图像回调函数"""
        try:
            # 转换ROS图像消息为OpenCV格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # 更新最新帧
            with self.frame_lock:
                self.latest_frame = cv_image.copy()
            
            # 推送到显示队列
            if self.display_queue is not None:
                try:
                    if self.display_queue.full():
                        _ = self.display_queue.get_nowait()
                    self.display_queue.put_nowait(cv_image.copy())
                except Exception:
                    pass
            
            # 发送TCP视频流
            self.send_frame_tcp(cv_image)
            
        except Exception as e:
            self.get_logger().error(f"图像处理失败: {e}")
    
    def send_frame_tcp(self, frame):
        """通过TCP发送视频帧"""
        try:
            # 压缩帧
            encode_param = [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality]
            result, encoded_frame = cv2.imencode('.jpg', frame, encode_param)
            
            if not result:
                return
            
            # 序列化数据
            data = pickle.dumps(encoded_frame)
            
            # 通过双向通信处理器发送
            success = self.tcp_handler.send_frame(data)
            
            if success:
                # 计算FPS
                self.frame_count += 1
                if self.frame_count % 1000 == 0:
                    elapsed = time.time() - self.start_time
                    current_fps = self.frame_count / elapsed
                    self.get_logger().info(f"发送FPS: {current_fps:.1f}, 帧大小: {len(data)/1024:.1f}KB")
            else:
                self.get_logger().warning("视频帧发送失败")
                
        except Exception as e:
            self.get_logger().error(f"TCP发送失败: {e}")
    
    def bbox_receiver_worker(self):
        """边界框接收线程"""
        try:
            self.get_logger().info(
                f"✓ TCP边界框接收启动 "
                f"(min_conf={self.min_conf}, min_size={self.min_size}, "
                f"ignore_tid0={self.ignore_tid0}, drop_zero_box={self.drop_zero_box})"
            )
            
            while rclpy.ok():
                try:
                    # 通过双向通信处理器接收边界框数据
                    payload = self.tcp_handler.receive_bbox()
                    # print(f"payload:{payload}")
                    if payload is None:
                        time.sleep(0.01)  # 短暂等待
                        continue
                    
                    # 解析边界框数据 - 从嵌套的bounding_box对象中提取
                    bbox_data = payload.get('bounding_box', {})
                    x = int(bbox_data.get('x', 0))
                    y = int(bbox_data.get('y', 0))
                    width = int(bbox_data.get('w', 0))  # 注意：字段名是'w'不是'width'
                    height = int(bbox_data.get('h', 0))  # 注意：字段名是'h'不是'height'
                    conf = float(bbox_data.get('confidence', 0.0))
                    person_id = payload.get('track_id', None)  # 注意：字段名是'track_id'不是'person_id'
                    
                    # 调试信息：显示解析后的数据
                    # self.get_logger().info(
                    #     f"解析边界框: x={x}, y={y}, w={width}, h={height}, "
                    #     f"conf={conf:.3f}, track_id={person_id}"
                    # )
                    
                    # 过滤条件
                    try:
                        tid_val = int(person_id) if person_id is not None else None
                    except Exception:
                        tid_val = None
                    
                    # 应用过滤规则
                    if self.drop_zero_box and x == 0 and y == 0 and width == 0 and height == 0:
                        self.get_logger().debug("↳ 过滤: 全零bbox")
                        continue
                    
                    if width <= 0 or height <= 0:
                        self.get_logger().debug(f"↳ 过滤: 非法尺寸 w={width}, h={height}")
                        continue
                    
                    if min(width, height) < self.min_size:
                        self.get_logger().debug(f"↳ 过滤: 尺寸过小 min({width},{height})<{self.min_size}")
                        continue
                    
                    if conf < self.min_conf:
                        self.get_logger().debug(f"↳ 过滤: 置信度过低 conf={conf:.3f} < {self.min_conf}")
                        continue
                    
                    if self.ignore_tid0 and (tid_val is None or tid_val <= 0):
                        self.get_logger().debug(f"↳ 过滤: person_id={person_id}")
                        continue
                    
                    # 转换为标准格式
                    xmin, ymin = x, y
                    xmax, ymax = x + width, y + height
                    converted = {
                        'xmin': xmin,
                        'ymin': ymin,
                        'xmax': xmax,
                        'ymax': ymax,
                        'confidence': conf,
                    }
                    
                    # 存储到缓冲区
                    self.store_bbox(converted, payload)
                    
                    # 打印信息
                    self.get_logger().info(
                        f"收到bbox: (xmin={xmin}, ymin={ymin}, xmax={xmax}, ymax={ymax}, "
                        f"conf={conf:.3f}) person_id={person_id}"
                    )
                    
                except Exception as e:
                    self.get_logger().error(f"解析bbox数据失败: {e}")
                    time.sleep(0.1)  # 错误时等待更长时间
                    
        except Exception as e:
            self.get_logger().error(f"边界框接收失败: {e}")
    
    def store_bbox(self, bbox, raw):
        """存储边界框到缓冲区"""
        try:
            with self.latest_bbox_lock:
                self.latest_bbox['bbox'] = bbox
                self.latest_bbox['raw'] = raw
                self.latest_bbox['used'] = False
        except Exception as e:
            self.get_logger().error(f"存储bbox失败: {e}")
    
    def display_worker(self):
        """显示线程：每帧消费一次bbox并叠加绿色框"""
        try:
            cv2.namedWindow('Tracking Result', cv2.WINDOW_NORMAL)
            self.get_logger().info("✓ 显示窗口已创建")
        except Exception as e:
            self.get_logger().error(f"创建显示窗口失败: {e}")
            return
        
        while rclpy.ok():
            try:
                frame = self.display_queue.get(timeout=1.0)
            except Exception:
                # 没有新帧，继续等待
                continue
            
            h, w = frame.shape[:2]
            bb = None
            raw = None
            
            # 获取最新的bbox
            with self.latest_bbox_lock:
                if self.latest_bbox['bbox'] is not None and not self.latest_bbox['used']:
                    bb = self.latest_bbox['bbox']
                    raw = self.latest_bbox['raw']
                    self.latest_bbox['used'] = True
                    self.latest_bbox['bbox'] = None
                    self.latest_bbox['raw'] = None
            
            # 绘制边界框
            if bb is not None:
                # 坐标裁剪
                xmin = max(0, min(w-1, int(bb['xmin'])))
                ymin = max(0, min(h-1, int(bb['ymin'])))
                xmax = max(0, min(w-1, int(bb['xmax'])))
                ymax = max(0, min(h-1, int(bb['ymax'])))
                
                # 计算边界框中心点
                center_x = (xmin + xmax) / 2.0
                center_y = (ymin + ymax) / 2.0
                
                # 发布中心点坐标
                try:
                    point_msg = PointStamped()
                    point_msg.header.stamp = self.get_clock().now().to_msg()
                    point_msg.header.frame_id = "camera_color_optical_frame"
                    point_msg.point.x = center_x
                    point_msg.point.y = center_y
                    point_msg.point.z = 0.0
                    
                    self.target_point_publisher.publish(point_msg)
                    self.get_logger().debug(f"发布目标点: ({center_x:.1f}, {center_y:.1f})")
                except Exception as e:
                    self.get_logger().error(f"发布目标点失败: {e}")
                
                color = (0, 255, 0)  # 绿色
                thickness = 2
                cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), color, thickness)
                
                # 绘制中心点
                center_point = (int(center_x), int(center_y))
                cv2.circle(frame, center_point, 5, (0, 0, 255), -1)  # 红色圆点
                
                try:
                    tid = raw.get('track_id', '?') if raw else '?'
                    conf = float(bb.get('confidence', 0.0))
                    label = f"id:{tid} conf:{conf:.2f}"
                    cv2.putText(frame, label, (xmin, max(0, ymin-5)), 
                              cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
                    
                    # 显示中心点坐标
                    center_label = f"({center_x:.1f}, {center_y:.1f})"
                    cv2.putText(frame, center_label, (int(center_x)-50, int(center_y)+20), 
                              cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
                except Exception:
                    pass
            
            try:
                cv2.imshow('Tracking Result', frame)
                if (cv2.waitKey(1) & 0xFF) == ord('q'):
                    self.get_logger().info("用户按下 'q' 键，退出显示")
                    break
            except Exception as e:
                self.get_logger().error(f"显示帧失败: {e}")
                break
        
        try:
            cv2.destroyWindow('Tracking Result')
        except Exception:
            pass
    
    def destroy_node(self):
        """节点销毁时的清理工作"""
        try:
            if hasattr(self, 'tcp_socket'):
                self.tcp_socket.close()
            if hasattr(self, 'tcp_bbox_socket'):
                self.tcp_bbox_socket.close()
            self.get_logger().info("✓ TCP socket已关闭")
        except Exception as e:
            self.get_logger().error(f"清理资源失败: {e}")
        
        super().destroy_node()


def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    try:
        node = VideoTCPNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n用户中断")
    except Exception as e:
        print(f"节点运行失败: {e}")
    finally:
        try:
            node.destroy_node()
        except:
            pass
        rclpy.shutdown()


if __name__ == '__main__':
    main()