#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
YOLOv8姿态检测模块 - ROS2适配版本
================================
移植自yolov8_pose.py，适配ROS2 Humble环境
提供人体姿态检测功能，支持关键点检测和骨架绘制

作者: AI Assistant
移植版本: v1.0.0
"""

import os
import sys
import time
import numpy as np
import cv2
import logging
from typing import List, Tuple, Optional, Union
from dataclasses import dataclass

# 导入RKNN Lite
try:
    from rknnlite.api import RKNNLite
    RKNN_AVAILABLE = True
except ImportError:
    RKNN_AVAILABLE = False
    print("⚠️ RKNN Lite未安装，姿态检测功能不可用")

# 配置日志
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# 全局配置
CLASSES = ['person']
NMS_THRESH = 0.4
OBJECT_THRESH = 0.5

# 姿态颜色配置
POSE_PALETTE = np.array([
    [255, 128, 0], [255, 153, 51], [255, 178, 102], [230, 230, 0], [255, 153, 255],
    [153, 204, 255], [255, 102, 255], [255, 51, 255], [102, 178, 255], [51, 153, 255],
    [255, 153, 153], [255, 102, 102], [255, 51, 51], [153, 255, 153], [102, 255, 102],
    [51, 255, 51], [0, 255, 0], [0, 0, 255], [255, 0, 0], [255, 255, 255]
], dtype=np.uint8)

KPT_COLOR = POSE_PALETTE[[16, 16, 16, 16, 16, 0, 0, 0, 0, 0, 0, 9, 9, 9, 9, 9, 9]]

SKELETON = [
    [16, 14], [14, 12], [17, 15], [15, 13], [12, 13], [6, 12], [7, 13], [6, 7], [6, 8], 
    [7, 9], [8, 10], [9, 11], [2, 3], [1, 2], [1, 3], [2, 4], [3, 5], [4, 6], [5, 7]
]

LIMB_COLOR = POSE_PALETTE[[9, 9, 9, 9, 7, 7, 7, 0, 0, 0, 0, 0, 16, 16, 16, 16, 16, 16, 16]]


@dataclass
class PoseResult:
    """姿态检测结果数据类"""
    bbox: Tuple[int, int, int, int]  # (xmin, ymin, xmax, ymax)
    score: float
    keypoints: np.ndarray  # shape: (17, 3) - [x, y, confidence]
    class_id: int = 0


class YOLOv8PoseDetector:
    """YOLOv8姿态检测器"""
    
    def __init__(self, model_path: Optional[str] = None):
        """初始化姿态检测器"""
        self.rknn = None
        self.model_loaded = False
        self.model_path = model_path
        
        if not RKNN_AVAILABLE:
            logger.error("RKNN Lite不可用，无法初始化姿态检测器")
            return
        
        # 自动查找模型文件
        if self.model_path is None:
            self.model_path = self._find_model_file()
        
        if self.model_path and os.path.exists(self.model_path):
            self._load_model()
        else:
            logger.error(f"未找到模型文件: {self.model_path}")
    
    def _find_model_file(self) -> Optional[str]:
        """查找YOLOv8姿态检测模型文件"""
        possible_paths = [
            # ROS2安装路径
            os.path.join(os.path.dirname(__file__), '..', '..', '..', 'install', 'following_robot', 'share', 'following_robot', 'data', 'yolov8_pose.rknn'),
            # 开发环境路径
            os.path.join(os.path.dirname(__file__), '..', 'data', 'yolov8_pose.rknn'),
            # 绝对路径
            '/userdata/try_again/SelfFollowingROS2/src/following_robot/data/yolov8_pose.rknn',
        ]
        
        for path in possible_paths:
            abs_path = os.path.abspath(path)
            logger.info(f"检查模型路径: {abs_path}")
            if os.path.exists(abs_path):
                logger.info(f"找到模型文件: {abs_path}")
                return abs_path
        
        logger.error("未找到YOLOv8姿态检测模型文件")
        return None
    
    def _load_model(self):
        """加载RKNN模型"""
        try:
            logger.info(f"正在加载YOLOv8姿态检测模型: {self.model_path}")
            
            self.rknn = RKNNLite(verbose=False)
            
            # 加载模型
            ret = self.rknn.load_rknn(self.model_path)
            if ret != 0:
                logger.error(f"加载RKNN模型失败: {self.model_path}")
                return
            
            # 初始化运行时环境
            logger.info("初始化运行时环境...")
            ret = self.rknn.init_runtime()
            if ret != 0:
                logger.error("初始化运行时环境失败")
                return
            
            self.model_loaded = True
            logger.info("✅ YOLOv8姿态检测模型加载成功")
            
        except Exception as e:
            logger.error(f"模型加载异常: {e}")
            self.model_loaded = False
    
    def letterbox_resize(self, image: np.ndarray, size: Tuple[int, int], bg_color: int = 56) -> Tuple[np.ndarray, float, int, int]:
        """按比例缩放图像并填充"""
        target_width, target_height = size
        image_height, image_width, _ = image.shape
        
        # 计算缩放比例
        aspect_ratio = min(target_width / image_width, target_height / image_height)
        new_width = int(image_width * aspect_ratio)
        new_height = int(image_height * aspect_ratio)
        
        # 按比例缩放
        image = cv2.resize(image, (new_width, new_height), interpolation=cv2.INTER_AREA)
        
        # 创建画布并填充
        result_image = np.ones((target_height, target_width, 3), dtype=np.uint8) * bg_color
        offset_x = (target_width - new_width) // 2
        offset_y = (target_height - new_height) // 2
        result_image[offset_y:offset_y + new_height, offset_x:offset_x + new_width] = image
        
        return result_image, aspect_ratio, offset_x, offset_y
    
    def sigmoid(self, x: np.ndarray) -> np.ndarray:
        """Sigmoid激活函数"""
        return 1 / (1 + np.exp(-np.clip(x, -250, 250)))
    
    def softmax(self, x: np.ndarray, axis: int = -1) -> np.ndarray:
        """Softmax函数"""
        exp_x = np.exp(x - np.max(x, axis=axis, keepdims=True))
        return exp_x / np.sum(exp_x, axis=axis, keepdims=True)
    
    def detect_pose(self, image: np.ndarray) -> List[PoseResult]:
        """检测人体姿态"""
        if not self.model_loaded or self.rknn is None:
            logger.warning("模型未加载，无法进行姿态检测")
            return []
        
        try:
            # 图像预处理
            letterbox_img, aspect_ratio, offset_x, offset_y = self.letterbox_resize(image, (640, 640), 56)
            infer_img = letterbox_img[..., ::-1]  # BGR2RGB
            infer_img = np.expand_dims(infer_img, axis=0)  # 添加批次维度
            
            # 模型推理
            results = self.rknn.inference(inputs=[infer_img])
            
            if results is None or len(results) < 4:
                logger.warning("模型推理结果异常")
                return []
            
            # 处理输出
            all_detections = []
            keypoints_data = results[3]
            
            for i, output in enumerate(results[:3]):
                if output is None:
                    continue
                    
                # 确定stride和index
                if output.shape[2] == 20:
                    stride = 32
                    index = 20 * 4 * 20 * 4 + 20 * 2 * 20 * 2
                elif output.shape[2] == 40:
                    stride = 16
                    index = 20 * 4 * 20 * 4
                elif output.shape[2] == 80:
                    stride = 8
                    index = 0
                else:
                    continue
                
                feature = output.reshape(1, 65, -1)
                detections = self._process_output(feature, keypoints_data, index, 
                                               output.shape[3], output.shape[2], stride)
                all_detections.extend(detections)
            
            # NMS
            final_detections = self._nms(all_detections)
            
            # 转换为PoseResult格式
            pose_results = []
            for det in final_detections:
                # 坐标转换回原图
                xmin = int((det['xmin'] - offset_x) / aspect_ratio)
                ymin = int((det['ymin'] - offset_y) / aspect_ratio)
                xmax = int((det['xmax'] - offset_x) / aspect_ratio)
                ymax = int((det['ymax'] - offset_y) / aspect_ratio)
                
                # 关键点坐标转换
                keypoints_converted = det['keypoint'].reshape(-1, 3).copy()
                keypoints_converted[:, 0] = (keypoints_converted[:, 0] - offset_x) / aspect_ratio
                keypoints_converted[:, 1] = (keypoints_converted[:, 1] - offset_y) / aspect_ratio
                
                pose_result = PoseResult(
                    bbox=(xmin, ymin, xmax, ymax),
                    score=det['score'],
                    keypoints=keypoints_converted,
                    class_id=det['class_id']
                )
                pose_results.append(pose_result)
            
            return pose_results
            
        except Exception as e:
            logger.error(f"姿态检测异常: {e}")
            import traceback
            traceback.print_exc()
            return []
    
    def _process_output(self, output: np.ndarray, keypoints: np.ndarray, index: int, 
                      model_w: int, model_h: int, stride: int) -> List[dict]:
        """处理模型输出"""
        detections = []
        
        xywh = output[:, :64, :]
        conf = self.sigmoid(output[:, 64:, :])
        
        for h in range(model_h):
            for w in range(model_w):
                for c in range(len(CLASSES)):
                    confidence = conf[0, c, (h * model_w) + w]
                    if confidence > OBJECT_THRESH:
                        # 处理边界框
                        xywh_ = xywh[0, :, (h * model_w) + w].reshape(1, 4, 16, 1)
                        data = np.array([i for i in range(16)]).reshape(1, 1, 16, 1)
                        xywh_ = self.softmax(xywh_, 2)
                        xywh_ = np.multiply(data, xywh_)
                        xywh_ = np.sum(xywh_, axis=2, keepdims=True).reshape(-1)
                        
                        # 转换坐标
                        xywh_temp = xywh_.copy()
                        xywh_temp[0] = (w + 0.5) - xywh_[0]
                        xywh_temp[1] = (h + 0.5) - xywh_[1]
                        xywh_temp[2] = (w + 0.5) + xywh_[2]
                        xywh_temp[3] = (h + 0.5) + xywh_[3]
                        
                        xywh_[0] = ((xywh_temp[0] + xywh_temp[2]) / 2)
                        xywh_[1] = ((xywh_temp[1] + xywh_temp[3]) / 2)
                        xywh_[2] = (xywh_temp[2] - xywh_temp[0])
                        xywh_[3] = (xywh_temp[3] - xywh_temp[1])
                        xywh_ = xywh_ * stride
                        
                        xmin = xywh_[0] - xywh_[2] / 2
                        ymin = xywh_[1] - xywh_[3] / 2
                        xmax = xywh_[0] + xywh_[2] / 2
                        ymax = xywh_[1] + xywh_[3] / 2
                        
                        # 提取关键点
                        keypoint = keypoints[..., (h * model_w) + w + index]
                        keypoint[..., 0:2] = keypoint[..., 0:2] // 1
                        
                        detection = {
                            'class_id': c,
                            'score': confidence,
                            'xmin': xmin,
                            'ymin': ymin,
                            'xmax': xmax,
                            'ymax': ymax,
                            'keypoint': keypoint
                        }
                        detections.append(detection)
        
        return detections
    
    def _iou(self, box1: dict, box2: dict) -> float:
        """计算两个框的IoU"""
        xmin = max(box1['xmin'], box2['xmin'])
        ymin = max(box1['ymin'], box2['ymin'])
        xmax = min(box1['xmax'], box2['xmax'])
        ymax = min(box1['ymax'], box2['ymax'])
        
        inner_width = max(0, xmax - xmin)
        inner_height = max(0, ymax - ymin)
        inner_area = inner_width * inner_height
        
        area1 = (box1['xmax'] - box1['xmin']) * (box1['ymax'] - box1['ymin'])
        area2 = (box2['xmax'] - box2['xmin']) * (box2['ymax'] - box2['ymin'])
        
        total = area1 + area2 - inner_area
        
        return inner_area / total if total > 0 else 0
    
    def _nms(self, detections: List[dict]) -> List[dict]:
        """非极大值抑制"""
        if not detections:
            return []
        
        # 按置信度排序
        sorted_detections = sorted(detections, key=lambda x: x['score'], reverse=True)
        result = []
        
        for i, det in enumerate(sorted_detections):
            if det['class_id'] == -1:  # 已被抑制
                continue
                
            result.append(det)
            
            # 抑制重叠的检测框
            for j in range(i + 1, len(sorted_detections)):
                if sorted_detections[j]['class_id'] == det['class_id']:
                    iou_val = self._iou(det, sorted_detections[j])
                    if iou_val > NMS_THRESH:
                        sorted_detections[j]['class_id'] = -1
        
        return result
    
    def draw_pose(self, image: np.ndarray, pose_results: List[PoseResult]) -> np.ndarray:
        """在图像上绘制姿态检测结果"""
        result_image = image.copy()
        
        for i, pose in enumerate(pose_results):
            xmin, ymin, xmax, ymax = pose.bbox
            
            # 绘制边界框
            cv2.rectangle(result_image, (xmin, ymin), (xmax, ymax), (0, 255, 0), 2)
            
            # 绘制标签
            label = f"Person {i+1}: {pose.score:.2f}"
            cv2.putText(result_image, label, (xmin, ymin - 10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
            # 绘制关键点
            keypoints = pose.keypoints
            for k, (x, y, conf) in enumerate(keypoints):
                if x > 0 and y > 0 and conf > 0.5:  # 只绘制置信度高的关键点
                    color = [int(c) for c in KPT_COLOR[k]]
                    cv2.circle(result_image, (int(x), int(y)), 5, color, -1, lineType=cv2.LINE_AA)
            
            # 绘制骨架
            for k, (start_idx, end_idx) in enumerate(SKELETON):
                start_point = keypoints[start_idx - 1]  # 索引从1开始
                end_point = keypoints[end_idx - 1]
                
                if (start_point[0] > 0 and start_point[1] > 0 and start_point[2] > 0.5 and
                    end_point[0] > 0 and end_point[1] > 0 and end_point[2] > 0.5):
                    
                    color = [int(c) for c in LIMB_COLOR[k]]
                    cv2.line(result_image, 
                            (int(start_point[0]), int(start_point[1])),
                            (int(end_point[0]), int(end_point[1])),
                            color, thickness=2, lineType=cv2.LINE_AA)
        
        return result_image


# 全局检测器实例
_global_pose_detector = None

def get_pose_detector() -> YOLOv8PoseDetector:
    """获取全局姿态检测器实例"""
    global _global_pose_detector
    if _global_pose_detector is None:
        _global_pose_detector = YOLOv8PoseDetector()
    return _global_pose_detector

def detect_human_pose(image: np.ndarray) -> List[PoseResult]:
    """检测人体姿态（便捷函数）"""
    detector = get_pose_detector()
    return detector.detect_pose(image)

def draw_human_pose(image: np.ndarray, pose_results: List[PoseResult]) -> np.ndarray:
    """绘制人体姿态（便捷函数）"""
    detector = get_pose_detector()
    return detector.draw_pose(image, pose_results)


if __name__ == '__main__':
    # 测试代码
    print("🧪 测试YOLOv8姿态检测模块...")
    
    detector = YOLOv8PoseDetector()
    if detector.model_loaded:
        print("✅ 模型加载成功")
        
        # 创建测试图像
        test_image = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.rectangle(test_image, (200, 100), (400, 400), (255, 255, 255), -1)
        
        # 进行检测
        results = detector.detect_pose(test_image)
        print(f"检测到 {len(results)} 个人体姿态")
        
        if results:
            # 绘制结果
            result_image = detector.draw_pose(test_image, results)
            cv2.imwrite("/tmp/pose_detection_test.jpg", result_image)
            print("测试结果已保存到 /tmp/pose_detection_test.jpg")
    else:
        print("❌ 模型加载失败") 