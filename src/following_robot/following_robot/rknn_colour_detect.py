import cv2 as cv
from rknnlite.api import RKNNLite
# 修复sklearn导入问题，添加条件导入
try:
    from sklearn.cluster import KMeans
    SKLEARN_AVAILABLE = True
except ImportError:
    SKLEARN_AVAILABLE = False
    print("警告: sklearn未安装，将使用OpenCV的聚类算法作为替代")

from collections import Counter
import numpy as np
import os
import logging
from pathlib import Path
import json
import traceback
import time, sys

# 配置日志系统
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


# 性能监控工具
class PerformanceMonitor:
    """用于记录和分析性能数据的简单工具"""

    def __init__(self, window_size=30):
        self.timings = {}
        self.window_size = window_size
        self.stats = {}

    def start(self, name):
        """开始记录某个操作的时间"""
        if name not in self.timings:
            self.timings[name] = []
        self.timings[name].append({"start": time.time(), "end": None})

    def end(self, name):
        """结束记录某个操作的时间"""
        if name in self.timings and self.timings[name] and self.timings[name][-1]["end"] is None:
            self.timings[name][-1]["end"] = time.time()
            # 保持数据点数量在窗口大小内
            if len(self.timings[name]) > self.window_size:
                self.timings[name].pop(0)

    def get_stats(self, name):
        """获取某个操作的统计信息"""
        if name not in self.timings:
            return None

        durations = [t["end"] - t["start"] for t in self.timings[name] if t["end"] is not None]
        if not durations:
            return None

        stats = {
            "min": min(durations) * 1000,  # 转换为毫秒
            "max": max(durations) * 1000,
            "avg": sum(durations) * 1000 / len(durations),
            "samples": len(durations)
        }

        return stats

    def log_stats(self):
        """将所有操作的统计信息记录到日志"""
        for name in self.timings:
            stats = self.get_stats(name)
            if stats:
                logger.info(
                    f"性能统计 - {name}: 最小={stats['min']:.2f}ms, 最大={stats['max']:.2f}ms, 平均={stats['avg']:.2f}ms, 样本数={stats['samples']}")


# 创建全局性能监控实例
perf_monitor = PerformanceMonitor()

# 配置字典 - 保留关键功能的完整性，只优化非关键部分
CONFIG = {
    'conf_threshold': 0.3,  # 检测置信度阈值
    'nms_confidence_threshold': 0.05,  # 保持原始值，确保检测准确性
    'nms_iou_threshold': 0.1,  # 保持原始值，确保检测准确性
    'max_x_distance_ratio': 0.2,  # 保持原始值，确保匹配准确性
    'dominant_color_k': 4,  # 颜色聚类数量
    'resize_detection': False,  # 检测前是否缩放图像
    'detection_width': 640,  # 分辨率
    'detection_height': 640,  # 分辨率（注意：RKNN模型通常期望正方形输入）
    'color_sample_size': 50,  # 保持较大的采样尺寸以确保颜色准确性
    'preload_model': True,  # 预加载模型减少延迟
    'rknn_model_path': '../data/best3.rknn',  # RKNN模型文件路径（相对于following_robot包）
}

# 服装类别
CLOTHING_CATEGORIES = {
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

# 全局变量
MODEL = None  # 模型缓存

# YOLOv5模型的相关常量
OBJ_THRESH = 0.25
NMS_THRESH = 0.45
IMG_SIZE = 640

CLASSES = ('short_sleeved_shirt',
           'long_sleeved_shirt',
           'short_sleeved_outwear',
           'long_sleeved_outwear',
           'vest',
           'sling', 'shorts',
           'trousers',
           'skirt',
           'short_sleeved_dress',
           'long_sleeved_dress',
           'vest_dress',
           'sling_dress')


def sigmoid(x):
    """
    sigmoid激活函数
    """
    return 1 / (1 + np.exp(-x))


def xywh2xyxy(x):
    """
    将[x, y, w, h]格式转换为[x1, y1, x2, y2]格式
    """
    y = np.copy(x)
    y[:, 0] = x[:, 0] - x[:, 2] / 2  # top left x
    y[:, 1] = x[:, 1] - x[:, 3] / 2  # top left y
    y[:, 2] = x[:, 0] + x[:, 2] / 2  # bottom right x
    y[:, 3] = x[:, 1] + x[:, 3] / 2  # bottom right y
    return y


def process(input, mask, anchors):
    """
    处理YOLOv5输出特征图
    """
    anchors = [anchors[i] for i in mask]
    grid_h, grid_w = map(int, input.shape[0:2])

    box_confidence = sigmoid(input[..., 4])
    box_confidence = np.expand_dims(box_confidence, axis=-1)

    box_class_probs = sigmoid(input[..., 5:])

    box_xy = sigmoid(input[..., :2]) * 2 - 0.5

    col = np.tile(np.arange(0, grid_w), grid_w).reshape(-1, grid_w)
    row = np.tile(np.arange(0, grid_h).reshape(-1, 1), grid_h)
    col = col.reshape(grid_h, grid_w, 1, 1).repeat(3, axis=-2)
    row = row.reshape(grid_h, grid_w, 1, 1).repeat(3, axis=-2)
    grid = np.concatenate((col, row), axis=-1)
    box_xy += grid
    box_xy *= int(IMG_SIZE / grid_h)

    box_wh = pow(sigmoid(input[..., 2:4]) * 2, 2)
    box_wh = box_wh * anchors

    box = np.concatenate((box_xy, box_wh), axis=-1)

    return box, box_confidence, box_class_probs


def filter_boxes(boxes, box_confidences, box_class_probs):
    """
    过滤检测框，根据置信度阈值
    """
    boxes = boxes.reshape(-1, 4)
    box_confidences = box_confidences.reshape(-1)
    box_class_probs = box_class_probs.reshape(-1, box_class_probs.shape[-1])

    _box_pos = np.where(box_confidences >= OBJ_THRESH)
    boxes = boxes[_box_pos]
    box_confidences = box_confidences[_box_pos]
    box_class_probs = box_class_probs[_box_pos]

    class_max_score = np.max(box_class_probs, axis=-1)
    classes = np.argmax(box_class_probs, axis=-1)
    _class_pos = np.where(class_max_score >= OBJ_THRESH)

    boxes = boxes[_class_pos]
    classes = classes[_class_pos]
    scores = (class_max_score * box_confidences)[_class_pos]

    return boxes, classes, scores


def nms_boxes(boxes, scores):
    """
    非最大抑制，去除重叠框
    """
    x = boxes[:, 0]
    y = boxes[:, 1]
    w = boxes[:, 2] - boxes[:, 0]
    h = boxes[:, 3] - boxes[:, 1]

    areas = w * h
    order = scores.argsort()[::-1]

    keep = []
    while order.size > 0:
        i = order[0]
        keep.append(i)

        xx1 = np.maximum(x[i], x[order[1:]])
        yy1 = np.maximum(y[i], y[order[1:]])
        xx2 = np.minimum(x[i] + w[i], x[order[1:]] + w[order[1:]])
        yy2 = np.minimum(y[i] + h[i], y[order[1:]] + h[order[1:]])

        w1 = np.maximum(0.0, xx2 - xx1 + 0.00001)
        h1 = np.maximum(0.0, yy2 - yy1 + 0.00001)
        inter = w1 * h1

        ovr = inter / (areas[i] + areas[order[1:]] - inter)
        inds = np.where(ovr <= NMS_THRESH)[0]
        order = order[inds + 1]
    keep = np.array(keep)
    return keep


def yolov5_post_process(input_data):
    """
    YOLOv5后处理
    """
    masks = [[0, 1, 2], [3, 4, 5], [6, 7, 8]]
    anchors = [[10, 13], [16, 30], [33, 23], [30, 61], [62, 45],
               [59, 119], [116, 90], [156, 198], [373, 326]]

    boxes, classes, scores = [], [], []
    for input, mask in zip(input_data, masks):
        b, c, s = process(input, mask, anchors)
        b, c, s = filter_boxes(b, c, s)
        boxes.append(b)
        classes.append(c)
        scores.append(s)

    boxes = np.concatenate(boxes)
    boxes = xywh2xyxy(boxes)
    classes = np.concatenate(classes)
    scores = np.concatenate(scores)

    nboxes, nclasses, nscores = [], [], []
    for c in set(classes):
        inds = np.where(classes == c)
        b = boxes[inds]
        c = classes[inds]
        s = scores[inds]

        keep = nms_boxes(b, s)

        nboxes.append(b[keep])
        nclasses.append(c[keep])
        nscores.append(s[keep])

    if not nclasses and not nscores:
        return None, None, None

    boxes = np.concatenate(nboxes)
    classes = np.concatenate(nclasses)
    scores = np.concatenate(nscores)

    return boxes, classes, scores


def letterbox(im, new_shape=(640, 640), color=(0, 0, 0)):
    """
    调整图像大小并保持宽高比
    """
    shape = im.shape[:2]  # current shape [height, width]
    if isinstance(new_shape, int):
        new_shape = (new_shape, new_shape)

    # Scale ratio (new / old)
    r = min(new_shape[0] / shape[0], new_shape[1] / shape[1])

    # Compute padding
    ratio = r, r  # width, height ratios
    new_unpad = int(round(shape[1] * r)), int(round(shape[0] * r))
    dw, dh = new_shape[1] - new_unpad[0], new_shape[0] - new_unpad[1]  # wh padding

    dw /= 2  # divide padding into 2 sides
    dh /= 2

    if shape[::-1] != new_unpad:  # resize
        im = cv.resize(im, new_unpad, interpolation=cv.INTER_LINEAR)
    top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
    left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
    im = cv.copyMakeBorder(im, top, bottom, left, right, cv.BORDER_CONSTANT, value=color)  # add border
    return im, ratio, (dw, dh)


def load_model():
    """
    加载RKNN模型用于服装检测

    返回值:
        加载的RKNN模型对象
    """
    try:
        # 尝试多个路径来查找模型文件
        current_file_path = os.path.abspath(__file__)
        current_dir = os.path.dirname(current_file_path)
        
        # 候选路径列表
        candidate_paths = [
            # 1. 开发环境中的相对路径
            os.path.join(current_dir, CONFIG['rknn_model_path']),
            # 2. ROS2安装后的share路径
            os.path.join(current_dir, '..', '..', '..', 'share', 'following_robot', 'data', 'best3.rknn'),
            # 3. 源码workspace中的路径
            os.path.join(current_dir, '..', '..', 'data', 'best3.rknn'),
            # 4. 绝对路径尝试（基于当前工作目录）
            os.path.join(os.getcwd(), 'install', 'following_robot', 'share', 'following_robot', 'data', 'best3.rknn'),
            # 5. 另一种可能的安装路径
            '/userdata/try_again/SelfFollowingROS2/install/following_robot/share/following_robot/data/best3.rknn'
        ]
        
        model_file_path = None
        for path in candidate_paths:
            abs_path = os.path.abspath(path)
            if os.path.exists(abs_path):
                model_file_path = abs_path
                logger.info(f"找到模型文件: {model_file_path}")
                break
        
        if model_file_path is None:
            logger.error("未找到模型文件，尝试的路径:")
            for i, path in enumerate(candidate_paths, 1):
                logger.error(f"  {i}. {os.path.abspath(path)}")
            return None

        # 创建RKNN对象
        rknn = RKNNLite()

        # 加载RKNN模型
        logger.info('正在加载RKNN模型...')
        ret = rknn.load_rknn(model_file_path)
        if ret != 0:
            logger.error(f'加载RKNN模型失败: {ret}')
            return None

        # 初始化运行时环境
        logger.info('初始化运行时环境...')
        ret = rknn.init_runtime()
        if ret != 0:
            logger.error(f'初始化运行时环境失败: {ret}')
            return None

        logger.info(f"模型已成功加载: {model_file_path}")
        return rknn
    except Exception as e:
        logger.error(f"模型加载失败: {str(e)}")
        return None


# 如果配置为预加载，立即加载模型
if CONFIG['preload_model']:
    try:
        MODEL = load_model()
    except Exception as e:
        logger.error(f"预加载模型失败: {str(e)}")
        MODEL = None


def get_model():
    """
    获取模型，如果尚未加载则加载
    """
    global MODEL
    if MODEL is None:
        MODEL = load_model()
    return MODEL


def get_dominant_color(image, k=None, resize=True):
    """
    使用改进的K-means聚类计算图像的主要颜色

    参数:
        image (numpy.ndarray): 输入图像
        k (int, optional): 聚类数量
        resize (bool, optional): 是否缩放图像以加快处理

    返回值:
        tuple: 主要颜色的BGR值
    """
    if k is None:
        k = CONFIG['dominant_color_k']

    # 图像检查
    if image is None or image.size == 0:
        logger.warning("提供给get_dominant_color的图像为空")
        return (0, 0, 0)

    try:
        # 缩放处理以加快速度
        if resize:
            # 计算最佳缩放尺寸
            sample_size = CONFIG['color_sample_size']
            h, w = image.shape[:2]
            if h > sample_size or w > sample_size:
                scale = min(sample_size / h, sample_size / w)
                new_size = (max(2, int(w * scale)), max(2, int(h * scale)))
                resized_image = cv.resize(image, new_size, interpolation=cv.INTER_AREA)
            else:
                resized_image = image
        else:
            resized_image = image

        # 预处理：去除极暗和极亮的像素，这些通常是噪声或边缘
        # 转换为HSV颜色空间
        hsv_image = cv.cvtColor(resized_image, cv.COLOR_BGR2HSV)
        h, s, v = cv.split(hsv_image)

        # 创建掩码，过滤掉极暗和极亮的像素
        mask = cv.inRange(v, 30, 220)  # 保留亮度在合理范围内的像素

        # 同时过滤掉饱和度过低的像素（接近灰色/白色/黑色）
        mask = cv.bitwise_and(mask, cv.inRange(s, 30, 255))  # 保留饱和度足够高的像素

        # 应用掩码
        masked_image = cv.bitwise_and(resized_image, resized_image, mask=mask)

        # 如果掩码后几乎没有像素，就使用原始图像
        if cv.countNonZero(mask) < 50:
            masked_image = resized_image

        # 将图像转为2D数组，只包含有效像素
        pixels = masked_image.reshape((-1, 3))
        pixels = pixels[np.any(pixels != [0, 0, 0], axis=1)]  # 移除黑色像素(掩码外)

        # 如果没有足够的像素，回退到原始图像
        if len(pixels) < 50:
            pixels = resized_image.reshape((-1, 3))

        # 转换为float32类型，K-means需要
        pixels = np.float32(pixels)

        # 如果有足够的像素才进行聚类
        if pixels.shape[0] >= k:
            # K-means聚类参数（增加迭代次数和精度）
            criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 20, 0.5)

            # 进行K-means聚类
            _, labels, centers = cv.kmeans(
                pixels, k, None, criteria, 10, cv.KMEANS_PP_CENTERS  # 使用更好的初始化方法
            )

            # 统计每个聚类的像素数
            unique_labels, counts = np.unique(labels, return_counts=True)

            # 过滤出有意义的聚类（忽略极少数量的聚类）
            valid_clusters = []
            total_pixels = sum(counts)

            for i, (label, count) in enumerate(zip(unique_labels, counts)):
                # 只保留占比超过5%的聚类
                if count > total_pixels * 0.05:
                    valid_clusters.append((centers[label], count))

            # 如果有有效聚类，选择最大的
            if valid_clusters:
                valid_clusters.sort(key=lambda x: x[1], reverse=True)
                dominant_color = valid_clusters[0][0].astype(int)
                return tuple(dominant_color)

        # 降级方案：使用平均颜色
        mean_color = np.mean(pixels, axis=0).astype(int)
        return tuple(mean_color)

    except Exception as e:
        logger.error(f"在get_dominant_color中出错: {str(e)}")
        traceback.print_exc()
        return (0, 0, 0)  # 失败时返回黑色


def apply_nms(detections, confidence_threshold=None, iou_threshold=None):
    """
    应用非极大值抑制过滤重叠检测

    参数:
        detections (list): 检测列表 [xmin, ymin, xmax, ymax, conf, class_id]
        confidence_threshold (float, optional): 置信度阈值
        iou_threshold (float, optional): IoU阈值

    返回值:
        list: 过滤后的检测结果
    """
    if confidence_threshold is None:
        confidence_threshold = CONFIG['nms_confidence_threshold']
    if iou_threshold is None:
        iou_threshold = CONFIG['nms_iou_threshold']

    boxes = []
    confidences = []
    class_ids = []

    for detection in detections:
        xmin, ymin, xmax, ymax, conf, class_id = detection
        if conf > confidence_threshold:
            # 转换为 [x, y, w, h] 格式
            boxes.append([xmin, ymin, xmax - xmin, ymax - ymin])
            confidences.append(conf)
            class_ids.append(class_id)

    # 应用NMS
    filtered_detections = []
    if boxes:
        indices = cv.dnn.NMSBoxes(
            boxes, confidences, confidence_threshold, iou_threshold
        )

        if len(indices) > 0:
            for i in indices.flatten():
                box = boxes[i]
                xmin, ymin = box[0], box[1]
                xmax, ymax = box[0] + box[2], box[1] + box[3]
                filtered_detections.append(
                    [xmin, ymin, xmax, ymax, confidences[i], class_ids[i]]
                )

    return filtered_detections


def match_clothing_items(upper_items, lower_items, img, pairs, max_x_distance_ratio=None):
    """
    匹配上衣和下装 - 保持完整算法逻辑以确保准确性

    参数:
        upper_items (list): 上衣列表
        lower_items (list): 下装列表
        img (numpy.ndarray): 输入图像
        pairs (list): 输出匹配对列表
        max_x_distance_ratio (float, optional): 最大水平距离比例
    """
    if not upper_items and not lower_items:
        return

    if max_x_distance_ratio is None:
        max_x_distance_ratio = CONFIG['max_x_distance_ratio']

    height, width = img.shape[:2]
    max_x_distance = width * max_x_distance_ratio

    # 简化为坐标
    simplified_upper = [item[:4] for item in upper_items]
    simplified_lower = [item[:4] for item in lower_items]
    lower_items_copy = simplified_lower.copy()

    # 对每件上衣
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
            lower_items_copy.remove(closest_lower)  # 避免重复使用下装
        else:
            pairs.append([upper, (-1,)])

    # 添加剩余未配对的下装
    for lower in lower_items_copy:
        pairs.append([(-1,), lower])


def identify_clothing_colors(pairs, img):
    """
    识别每件服装的主要颜色 - 改进版本，提高颜色准确性

    参数:
        pairs (list): 服装配对列表
        img (numpy.ndarray): 输入图像
    """
    logger.debug(f"开始识别 {len(pairs)} 对服装的颜色")

    for pair_idx, pair in enumerate(pairs):
        logger.debug(f"处理服装对 {pair_idx + 1}/{len(pairs)}")

        for i, item in enumerate(pair[:2]):  # 只处理前两项(上衣和下装)
            item_type = "上衣" if i == 0 else "下装"

            if len(item) == 1:  # 占位符
                logger.debug(f"{item_type}为占位符，跳过颜色识别")
                pair.append(())
                continue

            if len(item) < 4:  # 无效项
                logger.debug(f"{item_type}项无效，跳过颜色识别")
                break

            # 提取服装区域
            xmin, ymin, xmax, ymax = item[0], item[1], item[2], item[3]
            logger.debug(f"{item_type}边界框: [{xmin}, {ymin}, {xmax}, {ymax}]")

            try:
                # 检查边界框是否有效
                if xmin >= xmax or ymin >= ymax or xmin < 0 or ymin < 0 or xmax > img.shape[1] or ymax > img.shape[0]:
                    logger.warning(f"无效的{item_type}边界框: {item}")
                    pair.append((0, 0, 0))
                    continue

                clothing_region = img[ymin:ymax, xmin:xmax]
                # clothing_region = get_isolated_clothing((xmin, ymin, xmax, ymax), img)
                logger.debug(f"{item_type}区域尺寸: {clothing_region.shape}")

                # 检查区域是否有效
                if clothing_region.size == 0 or clothing_region.shape[0] == 0 or clothing_region.shape[1] == 0:
                    logger.warning(
                        f"无效的{item_type}区域: shape={clothing_region.shape if clothing_region is not None else None}")
                    pair.append((0, 0, 0))
                    continue

                # 计算颜色提取开始时间
                color_extract_start = time.time()

                # 根据不同衣物类型调整处理方式
                if i == 0:  # 上衣
                    # 上衣通常颜色更鲜明，使用更多的聚类
                    k_value = 5
                    logger.debug(f"为上衣使用K={k_value}的聚类")
                    dominant_color = get_dominant_color(clothing_region, k=k_value)
                    logger.debug(f"上衣主要颜色: BGR={dominant_color}")
                else:  # 下装
                    # 下装通常颜色较暗，使用较少聚类
                    k_value = 3
                    logger.debug(f"为下装使用K={k_value}的聚类")
                    dominant_color = get_dominant_color(clothing_region, k=k_value)
                    logger.debug(f"下装主要颜色: BGR={dominant_color}")

                # 验证颜色是否有效（非黑色或纯白色，这些可能是错误值）
                is_black = all(c < 30 for c in dominant_color)
                is_white = all(c > 225 for c in dominant_color)

                if is_black or is_white:
                    logger.debug(f"{item_type}颜色识别为{'黑色' if is_black else '白色'}，进行验证...")

                    # 如果识别为黑色或白色，进行更细致的分析
                    hsv_region = cv.cvtColor(clothing_region, cv.COLOR_BGR2HSV)
                    h, s, v = cv.split(hsv_region)

                    # 计算平均饱和度和亮度
                    avg_s = np.mean(s)
                    avg_v = np.mean(v)
                    logger.debug(f"{item_type} HSV统计: 平均饱和度={avg_s:.2f}, 平均亮度={avg_v:.2f}")

                    if is_black and avg_v < 50:
                        # 确认是黑色
                        logger.debug(f"确认{item_type}颜色为黑色")
                        dominant_color = (0, 0, 0)
                    elif is_white and avg_v > 200 and avg_s < 30:
                        # 确认是白色
                        logger.debug(f"确认{item_type}颜色为白色")
                        dominant_color = (255, 255, 255)
                    else:
                        # 尝试使用更宽松的参数重新计算
                        logger.debug(f"{item_type}颜色可能不是纯黑/白，使用K=6重新计算")
                        dominant_color = get_dominant_color(clothing_region, k=6)
                        logger.debug(f"重新计算的{item_type}颜色: BGR={dominant_color}")

                # 将颜色转换为HSV以获取更多可读信息
                try:
                    bgr_color = np.uint8([[list(dominant_color)]])
                    hsv_color = cv.cvtColor(bgr_color, cv.COLOR_BGR2HSV)[0][0]
                    logger.debug(f"{item_type}颜色: BGR={dominant_color}, HSV={hsv_color}")
                except Exception as e:
                    logger.debug(f"{item_type}颜色转换失败: {e}, 使用原始BGR值")

                # 计算颜色提取耗时
                color_extract_time = (time.time() - color_extract_start) * 1000
                logger.debug(f"{item_type}颜色提取耗时: {color_extract_time:.2f}ms")

                # 添加颜色信息到配对中
                pair.append(dominant_color)

            except Exception as e:
                logger.error(f"{item_type}颜色识别出错: {str(e)}")
                traceback.print_exc()
                pair.append((0, 0, 0))  # 失败时使用黑色


def detect_picture(img):
    """
    使用RKNN检测图像中的服装

    参数:
        img (numpy.ndarray): 输入图像

    返回值:
        list: 检测到的服装配对列表
    """
    # 记录检测开始时间(用于性能监控)
    detect_start_time = time.time()

    model = get_model()
    if model is None:
        logger.error("模型未加载，无法进行检测")
        return []

    if img is None or img.size == 0:
        logger.error("提供给detect_picture的图像无效")
        return []

    try:
        # 保存原始图像副本
        original_img = img.copy()
        logger.debug(f"原始图像尺寸: {img.shape}")

        # 预处理图像 - 使用letterbox而不是简单缩放，保持宽高比
        perf_monitor.start("preprocessing")
        img_for_detection, ratio, pad = letterbox(img, new_shape=(IMG_SIZE, IMG_SIZE))
        logger.debug(f"预处理后图像尺寸: {img_for_detection.shape}, 缩放比例: {ratio}, 填充: {pad}")

        # 转换颜色空间从BGR到RGB（RKNN模型通常使用RGB输入）
        img_for_detection = cv.cvtColor(img_for_detection, cv.COLOR_BGR2RGB)

        # 扩展维度以匹配模型输入要求
        img_for_detection = np.expand_dims(img_for_detection, 0)
        logger.debug(f"模型输入尺寸: {img_for_detection.shape}")
        perf_monitor.end("preprocessing")
        preprocess_stats = perf_monitor.get_stats("preprocessing")
        if preprocess_stats:
            logger.debug(f"预处理耗时: {preprocess_stats['avg']:.2f}ms")

        # 运行RKNN推理
        logger.debug('开始RKNN推理...')
        perf_monitor.start("inference")
        outputs = model.inference(inputs=[img_for_detection])
        perf_monitor.end("inference")
        inference_stats = perf_monitor.get_stats("inference")
        if inference_stats:
            logger.debug(f"RKNN推理完成，耗时: {inference_stats['avg']:.2f}ms")

        # 处理RKNN输出
        perf_monitor.start("postprocessing")
        if outputs is None or len(outputs) < 3:
            logger.error("RKNN推理输出异常")
            return []
        
        input0_data = outputs[0]
        input1_data = outputs[1]
        input2_data = outputs[2]

        logger.debug(f"输出特征图形状: {input0_data.shape}, {input1_data.shape}, {input2_data.shape}")

        # 重塑输出以匹配YOLOv5后处理期望格式
        input0_data = input0_data.reshape([3, -1] + list(input0_data.shape[-2:]))
        input1_data = input1_data.reshape([3, -1] + list(input1_data.shape[-2:]))
        input2_data = input2_data.reshape([3, -1] + list(input2_data.shape[-2:]))

        # 转置以匹配YOLOv5后处理期望格式
        input_data = [
            np.transpose(input0_data, (2, 3, 0, 1)),
            np.transpose(input1_data, (2, 3, 0, 1)),
            np.transpose(input2_data, (2, 3, 0, 1))
        ]

        # YOLOv5后处理
        logger.debug('正在执行YOLOv5后处理...')
        boxes, classes, scores = yolov5_post_process(input_data)

        # 将坐标从letterbox空间转回原始图像空间
        if boxes is not None:
            # 获取原始图像尺寸
            orig_h, orig_w = original_img.shape[:2]

            # 获取缩放比例和填充信息
            r = ratio[0]  # 宽高使用相同的缩放比例
            dw, dh = pad  # 填充信息

            # 调整所有边界框坐标
            for i in range(len(boxes)):
                # 去除填充偏移并按比例缩放
                boxes[i][0] = (boxes[i][0] - dw) / r  # xmin
                boxes[i][1] = (boxes[i][1] - dh) / r  # ymin
                boxes[i][2] = (boxes[i][2] - dw) / r  # xmax
                boxes[i][3] = (boxes[i][3] - dh) / r  # ymax

                # 确保坐标在有效范围内
                boxes[i][0] = max(0, min(orig_w - 1, boxes[i][0]))
                boxes[i][1] = max(0, min(orig_h - 1, boxes[i][1]))
                boxes[i][2] = max(0, min(orig_w, boxes[i][2]))
                boxes[i][3] = max(0, min(orig_h, boxes[i][3]))

        perf_monitor.end("postprocessing")
        postprocess_stats = perf_monitor.get_stats("postprocessing")
        if postprocess_stats:
            logger.debug(f"后处理耗时: {postprocess_stats['avg']:.2f}ms")

        # 初始化结果容器
        upper_clothing = []
        lower_clothing = []
        pairs = []

        # 如果没有检测到任何物体
        if boxes is None:
            logger.debug("未检测到任何服装")
            return pairs

        logger.debug(f"检测到 {len(boxes)} 个物体，开始分类和处理...")

        # 处理检测结果
        for i, (box, score, cls) in enumerate(zip(boxes, scores, classes)):
            # 获取坐标
            xmin, ymin, xmax, ymax = box
            xmin = int(xmin)
            ymin = int(ymin)
            xmax = int(xmax)
            ymax = int(ymax)
            confidence = score
            class_name = CLASSES[cls]

            logger.debug(
                f"物体 {i + 1}: 类别={class_name}, 置信度={confidence:.4f}, 坐标=[{xmin}, {ymin}, {xmax}, {ymax}]")

            # 在图像上添加标签
            label = f"{class_name} {confidence:.2f}"
            cv.putText(
                original_img, label, (xmin, ymin - 10),
                cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2
            )

            # 分类检测结果
            if class_name in CLOTHING_CATEGORIES['upper']:
                upper_clothing.append((xmin, ymin, xmax, ymax, confidence, 'upper'))
                logger.debug(f"将 {class_name} 分类为上衣")
            elif class_name in CLOTHING_CATEGORIES['lower']:
                lower_clothing.append((xmin, ymin, xmax, ymax, confidence, 'lower'))
                logger.debug(f"将 {class_name} 分类为下装")

        # 应用NMS
        logger.debug(f"NMS前上衣数量: {len(upper_clothing)}, 下装数量: {len(lower_clothing)}")
        upper_clothing = apply_nms(upper_clothing)
        lower_clothing = apply_nms(lower_clothing)
        logger.debug(f"NMS后上衣数量: {len(upper_clothing)}, 下装数量: {len(lower_clothing)}")

        # 匹配服装项目
        logger.debug("开始匹配上下装...")
        match_start = time.time()
        match_clothing_items(upper_clothing, lower_clothing, original_img, pairs)
        logger.debug(f"匹配完成，找到 {len(pairs)} 对服装，耗时: {(time.time() - match_start) * 1000:.2f}ms")

        # 获取颜色
        logger.debug("开始识别服装颜色...")
        color_start = time.time()
        identify_clothing_colors(pairs, original_img)
        logger.debug(f"颜色识别完成，耗时: {(time.time() - color_start) * 1000:.2f}ms")

        # 使用原始图像进行后续操作
        img = original_img

        # 汇总检测结果
        for i, pair in enumerate(pairs):
            upper_info = f"上衣: {pair[0]}" if len(pair[0]) > 1 else "未检测到上衣"
            lower_info = f"下装: {pair[1]}" if len(pair[1]) > 1 else "未检测到下装"
            color_info = ""
            if len(pair) > 2 and pair[2]:
                color_info += f", 上衣颜色: {pair[2]}"
            if len(pair) > 3 and pair[3]:
                color_info += f", 下装颜色: {pair[3]}"
            logger.debug(f"服装组合 {i + 1}: {upper_info}, {lower_info}{color_info}")

        # 计算总处理时间
        total_time = time.time() - detect_start_time
        logger.debug(f"检测完成，总耗时: {total_time * 1000:.2f}ms, 约 {1 / total_time:.1f} FPS")

        return pairs

    except Exception as e:
        logger.error(f"图像检测出错: {str(e)}")
        traceback.print_exc()
        return []


def Determine_the_position_of_the_entire_body(c, p, img):
    """
    估计整个身体位置

    参数:
        c (tuple): 上衣坐标
        p (tuple): 下装坐标
        img (numpy.ndarray): 图像

    返回值:
        list: 身体位置坐标
    """
    person_position = []

    if img is None or img.size == 0:
        return person_position

    img_height, img_width = img.shape[:2]

    try:
        if len(c) == 4 and len(p) == 4:
            # 两者都检测到 - 计算整个人体区域
            person_xmin = min(c[0], p[0])
            person_ymin = min(c[1], p[1])
            person_xmax = max(c[2], p[2])
            person_ymax = max(c[3], p[3])

            # 添加头部和脚部扩展
            upper_height = c[3] - c[1]
            lower_height = p[3] - p[1]
            upper_width = c[2] - c[0]

            head_extension = int(upper_height * 0.5)
            foot_extension = int(lower_height * 0.6)
            side_extension = int(upper_width * 0.3)

            # 应用扩展（带边界检查）
            person_xmin = max(0, person_xmin - side_extension)
            person_xmax = min(img_width, person_xmax + side_extension)
            person_ymin = max(0, person_ymin - head_extension)
            person_ymax = min(img_height, person_ymax + foot_extension)

        elif len(c) == 4 and len(p) == 1:
            # 只有上衣
            upper_height = c[3] - c[1]

            person_xmin = max(0, c[0] - 40)
            person_ymin = max(0, c[1] - int(upper_height * 0.4))
            person_xmax = min(img_width, c[2] + 40)
            person_ymax = min(img_height, c[3] + int(upper_height * 2.5))

        elif len(c) == 1 and len(p) == 4:
            # 只有下装
            lower_height = p[3] - p[1]

            person_xmin = max(0, p[0] - 40)
            person_ymin = max(0, p[1] - int(lower_height * 1.8))
            person_xmax = min(img_width, p[2] + 40)
            person_ymax = min(img_height, p[3] + int(lower_height * 0.7))
        else:
            # 无效输入
            return person_position

        person_position.append((int(person_xmin), int(person_ymin),
                                int(person_xmax), int(person_ymax)))

    except Exception as e:
        logger.error(f"确定身体位置出错: {str(e)}")

    return person_position


def detect_position(c, p, img):
    """
    在图像上绘制检测框

    参数:
        c (tuple): 上衣坐标
        p (tuple): 下装坐标
        img (numpy.ndarray): 图像
    """
    if len(c) == 4:
        cv.rectangle(img, (c[0], c[1]), (c[2], c[3]), (255, 0, 0), 2)

    if len(p) == 4:
        cv.rectangle(img, (p[0], p[1]), (p[2], p[3]), (0, 255, 0), 2)


# 遗留兼容性函数
def Identify_clothing_color(pairs, img):
    """兼容旧接口"""
    identify_clothing_colors(pairs, img)


def Obtain_the_target_color(img):
    """兼容旧接口"""
    return detect_picture(img)


def batch_process_images(input_folder, output_folder):
    """
    批量处理图像

    参数:
        input_folder (str): 输入文件夹路径
        output_folder (str): 输出文件夹路径
    """
    # 创建输出文件夹（如果不存在）
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)
        logger.info(f"已创建输出目录: {output_folder}")

    # 有效图像扩展名
    image_extensions = ('.png', '.jpg', '.jpeg', '.bmp', '.tiff')

    try:
        # 获取图像文件列表
        image_files = [
            f for f in os.listdir(input_folder)
            if os.path.isfile(os.path.join(input_folder, f)) and
               f.lower().endswith(image_extensions)
        ]

        logger.info(f"找到 {len(image_files)} 个图像待处理")

        # 处理每个图像
        for filename in image_files:
            input_path = os.path.join(input_folder, filename)
            output_path = os.path.join(output_folder, filename)

            try:
                # 读取图像
                image = cv.imread(input_path)

                if image is None:
                    logger.warning(f"读取图像失败: {input_path}")
                    continue

                # 处理图像
                pairs = detect_picture(image)

                # 计算身体位置
                for pair in pairs:
                    Determine_the_position_of_the_entire_body(pair[0], pair[1], image)

                # 保存结果
                cv.imwrite(output_path, image)
                logger.info(f"已保存: {output_path}")

            except Exception as e:
                logger.error(f"处理 {input_path} 时出错: {str(e)}")
    except Exception as e:
        logger.error(f"批处理出错: {str(e)}")


def create_performance_heatmap(stats, width=400, height=80):
    """
    创建一个性能热力图，显示各个阶段的处理时间

    参数:
        stats (dict): 性能统计数据
        width (int): 图像宽度
        height (int): 图像高度

    返回:
        numpy.ndarray: 热力图图像
    """
    # 创建一个空白图像
    heatmap = np.ones((height, width, 3), dtype=np.uint8) * 255

    # 性能阶段和颜色映射
    stages = [
        ("preprocessing", (0, 255, 0)),  # 绿色
        ("inference", (0, 0, 255)),  # 红色
        ("postprocessing", (255, 0, 0)),  # 蓝色
        ("frame_processing", (0, 255, 255))  # 黄色
    ]

    # 找出最大处理时间，用于归一化
    max_time = 0
    for stage, _ in stages:
        if stage in stats and stats[stage]:
            max_time = max(max_time, stats[stage]["max"])

    if max_time == 0:
        cv.putText(heatmap, "无性能数据", (10, height // 2), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
        return heatmap

    # 绘制各阶段的性能条
    y_step = height // (len(stages) + 1)
    for i, (stage, color) in enumerate(stages):
        y = (i + 1) * y_step

        if stage in stats and stats[stage]:
            # 获取该阶段的统计数据
            avg = stats[stage]["avg"]
            min_val = stats[stage]["min"]
            max_val = stats[stage]["max"]

            # 计算条的长度
            avg_width = int((avg / max_time) * (width - 100))
            min_width = int((min_val / max_time) * (width - 100))
            max_width = int((max_val / max_time) * (width - 100))

            # 绘制最小值到最大值的范围
            cv.rectangle(heatmap, (50, y - 5), (50 + max_width, y + 5), (200, 200, 200), -1)

            # 绘制平均值条
            cv.rectangle(heatmap, (50, y - 5), (50 + avg_width, y + 5), color, -1)

            # 添加标签
            cv.putText(heatmap, f"{stage}", (5, y + 3), cv.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 1)
            cv.putText(heatmap, f"{avg:.1f}ms", (55 + avg_width, y + 3), cv.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 1)

    # 添加标题
    cv.putText(heatmap, "性能热力图 (毫秒)", (width // 3, 15), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)

    return heatmap


def process_single_image(image_path, show_result=True, save_result=False, output_path=None):
    """
    处理单张图像文件

    参数:
        image_path (str): 输入图像的路径
        show_result (bool): 是否显示处理结果
        save_result (bool): 是否保存处理结果
        output_path (str): 输出图像的路径（如果保存结果）

    返回:
        tuple: (处理后的图像, 检测到的服装配对列表)
    """
    try:
        # 记录处理开始时间
        process_start_time = time.time()

        # 读取图像
        logger.info(f"正在读取图像: {image_path}")
        image = cv.imread(image_path)

        if image is None:
            logger.error(f"无法读取图像: {image_path}")
            return None, []

        # 创建调试图像（原始图像的副本）
        debug_image = image.copy()

        # 检测图像中的服装
        logger.info("开始检测服装...")
        pairs = detect_picture(image)
        logger.info(f"检测到 {len(pairs)} 套服装")

        # 计算处理时间
        process_time = (time.time() - process_start_time) * 1000
        logger.info(f"处理完成，耗时: {process_time:.2f}ms")

        # 在调试图像上显示处理信息
        font = cv.FONT_HERSHEY_SIMPLEX
        font_scale = 0.5
        font_color = (0, 255, 255)  # 黄色
        line_type = 1

        cv.putText(debug_image, f"Process Time: {process_time:.1f}ms",
                   (10, 20), font, font_scale, font_color, line_type)
        cv.putText(debug_image, f"Detected {len(pairs)} Outfits",
                   (10, 40), font, font_scale, font_color, line_type)

        # 遍历每一对服装，处理并显示信息
        for i, pair in enumerate(pairs):
            # 获取全身位置
            person_positions = Determine_the_position_of_the_entire_body(pair[0], pair[1], image)

            # 在图像上绘制检测框
            detect_position(pair[0], pair[1], image)

            # 显示上下装颜色信息（如果有）
            y_offset = 60 + i * 40

            if len(pair) > 2 and pair[2]:  # 上衣颜色
                color_str = f"Upper: BGR{pair[2]}"
                cv.putText(debug_image, color_str,
                           (10, y_offset), font, font_scale, font_color, line_type)
                logger.info(f"服装{i + 1} - {color_str}")

            if len(pair) > 3 and pair[3]:  # 下装颜色
                color_str = f"Lower: BGR{pair[3]}"
                cv.putText(debug_image, color_str,
                           (10, y_offset + 20), font, font_scale, font_color, line_type)
                logger.info(f"服装{i + 1} - {color_str}")

            # 在全身位置画框
            for pos in person_positions:
                if len(pos) == 4:
                    # 用紫色框标记整个人体
                    cv.rectangle(image, (pos[0], pos[1]), (pos[2], pos[3]), (255, 0, 255), 1)

        # 显示处理结果
        if show_result:
            cv.imshow('result', image)
            cv.imshow('debug', debug_image)
            logger.info("按任意键继续...")
            cv.waitKey(0)
            cv.destroyAllWindows()

        # 保存结果
        if save_result:
            if output_path is None:
                # 如果没有指定输出路径，在输入文件名基础上添加后缀
                filename, ext = os.path.splitext(image_path)
                output_path = f"{filename}_processed{ext}"

            # 创建一个组合图像（左侧原始检测结果，右侧调试信息）
            h, w = image.shape[:2]
            combined = np.zeros((h, w * 2, 3), dtype=np.uint8)
            combined[:, :w] = image

            # 调整调试图像大小以匹配原始图像
            debug_resized = cv.resize(debug_image, (w, h))
            combined[:, w:] = debug_resized

            cv.imwrite(output_path, combined)
            logger.info(f"已保存处理结果到: {output_path}")

        return image, pairs

    except Exception as e:
        logger.error(f"处理图像时出错: {str(e)}")
        traceback.print_exc()
        return None, []


def match_clothing_items_with_confidence(upper_items, lower_items, img, pairs, max_x_distance_ratio=None):
    """
    匹配上衣和下装，同时保留置信度信息

    参数:
        upper_items (list): 上衣列表，格式为 [(xmin, ymin, xmax, ymax, confidence, 'upper'), ...]
        lower_items (list): 下装列表，格式为 [(xmin, ymin, xmax, ymax, confidence, 'lower'), ...]
        img (numpy.ndarray): 输入图像
        pairs (list): 输出匹配对列表
        max_x_distance_ratio (float, optional): 最大水平距离比例
    """
    if not upper_items and not lower_items:
        return

    if max_x_distance_ratio is None:
        max_x_distance_ratio = CONFIG['max_x_distance_ratio']

    height, width = img.shape[:2]
    max_x_distance = width * max_x_distance_ratio

    # 分离坐标和置信度信息
    upper_coords = [(item[0], item[1], item[2], item[3]) for item in upper_items]
    upper_confidences = [item[4] for item in upper_items]

    lower_coords = [(item[0], item[1], item[2], item[3]) for item in lower_items]
    lower_confidences = [item[4] for item in lower_items]

    # 需要跟踪配对过程中使用的索引
    lower_coords_copy = lower_coords.copy()
    lower_indices = list(range(len(lower_coords)))
    lower_indices_copy = lower_indices.copy()

    # 对每件上衣
    for upper_idx, upper in enumerate(upper_coords):
        closest_lower_idx = None
        closest_lower = None
        min_distance = float('inf')

        # 计算上衣中心点
        upper_center_x = (upper[0] + upper[2]) // 2
        upper_center_y = (upper[1] + upper[3]) // 2

        for i, lower in enumerate(lower_coords_copy):
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
                closest_lower_idx = i

        # 添加配对或未配对的上衣（包含置信度信息）
        if closest_lower:
            # 获取下装对应的真实索引和置信度
            original_lower_idx = lower_indices_copy[closest_lower_idx]
            lower_confidence = lower_confidences[original_lower_idx]

            # 将配对添加到结果中，包含上衣和下装的置信度
            pairs.append([upper, closest_lower, upper_confidences[upper_idx], lower_confidence])

            # 移除已使用的下装
            lower_coords_copy.pop(closest_lower_idx)
            lower_indices_copy.pop(closest_lower_idx)
        else:
            # 未配对的上衣，下装位置为占位符，下装置信度为0
            pairs.append([upper, (-1,), upper_confidences[upper_idx], 0.0])

    # 添加剩余未配对的下装（包含置信度信息）
    for i, lower in enumerate(lower_coords_copy):
        original_lower_idx = lower_indices_copy[i]
        lower_confidence = lower_confidences[original_lower_idx]
        # 未配对的下装，上衣位置为占位符，上衣置信度为0
        pairs.append([(-1,), lower, 0.0, lower_confidence])


def identify_clothing_colors_with_confidence(pairs, img):
    """
    识别每件服装的主要颜色，保留置信度信息

    参数:
        pairs (list): 服装配对列表，格式为 [upper_coords, lower_coords, upper_confidence, lower_confidence]
        img (numpy.ndarray): 输入图像
    """
    logger.debug(f"开始识别 {len(pairs)} 对服装的颜色")

    for pair_idx, pair in enumerate(pairs):
        logger.debug(f"处理服装对 {pair_idx + 1}/{len(pairs)}")

        # 提取置信度信息（暂存）
        upper_confidence = pair[2]
        lower_confidence = pair[3]

        # 移除置信度信息，以便后续处理（原函数期望只有坐标信息）
        coords_only_pair = [pair[0], pair[1]]

        for i, item in enumerate(coords_only_pair[:2]):  # 只处理前两项(上衣和下装)
            item_type = "上衣" if i == 0 else "下装"

            if len(item) == 1:  # 占位符
                logger.debug(f"{item_type}为占位符，跳过颜色识别")
                coords_only_pair.append(())
                continue

            if len(item) < 4:  # 无效项
                logger.debug(f"{item_type}项无效，跳过颜色识别")
                break

            # 提取服装区域
            xmin, ymin, xmax, ymax = item[0], item[1], item[2], item[3]
            logger.debug(f"{item_type}边界框: [{xmin}, {ymin}, {xmax}, {ymax}]")

            try:
                # 先确保坐标在图像范围内
                height, width = img.shape[:2]
                xmin = max(0, xmin)
                ymin = max(0, ymin)
                xmax = min(width, xmax)
                ymax = min(height, ymax)

                # 检查调整后的边界框是否仍有效
                if xmin >= xmax or ymin >= ymax:
                    logger.warning(f"无效的{item_type}边界框: {item}, 调整后: [{xmin}, {ymin}, {xmax}, {ymax}]")
                    pair.append((0, 0, 0))
                    continue

                clothing_region = img[ymin:ymax, xmin:xmax]
                logger.debug(f"{item_type}区域尺寸: {clothing_region.shape}")

                # 检查区域是否有效
                if clothing_region.size == 0 or clothing_region.shape[0] == 0 or clothing_region.shape[1] == 0:
                    logger.warning(
                        f"无效的{item_type}区域: shape={clothing_region.shape if clothing_region is not None else None}")
                    coords_only_pair.append((0, 0, 0))
                    continue

                # 计算颜色提取开始时间
                color_extract_start = time.time()

                # 根据不同衣物类型调整处理方式
                if i == 0:  # 上衣
                    # 上衣通常颜色更鲜明，使用更多的聚类
                    k_value = 5
                    logger.debug(f"为上衣使用K={k_value}的聚类")
                    dominant_color = get_dominant_color(clothing_region, k=k_value)
                    logger.debug(f"上衣主要颜色: BGR={dominant_color}")
                else:  # 下装
                    # 下装通常颜色较暗，使用较少聚类
                    k_value = 3
                    logger.debug(f"为下装使用K={k_value}的聚类")
                    dominant_color = get_dominant_color(clothing_region, k=k_value)
                    logger.debug(f"下装主要颜色: BGR={dominant_color}")

                # 验证颜色是否有效（非黑色或纯白色，这些可能是错误值）
                is_black = all(c < 30 for c in dominant_color)
                is_white = all(c > 225 for c in dominant_color)

                if is_black or is_white:
                    logger.debug(f"{item_type}颜色识别为{'黑色' if is_black else '白色'}，进行验证...")

                    # 如果识别为黑色或白色，进行更细致的分析
                    hsv_region = cv.cvtColor(clothing_region, cv.COLOR_BGR2HSV)
                    h, s, v = cv.split(hsv_region)

                    # 计算平均饱和度和亮度
                    avg_s = np.mean(s)
                    avg_v = np.mean(v)
                    logger.debug(f"{item_type} HSV统计: 平均饱和度={avg_s:.2f}, 平均亮度={avg_v:.2f}")

                    if is_black and avg_v < 50:
                        # 确认是黑色
                        logger.debug(f"确认{item_type}颜色为黑色")
                        dominant_color = (0, 0, 0)
                    elif is_white and avg_v > 200 and avg_s < 30:
                        # 确认是白色
                        logger.debug(f"确认{item_type}颜色为白色")
                        dominant_color = (255, 255, 255)
                    else:
                        # 尝试使用更宽松的参数重新计算
                        logger.debug(f"{item_type}颜色可能不是纯黑/白，使用K=6重新计算")
                        dominant_color = get_dominant_color(clothing_region, k=6)
                        logger.debug(f"重新计算的{item_type}颜色: BGR={dominant_color}")

                # 将颜色转换为HSV以获取更多可读信息
                bgr_color = np.uint8([[dominant_color]])
                hsv_color = cv.cvtColor(bgr_color, cv.COLOR_BGR2HSV)[0][0]
                logger.debug(f"{item_type}颜色: BGR={dominant_color}, HSV={hsv_color}")

                # 计算颜色提取耗时
                color_extract_time = (time.time() - color_extract_start) * 1000
                logger.debug(f"{item_type}颜色提取耗时: {color_extract_time:.2f}ms")

                # 添加颜色信息到配对中
                coords_only_pair.append(dominant_color)

            except Exception as e:
                logger.error(f"{item_type}颜色识别出错: {str(e)}")
                traceback.print_exc()
                coords_only_pair.append((0, 0, 0))  # 失败时使用黑色

        # 重新组合结果，包含颜色和置信度
        # 结构: [上衣坐标, 下装坐标, 上衣颜色, 下装颜色, 上衣置信度, 下装置信度]
        result_pair = []

        # 复制坐标信息
        result_pair.append(pair[0])  # 上衣坐标
        result_pair.append(pair[1])  # 下装坐标

        # 复制颜色信息（如果有）
        if len(coords_only_pair) > 2:
            result_pair.append(coords_only_pair[2])  # 上衣颜色
        else:
            result_pair.append(())  # 无上衣颜色

        if len(coords_only_pair) > 3:
            result_pair.append(coords_only_pair[3])  # 下装颜色
        else:
            result_pair.append(())  # 无下装颜色

        # 添加置信度信息
        result_pair.append(upper_confidence)  # 上衣置信度
        result_pair.append(lower_confidence)  # 下装置信度

        # 更新原始pairs中的元素
        pairs[pair_idx] = result_pair


def detect_picture_with_confidence(img):
    """
    使用RKNN检测图像中的服装，并包含置信度信息

    参数:
        img (numpy.ndarray): 输入图像

    返回值:
        list: 检测到的服装配对列表，每对格式为[上衣位置，下装位置，上衣颜色，下装颜色，上衣置信度，下装置信度]
    """
    # 记录检测开始时间(用于性能监控)
    detect_start_time = time.time()

    model = get_model()
    if model is None:
        logger.error("模型未加载，无法进行检测")
        return []

    if img is None or img.size == 0:
        logger.error("提供给detect_picture_with_confidence的图像无效")
        return []

    try:
        # 保存原始图像副本 - 移除直接的resize操作
        original_img = img.copy()
        logger.debug(f"原始图像尺寸: {img.shape}")

        # 预处理图像 - 使用letterbox而不是简单缩放，保持宽高比
        perf_monitor.start("preprocessing")
        img_for_detection, ratio, pad = letterbox(img, new_shape=(IMG_SIZE, IMG_SIZE))
        logger.debug(f"预处理后图像尺寸: {img_for_detection.shape}, 缩放比例: {ratio}, 填充: {pad}")

        # 转换颜色空间从BGR到RGB（RKNN模型通常使用RGB输入）
        img_for_detection = cv.cvtColor(img_for_detection, cv.COLOR_BGR2RGB)

        # 扩展维度以匹配模型输入要求
        img_for_detection = np.expand_dims(img_for_detection, 0)
        logger.debug(f"模型输入尺寸: {img_for_detection.shape}")
        perf_monitor.end("preprocessing")
        preprocess_stats = perf_monitor.get_stats("preprocessing")
        if preprocess_stats:
            logger.debug(f"预处理耗时: {preprocess_stats['avg']:.2f}ms")

        # 运行RKNN推理
        logger.debug('开始RKNN推理...')
        perf_monitor.start("inference")
        outputs = model.inference(inputs=[img_for_detection])
        perf_monitor.end("inference")
        inference_stats = perf_monitor.get_stats("inference")
        if inference_stats:
            logger.debug(f"RKNN推理完成，耗时: {inference_stats['avg']:.2f}ms")

        # 处理RKNN输出
        perf_monitor.start("postprocessing")
        input0_data = outputs[0]
        input1_data = outputs[1]
        input2_data = outputs[2]

        logger.debug(f"输出特征图形状: {input0_data.shape}, {input1_data.shape}, {input2_data.shape}")

        # 重塑输出以匹配YOLOv5后处理期望格式
        input0_data = input0_data.reshape([3, -1] + list(input0_data.shape[-2:]))
        input1_data = input1_data.reshape([3, -1] + list(input1_data.shape[-2:]))
        input2_data = input2_data.reshape([3, -1] + list(input2_data.shape[-2:]))

        # 转置以匹配YOLOv5后处理期望格式
        input_data = [
            np.transpose(input0_data, (2, 3, 0, 1)),
            np.transpose(input1_data, (2, 3, 0, 1)),
            np.transpose(input2_data, (2, 3, 0, 1))
        ]

        # YOLOv5后处理
        logger.debug('正在执行YOLOv5后处理...')
        boxes, classes, scores = yolov5_post_process(input_data)

        # 将坐标从letterbox空间转回原始图像空间
        if boxes is not None:
            # 获取原始图像尺寸
            orig_h, orig_w = original_img.shape[:2]

            # 获取缩放比例和填充信息
            r = ratio[0]  # 宽高使用相同的缩放比例
            dw, dh = pad  # 填充信息

            # 调整所有边界框坐标
            for i in range(len(boxes)):
                # 去除填充偏移并按比例缩放
                boxes[i][0] = (boxes[i][0] - dw) / r  # xmin
                boxes[i][1] = (boxes[i][1] - dh) / r  # ymin
                boxes[i][2] = (boxes[i][2] - dw) / r  # xmax
                boxes[i][3] = (boxes[i][3] - dh) / r  # ymax

                # 确保坐标在有效范围内
                boxes[i][0] = max(0, min(orig_w - 1, boxes[i][0]))
                boxes[i][1] = max(0, min(orig_h - 1, boxes[i][1]))
                boxes[i][2] = max(0, min(orig_w, boxes[i][2]))
                boxes[i][3] = max(0, min(orig_h, boxes[i][3]))

        perf_monitor.end("postprocessing")
        postprocess_stats = perf_monitor.get_stats("postprocessing")
        if postprocess_stats:
            logger.debug(f"后处理耗时: {postprocess_stats['avg']:.2f}ms")

        # 初始化结果容器
        upper_clothing = []
        lower_clothing = []
        pairs = []

        # 如果没有检测到任何物体
        if boxes is None:
            logger.debug("未检测到任何服装")
            return pairs

        logger.debug(f"检测到 {len(boxes)} 个物体，开始分类和处理...")

        # 处理检测结果
        for i, (box, score, cls) in enumerate(zip(boxes, scores, classes)):
            # 获取坐标
            xmin, ymin, xmax, ymax = box
            height, width = original_img.shape[:2]  # 获取当前图像尺寸
            xmin = max(0, min(width - 1, int(xmin)))
            ymin = max(0, min(height - 1, int(ymin)))
            xmax = max(0, min(width, int(xmax)))
            ymax = max(0, min(height, int(ymax)))

            confidence = score
            class_name = CLASSES[cls]

            logger.debug(
                f"物体 {i + 1}: 类别={class_name}, 置信度={confidence:.4f}, 坐标=[{xmin}, {ymin}, {xmax}, {ymax}]")

            # 在图像上添加标签
            label = f"{class_name} {confidence:.2f}"
            cv.putText(
                original_img, label, (xmin, ymin - 10),
                cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2
            )

            # 分类检测结果
            if class_name in CLOTHING_CATEGORIES['upper']:
                upper_clothing.append((xmin, ymin, xmax, ymax, confidence, 'upper'))
                logger.debug(f"将 {class_name} 分类为上衣")
            elif class_name in CLOTHING_CATEGORIES['lower']:
                lower_clothing.append((xmin, ymin, xmax, ymax, confidence, 'lower'))
                logger.debug(f"将 {class_name} 分类为下装")

        # 应用NMS - 注意保留置信度信息
        logger.debug(f"NMS前上衣数量: {len(upper_clothing)}, 下装数量: {len(lower_clothing)}")
        upper_clothing = apply_nms(upper_clothing)
        lower_clothing = apply_nms(lower_clothing)
        logger.debug(f"NMS后上衣数量: {len(upper_clothing)}, 下装数量: {len(lower_clothing)}")

        # 匹配服装项目 - 使用新的匹配函数，保留置信度
        logger.debug("开始匹配上下装...")
        match_start = time.time()
        match_clothing_items_with_confidence(upper_clothing, lower_clothing, original_img, pairs)
        logger.debug(f"匹配完成，找到 {len(pairs)} 对服装，耗时: {(time.time() - match_start) * 1000:.2f}ms")

        # 获取颜色 - 使用新的颜色识别函数，同时保留置信度
        logger.debug("开始识别服装颜色...")
        color_start = time.time()
        identify_clothing_colors_with_confidence(pairs, original_img)
        logger.debug(f"颜色识别完成，耗时: {(time.time() - color_start) * 1000:.2f}ms")

        # 使用原始图像进行后续操作
        img = original_img

        # 汇总检测结果
        for i, pair in enumerate(pairs):
            upper_info = f"上衣: {pair[0]}" if len(pair[0]) > 1 else "未检测到上衣"
            lower_info = f"下装: {pair[1]}" if len(pair[1]) > 1 else "未检测到下装"
            color_info = ""
            if len(pair) > 2 and pair[2]:
                color_info += f", 上衣颜色: {pair[2]}"
            if len(pair) > 3 and pair[3]:
                color_info += f", 下装颜色: {pair[3]}"
            conf_info = ""
            if len(pair) > 4:
                conf_info += f", 上衣置信度: {pair[4]:.4f}"
            if len(pair) > 5:
                conf_info += f", 下装置信度: {pair[5]:.4f}"
            logger.debug(f"服装组合 {i + 1}: {upper_info}, {lower_info}{color_info}{conf_info}")

        # 计算总处理时间
        total_time = time.time() - detect_start_time
        logger.debug(f"检测完成，总耗时: {total_time * 1000:.2f}ms, 约 {1 / total_time:.1f} FPS")

        return pairs

    except Exception as e:
        logger.error(f"图像检测出错: {str(e)}")
        traceback.print_exc()
        return []


def process_video(video_path, output_path=None, show_result=True, save_result=False):
    """
    处理视频文件，检测和标记服装

    参数:
        video_path (str): 输入视频的路径
        output_path (str, optional): 输出视频的路径
        show_result (bool): 是否实时显示处理结果
        save_result (bool): 是否保存处理结果

    返回值:
        bool: 处理是否成功
    """
    try:
        # 打开视频文件
        cap = cv.VideoCapture(video_path)
        if not cap.isOpened():
            logger.error(f"无法打开视频: {video_path}")
            return False

        # 获取视频属性
        frame_width = int(cap.get(cv.CAP_PROP_FRAME_WIDTH))
        frame_height = int(cap.get(cv.CAP_PROP_FRAME_HEIGHT))
        fps = cap.get(cv.CAP_PROP_FPS)
        total_frames = int(cap.get(cv.CAP_PROP_FRAME_COUNT))

        logger.info(f"视频信息 - 分辨率: {frame_width}x{frame_height}, FPS: {fps:.1f}, 总帧数: {total_frames}")

        # 设置输出视频路径（如果未提供）
        if save_result and not output_path:
            file_name = os.path.basename(video_path)
            file_base, file_ext = os.path.splitext(file_name)
            output_path = f"{file_base}_processed.mp4"
            logger.info(f"未指定输出路径，将使用: {output_path}")

        # 视频写入器
        video_writer = None
        if save_result:
            fourcc = cv.VideoWriter_fourcc(*'mp4v')  # 或其他编解码器，如 'XVID'
            video_writer = cv.VideoWriter(output_path, fourcc, fps, (frame_width, frame_height))
            logger.info(f"将保存处理结果到: {output_path}")

        # 处理进度
        processed_frames = 0
        processing_start_time = time.time()

        # 显示调试信息的字体设置
        font = cv.FONT_HERSHEY_SIMPLEX
        font_scale = 0.6
        font_thickness = 2

        # 颜色定义
        upper_color = (255, 0, 0)  # 蓝色 (BGR)
        lower_color = (0, 255, 0)  # 绿色 (BGR)
        person_color = (255, 0, 255)  # 紫色 (BGR)
        info_color = (255, 255, 255)  # 白色 (BGR)

        # 处理每一帧
        while True:
            ret, frame = cap.read()
            if not ret:
                logger.info("视频帧读取结束")
                break  # 视频结束

            frame = cv.resize(frame,(640,640))

            # 创建帧的副本用于显示和保存
            frame_with_annotations = frame.copy()

            # 记录帧处理开始时间
            frame_start_time = time.time()

            # 使用带置信度的检测函数处理帧
            pairs = detect_picture_with_confidence(frame)

            # 计算处理时间
            frame_process_time = (time.time() - frame_start_time) * 1000

            # 在帧上标记检测结果 - 使用不同颜色区分上衣和下装
            for pair in pairs:
                upper_coords = pair[0]
                lower_coords = pair[1]

                # 如果检测到上衣（坐标长度为4），用蓝色框标记
                if len(upper_coords) == 4:
                    cv.rectangle(frame_with_annotations,
                                 (upper_coords[0], upper_coords[1]),
                                 (upper_coords[2], upper_coords[3]),
                                 upper_color,  # 蓝色标记上衣
                                 2)

                    # 添加上衣标签
                    upper_label = "Upper"
                    if len(pair) > 4:  # 如果有置信度信息
                        upper_conf = pair[4]
                        upper_label += f": {upper_conf:.2f}"

                    cv.putText(frame_with_annotations,
                               upper_label,
                               (upper_coords[0], upper_coords[1] - 10),
                               font, font_scale, upper_color, font_thickness)

                # 如果检测到下装（坐标长度为4），用绿色框标记
                if len(lower_coords) == 4:
                    cv.rectangle(frame_with_annotations,
                                 (lower_coords[0], lower_coords[1]),
                                 (lower_coords[2], lower_coords[3]),
                                 lower_color,  # 绿色标记下装
                                 2)

                    # 添加下装标签
                    lower_label = "Lower"
                    if len(pair) > 5:  # 如果有置信度信息
                        lower_conf = pair[5]
                        lower_label += f": {lower_conf:.2f}"

                    cv.putText(frame_with_annotations,
                               lower_label,
                               (lower_coords[0], lower_coords[1] - 10),
                               font, font_scale, lower_color, font_thickness)

                # 绘制全身区域 - 使用紫色框
                person_positions = Determine_the_position_of_the_entire_body(upper_coords, lower_coords, frame)
                for pos in person_positions:
                    if len(pos) == 4:
                        cv.rectangle(frame_with_annotations,
                                     (pos[0], pos[1]),
                                     (pos[2], pos[3]),
                                     person_color,  # 紫色标记整个人
                                     1)

            # 在帧上添加处理信息
            processed_frames += 1
            elapsed_time = time.time() - processing_start_time
            avg_fps = processed_frames / elapsed_time if elapsed_time > 0 else 0
            progress = processed_frames / total_frames * 100 if total_frames > 0 else 0

            # 在帧的顶部添加信息
            info_text = f"Progress: {progress:.1f}%, FPS: {avg_fps:.1f}, Process time: {frame_process_time:.1f}ms"
            cv.putText(frame_with_annotations, info_text,
                       (10, 30), font, font_scale, info_color, font_thickness)

            # 显示处理后的帧
            if show_result:
                cv.imshow('Video Processing', frame_with_annotations)

                # 按ESC或q键退出, 空格键暂停/继续
                key = cv.waitKey(1) & 0xFF
                if key == 27 or key == ord('q'):
                    logger.info("用户中断处理")
                    break
                elif key == 32:  # 空格键
                    logger.info("处理暂停，按任意键继续...")
                    cv.waitKey(0)

            # 保存处理后的帧
            if video_writer is not None:
                video_writer.write(frame_with_annotations)

            # 每处理100帧或是总帧数的整百分比时记录一次进度
            if processed_frames % 100 == 0 or processed_frames == total_frames:
                logger.info(f"已处理 {processed_frames}/{total_frames} 帧 ({progress:.1f}%), 平均 {avg_fps:.1f} FPS")

        # 清理资源
        cap.release()
        if video_writer is not None:
            video_writer.release()
        if show_result:
            cv.destroyAllWindows()

        # 打印处理统计信息
        total_time = time.time() - processing_start_time
        logger.info(
            f"视频处理完成，总时间: {total_time:.2f}秒, 处理了 {processed_frames} 帧, 平均FPS: {processed_frames / total_time:.1f}")

        return True

    except Exception as e:
        logger.error(f"处理视频时出错: {str(e)}")
        traceback.print_exc()
        return False


def analyze_clothing_group(upper_coords, lower_coords, img, show_debug=False):
    """
    分析上衣和下装是否属于同一个人体，返回匹配可能性分数

    参数:
        upper_coords (tuple): 上衣坐标 (xmin, ymin, xmax, ymax)
        lower_coords (tuple): 下装坐标 (xmin, ymin, xmax, ymax)
        img (numpy.ndarray): 输入图像
        show_debug (bool): 是否显示调试信息

    返回值:
        float: 匹配可能性分数 (0.0-1.0)，越高越可能属于同一个人
    """
    # 如果任一项为占位符，返回零分
    if len(upper_coords) == 1 or len(lower_coords) == 1:
        return 0.0

    try:
        height, width = img.shape[:2]

        # 提取坐标
        ux1, uy1, ux2, uy2 = upper_coords
        lx1, ly1, lx2, ly2 = lower_coords

        # 计算中心点
        upper_center_x = (ux1 + ux2) // 2
        upper_center_y = (uy1 + uy2) // 2
        lower_center_x = (lx1 + lx2) // 2
        lower_center_y = (ly1 + ly2) // 2

        # 计算尺寸
        upper_width = ux2 - ux1
        upper_height = uy2 - uy1
        lower_width = lx2 - lx1
        lower_height = ly2 - ly1

        # 特征1: 水平位置关系
        x_distance = abs(upper_center_x - lower_center_x)
        x_distance_norm = x_distance / width  # 归一化为0-1
        x_score = max(0, 1.0 - x_distance_norm * 5.0)  # 水平距离越近越好

        # 特征2: 垂直位置关系
        y_distance = lower_center_y - upper_center_y
        if y_distance <= 0:  # 下装中心必须在上衣中心下方
            y_score = 0.0
        else:
            expected_y_distance = (upper_height + lower_height) * 0.5  # 期望的垂直距离
            y_ratio = min(y_distance, expected_y_distance) / max(y_distance, expected_y_distance)
            y_score = y_ratio

        # 特征3: 宽度比例
        width_ratio = min(upper_width, lower_width) / max(upper_width, lower_width)
        width_score = width_ratio

        # 特征4: 垂直连接性 - 上衣底部和下装顶部
        vertical_gap = ly1 - uy2
        if vertical_gap > upper_height * 0.5:  # 间隙过大
            connection_score = 0.0
        elif vertical_gap < -upper_height * 0.3:  # 重叠过多
            connection_score = 0.0
        else:
            # 理想情况是轻微重叠或小间隙
            connection_score = 1.0 - abs(vertical_gap) / (upper_height * 0.5)
            connection_score = max(0, connection_score)

        # 特征5: 水平重叠度
        horizontal_overlap = max(0, min(ux2, lx2) - max(ux1, lx1))
        horizontal_union = max(ux2, lx2) - min(ux1, lx1)
        horizontal_iou = horizontal_overlap / horizontal_union if horizontal_union > 0 else 0
        overlap_score = horizontal_iou

        # 计算最终得分 - 加权平均
        weights = [0.25, 0.25, 0.15, 0.2, 0.15]  # 各特征权重
        scores = [x_score, y_score, width_score, connection_score, overlap_score]
        final_score = sum(w * s for w, s in zip(weights, scores))

        # 如果开启调试模式，显示详细分数
        if show_debug:
            logger.debug(f"水平位置分数: {x_score:.3f}")
            logger.debug(f"垂直位置分数: {y_score:.3f}")
            logger.debug(f"宽度比例分数: {width_score:.3f}")
            logger.debug(f"垂直连接分数: {connection_score:.3f}")
            logger.debug(f"水平重叠分数: {overlap_score:.3f}")
            logger.debug(f"最终匹配分数: {final_score:.3f}")

        return final_score

    except Exception as e:
        logger.error(f"分析服装组合时出错: {str(e)}")
        return 0.0


def improved_match_clothing_items_with_confidence(upper_items, lower_items, img, pairs, max_x_distance_ratio=None):
    """
    改进的上衣和下装匹配算法，适用于多人场景，并支持置信度信息

    参数:
        upper_items (list): 上衣列表，格式为 [(xmin, ymin, xmax, ymax, confidence, 'upper'), ...]
        lower_items (list): 下装列表，格式为 [(xmin, ymin, xmax, ymax, confidence, 'lower'), ...]
        img (numpy.ndarray): 输入图像
        pairs (list): 输出匹配对列表
        max_x_distance_ratio (float, optional): 最大水平距离比例
    """
    if not upper_items and not lower_items:
        return

    # 提取坐标和置信度信息
    upper_coords = [(item[0], item[1], item[2], item[3]) for item in upper_items]
    upper_confidences = [item[4] for item in upper_items]

    lower_coords = [(item[0], item[1], item[2], item[3]) for item in lower_items]
    lower_confidences = [item[4] for item in lower_items]

    # 计算所有可能的配对得分
    pairing_scores = []

    for upper_idx, upper in enumerate(upper_coords):
        for lower_idx, lower in enumerate(lower_coords):
            # 分析这对上衣和下装的匹配度
            match_score = analyze_clothing_group(upper, lower, img)

            # 结合检测置信度
            confidence_factor = (upper_confidences[upper_idx] + lower_confidences[lower_idx]) / 2.0

            # 最终得分 (匹配度和置信度的结合)
            final_score = match_score * 0.8 + confidence_factor * 0.2

            # 只有得分超过阈值的才考虑匹配
            if final_score > 0.6:  # 阈值可调整
                pairing_scores.append((upper_idx, lower_idx, final_score))

    # 按得分降序排序
    pairing_scores.sort(key=lambda x: x[2], reverse=True)

    # 贪心匹配 - 优先选择得分最高的配对
    used_uppers = set()
    used_lowers = set()

    # 进行配对
    for upper_idx, lower_idx, score in pairing_scores:
        # 如果上衣和下装都没被使用过
        if upper_idx not in used_uppers and lower_idx not in used_lowers:
            pairs.append([
                upper_coords[upper_idx],
                lower_coords[lower_idx],
                upper_confidences[upper_idx],
                lower_confidences[lower_idx]
            ])
            used_uppers.add(upper_idx)
            used_lowers.add(lower_idx)

    # 添加未匹配的上衣
    for upper_idx, upper in enumerate(upper_coords):
        if upper_idx not in used_uppers:
            pairs.append([upper, (-1,), upper_confidences[upper_idx], 0.0])

    # 添加未匹配的下装
    for lower_idx, lower in enumerate(lower_coords):
        if lower_idx not in used_lowers:
            pairs.append([(-1,), lower, 0.0, lower_confidences[lower_idx]])


def detect_picture_with_improved_matching(img):
    """
    使用RKNN检测图像中的服装，使用改进的匹配算法处理多人场景

    参数:
        img (numpy.ndarray): 输入图像

    返回值:
        list: 检测到的服装配对列表，每对格式为[上衣位置，下装位置，上衣颜色，下装颜色，上衣置信度，下装置信度]
    """
    # 记录检测开始时间(用于性能监控)
    detect_start_time = time.time()

    model = get_model()
    if model is None:
        logger.error("模型未加载，无法进行检测")
        return []

    if img is None or img.size == 0:
        logger.error("提供给detect_picture_with_improved_matching的图像无效")
        return []

    try:
        # 保存原始图像副本 - 移除直接的resize操作
        original_img = img.copy()
        logger.debug(f"原始图像尺寸: {img.shape}")

        # 预处理图像 - 使用letterbox而不是简单缩放，保持宽高比
        perf_monitor.start("preprocessing")
        img_for_detection, ratio, pad = letterbox(img, new_shape=(IMG_SIZE, IMG_SIZE))
        logger.debug(f"预处理后图像尺寸: {img_for_detection.shape}, 缩放比例: {ratio}, 填充: {pad}")

        # 转换颜色空间从BGR到RGB（RKNN模型通常使用RGB输入）
        img_for_detection = cv.cvtColor(img_for_detection, cv.COLOR_BGR2RGB)

        # 扩展维度以匹配模型输入要求
        img_for_detection = np.expand_dims(img_for_detection, 0)
        logger.debug(f"模型输入尺寸: {img_for_detection.shape}")
        perf_monitor.end("preprocessing")
        preprocess_stats = perf_monitor.get_stats("preprocessing")
        if preprocess_stats:
            logger.debug(f"预处理耗时: {preprocess_stats['avg']:.2f}ms")

        # 运行RKNN推理
        logger.debug('开始RKNN推理...')
        perf_monitor.start("inference")
        outputs = model.inference(inputs=[img_for_detection])
        perf_monitor.end("inference")
        inference_stats = perf_monitor.get_stats("inference")
        if inference_stats:
            logger.debug(f"RKNN推理完成，耗时: {inference_stats['avg']:.2f}ms")

        # 处理RKNN输出
        perf_monitor.start("postprocessing")
        input0_data = outputs[0]
        input1_data = outputs[1]
        input2_data = outputs[2]

        logger.debug(f"输出特征图形状: {input0_data.shape}, {input1_data.shape}, {input2_data.shape}")

        # 重塑输出以匹配YOLOv5后处理期望格式
        input0_data = input0_data.reshape([3, -1] + list(input0_data.shape[-2:]))
        input1_data = input1_data.reshape([3, -1] + list(input1_data.shape[-2:]))
        input2_data = input2_data.reshape([3, -1] + list(input2_data.shape[-2:]))

        # 转置以匹配YOLOv5后处理期望格式
        input_data = [
            np.transpose(input0_data, (2, 3, 0, 1)),
            np.transpose(input1_data, (2, 3, 0, 1)),
            np.transpose(input2_data, (2, 3, 0, 1))
        ]

        # YOLOv5后处理
        logger.debug('正在执行YOLOv5后处理...')
        boxes, classes, scores = yolov5_post_process(input_data)

        # 将坐标从letterbox空间转回原始图像空间
        if boxes is not None:
            # 获取原始图像尺寸
            orig_h, orig_w = original_img.shape[:2]

            # 获取缩放比例和填充信息
            r = ratio[0]  # 宽高使用相同的缩放比例
            dw, dh = pad  # 填充信息

            # 调整所有边界框坐标
            for i in range(len(boxes)):
                # 去除填充偏移并按比例缩放
                boxes[i][0] = (boxes[i][0] - dw) / r  # xmin
                boxes[i][1] = (boxes[i][1] - dh) / r  # ymin
                boxes[i][2] = (boxes[i][2] - dw) / r  # xmax
                boxes[i][3] = (boxes[i][3] - dh) / r  # ymax

                # 确保坐标在有效范围内
                boxes[i][0] = max(0, min(orig_w - 1, boxes[i][0]))
                boxes[i][1] = max(0, min(orig_h - 1, boxes[i][1]))
                boxes[i][2] = max(0, min(orig_w, boxes[i][2]))
                boxes[i][3] = max(0, min(orig_h, boxes[i][3]))

        perf_monitor.end("postprocessing")
        postprocess_stats = perf_monitor.get_stats("postprocessing")
        if postprocess_stats:
            logger.debug(f"后处理耗时: {postprocess_stats['avg']:.2f}ms")

        # 初始化结果容器
        upper_clothing = []
        lower_clothing = []
        pairs = []

        # 如果没有检测到任何物体
        if boxes is None:
            logger.debug("未检测到任何服装")
            return pairs

        logger.debug(f"检测到 {len(boxes)} 个物体，开始分类和处理...")

        # 处理检测结果
        for i, (box, score, cls) in enumerate(zip(boxes, scores, classes)):
            # 获取坐标
            xmin, ymin, xmax, ymax = box
            xmin = int(xmin)
            ymin = int(ymin)
            xmax = int(xmax)
            ymax = int(ymax)
            confidence = score
            class_name = CLASSES[cls]

            logger.debug(
                f"物体 {i + 1}: 类别={class_name}, 置信度={confidence:.4f}, 坐标=[{xmin}, {ymin}, {xmax}, {ymax}]")

            # 在图像上添加标签
            label = f"{class_name} {confidence:.2f}"
            cv.putText(
                original_img, label, (xmin, ymin - 10),
                cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2
            )

            # 分类检测结果
            if class_name in CLOTHING_CATEGORIES['upper']:
                upper_clothing.append((xmin, ymin, xmax, ymax, confidence, 'upper'))
                logger.debug(f"将 {class_name} 分类为上衣")
            elif class_name in CLOTHING_CATEGORIES['lower']:
                lower_clothing.append((xmin, ymin, xmax, ymax, confidence, 'lower'))
                logger.debug(f"将 {class_name} 分类为下装")

        # 应用NMS - 注意保留置信度信息
        logger.debug(f"NMS前上衣数量: {len(upper_clothing)}, 下装数量: {len(lower_clothing)}")
        upper_clothing = apply_nms(upper_clothing)
        lower_clothing = apply_nms(lower_clothing)
        logger.debug(f"NMS后上衣数量: {len(upper_clothing)}, 下装数量: {len(lower_clothing)}")

        # 使用改进的匹配算法
        logger.debug("开始使用改进的算法匹配上下装...")
        match_start = time.time()
        improved_match_clothing_items_with_confidence(upper_clothing, lower_clothing, original_img, pairs)
        logger.debug(f"匹配完成，找到 {len(pairs)} 对服装，耗时: {(time.time() - match_start) * 1000:.2f}ms")

        # 获取颜色 - 使用新的颜色识别函数，同时保留置信度
        logger.debug("开始识别服装颜色...")
        color_start = time.time()
        identify_clothing_colors_with_confidence(pairs, original_img)
        logger.debug(f"颜色识别完成，耗时: {(time.time() - color_start) * 1000:.2f}ms")

        # 使用原始图像进行后续操作
        img = original_img

        # 汇总检测结果
        for i, pair in enumerate(pairs):
            upper_info = f"上衣: {pair[0]}" if len(pair[0]) > 1 else "未检测到上衣"
            lower_info = f"下装: {pair[1]}" if len(pair[1]) > 1 else "未检测到下装"
            color_info = ""
            if len(pair) > 2 and pair[2]:
                color_info += f", 上衣颜色: {pair[2]}"
            if len(pair) > 3 and pair[3]:
                color_info += f", 下装颜色: {pair[3]}"
            conf_info = ""
            if len(pair) > 4:
                conf_info += f", 上衣置信度: {pair[4]:.4f}"
            if len(pair) > 5:
                conf_info += f", 下装置信度: {pair[5]:.4f}"
            logger.debug(f"服装组合 {i + 1}: {upper_info}, {lower_info}{color_info}{conf_info}")

        # 计算总处理时间
        total_time = time.time() - detect_start_time
        logger.debug(f"检测完成，总耗时: {total_time * 1000:.2f}ms, 约 {1 / total_time:.1f} FPS")

        return pairs

    except Exception as e:
        logger.error(f"图像检测出错: {str(e)}")
        traceback.print_exc()
        return []


# 更新视频处理函数以使用改进的匹配算法
def process_video_with_improved_matching(video_path, output_path=None, show_result=True, save_result=False):
    """
    使用改进的匹配算法处理视频文件，检测和标记服装

    参数:
        video_path (str): 输入视频的路径
        output_path (str, optional): 输出视频的路径
        show_result (bool): 是否实时显示处理结果
        save_result (bool): 是否保存处理结果

    返回值:
        bool: 处理是否成功
    """
    try:
        # 打开视频文件
        cap = cv.VideoCapture(video_path)
        if not cap.isOpened():
            logger.error(f"无法打开视频: {video_path}")
            return False

        # 获取视频属性
        frame_width = int(cap.get(cv.CAP_PROP_FRAME_WIDTH))
        frame_height = int(cap.get(cv.CAP_PROP_FRAME_HEIGHT))
        fps = cap.get(cv.CAP_PROP_FPS)
        total_frames = int(cap.get(cv.CAP_PROP_FRAME_COUNT))

        logger.info(f"视频信息 - 分辨率: {frame_width}x{frame_height}, FPS: {fps:.1f}, 总帧数: {total_frames}")

        # 设置输出视频路径（如果未提供）
        if save_result and not output_path:
            file_name = os.path.basename(video_path)
            file_base, file_ext = os.path.splitext(file_name)
            output_path = f"{file_base}_processed.mp4"
            logger.info(f"未指定输出路径，将使用: {output_path}")

        # 视频写入器
        video_writer = None
        if save_result:
            fourcc = cv.VideoWriter_fourcc(*'mp4v')  # 或其他编解码器，如 'XVID'
            video_writer = cv.VideoWriter(output_path, fourcc, fps, (frame_width, frame_height))
            logger.info(f"将保存处理结果到: {output_path}")

        # 处理进度
        processed_frames = 0
        processing_start_time = time.time()

        # 显示调试信息的字体设置
        font = cv.FONT_HERSHEY_SIMPLEX
        font_scale = 0.6
        font_thickness = 2

        # 颜色定义
        upper_color = (255, 0, 0)  # 蓝色 (BGR)
        lower_color = (0, 255, 0)  # 绿色 (BGR)
        person_color = (255, 0, 255)  # 紫色 (BGR)
        info_color = (255, 255, 255)  # 白色 (BGR)

        # 处理每一帧
        while True:
            ret, frame = cap.read()
            if not ret:
                logger.info("视频帧读取结束")
                break  # 视频结束
            frame = cv.resize(frame,(640,640))

            # 创建帧的副本用于显示和保存
            frame_with_annotations = frame.copy()

            # 记录帧处理开始时间
            frame_start_time = time.time()

            # 使用改进的匹配算法处理帧
            pairs = detect_picture_with_improved_matching(frame)

            # 计算处理时间
            frame_process_time = (time.time() - frame_start_time) * 1000

            # 在帧上标记检测结果 - 使用不同颜色区分上衣和下装
            for pair in pairs:
                upper_coords = pair[0]
                lower_coords = pair[1]

                # 如果检测到上衣（坐标长度为4），用蓝色框标记
                if len(upper_coords) == 4:
                    cv.rectangle(frame_with_annotations,
                                 (upper_coords[0], upper_coords[1]),
                                 (upper_coords[2], upper_coords[3]),
                                 upper_color,  # 蓝色标记上衣
                                 2)

                    # 添加上衣标签
                    upper_label = "Upper"
                    if len(pair) > 4:  # 如果有置信度信息
                        upper_conf = pair[4]
                        upper_label += f": {upper_conf:.2f}"

                    cv.putText(frame_with_annotations,
                               upper_label,
                               (upper_coords[0], upper_coords[1] - 10),
                               font, font_scale, upper_color, font_thickness)

                # 如果检测到下装（坐标长度为4），用绿色框标记
                if len(lower_coords) == 4:
                    cv.rectangle(frame_with_annotations,
                                 (lower_coords[0], lower_coords[1]),
                                 (lower_coords[2], lower_coords[3]),
                                 lower_color,  # 绿色标记下装
                                 2)

                    # 添加下装标签
                    lower_label = "Lower"
                    if len(pair) > 5:  # 如果有置信度信息
                        lower_conf = pair[5]
                        lower_label += f": {lower_conf:.2f}"

                    cv.putText(frame_with_annotations,
                               lower_label,
                               (lower_coords[0], lower_coords[1] - 10),
                               font, font_scale, lower_color, font_thickness)

                # 绘制全身区域 - 使用紫色框
                person_positions = Determine_the_position_of_the_entire_body(upper_coords, lower_coords, frame)
                for pos in person_positions:
                    if len(pos) == 4:
                        cv.rectangle(frame_with_annotations,
                                     (pos[0], pos[1]),
                                     (pos[2], pos[3]),
                                     person_color,  # 紫色标记整个人
                                     1)

            # 在帧上添加处理信息
            processed_frames += 1
            elapsed_time = time.time() - processing_start_time
            avg_fps = processed_frames / elapsed_time if elapsed_time > 0 else 0
            progress = processed_frames / total_frames * 100 if total_frames > 0 else 0

            # 在帧的顶部添加信息
            info_text = f"Progress: {progress:.1f}%, FPS: {avg_fps:.1f}, Process time: {frame_process_time:.1f}ms"
            cv.putText(frame_with_annotations, info_text,
                       (10, 30), font, font_scale, info_color, font_thickness)

            # 显示处理后的帧
            if show_result:
                cv.imshow('Video Processing', frame_with_annotations)

                # 按ESC或q键退出, 空格键暂停/继续
                key = cv.waitKey(1) & 0xFF
                if key == 27 or key == ord('q'):
                    logger.info("用户中断处理")
                    break
                elif key == 32:  # 空格键
                    logger.info("处理暂停，按任意键继续...")
                    cv.waitKey(0)

            # 保存处理后的帧
            if video_writer is not None:
                video_writer.write(frame_with_annotations)

            # 每处理100帧或是总帧数的整百分比时记录一次进度
            if processed_frames % 100 == 0 or processed_frames == total_frames:
                logger.info(f"已处理 {processed_frames}/{total_frames} 帧 ({progress:.1f}%), 平均 {avg_fps:.1f} FPS")

        # 清理资源
        cap.release()
        if video_writer is not None:
            video_writer.release()
        if show_result:
            cv.destroyAllWindows()

        # 打印处理统计信息
        total_time = time.time() - processing_start_time
        logger.info(
            f"视频处理完成，总时间: {total_time:.2f}秒, 处理了 {processed_frames} 帧, 平均FPS: {processed_frames / total_time:.1f}")

        return True

    except Exception as e:
        logger.error(f"处理视频时出错: {str(e)}")
        traceback.print_exc()
        return False


def isolate_clothing_region(clothing_box, img):
    """使用GrabCut算法分离服装和背景"""
    xmin, ymin, xmax, ymax = clothing_box
    clothing_region = img[ymin:ymax, xmin:xmax]

    # 创建初始掩码
    mask = np.zeros(clothing_region.shape[:2], np.uint8)

    # 定义可能的前景区域（比边界框略小）
    margin = int(min(clothing_region.shape[0], clothing_region.shape[1]) * 0.1)
    mask[margin:-margin, margin:-margin] = cv.GC_PR_FGD  # 可能前景
    mask[0:margin, :] = cv.GC_PR_BGD  # 可能背景（边缘）
    mask[-margin:, :] = cv.GC_PR_BGD
    mask[:, 0:margin] = cv.GC_PR_BGD
    mask[:, -margin:] = cv.GC_PR_BGD

    # GrabCut参数
    bgd_model = np.zeros((1, 65), np.float64)
    fgd_model = np.zeros((1, 65), np.float64)

    # 应用GrabCut
    cv.grabCut(clothing_region, mask, None, bgd_model, fgd_model, 5, cv.GC_INIT_WITH_MASK)

    # 创建二值掩码
    mask2 = np.where((mask == cv.GC_PR_FGD) | (mask == cv.GC_FGD), 255, 0).astype('uint8')

    # 应用掩码
    isolated_clothing = cv.bitwise_and(clothing_region, clothing_region, mask=mask2)

    return mask2, isolated_clothing

def isolate_clothing_by_color(clothing_box, img):
    """使用颜色阈值分离服装"""
    xmin, ymin, xmax, ymax = clothing_box
    clothing_region = img[ymin:ymax, xmin:xmax]

    # 转换到HSV颜色空间
    hsv = cv.cvtColor(clothing_region, cv.COLOR_BGR2HSV)

    # 计算颜色直方图找到主要颜色
    h_hist = cv.calcHist([hsv], [0], None, [180], [0, 180])

    # 找到主要色调
    dominant_h = np.argmax(h_hist)

    # 为主要色调创建掩码（带容差）
    h_tolerance = 15  # 根据颜色变化调整
    lower_h = max(0, dominant_h - h_tolerance)
    upper_h = min(180, dominant_h + h_tolerance)

    # 创建掩码 - 使用NumPy数组而不是元组
    lower_bound = np.array([lower_h, 30, 30], dtype=np.uint8)
    upper_bound = np.array([upper_h, 255, 255], dtype=np.uint8)
    mask = cv.inRange(hsv, lower_bound, upper_bound)

    # 应用形态学操作优化掩码
    kernel = np.ones((5, 5), np.uint8)
    mask = cv.morphologyEx(mask, cv.MORPH_OPEN, kernel)
    mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, kernel)

    # 应用掩码
    isolated_clothing = cv.bitwise_and(clothing_region, clothing_region, mask=mask)

    return isolated_clothing


def isolate_clothing_by_edges(clothing_box, img):
    """使用边缘检测分离服装区域"""
    xmin, ymin, xmax, ymax = clothing_box
    clothing_region = img[ymin:ymax, xmin:xmax]

    # 转为灰度图
    gray = cv.cvtColor(clothing_region, cv.COLOR_BGR2GRAY)

    # 高斯模糊减少噪声
    blurred = cv.GaussianBlur(gray, (5, 5), 0)

    # 检测边缘
    edges = cv.Canny(blurred, 30, 100)

    # 膨胀边缘连接间隙
    kernel = np.ones((3, 3), np.uint8)
    dilated_edges = cv.dilate(edges, kernel, iterations=2)

    # 寻找轮廓
    contours, _ = cv.findContours(dilated_edges, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

    # 创建掩码
    mask = np.zeros_like(gray)

    # 找到最大轮廓（假设是服装）
    if contours:
        largest_contour = max(contours, key=cv.contourArea)
        cv.drawContours(mask, [largest_contour], -1, 255, -1)

    # 应用掩码
    isolated_clothing = cv.bitwise_and(clothing_region, clothing_region, mask=mask)

    return isolated_clothing


def get_isolated_clothing(clothing_box, img, method="combined"):
    """
    综合多种方法提取服装区域

    参数:
        clothing_box (tuple): 服装坐标
        img (numpy.ndarray): 输入图像
        method (str): 分割方法 ("grabcut", "color", "edge", "combined")
    """
    xmin, ymin, xmax, ymax = clothing_box
    clothing_region = img[ymin:ymax, xmin:xmax]

    try:
        if method == "combined":
            # 首先尝试GrabCut
            mask1, isolated1 = isolate_clothing_region(clothing_box, img)

            # 检查GrabCut结果是否合理
            if np.count_nonzero(mask1) < 0.1 * mask1.size:
                # 如果前景不足，尝试基于颜色的方法
                isolated2 = isolate_clothing_by_color(clothing_box, img)

                # 结合结果
                mask1_normalized = mask1 / 255.0
                combined = cv.addWeighted(isolated1, 0.7, isolated2, 0.3, 0)
                return combined
            else:
                return isolated1
        elif method == "grabcut":
            _, isolated = isolate_clothing_region(clothing_box, img)
            return isolated
        elif method == "color":
            return isolate_clothing_by_color(clothing_box, img)
        elif method == "edge":
            return isolate_clothing_by_edges(clothing_box, img)
        else:
            return clothing_region
    except Exception as e:
        logger.warning(f"服装分离失败: {str(e)}，使用原始区域")
        return clothing_region


if __name__ == '__main__':
    try:
        # 导入命令行参数解析库
        import argparse

        # 设置命令行参数解析
        parser = argparse.ArgumentParser(description='服装检测与颜色识别程序')

        # 输入类型组 - 互斥参数，只能指定一种输入类型
        input_group = parser.add_mutually_exclusive_group()
        input_group.add_argument('-i', '--image', type=str, help='要处理的单张图像路径',default='/home/monster/下载/1.png')
        input_group.add_argument('-v', '--video', type=str, help='要处理的视频文件路径')
        input_group.add_argument('-c', '--camera', action='store_true', help='使用摄像头进行实时处理',default=False)

        # 输出选项
        parser.add_argument('-o', '--output', type=str, help='输出文件路径')
        parser.add_argument('-s', '--save', action='store_true', help='保存处理结果')
        parser.add_argument('-n', '--no-display', action='store_true', help='不显示处理结果')
        parser.add_argument('-d', '--debug', action='store_true', help='启用调试模式')
        parser.add_argument('--camera-id', type=int, default=1, help='摄像头ID，默认为0')
        parser.add_argument('--legacy', action='store_true', help='使用传统匹配算法（不使用改进算法）',default=True)

        args = parser.parse_args()

        # 设置更详细的日志记录
        performance_log_interval = 100

        # 根据调试标志设置日志级别
        if args.debug:
            logger.setLevel(logging.DEBUG)
        else:
            logger.setLevel(logging.INFO)

        # 创建文件处理器，将日志写入文件
        file_handler = logging.FileHandler('cloth_detection.log')
        file_handler.setLevel(logging.DEBUG)
        # 创建控制台处理器，显示在控制台
        console_handler = logging.StreamHandler()
        console_handler.setLevel(logging.INFO)
        # 设置日志格式
        formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
        file_handler.setFormatter(formatter)
        console_handler.setFormatter(formatter)
        # 添加处理器到日志记录器
        logger.addHandler(file_handler)
        logger.addHandler(console_handler)

        # 记录系统信息
        logger.info(f"系统信息: Python {sys.version}")
        logger.info(f"OpenCV版本: {cv.__version__}")
        logger.info(f"NumPy版本: {np.__version__}")

        # 决定使用哪个检测函数
        detection_function = detect_picture_with_confidence if args.legacy else detect_picture_with_improved_matching
        logger.info(f"使用{'传统' if args.legacy else '改进'}匹配算法")

        # 预加载模型以减少首次延迟
        logger.info("正在加载模型...")
        model = get_model()
        if model is None:
            logger.error("模型加载失败，无法继续")
            exit(1)
        else:
            logger.info("模型加载成功")

        # 如果指定了图像路径，处理单张图像
        if args.image:
            logger.info(f"处理单张图像: {args.image}")
            # 读取图像
            image = cv.imread(args.image)
            if image is None:
                logger.error(f"无法读取图像: {args.image}")
                exit(1)

            # 处理图像
            pairs = detection_function(image)

            # 在图像上绘制检测结果
            font = cv.FONT_HERSHEY_SIMPLEX
            font_scale = 0.6
            font_thickness = 2

            # 颜色定义
            upper_color = (255, 0, 0)  # 蓝色 (BGR)
            lower_color = (0, 255, 0)  # 绿色 (BGR)
            person_color = (255, 0, 255)  # 紫色 (BGR)

            # 在图像上标记检测结果
            for pair in pairs:
                upper_coords = pair[0]
                lower_coords = pair[1]

                # 如果检测到上衣（坐标长度为4），用蓝色框标记
                if len(upper_coords) == 4:
                    cv.rectangle(image,
                                 (upper_coords[0], upper_coords[1]),
                                 (upper_coords[2], upper_coords[3]),
                                 upper_color,  # 蓝色标记上衣
                                 2)

                    # 添加上衣标签
                    upper_label = "Upper"
                    if len(pair) > 4:  # 如果有置信度信息
                        upper_conf = pair[4]
                        upper_label += f": {upper_conf:.2f}"

                    cv.putText(image,
                               upper_label,
                               (upper_coords[0], upper_coords[1] - 10),
                               font, font_scale, upper_color, font_thickness)

                # 如果检测到下装（坐标长度为4），用绿色框标记
                if len(lower_coords) == 4:
                    cv.rectangle(image,
                                 (lower_coords[0], lower_coords[1]),
                                 (lower_coords[2], lower_coords[3]),
                                 lower_color,  # 绿色标记下装
                                 2)

                    # 添加下装标签
                    lower_label = "Lower"
                    if len(pair) > 5:  # 如果有置信度信息
                        lower_conf = pair[5]
                        lower_label += f": {lower_conf:.2f}"

                    cv.putText(image,
                               lower_label,
                               (lower_coords[0], lower_coords[1] - 10),
                               font, font_scale, lower_color, font_thickness)

                # 绘制全身区域 - 使用紫色框
                person_positions = Determine_the_position_of_the_entire_body(upper_coords, lower_coords, image)
                for pos in person_positions:
                    if len(pos) == 4:
                        cv.rectangle(image,
                                     (pos[0], pos[1]),
                                     (pos[2], pos[3]),
                                     person_color,  # 紫色标记整个人
                                     1)

            # 显示处理结果
            if not args.no_display:
                cv.imshow('Processing Result', image)
                logger.info("按任意键继续...")
                cv.waitKey(0)
                cv.destroyAllWindows()

            # 保存处理结果
            if args.save:
                output_path = args.output if args.output else f"{os.path.splitext(args.image)[0]}_processed.jpg"
                cv.imwrite(output_path, image)
                logger.info(f"已保存处理结果到: {output_path}")

        # 如果指定了视频路径，处理视频
        elif args.video:
            logger.info(f"处理视频: {args.video}")
            if args.legacy:
                process_video(args.video,
                              output_path=args.output,
                              show_result=not args.no_display,
                              save_result=args.save)
            else:
                process_video_with_improved_matching(args.video,
                                                     output_path=args.output,
                                                     show_result=not args.no_display,
                                                     save_result=args.save)

        # 否则使用摄像头进行实时处理
        else:
            logger.info("使用摄像头进行实时处理...")

            # 设置帧率计算变量
            fps_time = time.time()
            fps_count = 0
            fps_display = 0

            # 打开摄像头
            cap = cv.VideoCapture(args.camera_id)  # 使用指定的摄像头ID

            if not cap.isOpened():
                logger.error("无法打开摄像头!")
                exit(1)

            logger.info("摄像头初始化成功，开始处理视频流")

            # 获取摄像头分辨率
            frame_width = int(cap.get(cv.CAP_PROP_FRAME_WIDTH))
            frame_height = int(cap.get(cv.CAP_PROP_FRAME_HEIGHT))
            logger.info(f"摄像头分辨率: {frame_width}x{frame_height}")

            # 视频写入器 - 如果需要保存
            video_writer = None
            if args.save:
                if not args.output:
                    output_path = f"camera_capture_{time.strftime('%Y%m%d_%H%M%S')}.mp4"
                else:
                    output_path = args.output

                fourcc = cv.VideoWriter_fourcc(*'mp4v')
                video_writer = cv.VideoWriter(output_path, fourcc, 30.0, (frame_width, frame_height))
                logger.info(f"将保存视频流到: {output_path}")

            # 显示调试信息的字体设置
            font = cv.FONT_HERSHEY_SIMPLEX
            font_scale = 0.6
            font_thickness = 2
            info_color = (255, 255, 255)  # 白色

            # 服装检测框颜色
            upper_color = (255, 0, 0)  # 蓝色 (BGR) - 上衣
            lower_color = (0, 255, 0)  # 绿色 (BGR) - 下装
            person_color = (255, 0, 255)  # 紫色 (BGR) - 全身

            frame_counter = 0

            while True:
                # 捕获帧
                ret, frame = cap.read()
                if not ret:
                    logger.error("无法读取摄像头帧!")
                    break

                # 创建帧的副本用于显示和保存
                display_frame = frame.copy()

                # 计算FPS
                fps_count += 1
                if (time.time() - fps_time) > 1.0:
                    fps_display = fps_count / (time.time() - fps_time)
                    fps_count = 0
                    fps_time = time.time()

                # 显示FPS
                cv.putText(display_frame, f"FPS: {fps_display:.1f}",
                           (10, 30), font, font_scale, info_color, font_thickness)

                # 记录处理开始时间（用于性能分析）
                process_start_time = time.time()

                try:
                    # 增加帧计数器
                    frame_counter += 1

                    # 监控整体处理性能
                    perf_monitor.start("frame_processing")

                    # 检测和处理图像
                    logger.debug("开始处理当前帧...")
                    pairs = detection_function(frame)

                    # 结束性能监控
                    perf_monitor.end("frame_processing")

                    # 每隔一定帧数记录性能统计
                    if frame_counter % performance_log_interval == 0:
                        logger.info(f"已处理 {frame_counter} 帧")
                        perf_monitor.log_stats()

                    # 显示处理时间
                    process_time = (time.time() - process_start_time) * 1000
                    cv.putText(display_frame, f"Process Time: {process_time:.1f}ms",
                               (10, 60), font, font_scale, info_color, font_thickness)

                    # 获取并显示平均处理时间
                    avg_stats = perf_monitor.get_stats("frame_processing")
                    if avg_stats:
                        cv.putText(display_frame, f"Avg Time: {avg_stats['avg']:.1f}ms",
                                   (10, 90), font, font_scale, info_color, font_thickness)

                    # 显示检测到的服装数量
                    cv.putText(display_frame, f"Detected {len(pairs)} Outfits",
                               (10, 120), font, font_scale, info_color, font_thickness)

                    # 遍历每一对服装，显示详细信息
                    for i, pair in enumerate(pairs):
                        upper_coords = pair[0]
                        lower_coords = pair[1]

                        # 绘制上衣检测框 - 蓝色
                        if len(upper_coords) == 4:
                            cv.rectangle(display_frame,
                                         (upper_coords[0], upper_coords[1]),
                                         (upper_coords[2], upper_coords[3]),
                                         upper_color, 2)

                            # 添加上衣置信度信息
                            if len(pair) > 4:  # 如果有置信度信息
                                upper_conf = pair[4]
                                cv.putText(display_frame,
                                           f"Upper: {upper_conf:.2f}",
                                           (upper_coords[0], upper_coords[1] - 10),
                                           font, font_scale, upper_color, font_thickness)

                        # 绘制下装检测框 - 绿色
                        if len(lower_coords) == 4:
                            cv.rectangle(display_frame,
                                         (lower_coords[0], lower_coords[1]),
                                         (lower_coords[2], lower_coords[3]),
                                         lower_color, 2)

                            # 添加下装置信度信息
                            if len(pair) > 5:  # 如果有置信度信息
                                lower_conf = pair[5]
                                cv.putText(display_frame,
                                           f"Lower: {lower_conf:.2f}",
                                           (lower_coords[0], lower_coords[1] - 10),
                                           font, font_scale, lower_color, font_thickness)

                        # 在全身位置画框 - 紫色
                        person_positions = Determine_the_position_of_the_entire_body(upper_coords, lower_coords, frame)
                        for pos in person_positions:
                            if len(pos) == 4:
                                cv.rectangle(display_frame,
                                             (pos[0], pos[1]),
                                             (pos[2], pos[3]),
                                             person_color, 1)

                    # 创建并显示性能热力图
                    perf_data = {}
                    for metric in ["preprocessing", "inference", "postprocessing", "frame_processing"]:
                        stats = perf_monitor.get_stats(metric)
                        if stats:
                            perf_data[metric] = stats

                    if perf_data:
                        heatmap = create_performance_heatmap(perf_data)
                        # 将热力图放在display_frame的底部
                        hm_height, hm_width = heatmap.shape[:2]
                        display_frame_height = display_frame.shape[0]

                        # 确保热力图能够放入display_frame
                        if display_frame_height > hm_height + 10 and display_frame.shape[1] >= hm_width + 20:
                            display_frame[display_frame_height - hm_height - 10:display_frame_height - 10,
                            10:10 + hm_width] = heatmap

                    # 显示结果
                    if not args.no_display:
                        cv.imshow('Camera Feed', display_frame)

                    # 如果需要保存视频
                    if video_writer is not None:
                        video_writer.write(display_frame)

                except Exception as e:
                    logger.error(f"处理帧时出错: {str(e)}")
                    traceback.print_exc()
                    cv.putText(display_frame, f"错误: {str(e)}",
                               (10, frame_height - 20), font, font_scale, (0, 0, 255), font_thickness)
                    if not args.no_display:
                        cv.imshow('Camera Feed', display_frame)

                # 如果显示界面，则等待按键事件
                if not args.no_display:
                    key = cv.waitKey(1) & 0xFF

                    # ESC(27)或q键退出
                    if key == 27 or key == ord('q'):
                        logger.info("用户退出程序")
                        break

                    # 空格键暂停/继续
                    elif key == 32:  # 空格键
                        logger.info("暂停/继续")
                        cv.waitKey(0)  # 等待任意键继续

                    # s键保存当前帧
                    elif key == ord('s'):
                        save_path = f"captured_frame_{time.strftime('%Y%m%d_%H%M%S')}.jpg"
                        cv.imwrite(save_path, display_frame)
                        logger.info(f"已保存当前帧到: {save_path}")

                    # d键切换调试信息显示
                    elif key == ord('d'):
                        # 切换日志级别
                        if logger.level == logging.INFO:
                            logger.setLevel(logging.DEBUG)
                            logger.info("已切换到调试模式")
                        else:
                            logger.setLevel(logging.INFO)
                            logger.info("已切换到普通模式")

                    # +/- 键调整置信度阈值
                    elif key == ord('+') or key == ord('='):
                        CONFIG['conf_threshold'] = min(0.95, CONFIG['conf_threshold'] + 0.05)
                        logger.info(f"增加置信度阈值到: {CONFIG['conf_threshold']:.2f}")
                    elif key == ord('-') or key == ord('_'):
                        CONFIG['conf_threshold'] = max(0.05, CONFIG['conf_threshold'] - 0.05)
                        logger.info(f"降低置信度阈值到: {CONFIG['conf_threshold']:.2f}")

                    # h键显示帮助信息
                    elif key == ord('h'):
                        help_info = np.ones((300, 400, 3), dtype=np.uint8) * 255
                        cv.putText(help_info, "Keyboard Controls:", (10, 30), font, font_scale * 1.2, (0, 0, 0), 1)
                        cv.putText(help_info, "ESC/q - Exit", (20, 60), font, font_scale, (0, 0, 0), 1)
                        cv.putText(help_info, "Space - Pause/Resume", (20, 90), font, font_scale, (0, 0, 0), 1)
                        cv.putText(help_info, "s - Save Frame", (20, 120), font, font_scale, (0, 0, 0), 1)
                        cv.putText(help_info, "d - Toggle Debug Mode", (20, 150), font, font_scale, (0, 0, 0), 1)
                        cv.putText(help_info, "+/- - Adjust Confidence", (20, 180), font, font_scale, (0, 0, 0), 1)
                        cv.putText(help_info, "h - Show Help", (20, 210), font, font_scale, (0, 0, 0), 1)
                        cv.imshow('Help', help_info)

            # 释放资源
            cap.release()
            if video_writer is not None:
                video_writer.release()
            if not args.no_display:
                cv.destroyAllWindows()
            logger.info("程序正常退出")

    except Exception as e:
        logger.error(f"主函数出错: {str(e)}")
        traceback.print_exc()