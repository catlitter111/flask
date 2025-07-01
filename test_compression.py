#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
图像压缩效果测试脚本
用于验证不同质量预设下的压缩效果和延迟

使用方法:
python test_compression.py --image_path /path/to/test/image.jpg
"""

import cv2
import time
import base64
import argparse
import numpy as np
from pathlib import Path

def preprocess_image(image, quality_level='poor'):
    """图像预处理优化"""
    try:
        if quality_level == 'poor':
            # 降噪和锐化
            image = cv2.bilateralFilter(image, 5, 50, 50)
            # 减少颜色深度
            image = (image // 8) * 8
        elif quality_level == 'fair':
            # 轻微降噪
            image = cv2.bilateralFilter(image, 3, 30, 30)
        
        return image
    except Exception as e:
        print(f'图像预处理失败: {e}')
        return image

def encode_image_optimized(image, quality_level='good', quality_value=65):
    """优化的图像编码"""
    try:
        # 根据网络质量选择编码参数
        if quality_level == 'poor':
            # 极低质量，最大压缩
            quality = max(20, quality_value // 3)
            encode_param = [
                int(cv2.IMWRITE_JPEG_QUALITY), quality,
                int(cv2.IMWRITE_JPEG_OPTIMIZE), 1,
                int(cv2.IMWRITE_JPEG_PROGRESSIVE), 1
            ]
        elif quality_level == 'fair':
            # 中等质量
            quality = max(40, quality_value // 2)
            encode_param = [
                int(cv2.IMWRITE_JPEG_QUALITY), quality,
                int(cv2.IMWRITE_JPEG_OPTIMIZE), 1
            ]
        else:
            # 标准质量
            quality = quality_value
            encode_param = [
                int(cv2.IMWRITE_JPEG_QUALITY), quality,
                int(cv2.IMWRITE_JPEG_OPTIMIZE), 1
            ]
        
        # 尝试JPEG编码
        success, encoded_data = cv2.imencode('.jpg', image, encode_param)
        
        if success:
            return encoded_data.tobytes(), 'jpeg', quality
        else:
            print('JPEG编码失败')
            return None, None, None
            
    except Exception as e:
        print(f'图像编码失败: {e}')
        return None, None, None

def test_quality_preset(image, preset_name, preset_config):
    """测试特定质量预设"""
    print(f"\n🎯 测试质量预设: {preset_name}")
    print(f"   配置: {preset_config}")
    
    # 调整分辨率
    start_time = time.time()
    resized = cv2.resize(image, (preset_config['width'], preset_config['height']), 
                        interpolation=cv2.INTER_AREA)
    resize_time = time.time() - start_time
    
    # 预处理
    start_time = time.time()
    if preset_name in ['ultra_low', 'minimum']:
        network_quality = 'poor'
    elif preset_name in ['very_low', 'low']:
        network_quality = 'fair'
    else:
        network_quality = 'good'
    
    processed = preprocess_image(resized, network_quality)
    preprocess_time = time.time() - start_time
    
    # 编码
    start_time = time.time()
    encoded_data, format_type, actual_quality = encode_image_optimized(
        processed, network_quality, preset_config['quality'])
    encode_time = time.time() - start_time
    
    if encoded_data is not None:
        # 转换为base64
        start_time = time.time()
        img_base64 = base64.b64encode(encoded_data).decode('utf-8')
        base64_time = time.time() - start_time
        
        # 计算压缩率
        original_size = image.shape[0] * image.shape[1] * image.shape[2]
        compressed_size = len(encoded_data)
        compression_ratio = original_size / compressed_size
        base64_size = len(img_base64.encode('utf-8'))
        
        # 总处理时间
        total_time = resize_time + preprocess_time + encode_time + base64_time
        
        print(f"   ✅ 处理完成:")
        print(f"      - 分辨率: {preset_config['width']}x{preset_config['height']}")
        print(f"      - 实际质量: {actual_quality}")
        print(f"      - 压缩前: {original_size:,} 字节")
        print(f"      - 压缩后: {compressed_size:,} 字节")
        print(f"      - Base64: {base64_size:,} 字节")
        print(f"      - 压缩率: {compression_ratio:.1f}x")
        print(f"      - 总用时: {total_time*1000:.1f}ms")
        print(f"        (调整:{resize_time*1000:.1f}ms + 预处理:{preprocess_time*1000:.1f}ms + "
              f"编码:{encode_time*1000:.1f}ms + Base64:{base64_time*1000:.1f}ms)")
        
        return {
            'preset': preset_name,
            'width': preset_config['width'],
            'height': preset_config['height'],
            'actual_quality': actual_quality,
            'original_size': original_size,
            'compressed_size': compressed_size,
            'base64_size': base64_size,
            'compression_ratio': compression_ratio,
            'total_time': total_time,
            'resize_time': resize_time,
            'preprocess_time': preprocess_time,
            'encode_time': encode_time,
            'base64_time': base64_time
        }
    else:
        print(f"   ❌ 处理失败")
        return None

def main():
    parser = argparse.ArgumentParser(description='图像压缩效果测试')
    parser.add_argument('--image_path', type=str, 
                       default='images/02a1fc76dc79b78d_jpg.rf.a7a2e633184e136322cbf5cee8625592.jpg',
                       help='测试图像路径')
    
    args = parser.parse_args()
    
    # 质量预设配置
    quality_presets = {
        'ultra_high': {'width': 800, 'height': 600, 'quality': 85, 'fps': 20},
        'high': {'width': 640, 'height': 480, 'quality': 75, 'fps': 15},
        'medium': {'width': 480, 'height': 360, 'quality': 65, 'fps': 12},
        'low': {'width': 320, 'height': 240, 'quality': 55, 'fps': 10},
        'very_low': {'width': 240, 'height': 180, 'quality': 45, 'fps': 8},
        'minimum': {'width': 160, 'height': 120, 'quality': 35, 'fps': 5},
        'ultra_low': {'width': 120, 'height': 90, 'quality': 25, 'fps': 3}
    }
    
    # 加载测试图像
    image_path = Path(args.image_path)
    if not image_path.exists():
        print(f"❌ 测试图像不存在: {image_path}")
        return
    
    print(f"📷 加载测试图像: {image_path}")
    image = cv2.imread(str(image_path))
    
    if image is None:
        print(f"❌ 无法加载图像: {image_path}")
        return
    
    print(f"   原始尺寸: {image.shape[1]}x{image.shape[0]}")
    print(f"   原始大小: {image.shape[0] * image.shape[1] * image.shape[2]:,} 字节")
    
    # 测试所有质量预设
    results = []
    
    for preset_name, preset_config in quality_presets.items():
        result = test_quality_preset(image, preset_name, preset_config)
        if result:
            results.append(result)
    
    # 输出总结
    print(f"\n📊 压缩效果总结:")
    print(f"{'预设':<12} {'分辨率':<12} {'质量':<6} {'压缩后(KB)':<12} {'Base64(KB)':<12} {'压缩率':<8} {'处理时间(ms)':<12}")
    print("-" * 80)
    
    for result in results:
        print(f"{result['preset']:<12} "
              f"{result['width']}x{result['height']:<6} "
              f"{result['actual_quality']:<6} "
              f"{result['compressed_size']/1024:.1f}<{12} "
              f"{result['base64_size']/1024:.1f}<{12} "
              f"{result['compression_ratio']:.1f}x<{8} "
              f"{result['total_time']*1000:.1f}")
    
    # 推荐设置
    print(f"\n💡 推荐设置:")
    print(f"   • 低延迟优先: ultra_low 或 minimum")
    print(f"   • 平衡模式: low 或 very_low") 
    print(f"   • 质量优先: medium 或 high")
    
    print(f"\n🚀 应用建议:")
    print(f"   1. 延迟 > 1000ms: 使用 ultra_low")
    print(f"   2. 延迟 500-1000ms: 使用 minimum")
    print(f"   3. 延迟 200-500ms: 使用 very_low")
    print(f"   4. 延迟 < 200ms: 使用 low 或更高")

if __name__ == "__main__":
    main() 