#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
特征数据流测试脚本
================

用于测试和验证特征数据从ROS2节点到小程序的完整流程：
1. 特征提取节点 -> WebSocket桥接节点
2. WebSocket桥接节点 -> 服务端
3. 服务端 -> 小程序客户端

作者: AI Assistant
日期: 2025-07-05
"""

import json
import time
import random

def generate_test_feature_data():
    """生成测试特征数据"""
    # 生成模拟的身体比例数据（16个非零值）
    body_ratios = [round(random.uniform(0.1, 2.5), 3) for _ in range(16)]
    
    # 生成模拟的颜色数据
    shirt_color = [random.randint(20, 255), random.randint(20, 255), random.randint(20, 255)]
    pants_color = [random.randint(20, 255), random.randint(20, 255), random.randint(20, 255)]
    
    # 构建特征提取节点的输出格式
    feature_extraction_output = {
        'type': 'feature_extraction_complete',
        'person_name': 'test_person',
        'timestamp': int(time.time() * 1000),
        'extraction_id': f"extract_{int(time.time() * 1000)}",
        'files': {
            'result_image': '/test/path/result.jpg',
            'feature_data': '/test/path/features.xlsx',
            'result_video': ''
        },
        'features': {
            'body_ratios': body_ratios,
            'shirt_color': shirt_color,
            'pants_color': pants_color,
            'has_valid_data': True
        }
    }
    
    return feature_extraction_output

def simulate_websocket_bridge_processing(feature_data):
    """模拟WebSocket桥接节点的处理"""
    print("\n🔄 模拟WebSocket桥接节点处理...")
    
    # 提取特征数据
    features = feature_data.get('features', {})
    body_ratios = features.get('body_ratios', [0.0] * 16)
    shirt_color = features.get('shirt_color', [0, 0, 0])
    pants_color = features.get('pants_color', [0, 0, 0])
    
    print(f"📊 提取的body_ratios: {body_ratios}")
    print(f"📊 提取的shirt_color: {shirt_color}")
    print(f"📊 提取的pants_color: {pants_color}")
    
    # 模拟格式化函数
    def rgb_to_hex(rgb):
        try:
            r, g, b = int(rgb[0]), int(rgb[1]), int(rgb[2])
            return f"#{r:02x}{g:02x}{b:02x}"
        except:
            return "#000000"
    
    def get_color_name(rgb):
        try:
            r, g, b = int(rgb[0]), int(rgb[1]), int(rgb[2])
            if r > g and r > b:
                return "红色"
            elif g > r and g > b:
                return "绿色"
            elif b > r and b > g:
                return "蓝色"
            else:
                return "灰色"
        except:
            return "未知"
    
    def format_body_proportions(body_ratios):
        if not body_ratios or len(body_ratios) < 16:
            body_ratios = [0.0] * 16
        
        return {
            'height': f"{body_ratios[0]:.3f}",
            'shoulderWidth': f"{body_ratios[3]:.3f}",
            'chest': f"{body_ratios[5]:.3f}",
            'waist': f"{body_ratios[7]:.3f}",
            'hip': f"{body_ratios[9]:.3f}",
            'armLength': f"{body_ratios[10]:.3f}",
            'legLength': f"{body_ratios[12]:.3f}",
            'headHeight': f"{body_ratios[1]:.3f}",
            'neckHeight': f"{body_ratios[2]:.3f}",
            'torsoLength': f"{(body_ratios[5] + body_ratios[7]):.3f}",
            'thighLength': f"{body_ratios[13]:.3f}",
            'calfLength': f"{body_ratios[14]:.3f}",
            'footLength': f"{body_ratios[15]:.3f}",
            'handLength': f"{(body_ratios[10] * 0.15):.3f}",
            'forearmLength': f"{body_ratios[11]:.3f}",
            'upperArmLength': f"{(body_ratios[10] - body_ratios[11]):.3f}"
        }
    
    def format_detailed_proportions(body_ratios):
        if not body_ratios or len(body_ratios) < 16:
            body_ratios = [0.0] * 16
        
        labels = [
            '身高', '头部高度', '颈部高度', '肩膀宽度',
            '胸部宽度', '胸围', '腰部宽度', '腰围',
            '臀部宽度', '臀围', '手臂长度', '前臂长度',
            '腿部长度', '大腿长度', '小腿长度', '脚部长度'
        ]
        
        return [
            {
                'label': labels[i] if i < len(labels) else f'特征{i+1}',
                'value': f"{body_ratios[i]:.3f}" if i < len(body_ratios) else "0.000"
            }
            for i in range(16)
        ]
    
    # 构建格式化的特征数据
    formatted_features = {
        'body_ratios': body_ratios,
        'clothing_colors': {
            'top': {
                'name': get_color_name(shirt_color),
                'color': rgb_to_hex(shirt_color),
                'confidence': 85
            },
            'bottom': {
                'name': get_color_name(pants_color),
                'color': rgb_to_hex(pants_color),
                'confidence': 85
            }
        },
        'body_proportions': format_body_proportions(body_ratios),
        'detailed_proportions': format_detailed_proportions(body_ratios)
    }
    
    print(f"✅ 格式化后的body_proportions: {formatted_features['body_proportions']}")
    print(f"✅ 格式化后的detailed_proportions前5个: {formatted_features['detailed_proportions'][:5]}")
    
    # 构建转发消息（WebSocket桥接节点发送给服务端的格式）
    forward_message = {
        'type': 'processed_image_notification',
        'extraction_id': feature_data.get('extraction_id'),
        'person_name': feature_data.get('person_name', ''),
        'timestamp': feature_data.get('timestamp'),
        'robot_id': 'test_robot',
        'original_image': 'data:image/jpeg;base64,test_image_data',
        'processed_image': 'data:image/jpeg;base64,test_image_data',
        'result_image': 'data:image/jpeg;base64,test_image_data',
        'features': formatted_features,
        'colors': formatted_features['clothing_colors'],
        'proportions': formatted_features['body_proportions'],
        'topColor': rgb_to_hex(shirt_color),
        'bottomColor': rgb_to_hex(pants_color),
        'topColorName': get_color_name(shirt_color),
        'bottomColorName': get_color_name(pants_color),
        'body_proportions': formatted_features['body_proportions'],
        'detailed_proportions': formatted_features['detailed_proportions'],
        'files': feature_data.get('files', {}),
        'processing_info': {
            'has_result_image': True,
            'has_feature_data': True,
            'has_result_video': False,
            'image_size_bytes': 1024,
            'feature_count': len(body_ratios),
            'has_valid_features': True
        }
    }
    
    return forward_message

def simulate_miniprogram_processing(server_data):
    """模拟小程序处理"""
    print("\n📱 模拟小程序处理...")
    
    print(f"🔍 接收到的数据keys: {list(server_data.keys())}")
    
    # 模拟小程序的数据提取过程
    features = server_data.get('features', {})
    bodyRatios = features.get('body_ratios', [0.0] * 16)
    clothingColors = features.get('clothing_colors', server_data.get('colors', {}))
    bodyProportions = features.get('body_proportions', server_data.get('proportions', {}))
    detailedProportions = features.get('detailed_proportions', server_data.get('detailed_proportions', []))
    
    print(f"📊 小程序提取的bodyRatios: {bodyRatios}")
    print(f"📊 小程序提取的bodyRatios类型: {type(bodyRatios)}")
    print(f"📊 小程序提取的bodyRatios长度: {len(bodyRatios) if isinstance(bodyRatios, list) else 'not list'}")
    print(f"📊 小程序提取的bodyRatios前5个值: {bodyRatios[:5] if isinstance(bodyRatios, list) and len(bodyRatios) >= 5 else 'insufficient data'}")
    print(f"📊 小程序提取的clothingColors: {clothingColors}")
    print(f"📊 小程序提取的bodyProportions: {bodyProportions}")
    
    # 模拟保存的数据结构
    processedImageEntry = {
        'id': server_data.get('extraction_id'),
        'name': server_data.get('person_name', '测试目标'),
        'features': {
            'body_ratios': bodyRatios,
            'clothing_colors': clothingColors,
            'body_proportions': bodyProportions,
            'detailed_proportions': detailedProportions
        },
        'body_proportions': bodyProportions,
        'detailed_proportions': detailedProportions,
        'clothing_colors': clothingColors,
        'status': 'success',
        'extraction_type': 'processed_image'
    }
    
    return processedImageEntry

def main():
    """主测试函数"""
    print("🧪 开始特征数据流测试...\n")
    
    # 1. 生成测试数据
    print("1️⃣ 生成特征提取节点测试数据...")
    feature_data = generate_test_feature_data()
    print(f"✅ 原始body_ratios: {feature_data['features']['body_ratios']}")
    print(f"✅ 原始shirt_color: {feature_data['features']['shirt_color']}")
    print(f"✅ 原始pants_color: {feature_data['features']['pants_color']}")
    
    # 2. 模拟WebSocket桥接节点处理
    print("\n2️⃣ 模拟WebSocket桥接节点处理...")
    bridge_output = simulate_websocket_bridge_processing(feature_data)
    
    # 3. 模拟小程序处理
    print("\n3️⃣ 模拟小程序处理...")
    miniprogram_data = simulate_miniprogram_processing(bridge_output)
    
    # 4. 验证数据完整性
    print("\n4️⃣ 验证数据完整性...")
    original_ratios = feature_data['features']['body_ratios']
    final_ratios = miniprogram_data['features']['body_ratios']
    
    if original_ratios == final_ratios:
        print("✅ 数据传输完整，body_ratios保持一致")
    else:
        print("❌ 数据传输有问题，body_ratios不一致")
        print(f"   原始: {original_ratios}")
        print(f"   最终: {final_ratios}")
    
    # 5. 检查身体比例格式化
    body_proportions = miniprogram_data['body_proportions']
    print(f"\n5️⃣ 检查身体比例格式化...")
    print(f"✅ 身体比例数据: {body_proportions}")
    
    # 检查是否有非零值
    non_zero_count = sum(1 for key, value in body_proportions.items() if float(value) != 0.0)
    print(f"✅ 非零比例数量: {non_zero_count}/16")
    
    if non_zero_count > 0:
        print("✅ 身体比例数据正常，包含非零值")
    else:
        print("❌ 身体比例数据异常，全部为零")
    
    print("\n🎯 测试完成！")
    
    # 生成调试JSON文件
    debug_data = {
        'original_feature_data': feature_data,
        'bridge_output': bridge_output,
        'miniprogram_data': miniprogram_data,
        'data_integrity_check': {
            'ratios_match': original_ratios == final_ratios,
            'non_zero_count': non_zero_count,
            'original_ratios': original_ratios,
            'final_ratios': final_ratios
        }
    }
    
    with open('feature_data_flow_debug.json', 'w', encoding='utf-8') as f:
        json.dump(debug_data, f, ensure_ascii=False, indent=2)
    
    print("📄 调试数据已保存到 feature_data_flow_debug.json")

if __name__ == '__main__':
    main() 