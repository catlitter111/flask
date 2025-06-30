#!/usr/bin/env python3
"""
视频文件兼容性测试脚本
用于验证视频文件是否可以被ByteTracker正确读取和处理
"""

import cv2
import sys
import os

def test_video_file(video_path):
    """测试视频文件兼容性"""
    print("=" * 60)
    print("🎬 ByteTracker 视频文件兼容性测试")
    print("=" * 60)
    
    # 检查文件是否存在
    if not os.path.exists(video_path):
        print(f"❌ 视频文件不存在: {video_path}")
        return False
    
    print(f"📁 测试视频文件: {video_path}")
    
    try:
        # 尝试打开视频文件
        cap = cv2.VideoCapture(video_path)
        
        if not cap.isOpened():
            print("❌ 无法打开视频文件")
            return False
        
        # 获取视频信息
        fps = cap.get(cv2.CAP_PROP_FPS)
        frame_count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        duration = frame_count / fps if fps > 0 else 0
        
        print(f"✅ 视频信息:")
        print(f"   分辨率: {width}x{height}")
        print(f"   帧率: {fps:.1f} FPS")
        print(f"   总帧数: {frame_count}")
        print(f"   时长: {duration:.1f} 秒")
        
        # 测试读取前几帧
        success_count = 0
        test_frames = min(10, frame_count)
        
        print(f"\n🔍 测试读取前 {test_frames} 帧:")
        for i in range(test_frames):
            ret, frame = cap.read()
            if ret:
                success_count += 1
                if i == 0:
                    print(f"   第 1 帧: ✅ {frame.shape}")
            else:
                print(f"   第 {i+1} 帧: ❌ 读取失败")
                break
        
        cap.release()
        
        # 兼容性评估
        print(f"\n📊 兼容性评估:")
        print(f"   帧读取成功率: {success_count}/{test_frames} ({success_count/test_frames*100:.1f}%)")
        
        # 分辨率建议
        if width < 640 or height < 480:
            print(f"   ⚠️ 分辨率较低 ({width}x{height})，建议使用 720p 或更高分辨率")
        else:
            print(f"   ✅ 分辨率合适 ({width}x{height})")
        
        # 帧率建议
        if fps < 15:
            print(f"   ⚠️ 帧率较低 ({fps:.1f} FPS)，可能影响跟踪效果")
        elif fps > 60:
            print(f"   ⚠️ 帧率较高 ({fps:.1f} FPS)，系统将限制到30FPS")
        else:
            print(f"   ✅ 帧率合适 ({fps:.1f} FPS)")
        
        # 时长建议
        if duration < 5:
            print(f"   ⚠️ 视频较短 ({duration:.1f}秒)，建议使用更长的测试视频")
        else:
            print(f"   ✅ 视频时长合适 ({duration:.1f}秒)")
        
        print(f"\n🎯 ByteTracker 兼容性:")
        if success_count == test_frames and width >= 640 and height >= 480:
            print("   ✅ 视频文件完全兼容 ByteTracker")
            print("   💡 可以直接用于调试和测试")
        else:
            print("   ⚠️ 视频文件存在兼容性问题")
            print("   💡 建议转换视频格式或检查文件完整性")
        
        return True
        
    except Exception as e:
        print(f"❌ 测试过程中发生错误: {e}")
        return False

def main():
    """主函数"""
    if len(sys.argv) != 2:
        print("使用方法: python3 test_video_compatibility.py <视频文件路径>")
        print("示例: python3 test_video_compatibility.py test_video.mp4")
        sys.exit(1)
    
    video_path = sys.argv[1]
    success = test_video_file(video_path)
    
    print("\n" + "=" * 60)
    if success:
        print("✅ 测试完成")
    else:
        print("❌ 测试失败")
    print("=" * 60)

if __name__ == "__main__":
    main() 