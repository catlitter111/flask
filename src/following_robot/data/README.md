# 模型文件目录

此目录用于存放姿态检测的RKNN模型文件。

## 需要的文件

请将以下模型文件放置在此目录中：

- `yolov8_pose.rknn` - YOLOv8姿态检测RKNN模型文件

## 获取模型文件

从原始项目中复制模型文件：
```bash
cp /path/to/your/yolov8_pose.rknn ./data/
```

## 验证文件

确保模型文件存在：
```bash
ls -la data/yolov8_pose.rknn
```

文件大小应该在几十MB左右。
