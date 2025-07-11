from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'following_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'test'), glob('test/*.py')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.py')),
        (os.path.join('share', package_name, 'data'), glob('data/*')),
    ],
    install_requires=[
        'setuptools',
        'opencv-python',
        'numpy',
        'scipy',     # 用于线性代数运算和匈牙利算法
        'pillow',
        'pathlib2',  # 为了Path支持
        'openpyxl',  # Excel文件处理
        'cv_bridge',  # ROS图像转换
        'lap',       # 匈牙利算法优化库（可选）
        'websockets', # WebSocket客户端
        'asyncio',   # 异步编程支持
    ],
    zip_safe=True,
    maintainer='monster',
    maintainer_email='monster@todo.todo',
    description='Following robot with stereo vision and human detection capabilities using RKNN',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'stereo_vision_node = following_robot.stereo_vision_node:main',
            'feature_extraction_node = following_robot.feature_extraction_node:main',
            'robot_control_node = following_robot.robot_control_node:main',
            'test_display = scripts.test_display:test_display',
            'bytetracker_node = following_robot.bytetracker_node:main',
            'websocket_bridge_node = following_robot.websocket_bridge_node:main',
            'person_detection_distance_node = following_robot.person_detection_distance_node:main',
        ],
    },
)
