from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'dlrobot_robot_python'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 添加启动文件
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        # 添加配置文件
        (os.path.join('share', package_name, 'config'),
            glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='monster',
    maintainer_email='13921994762@163.com',
    description='Python implementation of DLRobot chassis driver for ROS2 Humble',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dlrobot_robot_node = dlrobot_robot_python.dlrobot_robot_node:main',
            'cmd_vel_to_ackermann = dlrobot_robot_python.cmd_vel_to_ackermann:main',
            'parameter_node = dlrobot_robot_python.parameter_node:main',
        ],
    },
)
