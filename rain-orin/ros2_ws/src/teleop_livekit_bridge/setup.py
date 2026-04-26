import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'teleop_livekit_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 安装 config yaml 文件 (Stage 2 才会有内容, Stage 1 留空目录)
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rhz',
    maintainer_email='rhz@users.noreply.github.com',
    description='LiveKit ↔ ROS2 桥接节点 (推视频 + 接 VP head pose)',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'livekit_bridge_node = teleop_livekit_bridge.livekit_bridge_node:main',
            'lk_data_bridge = teleop_livekit_bridge.lk_data_bridge_node:main'
        ],
    },
)
