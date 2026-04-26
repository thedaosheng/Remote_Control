import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'airbot_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # ament 索引
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # launch 文件
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # config 文件
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rhz',
    maintainer_email='rhz@users.noreply.github.com',
    description='Airbot 双臂遥操作系统启动与桥接节点',
    license='MIT',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            # ros2 run airbot_bringup teleop_bridge_node
            'teleop_bridge_node = airbot_bringup.teleop_bridge_node:main',
            # ros2 run airbot_bringup mock_data_publisher
            'mock_data_publisher = airbot_bringup.mock_data_publisher:main',
        ],
    },
)
