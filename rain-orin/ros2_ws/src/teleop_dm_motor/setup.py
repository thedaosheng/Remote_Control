import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'teleop_dm_motor'

setup(
    name=package_name,
    version='0.0.0',
    # find_packages 会找到 teleop_dm_motor + teleop_dm_motor.lib (vendored DM_Control_Python)
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 安装 config yaml 文件 (Stage 3 才会有内容)
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rhz',
    maintainer_email='rhz@users.noreply.github.com',
    description='达妙电机 ROS2 控制节点 (订阅 head pose 控制双轴 + 5 档预设)',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'dm_motor_controller_node = teleop_dm_motor.dm_motor_controller_node:main'
        ],
    },
)
