from setuptools import setup
import os
from glob import glob

package_name = 'teleop_mujoco_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # launch 文件
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        # 配置文件
        (os.path.join('share', package_name, 'config'),
            glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rhz',
    maintainer_email='rhz@teleop.dev',
    description='MuJoCo 四舵轮仿真 ROS2 桥',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 仿真消费端: 订阅话题 → IK → MuJoCo 物理 → 发布状态
            'mujoco_sim_node = teleop_mujoco_sim.mujoco_sim_node:main',
            # 键盘控制层: 读键盘 → 发布 /cmd_vel 等话题
            'keyboard_teleop_node = teleop_mujoco_sim.keyboard_teleop_node:main',
            # Touch 力反馈笔: 读笔→发 /arm/target_pose, 订阅力→力反馈
            'touch_haptic_node = teleop_mujoco_sim.touch_haptic_node:main',
        ],
    },
)
