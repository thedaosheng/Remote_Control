import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'teleop_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 安装 launch / config / README
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        (os.path.join('share', package_name),
            ['README.md'] if os.path.exists('README.md') else []),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rhz',
    maintainer_email='rhz@users.noreply.github.com',
    description='Teleop 系统 launch 入口 (一行命令起 sender + 电机 + 调试)',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
