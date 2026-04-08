from setuptools import find_packages, setup

package_name = 'airbot_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rhz',
    maintainer_email='rhz@users.noreply.github.com',
    description='Airbot dual-arm teleoperation control nodes',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            # 注册两个可执行节点，colcon build 后可直接 ros2 run 调用
            'teleop_bridge_node = airbot_control.teleop_bridge_node:main',
            'motor_control_node = airbot_control.motor_control_node:main',
        ],
    },
)
