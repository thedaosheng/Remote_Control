"""
MuJoCo 四舵轮仿真 launch 文件

用法:
  ros2 launch teleop_mujoco_sim sim.launch.py
  ros2 launch teleop_mujoco_sim sim.launch.py enable_touch:=false
  ros2 launch teleop_mujoco_sim sim.launch.py enable_keyboard:=false  # 纯 ROS2 话题控制
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # ---- 参数 ----
        DeclareLaunchArgument('enable_touch', default_value='true',
                              description='启用 Touch 力反馈笔'),
        DeclareLaunchArgument('enable_keyboard', default_value='true',
                              description='启用键盘控制 (false = 仅接受 ROS2 话题)'),
        DeclareLaunchArgument('publish_rate', default_value='50.0',
                              description='ROS2 话题发布频率 (Hz)'),

        # ---- MuJoCo 仿真节点 ----
        Node(
            package='teleop_mujoco_sim',
            executable='mujoco_sim_node',
            name='mujoco_sim',
            output='screen',
            parameters=[{
                'enable_touch': LaunchConfiguration('enable_touch'),
                'enable_keyboard': LaunchConfiguration('enable_keyboard'),
                'publish_rate': LaunchConfiguration('publish_rate'),
            }],
            # MuJoCo viewer 需要 DISPLAY
            additional_env={'DISPLAY': ':0'},
        ),
    ])
