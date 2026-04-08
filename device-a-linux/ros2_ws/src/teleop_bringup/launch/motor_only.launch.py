"""
==============================================================
  motor_only.launch.py — 只起电机, 不起 sender
==============================================================

适用场景:
  - 调试电机控制本身, 不依赖 VP / LiveKit
  - 配合 ros2 topic pub mock 数据测试
  - 例: ros2 topic pub --rate 30 /vp/head_pose teleop_msgs/msg/HeadPose "..."

用法:
  ros2 launch teleop_bringup motor_only.launch.py preset:=L4
  ros2 launch teleop_bringup motor_only.launch.py mock_motor:=true
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    full_launch = PathJoinSubstitution([
        FindPackageShare('teleop_bringup'), 'launch', 'full.launch.py'
    ])
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(full_launch),
            launch_arguments={'use_sender': 'false'}.items(),
        ),
    ])
