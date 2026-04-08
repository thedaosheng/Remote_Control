"""
==============================================================
  sender_only.launch.py — 只起 sender, 不起电机
==============================================================

适用场景:
  - 调试视频推流 / VP 端连接, 跟电机无关
  - 也可以让 livekit_bridge 把 pose publish 到 topic, 你可以 ros2 topic echo 看
  - 老的 systemd lk-s2 已经在跑这个的话, 这个会因为 ZED Mini 占用启动失败,
    需要先 systemctl --user stop lk-s2

用法:
  systemctl --user stop lk-s2
  ros2 launch teleop_bringup sender_only.launch.py
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    full_launch = PathJoinSubstitution([
        FindPackageShare('teleop_bringup'), 'launch', 'full.launch.py'
    ])
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(full_launch),
            launch_arguments={'use_motor': 'false'}.items(),
        ),
    ])
