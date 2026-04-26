"""
display.launch.py — 可视化验证 launch 文件

启动内容：
  1. robot_state_publisher   — 解析 xacro，发布 /robot_description 和 TF
  2. joint_state_publisher_gui — GUI 滑条拖动关节（不接 ros2_control 时用）
  3. rviz2                   — 可视化

用法：
  ros2 launch airbot_description display.launch.py

后续切换到 ros2_control 模式时用 control.launch.py（另一个文件）
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    # 获取本包的 share 路径（安装后的路径）
    pkg_share = get_package_share_directory('airbot_description')

    # xacro 文件路径
    xacro_file = os.path.join(pkg_share, 'urdf', 'airbot_dual_arm.xacro')

    # 用 xacro 命令将 .xacro 展开为 URDF 字符串
    # 必须用 ParameterValue(..., value_type=str) 包装，否则 ROS2 会尝试把它解析成 yaml 而报错
    # 展开 xacro 时传入 hardware_plugin 参数（display 模式下不用 ros2_control，但 xacro 需要这个参数）
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' hardware_plugin:=mock_components/GenericSystem']),
        value_type=str
    )

    # RViz2 配置文件路径（首次启动没有，RViz 会用默认配置）
    rviz_config = os.path.join(pkg_share, 'config', 'display.rviz')

    return LaunchDescription([

        # ===== robot_state_publisher =====
        # 订阅 /joint_states，结合 URDF 计算所有 TF，发布 /tf
        # ROS1 对比：等价于 robot_state_publisher 节点
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                # publish_frequency: TF 发布频率，调试时 50Hz 足够
                'publish_frequency': 50.0,
            }]
        ),

        # ===== joint_state_publisher_gui =====
        # 提供 GUI 滑条手动发布 /joint_states，用于验证模型
        # 注意：接入 ros2_control 后这个节点要关掉，否则冲突
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
        ),

        # ===== rviz2 =====
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            # 如果 rviz 配置文件存在则加载，否则用默认配置
            arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
        ),

    ])
