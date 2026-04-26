"""
==============================================================
  full.launch.py — 完整 Teleop 系统启动 (sender + 电机)
==============================================================

一行命令起 LiveKit sender + dm_motor_controller, 配合 VP 端 RemoteControl App
就能跑完整的 VP→电机 遥操作链路。

用法:
  ros2 launch teleop_bringup full.launch.py

  # 切档位
  ros2 launch teleop_bringup full.launch.py preset:=L5

  # 只起 sender, 不接电机 (调试视频用)
  ros2 launch teleop_bringup full.launch.py use_motor:=false

  # 只起电机, 不接 sender (mock 测试用, 配合 ros2 topic pub mock /vp/head_pose)
  ros2 launch teleop_bringup full.launch.py use_sender:=false

  # 电机 mock 模式 (没硬件时)
  ros2 launch teleop_bringup full.launch.py mock_motor:=true

  # 调试模式 (起 rqt_plot + rosbag)
  ros2 launch teleop_bringup full.launch.py debug:=true

  # 组合
  ros2 launch teleop_bringup full.launch.py preset:=L5 mock_motor:=true debug:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ===== 命令行参数 =====
    args = [
        DeclareLaunchArgument(
            'preset', default_value='L4',
            description='跟手感档位 L1/L2/L3/L4/L5 (详见 dm_motor/PRESETS.md)',
            choices=['L1', 'L2', 'L3', 'L4', 'L5'],
        ),
        DeclareLaunchArgument(
            'use_sender', default_value='true',
            description='是否启动 LiveKit sender 节点 (推视频 + 接 VP pose)',
            choices=['true', 'false'],
        ),
        DeclareLaunchArgument(
            'use_motor', default_value='true',
            description='是否启动达妙电机控制节点',
            choices=['true', 'false'],
        ),
        DeclareLaunchArgument(
            'mock_motor', default_value='false',
            description='电机是否用 mock 模式 (没硬件时设 true)',
            choices=['true', 'false'],
        ),
        DeclareLaunchArgument(
            'debug', default_value='false',
            description='调试模式: 自动开 rqt_plot 显示 pose + rosbag 录制',
            choices=['true', 'false'],
        ),
        DeclareLaunchArgument(
            'log_level', default_value='INFO',
            description='ROS 日志等级 DEBUG/INFO/WARN/ERROR',
        ),
    ]

    # ===== Node 1: LiveKit sender + bridge =====
    livekit_bridge = Node(
        package='teleop_livekit_bridge',
        executable='livekit_bridge_node',
        name='livekit_bridge',
        condition=IfCondition(LaunchConfiguration('use_sender')),
        output='screen',
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
    )

    # ===== Node 2: 达妙电机控制 =====
    dm_motor_config = PathJoinSubstitution([
        FindPackageShare('teleop_dm_motor'), 'config', 'dm_motor.yaml'
    ])
    dm_motor_calib = PathJoinSubstitution([
        FindPackageShare('teleop_dm_motor'), 'config', 'dm_motor_calibration.yaml'
    ])
    dm_motor_controller = Node(
        package='teleop_dm_motor',
        executable='dm_motor_controller_node',
        name='dm_motor_controller',
        condition=IfCondition(LaunchConfiguration('use_motor')),
        output='screen',
        parameters=[
            dm_motor_config,
            dm_motor_calib,
            # 命令行参数覆盖 yaml 默认值
            {
                'preset':    LaunchConfiguration('preset'),
                'mock_mode': LaunchConfiguration('mock_motor'),
            },
        ],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
    )

    # ===== Debug: rqt_plot + rosbag =====
    rqt_plot = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'rqt_plot', 'rqt_plot',
            '/vp/head_pose_rpy/vector/x',
            '/vp/head_pose_rpy/vector/y',
            '/vp/head_pose_rpy/vector/z',
        ],
        condition=IfCondition(LaunchConfiguration('debug')),
        output='screen',
    )

    rosbag = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '-o', '/tmp/teleop_bag',
            '/vp/head_pose',
            '/vp/head_pose_rpy',
            '/dm_motor/joint_states',
            '/dm_motor/target_states',
        ],
        condition=IfCondition(LaunchConfiguration('debug')),
        output='log',
    )

    return LaunchDescription(args + [
        livekit_bridge,
        dm_motor_controller,
        rqt_plot,
        rosbag,
    ])
