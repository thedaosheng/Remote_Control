"""
control.launch.py — ros2_control mock hardware 验证 launch

与 display.launch.py 的区别：
  - 不启动 joint_state_publisher_gui（会和 ros2_control 冲突）
  - 启动 ros2_control_node（controller_manager）
  - 自动 spawn joint_state_broadcaster 和 forward_command_controller

节点图：
  robot_state_publisher  ←  /robot_description
  controller_manager     ←  /robot_description + controllers.yaml
      ├── joint_state_broadcaster  →  /joint_states  →  robot_state_publisher → /tf
      └── forward_command_controller  ←  /forward_command_controller/commands
  rviz2                  ←  /tf + /robot_description
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    pkg_share = get_package_share_directory('airbot_description')
    xacro_file   = os.path.join(pkg_share, 'urdf',   'airbot_dual_arm.xacro')
    controllers_yaml = os.path.join(pkg_share, 'config', 'controllers.yaml')
    fcc_params_yaml  = os.path.join(pkg_share, 'config', 'fcc_params.yaml')
    rviz_config  = os.path.join(pkg_share, 'config', 'display.rviz')

    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    # ── 1. robot_state_publisher ──────────────────────────────────────────────
    # 订阅 /joint_states（由 joint_state_broadcaster 发布），计算并发布 /tf
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'publish_frequency': 100.0,
        }]
    )

    # ── 2. ros2_control_node（controller_manager）──────────────────────────────
    # 核心控制器管理器，加载 fake_components/GenericSystem mock hardware
    # 同时读取 controllers.yaml 得知有哪些控制器可用
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        name='controller_manager',
        output='screen',
        parameters=[
            {'robot_description': robot_description},
            controllers_yaml,           # 控制器类型和参数
        ]
    )

    # ── 3. joint_state_broadcaster spawner ───────────────────────────────────
    # 延迟 2 秒等 controller_manager 启动完毕再 spawn
    # 作用：从 hw state_interface 读关节位置/速度，发布到 /joint_states
    jsb_spawner = TimerAction(
        period=2.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            name='jsb_spawner',
            arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
            output='screen',
        )]
    )

    # ── 4. forward_command_controller spawner ────────────────────────────────
    # 延迟 3 秒（等 jsb 先起来）
    # 作用：订阅 /forward_command_controller/commands (Float64MultiArray)
    #       将 12 个关节位置指令直接写入 hw command_interface
    # 用独立的 fcc_params.yaml（/**:通配符格式），确保控制器能读到 joints 参数
    fcc_spawner = TimerAction(
        period=3.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            name='fcc_spawner',
            arguments=['forward_command_controller',
                       '--controller-manager', '/controller_manager',
                       '--param-file', fcc_params_yaml],
            output='screen',
        )]
    )

    # ── 5. rviz2 ─────────────────────────────────────────────────────────────
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
    )

    return LaunchDescription([
        robot_state_pub,
        controller_manager,
        jsb_spawner,
        fcc_spawner,
        rviz,
    ])
