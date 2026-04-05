"""
system.launch.py — Airbot 双臂遥操作系统启动文件

支持模式（mode 参数）：
  mock       → mock_components/GenericSystem（无需硬件，command 直接回传 state）
  sim_disc   → gz_ros2_control/GazeboSimSystem（DISCOVERSE/Gazebo 仿真占位）
  airbot     → airbot_hardware/AirbotSystem（路线A：官方 SDK 驱动）
  custom     → custom_hardware/CustomCANSystem（路线B：自定义 CAN 驱动）

启动内容：
  1. robot_state_publisher — 加载 URDF，发布 /robot_description + /tf
  2. controller_manager    — ros2_control 核心，管理所有硬件和控制器
  3. joint_state_broadcaster — 从硬件读取关节状态，发布 /joint_states
  4. forward_command_controller — 接收 Float64MultiArray 下发关节位置指令

用法：
  ros2 launch airbot_bringup system.launch.py mode:=mock
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    LaunchConfiguration, Command, PythonExpression
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


# 模式 → 硬件插件映射表
PLUGIN_MAP = {
    'mock': 'mock_components/GenericSystem',
    'sim_disc': 'gz_ros2_control/GazeboSimSystem',
    'airbot': 'airbot_hardware/AirbotSystem',
    'custom': 'custom_hardware/CustomCANSystem',
}


def generate_launch_description():

    # ===== 声明参数 =====
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='mock',
        description='硬件模式: mock / sim_disc / airbot / custom',
        choices=list(PLUGIN_MAP.keys()),
    )
    mode = LaunchConfiguration('mode')

    # ===== 获取包路径 =====
    desc_share = get_package_share_directory('airbot_description')
    bringup_share = get_package_share_directory('airbot_bringup')
    xacro_file = os.path.join(desc_share, 'urdf', 'airbot_dual_arm.xacro')
    controllers_yaml = os.path.join(bringup_share, 'config', 'controllers.yaml')

    # ===== 根据 mode 选择 hardware_plugin =====
    # PythonExpression 在 launch 运行时求值，从字典中取出对应的插件名
    hardware_plugin = PythonExpression([
        "{'mock': 'mock_components/GenericSystem', ",
        "'sim_disc': 'gz_ros2_control/GazeboSimSystem', ",
        "'airbot': 'airbot_hardware/AirbotSystem', ",
        "'custom': 'custom_hardware/CustomCANSystem'}",
        "['", mode, "']",
    ])

    # ===== xacro 展开为 URDF =====
    robot_description = ParameterValue(
        Command([
            'xacro ', xacro_file,
            ' hardware_plugin:=', hardware_plugin,
        ]),
        value_type=str,
    )

    # ===== robot_state_publisher =====
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'publish_frequency': 50.0,
        }],
    )

    # ===== controller_manager =====
    # 加载 URDF + 控制器参数，100Hz 更新率
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description},
            controllers_yaml,
        ],
        output='screen',
    )

    # ===== joint_state_broadcaster（等 controller_manager 就绪后 spawn）=====
    jsb_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    # ===== forward_command_controller（等 jsb 启动后再 spawn）=====
    fcc_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['forward_command_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    # 保证启动顺序：jsb 完成后再启动 fcc
    delayed_fcc = RegisterEventHandler(
        OnProcessExit(
            target_action=jsb_spawner,
            on_exit=[fcc_spawner],
        )
    )

    return LaunchDescription([
        mode_arg,
        robot_state_pub,
        controller_manager,
        jsb_spawner,
        delayed_fcc,
    ])
