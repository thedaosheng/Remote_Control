#!/bin/bash
# ==============================================================
#  Teleop Docker 入口脚本
# ==============================================================
#
# 用法:
#   driver     — 只启动电机驱动节点 (生产部署)
#   sim        — 启动仿真 + 驱动 (开发调试)
#   keyboard   — 启动键盘控制 (本地调试)
#   all        — 启动全部节点
#   bash       — 进入 shell
#   <其他>     — 直接执行命令

set -e

# source ROS2 环境
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
export PYTHONPATH="/opt/dm_can:${PYTHONPATH}"

# 配置文件路径 (可通过 volume 挂载覆盖)
CONFIG_FILE="${CONFIG_FILE:-/ros2_ws/install/teleop_mujoco_sim/share/teleop_mujoco_sim/config/swerve_dm_motor.yaml}"

case "${1:-driver}" in
  driver)
    echo "=== 启动电机驱动节点 ==="
    exec ros2 run teleop_mujoco_sim swerve_dm_driver_node \
      --ros-args --params-file "$CONFIG_FILE"
    ;;

  sim)
    echo "=== 启动 MuJoCo 仿真 + 电机驱动 ==="
    # 后台启动驱动
    ros2 run teleop_mujoco_sim swerve_dm_driver_node \
      --ros-args --params-file "$CONFIG_FILE" &
    sleep 15
    # 前台启动仿真
    exec ros2 run teleop_mujoco_sim mujoco_sim_node \
      --ros-args -p enable_touch:=false
    ;;

  keyboard)
    echo "=== 启动键盘控制 ==="
    exec ros2 run teleop_mujoco_sim keyboard_teleop_node
    ;;

  all)
    echo "=== 启动全部节点 ==="
    ros2 run teleop_mujoco_sim swerve_dm_driver_node \
      --ros-args --params-file "$CONFIG_FILE" &
    sleep 15
    ros2 run teleop_mujoco_sim mujoco_sim_node \
      --ros-args -p enable_touch:=false &
    sleep 3
    exec ros2 run teleop_mujoco_sim keyboard_teleop_node
    ;;

  bash)
    exec /bin/bash
    ;;

  *)
    exec "$@"
    ;;
esac
