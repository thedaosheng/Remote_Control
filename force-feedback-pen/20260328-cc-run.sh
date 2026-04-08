#!/bin/bash
# ============================================================================
# 20260328-cc-run.sh - 力反馈笔演示程序启动脚本
# 自动配置环境变量并运行程序
# ============================================================================

# 脚本所在目录（自动定位）
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# OpenHaptics 库路径
HAPTICS_LIB="/home/rhz/haptics_install/local/lib"

# GTDD_HOME �� Touch 驱动寻找配置文件的目录
# 使用本项目的 config 目录作为配置目录
export GTDD_HOME="${SCRIPT_DIR}/config"

# 将 OpenHaptics 库路径加入动态链接器搜索路径
export LD_LIBRARY_PATH="${HAPTICS_LIB}:${LD_LIBRARY_PATH}"

# 确保配置目录存在
mkdir -p "${GTDD_HOME}"

echo "环境变量:"
echo "  GTDD_HOME=${GTDD_HOME}"
echo "  LD_LIBRARY_PATH=${LD_LIBRARY_PATH}"
echo ""

# 检查设备连接
if lsusb | grep -q "2988:0302"; then
    echo "✓ 3D Systems Touch 设备已连接"
else
    echo "✗ 警告: 未检测�� 3D Systems Touch 设备"
    echo "  请检查 USB 连接"
fi

# 检查串口权限
if [ -e /dev/ttyACM0 ]; then
    if [ -r /dev/ttyACM0 ] && [ -w /dev/ttyACM0 ]; then
        echo "✓ /dev/ttyACM0 权限正常"
    else
        echo "✗ /dev/ttyACM0 权限不足，请将用户添加到 dialout 组:"
        echo "  sudo usermod -a -G dialout \$USER"
        exit 1
    fi
else
    echo "✗ /dev/ttyACM0 不存在"
    exit 1
fi

echo ""
echo "正在启动程序..."
echo "================================"

# 运行演示程序
exec "${SCRIPT_DIR}/20260328-cc-haptic_demo"
