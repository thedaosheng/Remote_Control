#!/bin/bash
# 启动完整的 MuJoCo 仿真：底盘 + 升降 + 双臂 + Touch 笔遥控
#
# 必须设置:
#   LD_PRELOAD=/tmp/touch_redirect.so   把 ttyACM0/1/3 重定向到 ttyACM2 (Touch)
#   LD_LIBRARY_PATH 包含 /tmp/patched_lib (打过补丁的 libPhantomIOLib42.so)
#
# 前置条件:
#   1. ModemManager 已停止: sudo systemctl stop ModemManager
#   2. 补丁库已生成: bash 20260408-cc-patch_haptic.sh
#
# 用法:
#   bash 20260408-cc-run_full.sh         # 完整模式（含 Touch 笔）
#   ENABLE_TOUCH=0 bash 20260408-cc-run_full.sh  # 不连接 Touch（仅键盘）

set -e
cd "$(dirname "$0")"

# 检查补丁库
if [ ! -f /tmp/patched_lib/libPhantomIOLib42.so ]; then
    echo "[ERROR] 补丁库不存在，先运行: bash 20260408-cc-patch_haptic.sh"
    exit 1
fi

# 检查 redirect shim
if [ ! -f /tmp/touch_redirect.so ]; then
    echo "[INFO] 编译 touch_redirect.so..."
    gcc -shared -fPIC -o /tmp/touch_redirect.so /tmp/touch_redirect.c -ldl
fi

# 检查 ModemManager
if systemctl is-active ModemManager &>/dev/null; then
    echo "[WARN] ModemManager 在运行，会占用 Touch 设备"
    echo "       建议: sudo systemctl stop ModemManager"
fi

export DISPLAY=:0
export LD_PRELOAD=/tmp/touch_redirect.so
export LD_LIBRARY_PATH=/tmp/patched_lib:/tmp/fakelibs:/usr/lib

echo "=== 启动完整 MuJoCo 仿真 ==="
echo "  模型: 四舵轮底盘 + 升降 + 双臂 + Touch 笔遥控"
echo "  键盘: WASD/QE/GH/R/ESC"
echo "  Touch: Button1 切换右臂跟随"
echo ""

exec /home/rhz/miniconda3/envs/disc/bin/python -u 20260408-cc-swerve_keyboard_control.py
