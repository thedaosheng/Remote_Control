#!/bin/bash
# ==============================================================================
# 20260328-cc-run_sim.sh
# Airbot Play MuJoCo 力控仿真启动脚本
#
# 用法:
#   ./20260328-cc-run_sim.sh                   # 直接连接力反馈笔
#   ./20260328-cc-run_sim.sh udp               # UDP 模式
#   ./20260328-cc-run_sim.sh none              # 无输入（仅查看）
#   ./20260328-cc-run_sim.sh keyboard          # UDP模式 + 键盘客户端（双窗口）
# ==============================================================================

# 切换到脚本所在目录
cd "$(dirname "$0")"

# 设置 DISPLAY（VNC 远程访问）
export DISPLAY=:0

# 运行模式
MODE="${1:-haptic}"

if [ "$MODE" = "keyboard" ]; then
    echo "=== 启动 UDP 模式仿真 + 键盘控制客户端 ==="
    echo "  仿真器将在后台启动，键盘客户端在前台运行"
    echo ""

    # 后台启动仿真器（UDP 模式）
    python3 20260328-cc-airbot_force_sim.py --mode udp &
    SIM_PID=$!
    echo "  仿真器 PID: $SIM_PID"

    # 等待仿真器初始化
    sleep 2

    # 前台启动键盘客户端
    python3 20260328-cc-pen_keyboard_client.py

    # 键盘客户端退出后，也停止仿真器
    echo "正在停止仿真器..."
    kill $SIM_PID 2>/dev/null
    wait $SIM_PID 2>/dev/null
    echo "全部停止。"
else
    echo "=== 启动 Airbot Play 力控仿真 ==="
    echo "  模式: $MODE"
    echo ""
    python3 20260328-cc-airbot_force_sim.py --mode "$MODE" "${@:2}"
fi
