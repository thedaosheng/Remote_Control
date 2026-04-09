#!/bin/bash
# Touch 力反馈笔 — 交互式 Demo 启动脚本
# 按数字键 0-8 切换效果, q 退出
# 用法: bash TouchUsage/scripts/run_interactive.sh

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

# 杀掉所有残留的 Touch 进程（这是 0x303 错误的头号原因！）
for PNAME in force_background force_interactive touch_demo; do
    for PID in $(pgrep -x "$PNAME" 2>/dev/null); do kill "$PID" 2>/dev/null; done
done
sleep 1

# 检查是否已编译
if [ ! -f /tmp/force_interactive ]; then
    echo "未编译，先运行部署..."
    bash "$SCRIPT_DIR/deploy.sh"
fi

# 检查补丁库
if [ ! -f /tmp/patched_lib/libPhantomIOLib42.so ]; then
    echo "补丁库丢失，重新部署..."
    bash "$SCRIPT_DIR/deploy.sh"
fi

# 自动检测 Channel
for d in /dev/ttyACM*; do
    [ -L "$d" ] && continue; [ ! -e "$d" ] && continue
    vid=$(udevadm info --name="$d" 2>/dev/null | grep "ID_VENDOR_ID=" | sed 's/.*=//')
    if [ "$vid" = "2988" ]; then
        CH=$(echo "$d" | grep -o '[0-9]*$')
        sed -i "s/^Channel=.*/Channel=$CH/" ~/.3dsystems/config/"Default Device.config" 2>/dev/null
        # Serial break 恢复
        python3 -c "import serial,time;s=serial.Serial('$d',115200,timeout=0.5);s.sendBreak(0.25);time.sleep(0.5);s.close()" 2>/dev/null
        break
    fi
done

echo ""
echo "  ⚠️  请把笔从底座上取下来，拿在手里!"
echo ""
sleep 1

LD_LIBRARY_PATH=/tmp/patched_lib:/tmp/fakelibs:/usr/lib exec /tmp/force_interactive
