#!/bin/bash
# Touch 力反馈笔 — 后台运行 (Agent/脚本控制)
#
# 启动后通过文件控制:
#   echo N > /tmp/force_mode    # 切换效果 (N=0-8)
#   cat /tmp/force_status       # 读取实时状态
#   echo q > /tmp/force_mode    # 停止
#   cat /tmp/force_auto.log     # 查看日志
#
# 用法: bash TouchUsage/scripts/run_background.sh

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

# 检查是否已编译
if [ ! -f /tmp/force_background ]; then
    echo "未编译，先运行部署..."
    bash "$SCRIPT_DIR/deploy.sh"
fi

# 检查补丁库
if [ ! -f /tmp/patched_lib/libPhantomIOLib42.so ]; then
    echo "补丁库丢失，重新部署..."
    bash "$SCRIPT_DIR/deploy.sh"
fi

# 杀掉已有实例
pkill -f force_background 2>/dev/null
sleep 1

# 自动检测 Channel + serial break
for d in /dev/ttyACM*; do
    [ -L "$d" ] && continue; [ ! -e "$d" ] && continue
    vid=$(udevadm info --name="$d" 2>/dev/null | grep "ID_VENDOR_ID=" | sed 's/.*=//')
    if [ "$vid" = "2988" ]; then
        CH=$(echo "$d" | grep -o '[0-9]*$')
        sed -i "s/^Channel=.*/Channel=$CH/" ~/.3dsystems/config/"Default Device.config" 2>/dev/null
        python3 -c "import serial,time;s=serial.Serial('$d',115200,timeout=0.5);s.sendBreak(0.25);time.sleep(0.5);s.close()" 2>/dev/null
        echo "Touch 设备: $d (Channel=$CH)"
        break
    fi
done

# 后台启动
LD_LIBRARY_PATH=/tmp/patched_lib:/tmp/fakelibs:/usr/lib nohup /tmp/force_background > /dev/null 2>&1 &
PID=$!
sleep 2

# 检查启动状态
if [ -f /tmp/force_status ]; then
    echo "✅ 已启动 (PID=$PID)"
    echo ""
    cat /tmp/force_status
    echo ""
    echo "控制方式:"
    echo "  echo N > /tmp/force_mode   # 切效果 (0-8)"
    echo "  cat /tmp/force_status      # 看状态"
    echo "  echo q > /tmp/force_mode   # 停止"
else
    echo "❌ 启动失败，查看日志: cat /tmp/force_auto.log"
fi
