#!/bin/bash
# Touch 力反馈笔 — 后台运行
# 用法: bash /home/rhz/teleop/TouchUsage/scripts/run_background.sh

BASE="/home/rhz/teleop/TouchUsage"

# 杀残留进程
for P in $(pgrep -x force_background 2>/dev/null) $(pgrep -x force_interactive 2>/dev/null); do
    kill "$P" 2>/dev/null
done
sleep 1

# 没编译过或补丁丢了 → 先部署
if [ ! -f /tmp/force_background ] || [ ! -f /tmp/patched_lib/libPhantomIOLib42.so ]; then
    bash "$BASE/scripts/deploy.sh"
fi

# 重置设备
RESET_OUT=$(bash "$BASE/scripts/reset_device.sh" 2>&1)
echo "$RESET_OUT"

# 后台启动
LD_PRELOAD=/tmp/patched_lib/libPhantomIOLib42.so:/tmp/fakelibs/libncurses.so.5 \
LD_LIBRARY_PATH=/tmp/patched_lib:/tmp/fakelibs:/usr/lib \
nohup /tmp/force_background > /dev/null 2>&1 &
PID=$!
sleep 3

if [ -f /tmp/force_status ]; then
    echo "✅ 已启动 (PID=$PID)"
    echo ""
    cat /tmp/force_status
    echo ""
    echo "控制:"
    echo "  echo N > /tmp/force_mode   # 切效果 (0-8)"
    echo "  cat /tmp/force_status      # 看状态"
    echo "  echo q > /tmp/force_mode   # 停止"
else
    echo "❌ 启动失败"
    cat /tmp/force_auto.log 2>/dev/null | tail -5
fi
