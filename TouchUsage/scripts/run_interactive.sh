#!/bin/bash
# Touch 力反馈笔 — 交互式 Demo
# 用法: bash /home/rhz/teleop/TouchUsage/scripts/run_interactive.sh

BASE="/home/rhz/teleop/TouchUsage"

# 杀残留进程
for P in $(pgrep -x force_background 2>/dev/null) $(pgrep -x force_interactive 2>/dev/null); do
    kill "$P" 2>/dev/null
done
sleep 1

# 没编译过或补丁丢了 → 先部署
if [ ! -f /tmp/force_interactive ] || [ ! -f /tmp/patched_lib/libPhantomIOLib42.so ]; then
    bash "$BASE/scripts/deploy.sh"
fi

# 重置设备
bash "$BASE/scripts/reset_device.sh"

echo ""
echo "  ⚠️  请把笔从底座上取下来，拿在手里!"
echo ""
sleep 1

LD_LIBRARY_PATH=/tmp/patched_lib:/tmp/fakelibs:/usr/lib exec /tmp/force_interactive
