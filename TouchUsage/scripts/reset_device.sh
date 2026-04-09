#!/bin/bash
# Touch 设备重置 — 检测 Channel + serial break + 验证
# 被其他脚本和 C 程序调用，不需要手动执行

# 找 Touch 设备 (VID=2988)
TOUCH_DEV=""
for d in /dev/ttyACM*; do
    [ -L "$d" ] && continue
    [ ! -e "$d" ] && continue
    vid=$(udevadm info --name="$d" 2>/dev/null | grep "ID_VENDOR_ID=" | sed 's/.*=//')
    if [ "$vid" = "2988" ]; then
        TOUCH_DEV="$d"
        break
    fi
done

if [ -z "$TOUCH_DEV" ]; then
    echo "FAIL: no Touch device found"
    exit 1
fi

CH=$(echo "$TOUCH_DEV" | grep -o '[0-9]*$')

# 更新 Channel
CFG="$HOME/.3dsystems/config/Default Device.config"
if [ -f "$CFG" ]; then
    sed -i "s/^Channel=.*/Channel=$CH/" "$CFG"
fi

# Serial break 重置
python3 -c "
import serial, time
s = serial.Serial('$TOUCH_DEV', 115200, timeout=0.5)
s.sendBreak(0.25)
time.sleep(1)
s.close()
" 2>/dev/null

echo "OK $TOUCH_DEV Channel=$CH"
