#!/bin/bash
# Touch 设备重置 — 强力版 (DTR toggle + break + 长等待)

TOUCH_DEV=""
for d in /dev/ttyACM*; do
    [ -L "$d" ] && continue; [ ! -e "$d" ] && continue
    vid=$(udevadm info --name="$d" 2>/dev/null | grep "ID_VENDOR_ID=" | sed 's/.*=//')
    if [ "$vid" = "2988" ]; then TOUCH_DEV="$d"; break; fi
done

if [ -z "$TOUCH_DEV" ]; then
    echo "FAIL: no Touch device"
    exit 1
fi

CH=$(echo "$TOUCH_DEV" | grep -o '[0-9]*$')
CFG="$HOME/.3dsystems/config/Default Device.config"
[ -f "$CFG" ] && sed -i "s/^Channel=.*/Channel=$CH/" "$CFG"

# 强力重置: DTR toggle + break + 长等待
python3 -c "
import serial, time
s = serial.Serial('$TOUCH_DEV', 115200, timeout=0.5)
# DTR 断开重连 (模拟 USB 重枚举)
s.dtr = False
time.sleep(0.5)
s.dtr = True
time.sleep(0.5)
# Break 信号
s.sendBreak(0.5)
time.sleep(0.5)
s.close()
time.sleep(1)
# 重新打开验证
s = serial.Serial('$TOUCH_DEV', 115200, timeout=0.5)
s.close()
" 2>/dev/null

sleep 1
echo "OK $TOUCH_DEV Channel=$CH"
