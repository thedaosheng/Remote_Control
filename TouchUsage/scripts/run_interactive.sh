#!/bin/bash
# Touch 力反馈笔 — 交互式 Demo (带完整诊断日志)
# 用法: bash /home/rhz/teleop/TouchUsage/scripts/run_interactive.sh

BASE="/home/rhz/teleop/TouchUsage"
LOG="/tmp/touch_launch.log"

echo "$(date) === LAUNCH START ===" > "$LOG"
echo "USER=$(whoami) SHELL=$SHELL" >> "$LOG"
echo "PWD=$PWD" >> "$LOG"
echo "LD_LIBRARY_PATH=${LD_LIBRARY_PATH:-<unset>}" >> "$LOG"
echo "LD_PRELOAD=${LD_PRELOAD:-<unset>}" >> "$LOG"
echo "PATH=$PATH" >> "$LOG"

# 1. 杀残留
echo "--- kill old ---" >> "$LOG"
for P in $(pgrep -x force_background 2>/dev/null) $(pgrep -x force_interactive 2>/dev/null); do
    echo "  killing PID=$P" >> "$LOG"
    kill "$P" 2>/dev/null
done
sleep 1

# 2. 检查残留
echo "--- post-kill check ---" >> "$LOG"
fuser /dev/ttyACM* >> "$LOG" 2>&1
pgrep -la "force_" >> "$LOG" 2>&1 || echo "  no force_ processes" >> "$LOG"

# 3. 补丁库检查
echo "--- patched lib check ---" >> "$LOG"
if [ ! -f /tmp/patched_lib/libPhantomIOLib42.so ]; then
    echo "  MISSING - running deploy" >> "$LOG"
    bash "$BASE/scripts/deploy.sh" >> "$LOG" 2>&1
fi
python3 -c "
with open('/tmp/patched_lib/libPhantomIOLib42.so','rb') as f:
    f.seek(0x3f2c2); p1=f.read(7).hex()
    f.seek(0x32446); p3=f.read(6).hex()
    f.seek(0x3b166); p4=f.read(6).hex()
print(f'Patch1={p1} Patch3={p3} Patch4={p4}')
ok = p1=='4531e490909090' and p3=='909090909090' and p4=='909090909090'
print(f'ALL_OK={ok}')
" >> "$LOG" 2>&1

# 4. 编译检查
if [ ! -f /tmp/force_interactive ]; then
    echo "  binary missing - compiling" >> "$LOG"
    gcc -o /tmp/force_interactive "$BASE/src/force_interactive.c" -ldl -lm >> "$LOG" 2>&1
fi

# 5. 设备检查
echo "--- device scan ---" >> "$LOG"
for d in /dev/ttyACM*; do
    [ ! -e "$d" ] && continue
    link=""
    [ -L "$d" ] && link=" -> $(readlink "$d")"
    vid=$(udevadm info --name="$d" 2>/dev/null | grep "ID_VENDOR_ID=" | sed 's/.*=//')
    model=$(udevadm info --name="$d" 2>/dev/null | grep "ID_MODEL=" | sed 's/.*=//')
    perm=$(ls -la "$d" 2>/dev/null | awk '{print $1}')
    echo "  $d VID=$vid MODEL=$model PERM=$perm$link" >> "$LOG"
done

# 6. 找 Touch 设备
TOUCH_DEV=""
for d in /dev/ttyACM*; do
    [ -L "$d" ] && continue; [ ! -e "$d" ] && continue
    vid=$(udevadm info --name="$d" 2>/dev/null | grep "ID_VENDOR_ID=" | sed 's/.*=//')
    if [ "$vid" = "2988" ]; then TOUCH_DEV="$d"; break; fi
done
echo "TOUCH_DEV=$TOUCH_DEV" >> "$LOG"

if [ -z "$TOUCH_DEV" ]; then
    echo "❌ 未找到 Touch 设备!"
    echo "ABORT: no device" >> "$LOG"
    cat "$LOG"
    exit 1
fi

# 7. 更新 Channel
CH=$(echo "$TOUCH_DEV" | grep -o '[0-9]*$')
echo "Channel=$CH" >> "$LOG"
sed -i "s/^Channel=.*/Channel=$CH/" ~/.3dsystems/config/"Default Device.config" 2>/dev/null
grep "^Channel=" ~/.3dsystems/config/"Default Device.config" >> "$LOG"

# 8. Serial break
echo "--- serial break ---" >> "$LOG"
python3 -c "
import serial, time
s = serial.Serial('$TOUCH_DEV', 115200, timeout=0.5)
s.sendBreak(0.25)
time.sleep(1.5)
s.close()
print('BREAK_OK')
" >> "$LOG" 2>&1

# 9. 验证设备可打开
echo "--- serial open test ---" >> "$LOG"
python3 -c "
import serial
s = serial.Serial('$TOUCH_DEV', 115200, timeout=0.5)
print(f'OPEN_OK fd={s.fileno()}')
s.close()
print('CLOSE_OK')
" >> "$LOG" 2>&1

# 10. 确认 fakelibs
echo "--- fakelibs ---" >> "$LOG"
ls -la /tmp/fakelibs/libncurses.so.5 >> "$LOG" 2>&1
nm -D /tmp/fakelibs/libncurses.so.5 2>/dev/null | grep wgetch >> "$LOG" 2>&1 || echo "  wgetch NOT exported!" >> "$LOG"

# 11. 确认 LD_LIBRARY_PATH 下的库
echo "--- lib resolution ---" >> "$LOG"
export LD_LIBRARY_PATH=/tmp/patched_lib:/tmp/fakelibs:/usr/lib
ldconfig -p 2>/dev/null | grep -i phantom >> "$LOG" 2>&1
echo "  LD_LIBRARY_PATH=$LD_LIBRARY_PATH" >> "$LOG"
ls -la /tmp/patched_lib/libPhantomIOLib42.so /usr/lib/libPhantomIOLib42.so >> "$LOG" 2>&1

# 12. 最终确认: 无进程占设备
echo "--- final fuser ---" >> "$LOG"
fuser "$TOUCH_DEV" >> "$LOG" 2>&1 || echo "  device free" >> "$LOG"

echo "$(date) === LAUNCHING ===" >> "$LOG"

echo ""
echo "  ⚠️  请把笔从底座上取下来，拿在手里!"
echo "  (诊断日志: $LOG)"
echo ""
sleep 1

LD_PRELOAD=/tmp/patched_lib/libPhantomIOLib42.so:/tmp/fakelibs/libncurses.so.5 \
LD_LIBRARY_PATH=/tmp/patched_lib:/tmp/fakelibs:/usr/lib \
exec /tmp/force_interactive
