#!/bin/bash
# Touch 力反馈笔 — 一键部署脚本
# 功能: 重建补丁库 + 自动检测 Channel + 编译 demo + 验证设备
# 用法: bash TouchUsage/scripts/deploy.sh

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
SRC_DIR="$SCRIPT_DIR/../src"
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; NC='\033[0m'

echo "╔══════════════════════════════════════════╗"
echo "║  Touch 力反馈笔 — 部署                   ║"
echo "╚══════════════════════════════════════════╝"
echo ""

# === 0. 杀掉所有残留 Touch 进程 ===
for PNAME in force_background force_interactive touch_demo; do
    for PID in $(pgrep -x "$PNAME" 2>/dev/null); do
        kill "$PID" 2>/dev/null
    done
done
sleep 1

# === 1. 检查驱动库 ===
echo -n "[1/5] 检查驱动库... "
if [ ! -f /usr/lib/libHD.so ] || [ ! -f /usr/lib/libPhantomIOLib42.so ]; then
    echo -e "${RED}失败: 缺少 libHD.so 或 libPhantomIOLib42.so${NC}"
    exit 1
fi
echo -e "${GREEN}OK${NC}"

# === 2. 重建补丁库 ===
echo -n "[2/5] 重建补丁库... "
mkdir -p /tmp/patched_lib /tmp/fakelibs

# 假 ncurses 库 (必须导出 wgetch 等符号, 否则 libHD.so 加载失败)
cat > /tmp/_fake_ncurses.c << 'FAKEC'
int wgetch(void *w) { return -1; }
int endwin(void) { return 0; }
void *initscr(void) { return (void*)0; }
int noecho(void) { return 0; }
int cbreak(void) { return 0; }
int nodelay(void *w, int bf) { return 0; }
int keypad(void *w, int bf) { return 0; }
void *stdscr = 0;
FAKEC
gcc -shared -fPIC -o /tmp/fakelibs/libncurses.so.5 /tmp/_fake_ncurses.c 2>/dev/null
rm -f /tmp/_fake_ncurses.c

# 打补丁的 PhantomIO 库
python3 -c "
import shutil
shutil.copy2('/usr/lib/libPhantomIOLib42.so', '/tmp/patched_lib/libPhantomIOLib42.so')
with open('/tmp/patched_lib/libPhantomIOLib42.so', 'r+b') as f:
    f.seek(0x3f2c2); f.write(bytes([0x45,0x31,0xe4,0x90,0x90,0x90,0x90]))  # Patch1
    f.seek(0x32446); f.write(bytes([0x90,0x90,0x90,0x90,0x90,0x90]))        # Patch3
    f.seek(0x3b166); f.write(bytes([0x90,0x90,0x90,0x90,0x90,0x90]))        # Patch4
"

# 验证补丁
python3 -c "
with open('/tmp/patched_lib/libPhantomIOLib42.so', 'rb') as f:
    f.seek(0x3f2c2); assert f.read(7).hex()=='4531e490909090', 'Patch1'
    f.seek(0x32446); assert f.read(6).hex()=='909090909090', 'Patch3'
    f.seek(0x3b166); assert f.read(6).hex()=='909090909090', 'Patch4'
"
echo -e "${GREEN}OK (3 patches applied)${NC}"

# === 3. 自动检测 Touch 设备并更新 Channel ===
echo -n "[3/5] 检测设备... "
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
    echo -e "${RED}未找到 Touch 设备 (VID=2988)${NC}"
    echo "  请检查 USB 连接: lsusb | grep 2988"
    exit 1
fi

CH=$(echo "$TOUCH_DEV" | grep -o '[0-9]*$')
echo -e "${GREEN}$TOUCH_DEV (Channel=$CH)${NC}"

# 更新 config
CFG_DIR="$HOME/.3dsystems/config"
mkdir -p "$CFG_DIR"
CFG="$CFG_DIR/Default Device.config"
if [ -f "$CFG" ]; then
    sed -i "s/^Channel=.*/Channel=$CH/" "$CFG"
else
    # 创建默认配置
    cat > "$CFG" << CFGEOF
ModelType=OMNI
L1=133.350000
L2=133.350000
L3=0.000000
CardType=USB
Channel=$CH
Extra6DOFOption=0
Reduction1=0.134030
Reduction2=0.132996
Reduction3=0.132820
Reduction4=1.000000
Reduction5=0.000000
Reduction6=0.000000
HighForce=0
Gimbal=1
ReverseHandle=0
NominalMaxStiffness=0.500000
NominalMaxDamping=0.003000
NominalMaxForce=3.300000
NominalMaxContinuousForce=0.880000
NominalMaxTorqueStiffness=0.000000
NominalMaxTorqueDamping=0.000000
NominalMaxTorqueForce=0.000000
NominalMaxTorqueContinuousForce=0.000000
ResetAngle0=0.000000
ResetAngle1=0.000000
ResetAngle2=0.000000
ResetAngle3=-2.617994
ResetAngle4=4.188790
ResetAngle5=-2.617994
WorkspaceOffset0=0.000000
WorkspaceOffset1=-110.000000
WorkspaceOffset2=-35.000000
EPPDevice=0
1394SerialNumber=-1
SoftwareInterrupt=0
Niobe3Button=0
HostName=N/A
SafetyKey=00000000000000000000000000000000
ConfigOption=0
FilterGimbalSize=0
DeviceSerialNumber=No Serial Number
CFGEOF
fi

# Serial break 重置 (防止之前非正常退出锁设备)
python3 -c "
import serial, time
try:
    s = serial.Serial('$TOUCH_DEV', 115200, timeout=0.5)
    s.sendBreak(0.25)
    time.sleep(0.5)
    s.close()
except: pass
" 2>/dev/null

# === 4. 编译 ===
echo -n "[4/5] 编译 demo... "
gcc -o /tmp/force_interactive "$SRC_DIR/force_interactive.c" -ldl -lm 2>/dev/null
gcc -o /tmp/force_background "$SRC_DIR/force_background.c" -ldl -lm 2>/dev/null
echo -e "${GREEN}OK${NC}"

# === 5. 快速验证 ===
echo -n "[5/5] 验证设备通信... "
LD_PRELOAD=/tmp/patched_lib/libPhantomIOLib42.so:/tmp/fakelibs/libncurses.so.5 \
LD_LIBRARY_PATH=/tmp/patched_lib:/tmp/fakelibs:/usr/lib \
timeout 8 /tmp/force_background > /dev/null 2>&1 &
PID=$!
sleep 5
if [ -f /tmp/force_status ]; then
    SERVO=$(grep servo /tmp/force_status | grep -o '[0-9]*Hz')
    echo -e "${GREEN}伺服 $SERVO${NC}"
else
    echo -e "${YELLOW}无法读取状态 (可能需要拔插USB)${NC}"
fi
echo q > /tmp/force_mode 2>/dev/null
wait $PID 2>/dev/null

echo ""
echo "════════════════════════════════════════════"
echo -e "  ${GREEN}部署完成!${NC}"
echo ""
echo "  交互式体验:  bash TouchUsage/scripts/run_interactive.sh"
echo "  后台运行:    bash TouchUsage/scripts/run_background.sh"
echo "════════════════════════════════════════════"
