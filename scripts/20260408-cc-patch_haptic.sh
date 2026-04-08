#!/bin/bash
# 20260408-cc-patch_haptic.sh — 重新打补丁 libPhantomIOLib42.so
#
# 包含 3 个必须补丁：
#   1. 0x32446 (序列号检查)
#   2. 0x3f2c2 (CommunicationReadData 无数据返回)
#   3. 0x3b166 (hdStartScheduler RTAI 初始化)
#
# 用法:
#   bash 20260408-cc-patch_haptic.sh
#
# 运行时环境变量:
#   LD_LIBRARY_PATH=/tmp/patched_lib:/tmp/fakelibs:/usr/lib
set -e

PATCHED_DIR="/tmp/patched_lib"
ORIG_LIB="/usr/lib/libPhantomIOLib42.so"
PATCHED_LIB="$PATCHED_DIR/libPhantomIOLib42.so"
FAKE_LIBS="/tmp/fakelibs"

echo "=== 重新打补丁 libPhantomIOLib42.so (3 个补丁) ==="

# 1. 复制原始库
mkdir -p "$PATCHED_DIR"
cp "$ORIG_LIB" "$PATCHED_LIB"

# 2. 打所有 3 个补丁
/usr/bin/python3 << 'PYEOF'
lib = "/tmp/patched_lib/libPhantomIOLib42.so"
with open(lib, 'r+b') as f:
    # 补丁 1: 0x32446 jne→NOP×6 (跳过序列号检查)
    f.seek(0x32446)
    orig = f.read(6)
    exp = bytes([0x0f, 0x85, 0x64, 0x01, 0x00, 0x00])
    if orig == exp:
        f.seek(0x32446)
        f.write(b'\x90' * 6)
        print("补丁 1 OK: 0x32446 序列号检查")
    else:
        print(f"补丁 1 WARN: 期望 {exp.hex()}, 得到 {orig.hex()}")

    # 补丁 2: 0x3f2c2 CommunicationReadData 无数据返回 0
    f.seek(0x3f2c2)
    orig = f.read(7)
    exp = bytes([0x44, 0x8b, 0xa3, 0x90, 0x60, 0x00, 0x00])
    if orig == exp:
        f.seek(0x3f2c2)
        f.write(bytes([0x45, 0x31, 0xe4, 0x90, 0x90, 0x90, 0x90]))
        print("补丁 2 OK: 0x3f2c2 CommunicationReadData")
    else:
        print(f"补丁 2 WARN: 期望 {exp.hex()}, 得到 {orig.hex()}")

    # 补丁 4: 0x3b166 hdStartScheduler RTAI 初始化
    f.seek(0x3b166)
    orig = f.read(6)
    exp = bytes([0x0f, 0x85, 0xc4, 0x00, 0x00, 0x00])
    if orig == exp:
        f.seek(0x3b166)
        f.write(b'\x90' * 6)
        print("补丁 4 OK: 0x3b166 hdStartScheduler RTAI")
    else:
        print(f"补丁 4 WARN: 期望 {exp.hex()}, 得到 {orig.hex()}")

print("所有补丁完成")
PYEOF

# 3. 创建 fakelibs (libncurses.so.5 壳)
mkdir -p "$FAKE_LIBS"
if [ ! -f "$FAKE_LIBS/libncurses.so.5" ]; then
    SRC_NCURSES=$(find /home/rhz/haptics_install -name "libncurses.so.5" 2>/dev/null | head -1)
    if [ -n "$SRC_NCURSES" ]; then
        ln -sf "$SRC_NCURSES" "$FAKE_LIBS/libncurses.so.5"
        echo "fakelibs: 链接 $SRC_NCURSES"
    else
        # 用系统 ncurses.so.6 做 soft link
        REAL_NC=$(ldconfig -p | grep "libncurses\.so\.6" | head -1 | awk '{print $NF}')
        ln -sf "$REAL_NC" "$FAKE_LIBS/libncurses.so.5"
        echo "fakelibs: 链接 $REAL_NC as libncurses.so.5"
    fi
fi

echo ""
echo "=== 补丁完成 ==="
echo "运行时设置 LD_LIBRARY_PATH:"
echo "  export LD_LIBRARY_PATH=$PATCHED_DIR:$FAKE_LIBS:/usr/lib"
