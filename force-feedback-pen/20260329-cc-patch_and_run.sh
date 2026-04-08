#!/bin/bash
#
# Touch 力反馈笔: 补丁 libPhantomIOLib42.so 并运行程序
#
# 用法:
#   ./20260329-cc-patch_and_run.sh              # 只打补丁
#   ./20260329-cc-patch_and_run.sh ./my_app     # 打补丁后运行 my_app
#   ./20260329-cc-patch_and_run.sh ./my_app arg1 arg2
#
# 说明:
#   本脚本将 /usr/lib/libPhantomIOLib42.so 打补丁后放到
#   /tmp/patched_lib/ 并设置 LD_LIBRARY_PATH 运行程序.
#
# 补丁内容:
#   1. offset 0x32446: jne→NOP (跳过序列号验证, 避免 HD_COMM_ERROR -28)
#   2. offset 0x3f2c2: 无数据时 CommunicationReadData 返回 0 而非错误
#
# 序列号: 220-390-00866 (外壳标签)
# 固件: V01.02.09.41 (设备固件未存储序列号)

set -e

PATCHED_DIR="/tmp/patched_lib"
ORIG_LIB="/usr/lib/libPhantomIOLib42.so"
PATCHED_LIB="$PATCHED_DIR/libPhantomIOLib42.so"
FAKE_LIBS="/tmp/fakelibs"

echo "=== Touch 力反馈笔补丁脚本 ==="

# 创建目录
mkdir -p "$PATCHED_DIR"

# 检查是否需要重新打补丁 (对比 md5sum)
NEED_PATCH=1
if [ -f "$PATCHED_LIB" ]; then
    # 检查补丁是否已应用 (验证 offset 0x32446 的字节)
    BYTES=$(python3 -c "
with open('$PATCHED_LIB', 'rb') as f:
    f.seek(0x32446)
    b = f.read(6)
    print(b.hex())
" 2>/dev/null)
    if [ "$BYTES" = "909090909090" ]; then
        echo "补丁已存在, 跳过打补丁步骤"
        NEED_PATCH=0
    fi
fi

if [ "$NEED_PATCH" = "1" ]; then
    echo "正在从 $ORIG_LIB 创建补丁库..."
    cp "$ORIG_LIB" "$PATCHED_LIB"

    python3 - << 'PYTHON_EOF'
import sys

lib_path = "/tmp/patched_lib/libPhantomIOLib42.so"

with open(lib_path, 'r+b') as f:
    # ====== 补丁 1: InitPhantom 序列号验证 ======
    # 0x32446: jne 325b0 = 0f 85 64 01 00 00 (6字节)
    # → 6xNOP: 90 90 90 90 90 90
    # 效果: 跳过序列号比较, 避免 -28 错误
    f.seek(0x32446)
    old = f.read(6)
    expected = bytes([0x0f, 0x85, 0x64, 0x01, 0x00, 0x00])
    if old == expected:
        f.seek(0x32446)
        f.write(bytes([0x90, 0x90, 0x90, 0x90, 0x90, 0x90]))
        print("补丁1 OK: jne→NOP (序列号检查)")
    elif old == bytes([0x90, 0x90, 0x90, 0x90, 0x90, 0x90]):
        print("补丁1 已存在")
    else:
        print(f"补丁1 WARN: 期望 {expected.hex()} 得到 {old.hex()}")

    # ====== 补丁 2: CommunicationReadData 无数据返回 ======
    # 0x3f2c2: mov 0x6090(%rbx),%r12d = 44 8b a3 90 60 00 00 (7字节)
    # → xor %r12d,%r12d; nop×4 = 45 31 e4 90 90 90 90
    # 效果: 无位置数据时返回0(成功), 不返回初始错误码
    f.seek(0x3f2c2)
    old = f.read(7)
    expected = bytes([0x44, 0x8b, 0xa3, 0x90, 0x60, 0x00, 0x00])
    if old == expected:
        f.seek(0x3f2c2)
        f.write(bytes([0x45, 0x31, 0xe4, 0x90, 0x90, 0x90, 0x90]))
        print("补丁2 OK: CommunicationReadData 无数据→返回0")
    elif old == bytes([0x45, 0x31, 0xe4, 0x90, 0x90, 0x90, 0x90]):
        print("补丁2 已存在")
    else:
        print(f"补丁2 WARN: 期望 {expected.hex()} 得到 {old.hex()}")

print("补丁完成!")
PYTHON_EOF

    echo "补丁库已写入: $PATCHED_LIB"
fi

# 如果没有 fakelibs, 创建它
if [ ! -f "$FAKE_LIBS/libncurses.so.5" ]; then
    echo "创建 libncurses.so.5 链接..."
    mkdir -p "$FAKE_LIBS"
    ln -sf /home/rhz/haptics_install/local/lib/libncurses.so.5 "$FAKE_LIBS/libncurses.so.5" 2>/dev/null || \
    ln -sf $(ldconfig -p | grep libncurses | grep "5\b" | head -1 | awk '{print $NF}') "$FAKE_LIBS/libncurses.so.5" 2>/dev/null || \
    echo "警告: 无法创建 libncurses.so.5 链接"
fi

echo ""
echo "环境变量:"
echo "  LD_LIBRARY_PATH=$PATCHED_DIR:$FAKE_LIBS:/usr/lib"

# 如果有参数, 运行指定程序
if [ $# -gt 0 ]; then
    echo ""
    echo "运行: $@"
    echo "================================"
    export LD_LIBRARY_PATH="$PATCHED_DIR:$FAKE_LIBS:/usr/lib"
    exec "$@"
else
    echo ""
    echo "使用方式:"
    echo "  source 这个脚本设置环境变量:"
    echo "  export LD_LIBRARY_PATH=$PATCHED_DIR:$FAKE_LIBS:/usr/lib"
fi
