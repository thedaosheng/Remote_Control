#!/usr/bin/env python3
"""
3D Systems Touch 伺服使能调试 v2
==================================
精确复现 SDK 的完整流程：
1. 先运行 stty 命令配置端口（SDK 通过 system() 调用）
2. 再以 O_RDWR|O_NOCTTY|O_NONBLOCK 方式打开
3. 先 read 一次清空（SDK 也这么做）
4. 发送 init → firmware → serial → servo
5. 用 4096 字节缓冲区读取（与 SDK 一致）
"""

import os
import time
import sys
import struct
import fcntl
import termios
import select

# ============================================================================
# SDK 使用的 stty 命令（从 strace 完整捕获）
# ============================================================================
STTY_CMD = (
    "stty -F /dev/ttyACM0 115200 "
    "-parenb -parodd -cmspar cs8 hupcl -cstopb cread clocal -crtscts "
    "-ignbrk -brkint ignpar -parmrk -inpck -istrip -inlcr -igncr -icrnl "
    "-ixon -ixoff -iuclc -ixany -imaxbel -iutf8 "
    "-opost -olcuc -ocrnl -onlcr -onocr -onlret -ofill -ofdel "
    "nl0 cr0 tab0 bs0 vt0 ff0 "
    "-isig -icanon -iexten -echo -echoe -echok -echonl "
    "-noflsh -xcase -tostop -echoprt -echoctl -echoke -flusho -extproc"
)

# ============================================================================
# 协议数据包（从 strace 精确复制）
# ============================================================================

# Init config - SDK 第一次尝试（首字节 0x00）
INIT_PACKET_V1 = bytes([
    0xa5, 0x5a, 0x04, 0x3c,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0xb2, 0x1c, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x1c,
    0x7f, 0x15, 0x04, 0x00, 0x00, 0x00, 0x11, 0x13,
    0x1a, 0x00, 0x12, 0x0f, 0x17, 0x16, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x02, 0x10, 0x00,
])  # 62 字节 - SDK 在 strace 中显示 64，可能有 2 字节尾部填充

# Init config - SDK 第二次尝试（首字节 0x2F）
INIT_PACKET_V2 = bytes([
    0xa5, 0x5a, 0x04, 0x3c,
    0x2f, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0xb2, 0x1c, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x1c,
    0x7f, 0x15, 0x04, 0x00, 0x00, 0x00, 0x11, 0x13,
    0x1a, 0x00, 0x12, 0x0f, 0x17, 0x16, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x02, 0x10, 0x00,
])

# 固件查询（36 字节）
FW_QUERY = b'\xa5\x5a\x0f\x20' + b'\x00' * 32

# 序列号查询（5 字节）
SN_QUERY = b'\xa5\x5a\x0c\x01\x00'

# 使能伺服（11 字节）
SERVO_ENABLE = b'\xa5\x5a\x07\x07\x01\x00\x00\x80\x00\x00\x00'


def hex_dump(data: bytes, prefix: str = "  ", max_bytes: int = 128) -> str:
    """格式化十六进制输出"""
    if not data:
        return f"{prefix}(空)"
    show = data[:max_bytes]
    lines = []
    for i in range(0, len(show), 16):
        chunk = show[i:i+16]
        hex_str = ' '.join(f'{b:02x}' for b in chunk)
        ascii_str = ''.join(chr(b) if 32 <= b < 127 else '.' for b in chunk)
        lines.append(f"{prefix}{i:04x}: {hex_str:<48s}  {ascii_str}")
    if len(data) > max_bytes:
        lines.append(f"{prefix}... (共 {len(data)} 字节，省略 {len(data)-max_bytes} 字节)")
    return '\n'.join(lines)


def run():
    port = '/dev/ttyACM0'

    print(f"\n{'='*60}")
    print(f"  3D Systems Touch - 伺服调试 v2 (精确复现 SDK)")
    print(f"{'='*60}\n")

    # ================================================================
    # 步骤 1: 运行 stty 配置端口（SDK 的做法）
    # ================================================================
    print("[1] 运行 stty 配置端口...")
    print(f"    cmd: {STTY_CMD[:80]}...")
    ret = os.system(STTY_CMD)
    if ret != 0:
        print(f"    ✗ stty 失败 (返回码 {ret})")
        return
    print(f"    ✓ stty 成功")
    time.sleep(0.1)

    # ================================================================
    # 步骤 2: 以 SDK 方式打开串口
    # O_RDWR | O_NOCTTY | O_NONBLOCK
    # ================================================================
    print("\n[2] 打开串口 (O_RDWR|O_NOCTTY|O_NONBLOCK)...")
    flags = os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK
    try:
        fd = os.open(port, flags)
        print(f"    ✓ fd={fd}")
    except OSError as e:
        print(f"    ✗ 打开失败: {e}")
        return

    try:
        # ================================================================
        # 步骤 3: 设置 termios（SDK 在 stty 后还做 TCGETS/TCSETS）
        # ================================================================
        print("\n[3] 读取/设置 termios...")
        attrs = termios.tcgetattr(fd)
        # 确认 B115200
        print(f"    ispeed={attrs[4]}, ospeed={attrs[5]}")
        # 设置 B115200（如果 stty 没设的话）
        attrs[4] = termios.B115200  # ispeed
        attrs[5] = termios.B115200  # ospeed
        termios.tcsetattr(fd, termios.TCSANOW, attrs)
        print(f"    ✓ termios 配置完成")

        # ================================================================
        # 步骤 4: 初始读取清空（SDK 也这么做）
        # ================================================================
        print("\n[4] 初始清空读取...")
        try:
            leftover = os.read(fd, 4096)
            print(f"    排空 {len(leftover)} 字节")
            if leftover:
                print(hex_dump(leftover[:64], "    "))
        except BlockingIOError:
            print(f"    缓冲区干净 (EAGAIN)")

        # ================================================================
        # 步骤 5: 发送 init config
        # ================================================================
        # 尝试两种 init 包（SDK 发送两种不同的）
        for attempt, init_pkt in enumerate([INIT_PACKET_V1, INIT_PACKET_V2], 1):
            print(f"\n{'='*60}")
            print(f"  尝试 #{attempt} (init 首字节: 0x{init_pkt[4]:02x})")
            print(f"{'='*60}")

            # 补齐到 64 字节（SDK 发送 64 字节，我们的包可能少 2 字节）
            pkt = init_pkt
            if len(pkt) < 64:
                pkt = pkt + b'\x00' * (64 - len(pkt))

            print(f"\n  [5.{attempt}] 发送 init config ({len(pkt)} 字节)...")
            os.write(fd, pkt)
            time.sleep(0.1)

            # 读取可能的响应
            try:
                resp = os.read(fd, 4096)
                if resp:
                    print(f"    init 响应 ({len(resp)} 字节):")
                    print(hex_dump(resp, "    "))
            except BlockingIOError:
                print(f"    init 无响应 (EAGAIN)")

            # ================================================================
            # 步骤 6: 固件查询
            # ================================================================
            print(f"\n  [6.{attempt}] 查询固件版本 ({len(FW_QUERY)} 字节)...")
            os.write(fd, FW_QUERY)

            # 等待响应
            fw_resp = b''
            for i in range(50):
                time.sleep(0.01)
                try:
                    chunk = os.read(fd, 128)
                    fw_resp += chunk
                    if b'\xa5\x5a' in fw_resp and len(fw_resp) >= 36:
                        break
                except BlockingIOError:
                    pass

            if fw_resp:
                idx = fw_resp.find(b'\xa5\x5a')
                if idx >= 0:
                    pkt = fw_resp[idx:]
                    cmd = pkt[2:4]
                    print(f"    ✓ 响应: cmd=0x{cmd.hex()}")
                    print(hex_dump(fw_resp, "    "))
                    if cmd == b'\x0e\x20':
                        fw_str = pkt[4:36].split(b'\x00')[0].decode('ascii', errors='replace').strip()
                        print(f"    固件版本: {fw_str}")
                else:
                    print(f"    响应（无包头）: {len(fw_resp)} 字节")
                    print(hex_dump(fw_resp[:64], "    "))
            else:
                print(f"    ✗ 无响应")
                continue  # 如果固件查询失败，跳到下一个尝试

            # ================================================================
            # 步骤 7: 序列号查询
            # ================================================================
            print(f"\n  [7.{attempt}] 查询序列号 ({len(SN_QUERY)} 字节)...")
            os.write(fd, SN_QUERY)

            sn_resp = b''
            for i in range(50):
                time.sleep(0.01)
                try:
                    chunk = os.read(fd, 128)
                    sn_resp += chunk
                    if b'\xa5\x5a' in sn_resp and len(sn_resp) >= 20:
                        break
                except BlockingIOError:
                    pass

            if sn_resp:
                print(f"    ✓ 响应: {len(sn_resp)} 字节")
                print(hex_dump(sn_resp, "    "))
            else:
                print(f"    ✗ 无响应")

            # 读取残留
            time.sleep(0.1)
            try:
                extra = os.read(fd, 4096)
                if extra:
                    print(f"    额外数据: {len(extra)} 字节")
            except BlockingIOError:
                pass

            # ================================================================
            # 步骤 8: 使能伺服！
            # ================================================================
            print(f"\n  [8.{attempt}] ★★★ 使能伺服 ★★★")
            print(f"    发送: {SERVO_ENABLE.hex()}")
            os.write(fd, SERVO_ENABLE)

            # 用 select + 大缓冲区读取（与 SDK 一致用 4096）
            print(f"\n    等待响应 (3秒, 4096字节缓冲区)...")
            servo_resp = b''
            start = time.time()
            while time.time() - start < 3.0:
                # 使用 select 检查是否有数据可读
                r, _, _ = select.select([fd], [], [], 0.05)
                if r:
                    try:
                        chunk = os.read(fd, 4096)
                        if chunk:
                            elapsed_ms = (time.time() - start) * 1000
                            servo_resp += chunk
                            print(f"    +{elapsed_ms:.0f}ms: 收到 {len(chunk)} 字节 (累计 {len(servo_resp)})")
                            # 显示前 64 字节
                            print(hex_dump(chunk[:64], "      "))
                    except BlockingIOError:
                        pass

            if servo_resp:
                print(f"\n    ★ 伺服响应总计: {len(servo_resp)} 字节")
                print(hex_dump(servo_resp, "    ", max_bytes=256))

                # 搜索 A5 5A 包头
                idx = servo_resp.find(b'\xa5\x5a')
                if idx >= 0:
                    pkt = servo_resp[idx:]
                    print(f"\n    找到数据包 @ 偏移 {idx}: cmd=0x{pkt[2:4].hex()}")
                    print(hex_dump(pkt[:64], "    "))
                else:
                    print(f"\n    未找到 A5 5A 包头")

                # 分析数据模式
                print(f"\n    字节分布:")
                from collections import Counter
                counts = Counter(servo_resp)
                top10 = counts.most_common(10)
                for byte_val, count in top10:
                    pct = count / len(servo_resp) * 100
                    print(f"      0x{byte_val:02x} ({chr(byte_val) if 32 <= byte_val < 127 else '.'}) : {count} ({pct:.1f}%)")
            else:
                print(f"\n    ✗ 伺服无响应（3 秒超时）")

            # ================================================================
            # 步骤 9: 关闭并重新打开（SDK 做两次尝试）
            # ================================================================
            if attempt == 1:
                print(f"\n  关闭 fd={fd}，准备第二次尝试...")
                os.close(fd)
                time.sleep(0.5)

                # 重新运行 stty
                os.system(STTY_CMD)
                time.sleep(0.1)

                # 重新打开
                fd = os.open(port, flags)
                print(f"  重新打开 fd={fd}")

                # 清空
                try:
                    os.read(fd, 4096)
                except BlockingIOError:
                    pass

    except Exception as e:
        print(f"\n[错误] {e}")
        import traceback
        traceback.print_exc()

    finally:
        try:
            os.close(fd)
        except:
            pass
        print(f"\n{'='*60}")
        print(f"  调试 v2 结束")
        print(f"{'='*60}\n")


if __name__ == "__main__":
    run()
