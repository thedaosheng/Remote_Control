#!/usr/bin/env python3
"""
3D Systems Touch 伺服使能调试脚本
==================================
精确复现 SDK 的串口通信序列，逐步调试 enable servo 失败的原因。

关键发现（来自 strace）：
- SDK 使用 O_RDWR|O_NOCTTY|O_NONBLOCK 打开串口
- SDK 在发送 init 之前先 read 一次（清空缓冲区）
- SDK 发送顺序: init_config → firmware_query → (读响应) → serial_query → (读响应) → enable_servo
- Enable servo 后设备不回复，SDK 放弃

本脚本尝试：
1. 精确复现 SDK 的打开方式和时序
2. 在 enable servo 前后详细记录所有数据
3. 尝试不同的 servo 命令变体
"""

import serial
import time
import sys
import os
import struct
import termios
import fcntl

# ============================================================================
# 协议常量（与 SDK strace 完全一致）
# ============================================================================

PACKET_HEADER = b'\xa5\x5a'

# 完整的 init config 包（64 字节，从 strace 精确捕获）
# A5 5A 04 3C + 60 bytes data（与 touch_serial_driver 一致）
INIT_PACKET = b'\xa5\x5a\x04\x3c' + bytes([
    0x2f, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0xb2, 0x1c, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x1c,
    0x7f, 0x15, 0x04, 0x00, 0x00, 0x00, 0x11, 0x13,
    0x1a, 0x00, 0x12, 0x0f, 0x17, 0x16, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x02, 0x10, 0x00
])  # 共 64 字节 (4 header+cmd + 60 data)

# 固件查询包（36 字节）
# A5 5A 0F 20 + 32 bytes zeros
FW_QUERY_PACKET = b'\xa5\x5a\x0f\x20' + b'\x00' * 32

# 序列号查询包（5 字节）
# A5 5A 0C 01 00
SN_QUERY_PACKET = b'\xa5\x5a\x0c\x01\x00'

# 使能伺服包（11 字节）
# A5 5A 07 07 01 00 00 80 00 00 00
SERVO_ENABLE_PACKET = b'\xa5\x5a\x07\x07\x01\x00\x00\x80\x00\x00\x00'


def hex_dump(data: bytes, prefix: str = "  ") -> str:
    """格式化十六进制输出"""
    if not data:
        return f"{prefix}(空)"
    lines = []
    for i in range(0, len(data), 16):
        chunk = data[i:i+16]
        hex_str = ' '.join(f'{b:02x}' for b in chunk)
        # ASCII 可打印字符
        ascii_str = ''.join(chr(b) if 32 <= b < 127 else '.' for b in chunk)
        lines.append(f"{prefix}{i:04x}: {hex_str:<48s}  {ascii_str}")
    return '\n'.join(lines)


def open_serial_raw(port: str) -> int:
    """
    以与 SDK 完全一致的方式打开串口
    SDK 使用: open("/dev/ttyACM0", O_RDWR|O_NOCTTY|O_NONBLOCK)
    然后用 stty / termios 配置为 115200 8N1 raw
    """
    # 使用 pyserial 打开，但确保配置与 SDK 一致
    ser = serial.Serial()
    ser.port = port
    ser.baudrate = 115200
    ser.bytesize = serial.EIGHTBITS
    ser.parity = serial.PARITY_NONE
    ser.stopbits = serial.STOPBITS_ONE
    ser.timeout = 0  # 非阻塞
    ser.write_timeout = 1.0
    ser.xonxoff = False
    ser.rtscts = False
    ser.dsrdtr = False
    ser.open()
    return ser


def run_debug():
    """主调试流程"""
    port = '/dev/ttyACM0'

    print(f"\n{'='*60}")
    print(f"  3D Systems Touch - 伺服使能调试")
    print(f"{'='*60}")
    print(f"  端口: {port}")
    print(f"  时间: {time.strftime('%Y-%m-%d %H:%M:%S')}")
    print(f"{'='*60}\n")

    # 打开串口
    print("[1] 打开串口...")
    try:
        ser = open_serial_raw(port)
        print(f"    成功: {port}")
    except Exception as e:
        print(f"    失败: {e}")
        return

    try:
        # ================================================================
        # 步骤 0: 清空缓冲区（SDK 也是先 read 一次）
        # ================================================================
        print("\n[2] 清空缓冲区...")
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        # 额外读取以排空任何残留数据
        time.sleep(0.1)
        leftover = ser.read(4096)
        if leftover:
            print(f"    排空 {len(leftover)} 字节残留数据")
        else:
            print(f"    缓冲区干净")

        # ================================================================
        # 步骤 1: 发送 init config（与 SDK 完全一致的 64 字节）
        # ================================================================
        print(f"\n[3] 发送初始化配置 ({len(INIT_PACKET)} 字节)...")
        print(hex_dump(INIT_PACKET))
        ser.write(INIT_PACKET)
        ser.flush()
        time.sleep(0.1)

        # 读取可能的 init 响应
        resp = ser.read(256)
        if resp:
            print(f"    init 响应 ({len(resp)} 字节):")
            print(hex_dump(resp))
        else:
            print(f"    init 无响应（正常 - SDK 也不期望响应）")

        # ================================================================
        # 步骤 2: 查询固件版本（36 字节）
        # ================================================================
        print(f"\n[4] 查询固件版本 ({len(FW_QUERY_PACKET)} 字节)...")
        ser.write(FW_QUERY_PACKET)
        ser.flush()

        # 等待响应（SDK 使用 NONBLOCK + 重试）
        fw_resp = b''
        for attempt in range(50):  # 最多等 500ms
            time.sleep(0.01)
            chunk = ser.read(256)
            if chunk:
                fw_resp += chunk
                # 检查是否收到完整响应
                if PACKET_HEADER in fw_resp and len(fw_resp) >= 36:
                    break

        if fw_resp:
            idx = fw_resp.find(PACKET_HEADER)
            if idx >= 0:
                pkt = fw_resp[idx:]
                cmd = pkt[2:4]
                print(f"    响应: cmd=0x{cmd.hex()}, {len(pkt)} 字节")
                print(hex_dump(pkt[:36]))
                # 解析固件版本
                if cmd == b'\x0e\x20':
                    fw_str = pkt[4:36].split(b'\x00')[0].decode('ascii', errors='replace').strip()
                    print(f"    ✓ 固件版本: {fw_str}")
            else:
                print(f"    响应（无包头）: {len(fw_resp)} 字节")
                print(hex_dump(fw_resp[:64]))
        else:
            print(f"    ✗ 无响应!")

        # ================================================================
        # 步骤 3: 查询序列号（5 字节）
        # ================================================================
        print(f"\n[5] 查询序列号 ({len(SN_QUERY_PACKET)} 字节)...")
        ser.write(SN_QUERY_PACKET)
        ser.flush()

        sn_resp = b''
        for attempt in range(50):
            time.sleep(0.01)
            chunk = ser.read(256)
            if chunk:
                sn_resp += chunk
                if PACKET_HEADER in sn_resp and len(sn_resp) >= 20:
                    break

        if sn_resp:
            idx = sn_resp.find(PACKET_HEADER)
            if idx >= 0:
                pkt = sn_resp[idx:]
                cmd = pkt[2:4]
                print(f"    响应: cmd=0x{cmd.hex()}, {len(pkt)} 字节")
                print(hex_dump(pkt[:64]))
                if cmd == b'\x09\x61':
                    sn_str = pkt[5:21].split(b'\x00')[0].decode('ascii', errors='replace').strip()
                    print(f"    ✓ 序列号: {sn_str}")
        else:
            print(f"    ✗ 无响应!")

        # ================================================================
        # 步骤 4: 使能伺服（关键步骤！）
        # ================================================================
        print(f"\n{'='*60}")
        print(f"  [6] 使能伺服（关键步骤）")
        print(f"{'='*60}")
        print(f"    发送 ({len(SERVO_ENABLE_PACKET)} 字节):")
        print(hex_dump(SERVO_ENABLE_PACKET))

        # 清空缓冲区
        ser.reset_input_buffer()
        time.sleep(0.05)

        ser.write(SERVO_ENABLE_PACKET)
        ser.flush()

        # 多段等待，详细记录
        print(f"\n    等待伺服响应...")
        servo_resp = b''
        for i in range(100):  # 等 1 秒
            time.sleep(0.01)
            chunk = ser.read(1024)
            if chunk:
                servo_resp += chunk
                print(f"    +{(i+1)*10}ms: 收到 {len(chunk)} 字节 (累计 {len(servo_resp)})")
                print(hex_dump(chunk, "      "))

        if servo_resp:
            print(f"\n    ✓ 伺服响应总计 {len(servo_resp)} 字节:")
            print(hex_dump(servo_resp[:128]))

            # 分析数据包
            idx = servo_resp.find(PACKET_HEADER)
            if idx >= 0:
                pkt = servo_resp[idx:]
                cmd = pkt[2:4]
                print(f"\n    响应命令: 0x{cmd.hex()}")
        else:
            print(f"\n    ✗ 伺服无响应（1 秒超时）")

        # ================================================================
        # 步骤 5: 尝试不同的 servo 命令变体
        # ================================================================
        print(f"\n{'='*60}")
        print(f"  [7] 尝试伺服命令变体")
        print(f"{'='*60}")

        variants = [
            ("原始 (flag=0x01, gain=0x80)",
             b'\xa5\x5a\x07\x07\x01\x00\x00\x80\x00\x00\x00'),
            ("flag=0x00",
             b'\xa5\x5a\x07\x07\x00\x00\x00\x80\x00\x00\x00'),
            ("flag=0x01, gain=0x00",
             b'\xa5\x5a\x07\x07\x01\x00\x00\x00\x00\x00\x00'),
            ("flag=0x01, gain=0xFF",
             b'\xa5\x5a\x07\x07\x01\x00\x00\xff\x00\x00\x00'),
            ("只有命令头 (4字节)",
             b'\xa5\x5a\x07\x07'),
            ("不同命令: 0x0700",
             b'\xa5\x5a\x07\x00\x01\x00\x00\x80\x00\x00\x00'),
        ]

        for desc, pkt in variants:
            ser.reset_input_buffer()
            time.sleep(0.05)

            print(f"\n  尝试: {desc}")
            ser.write(pkt)
            ser.flush()

            resp = b''
            for _ in range(30):  # 300ms
                time.sleep(0.01)
                chunk = ser.read(1024)
                if chunk:
                    resp += chunk

            if resp:
                print(f"    ✓ 响应 {len(resp)} 字节: {resp[:32].hex()}")
                idx = resp.find(PACKET_HEADER)
                if idx >= 0:
                    cmd = resp[idx+2:idx+4]
                    print(f"    响应命令: 0x{cmd.hex()}")
            else:
                print(f"    ✗ 无响应")

        # ================================================================
        # 步骤 6: 使能后尝试持续读取（也许设备需要持续交互）
        # ================================================================
        print(f"\n{'='*60}")
        print(f"  [8] 使能后持续读取 + 发送空力命令")
        print(f"{'='*60}")

        # 重新发送标准 enable servo
        ser.reset_input_buffer()
        ser.write(SERVO_ENABLE_PACKET)
        ser.flush()
        time.sleep(0.05)

        # 尝试以 1kHz 发送力命令（全零），看设备是否开始回传编码器数据
        # 力命令格式猜测：A5 5A + cmd + 3x int16 DAC值
        # 可能的力命令：0x04 0x00 或某个特定命令
        force_cmds = [
            b'\xa5\x5a\x04\x00' + b'\x00' * 6,   # 猜测1: cmd 0x0400
            b'\xa5\x5a\x07\x00' + b'\x00' * 6,   # 猜测2: cmd 0x0700
            b'\xa5\x5a\x08\x00' + b'\x00' * 6,   # 猜测3: cmd 0x0800
        ]

        for fc_desc_idx, force_cmd in enumerate(force_cmds):
            print(f"\n  力命令变体 {fc_desc_idx}: {force_cmd.hex()}")
            total_rx = b''

            for tick in range(50):  # 50ms
                ser.write(force_cmd)
                ser.flush()
                time.sleep(0.001)
                chunk = ser.read(1024)
                if chunk:
                    total_rx += chunk

            if total_rx:
                print(f"    ✓ 收到 {len(total_rx)} 字节!")
                print(hex_dump(total_rx[:64]))
                break  # 找到有效的力命令
            else:
                print(f"    ✗ 无响应")

        # ================================================================
        # 步骤 7: 检查设备是否还活着
        # ================================================================
        print(f"\n{'='*60}")
        print(f"  [9] 验证设备是否还在线")
        print(f"{'='*60}")

        ser.reset_input_buffer()
        time.sleep(0.1)
        ser.write(FW_QUERY_PACKET)
        ser.flush()

        check_resp = b''
        for _ in range(50):
            time.sleep(0.01)
            chunk = ser.read(256)
            if chunk:
                check_resp += chunk
                if PACKET_HEADER in check_resp:
                    break

        if check_resp and PACKET_HEADER in check_resp:
            print(f"    ✓ 设备在线，仍可通信")
        elif check_resp:
            print(f"    ? 收到数据但无有效包头: {check_resp[:32].hex()}")
        else:
            print(f"    ✗ 设备无响应（可能需要重新插拔 USB）")

    except Exception as e:
        print(f"\n[错误] {e}")
        import traceback
        traceback.print_exc()

    finally:
        ser.close()
        print(f"\n{'='*60}")
        print(f"  调试结束")
        print(f"{'='*60}\n")


if __name__ == "__main__":
    run_debug()
