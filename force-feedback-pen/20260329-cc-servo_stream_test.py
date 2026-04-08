#!/usr/bin/env python3
"""
3D Systems Touch - 伺服使能后数据流捕获测试
=============================================
核心假设：设备使能伺服后不发送 ACK，而是直接开始以 1kHz 广播编码器数据。
本脚本测试：
1. 发送 servo enable 后立即读取所有原始字节（不等 A55A 包头）
2. 同时以 1ms 间隔发送零力 DAC 命令，看设备是否需要双向通信才启动
3. 捕获并分析数据流格式

关键区别：之前的测试都在等 A55A 包头，本测试捕获所有原始字节。
"""

import serial
import time
import sys
import struct

# ============================================================================
# 协议常量
# ============================================================================

PACKET_HEADER = b'\xa5\x5a'

# 完整的 init config 包（从 strace 精确捕获）
INIT_PACKET = b'\xa5\x5a\x04\x3c' + bytes([
    0x2f, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0xb2, 0x1c, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x1c,
    0x7f, 0x15, 0x04, 0x00, 0x00, 0x00, 0x11, 0x13,
    0x1a, 0x00, 0x12, 0x0f, 0x17, 0x16, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x02, 0x10, 0x00
])

FW_QUERY_PACKET = b'\xa5\x5a\x0f\x20' + b'\x00' * 32
SN_QUERY_PACKET = b'\xa5\x5a\x0c\x01\x00'
SERVO_ENABLE_PACKET = b'\xa5\x5a\x07\x07\x01\x00\x00\x80\x00\x00\x00'


def hex_dump(data: bytes, prefix: str = "  ", max_bytes: int = 64) -> str:
    """格式化十六进制输出"""
    if not data:
        return f"{prefix}(空)"
    if len(data) > max_bytes:
        extra = f" ... (+{len(data)-max_bytes} 字节)"
        data = data[:max_bytes]
    else:
        extra = ""
    lines = []
    for i in range(0, len(data), 16):
        chunk = data[i:i+16]
        hex_str = ' '.join(f'{b:02x}' for b in chunk)
        ascii_str = ''.join(chr(b) if 32 <= b < 127 else '.' for b in chunk)
        lines.append(f"{prefix}{i:04x}: {hex_str:<48s}  {ascii_str}")
    return '\n'.join(lines) + extra


def open_port(port='/dev/ttyACM0'):
    """打开串口"""
    ser = serial.Serial(
        port=port,
        baudrate=115200,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=0,          # 非阻塞读取
        write_timeout=1.0,
        xonxoff=False,
        rtscts=False,
        dsrdtr=False,
    )
    return ser


def init_device(ser):
    """标准初始化序列（init + fw_query + sn_query）"""
    print("\n[初始化] 清空缓冲区...")
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    time.sleep(0.05)
    ser.read(4096)  # 排空残留

    print(f"[初始化] 发送 init_config ({len(INIT_PACKET)} 字节)")
    ser.write(INIT_PACKET)
    ser.flush()
    time.sleep(0.1)
    ser.read(256)  # 丢弃 init 响应

    print(f"[初始化] 发送 fw_query")
    ser.write(FW_QUERY_PACKET)
    ser.flush()
    time.sleep(0.2)
    resp = ser.read(256)
    idx = resp.find(PACKET_HEADER)
    if idx >= 0:
        fw_str = resp[idx+4:idx+36].split(b'\x00')[0].decode('ascii', errors='replace').strip()
        print(f"  固件: {fw_str}")
    else:
        print(f"  固件响应: {resp[:32].hex() if resp else '无'}")

    print(f"[初始化] 发送 sn_query")
    ser.write(SN_QUERY_PACKET)
    ser.flush()
    time.sleep(0.2)
    resp = ser.read(256)
    idx = resp.find(PACKET_HEADER)
    if idx >= 0:
        sn = resp[idx+5:idx+21].split(b'\x00')[0].decode('ascii', errors='replace').strip()
        print(f"  序列号: {sn!r}")
    else:
        print(f"  SN 响应: {resp[:32].hex() if resp else '无'}")


def test1_raw_capture(ser):
    """
    测试1：发送 servo enable 后立即捕获所有原始字节
    不过滤任何东西，直接看设备发了什么
    """
    print(f"\n{'='*60}")
    print(f"  测试1: 使能伺服 + 原始字节捕获 (3秒)")
    print(f"{'='*60}")

    ser.reset_input_buffer()
    time.sleep(0.02)

    print(f"  发送 SERVO_ENABLE: {SERVO_ENABLE_PACKET.hex()}")
    ser.write(SERVO_ENABLE_PACKET)
    ser.flush()

    # 立即开始捕获所有数据，不等待
    all_bytes = b''
    start = time.time()
    last_print = start
    prev_len = 0

    while time.time() - start < 3.0:
        chunk = ser.read(4096)
        if chunk:
            all_bytes += chunk

        # 每 200ms 打印一次进展
        now = time.time()
        if now - last_print > 0.2:
            if len(all_bytes) != prev_len:
                print(f"  +{now-start:.1f}s: 累计 {len(all_bytes)} 字节 | "
                      f"最近: {all_bytes[-8:].hex()}")
                prev_len = len(all_bytes)
            last_print = now

    print(f"\n  总计收到: {len(all_bytes)} 字节")
    if all_bytes:
        print(f"  前 128 字节:")
        print(hex_dump(all_bytes, "    ", 128))

        # 统计非零字节
        nonzero = sum(1 for b in all_bytes if b != 0)
        print(f"  非零字节: {nonzero}/{len(all_bytes)}")

        # 查找所有包头
        hdrs = []
        i = 0
        while i < len(all_bytes) - 1:
            if all_bytes[i] == 0xa5 and all_bytes[i+1] == 0x5a:
                hdrs.append(i)
            i += 1
        print(f"  找到 {len(hdrs)} 个 A55A 包头，位置: {hdrs[:10]}")
    else:
        print("  ✗ 设备完全无响应")

    return all_bytes


def test2_bidirectional(ser):
    """
    测试2：伺服使能 + 立即发 DAC 零力命令（1ms 间隔）
    假设设备需要持续收到 DAC 命令才会开始发送编码器数据
    """
    print(f"\n{'='*60}")
    print(f"  测试2: 使能伺服 + 双向交互（1ms 节拍）")
    print(f"{'='*60}")

    # 尝试不同的"运行命令"格式
    # 根据协议分析：运行时命令可能是 cmd=0x04，携带 3x int16 DAC 值
    # 尝试几种可能的格式
    run_cmds = [
        (b'\xa5\x5a\x04\x06' + b'\x00' * 6, "cmd=0x04, 6字节数据"),
        (b'\xa5\x5a\x04\x08' + b'\x00' * 8, "cmd=0x04, 8字节数据"),
        (b'\xa5\x5a\x01\x06' + b'\x00' * 6, "cmd=0x01, 6字节数据"),
        (b'\xa5\x5a\x02\x06' + b'\x00' * 6, "cmd=0x02, 6字节数据"),
    ]

    for run_cmd, desc in run_cmds:
        print(f"\n  [尝试] 运行命令: {desc}")
        print(f"         字节: {run_cmd.hex()}")

        ser.reset_input_buffer()
        time.sleep(0.02)

        # 先发 servo enable
        ser.write(SERVO_ENABLE_PACKET)
        ser.flush()
        time.sleep(0.005)  # 5ms 后开始发 DAC

        # 以 1ms 间隔发 50 个 DAC 命令，同时读取
        all_rx = b''
        for tick in range(50):
            ser.write(run_cmd)
            time.sleep(0.001)
            chunk = ser.read(1024)
            if chunk:
                all_rx += chunk

        if all_rx:
            print(f"  ✓ 收到 {len(all_rx)} 字节!")
            print(hex_dump(all_rx[:64], "    "))
            return all_rx, run_cmd
        else:
            print(f"  ✗ 无响应")

    return b'', b''


def test3_analyze_packets(data: bytes):
    """
    测试3：分析捕获到的数据包格式
    在成功捕获到数据后调用
    """
    if not data:
        return

    print(f"\n{'='*60}")
    print(f"  测试3: 数据包格式分析")
    print(f"{'='*60}")

    # 查找所有 A55A 包
    packets = []
    i = 0
    while i < len(data) - 1:
        if data[i] == 0xa5 and data[i+1] == 0x5a:
            # 找下一个包头
            j = i + 2
            while j < len(data) - 1:
                if data[j] == 0xa5 and data[j+1] == 0x5a:
                    packets.append(data[i:j])
                    break
                j += 1
            else:
                packets.append(data[i:])
            i = j
        else:
            i += 1

    print(f"  找到 {len(packets)} 个数据包")

    # 统计命令分布
    cmds = {}
    sizes = {}
    for pkt in packets:
        if len(pkt) >= 4:
            cmd = pkt[2:4].hex()
            cmds[cmd] = cmds.get(cmd, 0) + 1
            if cmd not in sizes:
                sizes[cmd] = []
            sizes[cmd].append(len(pkt))

    for cmd, cnt in sorted(cmds.items()):
        avg = sum(sizes[cmd]) / len(sizes[cmd])
        print(f"  cmd=0x{cmd}: {cnt} 包, 平均 {avg:.0f} 字节")

    # 显示前5个包的详细内容
    print(f"\n  前 5 个包:")
    for i, pkt in enumerate(packets[:5]):
        print(f"    [{i}] len={len(pkt)}: {pkt.hex()}")
        if len(pkt) >= 8:
            data_part = pkt[4:]
            # 尝试解析为 int16
            int16s = [struct.unpack_from('<h', data_part, j)[0]
                      for j in range(0, min(len(data_part)-1, 24), 2)]
            print(f"         int16: {int16s}")


def main():
    port = '/dev/ttyACM0'
    print(f"\n{'='*60}")
    print(f"  Touch 伺服使能 + 数据流测试")
    print(f"  端口: {port}")
    print(f"{'='*60}")

    try:
        ser = open_port(port)
        print(f"  串口已打开")
    except Exception as e:
        print(f"  [错误] 无法打开串口: {e}")
        return

    try:
        # 标准初始化
        init_device(ser)

        # 测试1：原始字节捕获
        raw_data = test1_raw_capture(ser)

        if raw_data:
            test3_analyze_packets(raw_data)
        else:
            # 测试2：双向通信
            print("\n  测试1 无数据，尝试双向交互...")
            rx_data, run_cmd = test2_bidirectional(ser)
            if rx_data:
                test3_analyze_packets(rx_data)
            else:
                print("\n  ✗ 所有测试都无响应")
                print("  可能原因:")
                print("    1. 伺服使能命令格式不对")
                print("    2. init_config 参数被设备拒绝")
                print("    3. 设备固件版本与协议不匹配")
                print("    4. 设备需要特定的 SafetyKey")

    finally:
        ser.close()
        print(f"\n  串口已关闭")


if __name__ == "__main__":
    main()
