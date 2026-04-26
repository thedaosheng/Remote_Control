#!/usr/bin/env python3
"""
================================================================
  CAN 总线嗅探脚本 — 识别 DM3519 一拖四的 CAN ID
================================================================
目的:在 Pi 上监听达妙 USB-CAN 适配器接收的全部帧,统计 CAN ID 分布,
     找出 DM3519 一拖四模块在上电/空闲时的反馈 ID (心跳/反馈帧)。

原理:达妙 USB2CAN 接收帧格式 (16 字节)
  [0]    = 0xAA     帧头
  [3:7]  = CAN ID   (4 字节 little-endian)
  [7:15] = 8 字节 CAN 数据
  [15]   = 0x55     帧尾

运行:
  python3 20260420-cc-pi_can_sniff.py [监听秒数]
"""

import sys
import os
import time
import glob
import subprocess
from collections import Counter, defaultdict

import serial


def find_hdsc_port():
    """扫描达妙 HDSC CDC 适配器"""
    for dev in sorted(glob.glob('/dev/ttyACM*')):
        try:
            r = subprocess.run(
                ['udevadm', 'info', '-q', 'property', dev],
                capture_output=True, text=True, timeout=2)
            if 'ID_VENDOR=HDSC' in r.stdout:
                return dev
        except Exception:
            continue
    cand = sorted(glob.glob('/dev/ttyACM*'))
    return cand[0] if cand else None


def parse_frames(buf):
    """
    从字节流里拆出达妙格式的 16 字节 CAN 帧。
    返回 [(can_id, data_bytes), ...]
    """
    frames = []
    i = 0
    while i <= len(buf) - 16:
        if buf[i] == 0xAA and buf[i + 15] == 0x55:
            can_id = (buf[i + 6] << 24) | (buf[i + 5] << 16) | \
                     (buf[i + 4] << 8) | buf[i + 3]
            data = bytes(buf[i + 7:i + 15])
            frames.append((can_id, data))
            i += 16
        else:
            i += 1
    return frames


def main():
    listen_secs = float(sys.argv[1]) if len(sys.argv) > 1 else 3.0

    port = find_hdsc_port()
    if not port:
        print('[ERR] 未找到 USB-CAN')
        return 1
    print(f'=== 监听 {port} @ 921600, {listen_secs}s ===')

    ser = serial.Serial(port, 921600, timeout=0.2)
    time.sleep(0.2)
    ser.reset_input_buffer()

    # 监听
    id_count = Counter()
    id_samples = {}   # 每个 id 存一条示例 data (hex)
    id_timing = defaultdict(list)  # 存每个 id 的到达时间

    deadline = time.time() + listen_secs
    buf = b''
    start = time.time()
    while time.time() < deadline:
        chunk = ser.read(512)
        if chunk:
            buf += chunk
            frames = parse_frames(buf)
            # 保留最后未解析的尾部 (最多 15 字节)
            if frames:
                last_idx = len(buf)
                # 粗暴地清空 buf,下一次从头开始 (不会丢重要帧)
                buf = b''
            for can_id, data in frames:
                id_count[can_id] += 1
                if can_id not in id_samples:
                    id_samples[can_id] = data.hex()
                id_timing[can_id].append(time.time() - start)
        else:
            time.sleep(0.005)

    ser.close()

    # 报告
    print()
    print(f'=== 共收到 {sum(id_count.values())} 帧 ===')
    if not id_count:
        print('(没有任何反馈帧。可能情况:')
        print(' - DM3519 一拖四没上电 / 没接 CAN')
        print(' - 波特率不匹配 (4310 是 FD 1M/5M, 3519 可能是 Classic 1M)')
        print(' - 达妙适配器工作模式错误 (需要切 Classic CAN 或 FD)')
        return 1

    print(f'{"CAN ID":>10} {"帧数":>6} {"Hz":>6}  样本数据')
    print('-' * 60)
    for cid in sorted(id_count.keys()):
        cnt = id_count[cid]
        hz = cnt / listen_secs
        sample = id_samples[cid]
        # 标注已知电机
        tag = ''
        if cid == 0x13: tag = '(FL 4310 master)'
        elif cid == 0x16: tag = '(FR 4310 master)'
        elif cid == 0x14: tag = '(RL 4310 master)'
        elif cid == 0x15: tag = '(RR 4310 master)'
        elif 0x201 <= cid <= 0x204: tag = '(可能 M3508/3519 一拖四反馈)'
        elif cid == 0x200: tag = '(可能 M3508/3519 一拖四指令)'
        elif 0x01 <= cid <= 0x10: tag = '(可能达妙电机 SlaveID)'
        print(f'0x{cid:08X} {cnt:>6} {hz:>6.1f}  {sample}  {tag}')

    return 0


if __name__ == '__main__':
    sys.exit(main())
