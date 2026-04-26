#!/usr/bin/env python3
"""
================================================================
  DM3519 一拖四 CAN ID 主动探测
================================================================
思路:逐个尝试可能的"一拖四"控制 ID,发 8 字节全 0 的无害控制帧,
     然后监听 200ms 内的所有反馈。有反馈的 ID 就是活跃的。

试的候选 (覆盖常见方案):
  - 0x200 / 0x1FF       — 大疆 RoboMaster M3508/M2006 风格
  - 0x100 / 0x101       — 达妙常见广播 ID
  - 0x300 / 0x301       — 某些固件用
  - 0x1FE / 0x2FE       — 扩展槽位

每个 ID 发送 8 字节全零 (电流/速度目标 = 0,电机不会动),
收反馈帧统计,打印所有响应 ID。
"""

import os
import sys
import time
import glob
import subprocess
import numpy as np
from collections import defaultdict

import serial


def find_hdsc_port():
    for dev in sorted(glob.glob('/dev/ttyACM*')):
        try:
            r = subprocess.run(['udevadm', 'info', '-q', 'property', dev],
                               capture_output=True, text=True, timeout=2)
            if 'ID_VENDOR=HDSC' in r.stdout:
                return dev
        except Exception:
            continue
    cand = sorted(glob.glob('/dev/ttyACM*'))
    return cand[0] if cand else None


# 达妙 USB2CAN 发送帧模板 (30 字节)
DAMIAO_TX = np.array(
    [0x55, 0xAA, 0x1e, 0x03, 0x01, 0x00, 0x00, 0x00,
     0x0a, 0x00, 0x00, 0x00, 0x00, 0, 0, 0, 0,
     0x00, 0x08, 0x00,
     0x00, 0, 0, 0, 0, 0, 0, 0, 0, 0x00], np.uint8)


def send_can_classic(ser, can_id, data8):
    """发一个 Classic CAN 11bit 标准帧"""
    frame = DAMIAO_TX.copy()
    frame[13] = can_id & 0xFF
    frame[14] = (can_id >> 8) & 0xFF
    frame[21:29] = data8
    ser.write(bytes(frame))


def parse_rx_frames(buf):
    """解析达妙 USB2CAN 16 字节接收帧"""
    frames = []
    i = 0
    while i <= len(buf) - 16:
        if buf[i] == 0xAA and buf[i + 15] == 0x55:
            cid = (buf[i + 6] << 24) | (buf[i + 5] << 16) | \
                  (buf[i + 4] << 8) | buf[i + 3]
            data = bytes(buf[i + 7:i + 15])
            frames.append((cid, data))
            i += 16
        else:
            i += 1
    return frames


def probe_id(ser, tx_id, listen_ms=300):
    """
    发一个全零 8 字节帧到 tx_id,监听 listen_ms 毫秒的反馈。
    返回 {rx_id: [data_hex_samples]}
    """
    ser.reset_input_buffer()
    send_can_classic(ser, tx_id, [0] * 8)
    time.sleep(0.02)
    # 再发一次 (有些固件第一次不响应)
    send_can_classic(ser, tx_id, [0] * 8)

    deadline = time.time() + listen_ms / 1000.0
    buf = b''
    while time.time() < deadline:
        chunk = ser.read(512)
        if chunk:
            buf += chunk
        else:
            time.sleep(0.005)

    responses = defaultdict(list)
    for cid, data in parse_rx_frames(buf):
        # 过滤掉发送回显 (如果 adapter 回显)
        if cid == tx_id:
            continue
        if len(responses[cid]) < 2:
            responses[cid].append(data.hex())
    return dict(responses)


def main():
    port = find_hdsc_port()
    if not port:
        print('[ERR] 未找到 USB-CAN')
        return 1
    print(f'=== DM3519 一拖四 ID 探测 @ {port} ===\n')

    ser = serial.Serial(port, 921600, timeout=0.05)
    time.sleep(0.2)
    ser.reset_input_buffer()

    # 先切 Classic CAN 1M (默认就是,但显式设一次保险)
    # 达妙适配器切换 CAN 比特率命令(参考 swerve_dm_driver_node)
    # 这里不切,保持默认。如果默认是 FD,后面再调。

    # 候选 ID 列表
    candidate_ids = [
        0x200, 0x1FF,       # 大疆 M3508/M2006 风格
        0x300, 0x301,       # 某些 3519 固件
        0x100, 0x101, 0x102,
        0x1FE, 0x2FE,
        0x400, 0x500,
    ]

    found = {}
    for tx in candidate_ids:
        resps = probe_id(ser, tx, listen_ms=250)
        if resps:
            found[tx] = resps
            print(f'[HIT] 发送 0x{tx:03X} → 收到反馈:')
            for rx, samples in sorted(resps.items()):
                print(f'        0x{rx:03X}: {samples}')
        else:
            print(f'[ -- ] 发送 0x{tx:03X}: 无反馈')

    ser.close()

    print()
    if not found:
        print('=== 所有候选 ID 都无反馈 ===')
        print('诊断建议:')
        print('  1. DM3519 一拖四是否已上电 (绿灯亮)?')
        print('  2. CAN 总线 120Ω 终端电阻是否都就位?')
        print('  3. 固件 CAN 模式:Classic CAN 还是 FDCAN?')
        print('     用达妙调试助手读一下')
        print('  4. 波特率:Classic CAN 通常 1M,FDCAN 默认 1M/5M')
        return 2

    print(f'=== 找到 {len(found)} 个有响应的发送 ID ===')
    print('下一步:根据反馈 ID 结构判断协议 (大疆风格 0x201-0x204? 达妙私有?)')
    return 0


if __name__ == '__main__':
    sys.exit(main())
