#!/usr/bin/env python3
"""
================================================================
  ZLAC 4 轮方向验证 — 按 FL → RL → RR → FR 顺序,各 +30 RPM 转 3 秒
================================================================
目的:让用户肉眼判断每个轮子"正转 +RPM"时的物理转向是否符合车前进方向。

映射 (用户确认):
  FL 左前 = Node 3 sub 2 (Right Motor)
  RL 左后 = Node 2 sub 1 (Left Motor)
  RR 右后 = Node 2 sub 2 (Right Motor)
  FR 右前 = Node 3 sub 1 (Left Motor)

期望:4 个轮子 +RPM 都应该让车"整体向前"运动。
     但若左右轮是镜像安装,同 +RPM 时左右轮实际转向相反 —
     这是正常的,需要在软件里加方向修正系数 direction[] = [±1, ±1, ±1, ±1]
"""

import os
import sys
import time
import numpy as np

import serial


DAMIAO_TX = np.array(
    [0x55, 0xAA, 0x1e, 0x03, 0x01, 0x00, 0x00, 0x00,
     0x0a, 0x00, 0x00, 0x00, 0x00, 0, 0, 0, 0,
     0x00, 0x08, 0x00,
     0x00, 0, 0, 0, 0, 0, 0, 0, 0, 0x00], np.uint8)


def send_can(ser, cid, data):
    f = DAMIAO_TX.copy()
    f[13] = cid & 0xFF
    f[14] = (cid >> 8) & 0xFF
    f[21:29] = data
    ser.write(bytes(f))


def recv_filtered(ser, rx_id, timeout=0.3):
    dl = time.time() + timeout
    buf = b''
    while time.time() < dl:
        c = ser.read_all()
        if c: buf += c
        i = 0
        while i <= len(buf) - 16:
            if buf[i] == 0xAA and buf[i + 15] == 0x55:
                cid = (buf[i + 6] << 24) | (buf[i + 5] << 16) | \
                      (buf[i + 4] << 8) | buf[i + 3]
                if cid == rx_id:
                    return buf[i + 7:i + 15]
                i += 16
            else:
                i += 1
        buf = buf[max(0, len(buf) - 15):]
        if not c: time.sleep(0.005)
    return None


def sdo_write(ser, nid, index, subindex, value, size):
    tx = 0x600 + nid
    rx = 0x580 + nid
    cmd = {1: 0x2F, 2: 0x2B, 4: 0x23}.get(size, 0x23)
    v = value if value >= 0 else value & 0xFFFFFFFF
    data = [cmd, index & 0xFF, (index >> 8) & 0xFF, subindex,
            v & 0xFF, (v >> 8) & 0xFF, (v >> 16) & 0xFF, (v >> 24) & 0xFF]
    ser.read_all()
    send_can(ser, tx, data)
    resp = recv_filtered(ser, rx, 0.3)
    return resp is not None and resp[0] == 0x60


def send_velocity(ser, nid, sub, rpm):
    """fire-and-forget"""
    tx = 0x600 + nid
    v = int(rpm) if rpm >= 0 else int(rpm) & 0xFFFFFFFF
    data = [0x23, 0xFF, 0x60, sub,
            v & 0xFF, (v >> 8) & 0xFF, (v >> 16) & 0xFF, (v >> 24) & 0xFF]
    send_can(ser, tx, data)


def enable_zlac(ser, nid):
    sdo_write(ser, nid, 0x6040, 0, 0x0000, 2); time.sleep(0.05)
    sdo_write(ser, nid, 0x6040, 0, 0x0080, 2); time.sleep(0.3)
    sdo_write(ser, nid, 0x6040, 0, 0x0000, 2); time.sleep(0.2)
    sdo_write(ser, nid, 0x200F, 0, 0, 2); time.sleep(0.05)
    sdo_write(ser, nid, 0x6060, 0, 3, 1); time.sleep(0.05)
    sdo_write(ser, nid, 0x6083, 1, 500, 4)
    sdo_write(ser, nid, 0x6083, 2, 500, 4)
    sdo_write(ser, nid, 0x6084, 1, 500, 4)
    sdo_write(ser, nid, 0x6084, 2, 500, 4)
    sdo_write(ser, nid, 0x60FF, 1, 0, 4)
    sdo_write(ser, nid, 0x60FF, 2, 0, 4)
    for cw in [0x06, 0x07, 0x0F, 0x0F]:
        sdo_write(ser, nid, 0x6040, 0, cw, 2); time.sleep(0.1)


# 用户指定的测试顺序:左前 → 左后 → 右后 → 右前 (沿车身逆时针一圈)
WHEELS_ORDER = [
    ('FL 左前', 3, 2),
    ('RL 左后', 2, 1),
    ('RR 右后', 2, 2),
    ('FR 右前', 3, 1),
]
RPM = 30
HOLD_S = 3.0       # 每个轮子转 3 秒,看清楚
PAUSE_S = 1.0      # 间隔 1 秒


def main():
    print('=' * 60)
    print('  ZLAC 4 轮方向验证')
    print('  顺序: 左前 → 左后 → 右后 → 右前, 各 +30 RPM 转 3 秒')
    print('=' * 60)

    ser = serial.Serial('/dev/ttyACM0', 921600, timeout=0.3)
    time.sleep(0.2)
    ser.write(bytes([0x55, 0x05, 0x03, 0xAA, 0x55]))  # 500K
    time.sleep(0.5)
    ser.read_all()

    send_can(ser, 0x000, [0x01, 0x00, 0, 0, 0, 0, 0, 0])
    time.sleep(0.3)

    print('\n[使能 Node 2 & Node 3]')
    for nid in [2, 3]:
        enable_zlac(ser, nid)
    print('  ✓ 使能完成\n')

    # 依次单独转
    for idx, (name, nid, sub) in enumerate(WHEELS_ORDER, 1):
        print(f'[{idx}/4] {name} (Node {nid} sub {sub}) +{RPM} RPM 转 {HOLD_S}s')
        # 先把所有 4 个通道都发 0 (确保其他不动)
        for _, n2, s2 in WHEELS_ORDER:
            if n2 != nid or s2 != sub:
                send_velocity(ser, n2, s2, 0)
        # 目标轮持续发 +RPM
        t_end = time.time() + HOLD_S
        while time.time() < t_end:
            send_velocity(ser, nid, sub, RPM)
            # 同时其他 3 个也不停发 0 保持静止
            for _, n2, s2 in WHEELS_ORDER:
                if n2 != nid or s2 != sub:
                    send_velocity(ser, n2, s2, 0)
            time.sleep(0.05)
        # 停顿 1 秒
        print(f'       → 停 {PAUSE_S}s')
        t_end = time.time() + PAUSE_S
        while time.time() < t_end:
            for _, n2, s2 in WHEELS_ORDER:
                send_velocity(ser, n2, s2, 0)
            time.sleep(0.05)

    # 失能
    print('\n[失能]')
    for nid in [2, 3]:
        sdo_write(ser, nid, 0x6040, 0, 0x0000, 2)

    ser.close()
    print('=' * 60)
    print('  完成. 请你判断 4 个轮子每个"正转 +RPM"时的实际方向:')
    print('    FL (1): 转向应该是?')
    print('    RL (2): 转向应该是?')
    print('    RR (3): 转向应该是?')
    print('    FR (4): 转向应该是?')
    print('=' * 60)


if __name__ == '__main__':
    main()
