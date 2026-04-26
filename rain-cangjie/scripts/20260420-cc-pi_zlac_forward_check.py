#!/usr/bin/env python3
"""
================================================================
  ZLAC 4 轮"同向前进"验证 (带极性修正)
================================================================
假设左右轮镜像安装,需要 direction = [+1 左, -1 右]

发"前进 30 RPM"指令时:
  FL (Node3 sub2) → 发 +30 RPM
  RL (Node2 sub1) → 发 +30 RPM
  RR (Node2 sub2) → 发 -30 RPM  (极性反)
  FR (Node3 sub1) → 发 -30 RPM  (极性反)

期望:4 个轮子都朝车头方向滚,车如果不架空会直线前进。
"""

import os, sys, time
import numpy as np
import serial


DAMIAO_TX = np.array(
    [0x55, 0xAA, 0x1e, 0x03, 0x01, 0x00, 0x00, 0x00,
     0x0a, 0x00, 0x00, 0x00, 0x00, 0, 0, 0, 0,
     0x00, 0x08, 0x00,
     0x00, 0, 0, 0, 0, 0, 0, 0, 0, 0x00], np.uint8)


def send_can(ser, cid, data):
    f = DAMIAO_TX.copy()
    f[13] = cid & 0xFF; f[14] = (cid >> 8) & 0xFF
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
            else: i += 1
        buf = buf[max(0, len(buf) - 15):]
        if not c: time.sleep(0.005)
    return None


def sdo_write(ser, nid, idx, sub, val, size):
    cmd = {1: 0x2F, 2: 0x2B, 4: 0x23}.get(size, 0x23)
    v = val if val >= 0 else val & 0xFFFFFFFF
    data = [cmd, idx & 0xFF, (idx >> 8) & 0xFF, sub,
            v & 0xFF, (v >> 8) & 0xFF, (v >> 16) & 0xFF, (v >> 24) & 0xFF]
    ser.read_all()
    send_can(ser, 0x600 + nid, data)
    resp = recv_filtered(ser, 0x580 + nid, 0.3)
    return resp is not None and resp[0] == 0x60


def send_velocity(ser, nid, sub, rpm):
    v = int(rpm) if rpm >= 0 else int(rpm) & 0xFFFFFFFF
    data = [0x23, 0xFF, 0x60, sub,
            v & 0xFF, (v >> 8) & 0xFF, (v >> 16) & 0xFF, (v >> 24) & 0xFF]
    send_can(ser, 0x600 + nid, data)


def enable_zlac(ser, nid):
    sdo_write(ser, nid, 0x6040, 0, 0x0000, 2); time.sleep(0.05)
    sdo_write(ser, nid, 0x6040, 0, 0x0080, 2); time.sleep(0.3)
    sdo_write(ser, nid, 0x6040, 0, 0x0000, 2); time.sleep(0.2)
    sdo_write(ser, nid, 0x200F, 0, 0, 2); time.sleep(0.05)
    sdo_write(ser, nid, 0x6060, 0, 3, 1); time.sleep(0.05)
    for sub in [1, 2]:
        sdo_write(ser, nid, 0x6083, sub, 500, 4)
        sdo_write(ser, nid, 0x6084, sub, 500, 4)
        sdo_write(ser, nid, 0x60FF, sub, 0, 4)
    for cw in [0x06, 0x07, 0x0F, 0x0F]:
        sdo_write(ser, nid, 0x6040, 0, cw, 2); time.sleep(0.1)


# (名称, Node ID, subindex, 方向系数)
WHEELS = [
    ('FL 左前', 3, 2, +1),  # 左侧: 正向
    ('RL 左后', 2, 1, +1),  # 左侧: 正向
    ('RR 右后', 2, 2, -1),  # 右侧: 反向
    ('FR 右前', 3, 1, -1),  # 右侧: 反向
]

SPEED_RPM = 30


def drive_forward(ser, rpm_forward, duration_s):
    """发一个车速度(正 = 前进),按 direction 映射到 4 轮"""
    t_end = time.time() + duration_s
    while time.time() < t_end:
        for name, nid, sub, direction in WHEELS:
            send_velocity(ser, nid, sub, rpm_forward * direction)
        time.sleep(0.05)


def main():
    print('=' * 60)
    print('  ZLAC "同向前进" 验证 (direction = [+L +L -R -R])')
    print('=' * 60)

    ser = serial.Serial('/dev/ttyACM0', 921600, timeout=0.3)
    time.sleep(0.2)
    ser.write(bytes([0x55, 0x05, 0x03, 0xAA, 0x55]))
    time.sleep(0.5)
    ser.read_all()
    send_can(ser, 0x000, [0x01, 0x00, 0, 0, 0, 0, 0, 0])
    time.sleep(0.3)

    print('[使能]')
    for nid in [2, 3]:
        enable_zlac(ser, nid)
    print('  ✓')

    print(f'\n[前进 {SPEED_RPM} RPM × 3 秒 — 4 轮应该都朝车头方向滚]')
    drive_forward(ser, +SPEED_RPM, 3.0)
    print('  → 停 1 秒')
    drive_forward(ser, 0, 1.0)

    print(f'\n[后退 {SPEED_RPM} RPM × 3 秒 — 4 轮应该都朝车尾方向滚]')
    drive_forward(ser, -SPEED_RPM, 3.0)
    print('  → 停 1 秒')
    drive_forward(ser, 0, 1.0)

    print('\n[失能]')
    for nid in [2, 3]:
        sdo_write(ser, nid, 0x6040, 0, 0x0000, 2)
    ser.close()

    print('=' * 60)
    print('  请确认:')
    print('    前进时: 4 轮都朝车头方向? ✓ = 极性修正正确')
    print('    后退时: 4 轮都朝车尾方向? ✓ = 极性修正正确')
    print('=' * 60)


if __name__ == '__main__':
    main()
