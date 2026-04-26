#!/usr/bin/env python3
"""
================================================================
  ZLAC 4 轮单独演示 — 依次让每个轮子单独转,看映射是否对
================================================================
映射表 (用户确认):
  FL 左前 = Node 3 sub 2 (Right Motor)
  FR 右前 = Node 3 sub 1 (Left Motor)
  RL 左后 = Node 2 sub 1 (Left Motor)
  RR 右后 = Node 2 sub 2 (Right Motor)

流程:
  1. 使能 2 台 ZLAC
  2. FL 单独转 2 秒 (只发 Node3 sub2, 其他 = 0)
  3. FR 单独转 2 秒
  4. RL 单独转 2 秒
  5. RR 单独转 2 秒
  6. 4 轮同时正转 2 秒 → 停
  7. 4 轮同时反转 2 秒 → 停
  8. 失能
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
    """fire-and-forget 发目标速度 (0x60FF)"""
    tx = 0x600 + nid
    v = int(rpm) if rpm >= 0 else int(rpm) & 0xFFFFFFFF
    data = [0x23, 0xFF, 0x60, sub,
            v & 0xFF, (v >> 8) & 0xFF, (v >> 16) & 0xFF, (v >> 24) & 0xFF]
    send_can(ser, tx, data)


def enable_zlac(ser, nid):
    """完整使能流程"""
    # 1. 故障复位 (上升沿)
    sdo_write(ser, nid, 0x6040, 0, 0x0000, 2); time.sleep(0.05)
    sdo_write(ser, nid, 0x6040, 0, 0x0080, 2); time.sleep(0.3)
    sdo_write(ser, nid, 0x6040, 0, 0x0000, 2); time.sleep(0.2)
    # 2. 异步模式 (必须!)
    sdo_write(ser, nid, 0x200F, 0, 0, 2); time.sleep(0.05)
    # 3. 速度模式
    sdo_write(ser, nid, 0x6060, 0, 3, 1); time.sleep(0.05)
    # 4. 加减速
    sdo_write(ser, nid, 0x6083, 1, 500, 4)
    sdo_write(ser, nid, 0x6083, 2, 500, 4)
    sdo_write(ser, nid, 0x6084, 1, 500, 4)
    sdo_write(ser, nid, 0x6084, 2, 500, 4)
    # 5. 初始 0
    sdo_write(ser, nid, 0x60FF, 1, 0, 4)
    sdo_write(ser, nid, 0x60FF, 2, 0, 4)
    # 6. 状态机
    for cw in [0x06, 0x07, 0x0F, 0x0F]:
        sdo_write(ser, nid, 0x6040, 0, cw, 2); time.sleep(0.1)


# 四个轮子的 (node_id, subindex) 映射
WHEELS = {
    'FL 左前': (3, 2),
    'FR 右前': (3, 1),
    'RL 左后': (2, 1),
    'RR 右后': (2, 2),
}

RPM = 30                  # 测试速度
HOLD_S = 2.0              # 每个轮子单独转的时间
UPDATE_DT = 0.05          # 指令刷新周期 20Hz


def drive_duration(ser, wheel_targets_dict, duration_s):
    """
    在 duration_s 内持续发目标速度。
    wheel_targets_dict: {'FL': rpm, 'FR': rpm, ...}
    未提到的轮子发 0。
    """
    t_end = time.time() + duration_s
    while time.time() < t_end:
        # 4 个轮子都要发 (包括 0)
        for name, (nid, sub) in WHEELS.items():
            rpm = wheel_targets_dict.get(name, 0)
            send_velocity(ser, nid, sub, rpm)
        time.sleep(UPDATE_DT)


def main():
    print('=' * 60)
    print('  ZLAC 4 轮单独演示')
    print('=' * 60)

    ser = serial.Serial('/dev/ttyACM0', 921600, timeout=0.3)
    time.sleep(0.2)
    # 切 500K
    ser.write(bytes([0x55, 0x05, 0x03, 0xAA, 0x55]))
    time.sleep(0.5)
    ser.read_all()

    # NMT Start 广播
    send_can(ser, 0x000, [0x01, 0x00, 0, 0, 0, 0, 0, 0])
    time.sleep(0.3)

    # 使能两台
    print('[1] 使能 Node 2 & Node 3 ...')
    for nid in [2, 3]:
        enable_zlac(ser, nid)
        print(f'    Node {nid}: ✓')

    # ---- 依次单独转 ----
    print(f'\n[2] 依次单独转 {HOLD_S}s (其他保持 0 RPM)')
    for name in ['FL 左前', 'FR 右前', 'RL 左后', 'RR 右后']:
        print(f'    >>> {name} 应该单独转,其他 3 个静止')
        drive_duration(ser, {name: RPM}, HOLD_S)
        print(f'        → 停 0.5s')
        drive_duration(ser, {}, 0.5)

    # ---- 4 轮同时 ----
    print(f'\n[3] 4 轮同时 +{RPM} RPM 转 2s')
    drive_duration(ser, {n: RPM for n in WHEELS}, 2.0)
    drive_duration(ser, {}, 0.5)

    print(f'[4] 4 轮同时 -{RPM} RPM 转 2s')
    drive_duration(ser, {n: -RPM for n in WHEELS}, 2.0)
    drive_duration(ser, {}, 0.5)

    # 失能
    print('\n[5] 失能')
    for nid in [2, 3]:
        sdo_write(ser, nid, 0x6040, 0, 0x0000, 2)

    ser.close()
    print('=' * 60)
    print('  完成. 应该看到:')
    print('    FL → FR → RL → RR 依次单独转,其他静止')
    print('    最后 4 轮同正转 + 同反转')
    print('=' * 60)


if __name__ == '__main__':
    main()
