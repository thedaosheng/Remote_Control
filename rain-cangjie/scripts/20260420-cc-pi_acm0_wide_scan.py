#!/usr/bin/env python3
"""
================================================================
  ACM0 宽范围 ID 扫描 — 找驱动电机 (DM3519) 的 CAN ID
================================================================
ACM1 确认是舵向 (0x03-0x06 四个 DM4310 在线)
ACM0 当前 0x03-0x06 都无响应,需要扩大扫描。

扫描范围 0x01 - 0x20 (覆盖达妙私有协议常见段)
如果 DM3519 用"一拖四"风格,反馈可能在 0x201-0x204 段,单独再试。
"""

import os
import sys
import time
import numpy as np
sys.path.insert(0, os.path.expanduser('~/dm_can'))
from DM_CAN import Motor, MotorControl, DM_Motor_Type, DM_variable

import serial


def scan_dm_range(ser, mc, id_start, id_end):
    """用达妙私有协议扫 [id_start, id_end] 段的 SlaveID"""
    online = []
    for sid in range(id_start, id_end + 1):
        mid = sid + 0x10
        m = Motor(DM_Motor_Type.DM4310, sid, mid)
        mc.addMotor(m)
        try:
            pmax = mc.read_motor_param(m, DM_variable.PMAX)
            if pmax is not None:
                vmax = mc.read_motor_param(m, DM_variable.VMAX)
                tmax = mc.read_motor_param(m, DM_variable.TMAX)
                print(f'  0x{sid:02X}: ✓ PMAX={pmax:.2f}  VMAX={vmax:.2f}  TMAX={tmax:.2f}')
                online.append((sid, pmax, vmax, tmax))
        except Exception:
            pass
    return online


def sniff_passive(ser, secs):
    """被动监听几秒,看总线上是否有活跃反馈帧"""
    buf = b''
    deadline = time.time() + secs
    while time.time() < deadline:
        chunk = ser.read(256)
        if chunk:
            buf += chunk
        else:
            time.sleep(0.005)
    # 解析达妙 16 字节接收帧
    ids = {}
    i = 0
    while i <= len(buf) - 16:
        if buf[i] == 0xAA and buf[i + 15] == 0x55:
            cid = (buf[i + 6] << 24) | (buf[i + 5] << 16) | \
                  (buf[i + 4] << 8) | buf[i + 3]
            ids[cid] = ids.get(cid, 0) + 1
            i += 16
        else:
            i += 1
    return ids


def main():
    print('=== /dev/ttyACM0 宽范围扫描 ===\n')
    ser = serial.Serial('/dev/ttyACM0', 921600, timeout=0.3)
    time.sleep(0.2)
    ser.reset_input_buffer()
    mc = MotorControl(ser)

    # 1) 被动监听 2 秒,看有没有心跳帧
    print('[1] 被动监听 2 秒 (看心跳/周期广播)...')
    passive = sniff_passive(ser, 2.0)
    if passive:
        print('    收到帧:')
        for cid, n in sorted(passive.items()):
            print(f'      0x{cid:04X}: {n} 帧')
    else:
        print('    无任何帧 (电机不主动广播,需要主动探测)')

    # 2) 达妙私有协议扫 0x01-0x1F
    print('\n[2] 达妙协议扫 SlaveID 0x01-0x1F...')
    ser.reset_input_buffer()
    online = scan_dm_range(ser, mc, 0x01, 0x1F)
    if not online:
        print('    全段无响应')

    ser.close()

    # 3) 判断
    print('\n=== 总结 ===')
    if online:
        print(f'  ACM0 上找到 {len(online)} 个达妙协议电机:')
        for sid, pmax, vmax, tmax in online:
            print(f'    0x{sid:02X}: PMAX={pmax:.2f} VMAX={vmax:.2f} TMAX={tmax:.2f}')
    else:
        print('  ACM0 上未发现达妙协议电机')
        print('  可能原因:')
        print('   a) DM3519 驱动电机未上电')
        print('   b) DM3519 用"一拖四"协议 (CAN ID 可能 0x200+)')
        print('   c) CAN 总线物理层问题 (终端电阻 / 接线)')
        print('   d) ACM0 这一路 CAN 没接电机')
    return 0


if __name__ == '__main__':
    sys.exit(main())
