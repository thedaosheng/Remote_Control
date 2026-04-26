#!/usr/bin/env python3
"""
================================================================
  全硬件扫描 — 确认 ACM0 ZLAC×2 + ACM1 4310×4 + 3519×1 都在线
================================================================
预期结果:
  ACM0 (CANopen 500K):
    - Node 2: ZLAC 驱动器 A
    - Node 3: ZLAC 驱动器 B
  ACM1 (达妙协议 1M):
    - SlaveID 0x03/0x04/0x05/0x06: DM4310 舵向 × 4
    - SlaveID 0x01 (MasterID 0x101): DM3519 一拖四 (MIT 模式)
"""

import os
import sys
import time
import struct
import numpy as np
sys.path.insert(0, os.path.expanduser('~/dm_can'))
from DM_CAN import Motor, MotorControl, DM_Motor_Type, DM_variable

import serial

# ============================================================
# 达妙 USB2CAN 帧工具 (用于 CANopen SDO 手动封装)
# ============================================================
DAMIAO_TX = np.array(
    [0x55, 0xAA, 0x1e, 0x03, 0x01, 0x00, 0x00, 0x00,
     0x0a, 0x00, 0x00, 0x00, 0x00, 0, 0, 0, 0,
     0x00, 0x08, 0x00,
     0x00, 0, 0, 0, 0, 0, 0, 0, 0, 0x00], np.uint8)


def send_can(ser, can_id, data8):
    f = DAMIAO_TX.copy()
    f[13] = can_id & 0xFF
    f[14] = (can_id >> 8) & 0xFF
    f[21:29] = data8
    ser.write(bytes(f))


def recv_filtered(ser, target_id, timeout=0.3):
    deadline = time.time() + timeout
    buf = b''
    while time.time() < deadline:
        chunk = ser.read_all()
        if chunk:
            buf += chunk
        i = 0
        while i <= len(buf) - 16:
            if buf[i] == 0xAA and buf[i + 15] == 0x55:
                cid = (buf[i + 6] << 24) | (buf[i + 5] << 16) | \
                      (buf[i + 4] << 8) | buf[i + 3]
                if cid == target_id:
                    return buf[i + 7:i + 15]
                i += 16
            else:
                i += 1
        buf = buf[max(0, len(buf) - 15):]
        if not chunk:
            time.sleep(0.005)
    return None


def sdo_read(ser, node_id, index, subindex=0, timeout=0.3):
    tx = 0x600 + node_id
    rx = 0x580 + node_id
    ser.read_all()
    send_can(ser, tx,
             [0x40, index & 0xFF, (index >> 8) & 0xFF, subindex, 0, 0, 0, 0])
    resp = recv_filtered(ser, rx, timeout)
    if resp is None:
        return None
    if resp[0] in (0x4F, 0x4B, 0x43, 0x42):
        return struct.unpack_from('<I', resp, 4)[0]
    return None


# ============================================================
# ACM0: ZLAC 扫描
# ============================================================

def scan_zlac(port='/dev/ttyACM0'):
    print(f'=== ACM0: ZLAC CANopen 扫描 (500K) ===')
    ser = serial.Serial(port, 921600, timeout=0.3)
    time.sleep(0.2)
    ser.reset_input_buffer()
    # 切 500K
    ser.write(bytes([0x55, 0x05, 0x03, 0xAA, 0x55]))
    time.sleep(0.5)
    ser.read_all()
    # NMT Start 广播
    send_can(ser, 0x000, [0x01, 0x00, 0, 0, 0, 0, 0, 0])
    time.sleep(0.3)

    found = []
    for nid in range(1, 9):
        sw = sdo_read(ser, nid, 0x6041, 0, timeout=0.2)
        if sw is not None:
            sw_l = sw & 0xFFFF
            sw_r = (sw >> 16) & 0xFFFF
            # 读当前 Node ID 寄存器二次确认
            canid = sdo_read(ser, nid, 0x200A, 0, timeout=0.15)
            canid_v = (canid & 0xFFFF) if canid is not None else '?'
            found.append((nid, sw_l, sw_r, canid_v))
            print(f'  ✓ Node {nid}: SW_L=0x{sw_l:04X} SW_R=0x{sw_r:04X}  '
                  f'0x200A={canid_v}')
        else:
            print(f'    Node {nid}: --')
    ser.close()

    print(f'  → 发现 {len(found)} 台 ZLAC')
    return found


# ============================================================
# ACM1: 达妙电机扫描 (DM4310 舵向 + DM3519 一拖四)
# ============================================================

def scan_damiao(port='/dev/ttyACM1'):
    print(f'\n=== ACM1: 达妙协议扫描 (1M) ===')
    ser = serial.Serial(port, 921600, timeout=0.3)
    time.sleep(0.2)
    ser.reset_input_buffer()
    # 不切波特率 — 达妙双路适配器 ACM1 保持默认 1M,
    # 避免错误切换命令把 ACM1 搞坏

    mc = MotorControl(ser)
    found = []

    # 舵向 4 个 DM4310: SlaveID 0x03-0x06
    print('  [4310 舵向] SlaveID 0x03-0x06:')
    for sid in [0x03, 0x04, 0x05, 0x06]:
        mid = sid + 0x10
        m = Motor(DM_Motor_Type.DM4310, sid, mid)
        mc.addMotor(m)
        pmax = mc.read_motor_param(m, DM_variable.PMAX)
        if pmax is not None:
            vmax = mc.read_motor_param(m, DM_variable.VMAX)
            print(f'    ✓ 0x{sid:02X} (MID=0x{mid:02X}): '
                  f'PMAX={pmax:.2f} VMAX={vmax:.2f}')
            found.append(('4310', sid, mid, pmax, vmax))
        else:
            print(f'      0x{sid:02X}: --')

    # DM3519 一拖四: SlaveID 0x01, MasterID 0x101 (用户给的)
    # 一拖四是特殊固件,可能 read_motor_param 不工作,
    # 所以先试标准协议,失败则仅声明地址
    print('  [3519 一拖四] SlaveID 0x01 MasterID 0x101:')
    m3519 = Motor(DM_Motor_Type.DM4310, 0x01, 0x101)  # 类型用 4310 兜底
    mc.addMotor(m3519)
    pmax = mc.read_motor_param(m3519, DM_variable.PMAX)
    if pmax is not None:
        vmax = mc.read_motor_param(m3519, DM_variable.VMAX)
        print(f'    ✓ 0x01: PMAX={pmax:.2f} VMAX={vmax:.2f} (标准协议响应)')
        found.append(('3519', 0x01, 0x101, pmax, vmax))
    else:
        # 被动监听 0.5 秒,看 0x101 有没有广播
        print(f'    read_motor_param 无响应 (一拖四可能不支持参数读)')
        ser.reset_input_buffer()
        t0 = time.time()
        buf = b''
        while time.time() - t0 < 0.5:
            c = ser.read_all()
            if c: buf += c
            else: time.sleep(0.01)
        # 解析,找 CAN ID = 0x101 的帧
        seen_101 = 0
        i = 0
        while i <= len(buf) - 16:
            if buf[i] == 0xAA and buf[i + 15] == 0x55:
                cid = (buf[i + 6] << 24) | (buf[i + 5] << 16) | \
                      (buf[i + 4] << 8) | buf[i + 3]
                if cid == 0x101:
                    seen_101 += 1
                i += 16
            else:
                i += 1
        if seen_101:
            print(f'    ⚠ 0x101 被动监听到 {seen_101} 帧 (3519 可能在广播反馈)')
        else:
            print(f'    ✗ 未识别 3519 (可能需要一拖四专用协议帧去唤醒)')

    ser.close()
    return found


# ============================================================
# 主流程
# ============================================================

def main():
    zlacs = scan_zlac()
    damiaos = scan_damiao()

    print('\n' + '=' * 56)
    print('  硬件完整性总结')
    print('=' * 56)
    print(f'  ZLAC 驱动器: {len(zlacs)}/2 在线')
    for nid, sw_l, sw_r, canid in zlacs:
        print(f'    Node {nid}: 0x200A={canid}')

    num_4310 = sum(1 for x in damiaos if x[0] == '4310')
    num_3519 = sum(1 for x in damiaos if x[0] == '3519')
    print(f'  DM4310 舵向: {num_4310}/4 在线')
    print(f'  DM3519 一拖四: {num_3519}/1 标准协议可读')

    if len(zlacs) == 2 and num_4310 == 4:
        print('\n  ✓ 核心硬件 (2 ZLAC + 4 4310) 全部在线')
        if num_3519 == 1:
            print('  ✓ 3519 也能通过标准达妙协议读参数')
        else:
            print('  ⚠ 3519 需要一拖四专用驱动例程')
    return 0


if __name__ == '__main__':
    sys.exit(main())
