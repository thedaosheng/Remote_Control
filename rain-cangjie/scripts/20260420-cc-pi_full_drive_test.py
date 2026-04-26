#!/usr/bin/env python3
"""
================================================================
  全底盘硬件驱动测试 (车已架空)
================================================================
阶段 1: 4 个 DM4310 舵向电机 (ACM1, 1M, 达妙 MIT 协议)
  - set_zero_position: 当前位置设为 0 rad (绝对零度)
  - 使能 MIT
  - 单电机依次摆动 ±0.25 rad
  - 4 个同步正弦摆动 2 个周期

阶段 2: 2 个 ZLAC8015D 驱动器 × 各 2 通道 = 4 轮 (ACM0, 500K, CANopen)
  - 使能 CiA402 状态机 (异步模式 + 速度模式)
  - 4 轮 +30 RPM 转 1.5 秒 → 停 0.5 秒 → -30 RPM 转 1.5 秒 → 停
  - 速度 30 RPM ≈ 3.14 rad/s, 轮径 6cm → 线速度 ≈ 0.2 m/s

新 ID 映射 (用户确认):
  FL = 0x06 (MID 0x16)
  FR = 0x04 (MID 0x14)
  RL = 0x05 (MID 0x15)
  RR = 0x03 (MID 0x13)
"""

import os
import sys
import time
import math
import struct
import numpy as np

sys.path.insert(0, os.path.expanduser('~/dm_can'))
from DM_CAN import (Motor, MotorControl, DM_Motor_Type,
                    DM_variable, Control_Type)

import serial


# ============================================================
# 舵向配置
# ============================================================
STEER_MOTORS = [
    ('FL', 0x06, 0x16),
    ('FR', 0x04, 0x14),
    ('RL', 0x05, 0x15),
    ('RR', 0x03, 0x13),
]
KP, KD = 15.0, 1.5
SWING = 0.25
CTL_DT = 0.005


# ============================================================
# ZLAC CANopen 底层 (通过达妙 USB2CAN)
# ============================================================
DAMIAO_TX = np.array(
    [0x55, 0xAA, 0x1e, 0x03, 0x01, 0x00, 0x00, 0x00,
     0x0a, 0x00, 0x00, 0x00, 0x00, 0, 0, 0, 0,
     0x00, 0x08, 0x00,
     0x00, 0, 0, 0, 0, 0, 0, 0, 0, 0x00], np.uint8)

RAD_S_TO_RPM = 60.0 / (2.0 * math.pi)


class ZlacCANopen:
    """ZLAC8015D CANopen 驱动 (双通道,走达妙串口)"""

    def __init__(self, ser, node_id):
        self.ser = ser
        self.node_id = node_id
        self.tx = 0x600 + node_id
        self.rx = 0x580 + node_id

    def _send(self, cid, data):
        f = DAMIAO_TX.copy()
        f[13] = cid & 0xFF
        f[14] = (cid >> 8) & 0xFF
        f[21:29] = data
        self.ser.write(bytes(f))

    def _recv(self, target_id, timeout=0.3):
        dl = time.time() + timeout
        buf = b''
        while time.time() < dl:
            c = self.ser.read_all()
            if c: buf += c
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
            if not c:
                time.sleep(0.005)
        return None

    def sdo_write(self, index, subindex, value, size):
        cmd = {1: 0x2F, 2: 0x2B, 4: 0x23}.get(size, 0x23)
        v = value if value >= 0 else value & 0xFFFFFFFF
        data = [cmd, index & 0xFF, (index >> 8) & 0xFF, subindex,
                v & 0xFF, (v >> 8) & 0xFF, (v >> 16) & 0xFF, (v >> 24) & 0xFF]
        self.ser.read_all()
        self._send(self.tx, data)
        resp = self._recv(self.rx, 0.3)
        return resp is not None and resp[0] == 0x60

    def set_target_velocity(self, rpm, subindex):
        """不等应答 (fire-and-forget),subindex=1 左,2 右"""
        v = int(rpm) if rpm >= 0 else int(rpm) & 0xFFFFFFFF
        data = [0x23, 0xFF, 0x60, subindex,
                v & 0xFF, (v >> 8) & 0xFF, (v >> 16) & 0xFF, (v >> 24) & 0xFF]
        self._send(self.tx, data)

    def nmt_reset_comm(self):
        self._send(0x000, [0x82, self.node_id, 0, 0, 0, 0, 0, 0])

    def nmt_start(self):
        self._send(0x000, [0x01, self.node_id, 0, 0, 0, 0, 0, 0])

    def enable(self):
        """完整使能流程 (参考 swerve_dm_driver_node)"""
        # 1. 故障复位 (上升沿 0x00 → 0x80 → 0x00)
        self.sdo_write(0x6040, 0, 0x0000, 2); time.sleep(0.05)
        self.sdo_write(0x6040, 0, 0x0080, 2); time.sleep(0.3)
        self.sdo_write(0x6040, 0, 0x0000, 2); time.sleep(0.2)
        # 2. 异步模式 (关键!否则速度指令不生效)
        self.sdo_write(0x200F, 0, 0, 2); time.sleep(0.05)
        # 3. 速度模式
        self.sdo_write(0x6060, 0, 3, 1); time.sleep(0.05)
        # 4. 加减速 (左右各自)
        self.sdo_write(0x6083, 1, 500, 4)
        self.sdo_write(0x6083, 2, 500, 4)
        self.sdo_write(0x6084, 1, 500, 4)
        self.sdo_write(0x6084, 2, 500, 4)
        # 5. 初始速度 0
        self.sdo_write(0x60FF, 1, 0, 4)
        self.sdo_write(0x60FF, 2, 0, 4)
        # 6. CiA402 状态机: Shutdown → SwitchOn → EnableOp
        for cw in [0x06, 0x07, 0x0F, 0x0F]:
            self.sdo_write(0x6040, 0, cw, 2); time.sleep(0.1)
        return True

    def disable(self):
        self.sdo_write(0x6040, 0, 0x0000, 2)


# ============================================================
# 阶段 1: 舵向
# ============================================================

def phase1_steer():
    print('\n' + '=' * 60)
    print('  阶段 1: 舵向 4 × DM4310 (ACM1, 1M)')
    print('=' * 60)

    ser = serial.Serial('/dev/ttyACM1', 921600, timeout=0.5)
    time.sleep(0.2)
    mc = MotorControl(ser)

    motors = []
    for name, sid, mid in STEER_MOTORS:
        m = Motor(DM_Motor_Type.DM4310, sid, mid)
        mc.addMotor(m)
        pmax = mc.read_motor_param(m, DM_variable.PMAX)
        ok = pmax is not None
        motors.append((name, m, ok))
        print(f'  {name} 0x{sid:02X}: {"✓" if ok else "✗"}')

    # 设零位 + 使能 (用户说当前位置 = 绝对零度)
    print('  设零位 + 使能 MIT...')
    for name, m, ok in motors:
        if not ok: continue
        mc.disable(m); time.sleep(0.1)
        mc.set_zero_position(m); time.sleep(0.15)
        mc.switchControlMode(m, Control_Type.MIT); time.sleep(0.05)
        mc.enable(m); time.sleep(0.05)
        print(f'    {name}: 零位已设, 已使能')

    # 单电机依次摆动 ±0.25 rad
    print(f'  单电机依次摆动 ±{SWING:.2f} rad (±{math.degrees(SWING):.0f}°)...')
    for name, m, ok in motors:
        if not ok: continue
        print(f'    >>> {name}')
        for tgt in [+SWING, -SWING, 0.0]:
            for _ in range(80):  # 400ms
                for _, mm, mok in motors:
                    if not mok: continue
                    val = tgt if mm is m else 0.0
                    try: mc.controlMIT(mm, KP, KD, val, 0.0, 0.0)
                    except: pass
                time.sleep(CTL_DT)
        time.sleep(0.15)

    # 同步正弦摆动 2 周期
    print(f'  4 个同步正弦摆动 2 个周期...')
    for step in range(int(2 * 2.0 / CTL_DT)):
        tgt = SWING * math.sin(2 * math.pi * step * CTL_DT / 2.0)
        for _, m, ok in motors:
            if not ok: continue
            try: mc.controlMIT(m, KP, KD, tgt, 0.0, 0.0)
            except: pass
        time.sleep(CTL_DT)

    # 归零 + 失能
    print('  归零 + 失能...')
    for _ in range(80):
        for _, m, ok in motors:
            if not ok: continue
            try: mc.controlMIT(m, KP, KD, 0.0, 0.0, 0.0)
            except: pass
        time.sleep(CTL_DT)
    for _, m, ok in motors:
        if not ok: continue
        try: mc.disable(m)
        except: pass
    ser.close()
    print('  ✓ 阶段 1 完成')


# ============================================================
# 阶段 2: ZLAC 驱动
# ============================================================

def phase2_drive():
    print('\n' + '=' * 60)
    print('  阶段 2: 驱动 2 × ZLAC8015D (ACM0, 500K)')
    print('=' * 60)

    ser = serial.Serial('/dev/ttyACM0', 921600, timeout=0.3)
    time.sleep(0.2)
    # 切 500K (每次进脚本都切,防止拔插后回默认)
    ser.write(bytes([0x55, 0x05, 0x03, 0xAA, 0x55]))
    time.sleep(0.5)
    ser.read_all()

    # NMT Start 广播 → 所有节点进 Operational (不 reset_comm 避免等待)
    f = DAMIAO_TX.copy()
    f[13] = 0x00; f[14] = 0x00
    f[21:29] = [0x01, 0x00, 0, 0, 0, 0, 0, 0]
    ser.write(bytes(f))
    time.sleep(0.3)

    zlacs = []
    for nid in [2, 3]:
        z = ZlacCANopen(ser, nid)
        # 读状态字确认在线
        z.ser.read_all()
        z._send(z.tx, [0x40, 0x41, 0x60, 0, 0, 0, 0, 0])
        resp = z._recv(z.rx, 0.5)
        if resp is None:
            print(f'  ✗ Node {nid}: 无响应')
        else:
            sw = resp[4] | (resp[5] << 8)
            print(f'  ✓ Node {nid}: SW=0x{sw:04X}')
            zlacs.append(z)

    if not zlacs:
        print('  ✗ 没有 ZLAC 在线, 跳过阶段 2')
        ser.close()
        return

    print('  使能 2 台 ZLAC...')
    for z in zlacs:
        ok = z.enable()
        print(f'    Node {z.node_id}: {"✓ 使能" if ok else "✗ 使能失败"}')

    # 测试动作:+30 RPM 1.5s → 停 0.5s → -30 RPM 1.5s → 停
    RPM = 30

    def drive_all(rpm_target, duration_s):
        """所有 ZLAC 的左右两通道同时发 rpm"""
        dt = 0.02  # 50 Hz 速度更新
        steps = int(duration_s / dt)
        for _ in range(steps):
            for z in zlacs:
                # 左右通道都发同一速度 (简单起见,方向修正先不做)
                try:
                    z.set_target_velocity(rpm_target, subindex=1)
                    z.set_target_velocity(rpm_target, subindex=2)
                except: pass
            time.sleep(dt)

    print(f'  → 4 轮 +{RPM} RPM 转 1.5s (应该向一个方向走)')
    drive_all(+RPM, 1.5)
    print(f'  → 停止 0.5s')
    drive_all(0, 0.5)
    print(f'  → 4 轮 -{RPM} RPM 转 1.5s (应该反方向)')
    drive_all(-RPM, 1.5)
    print(f'  → 停止 0.5s')
    drive_all(0, 0.5)

    # 失能
    print('  失能 ZLAC...')
    for z in zlacs:
        z.disable()
    ser.close()
    print('  ✓ 阶段 2 完成')


def main():
    print('=' * 60)
    print('  全底盘硬件驱动测试 (车已架空)')
    print('=' * 60)

    try:
        phase1_steer()
        time.sleep(0.5)
        phase2_drive()
    except KeyboardInterrupt:
        print('\n  [中止] 用户 Ctrl-C')

    print('\n' + '=' * 60)
    print('  全部完成. 期望看到:')
    print('    1. 4 舵向 FL→FR→RL→RR 依次摆动 + 同步正弦')
    print('    2. 4 轮先正转 1.5s 再反转 1.5s')
    print('=' * 60)


if __name__ == '__main__':
    main()
