#!/usr/bin/env python3
"""
================================================================
  DM4310 舵向零位持久化 (两步安全模式)
================================================================
用法:
  步骤 1: python3 this.py             # 只读当前位置,不改 Flash
  步骤 2: python3 this.py --commit    # 确认后: set_zero + 写 Flash

流程 (参考 DM_CAN.py):
  - set_zero_position  (0xFE) — 把当前物理位置定义为 0 rad, 只写 RAM
  - save_motor_param   (0xAA → 0x7FF) — 保存所有参数到 Flash (持久化)

正确步骤:
  1. 用手把 4 个轮子转到"车头方向"的直行姿态
  2. 跑 step1 确认当前位置大致对齐 (读到的数字没意义, 只是参考)
  3. 跑 step2 --commit 固化
  4. 断电重启 → 位置应该归零到你刚才摆正的姿态
"""

import os, sys, time
sys.path.insert(0, os.path.expanduser('~/dm_can'))
from DM_CAN import Motor, MotorControl, DM_Motor_Type, DM_variable, Control_Type
import serial


STEER_CONFIG = [
    ('FL', 0x06, 0x16),
    ('FR', 0x04, 0x14),
    ('RL', 0x05, 0x15),
    ('RR', 0x03, 0x13),
]


def main():
    commit = '--commit' in sys.argv

    print('=' * 60)
    print('  DM4310 舵向零位持久化')
    print(f'  模式: {"🔴 COMMIT (会写 Flash!)" if commit else "🟢 READ ONLY (只读)"}')
    print('=' * 60 + '\n')

    ser = serial.Serial('/dev/ttyACM1', 921600, timeout=0.5)
    time.sleep(0.2)
    mc = MotorControl(ser)

    motors = []
    for name, sid, mid in STEER_CONFIG:
        m = Motor(DM_Motor_Type.DM4310, sid, mid)
        mc.addMotor(m)
        pmax = mc.read_motor_param(m, DM_variable.PMAX)
        motors.append(m if pmax is not None else None)

    # ---- 步骤 A: 使能读当前位置 ----
    print('[读取当前位置]')
    for i, (name, sid, _) in enumerate(STEER_CONFIG):
        m = motors[i]
        if m is None:
            print(f'  ✗ {name}: 离线'); continue
        mc.disable(m); time.sleep(0.05)
        mc.switchControlMode(m, Control_Type.MIT); time.sleep(0.05)
        mc.enable(m); time.sleep(0.05)
        # 发零力矩触发反馈帧
        mc.controlMIT(m, 0.0, 0.5, 0.0, 0.0, 0.0); time.sleep(0.08)
        pos = m.getPosition()
        import math
        print(f'  {name} 0x{sid:02X}: 位置 = {pos:+.4f} rad  ({math.degrees(pos):+.2f}°)')
        # 读位置后立刻失能,避免意外转动
        mc.disable(m); time.sleep(0.05)

    if not commit:
        ser.close()
        print('\n' + '=' * 60)
        print('  只读模式. 如果位置对应你期望的"车头方向":')
        print('    请用手摆正 4 个轮子, 然后跑:')
        print('      python3 this.py --commit')
        print('  它会把当前物理位置固化为 Flash 里的零点')
        print('=' * 60)
        return

    # ---- 步骤 B: 持久化 (危险!只在 --commit 时执行) ----
    print('\n[🔴 持久化到 Flash]')
    for i, (name, sid, _) in enumerate(STEER_CONFIG):
        m = motors[i]
        if m is None: continue
        # 1. disable
        mc.disable(m); time.sleep(0.1)
        # 2. set_zero_position (0xFE)
        mc.set_zero_position(m); time.sleep(0.2)
        # 3. save_motor_param (0xAA → 0x7FF, 写 Flash)
        mc.save_motor_param(m); time.sleep(0.3)
        print(f'  ✓ {name}: 零位已写入 Flash')

    # ---- 验证 ----
    print('\n[验证新零位]')
    import math
    for i, (name, sid, _) in enumerate(STEER_CONFIG):
        m = motors[i]
        if m is None: continue
        mc.switchControlMode(m, Control_Type.MIT); time.sleep(0.05)
        mc.enable(m); time.sleep(0.05)
        mc.controlMIT(m, 0.0, 0.5, 0.0, 0.0, 0.0); time.sleep(0.08)
        pos = m.getPosition()
        status = '✓' if abs(pos) < 0.05 else '⚠'
        print(f'  {status} {name}: 位置 = {pos:+.4f} rad ({math.degrees(pos):+.2f}°)')
        mc.disable(m); time.sleep(0.05)

    ser.close()
    print('\n' + '=' * 60)
    print('  完成! 断电重启后零位应仍保持 = 0')
    print('=' * 60)


if __name__ == '__main__':
    main()
