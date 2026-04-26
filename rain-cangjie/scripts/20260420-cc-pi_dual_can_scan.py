#!/usr/bin/env python3
"""
================================================================
  双路 CAN 电机扫描 — 区分舵向路 vs 驱动路
================================================================
现场:HDSC USB2CAN 双路适配器,Linux 枚举为 /dev/ttyACM0 和 ttyACM1。
     两路物理隔离,各挂 4 个达妙电机,ID 均为 0x03/0x04/0x05/0x06。

本脚本分别在两路上扫描 SlaveID 0x03-0x06,读 PMAX + VMAX + 电机类型线索,
推断哪一路是舵向 (DM4310) 哪一路是驱动 (DM3519)。

判别思路:PMAX/VMAX 不同
  - DM4310: PMAX ≈ 12.5 rad,VMAX ≈ 30 rad/s
  - DM3519: PMAX 通常 很大 (是连续旋转电机,位置可能无限) 或 12.56,
           VMAX 较高 (~50+ rad/s)
"""

import os
import sys
import time
sys.path.insert(0, os.path.expanduser('~/dm_can'))
from DM_CAN import Motor, MotorControl, DM_Motor_Type, DM_variable

import serial


def scan_one_port(port, motor_type, type_name):
    """扫 1 路 CAN 上的 4 个电机,返回每个的 PMAX/VMAX/TMAX"""
    print(f'\n=== {port} ({type_name}) ===')
    try:
        ser = serial.Serial(port, 921600, timeout=0.5)
    except Exception as e:
        print(f'  打开失败: {e}')
        return []

    mc = MotorControl(ser)
    results = []
    for sid in [0x03, 0x04, 0x05, 0x06]:
        mid = sid + 0x10
        m = Motor(motor_type, sid, mid)
        mc.addMotor(m)
        try:
            pmax = mc.read_motor_param(m, DM_variable.PMAX)
            vmax = mc.read_motor_param(m, DM_variable.VMAX)
            tmax = mc.read_motor_param(m, DM_variable.TMAX)
            if pmax is not None:
                print(f'  0x{sid:02X}: ✓ PMAX={pmax:.3f}  VMAX={vmax:.3f}  TMAX={tmax:.3f}')
                results.append((sid, pmax, vmax, tmax))
            else:
                print(f'  0x{sid:02X}: ✗ 离线')
        except Exception as e:
            print(f'  0x{sid:02X}: 探测异常 {e}')
    ser.close()
    return results


def main():
    # 先用 DM4310 类型探测 ACM0,再 DM3519 类型探测 ACM1
    # (DM_Motor_Type 的区别主要是 PMAX/VMAX/TMAX 默认值,对探测不影响)
    r0 = scan_one_port('/dev/ttyACM0', DM_Motor_Type.DM4310, 'DM4310 type probe')
    r1 = scan_one_port('/dev/ttyACM1', DM_Motor_Type.DM4310, 'DM4310 type probe')

    print('\n=== 结论 ===')
    if r0 and not r1:
        print('  只有 /dev/ttyACM0 有响应 (可能对应舵向)')
    elif r1 and not r0:
        print('  只有 /dev/ttyACM1 有响应 (可能对应驱动)')
    elif r0 and r1:
        print(f'  ACM0: {len(r0)}/4 在线, ACM1: {len(r1)}/4 在线')
        # 对比 PMAX 判断类型
        pmax0 = r0[0][1] if r0 else 0
        pmax1 = r1[0][1] if r1 else 0
        vmax0 = r0[0][2] if r0 else 0
        vmax1 = r1[0][2] if r1 else 0
        print(f'  ACM0 典型 PMAX={pmax0:.2f} VMAX={vmax0:.2f}')
        print(f'  ACM1 典型 PMAX={pmax1:.2f} VMAX={vmax1:.2f}')
        # DM3519 的 VMAX 通常比 DM4310 大很多 (>40 rad/s)
        if vmax0 > vmax1 + 10:
            print('  推测 ACM0 = 驱动 (3519 高速), ACM1 = 舵向 (4310)')
        elif vmax1 > vmax0 + 10:
            print('  推测 ACM0 = 舵向 (4310), ACM1 = 驱动 (3519 高速)')
        else:
            print('  PMAX/VMAX 相近,无法从参数区分。需要实际测运动。')
    else:
        print('  两路都无响应!')

    return 0


if __name__ == '__main__':
    sys.exit(main())
