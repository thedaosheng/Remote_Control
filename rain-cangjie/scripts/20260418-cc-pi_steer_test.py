#!/usr/bin/env python3
"""
================================================================
  树莓派 4B 舵向电机(DM4310)摆动测试
================================================================
目的:在 Pi 上直接扫描并驱动 4 个转向电机摆动,验证:
  1) USB-CAN (HDSC CDC) 在 Pi 上能枚举
  2) 4 个 DM4310 (FL=0x03, FR=0x06, RL=0x04, RR=0x05) 全部在线
  3) 清错误 → 零位 → 使能 → MIT 控制流程打通
  4) 单电机小幅摆动 + 4 电机同时正弦摆动

运行:
  python3 20260418-cc-pi_steer_test.py

关键参数:
  Kp=15.0  Kd=1.5  (保守增益,先确保不剧烈振荡)
  摆动幅度 ±0.25 rad (约 ±14°,足够肉眼分辨)
"""

import sys
import os
import time
import math
import glob
import subprocess
import numpy as np

# DM_CAN 库路径 (Pi 上约定放在 ~/dm_can/)
sys.path.insert(0, os.path.expanduser('~/dm_can'))
from DM_CAN import (Motor, MotorControl, DM_Motor_Type,
                    DM_variable, Control_Type)

import serial


# ============================================================
# 配置
# ============================================================

# 4 个舵向电机: (名称, SlaveID, MasterID)
# MasterID = SlaveID + 0x10 (项目约定)
MOTORS_CONFIG = [
    ('FL', 0x03, 0x13),   # 左前
    ('FR', 0x06, 0x16),   # 右前
    ('RL', 0x04, 0x14),   # 左后 (已知历史 ERR=0x2)
    ('RR', 0x05, 0x15),   # 右后
]

# MIT 阻抗控制增益(先用保守值)
KP = 15.0      # 位置刚度
KD = 1.5       # 速度阻尼

# 摆动参数
SWING_RAD = 0.25      # 单向幅度 (rad) ≈ ±14°
HOLD_MS = 400         # 每个目标保持时间
CONTROL_DT = 0.005    # 200Hz 控制周期

SWEEP_PERIOD = 2.0    # 同步摆动周期 (秒)
SWEEP_CYCLES = 2      # 摆动 2 个周期


# ============================================================
# 工具函数
# ============================================================

def find_hdsc_port():
    """
    扫描 /dev/ttyACM*,通过 udevadm 查 ID_VENDOR=HDSC
    """
    for dev in sorted(glob.glob('/dev/ttyACM*')):
        try:
            r = subprocess.run(
                ['udevadm', 'info', '-q', 'property', dev],
                capture_output=True, text=True, timeout=2)
            if 'ID_VENDOR=HDSC' in r.stdout:
                return dev
        except Exception:
            continue
    # fallback: 有 ttyACM 就用第一个
    cand = sorted(glob.glob('/dev/ttyACM*'))
    return cand[0] if cand else None


def clear_motor_errors(mc, motor):
    """
    达妙电机错误清除: 发 0xFB 命令 + 失能 → 切 MIT → 使能 → 检查 ERR
    返回 ERR 码 (1=正常)
    """
    for attempt in range(3):
        # 发送清除错误命令 (CAN data = FF FF FF FF FF FF FF FB)
        clear = np.array(
            [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFB],
            np.uint8)
        frame = mc.send_data_frame.copy()
        frame[13] = motor.SlaveID & 0xFF
        frame[14] = (motor.SlaveID >> 8) & 0xFF
        frame[21:29] = clear
        mc.serial_.write(bytes(frame))
        time.sleep(0.3)
        mc.serial_.read_all()

        # 失能 → 切模式 → 使能
        mc.disable(motor); time.sleep(0.1)
        mc.switchControlMode(motor, Control_Type.MIT); time.sleep(0.1)
        mc.enable(motor); time.sleep(0.1)

        # 探查 ERR: 发零力矩触发反馈帧
        mc.serial_.reset_input_buffer()
        mc.controlMIT(motor, 0.0, 0.5, 0.0, 0.0, 0.0)
        time.sleep(0.05)
        raw = mc.serial_.read_all()

        err = -1
        for i in range(len(raw) - 15):
            if raw[i] == 0xAA and raw[i + 15] == 0x55:
                err = (raw[i + 7] >> 4) & 0x0F
                break

        if err == 1:
            return err
        mc.disable(motor); time.sleep(0.3)
    return err


def hold_position(mc, motors_on, target, duration_ms):
    """
    在指定时间内对所有在线电机持续发 MIT 指令 → 稳定到目标角度
    """
    n_steps = int(duration_ms / 1000.0 / CONTROL_DT)
    for _ in range(n_steps):
        for m in motors_on:
            try:
                mc.controlMIT(m, KP, KD, target, 0.0, 0.0)
            except Exception:
                pass
        time.sleep(CONTROL_DT)


# ============================================================
# 主流程
# ============================================================

def main():
    print('=' * 60)
    print('  树莓派 4B 舵向电机(DM4310)摆动测试')
    print('=' * 60)

    # ---- 子任务1: USB-CAN 枚举 ----
    port = find_hdsc_port()
    if port is None:
        print('[ERR] 未找到 USB-CAN 设备!')
        return 1
    print(f'[1/5] USB-CAN 设备: {port}')

    # ---- 子任务2: 打开串口 + 扫描 4 电机 ----
    ser = serial.Serial(port, 921600, timeout=0.5)
    mc = MotorControl(ser)

    motors = []
    online = []
    for name, sid, mid in MOTORS_CONFIG:
        m = Motor(DM_Motor_Type.DM4310, sid, mid)
        mc.addMotor(m)
        pmax = mc.read_motor_param(m, DM_variable.PMAX)
        is_on = pmax is not None
        motors.append(m)
        online.append(is_on)
        print(f'       {name} 0x{sid:02X}: '
              f'{"✓ 在线 (PMAX=%.2f)" % pmax if is_on else "✗ 离线"}')

    on_count = sum(online)
    if on_count == 0:
        print('[ERR] 没有电机在线!')
        ser.close()
        return 1
    print(f'[2/5] 在线电机: {on_count}/4')

    # ---- 子任务3: 清错误 + 设零位 + 使能 ----
    print('[3/5] 清错误 + 设零位 + 使能...')
    motors_on = []
    for i, m in enumerate(motors):
        if not online[i]:
            continue
        name = MOTORS_CONFIG[i][0]
        err = clear_motor_errors(mc, m)
        tag = '✓' if err == 1 else f'⚠ ERR=0x{err:X}'
        # 设零位: 把当前位置定义为 0 rad
        mc.disable(m); time.sleep(0.1)
        mc.set_zero_position(m); time.sleep(0.15)
        mc.switchControlMode(m, Control_Type.MIT); time.sleep(0.05)
        mc.enable(m); time.sleep(0.05)
        motors_on.append(m)
        print(f'       {name}: {tag}, 已使能, 零位已设')

    if not motors_on:
        print('[ERR] 没有电机完成使能!')
        ser.close()
        return 1

    # ---- 子任务4: 单电机逐个摆动 ----
    print(f'[4/5] 单电机逐个摆动 (±{SWING_RAD:.2f} rad ≈ ±{math.degrees(SWING_RAD):.0f}°)')
    for i, m in enumerate(motors):
        if not online[i]:
            continue
        name = MOTORS_CONFIG[i][0]
        # 其他电机保持 0 位
        others = [motors_on[j] for j, _ in enumerate(motors_on) if motors_on[j] != m]
        print(f'       >>> {name} 摆动中...')
        for target in [+SWING_RAD, -SWING_RAD, 0.0]:
            n_steps = int(HOLD_MS / 1000.0 / CONTROL_DT)
            for _ in range(n_steps):
                for mm in motors_on:
                    tgt = target if mm is m else 0.0
                    try:
                        mc.controlMIT(mm, KP, KD, tgt, 0.0, 0.0)
                    except Exception:
                        pass
                time.sleep(CONTROL_DT)
        time.sleep(0.2)

    # ---- 子任务5: 4 电机同步正弦摆动 ----
    print(f'[5/5] 所有在线电机同步正弦摆动 {SWEEP_CYCLES} 个周期...')
    total_steps = int(SWEEP_CYCLES * SWEEP_PERIOD / CONTROL_DT)
    for step in range(total_steps):
        phase = 2 * math.pi * step * CONTROL_DT / SWEEP_PERIOD
        tgt = SWING_RAD * math.sin(phase)
        for m in motors_on:
            try:
                mc.controlMIT(m, KP, KD, tgt, 0.0, 0.0)
            except Exception:
                pass
        time.sleep(CONTROL_DT)

    # ---- 归零 + 失能 ----
    print('[END] 归零 + 失能...')
    hold_position(mc, motors_on, 0.0, 400)
    for m in motors_on:
        try:
            mc.disable(m)
            time.sleep(0.03)
        except Exception:
            pass
    ser.close()
    print('=' * 60)
    print('  测试完成!应该看到 4 个舵向电机依次摆动 + 同步摆动')
    print('=' * 60)
    return 0


if __name__ == '__main__':
    sys.exit(main())
