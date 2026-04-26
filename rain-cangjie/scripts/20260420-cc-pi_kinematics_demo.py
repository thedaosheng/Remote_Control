#!/usr/bin/env python3
"""
================================================================
  舵轮底盘运动学演示 — 7 种经典运动模式
================================================================
架构:
  steer_thread @ 200Hz  → 4 × DM4310 MIT 位置 (ACM1, 1M 达妙)
  drive_thread @ 50Hz   → 2 × ZLAC 速度指令 (ACM0, 500K CANopen)
  主线程负责切换目标 (vx, vy, omega),运行学 IK 由两线程各自解算

演示模式 (每个 ~3 秒):
  1. 前进        (vx = +0.15 m/s)
  2. 后退        (vx = -0.15 m/s)
  3. 横移左      (vy = +0.15 m/s)
  4. 横移右      (vy = -0.15 m/s)
  5. 原地左转    (ω = +0.6 rad/s)
  6. 原地右转    (ω = -0.6 rad/s)
  7. 弧线前+左   (vx=0.1, ω=0.4)

舵轮 IK:
  对每个轮 (lx, ly):
    wx = vx - ω·ly,  wy = vy + ω·lx
    steer = atan2(wy, wx),  speed = hypot(wx, wy)

过渡策略: 每个模式前 0.6 秒舵向转到位 (drive_gate=0),
         然后 1.8 秒带速度跑,0.4 秒回零衔接下一个。
"""

import os, sys, time, math, threading, struct
import numpy as np
sys.path.insert(0, os.path.expanduser('~/dm_can'))
from DM_CAN import Motor, MotorControl, DM_Motor_Type, Control_Type, DM_variable
import serial


# ============================================================
# 硬件配置
# ============================================================

# 舵向 4 × DM4310 (FL, FR, RL, RR 顺序)
STEER_CONFIG = [
    ('FL', 0x06, 0x16),
    ('FR', 0x04, 0x14),
    ('RL', 0x05, 0x15),
    ('RR', 0x03, 0x13),
]

# ZLAC 轮子映射 (Node, subindex, direction)
ZLAC_MAP = [
    (3, 2, +1),   # FL = Node3 Right Motor
    (3, 1, -1),   # FR = Node3 Left Motor  (极性反)
    (2, 1, +1),   # RL = Node2 Left Motor
    (2, 2, -1),   # RR = Node2 Right Motor (极性反)
]

# 轮子几何 (半轴距 ≈ 0.16m, 半轮距 ≈ 0.15m, 轮径 6cm)
WHEEL_POS = np.array([
    [+0.16, +0.15],   # FL
    [+0.16, -0.15],   # FR
    [-0.16, +0.15],   # RL
    [-0.16, -0.15],   # RR
])
WHEEL_R = 0.06

KP, KD = 15.0, 1.5


# ============================================================
# 达妙 USB-CAN 帧
# ============================================================
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


def send_velocity_zlac(ser, nid, sub, rpm):
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


# ============================================================
# 舵轮逆运动学
# ============================================================
def swerve_ik(vx, vy, omega):
    """
    车身系速度 → 4 组 (steer_angle rad, drive_speed rad/s)
    返回: steer[4], rad_s[4]
    """
    steer = np.zeros(4)
    rad_s = np.zeros(4)
    for i, (lx, ly) in enumerate(WHEEL_POS):
        wx = vx - omega * ly
        wy = vy + omega * lx
        v = math.hypot(wx, wy)
        if v > 1e-4:
            steer[i] = math.atan2(wy, wx)
            rad_s[i] = v / WHEEL_R
    return steer, rad_s


# 用于避免舵轮大角度回转:如果新目标和当前差 > 90°,反转角度 + 反转驱动
def optimize_wheel(new_steer, prev_steer, speed):
    """>90° 翻转: 舵角转半圈 → 反转驱动,轮子实际方向一样"""
    delta = (new_steer - prev_steer + math.pi) % (2 * math.pi) - math.pi
    if abs(delta) > math.pi / 2:
        new_steer = (new_steer + math.pi + math.pi) % (2 * math.pi) - math.pi
        speed = -speed
    return new_steer, speed


# ============================================================
# 共享状态 & 线程
# ============================================================
state = {'vx': 0.0, 'vy': 0.0, 'omega': 0.0, 'drive_gate': 0.0}
state_lock = threading.Lock()
running = True
cur_steer = np.zeros(4)  # 当前舵向角度,给 optimize_wheel 用


def steer_loop(mc, motors):
    """200Hz MIT 位置控制"""
    global cur_steer
    while running:
        with state_lock:
            vx, vy, om = state['vx'], state['vy'], state['omega']
        steer, _ = swerve_ik(vx, vy, om)
        # >90° 优化
        for i in range(4):
            steer[i], _ = optimize_wheel(steer[i], cur_steer[i], 1.0)
        cur_steer = steer
        for i, m in enumerate(motors):
            if m is None: continue
            try:
                mc.controlMIT(m, KP, KD, float(steer[i]), 0.0, 0.0)
            except Exception:
                pass
        time.sleep(0.005)


def drive_loop(ser):
    """50Hz ZLAC 速度指令"""
    RAD_S_TO_RPM = 60.0 / (2.0 * math.pi)
    while running:
        with state_lock:
            vx, vy, om = state['vx'], state['vy'], state['omega']
            gate = state['drive_gate']
        steer, rad_s = swerve_ik(vx, vy, om)
        # 同样做 90° 反转对齐
        for i in range(4):
            _, rad_s[i] = optimize_wheel(steer[i], cur_steer[i], rad_s[i])
        for i, (nid, sub, direction) in enumerate(ZLAC_MAP):
            rpm = int(rad_s[i] * RAD_S_TO_RPM * direction * gate)
            rpm = max(-80, min(80, rpm))  # 限幅
            send_velocity_zlac(ser, nid, sub, rpm)
        time.sleep(0.02)


# ============================================================
# Pattern 执行
# ============================================================
def set_target(vx, vy, omega, gate):
    with state_lock:
        state['vx'] = vx
        state['vy'] = vy
        state['omega'] = omega
        state['drive_gate'] = gate


def pattern(name, vx, vy, omega, run_s=1.8, steer_align=0.6, settle=0.4):
    """一个运动模式:先舵向对齐 → 驱动运动 → 停止衔接"""
    print(f'\n>>> {name}  (vx={vx:+.2f} vy={vy:+.2f} ω={omega:+.2f})')
    print(f'   [舵向对齐 {steer_align}s]')
    set_target(vx, vy, omega, 0.0)
    time.sleep(steer_align)
    print(f'   [运动 {run_s}s]')
    set_target(vx, vy, omega, 1.0)
    time.sleep(run_s)
    print(f'   [停止 {settle}s]')
    set_target(0, 0, 0, 0.0)
    time.sleep(settle)


# ============================================================
# 主流程
# ============================================================
def main():
    global running

    print('=' * 60)
    print('  舵轮底盘运动学演示 (7 种模式)')
    print('=' * 60)

    # ---- ACM1: 4310 舵向 ----
    ser_steer = serial.Serial('/dev/ttyACM1', 921600, timeout=0.5)
    time.sleep(0.2)
    mc = MotorControl(ser_steer)
    motors = []
    print('[1] 初始化 4 个 DM4310 舵向电机...')
    for name, sid, mid in STEER_CONFIG:
        m = Motor(DM_Motor_Type.DM4310, sid, mid)
        mc.addMotor(m)
        pmax = mc.read_motor_param(m, DM_variable.PMAX)
        if pmax is None:
            print(f'    ✗ {name} 0x{sid:02X} 离线')
            motors.append(None)
        else:
            mc.disable(m); time.sleep(0.05)
            mc.switchControlMode(m, Control_Type.MIT); time.sleep(0.05)
            mc.enable(m); time.sleep(0.05)
            print(f'    ✓ {name} 0x{sid:02X} 已使能 (MIT)')
            motors.append(m)

    # ---- ACM0: ZLAC 驱动 ----
    ser_drive = serial.Serial('/dev/ttyACM0', 921600, timeout=0.3)
    time.sleep(0.2)
    ser_drive.write(bytes([0x55, 0x05, 0x03, 0xAA, 0x55]))
    time.sleep(0.5); ser_drive.read_all()
    send_can(ser_drive, 0x000, [0x01, 0x00, 0, 0, 0, 0, 0, 0])
    time.sleep(0.3)
    print('\n[2] 初始化 2 个 ZLAC8015D...')
    for nid in [2, 3]:
        enable_zlac(ser_drive, nid)
        print(f'    ✓ Node {nid} 已使能')

    # ---- 启动两个线程 ----
    t_steer = threading.Thread(target=steer_loop, args=(mc, motors), daemon=True)
    t_drive = threading.Thread(target=drive_loop, args=(ser_drive,), daemon=True)
    t_steer.start()
    t_drive.start()

    # ---- 先把舵向归零,等 0.5 秒 ----
    print('\n[3] 舵向归零...')
    set_target(0, 0, 0, 0.0)
    time.sleep(0.8)

    # ---- 7 种运动模式 ----
    print('\n[4] 开始运动学演示')
    try:
        pattern('1. 前进      FWD',     +0.15,  0.0,   0.0)
        pattern('2. 后退      BWD',     -0.15,  0.0,   0.0)
        pattern('3. 横移左    STRAFE_L', 0.0,   +0.15, 0.0)
        pattern('4. 横移右    STRAFE_R', 0.0,   -0.15, 0.0)
        pattern('5. 原地左转  SPIN_L',   0.0,   0.0,   +0.6)
        pattern('6. 原地右转  SPIN_R',   0.0,   0.0,   -0.6)
        pattern('7. 弧线前+左 ARC_FL',   +0.10, 0.0,   +0.4, run_s=2.5)
    except KeyboardInterrupt:
        print('  [中止]')

    # ---- 收尾 ----
    print('\n[5] 收尾: 归零 + 失能...')
    set_target(0, 0, 0, 0.0)
    time.sleep(1.0)
    running = False
    time.sleep(0.1)

    # 舵向失能
    for m in motors:
        if m is None: continue
        try: mc.disable(m)
        except: pass
    ser_steer.close()

    # ZLAC 失能
    for nid in [2, 3]:
        sdo_write(ser_drive, nid, 0x6040, 0, 0x0000, 2)
    ser_drive.close()

    print('=' * 60)
    print('  演示结束')
    print('=' * 60)


if __name__ == '__main__':
    main()
