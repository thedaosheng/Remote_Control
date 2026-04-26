#!/usr/bin/env python3
"""
================================================================
  DM3519 MIT 协议测试 (SlaveID=0x01, MasterID=0x101)
================================================================
MIT 协议帧格式 (达妙标准):

使能:    CAN ID = 0x01, data = [0xFF×7, 0xFC]
失能:    CAN ID = 0x01, data = [0xFF×7, 0xFD]
控制:    CAN ID = 0x01, data = 8 字节 (p16+v12+kp12+kd12+tau12)
反馈:    CAN ID = 0x101 (MasterID), data = 8 字节

PMAX=12.5 rad, VMAX=30 rad/s, TMAX=10 N·m (达妙 3519 默认)

本测试:
  阶段 A: 发 MIT 使能帧
  阶段 B: tau = 0.5 N·m 控制 1 秒 (纯力矩, kp=0 kd=0)
  阶段 C: tau = 0 衔接
  阶段 D: 失能
"""

import os, sys, time, struct
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
    f[21:29] = list(data)
    ser.write(bytes(f))


def parse_frames(buf):
    out = []
    i = 0
    while i <= len(buf) - 16:
        if buf[i] == 0xAA and buf[i + 15] == 0x55:
            cid = (buf[i + 6] << 24) | (buf[i + 5] << 16) | \
                  (buf[i + 4] << 8) | buf[i + 3]
            out.append((cid, bytes(buf[i + 7:i + 15])))
            i += 16
        else:
            i += 1
    return out


# ============== MIT 协议打包/解析 ==============
PMAX, VMAX, TMAX = 12.5, 30.0, 10.0
KP_MAX, KD_MAX = 500.0, 5.0


def f2u(x, x_min, x_max, bits):
    span = x_max - x_min
    v = int((x - x_min) * ((1 << bits) - 1) / span)
    return max(0, min((1 << bits) - 1, v))


def u2f(x, x_min, x_max, bits):
    span = x_max - x_min
    return x * span / ((1 << bits) - 1) + x_min


def mit_ctrl_data(pos, vel, kp, kd, tau):
    """生成 8 字节 MIT 控制帧 data"""
    p = f2u(pos, -PMAX, PMAX, 16)
    v = f2u(vel, -VMAX, VMAX, 12)
    kp_i = f2u(kp, 0, KP_MAX, 12)
    kd_i = f2u(kd, 0, KD_MAX, 12)
    t_i = f2u(tau, -TMAX, TMAX, 12)
    return [
        (p >> 8) & 0xFF,
        p & 0xFF,
        (v >> 4) & 0xFF,
        ((v & 0xF) << 4) | ((kp_i >> 8) & 0xF),
        kp_i & 0xFF,
        (kd_i >> 4) & 0xFF,
        ((kd_i & 0xF) << 4) | ((t_i >> 8) & 0xF),
        t_i & 0xFF,
    ]


def parse_mit_fb(data):
    """解析 0x101 MIT 反馈帧, 返回 (id, state, pos, vel, tau, Tmos, Tcoil)"""
    id_ = data[0] & 0x0F
    state = (data[0] >> 4) & 0x0F
    p_int = (data[1] << 8) | data[2]
    v_int = (data[3] << 4) | (data[4] >> 4)
    t_int = ((data[4] & 0xF) << 8) | data[5]
    pos = u2f(p_int, -PMAX, PMAX, 16)
    vel = u2f(v_int, -VMAX, VMAX, 12)
    tau = u2f(t_int, -TMAX, TMAX, 12)
    return id_, state, pos, vel, tau, data[6], data[7]


def main():
    ser = serial.Serial('/dev/ttyACM1', 921600, timeout=0.3)
    time.sleep(0.2)
    ser.reset_input_buffer()

    print('=' * 60)
    print('  DM3519 MIT 协议测试 (ID=0x01, MasterID=0x101)')
    print('=' * 60)

    # ---- A: 使能 MIT 模式 ----
    print('\n[A] 发 MIT 使能帧 (ID=0x01, data=FF×7+FC)')
    send_can(ser, 0x01, [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC])
    time.sleep(0.1)

    # ---- B: tau = 0.5 N·m, kp=kd=0 (纯力矩), 1 秒 ----
    TAU = 0.5
    print(f'\n[B] 发 MIT 力矩指令 tau={TAU}N·m (p=0 v=0 kp=0 kd=0) × 1 秒')
    print(f'    ⚠️ 应该看到电机开始转动, LED 可能变亮/快闪')

    ser.reset_input_buffer()
    t_end = time.time() + 1.0
    fb_count = {0x101: 0, 0x201: 0}
    last_print = 0
    last_mit = None
    while time.time() < t_end:
        # 200Hz 发控制帧
        send_can(ser, 0x01, mit_ctrl_data(0.0, 0.0, 0.0, 0.0, TAU))
        raw = ser.read_all()
        for cid, d in parse_frames(raw):
            if cid in fb_count:
                fb_count[cid] += 1
            if cid == 0x101:
                last_mit = d
                if time.time() - last_print > 0.15:
                    id_, state, pos, vel, tau, Tmos, Tcoil = parse_mit_fb(d)
                    print(f'    MIT[0x101] state={state} pos={pos:+.3f}rad '
                          f'vel={vel:+6.2f}rad/s tau={tau:+.2f}N·m '
                          f'Tmos={Tmos}°C Tcoil={Tcoil}°C')
                    last_print = time.time()
        time.sleep(0.005)

    # ---- C: tau = 0 过渡 ----
    print(f'\n[C] tau=0 过渡 0.3 秒')
    t_end = time.time() + 0.3
    while time.time() < t_end:
        send_can(ser, 0x01, mit_ctrl_data(0.0, 0.0, 0.0, 0.0, 0.0))
        time.sleep(0.01)

    # ---- D: 失能 ----
    print('\n[D] 发 MIT 失能帧 (data=FF×7+FD)')
    send_can(ser, 0x01, [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD])
    time.sleep(0.1)

    ser.close()
    print('\n' + '=' * 60)
    print(f'  反馈统计:')
    print(f'    0x101 (MIT): {fb_count[0x101]} 帧')
    print(f'    0x201 (一拖四电流模式): {fb_count[0x201]} 帧')
    if fb_count[0x101] > 0:
        print(f'  → MIT 协议生效!')
    else:
        print(f'  → MIT 协议未响应 (0x101 无反馈)')
        print(f'    可能固件只支持 0x200 一拖四, 或需要先切协议模式')
    print('=' * 60)


if __name__ == '__main__':
    main()
