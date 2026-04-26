#!/usr/bin/env python3
"""
================================================================
  DM3519 一拖四测试 (M1 单个电机)
================================================================
协议 (来自达妙官方 stm32 例程 dm_ctrl(DM3519 一拖四)):

发送帧 (无需使能,直接发):
  CAN ID = 0x200
  Data[0:2] = M1 电流 int16 big-endian
  Data[2:4] = M2 电流
  Data[4:6] = M3 电流
  Data[6:8] = M4 电流
  映射: int16 = current_A × 16384 / 20

反馈帧 (电机主动广播):
  ID = 0x201 (M1), 0x202 (M2), ...
  Data[0:2] = pos_int16 → pos(°) = val / 8192 × 360
  Data[2:4] = spd_int16 (直接 RPM)
  Data[4:6] = cur_int16 → A = val × 20 / 16384
  Data[6]   = Tcoil (线圈温度 °C)
  Data[7]   = Tmos  (MOS 温度 °C)

注意:3519 和 4 个 4310 共用 ACM1 1M 总线,协议 ID 不冲突。

本测试:
  阶段 A: 被动监听 2s,看 3519 是否主动广播 0x201
  阶段 B: 发 0.3A 到 M1 持续 2s,看电机是否响应 + 反馈
  阶段 C: 发 0A 停止
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
    f[21:29] = data
    ser.write(bytes(f))


def parse_frames(buf):
    """解析达妙 16 字节接收帧, 返回 [(cid, data8), ...]"""
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


def dm3519_current_set(ser, m1_A, m2_A=0.0, m3_A=0.0, m4_A=0.0):
    """构造 0x200 电流控制帧并发送"""
    def to_int16(a):
        v = int(a * 16384.0 / 20.0)
        v = max(-32768, min(32767, v))
        return v & 0xFFFF

    v1, v2, v3, v4 = [to_int16(x) for x in (m1_A, m2_A, m3_A, m4_A)]
    data = [
        (v1 >> 8) & 0xFF, v1 & 0xFF,
        (v2 >> 8) & 0xFF, v2 & 0xFF,
        (v3 >> 8) & 0xFF, v3 & 0xFF,
        (v4 >> 8) & 0xFF, v4 & 0xFF,
    ]
    send_can(ser, 0x200, data)


def parse_3519_fb(data):
    """返回 (pos_deg, vel_rpm, cur_A, Tcoil, Tmos)"""
    pos_i = struct.unpack('>h', data[0:2])[0]
    spd_i = struct.unpack('>h', data[2:4])[0]
    cur_i = struct.unpack('>h', data[4:6])[0]
    pos_deg = pos_i / 8192.0 * 360.0
    vel_rpm = spd_i
    cur_A = cur_i * 20.0 / 16384.0
    return pos_deg, vel_rpm, cur_A, data[6], data[7]


def main():
    ser = serial.Serial('/dev/ttyACM1', 921600, timeout=0.3)
    time.sleep(0.2)
    ser.reset_input_buffer()

    print('=' * 60)
    print('  DM3519 一拖四测试 (ACM1 1M Classic CAN)')
    print('=' * 60)

    # ---- 阶段 A: 被动监听 ----
    print('\n[A] 被动监听 2 秒,看 3519 是否主动广播 0x201 反馈...')
    t_end = time.time() + 1.0
    seen = {}
    while time.time() < t_end:
        raw = ser.read_all()
        if raw:
            for cid, d in parse_frames(raw):
                seen.setdefault(cid, {'count': 0, 'last': None})
                seen[cid]['count'] += 1
                seen[cid]['last'] = d
        else:
            time.sleep(0.01)

    print(f'    监听到 {len(seen)} 个不同 ID:')
    for cid, v in sorted(seen.items()):
        if cid == 0x201:
            pos, vel, cur, Tc, Tm = parse_3519_fb(v['last'])
            print(f'    ★ 0x201 (M1 反馈): {v["count"]} 帧')
            print(f'        pos={pos:+.1f}°  vel={vel}rpm  cur={cur:+.3f}A  '
                  f'Tcoil={Tc}°C  Tmos={Tm}°C')
        elif cid in (0x202, 0x203, 0x204):
            print(f'    ★ 0x{cid:03X} (M{cid-0x200} 反馈): {v["count"]} 帧')
        else:
            print(f'      0x{cid:03X}: {v["count"]} 帧 ({v["last"].hex()})')

    has_3519 = 0x201 in seen

    # ---- 阶段 B: 发 0.3A ----
    print('\n[B] 发 M1=1.5A 持续 1 秒 (其他通道 0A)')
    print('    ⚠️ 电机应该空载高速转动. LED 应从绿闪 → 常亮/快闪')
    print('    若无动作, Ctrl-C 并告知')

    ser.reset_input_buffer()
    t_end = time.time() + 1.0
    last_print = 0
    m1_count = 0
    while time.time() < t_end:
        dm3519_current_set(ser, 1.5, 0.0, 0.0, 0.0)
        raw = ser.read_all()
        for cid, d in parse_frames(raw):
            if cid == 0x201:
                m1_count += 1
                if time.time() - last_print > 0.25:
                    pos, vel, cur, Tc, Tm = parse_3519_fb(d)
                    print(f'    [M1] pos={pos:+7.1f}° vel={vel:+5d}rpm '
                          f'cur={cur:+.3f}A Tc={Tc}°C Tmos={Tm}°C')
                    last_print = time.time()
        time.sleep(0.01)

    # ---- 阶段 C: 停止 ----
    print('\n[C] 发 0A 停止...')
    for _ in range(30):
        dm3519_current_set(ser, 0.0, 0.0, 0.0, 0.0)
        time.sleep(0.02)

    ser.close()
    print('\n' + '=' * 60)
    print(f'  总结:')
    print(f'    主动广播 0x201: {"✓" if has_3519 else "✗"}')
    print(f'    发电流后反馈 0x201: {m1_count} 帧')
    print(f'    电机是否转动: 请你目视确认')
    print('=' * 60)


if __name__ == '__main__':
    main()
