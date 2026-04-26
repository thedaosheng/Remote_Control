#!/usr/bin/env python3
"""
================================================================
  ACM0 ZLAC8015D 驱动器非破坏性扫描
================================================================
场景:ACM0 路接了 2 个 ZLAC8015D 驱动器,每个驱动 L/R 2 个轮子,共 4 轮。
     协议:CANopen CiA402,波特率 500 Kbps。

本脚本只做以下 3 件事,不让电机转动:
  1) 切达妙适配器到 CAN 500 Kbps
  2) NMT Start 广播 + 扫描 Node ID 1-8 (读 0x6041 状态字)
  3) 对在线节点读关键信息:运行模式 0x6061、电机类型等

ZLAC8015D 默认 CAN Node ID = 1,如果两台共线,第二台通常改成 2/3/4。
"""

import os
import sys
import time
import struct
import glob
import subprocess
import numpy as np

import serial


# ============================================================
# 达妙 USB2CAN 发送帧模板 (30 字节)
# ============================================================
DAMIAO_TX = np.array(
    [0x55, 0xAA, 0x1e, 0x03, 0x01, 0x00, 0x00, 0x00,
     0x0a, 0x00, 0x00, 0x00, 0x00, 0, 0, 0, 0,
     0x00, 0x08, 0x00,
     0x00, 0, 0, 0, 0, 0, 0, 0, 0, 0x00], np.uint8)


def send_can(ser, can_id, data8):
    """发一个标准 CAN 帧 (11bit ID, 8 字节数据)"""
    frame = DAMIAO_TX.copy()
    frame[13] = can_id & 0xFF
    frame[14] = (can_id >> 8) & 0xFF
    frame[21:29] = data8
    ser.write(bytes(frame))


def recv_filtered(ser, target_id, timeout=0.3):
    """
    接收并过滤指定 CAN ID 的响应。
    达妙接收帧 16 字节:
      [0]=0xAA, [3:7]=CAN ID, [7:15]=data, [15]=0x55
    返回 8 字节 data 或 None
    """
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
    """
    CANopen SDO 读 (expedited)
    TX COB-ID = 0x600+NodeID
    RX COB-ID = 0x580+NodeID
    """
    tx_id = 0x600 + node_id
    rx_id = 0x580 + node_id
    data = [
        0x40,                         # 读请求
        index & 0xFF, (index >> 8) & 0xFF,
        subindex,
        0, 0, 0, 0
    ]
    ser.read_all()
    send_can(ser, tx_id, data)
    resp = recv_filtered(ser, rx_id, timeout)
    if resp is None:
        return None, 'timeout'
    cmd = resp[0]
    if cmd == 0x80:
        abort = struct.unpack_from('<I', resp, 4)[0]
        return None, f'abort=0x{abort:08X}'
    if cmd in (0x4F, 0x4B, 0x43, 0x42):
        val = struct.unpack_from('<I', resp, 4)[0]
        return val, 'ok'
    return None, f'unknown cmd=0x{cmd:02X}'


def nmt_start_all(ser):
    """NMT 广播 Start (COB-ID=0x000, Node=0 表广播)"""
    send_can(ser, 0x000, [0x01, 0x00, 0, 0, 0, 0, 0, 0])


# ============================================================
# 主流程
# ============================================================

def main():
    print('=== ACM0 ZLAC8015D 扫描 (CANopen 500Kbps) ===\n')

    port = '/dev/ttyACM0'
    try:
        ser = serial.Serial(port, 921600, timeout=0.3)
    except Exception as e:
        print(f'[ERR] 打开 {port} 失败: {e}')
        return 1

    time.sleep(0.2)
    ser.reset_input_buffer()

    # ---- 1) 切适配器到 CAN 500 Kbps ----
    # 达妙 USB2CAN 波特率切换命令 (参考 swerve_dm_driver_node):
    #   0x55, 0x05, 0x03, 0xAA, 0x55 → 500 Kbps Classic CAN
    print('[1] 切适配器到 500 Kbps CAN...')
    ser.write(bytes([0x55, 0x05, 0x03, 0xAA, 0x55]))
    time.sleep(0.5)
    ser.read_all()
    print('    已切换')

    # ---- 2) 被动监听 1 秒,看 ZLAC 有没有自动广播心跳 ----
    print('\n[2] 被动监听 1 秒 (ZLAC 心跳 COB-ID=0x700+NodeID)...')
    t0 = time.time()
    buf = b''
    while time.time() - t0 < 1.0:
        c = ser.read_all()
        if c:
            buf += c
        else:
            time.sleep(0.01)
    seen = {}
    i = 0
    while i <= len(buf) - 16:
        if buf[i] == 0xAA and buf[i + 15] == 0x55:
            cid = (buf[i + 6] << 24) | (buf[i + 5] << 16) | \
                  (buf[i + 4] << 8) | buf[i + 3]
            seen[cid] = seen.get(cid, 0) + 1
            i += 16
        else:
            i += 1
    if seen:
        for cid, n in sorted(seen.items()):
            hb_hint = ''
            if 0x700 <= cid <= 0x77F:
                nid = cid - 0x700
                hb_hint = f'(心跳! Node ID={nid})'
            print(f'    0x{cid:04X}: {n} 帧 {hb_hint}')
    else:
        print('    无任何帧 (ZLAC 可能未配置 heartbeat 或还没启动)')

    # ---- 3) NMT Start 广播,让所有节点进 Operational ----
    print('\n[3] NMT Start 广播 → 所有节点进 Operational')
    nmt_start_all(ser)
    time.sleep(0.3)

    # ---- 4) SDO 扫 Node ID 1-8,读状态字 (0x6041) ----
    print('\n[4] SDO 扫描 Node ID 1-8,读状态字 0x6041...')
    online = []
    for nid in range(1, 9):
        val, st = sdo_read(ser, nid, 0x6041, 0, timeout=0.2)
        if val is not None:
            sw_l = val & 0xFFFF
            sw_r = (val >> 16) & 0xFFFF
            # 解析 CiA402 状态:bit0-3, bit5
            state_bits = sw_l & 0x6F
            state_names = {
                0x00: 'Not Ready', 0x20: 'Switch On Disabled',
                0x21: 'Ready to Switch On', 0x23: 'Switched On',
                0x27: 'Operation Enabled', 0x07: 'Quick Stop',
                0x0F: 'Fault Reaction', 0x08: 'Fault'
            }
            name = state_names.get(state_bits, f'未知 0x{state_bits:02X}')
            print(f'  Node {nid}: ✓ SW_L=0x{sw_l:04X} SW_R=0x{sw_r:04X}  [{name}]')
            online.append((nid, sw_l, sw_r, name))
        else:
            print(f'  Node {nid}: ✗ ({st})')

    # ---- 5) 对在线节点读更多信息 ----
    if online:
        print('\n[5] 读在线节点的详细信息:')
        for nid, _, _, _ in online:
            mode, _ = sdo_read(ser, nid, 0x6061, 0, timeout=0.2)
            mode = mode & 0xFF if mode is not None else None
            mode_map = {1: 'Profile Position', 3: 'Profile Velocity',
                        4: 'Torque', 7: 'Interpolated', 8: 'CSP'}
            mn = mode_map.get(mode, f'Unknown ({mode})') if mode is not None else 'N/A'
            print(f'  Node {nid}: 运行模式 = {mn}')

    ser.close()

    # ---- 总结 ----
    print('\n=== 总结 ===')
    if len(online) == 0:
        print('  ✗ ACM0 上没有发现 ZLAC 节点')
        print('  排查建议:')
        print('   1. ZLAC 是否上电? 驱动器绿灯亮?')
        print('   2. ACM0 的 CANH/CANL 是否接到 ZLAC 的 CAN 接口?')
        print('   3. ZLAC 的 DIP 开关是否切到 CAN 模式 (不是 RS485)?')
        print('   4. 终端电阻是否两端 120Ω 就位?')
    elif len(online) == 1:
        print(f'  ⚠ 只找到 1 个 ZLAC (Node {online[0][0]}),期望是 2 个')
        print('  排查:第二个 ZLAC 是否上电?Node ID 是否和第一个冲突?')
    else:
        print(f'  ✓ 找到 {len(online)} 个 ZLAC: Node IDs = {[x[0] for x in online]}')
        print('  下一步可以做使能 + 低速测试 (需要把车架空)')

    return 0 if online else 1


if __name__ == '__main__':
    sys.exit(main())
