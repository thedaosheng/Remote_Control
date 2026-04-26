#!/usr/bin/env python3
"""
================================================================
  ZLAC8015D CAN Node ID 修改 (1 → 2)
================================================================
参考:ZLTECH ZLAC 的 Modbus 寄存器布局与 CANopen 对象字典一一对应
     (常见惯例,厂商特定扩展对象 0x2000-0x2FFF 段)

  - 0x200A (常见): CAN Node ID
  - 0x2010 (常见): 参数保存到 EEPROM (写 1 触发)
  - 标准 0x1010:01: CANopen 通用保存命令,写 "save" ASCII

策略:
  1) 读几个候选对象 (0x200A, 0x2005, 0x2007) 看哪个返回值 = 1
  2) 对命中的写 2
  3) 写 0x2010:0 = 1 (厂商保存) 和 0x1010:01 = "save" (标准保存) 双保险
  4) 再次读验证
  5) 提示用户断电重启

注意:本脚本不使能电机,不让车动。安全。
"""

import os
import sys
import time
import struct
import numpy as np

import serial

# ============================================================
# 达妙 USB2CAN 帧
# ============================================================
DAMIAO_TX = np.array(
    [0x55, 0xAA, 0x1e, 0x03, 0x01, 0x00, 0x00, 0x00,
     0x0a, 0x00, 0x00, 0x00, 0x00, 0, 0, 0, 0,
     0x00, 0x08, 0x00,
     0x00, 0, 0, 0, 0, 0, 0, 0, 0, 0x00], np.uint8)


def send_can(ser, can_id, data8):
    frame = DAMIAO_TX.copy()
    frame[13] = can_id & 0xFF
    frame[14] = (can_id >> 8) & 0xFF
    frame[21:29] = data8
    ser.write(bytes(frame))


def recv_filtered(ser, target_id, timeout=0.5):
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


def sdo_read(ser, node_id, index, subindex=0, timeout=0.5):
    """返回 (value_uint32, status_str)"""
    tx_id = 0x600 + node_id
    rx_id = 0x580 + node_id
    ser.read_all()
    send_can(ser, tx_id,
             [0x40, index & 0xFF, (index >> 8) & 0xFF, subindex, 0, 0, 0, 0])
    resp = recv_filtered(ser, rx_id, timeout)
    if resp is None:
        return None, 'timeout'
    if resp[0] == 0x80:
        abort = struct.unpack_from('<I', resp, 4)[0]
        return None, f'abort=0x{abort:08X}'
    if resp[0] in (0x4F, 0x4B, 0x43, 0x42):
        return struct.unpack_from('<I', resp, 4)[0], 'ok'
    return None, f'cmd=0x{resp[0]:02X}'


def sdo_write(ser, node_id, index, subindex, value, size):
    """写 size 字节 (1/2/4)"""
    tx_id = 0x600 + node_id
    rx_id = 0x580 + node_id
    cmd_map = {1: 0x2F, 2: 0x2B, 4: 0x23}
    cmd = cmd_map.get(size, 0x23)
    if value < 0:
        value = value & (0xFF << (8 * size) - 1 | 0xFFFFFFFF)
    data = [
        cmd, index & 0xFF, (index >> 8) & 0xFF, subindex,
        value & 0xFF, (value >> 8) & 0xFF,
        (value >> 16) & 0xFF, (value >> 24) & 0xFF,
    ]
    ser.read_all()
    send_can(ser, tx_id, data)
    resp = recv_filtered(ser, rx_id, 0.5)
    if resp is None:
        return False, 'timeout'
    if resp[0] == 0x60:
        return True, 'ok'
    if resp[0] == 0x80:
        abort = struct.unpack_from('<I', resp, 4)[0]
        return False, f'abort=0x{abort:08X}'
    return False, f'cmd=0x{resp[0]:02X}'


# ============================================================
# 主流程
# ============================================================

def main():
    NODE_ID = 2         # 当前 Node ID (要改的那个)
    NEW_NODE_ID = 3     # 目标 Node ID

    print(f'=== ZLAC8015D Node ID 修改: {NODE_ID} → {NEW_NODE_ID} ===\n')

    ser = serial.Serial('/dev/ttyACM0', 921600, timeout=0.3)
    time.sleep(0.2)
    ser.reset_input_buffer()

    # 切 500 Kbps Classic CAN
    print('[1] 切适配器到 500 Kbps')
    ser.write(bytes([0x55, 0x05, 0x03, 0xAA, 0x55]))
    time.sleep(0.5)
    ser.read_all()

    # NMT Start 广播 (让 ZLAC 进 Operational,便于 SDO 通信)
    send_can(ser, 0x000, [0x01, 0x00, 0, 0, 0, 0, 0, 0])
    time.sleep(0.3)

    # ---- 2) 确认 Node 1 在线 ----
    print(f'[2] 验证 Node {NODE_ID} 在线...')
    sw, st = sdo_read(ser, NODE_ID, 0x6041, 0)
    if sw is None:
        print(f'    ✗ 0x6041 读失败 ({st}),Node {NODE_ID} 可能离线')
        ser.close()
        return 1
    print(f'    ✓ SW_L=0x{sw & 0xFFFF:04X}  节点在线')

    # ---- 3) 遍历候选对象,找到当前 Node ID 所在 ----
    print('\n[3] 探测 Node ID 所在对象字典...')
    candidates = [
        (0x200A, 0, 'ZLTECH 常见 CAN Node ID'),
        (0x2005, 0, 'ZLTECH 可能 (旧文档)'),
        (0x2007, 0, 'ZLTECH 可能 (新文档)'),
        (0x2008, 0, 'RS485 Node ID (参考,改这个没用)'),
        (0x100B, 0, 'CANopen 标准 Node ID (只读, DS301)'),
        (0x2022, 0, '某些固件位置'),
    ]
    found_obj = None
    for idx, sub, note in candidates:
        val, st = sdo_read(ser, NODE_ID, idx, sub, timeout=0.3)
        if val is not None:
            val_lo = val & 0xFFFF
            marker = ''
            if val_lo == NODE_ID:
                marker = '  ← 当前值 = Node ID,可能就是!'
                if found_obj is None and idx not in (0x2008, 0x100B):
                    found_obj = (idx, sub)
            print(f'    0x{idx:04X}:{sub} = {val} (0x{val_lo:04X})  [{note}]{marker}')
        else:
            print(f'    0x{idx:04X}:{sub} 读失败 ({st})  [{note}]')

    if found_obj is None:
        print('\n    ⚠ 没有候选对象返回值 = 1')
        print('    退回默认猜测:用 0x200A')
        found_obj = (0x200A, 0)

    idx, sub = found_obj
    print(f'\n[4] 选定 0x{idx:04X}:{sub} 作为 Node ID 寄存器')

    # ---- 4) 写 NEW_NODE_ID ----
    print(f'[5] SDO 写 0x{idx:04X}:{sub} = {NEW_NODE_ID} (尝试 2/4 字节)')
    ok = False
    for size in (2, 1, 4):
        r, st = sdo_write(ser, NODE_ID, idx, sub, NEW_NODE_ID, size)
        if r:
            print(f'    ✓ 写入成功 (size={size})')
            ok = True
            break
        else:
            print(f'    尝试 size={size} 失败: {st}')
    if not ok:
        print('    ✗ 所有大小都写失败!')
        ser.close()
        return 2

    # ---- 5) 回读验证 ----
    time.sleep(0.2)
    val, st = sdo_read(ser, NODE_ID, idx, sub)
    if val is not None:
        print(f'    回读 0x{idx:04X}:{sub} = {val & 0xFFFF}')
        if (val & 0xFFFF) != NEW_NODE_ID:
            print(f'    ⚠ 回读值 {val & 0xFFFF} 不等于期望 {NEW_NODE_ID}')

    # ---- 6) 触发 EEPROM 保存 ----
    print(f'\n[6] 保存到 EEPROM...')
    # 先试 0x2010:0 = 1 (厂商保存命令)
    r1, s1 = sdo_write(ser, NODE_ID, 0x2010, 0, 1, 2)
    print(f'    写 0x2010:0 = 1 → {s1}')
    time.sleep(0.3)

    # 再试 0x1010:01 = 0x65766173 ("save" ASCII, 标准 CANopen)
    r2, s2 = sdo_write(ser, NODE_ID, 0x1010, 1, 0x65766173, 4)
    print(f'    写 0x1010:1 = "save" → {s2}')
    time.sleep(0.5)

    if not (r1 or r2):
        print('    ⚠ 两种保存命令都失败,Node ID 可能未持久化,断电重启后会回到 1')
    else:
        print('    ✓ 至少一种保存命令成功')

    ser.close()

    # ---- 提示用户 ----
    print('\n' + '=' * 56)
    print('  下一步:')
    print(f'    1. 断开 ZLAC 电源 (等 5 秒)')
    print(f'    2. 重新上电')
    print(f'    3. 再跑 20260420-cc-pi_zlac_scan.py 扫描')
    print(f'       应该看到 Node 2 在线 (不再是 Node 1)')
    print('=' * 56)
    return 0


if __name__ == '__main__':
    sys.exit(main())
