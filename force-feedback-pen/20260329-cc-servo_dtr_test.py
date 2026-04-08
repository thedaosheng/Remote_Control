#!/usr/bin/env python3
"""
3D Systems Touch - DTR/RTS 控制线测试
======================================
USB CDC ACM 设备的固件可能检查 DTR 信号。
测试设置 DTR=True 后 servo enable 是否能收到响应。
"""

import serial
import time
import sys
import struct
import os
import fcntl

# TIOCM ioctl 常量
TIOCMGET = 0x5415
TIOCMSET = 0x5418
TIOCM_DTR = 0x002
TIOCM_RTS = 0x004

# 协议数据包
INIT_PACKET = b'\xa5\x5a\x04\x3c' + bytes([
    0x2f, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0xb2, 0x1c, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x1c,
    0x7f, 0x15, 0x04, 0x00, 0x00, 0x00, 0x11, 0x13,
    0x1a, 0x00, 0x12, 0x0f, 0x17, 0x16, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x02, 0x10, 0x00,
])

FW_QUERY = b'\xa5\x5a\x0f\x20' + b'\x00' * 32
SN_QUERY = b'\xa5\x5a\x0c\x01\x00'
SERVO_ENABLE = b'\xa5\x5a\x07\x07\x01\x00\x00\x80\x00\x00\x00'

HEADER = b'\xa5\x5a'


def hex_dump(data, prefix="  "):
    """简短十六进制输出"""
    if not data:
        return f"{prefix}(空)"
    lines = []
    for i in range(0, min(len(data), 96), 16):
        chunk = data[i:i+16]
        h = ' '.join(f'{b:02x}' for b in chunk)
        a = ''.join(chr(b) if 32 <= b < 127 else '.' for b in chunk)
        lines.append(f"{prefix}{i:04x}: {h:<48s}  {a}")
    if len(data) > 96:
        lines.append(f"{prefix}... 共 {len(data)} 字节")
    return '\n'.join(lines)


def read_response(ser, timeout=0.5):
    """读取串口响应"""
    data = b''
    start = time.time()
    while time.time() - start < timeout:
        chunk = ser.read(4096)
        if chunk:
            data += chunk
            # 如果找到完整包就返回
            if HEADER in data and len(data) >= 20:
                time.sleep(0.05)  # 等一小会确保读完
                data += ser.read(4096)
                break
        time.sleep(0.005)
    return data


def test_with_dtr_config(dtr_val, rts_val, desc):
    """用指定的 DTR/RTS 配置测试"""
    port = '/dev/ttyACM0'

    print(f"\n{'='*60}")
    print(f"  测试: {desc}")
    print(f"  DTR={dtr_val}, RTS={rts_val}")
    print(f"{'='*60}")

    # 先运行 SDK 的 stty 命令
    os.system(
        "stty -F /dev/ttyACM0 115200 "
        "-parenb -parodd -cmspar cs8 hupcl -cstopb cread clocal -crtscts "
        "-ignbrk -brkint ignpar -parmrk -inpck -istrip -inlcr -igncr -icrnl "
        "-ixon -ixoff -iuclc -ixany -imaxbel -iutf8 "
        "-opost -olcuc -ocrnl -onlcr -onocr -onlret -ofill -ofdel "
        "nl0 cr0 tab0 bs0 vt0 ff0 "
        "-isig -icanon -iexten -echo -echoe -echok -echonl "
        "-noflsh -xcase -tostop -echoprt -echoctl -echoke -flusho -extproc"
    )
    time.sleep(0.1)

    # 打开串口
    ser = serial.Serial()
    ser.port = port
    ser.baudrate = 115200
    ser.bytesize = serial.EIGHTBITS
    ser.parity = serial.PARITY_NONE
    ser.stopbits = serial.STOPBITS_ONE
    ser.timeout = 0.1
    ser.write_timeout = 1.0
    ser.xonxoff = False
    ser.rtscts = False
    ser.dsrdtr = False  # 先不让 pyserial 自动管理

    # 关键：在打开前设置 DTR/RTS 初始值
    ser.dtr = dtr_val
    ser.rts = rts_val

    try:
        ser.open()
        print(f"  串口已打开")

        # 显式设置 DTR/RTS（通过 ioctl）
        modem_bits = fcntl.ioctl(ser.fd, TIOCMGET, b'\x00\x00\x00\x00')
        status = struct.unpack('I', modem_bits)[0]
        print(f"  当前 modem 状态: 0x{status:04x} "
              f"(DTR={'ON' if status & TIOCM_DTR else 'OFF'}, "
              f"RTS={'ON' if status & TIOCM_RTS else 'OFF'})")

        # 强制设置 DTR/RTS
        if dtr_val:
            status |= TIOCM_DTR
        else:
            status &= ~TIOCM_DTR
        if rts_val:
            status |= TIOCM_RTS
        else:
            status &= ~TIOCM_RTS
        fcntl.ioctl(ser.fd, TIOCMSET, struct.pack('I', status))

        # 验证设置结果
        modem_bits = fcntl.ioctl(ser.fd, TIOCMGET, b'\x00\x00\x00\x00')
        status = struct.unpack('I', modem_bits)[0]
        print(f"  设置后状态: 0x{status:04x} "
              f"(DTR={'ON' if status & TIOCM_DTR else 'OFF'}, "
              f"RTS={'ON' if status & TIOCM_RTS else 'OFF'})")

        # 清空缓冲区
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        time.sleep(0.2)
        ser.read(4096)  # 丢弃残留

        # 发送 init
        print(f"\n  [init] 发送 {len(INIT_PACKET)} 字节...")
        ser.write(INIT_PACKET)
        ser.flush()
        time.sleep(0.1)

        # 固件查询
        print(f"  [fw] 查询固件版本...")
        ser.write(FW_QUERY)
        ser.flush()
        resp = read_response(ser, timeout=1.0)
        if resp and HEADER in resp:
            idx = resp.find(HEADER)
            cmd = resp[idx+2:idx+4]
            if cmd == b'\x0e\x20':
                fw = resp[idx+4:idx+36].split(b'\x00')[0].decode('ascii', errors='replace').strip()
                print(f"  [fw] ✓ 固件: {fw}")
            else:
                print(f"  [fw] 响应 cmd=0x{cmd.hex()}")
        else:
            print(f"  [fw] ✗ 无响应")
            ser.close()
            return False

        # 序列号查询
        print(f"  [sn] 查询序列号...")
        ser.write(SN_QUERY)
        ser.flush()
        resp = read_response(ser, timeout=1.0)
        if resp and HEADER in resp:
            idx = resp.find(HEADER)
            sn = resp[idx+5:idx+21].split(b'\x00')[0].decode('ascii', errors='replace').strip()
            print(f"  [sn] ✓ 序列号: {sn}")
        else:
            print(f"  [sn] ✗ 无响应")

        # 清空残留
        time.sleep(0.1)
        ser.read(4096)

        # ★★★ 使能伺服 ★★★
        print(f"\n  [servo] ★★★ 使能伺服 ★★★")
        print(f"  [servo] 发送: {SERVO_ENABLE.hex()}")

        ser.reset_input_buffer()
        ser.write(SERVO_ENABLE)
        ser.flush()

        # 等待并读取响应（3 秒）
        servo_resp = b''
        start = time.time()
        while time.time() - start < 3.0:
            chunk = ser.read(4096)
            if chunk:
                servo_resp += chunk
                elapsed = (time.time() - start) * 1000
                print(f"  [servo] +{elapsed:.0f}ms: 收到 {len(chunk)} 字节 (累计 {len(servo_resp)})")
                print(hex_dump(chunk[:48], "    "))
            time.sleep(0.005)

        if servo_resp:
            print(f"\n  [servo] ★ 总计 {len(servo_resp)} 字节:")
            print(hex_dump(servo_resp, "    "))

            # 搜索包头
            idx = servo_resp.find(HEADER)
            if idx >= 0:
                pkt = servo_resp[idx:]
                print(f"  [servo] 数据包: cmd=0x{pkt[2:4].hex()}")
            return True
        else:
            print(f"\n  [servo] ✗ 无响应（3 秒超时）")

            # 检查设备是否还活着
            print(f"\n  [check] 设备是否还在线...")
            ser.write(FW_QUERY)
            ser.flush()
            check = read_response(ser, timeout=1.0)
            if check and HEADER in check:
                print(f"  [check] ✓ 设备在线但不回复 servo")
            else:
                print(f"  [check] ✗ 设备完全无响应（servo 后设备静默）")
            return False

    except Exception as e:
        print(f"  错误: {e}")
        import traceback
        traceback.print_exc()
        return False

    finally:
        try:
            ser.close()
        except:
            pass


def main():
    print(f"\n{'='*60}")
    print(f"  3D Systems Touch - DTR/RTS 控制线测试")
    print(f"  时间: {time.strftime('%Y-%m-%d %H:%M:%S')}")
    print(f"{'='*60}")

    # 测试不同的 DTR/RTS 组合
    configs = [
        (True, True, "DTR=ON, RTS=ON（最可能的正确配置）"),
        (True, False, "DTR=ON, RTS=OFF"),
        (False, True, "DTR=OFF, RTS=ON"),
        (False, False, "DTR=OFF, RTS=OFF（当前的默认配置）"),
    ]

    for dtr, rts, desc in configs:
        success = test_with_dtr_config(dtr, rts, desc)
        if success:
            print(f"\n\n★★★ 成功! 配置: {desc} ★★★")
            break

        # 设备可能需要重置 - 等待一下
        print(f"\n  等待设备恢复...")
        time.sleep(2)

    print(f"\n{'='*60}")
    print(f"  测试结束")
    print(f"{'='*60}\n")


if __name__ == "__main__":
    main()
