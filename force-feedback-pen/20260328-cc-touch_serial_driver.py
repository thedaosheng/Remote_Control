#!/usr/bin/env python3
"""
3D Systems Touch 力反馈笔 - 直接串口协议驱动
==============================================
通过逆向工程 OpenHaptics SDK 的串口通信协议，直接与设备通信。
无需安装 OpenHaptics SDK 系统级组件。

协议格式（从 strace 逆向分析）：
- 包头: 0xA5 0x5A
- 命令字: 1-2 字节
- 数据: 可变长度
- 波特率: 115200, 8N1, raw mode

命令列表（已识别）：
- 0x04 0x3C: 设备初始化/配置 (64 字节)
- 0x0F 0x20: 固件版本查询 (36 字节)
- 0x0C 0x01: 序列号查询 (5 字节)
- 0x07 0x07: 使能放大器/启动伺服 (11 字节)

作者: Claude Code (逆向分析)
日期: 2026-03-28
"""

import serial
import struct
import time
import sys
import signal
import threading
from dataclasses import dataclass
from typing import Optional, Tuple

# ============================================================================
# 协议常量
# ============================================================================

# 包头标记
PACKET_HEADER = b'\xa5\x5a'

# 命令字节
CMD_INIT_CONFIG = b'\x04\x3c'      # 设备初始化配置
CMD_FIRMWARE_QUERY = b'\x0f\x20'    # 固件版本查询
CMD_SERIAL_QUERY = b'\x0c\x01'     # 序列号查询
CMD_ENABLE_SERVO = b'\x07\x07'     # 使能伺服/放大器

# 响应命令字节
RESP_FIRMWARE = b'\x0e\x20'        # 固件版本响应（对应查询 0x0f 0x20）
RESP_SERIAL = b'\x09\x61'          # 序列号响应（对应查询 0x0c 0x01）

# 初始化配置数据（从 strace 捕获）
# 这是 SDK 发送的标准初始化序列
INIT_CONFIG_DATA = bytes([
    0x2f, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0xb2, 0x1c, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x1c,
    0x7f, 0x15, 0x04, 0x00, 0x00, 0x00, 0x11, 0x13,
    0x1a, 0x00, 0x12, 0x0f, 0x17, 0x16, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x02, 0x10, 0x00
])

# 使能伺服数据
ENABLE_SERVO_DATA = bytes([0x01, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00])

# 伺服循环数据包格式（猜测 - 需要进一步逆向）
# 设备在伺服模式下应该以 1kHz 持续发送编码器数据


# ============================================================================
# 数据结构
# ============================================================================

@dataclass
class TouchDeviceInfo:
    """设备信息"""
    firmware_version: str = ""
    serial_number: str = ""
    board_serial: str = ""
    hardware_rev: int = 0
    connected: bool = False


@dataclass
class TouchState:
    """力反馈笔实时状态"""
    # 编码器原始值（3 轴）
    encoder_values: Tuple[int, int, int] = (0, 0, 0)
    # 万向节电位器值（3 轴）
    gimbal_values: Tuple[int, int, int] = (0, 0, 0)
    # 按钮状态
    button1: bool = False
    button2: bool = False
    # 安全开关（墨水瓶/支架）
    inkwell: bool = False
    # 原始数据包（用于调试）
    raw_packet: bytes = b''
    # 时间戳
    timestamp: float = 0.0
    # 包计数
    packet_count: int = 0


# ============================================================================
# 主驱动类
# ============================================================================

class TouchSerialDriver:
    """
    3D Systems Touch 直接串口驱动

    绕过 OpenHaptics SDK，直接通过串口与设备通信。
    """

    def __init__(self, port: str = '/dev/ttyACM0', baudrate: int = 115200):
        """
        初始化驱动

        参数:
            port: 串口设备路径（默认 /dev/ttyACM0）
            baudrate: 波特率（默认 115200）
        """
        self.port = port
        self.baudrate = baudrate
        self._ser = None          # serial.Serial 对象
        self._info = TouchDeviceInfo()
        self._state = TouchState()
        self._running = False
        self._lock = threading.Lock()
        self._servo_thread = None
        self._target_force = [0, 0, 0]  # DAC 值（用于力输出）

    def _build_packet(self, cmd: bytes, data: bytes = b'') -> bytes:
        """
        构建协议数据包

        格式: [0xA5][0x5A][CMD_1][CMD_2][DATA...]
        """
        return PACKET_HEADER + cmd + data

    def _send(self, packet: bytes):
        """发送数据包"""
        self._ser.write(packet)
        self._ser.flush()

    def _recv(self, size: int, timeout: float = 1.0) -> bytes:
        """
        接收指定大小的响应数据

        参数:
            size: 期望接收的字节数
            timeout: 超时时间（秒）

        返回:
            接收到的字节数据
        """
        old_timeout = self._ser.timeout
        self._ser.timeout = timeout
        data = b''
        start = time.time()

        while len(data) < size and time.time() - start < timeout:
            chunk = self._ser.read(size - len(data))
            if chunk:
                data += chunk
            else:
                # 短暂等待后重试
                time.sleep(0.01)

        self._ser.timeout = old_timeout
        return data

    def _find_packet_header(self, data: bytes) -> int:
        """
        在数据中查找包头位置

        返回: 包头的起始索引，未找到返回 -1
        """
        return data.find(PACKET_HEADER)

    def connect(self) -> bool:
        """
        连接设备并配置串口

        返回:
            True 连接成功，False 失败
        """
        print(f"[TouchDriver] 正在连接 {self.port}...")

        try:
            # 打开串口
            self._ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.5,
                write_timeout=1.0,
                # 原始模式（无回显、无 CRLF 转换）
                xonxoff=False,
                rtscts=False,
                dsrdtr=False,
            )

            # 清空缓冲区
            self._ser.reset_input_buffer()
            self._ser.reset_output_buffer()
            time.sleep(0.1)

            print(f"[TouchDriver] 串口已打开: {self.port} @ {self.baudrate}")
            return True

        except serial.SerialException as e:
            print(f"[TouchDriver] 串口打开失败: {e}")
            return False

    def initialize(self) -> bool:
        """
        初始化设备：发送配置、查询固件和序列号

        返回:
            True 初始化成功，False 失败
        """
        if not self._ser or not self._ser.is_open:
            print("[TouchDriver] 错误: 串口未打开")
            return False

        print("[TouchDriver] 正在初始化设备...")

        # 步骤1: 发送初始化配置
        print("  [1/3] 发送初始化配置...")
        init_packet = self._build_packet(CMD_INIT_CONFIG, INIT_CONFIG_DATA)
        self._send(init_packet)
        time.sleep(0.1)

        # 步骤2: 查询固件版本
        print("  [2/3] 查询固件版本...")
        fw_query = self._build_packet(CMD_FIRMWARE_QUERY, b'\x00' * 32)
        self._send(fw_query)
        time.sleep(0.3)

        # 读取响应
        resp = self._recv(128, timeout=1.0)
        if resp:
            # 查找包头
            idx = self._find_packet_header(resp)
            if idx >= 0:
                resp = resp[idx:]
                if len(resp) >= 20 and resp[2:4] == b'\x0e\x20':
                    # 解析固件版本（从偏移4开始，以 \x00 结束）
                    fw_data = resp[4:36]
                    fw_str = fw_data.split(b'\x00')[0].decode('ascii', errors='replace').strip()
                    self._info.firmware_version = fw_str
                    print(f"  固件版本: {fw_str}")

                    # 从响应中提取硬件信息
                    # 响应格式: [A5 5A] [0E 20] [空格x3] [版本字符串] [00 00] [其他数据...]
                    if len(resp) >= 36:
                        hw_data = resp[20:36]
                        print(f"  硬件数据: {hw_data.hex()}")
                else:
                    print(f"  未知响应: cmd={resp[2:4].hex()}, data={resp[:36].hex()}")
            else:
                print(f"  原始响应: {resp[:64].hex()}")
        else:
            print("  [警告] 未收到固件版本响应")

        # 步骤3: 查询序列号
        print("  [3/3] 查询序列号...")
        sn_query = self._build_packet(CMD_SERIAL_QUERY, b'\x00')
        self._send(sn_query)
        time.sleep(0.3)

        resp = self._recv(128, timeout=1.0)
        if resp:
            idx = self._find_packet_header(resp)
            if idx >= 0:
                resp = resp[idx:]
                if len(resp) >= 20:
                    # 解析序列号响应
                    # 格式: [A5 5A] [09 61] [0C] [序列号1 (16字节)] [数据...] [序列号2] [...]
                    sn_data = resp[5:]  # 跳过包头+命令+长度
                    sn1 = sn_data[:16].split(b'\x00')[0].decode('ascii', errors='replace').strip()
                    self._info.serial_number = sn1

                    # 提取第二个序列号（板级序列号）
                    # 从响应数据中搜索数字模式
                    full_text = resp.decode('ascii', errors='replace')
                    # 查找数字序列（如 22039000866）
                    import re
                    numbers = re.findall(r'\d{8,}', full_text)
                    if numbers:
                        self._info.board_serial = numbers[0]

                    print(f"  设备序列号: {sn1}")
                    print(f"  板级序列号: {self._info.board_serial}")
            else:
                print(f"  原始响应: {resp[:64].hex()}")
        else:
            print("  [警告] 未收到序列号响应")

        self._info.connected = True

        print(f"\n{'='*50}")
        print(f"  3D Systems Touch 力反馈笔 - 设备信息")
        print(f"{'='*50}")
        print(f"  固件:     {self._info.firmware_version}")
        print(f"  序列号:   {self._info.serial_number}")
        print(f"  板级SN:   {self._info.board_serial}")
        print(f"  端口:     {self.port}")
        print(f"  波特率:   {self.baudrate}")
        print(f"{'='*50}\n")

        return True

    def enable_servo(self) -> bool:
        """
        尝试使能伺服循环（使能电机放大器）

        返回:
            True 成功，False 失败
        """
        print("[TouchDriver] 正在使能伺服...")
        servo_packet = self._build_packet(CMD_ENABLE_SERVO, ENABLE_SERVO_DATA)
        self._send(servo_packet)
        time.sleep(0.5)

        # 读取响应
        resp = self._recv(128, timeout=1.0)
        if resp:
            idx = self._find_packet_header(resp)
            if idx >= 0:
                resp = resp[idx:]
                print(f"  伺服响应: cmd={resp[2:4].hex()}, 数据={resp[:20].hex()}")
                return True
            else:
                print(f"  原始响应: {resp[:32].hex()}")
        else:
            print("  未收到伺服响应")

        return False

    def read_raw_stream(self, duration: float = 5.0):
        """
        读取设备的原始数据流

        在使能伺服后，设备应以 1kHz 发送编码器数据。
        此函数捕获并分析数据流格式。

        参数:
            duration: 读取持续时间（秒）
        """
        print(f"\n[TouchDriver] 读取原始数据流 ({duration:.0f} 秒)...")
        print(f"  请移动力反馈笔以产生数据变化\n")

        self._ser.reset_input_buffer()
        start = time.time()
        total_bytes = 0
        packets = []
        raw_buffer = b''

        while time.time() - start < duration:
            # 非阻塞读取
            data = self._ser.read(1024)
            if data:
                raw_buffer += data
                total_bytes += len(data)

                # 在缓冲区中搜索包头
                while True:
                    idx = raw_buffer.find(PACKET_HEADER)
                    if idx < 0:
                        # 没有找到包头，保留最后 1 个字节（可能是包头的第一个字节）
                        if len(raw_buffer) > 1:
                            raw_buffer = raw_buffer[-1:]
                        break

                    # 跳过包头之前的数据
                    if idx > 0:
                        raw_buffer = raw_buffer[idx:]

                    # 确保至少有 4 字节（包头 + 命令）
                    if len(raw_buffer) < 4:
                        break

                    # 提取命令字节
                    cmd = raw_buffer[2:4]

                    # 根据命令确定包长度（需要从实际通信中学习）
                    # 目前只记录并分析
                    # 假设每个包至少 20 字节
                    if len(raw_buffer) < 20:
                        break

                    # 查找下一个包头来确定当前包的长度
                    next_idx = raw_buffer[2:].find(PACKET_HEADER)
                    if next_idx < 0:
                        # 没有下一个包头，等待更多数据
                        break

                    packet_len = next_idx + 2  # 当前包长度
                    packet = raw_buffer[:packet_len]
                    raw_buffer = raw_buffer[packet_len:]

                    packets.append(packet)

                    # 实时显示
                    if len(packets) % 100 == 1 or len(packets) <= 10:
                        elapsed = time.time() - start
                        sys.stdout.write(
                            f"\r  收到 {len(packets)} 包 | {total_bytes} 字节 | "
                            f"cmd=0x{cmd.hex()} | len={packet_len} | "
                            f"数据={packet[4:min(20,len(packet))].hex()}"
                            f"     "
                        )
                        sys.stdout.flush()

            time.sleep(0.001)

        print(f"\n\n  === 数据流统计 ===")
        print(f"  总字节数: {total_bytes}")
        print(f"  总数据包: {len(packets)}")
        if duration > 0:
            print(f"  平均速率: {total_bytes / duration:.0f} bytes/s, {len(packets) / duration:.0f} packets/s")

        # 分析数据包类型分布
        if packets:
            cmd_counts = {}
            cmd_sizes = {}
            for p in packets:
                cmd = p[2:4].hex()
                cmd_counts[cmd] = cmd_counts.get(cmd, 0) + 1
                if cmd not in cmd_sizes:
                    cmd_sizes[cmd] = []
                cmd_sizes[cmd].append(len(p))

            print(f"\n  === 命令类型分布 ===")
            for cmd, count in sorted(cmd_counts.items()):
                sizes = cmd_sizes[cmd]
                avg_size = sum(sizes) / len(sizes)
                print(f"  cmd=0x{cmd}: {count} 包, 平均大小={avg_size:.0f} 字节")

            # 显示前几个包的详细内容
            print(f"\n  === 前 5 个数据包详情 ===")
            for i, p in enumerate(packets[:5]):
                print(f"  包 {i}: cmd=0x{p[2:4].hex()}, len={len(p)}")
                print(f"         hex: {p.hex()}")
                # 尝试解析为整数数组
                if len(p) >= 10:
                    try:
                        # 假设数据是 little-endian int16 或 int32
                        data = p[4:]
                        int16s = [struct.unpack_from('<h', data, j)[0]
                                  for j in range(0, len(data) - 1, 2)]
                        print(f"         int16: {int16s[:10]}")
                    except:
                        pass

            # 检查数据变化（如果包类型一致）
            if len(packets) >= 10:
                main_cmd = max(cmd_counts, key=cmd_counts.get)
                same_cmd_packets = [p for p in packets if p[2:4].hex() == main_cmd]
                if len(same_cmd_packets) >= 2:
                    print(f"\n  === 数据变化分析 (cmd=0x{main_cmd}) ===")
                    first = same_cmd_packets[0]
                    last = same_cmd_packets[-1]
                    print(f"  首包: {first[4:].hex()}")
                    print(f"  末包: {last[4:].hex()}")
                    # 找到变化的字节位置
                    changes = []
                    for i in range(4, min(len(first), len(last))):
                        if first[i] != last[i]:
                            changes.append(i - 4)
                    if changes:
                        print(f"  变化位置: {changes}")
                    else:
                        print(f"  数据未变化（请移动力反馈笔再试）")

        return packets

    def probe_commands(self):
        """
        探测设备支持的命令

        发送一系列探测命令，记录设备响应
        """
        print("\n[TouchDriver] 探测设备命令...")

        # 尝试发送一些可能的伺服读取命令
        probe_cmds = [
            (b'\x01\x00', "读取状态?"),
            (b'\x02\x00', "读取编码器?"),
            (b'\x03\x00', "读取按钮?"),
            (b'\x05\x00', "读取位置?"),
            (b'\x06\x00', "读取速度?"),
            (b'\x08\x00', "心跳?"),
            (b'\x0a\x00', "读取温度?"),
            (b'\x0b\x00', "读取校准?"),
            (b'\x0d\x00', "读取状态2?"),
            (b'\x10\x00', "读取GPIO?"),
        ]

        self._ser.reset_input_buffer()

        for cmd, desc in probe_cmds:
            # 发送探测命令
            packet = self._build_packet(cmd, b'\x00' * 4)
            self._send(packet)
            time.sleep(0.2)

            # 读取响应
            resp = self._ser.read(256)
            if resp:
                idx = resp.find(PACKET_HEADER)
                if idx >= 0:
                    resp_cmd = resp[idx + 2:idx + 4]
                    print(f"  cmd=0x{cmd.hex()} ({desc})")
                    print(f"    响应: cmd=0x{resp_cmd.hex()}, len={len(resp)-idx}, data={resp[idx:idx+20].hex()}")
                else:
                    print(f"  cmd=0x{cmd.hex()} ({desc}) -> 无包头响应: {resp[:20].hex()}")
            else:
                print(f"  cmd=0x{cmd.hex()} ({desc}) -> 无响应")

    def send_force_command(self, dac0: int = 0, dac1: int = 0, dac2: int = 0):
        """
        尝试发送力反馈 DAC 命令

        参数:
            dac0, dac1, dac2: 三轴 DAC 值（10-bit PWM, 范围 0-1023）
        """
        # 构建力输出数据包
        # 基于协议分析，力反馈可能使用特定命令字
        # 这需要进一步逆向来确认正确的格式
        data = struct.pack('<hhh', dac0, dac1, dac2)
        packet = self._build_packet(b'\x04\x00', data)
        self._send(packet)

    def close(self):
        """关闭连接"""
        self._running = False
        if self._servo_thread:
            self._servo_thread.join(timeout=2.0)
        if self._ser and self._ser.is_open:
            # 发送零力命令
            self.send_force_command(0, 0, 0)
            time.sleep(0.05)
            self._ser.close()
            print("[TouchDriver] 连接已关闭")

    @property
    def info(self) -> TouchDeviceInfo:
        return self._info


# ============================================================================
# 主程序
# ============================================================================

def main():
    """主程序：连接设备、初始化、探测协议、读取数据流"""

    print("\n" + "=" * 55)
    print("  3D Systems Touch 力反馈笔 - 直接串口驱动")
    print("  " + "=" * 51)
    print("  协议: 逆向分析 (0xA55A 二进制协议)")
    print("  日期: 2026-03-28")
    print("=" * 55 + "\n")

    driver = TouchSerialDriver(port='/dev/ttyACM0')

    def signal_handler(sig, frame):
        print("\n\n[中断] Ctrl+C - 安全退出...")
        driver.close()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)

    try:
        # 1. 连接设备
        if not driver.connect():
            print("[错误] 无法连接设备")
            return

        # 2. 初始化并查询设备信息
        if not driver.initialize():
            print("[警告] 设备初始化可能未完全成功，继续探测...")

        # 3. 尝试使能伺服
        print("\n--- 尝试使能伺服循环 ---")
        driver.enable_servo()
        time.sleep(0.5)

        # 4. 读取原始数据流
        print("\n--- 读取原始数据流 ---")
        packets = driver.read_raw_stream(duration=5.0)

        # 5. 探测其他命令
        print("\n--- 探测设备命令 ---")
        driver.probe_commands()

        # 6. 再次读取数据流（探测后可能有新数据）
        print("\n--- 最终数据流读取 ---")
        packets = driver.read_raw_stream(duration=3.0)

        print("\n" + "=" * 55)
        print("  探测完成!")
        print("=" * 55 + "\n")

    except Exception as e:
        print(f"\n[错误] {e}")
        import traceback
        traceback.print_exc()

    finally:
        driver.close()


if __name__ == "__main__":
    main()
