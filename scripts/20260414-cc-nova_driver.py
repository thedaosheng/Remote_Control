#!/usr/bin/env python3
"""
SenseGlove Nova Linux 驱动 — 直接串口通信，无需 SenseCom

通过逆向 SGConnect/SGCore 二进制库获得的串口协议，
在 Linux 上直接通过蓝牙 rfcomm 串口驱动 SenseGlove Nova 手套。

协议命令:
  {S}  开始传感器数据流 (~66Hz)
  {s}  停止传感器数据流
  {Z<ffb×4><wrist>}  力反馈制动 (Nova 2 格式)
  {Z<ffb×4><wrist><buzz×2>}  力反馈+振动 (Nova 1 格式)

值编码: float 0.0~1.0 → chr(round(f*100) + 1)

用法:
  # 先蓝牙连接手套
  sudo rfcomm connect /dev/rfcomm0 <MAC> 1

  # 运行驱动测试
  python3 /home/rhz/teleop/SenseGlove/20260414-cc-nova_driver.py

  # 指定端口
  python3 /home/rhz/teleop/SenseGlove/20260414-cc-nova_driver.py /dev/rfcomm1
"""

import serial
import time
import threading
import sys


class NovaGlove:
    """SenseGlove Nova 手套驱动

    用法:
        glove = NovaGlove('/dev/rfcomm0')
        glove.start()

        # 读传感器
        data = glove.get_sensor_data()
        # data = {'sensors': [768, 1936, -1575, 856, 1874],
        #         'imu': [-1.65, 2.53, 0.78, -0.34],
        #         'battery': 93, 'status': 0}

        # 力反馈: 制动食指
        glove.set_force_feedback([0.0, 1.0, 0.0, 0.0])

        # 释放
        glove.set_force_feedback([0.0, 0.0, 0.0, 0.0])

        glove.stop()
    """

    def __init__(self, port='/dev/rfcomm0', baudrate=115200):
        """初始化手套驱动

        Args:
            port: rfcomm 串口路径
            baudrate: 波特率 (蓝牙 SPP 无实际意义，保持默认)
        """
        self.ser = serial.Serial(port, baudrate, timeout=0.5)
        self.running = False
        self.lock = threading.Lock()
        self.latest_data = None
        self._reader_thread = None
        self._frame_count = 0

    @staticmethod
    def _sg_byte(f):
        """将 0.0~1.0 的 float 转为 SG 协议单字节

        编码规则: chr(round(f * 100) + 1)
        0.0 → 0x01, 0.5 → 0x33, 1.0 → 0x65
        """
        v = max(0, min(100, round(f * 100)))
        return chr(v + 1)

    def start(self):
        """开始传感器数据流"""
        self.ser.reset_input_buffer()
        self.ser.write(b'{S}')
        self.running = True
        self._reader_thread = threading.Thread(target=self._read_loop, daemon=True)
        self._reader_thread.start()

    def stop(self):
        """停止数据流 + 释放力反馈"""
        self.set_force_feedback([0.0, 0.0, 0.0, 0.0])
        self.running = False
        time.sleep(0.1)
        self.ser.write(b'{s}')
        if self._reader_thread:
            self._reader_thread.join(timeout=2)
        self.ser.close()

    def _read_loop(self):
        """后台持续读取传感器数据帧"""
        buf = ''
        while self.running:
            try:
                chunk = self.ser.read(512)
                if not chunk:
                    continue
                buf += chunk.decode('ascii', errors='replace')
                # 按行解析
                while '\n' in buf:
                    line, buf = buf.split('\n', 1)
                    line = line.strip()
                    if line.startswith('[') and line.endswith(']'):
                        parsed = self._parse_frame(line)
                        if parsed:
                            with self.lock:
                                self.latest_data = parsed
                                self._frame_count += 1
            except Exception:
                if self.running:
                    time.sleep(0.01)

    @staticmethod
    def _parse_frame(line):
        """解析传感器帧

        格式: [s1;s2|s3|s4|s5:qw;qx;qy;qz:battery;status]

        Returns:
            dict: {'sensors': [5 ints], 'imu': [4 floats],
                   'battery': int, 'status': int}
        """
        try:
            inner = line[1:-1]
            parts = inner.split(':')
            if len(parts) < 3:
                return None

            # 传感器值: ; 和 | 都是分隔符
            sensor_str = parts[0]
            sensor_vals = [int(x) for x in sensor_str.replace(';', '|').split('|')]

            # IMU 四元数
            imu = [float(x) for x in parts[1].split(';')]

            # 电池和状态
            misc = parts[2].split(';')
            battery = int(misc[0])
            status = int(misc[1]) if len(misc) > 1 else 0

            return {
                'sensors': sensor_vals,
                'imu': imu,
                'battery': battery,
                'status': status,
            }
        except (ValueError, IndexError):
            return None

    def get_sensor_data(self):
        """获取最新一帧传感器数据

        Returns:
            dict 或 None (尚未收到数据)
        """
        with self.lock:
            return self.latest_data

    @property
    def frame_count(self):
        """已接收的帧数"""
        with self.lock:
            return self._frame_count

    def set_force_feedback(self, ffb, wrist=0.0):
        """设置力反馈制动 (Nova 2 流式格式)

        磁力刹车: 只能提供阻力(制动)，不能主动推拉手指。
        需要持续发送以保持制动状态。

        Args:
            ffb: [thumb, index, middle, ring] 每个 0.0(无)~1.0(全力制动)
            wrist: 腕部挤压 0.0~1.0
        """
        cmd = '{Z'
        for i in range(4):
            cmd += self._sg_byte(ffb[i] if i < len(ffb) else 0.0)
        cmd += self._sg_byte(wrist)
        cmd += '}'
        self.ser.write(cmd.encode('latin-1'))

    def set_vibration(self, buzz, wrist_pct=0):
        """设置振动反馈 (Nova 1 格式)

        Args:
            buzz: [motor0, motor1] 每个 0.0~1.0
            wrist_pct: 腕部振动 0~100 整数
        """
        cmd = '{Z'
        # FFB 全零
        cmd += self._sg_byte(0.0) * 4
        # 腕部 (整数编码)
        cmd += chr(int(max(0, min(100, wrist_pct))) + 1)
        # 振动马达
        for i in range(2):
            cmd += self._sg_byte(buzz[i] if i < len(buzz) else 0.0)
        cmd += '}'
        self.ser.write(cmd.encode('latin-1'))


# ============== 交互式测试 ==============
if __name__ == '__main__':
    port = sys.argv[1] if len(sys.argv) > 1 else '/dev/rfcomm0'

    print('=' * 50)
    print('  SenseGlove Nova Linux 驱动测试')
    print('=' * 50)
    print('  端口: %s' % port)
    print()

    glove = NovaGlove(port)
    glove.start()
    time.sleep(0.5)

    # 1. 读数据
    print('【1】传感器数据 (5秒)')
    t0 = time.time()
    while time.time() - t0 < 5:
        data = glove.get_sensor_data()
        if data:
            s = data['sensors']
            print('  传感器: %-40s  电池: %d%%' % (s, data['battery']))
        time.sleep(0.1)

    print()
    print('帧率: %.0f Hz' % (glove.frame_count / 5.0))

    # 2. 逐指制动
    fingers = ['拇指', '食指', '中指', '无名指']
    for i, name in enumerate(fingers):
        print('\n【%d】制动 %s (2秒)' % (i + 2, name))
        ffb = [0.0, 0.0, 0.0, 0.0]
        ffb[i] = 1.0
        for _ in range(40):
            glove.set_force_feedback(ffb)
            time.sleep(0.05)
        # 释放
        glove.set_force_feedback([0.0, 0.0, 0.0, 0.0])
        time.sleep(0.5)

    # 3. 全部制动
    print('\n【6】全部制动 (3秒)')
    for _ in range(60):
        glove.set_force_feedback([1.0, 1.0, 1.0, 1.0])
        time.sleep(0.05)

    print('【7】释放')
    glove.set_force_feedback([0.0, 0.0, 0.0, 0.0])
    time.sleep(0.5)

    glove.stop()
    print('\n完成!')
