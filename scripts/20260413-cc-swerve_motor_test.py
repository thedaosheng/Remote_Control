#!/usr/bin/env python3
"""
==============================================================
  舵轮底盘 DM4310 电机扫描 & 测试
==============================================================

功能：
  1. 扫描两个 USB-CAN 适配器 (/dev/ttyACM0, /dev/ttyACM1)
  2. 依次尝试与 SlaveID 1~8 的电机通信
  3. 读取电机参数（PMAX, VMAX, TMAX, 固件版本）确认通信正常
  4. 可选：使能电机并做小幅正弦摆动验证

硬件前提：
  - 达妙 DM4310 电机通过 DISCOVER Robotics USB-CAN 适配器连接
  - 串口波特率 921600 bps
  - 电机已上电

用法：
  python3 /home/rhz/teleop/scripts/20260413-cc-swerve_motor_test.py

  # 仅扫描，不运动
  python3 /home/rhz/teleop/scripts/20260413-cc-swerve_motor_test.py --scan-only

  # 指定适配器
  python3 /home/rhz/teleop/scripts/20260413-cc-swerve_motor_test.py --port /dev/ttyACM0

  # 指定要扫描的 ID 范围
  python3 /home/rhz/teleop/scripts/20260413-cc-swerve_motor_test.py --ids 1,2,3,4
"""

import sys
import os
import time
import math
import argparse
import serial

# 加载达妙 CAN 库
sys.path.insert(0, '/home/rhz/teleop/DM_Control_Python')
from DM_CAN import (Motor, MotorControl, DM_Motor_Type, DM_variable,
                     Control_Type)


# ========================= 默认参数 =========================

# 两个 USB-CAN 适配器的串口路径
DEFAULT_PORTS = ['/dev/ttyACM0', '/dev/ttyACM1']

# 要扫描的电机 SlaveID 范围
# 舵轮底盘标准配置: 1~4 转向, 5~8 驱动
DEFAULT_IDS = list(range(1, 9))

# MasterID = SlaveID + 0x10（达妙推荐的映射）
MASTER_ID_OFFSET = 0x10

# 串口波特率
BAUD = 921600

# 测试运动参数（小幅正弦摆动）
TEST_AMPLITUDE = 0.3    # rad (约 17°)
TEST_FREQUENCY = 0.5    # Hz
TEST_DURATION = 3.0     # 秒
TEST_KP = 10.0          # MIT 模式位置刚度（低值，安全）
TEST_KD = 1.0           # MIT 模式速度阻尼


def scan_port(port_path, motor_ids):
    """
    在指定串口上扫描电机。

    对每个 SlaveID，尝试读取 PMAX 参数：
      - 能读到 → 电机在线
      - 读不到 → 电机不在此总线或未上电

    参数:
      port_path: 串口设备路径，如 '/dev/ttyACM0'
      motor_ids: 要扫描的 SlaveID 列表

    返回:
      dict: {slave_id: {'pmax': float, 'vmax': float, 'tmax': float, 'sw_ver': int}}
      仅包含在线的电机
    """
    # 检查串口是否存在
    if not os.path.exists(port_path):
        print(f"  ✗ {port_path} 不存在，跳过")
        return {}

    try:
        ser = serial.Serial(port_path, BAUD, timeout=0.5)
    except serial.SerialException as e:
        print(f"  ✗ 无法打开 {port_path}: {e}")
        return {}

    mc = MotorControl(ser)
    found = {}

    for sid in motor_ids:
        mid = sid + MASTER_ID_OFFSET
        motor = Motor(DM_Motor_Type.DM4310, sid, mid)
        mc.addMotor(motor)

        # 尝试读取 PMAX 参数，作为通信测试
        pmax = mc.read_motor_param(motor, DM_variable.PMAX)
        if pmax is not None:
            # 电机在线，继续读取其它参数
            vmax = mc.read_motor_param(motor, DM_variable.VMAX)
            tmax = mc.read_motor_param(motor, DM_variable.TMAX)
            sw_ver = mc.read_motor_param(motor, DM_variable.sw_ver)
            ctrl_mode = mc.read_motor_param(motor, DM_variable.CTRL_MODE)

            found[sid] = {
                'pmax': pmax,
                'vmax': vmax,
                'tmax': tmax,
                'sw_ver': sw_ver,
                'ctrl_mode': ctrl_mode,
                'motor': motor,
                'mc': mc,
                'port': port_path,
            }
            mode_name = {1: 'MIT', 2: 'POS_VEL', 3: 'VEL', 4: 'Torque_Pos'}.get(
                int(ctrl_mode) if ctrl_mode else 0, '???')
            print(f"  ✓ ID={sid:2d} (Master=0x{mid:02X})  "
                  f"P_max={pmax:.1f}  V_max={vmax:.1f}  T_max={tmax:.1f}  "
                  f"FW={sw_ver}  Mode={mode_name}")
        else:
            print(f"  · ID={sid:2d}  无响应")

    # 如果没有找到任何电机，关闭串口
    if not found:
        ser.close()

    return found


def test_motor_motion(motor_info, scan_only=False):
    """
    对在线电机做小幅正弦摆动测试。

    使用 MIT 模式（阻抗控制），低刚度低阻尼，安全第一。
    摆动 TEST_DURATION 秒后回零并失能。

    参数:
      motor_info: scan_port 返回的 dict
      scan_only: 如果 True，跳过运动测试
    """
    if scan_only:
        print("\n>>> --scan-only 模式，跳过运动测试")
        return

    if not motor_info:
        print("\n>>> 没有在线电机，无法测试")
        return

    print(f"\n>>> 开始运动测试（幅度 {TEST_AMPLITUDE:.1f} rad, "
          f"频率 {TEST_FREQUENCY} Hz, 持续 {TEST_DURATION} s）")
    print(">>> 按 Ctrl+C 随时停止\n")

    # 按串口分组电机（同一串口共享 MotorControl）
    port_groups = {}
    for sid, info in motor_info.items():
        port = info['port']
        if port not in port_groups:
            port_groups[port] = {'mc': info['mc'], 'motors': []}
        port_groups[port]['motors'].append((sid, info['motor']))

    # 切换控制模式 + 使能
    for port, group in port_groups.items():
        mc = group['mc']
        for sid, motor in group['motors']:
            print(f"  切换 ID={sid} → MIT 模式...", end=' ')
            ok = mc.switchControlMode(motor, Control_Type.MIT)
            print("✓" if ok else "✗ (继续尝试)")
            time.sleep(0.05)

        for sid, motor in group['motors']:
            print(f"  使能 ID={sid}...", end=' ')
            mc.enable(motor)
            print("✓")
            time.sleep(0.05)

    # 正弦摆动
    print("\n  正弦摆动中...")
    t0 = time.time()
    dt = 0.005  # 200Hz 控制频率

    try:
        while True:
            t = time.time() - t0
            if t > TEST_DURATION:
                break

            # 正弦目标位置
            q_target = TEST_AMPLITUDE * math.sin(2.0 * math.pi * TEST_FREQUENCY * t)

            for port, group in port_groups.items():
                mc = group['mc']
                for sid, motor in group['motors']:
                    mc.controlMIT(motor, TEST_KP, TEST_KD, q_target, 0.0, 0.0)

            time.sleep(dt)

    except KeyboardInterrupt:
        print("\n  >>> 用户中断")

    # 回零 + 失能
    print("  回零中...")
    for _ in range(100):  # 0.5 秒回零
        for port, group in port_groups.items():
            mc = group['mc']
            for sid, motor in group['motors']:
                mc.controlMIT(motor, TEST_KP, TEST_KD, 0.0, 0.0, 0.0)
        time.sleep(0.005)

    for port, group in port_groups.items():
        mc = group['mc']
        for sid, motor in group['motors']:
            mc.disable(motor)
            print(f"  ✓ ID={sid} 已失能")
            time.sleep(0.05)

    print("\n>>> 运动测试完成")


def main():
    parser = argparse.ArgumentParser(description='DM4310 舵轮电机扫描测试')
    parser.add_argument('--port', type=str, default=None,
                        help='指定 USB-CAN 串口 (默认扫描 ttyACM0 和 ttyACM1)')
    parser.add_argument('--ids', type=str, default=None,
                        help='要扫描的电机 ID，逗号分隔 (默认 1~8)')
    parser.add_argument('--scan-only', action='store_true',
                        help='仅扫描，不做运动测试')
    args = parser.parse_args()

    # 解析端口列表
    ports = [args.port] if args.port else DEFAULT_PORTS

    # 解析 ID 列表
    if args.ids:
        motor_ids = [int(x.strip()) for x in args.ids.split(',')]
    else:
        motor_ids = DEFAULT_IDS

    print("=" * 60)
    print("  DM4310 舵轮底盘电机扫描")
    print("=" * 60)
    print(f"  扫描端口: {ports}")
    print(f"  扫描 ID:  {motor_ids}")
    print()

    # 扫描所有端口
    all_found = {}
    for port in ports:
        print(f"--- {port} ---")
        found = scan_port(port, motor_ids)
        all_found.update(found)
        print()

    # 汇总
    print("=" * 60)
    if all_found:
        print(f"  共发现 {len(all_found)} 个在线电机: "
              f"ID = {sorted(all_found.keys())}")

        # 按 MC02 惯例判断角色
        steer_ids = [sid for sid in all_found if sid <= 4]
        drive_ids = [sid for sid in all_found if sid >= 5]
        if steer_ids:
            print(f"  转向电机 (ID 1~4): {steer_ids}")
        if drive_ids:
            print(f"  驱动电机 (ID 5~8): {drive_ids}")
    else:
        print("  ✗ 未发现任何电机！")
        print("  检查:")
        print("    1. 电机已上电？")
        print("    2. USB-CAN 适配器指示灯亮？")
        print("    3. CAN 总线接线正确（CANH/CANL + 终端电阻 120Ω）？")
        print("    4. 串口权限？ → sudo chmod 666 /dev/ttyACM*")
        return

    print("=" * 60)

    # 运动测试
    test_motor_motion(all_found, scan_only=args.scan_only)

    # 关闭所有串口
    closed_ports = set()
    for sid, info in all_found.items():
        port = info['port']
        if port not in closed_ports:
            try:
                info['mc'].serial_.close()
            except:
                pass
            closed_ports.add(port)

    print("\n>>> 所有串口已关闭，测试结束")


if __name__ == '__main__':
    main()
