#!/usr/bin/env python3
"""
=====================================================
  达妙电机 — 永久零点校准
=====================================================

把电机当前的物理位置定义为零点, 并写入 Flash (断电不丢)。
只需要在硬件首次安装、或者电机被拆装过之后跑一次。

操作步骤:
  1. 把电机摆到你想要的"零位"物理姿态 (用手掰)
  2. 跑这个脚本
  3. 脚本会执行: enable → set_zero_position(0xFE) → save_motor_param(0xAA)
                → disable → enable → 验证位置接近 0
  4. 完成, 此后任何时候上电这个位置都是 0

注意:
  - 这个零点是相对的, 不一定要在机械中央
  - 后面 axis_verify / vp_control 都会基于这个零点工作
  - 如果想换零点, 重新跑这个脚本会覆盖 Flash 里的值

用法:
  /usr/bin/python3 20260407-cc-dm_motor_zero_calibration.py
  /usr/bin/python3 20260407-cc-dm_motor_zero_calibration.py --serial /dev/ttyACM1

作者: Claude Code
日期: 2026-04-07
"""

import sys, os, time, math, argparse
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))), "DM_Control_Python"))
import serial
from DM_CAN import Motor, MotorControl, DM_Motor_Type, Control_Type


def find_serial_port(preferred: str) -> str:
    """如果 preferred 不存在, 自动遍历 ttyACM0-3 找一个存在的"""
    if os.path.exists(preferred):
        return preferred
    for c in ["/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyACM2", "/dev/ttyACM3"]:
        if os.path.exists(c):
            print(f"  [提示] {preferred} 不存在, 自动用 {c}")
            return c
    print(f"  [错误] 找不到任何 /dev/ttyACM* 串口, 请检查 USB 连接")
    sys.exit(1)


def main():
    p = argparse.ArgumentParser(description="达妙电机 永久零点校准")
    p.add_argument("--serial", default="/dev/ttyACM0",
                   help="串口路径 (达妙 HDSC USB-CAN, 默认 /dev/ttyACM0)")
    args = p.parse_args()

    print("=" * 55)
    print("  达妙电机 — 永久零点校准")
    print("  (使能 → 设零点 → 写 Flash → 失能 → 使能)")
    print("=" * 55)

    port = find_serial_port(args.serial)
    print(f"\n  打开串口 {port} @ 921600...")
    ser = serial.Serial(port, 921600, timeout=1)
    mc  = MotorControl(ser)

    # 两个电机 (CAN_ID 0x01 + 0x02)
    m1 = Motor(DM_Motor_Type.DM4310, 0x01, 0x11)
    m2 = Motor(DM_Motor_Type.DM4310, 0x02, 0x12)
    mc.addMotor(m1)
    mc.addMotor(m2)
    time.sleep(0.5)

    motors = [m1, m2]
    labels = ["电机1(0x01)", "电机2(0x02)"]

    # ---------- 步骤 1: 切到 MIT 模式 + 使能 ----------
    print("\n[1/5] 切到 MIT 模式 + 使能...")
    for m in motors:
        mc.switchControlMode(m, Control_Type.MIT)
    for m in motors:
        mc.enable(m)
    time.sleep(0.3)

    # 读使能后位置 (这就是当前的"原点之前"角度)
    print("\n  设零点之前的位置:")
    for m, label in zip(motors, labels):
        mc.refresh_motor_status(m)
        p_now = float(m.getPosition())
        print(f"    {label}: {p_now:+.4f} rad ({math.degrees(p_now):+7.2f}°)")

    # ---------- 步骤 2: 设置零点 (RAM, 0xFE) ----------
    print("\n[2/5] 发送 set_zero_position (0xFE) - 把当前位置定义为零点...")
    for m in motors:
        mc.set_zero_position(m)
    time.sleep(0.3)
    print("  ✓ RAM 零点已设置")

    # ---------- 步骤 3: 写 Flash (save_motor_param, 0xAA) ----------
    print("\n[3/5] save_motor_param - 写入 Flash (永久保存, 断电不丢)...")
    for m, label in zip(motors, labels):
        print(f"  → 写 {label} ...")
        mc.save_motor_param(m)
        time.sleep(0.5)   # Flash 写入需要时间
    print("  ✓ Flash 已写入")

    # ---------- 步骤 4: save_motor_param 内部会失能, 显式 disable 一次确保状态干净 ----------
    print("\n[4/5] 失能电机 (清干净状态)...")
    for m in motors:
        try:
            mc.disable(m)
        except Exception as e:
            print(f"  [警告] disable 异常: {e}")
    time.sleep(0.3)
    print("  ✓ 电机已失能")

    # ---------- 步骤 5: 重新切模式 + 使能 ----------
    print("\n[5/5] 重新切到 MIT 模式 + 使能...")
    for m in motors:
        mc.switchControlMode(m, Control_Type.MIT)
    for m in motors:
        mc.enable(m)
    time.sleep(0.3)
    print("  ✓ 电机已重新使能")

    # 验证: 读位置, 应该是接近 0
    print("\n  设零点之后的位置 (应该接近 0):")
    all_ok = True
    for m, label in zip(motors, labels):
        mc.refresh_motor_status(m)
        p_now = float(m.getPosition())
        deg = math.degrees(p_now)
        status = "✓" if abs(deg) < 1.0 else "⚠"
        if abs(deg) >= 1.0:
            all_ok = False
        print(f"    {status} {label}: {p_now:+.4f} rad ({deg:+7.2f}°)")

    print("\n" + "=" * 55)
    if all_ok:
        print("  ✓ 零点已永久写入 Flash")
    else:
        print("  ⚠ 零点写入完成, 但读回值偏离 0 较多")
        print("    可能原因: Flash 写入未完成 / 电机异常 / 通讯波动")
        print("    建议: 重启电机电源后再跑一次此脚本验证")
    print("  电机当前处于使能状态, 拔电源即完全断开")
    print("=" * 55)

    ser.close()


if __name__ == "__main__":
    main()
