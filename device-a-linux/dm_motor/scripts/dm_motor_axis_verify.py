#!/usr/bin/env python3
"""
=====================================================
  达妙电机 — 零点校准 + 编码器读取 + 单轴运动验证
=====================================================

这个脚本用于在接好达妙电机硬件后:
  1. 把当前位置永久设为零点(写 Flash, 断电不丢)
  2. 失能电机, 进入"互动读编码器"模式 ——
     用户用手掰电机, 脚本实时打印绝对位置(rad / 度),
     用户按回车记录极限位置, 得到机械限位的幅度范围
  3. 重新使能, 单独让 Pitch 电机做小幅度往复 (±15°/±30°)
     用户观察实际物理轴动的是 Pitch (点头) 还是其他
  4. 单独让 Yaw 电机做小幅度往复
     用户观察实际物理轴动的是 Yaw (左右转头) 还是其他
  5. 全程不超过 ±30°,任何阶段 Ctrl+C 都安全归零

测试目的:
  - 确认 CAN_ID 0x01 和 0x02 哪个对应物理 Pitch 轴, 哪个对应 Yaw 轴
  - 确认电机正方向与 VisionPro 的 PITCH/YAW 正方向是否一致
  - 测出每个轴的机械限位范围 (用户掰)
  - 在 ±30° 安全范围内确认运动平滑

电机分配 (与之前 visionpro_control.py 一致, 这次重新验证):
  电机1 (CAN_ID=0x01, MasterID=0x11) ← 之前定义为 motor_yaw
  电机2 (CAN_ID=0x02, MasterID=0x12) ← 之前定义为 motor_pitch
  ★ 这次测试就是要确认这个映射对不对!

用法:
  /usr/bin/python3 20260407-cc-dm_motor_axis_verify.py
  /usr/bin/python3 20260407-cc-dm_motor_axis_verify.py --serial /dev/ttyACM0
  /usr/bin/python3 20260407-cc-dm_motor_axis_verify.py --skip-zero  # 跳过设零点(已经设过)
  /usr/bin/python3 20260407-cc-dm_motor_axis_verify.py --amplitude 15  # 振幅 ±15°(默认 30°)

作者: Claude Code
日期: 2026-04-07
"""

import sys
import os
import time
import math
import argparse
import signal
import threading

# ====================================================================
# 加载达妙驱动库 (脚本在 scripts/, 库在上一级 DM_Control_Python/)
# ====================================================================
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DM_LIB_PATH = os.path.join(os.path.dirname(SCRIPT_DIR), "DM_Control_Python")
sys.path.insert(0, DM_LIB_PATH)

import serial
from DM_CAN import Motor, MotorControl, DM_Motor_Type, Control_Type

# ====================================================================
# 安全常量 (改这里调节实验严苛度)
# ====================================================================
DEFAULT_AMPLITUDE_DEG = 30.0    # 默认正弦扫描振幅 (±度)
HARD_LIMIT_DEG        = 35.0    # 硬限幅 ±度, 任何情况下都不超过
SWEEP_FREQ_HZ         = 0.25    # 正弦波频率 (一次往返 4 秒)
SWEEP_DURATION_SEC    = 12.0    # 每个轴扫描总时长 (大约 3 个完整周期)
CTRL_FREQ_HZ          = 200     # 控制循环频率
SOFT_START_SECS       = 1.5     # 振幅线性淡入

# MIT 控制参数 (低刚度 = 安全, 测试时电机会"偏软", 这是 OK 的)
KP = 8.0    # 位置刚度
KD = 1.2    # 速度阻尼

# 电机定义 —— 这次的测试就是要确认 CAN_ID 和物理轴的映射
# 命名暂时按 "电机1" "电机2", 不假设它对应哪个物理轴
MOTOR1_CAN_ID    = 0x01
MOTOR1_MASTER_ID = 0x11
MOTOR2_CAN_ID    = 0x02
MOTOR2_MASTER_ID = 0x12


# ====================================================================
# 工具: 自动找串口
# ====================================================================
def find_serial_port(preferred: str) -> str:
    """如果 preferred 不存在, 自动遍历 ttyACM0-3 找一个存在的"""
    if os.path.exists(preferred):
        return preferred
    for c in ["/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyACM2", "/dev/ttyACM3"]:
        if os.path.exists(c):
            print(f"  [提示] {preferred} 不存在, 自动用 {c}")
            return c
    print(f"  [错误] 找不到任何可用串口, 请检查 USB 连接")
    sys.exit(1)


# ====================================================================
# 工具: 安全归零 + 失能
# ====================================================================
def safe_disable(mc, motors):
    """先慢慢回零, 再失能"""
    print("  → 安全归零...")
    try:
        for m in motors:
            mc.refresh_motor_status(m)
        starts = [m.getPosition() for m in motors]

        steps = 200
        dt = 1.0 / CTRL_FREQ_HZ
        for i in range(steps):
            ratio = (i + 1) / steps
            smooth = 1.0 - (1.0 - ratio) ** 2     # ease-out
            for m, s in zip(motors, starts):
                p = s * (1.0 - smooth)
                mc.controlMIT(m, KP, KD, p, 0.0, 0.0)
            time.sleep(dt)

        # 在零位 hold 0.5 秒
        for _ in range(int(CTRL_FREQ_HZ * 0.5)):
            for m in motors:
                mc.controlMIT(m, KP, KD, 0.0, 0.0, 0.0)
            time.sleep(dt)
    except Exception as e:
        print(f"  [警告] 归零异常: {e}")

    print("  → 失能电机...")
    for m in motors:
        try:
            mc.disable(m)
        except Exception:
            pass


# ====================================================================
# 步骤 1 — 设置零点并写 Flash
# ====================================================================
def step_set_zero(mc, motors):
    """
    把当前位置定义为零点, 并写入 Flash (永久, 断电不丢)。
    每次执行 save_motor_param 后电机会失能, 需要再 enable。
    """
    print("\n" + "=" * 55)
    print("  步骤 1 — 永久设置零点")
    print("=" * 55)
    print("  ★ 把电机摆到你想要的'零位'物理姿态, 然后按回车")
    input("  按回车继续 → ")

    # 切到 MIT 模式 + 使能 (零点命令需要电机活着)
    for m in motors:
        mc.switchControlMode(m, Control_Type.MIT)
    for m in motors:
        mc.enable(m)
    time.sleep(0.3)

    # 在 RAM 中设置零点 (0xFE)
    print("  → 发送 set_zero_position (0xFE) ...")
    for m in motors:
        mc.set_zero_position(m)
    time.sleep(0.3)

    # 写 Flash (0xAA), save_motor_param 内部会先 disable
    print("  → 写入 Flash (save_motor_param, 0xAA) ...")
    for m in motors:
        mc.save_motor_param(m)
        time.sleep(0.5)         # 每个电机给 Flash 写入留时间

    # 重新使能并验证
    print("  → 重新使能并读位置...")
    for m in motors:
        mc.switchControlMode(m, Control_Type.MIT)
    for m in motors:
        mc.enable(m)
    time.sleep(0.3)

    for i, m in enumerate(motors, start=1):
        mc.refresh_motor_status(m)
        p = m.getPosition()
        print(f"    电机{i} (CAN_ID=0x{m.SlaveID:02X}): {p:+.4f} rad ({math.degrees(p):+6.2f}°)")
    print("  ✓ 零点已永久写入 Flash, 下次上电也保持")


# ====================================================================
# 步骤 2 — 互动读编码器 (用户掰电机, 我读位置)
# ====================================================================
def step_read_encoders_interactive(mc, motors, motor_labels):
    """
    失能两个电机, 让用户手掰, 实时打印 Position(rad/deg)。
    用户按 Ctrl+C 退出本步骤(脚本会接住, 进下一步)。
    """
    print("\n" + "=" * 55)
    print("  步骤 2 — 互动读编码器 (用户掰电机)")
    print("=" * 55)
    print("  现在两个电机会被失能, 你可以用手掰电机到机械限位,")
    print("  脚本会实时打印每个电机的绝对位置 (rad 和 度)。")
    print()
    print("  目的: 找到每个轴的机械极限范围, 用于后续设定安全限幅")
    print("  退出本步骤: 按 Ctrl+C, 脚本会接住, 自动进入下一步")
    print()
    input("  按回车 → 失能电机进入读取模式")

    # 失能电机, 让用户能转动
    for m in motors:
        try:
            mc.disable(m)
        except Exception:
            pass
    time.sleep(0.2)
    print("  ✓ 电机已失能, 可以掰了\n")

    # 跟踪 min/max
    min_pos = [float('inf'), float('inf')]
    max_pos = [-float('inf'), -float('inf')]

    interrupted = threading.Event()

    def on_sig(sig, frame):
        interrupted.set()

    old_handler = signal.signal(signal.SIGINT, on_sig)

    try:
        last_print = 0.0
        while not interrupted.is_set():
            # 刷新所有电机状态 (从总线读最新位置)
            for m in motors:
                try:
                    mc.refresh_motor_status(m)
                except Exception:
                    pass

            # 提取位置
            positions = []
            for i, m in enumerate(motors):
                p = m.getPosition()
                positions.append(p)
                if p < min_pos[i]: min_pos[i] = p
                if p > max_pos[i]: max_pos[i] = p

            now = time.time()
            if now - last_print > 0.1:    # 10 Hz 打印
                line = "  "
                for i, p in enumerate(positions):
                    line += (f"{motor_labels[i]}: {p:+7.3f} rad "
                             f"({math.degrees(p):+7.2f}°)  "
                             f"[min {math.degrees(min_pos[i]):+6.1f}° "
                             f"max {math.degrees(max_pos[i]):+6.1f}°]  ")
                print("\r" + line, end="", flush=True)
                last_print = now

            time.sleep(0.02)
    finally:
        signal.signal(signal.SIGINT, old_handler)

    print()    # 换行
    print("\n  ✓ 退出读取模式")
    print(f"\n  === 各轴绝对极限范围 ===")
    for i, label in enumerate(motor_labels):
        rng_deg = math.degrees(max_pos[i] - min_pos[i])
        print(f"    {label}: "
              f"min {math.degrees(min_pos[i]):+7.2f}° → "
              f"max {math.degrees(max_pos[i]):+7.2f}° "
              f"(总幅度 {rng_deg:.1f}°)")
    return min_pos, max_pos


# ====================================================================
# 步骤 3 — 单轴正弦扫描验证物理映射
# ====================================================================
def step_single_axis_sweep(mc, active_motor, hold_motors, amplitude_deg, label):
    """
    让 active_motor 做正弦扫描 ±amplitude_deg, 其他电机保持在 0。
    用户观察哪个物理轴在动。
    """
    amp_rad = math.radians(min(amplitude_deg, HARD_LIMIT_DEG))    # 硬限幅
    print("\n" + "=" * 55)
    print(f"  步骤 3 — {label} 单轴运动")
    print("=" * 55)
    print(f"  ★ 让 [{label}] 做正弦运动: 振幅 ±{math.degrees(amp_rad):.0f}°, "
          f"频率 {SWEEP_FREQ_HZ} Hz, 持续 {SWEEP_DURATION_SEC:.0f} 秒")
    print(f"  其他电机保持在 0°(锁紧)")
    print()
    print("  请观察物理上哪个轴在动 (点头/转头/侧倾)")
    print("  Ctrl+C 可以提前结束并安全归零")
    input("  按回车开始 → ")

    # 重新使能
    mc.switchControlMode(active_motor, Control_Type.MIT)
    mc.enable(active_motor)
    for hm in hold_motors:
        mc.switchControlMode(hm, Control_Type.MIT)
        mc.enable(hm)
    time.sleep(0.2)

    dt = 1.0 / CTRL_FREQ_HZ
    steps = int(SWEEP_DURATION_SEC * CTRL_FREQ_HZ)
    t0 = time.time()
    interrupted = threading.Event()

    def on_sig(sig, frame):
        interrupted.set()
    old_handler = signal.signal(signal.SIGINT, on_sig)

    try:
        for i in range(steps):
            if interrupted.is_set():
                print("\n  [中断] 提前结束扫描")
                break

            elapsed = time.time() - t0

            # 软启动: 前 SOFT_START_SECS 秒振幅线性淡入, 防突跳
            envelope = min(1.0, elapsed / SOFT_START_SECS)
            target = amp_rad * envelope * math.sin(2 * math.pi * SWEEP_FREQ_HZ * elapsed)

            # 硬限幅
            target = max(-math.radians(HARD_LIMIT_DEG),
                         min( math.radians(HARD_LIMIT_DEG), target))

            mc.controlMIT(active_motor, KP, KD, target, 0.0, 0.0)
            for hm in hold_motors:
                mc.controlMIT(hm, KP, KD, 0.0, 0.0, 0.0)

            # 每秒打印一次
            if i % CTRL_FREQ_HZ == 0:
                mc.refresh_motor_status(active_motor)
                actual = active_motor.getPosition()
                print(f"    t={elapsed:5.1f}s | 目标 {math.degrees(target):+6.1f}° | "
                      f"实际 {math.degrees(actual):+6.1f}°")

            time.sleep(dt)
    finally:
        signal.signal(signal.SIGINT, old_handler)
        # 不管怎么退出, 把这一轴归零
        safe_disable(mc, [active_motor] + list(hold_motors))


# ====================================================================
# 主流程
# ====================================================================
def run(serial_port: str, amplitude_deg: float, skip_zero: bool):
    print("=" * 55)
    print("  达妙电机 - 零点校准 / 编码器读取 / 单轴验证")
    print("=" * 55)
    print(f"  串口:        {serial_port}")
    print(f"  扫描振幅:    ±{amplitude_deg:.0f}°  (硬限幅 ±{HARD_LIMIT_DEG:.0f}°)")
    print(f"  MIT KP:      {KP}  KD: {KD}")
    print(f"  跳过设零点:  {'是' if skip_zero else '否'}")
    print("=" * 55)

    # ----------------- 打开串口 -----------------
    port = find_serial_port(serial_port)
    print(f"\n  打开串口 {port} @ 921600...")
    ser = serial.Serial(port, 921600, timeout=1)
    mc  = MotorControl(ser)

    # ----------------- 创建电机对象 -----------------
    # ★ 注意: 这里只是按 CAN_ID 创建, 不假设它对应物理 Pitch 还是 Yaw
    motor1 = Motor(DM_Motor_Type.DM4310, MOTOR1_CAN_ID, MOTOR1_MASTER_ID)
    motor2 = Motor(DM_Motor_Type.DM4310, MOTOR2_CAN_ID, MOTOR2_MASTER_ID)
    mc.addMotor(motor1)
    mc.addMotor(motor2)
    motors = [motor1, motor2]
    labels = [
        f"电机1(0x{MOTOR1_CAN_ID:02X})",
        f"电机2(0x{MOTOR2_CAN_ID:02X})",
    ]

    # 验证通讯
    time.sleep(0.5)
    print("  → 验证通讯 (read PMAX) ...")
    try:
        from DM_CAN import DM_variable
        pmax1 = mc.read_motor_param(motor1, DM_variable.PMAX)
        pmax2 = mc.read_motor_param(motor2, DM_variable.PMAX)
        if pmax1 is None or pmax2 is None:
            raise RuntimeError("读 PMAX 失败")
        print(f"    {labels[0]} PMAX={pmax1}")
        print(f"    {labels[1]} PMAX={pmax2}")
        print("  ✓ 通讯正常")
    except Exception as e:
        print(f"  [错误] 通讯失败: {e}")
        print("    检查: USB 线 / 电源 / 波特率 / CAN ID 配置")
        ser.close()
        sys.exit(1)

    # 注册全局 Ctrl+C 处理 (脚本退出时安全失能)
    def on_global_exit(sig, frame):
        print("\n\n[全局退出] 安全归零并失能...")
        try:
            safe_disable(mc, motors)
        finally:
            if ser.is_open:
                ser.close()
            print("[全局退出] 完成")
            sys.exit(0)
    signal.signal(signal.SIGTERM, on_global_exit)
    # 注意: SIGINT 在每个步骤里临时接管, 不在这里全局接管

    try:
        # ----------------- 步骤 1: 设零点 -----------------
        if not skip_zero:
            step_set_zero(mc, motors)
        else:
            print("\n  [跳过] 步骤 1 设零点 (--skip-zero)")
            for m in motors:
                mc.switchControlMode(m, Control_Type.MIT)
                mc.enable(m)

        # ----------------- 步骤 2: 互动读编码器 -----------------
        min_pos, max_pos = step_read_encoders_interactive(mc, motors, labels)

        # ----------------- 步骤 3a: 电机1 单独动 -----------------
        step_single_axis_sweep(
            mc,
            active_motor=motor1,
            hold_motors=[motor2],
            amplitude_deg=amplitude_deg,
            label=labels[0],
        )
        input("\n  请记录: 物理上动的是 [Pitch/Yaw/侧倾] ? 按回车继续 → ")

        # ----------------- 步骤 3b: 电机2 单独动 -----------------
        step_single_axis_sweep(
            mc,
            active_motor=motor2,
            hold_motors=[motor1],
            amplitude_deg=amplitude_deg,
            label=labels[1],
        )
        input("\n  请记录: 物理上动的是 [Pitch/Yaw/侧倾] ? 按回车继续 → ")

        # ----------------- 总结 -----------------
        print("\n" + "=" * 55)
        print("  全部测试完成 ✓")
        print("=" * 55)
        print("  请告诉我:")
        print(f"    1. {labels[0]} 实际动的物理轴 = ?  (Pitch / Yaw / 侧倾)")
        print(f"    2. {labels[1]} 实际动的物理轴 = ?  (Pitch / Yaw / 侧倾)")
        print(f"    3. 正方向是否符合直觉 (向上/向右是不是正)?")
        print(f"    4. 编码器掰到的极限范围:")
        for i, label in enumerate(labels):
            print(f"       {label}: "
                  f"{math.degrees(min_pos[i]):+6.1f}° ~ "
                  f"{math.degrees(max_pos[i]):+6.1f}°")
        print()
        print("  根据结果, 我会更新 dm_motor_visionpro_control.py 的:")
        print("    - motor_pitch / motor_yaw 的 CAN_ID 绑定")
        print("    - 正方向系数 (是否需要 -1)")
        print("    - 安全限幅 (基于实测的机械极限)")
        print("=" * 55)

    finally:
        # 兜底: 任何异常都要安全失能
        print("\n[收尾] 失能电机, 关串口...")
        try:
            safe_disable(mc, motors)
        except Exception:
            pass
        if ser.is_open:
            ser.close()
        print("[收尾] 已退出")


# ====================================================================
# 命令行
# ====================================================================
if __name__ == "__main__":
    p = argparse.ArgumentParser(description="达妙电机 零点校准 / 编码器读取 / 单轴验证")
    p.add_argument("--serial",    default="/dev/ttyACM0",
                   help="串口设备路径 (达妙 HDSC USB-CAN, 默认 /dev/ttyACM0; "
                        "/dev/ttyACM1 是 airbot 机械臂 SocketCAN; "
                        "/dev/ttyACM2 是 Touch 力反馈笔)")
    p.add_argument("--amplitude", type=float, default=DEFAULT_AMPLITUDE_DEG,
                   help=f"扫描振幅(度), 默认 {DEFAULT_AMPLITUDE_DEG}, 硬限幅 {HARD_LIMIT_DEG}")
    p.add_argument("--skip-zero", action="store_true",
                   help="跳过设零点(零点已经写过 Flash 不想再覆盖时用)")
    args = p.parse_args()

    if args.amplitude > HARD_LIMIT_DEG:
        print(f"[警告] amplitude {args.amplitude}° 超过硬限幅 {HARD_LIMIT_DEG}°, "
              f"自动钳到 {HARD_LIMIT_DEG}°")
        args.amplitude = HARD_LIMIT_DEG

    run(args.serial, args.amplitude, args.skip_zero)
