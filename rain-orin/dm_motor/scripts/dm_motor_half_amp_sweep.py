#!/usr/bin/env python3
"""
=====================================================
  达妙电机 — 半幅摆动测试 (第三轮交互)
=====================================================

读取 dm_motor_calibration.json 里之前两轮交互记录的两侧极值,
在工作区(min+5°, max-5°)内做"半幅"正弦摆动。

半幅 = 工作区跨度 / 4 (从中心 ±半幅 = 全幅一半)

流程:
  1. 当前位置 → cosine ramp 到工作区中心 (3 秒)
  2. 中心 hold 1 秒
  3. Yaw 单独半幅扫描 15 秒 (Pitch 保持中心)
  4. Yaw 回中心 + 暂停 2 秒
  5. Pitch 单独半幅扫描 15 秒 (Yaw 保持中心)
  6. Pitch 回中心
  7. 中心 hold 3 秒后退出 (电机保持 enable)

物理映射:
  电机1 (CAN 0x01) = Yaw   (左右转头)
  电机2 (CAN 0x02) = Pitch (俯仰点头)

用法:
  /usr/bin/python3 20260407-cc-dm_motor_half_amp_sweep.py

作者: Claude Code
日期: 2026-04-07
"""

import sys, time, math, json, signal
sys.path.insert(0, '/home/rhz/teleop/DM_Control_Python')
import serial
from DM_CAN import Motor, MotorControl, DM_Motor_Type, Control_Type

CALIB_PATH = '/home/rhz/teleop/scripts/dm_motor_calibration.json'

# ============================================================
# 控制参数
# ============================================================
SAFETY_MARGIN_DEG = 5.0     # 工作区每侧内缩
KP, KD            = 8.0, 1.2
CTRL_FREQ         = 200
DT                = 1.0 / CTRL_FREQ
SWEEP_FREQ        = 0.2     # 0.2 Hz = 一次往返 5 秒
SWEEP_DURATION    = 15.0    # 每个轴扫 15 秒 (3 个完整周期)
SOFT_START        = 1.5
RAMP_DURATION     = 3.0     # 从当前位置 ramp 到中心


def main():
    # ----- 加载校准 -----
    with open(CALIB_PATH) as f:
        calib = json.load(f)

    yaw_min   = calib['summary']['motor1_yaw_can01']['min_deg']   + SAFETY_MARGIN_DEG
    yaw_max   = calib['summary']['motor1_yaw_can01']['max_deg']   - SAFETY_MARGIN_DEG
    pitch_min = calib['summary']['motor2_pitch_can02']['min_deg'] + SAFETY_MARGIN_DEG
    pitch_max = calib['summary']['motor2_pitch_can02']['max_deg'] - SAFETY_MARGIN_DEG

    yaw_center     = (yaw_min + yaw_max) / 2
    yaw_half_amp   = (yaw_max - yaw_min) / 4
    pitch_center   = (pitch_min + pitch_max) / 2
    pitch_half_amp = (pitch_max - pitch_min) / 4

    print("=" * 60)
    print("  半幅摆动测试 (在工作区中心附近)")
    print("=" * 60)
    print(f"  Yaw   工作区 [{yaw_min:+6.2f}°, {yaw_max:+6.2f}°]  "
          f"中心 {yaw_center:+6.2f}°  半幅 ±{yaw_half_amp:.2f}°")
    print(f"  Pitch 工作区 [{pitch_min:+6.2f}°, {pitch_max:+6.2f}°]  "
          f"中心 {pitch_center:+6.2f}°  半幅 ±{pitch_half_amp:.2f}°")

    # 角度 → 弧度
    yc_rad = math.radians(yaw_center);   yh_rad = math.radians(yaw_half_amp)
    pc_rad = math.radians(pitch_center); ph_rad = math.radians(pitch_half_amp)

    # 硬限幅 (任何情况都不超过 工作区+1°)
    yaw_hard_lo = math.radians(yaw_min - 1);   yaw_hard_hi = math.radians(yaw_max + 1)
    pit_hard_lo = math.radians(pitch_min - 1); pit_hard_hi = math.radians(pitch_max + 1)

    def clamp(v, lo, hi): return max(lo, min(hi, v))

    # ----- 打开串口 + 创建电机 -----
    ser = serial.Serial('/dev/ttyACM0', 921600, timeout=1)
    mc  = MotorControl(ser)
    m_yaw   = Motor(DM_Motor_Type.DM4310, 0x01, 0x11)
    m_pitch = Motor(DM_Motor_Type.DM4310, 0x02, 0x12)
    mc.addMotor(m_yaw)
    mc.addMotor(m_pitch)
    time.sleep(0.3)

    mc.switchControlMode(m_yaw,   Control_Type.MIT)
    mc.switchControlMode(m_pitch, Control_Type.MIT)
    mc.enable(m_yaw)
    mc.enable(m_pitch)
    time.sleep(0.3)

    # warmup 读初始位置
    for _ in range(3):
        mc.refresh_motor_status(m_yaw)
        mc.refresh_motor_status(m_pitch)
        time.sleep(0.05)
    cur_y = float(m_yaw.getPosition())
    cur_p = float(m_pitch.getPosition())
    print(f"\n  当前位置: Yaw {math.degrees(cur_y):+.2f}°  Pitch {math.degrees(cur_p):+.2f}°")
    print(f"  → ramp 到中心 [Yaw {yaw_center:+.2f}°, Pitch {pitch_center:+.2f}°]")

    # Ctrl+C 安全标志
    running = [True]
    def on_sig(sig, frame):
        running[0] = False
        print("\n  [SIGINT] 收到 Ctrl+C, 当前阶段跑完会退出...")
    signal.signal(signal.SIGINT, on_sig)

    for n in [3, 2, 1]:
        print(f"  {n}...")
        time.sleep(1)

    # ----- Step 1: cosine ease ramp 到中心 -----
    print(f"\n[Step 1/5] Ramp 到中心 ({RAMP_DURATION}s)")
    ramp_steps = int(RAMP_DURATION * CTRL_FREQ)
    for i in range(ramp_steps):
        if not running[0]: break
        ratio  = (i + 1) / ramp_steps
        smooth = 0.5 - 0.5 * math.cos(math.pi * ratio)
        ty = cur_y + (yc_rad - cur_y) * smooth
        tp = cur_p + (pc_rad - cur_p) * smooth
        mc.controlMIT(m_yaw,   KP, KD, clamp(ty, yaw_hard_lo, yaw_hard_hi), 0, 0)
        mc.controlMIT(m_pitch, KP, KD, clamp(tp, pit_hard_lo, pit_hard_hi), 0, 0)
        time.sleep(DT)

    # ----- Step 2: 中心 hold 1 秒 -----
    print(f"[Step 2/5] 中心 hold 1 秒")
    for _ in range(int(CTRL_FREQ)):
        if not running[0]: break
        mc.controlMIT(m_yaw,   KP, KD, yc_rad, 0, 0)
        mc.controlMIT(m_pitch, KP, KD, pc_rad, 0, 0)
        time.sleep(DT)

    # ----- Step 3: Yaw 半幅扫描 -----
    if running[0]:
        print(f"\n[Step 3/5] Yaw 半幅扫描 ±{yaw_half_amp:.2f}° 围绕 {yaw_center:+.2f}°, "
              f"{SWEEP_FREQ}Hz, {SWEEP_DURATION:.0f}s  ★ 应该看到'左右转头'")
        sweep_steps = int(SWEEP_DURATION * CTRL_FREQ)
        t0 = time.time()
        for i in range(sweep_steps):
            if not running[0]: break
            elapsed = time.time() - t0
            env = min(1.0, elapsed / SOFT_START)
            sw  = math.sin(2 * math.pi * SWEEP_FREQ * elapsed)
            ty  = yc_rad + yh_rad * env * sw
            mc.controlMIT(m_yaw,   KP, KD, clamp(ty, yaw_hard_lo, yaw_hard_hi), 0, 0)
            mc.controlMIT(m_pitch, KP, KD, pc_rad, 0, 0)
            if i % CTRL_FREQ == 0:
                mc.refresh_motor_status(m_yaw)
                actual = float(m_yaw.getPosition())
                print(f"           t={elapsed:5.1f}s  目标 {math.degrees(ty):+6.2f}°  "
                      f"实际 {math.degrees(actual):+6.2f}°")
            time.sleep(DT)

    # Yaw 回中心 0.5s
    print(f"           Yaw 回中心 (0.5s)")
    ramp_back_steps = int(0.5 * CTRL_FREQ)
    y_at_end = float(m_yaw.getPosition())
    for i in range(ramp_back_steps):
        if not running[0]: break
        ratio = (i + 1) / ramp_back_steps
        smooth = 0.5 - 0.5 * math.cos(math.pi * ratio)
        ty = y_at_end + (yc_rad - y_at_end) * smooth
        mc.controlMIT(m_yaw,   KP, KD, clamp(ty, yaw_hard_lo, yaw_hard_hi), 0, 0)
        mc.controlMIT(m_pitch, KP, KD, pc_rad, 0, 0)
        time.sleep(DT)

    # 暂停 2 秒
    print(f"           中心暂停 2 秒")
    for _ in range(int(2 * CTRL_FREQ)):
        if not running[0]: break
        mc.controlMIT(m_yaw,   KP, KD, yc_rad, 0, 0)
        mc.controlMIT(m_pitch, KP, KD, pc_rad, 0, 0)
        time.sleep(DT)

    # ----- Step 4: Pitch 半幅扫描 -----
    if running[0]:
        print(f"\n[Step 4/5] Pitch 半幅扫描 ±{pitch_half_amp:.2f}° 围绕 {pitch_center:+.2f}°, "
              f"{SWEEP_FREQ}Hz, {SWEEP_DURATION:.0f}s  ★ 应该看到'点头/抬头'")
        sweep_steps = int(SWEEP_DURATION * CTRL_FREQ)
        t0 = time.time()
        for i in range(sweep_steps):
            if not running[0]: break
            elapsed = time.time() - t0
            env = min(1.0, elapsed / SOFT_START)
            sw  = math.sin(2 * math.pi * SWEEP_FREQ * elapsed)
            tp  = pc_rad + ph_rad * env * sw
            mc.controlMIT(m_yaw,   KP, KD, yc_rad, 0, 0)
            mc.controlMIT(m_pitch, KP, KD, clamp(tp, pit_hard_lo, pit_hard_hi), 0, 0)
            if i % CTRL_FREQ == 0:
                mc.refresh_motor_status(m_pitch)
                actual = float(m_pitch.getPosition())
                print(f"           t={elapsed:5.1f}s  目标 {math.degrees(tp):+6.2f}°  "
                      f"实际 {math.degrees(actual):+6.2f}°")
            time.sleep(DT)

    # Pitch 回中心 0.5s
    print(f"           Pitch 回中心 (0.5s)")
    p_at_end = float(m_pitch.getPosition())
    for i in range(ramp_back_steps):
        if not running[0]: break
        ratio = (i + 1) / ramp_back_steps
        smooth = 0.5 - 0.5 * math.cos(math.pi * ratio)
        tp = p_at_end + (pc_rad - p_at_end) * smooth
        mc.controlMIT(m_yaw,   KP, KD, yc_rad, 0, 0)
        mc.controlMIT(m_pitch, KP, KD, clamp(tp, pit_hard_lo, pit_hard_hi), 0, 0)
        time.sleep(DT)

    # ----- Step 5: 最终中心 hold 3 秒 -----
    print(f"\n[Step 5/5] 中心 hold 3 秒后退出 (电机保持 enable)")
    for _ in range(int(3 * CTRL_FREQ)):
        mc.controlMIT(m_yaw,   KP, KD, yc_rad, 0, 0)
        mc.controlMIT(m_pitch, KP, KD, pc_rad, 0, 0)
        time.sleep(DT)

    print("\n" + "=" * 60)
    print(f"  ✓ 完成. 电机停在 [Yaw {yaw_center:+.2f}°, Pitch {pitch_center:+.2f}°]")
    print("=" * 60)
    ser.close()


if __name__ == "__main__":
    main()
