#!/usr/bin/env python3
"""
========================================
  达妙电机 × Vision Pro 头部随动控制
========================================

数据链路:
  Vision Pro → Linux 接收进程（四元数→欧拉角）→ UDP 127.0.0.1:9000 → 本脚本 → 电机

UDP 数据格式 (JSON, ~90Hz):
  {"roll":0.0, "pitch":0.0, "yaw":0.0, "pos":[x,y,z], "quat":[qx,qy,qz,qw], "t":...}
  roll/pitch/yaw 单位为度（degree）

电机分配:
  电机1 (CAN_ID=0x01): Pitch（点头/抬头）
  电机2 (CAN_ID=0x02): Yaw（左右转头）

生命周期:
  启动 → 电机归零 → 收到数据后开始随动 → 数据超时则缓慢回零 → Ctrl+C 安全退出

依赖:
  pip install numpy pyserial

作者: Claude Code
日期: 2026-03-26
修复: 2026-04-04 — 关闭前馈力矩/速度，消除高频震动
"""

import sys
import os
import time
import math
import signal
import socket
import json
import threading
import argparse
from dataclasses import dataclass, field

# ============================================================
# 达妙控制库
# ============================================================
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DM_LIB_PATH = os.path.join(os.path.dirname(SCRIPT_DIR), "DM_Control_Python")
sys.path.insert(0, DM_LIB_PATH)

import serial
import numpy as np
from DM_CAN import (
    Motor, MotorControl, DM_Motor_Type, DM_variable, Control_Type
)


# ============================================================
# 共享状态（线程安全）
# ============================================================
@dataclass
class SharedState:
    """UDP 接收线程和电机控制循环之间的共享数据"""
    pitch: float = 0.0          # 度
    yaw: float = 0.0            # 度
    last_update: float = 0.0
    streaming: bool = False
    lock: threading.Lock = field(default_factory=threading.Lock)

    def update(self, pitch: float, yaw: float):
        with self.lock:
            self.pitch = pitch
            self.yaw = yaw
            self.last_update = time.time()
            self.streaming = True

    def get(self):
        """返回 (pitch_deg, yaw_deg, data_age_seconds)"""
        with self.lock:
            age = time.time() - self.last_update if self.last_update > 0 else float('inf')
            return self.pitch, self.yaw, age


# ============================================================
# UDP 接收线程
# ============================================================
def udp_receive_thread(state: SharedState, host: str = "127.0.0.1", port: int = 9000):
    """
    监听 UDP 端口，解析 JSON，提取 pitch 和 yaw 更新到共享状态。
    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((host, port))
    sock.settimeout(1.0)  # 1 秒超时，方便退出
    print(f"[UDP] 监听 {host}:{port}")

    _dbg_count = 0
    while True:
        try:
            data, addr = sock.recvfrom(4096)
            msg = json.loads(data.decode("utf-8"))
            pitch = float(msg.get("pitch", 0.0))
            yaw = float(msg.get("yaw", 0.0))
            state.update(pitch, yaw)
            _dbg_count += 1
            if _dbg_count % 30 == 0:   # 约 3Hz 打一次
                print(f"[UDP raw] pitch={pitch:+7.2f}°  yaw={yaw:+7.2f}°")
        except socket.timeout:
            pass
        except (json.JSONDecodeError, ValueError, KeyError):
            pass


def start_udp_thread(state: SharedState, host: str = "127.0.0.1", port: int = 9000):
    t = threading.Thread(target=udp_receive_thread, args=(state, host, port), daemon=True)
    t.start()
    return t


# ============================================================
# 电机控制器
# ============================================================
class DmMotorController:
    """
    达妙电机控制器，使用 MIT 模式（纯阻抗控制，无前馈）。

    MIT 控制公式:
      τ = KP × (q_target - q_actual) + KD × (dq_target - dq_actual) + τ_ff

    本版本只使用 KP 和 KD，dq_target 和 τ_ff 均设为 0，
    避免数值微分带来的高频噪声导致电机震动。
    """

    def __init__(self, serial_port: str, baudrate: int = 921600,
                 kp: float = 30.0, kd: float = 0.5):
        self.serial_port = serial_port
        self.baudrate = baudrate
        self.kp = kp    # 位置刚度 (0~500)
        self.kd = kd    # 速度阻尼 (0~5)
        self.ser = None
        self.mc = None
        self.motor_pitch = None   # 电机1 (0x01)
        self.motor_yaw = None     # 电机2 (0x02)
        self._enabled = False

    def connect(self):
        """打开串口，创建电机对象，验证通讯"""
        port = self.serial_port
        if not os.path.exists(port):
            for c in ["/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyACM2", "/dev/ttyACM3"]:
                if os.path.exists(c):
                    port = c

        print(f"[电机] 打开串口 {port} @ {self.baudrate}")
        self.ser = serial.Serial(port, self.baudrate, timeout=1)
        self.mc = MotorControl(self.ser)

        self.motor_pitch = Motor(DM_Motor_Type.DM4310, 0x02, 0x12)  # 物理 Pitch 轴
        self.motor_yaw = Motor(DM_Motor_Type.DM4310, 0x01, 0x11)   # 物理 Yaw 轴
        self.mc.addMotor(self.motor_pitch)
        self.mc.addMotor(self.motor_yaw)

        time.sleep(0.5)
        p1 = self.mc.read_motor_param(self.motor_pitch, DM_variable.PMAX)
        p2 = self.mc.read_motor_param(self.motor_yaw, DM_variable.PMAX)
        if p1 is None or p2 is None:
            raise RuntimeError("电机通讯失败，请检查接线和波特率")
        print(f"[电机] 通讯正常 — Pitch PMAX={p1}, Yaw PMAX={p2}")

    def enable(self):
        """切换到 MIT 模式并使能"""
        self.mc.switchControlMode(self.motor_pitch, Control_Type.MIT)
        self.mc.switchControlMode(self.motor_yaw, Control_Type.MIT)
        self.mc.enable(self.motor_pitch)
        self.mc.enable(self.motor_yaw)
        self._enabled = True
        print(f"[电机] 已使能（MIT 模式, KP={self.kp}, KD={self.kd}）")

    def go_to(self, pitch_rad: float, yaw_rad: float):
        """
        MIT 模式发送控制命令。
        纯位置+阻尼控制，不使用前馈速度和前馈力矩，避免高频震动。
        """
        if not self._enabled:
            return

        # controlMIT(motor, kp, kd, q_target, dq_target=0, tau_ff=0)
        self.mc.controlMIT(self.motor_pitch, self.kp, self.kd,
                           pitch_rad, 0, 0)
        self.mc.controlMIT(self.motor_yaw, self.kp, self.kd,
                           yaw_rad, 0, 0)

    def get_positions(self):
        return self.motor_pitch.getPosition(), self.motor_yaw.getPosition()

    def smooth_return_to_zero(self, duration: float = 2.0):
        """从当前位置 ease-out 缓慢回到零位"""
        if not self._enabled:
            return
        print(f"[电机] 缓慢回零（{duration}s）...")

        self.mc.refresh_motor_status(self.motor_pitch)
        self.mc.refresh_motor_status(self.motor_yaw)
        start_p = self.motor_pitch.getPosition()
        start_y = self.motor_yaw.getPosition()

        t0 = time.time()
        while True:
            elapsed = time.time() - t0
            if elapsed >= duration:
                break
            ratio = min(elapsed / duration, 1.0)
            smooth = 1.0 - (1.0 - ratio) ** 2
            p = start_p * (1.0 - smooth)
            y = start_y * (1.0 - smooth)
            self.mc.controlMIT(self.motor_pitch, self.kp, self.kd, p, 0, 0)
            self.mc.controlMIT(self.motor_yaw, self.kp, self.kd, y, 0, 0)
            time.sleep(0.005)

        for _ in range(100):
            self.mc.controlMIT(self.motor_pitch, self.kp, self.kd, 0, 0, 0)
            self.mc.controlMIT(self.motor_yaw, self.kp, self.kd, 0, 0, 0)
            time.sleep(0.005)
        print("[电机] 已回到零位")

    def disable(self):
        if self._enabled:
            self.mc.disable(self.motor_pitch)
            self.mc.disable(self.motor_yaw)
            self._enabled = False
            print("[电机] 已失能")

    def close(self):
        if self._enabled:
            self.smooth_return_to_zero()
            self.disable()
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("[电机] 串口已关闭")


# ============================================================
# 主控制循环
# ============================================================
def control_loop(controller: DmMotorController, state: SharedState,
                 freq: int = 200, scale: float = 1.0,
                 max_angle_rad: float = 6.0, timeout: float = 0.5):
    """
    主循环：读取 UDP 数据 → 度转弧度 → 限幅 → 发送给电机。
    数据超时时缓慢回零。

    软启动斜坡：每次数据流重新建立时（was_streaming False→True），
    从当前电机位置线性插值到目标，持续 SOFT_START_SECS 秒，
    防止 Vision Pro 已有偏角时电机突然跳变。
    """
    SOFT_START_SECS = 1.5   # 软启动时长（秒）

    dt = 1.0 / freq
    loop_count = 0
    was_streaming = False

    # 逐帧限速：每帧目标角度变化不超过此值（rad），防止 VP 数据突跳
    MAX_DELTA_PER_FRAME = math.radians(3.0)   # 3°/帧，200Hz 下约 600°/s 上限
    prev_pitch_rad = 0.0
    prev_yaw_rad   = 0.0

    # 软启动状态
    soft_start_active = False
    soft_start_t0 = 0.0
    soft_start_from_p = 0.0
    soft_start_from_y = 0.0

    print(f"\n[控制] 启动 @ {freq}Hz | 放大 ×{scale} | 限幅 ±{max_angle_rad} rad | 超时 {timeout}s")
    print("[控制] 等待 Vision Pro 数据...\n")

    try:
        while True:
            t_start = time.time()
            pitch_deg, yaw_deg, age = state.get()

            # 度 → 弧度 → 限幅 → 逐帧限速
            raw_p = max(-max_angle_rad, min(max_angle_rad, math.radians(pitch_deg) * scale))
            raw_y = max(-max_angle_rad, min(max_angle_rad, math.radians(yaw_deg) * scale))
            delta_p = raw_p - prev_pitch_rad
            delta_y = raw_y - prev_yaw_rad
            pitch_rad = prev_pitch_rad + max(-MAX_DELTA_PER_FRAME, min(MAX_DELTA_PER_FRAME, delta_p))
            yaw_rad   = prev_yaw_rad   + max(-MAX_DELTA_PER_FRAME, min(MAX_DELTA_PER_FRAME, delta_y))

            if age <= timeout:
                # ==== 有数据 ====
                if not was_streaming:
                    controller.mc.refresh_motor_status(controller.motor_pitch)
                    controller.mc.refresh_motor_status(controller.motor_yaw)
                    soft_start_from_p = controller.motor_pitch.getPosition()
                    soft_start_from_y = controller.motor_yaw.getPosition()
                    soft_start_t0 = time.time()
                    soft_start_active = True
                    was_streaming = True
                    print(f"[控制] 收到数据，软启动 {SOFT_START_SECS}s 斜坡 "
                          f"(from P={math.degrees(soft_start_from_p):+.1f}° "
                          f"Y={math.degrees(soft_start_from_y):+.1f}°)")

                if soft_start_active:
                    elapsed = time.time() - soft_start_t0
                    ratio = min(elapsed / SOFT_START_SECS, 1.0)
                    smooth = 1.0 - (1.0 - ratio) ** 2   # ease-out
                    interp_p = soft_start_from_p + (pitch_rad - soft_start_from_p) * smooth
                    interp_y = soft_start_from_y + (yaw_rad - soft_start_from_y) * smooth
                    controller.go_to(interp_p, interp_y)
                    if ratio >= 1.0:
                        soft_start_active = False
                        print("[控制] 软启动完成，进入正常随动")
                else:
                    controller.go_to(pitch_rad, yaw_rad)

            else:
                # ==== 数据超时 → 回零 ====
                if was_streaming:
                    print("[控制] 数据超时，缓慢回零...")
                    controller.smooth_return_to_zero(duration=2.0)
                    was_streaming = False
                    soft_start_active = False
                    print("[控制] 等待 Vision Pro 数据...\n")
                else:
                    controller.go_to(0, 0)

            # 每秒打印一次
            loop_count += 1
            if loop_count % freq == 0 and was_streaming:
                p_pos, y_pos = controller.get_positions()
                print(
                    f"[状态] VP: P={pitch_deg:+6.1f}° Y={yaw_deg:+6.1f}° | "
                    f"电机: P={p_pos:+.2f} Y={y_pos:+.2f} rad"
                )

            prev_pitch_rad = pitch_rad
            prev_yaw_rad   = yaw_rad

            elapsed = time.time() - t_start
            if dt - elapsed > 0:
                time.sleep(dt - elapsed)

    except KeyboardInterrupt:
        print("\n[控制] Ctrl+C")


# ============================================================
# 演示模式
# ============================================================
def demo_loop(controller: DmMotorController, freq: int = 200):
    dt = 1.0 / freq
    loop_count = 0
    print(f"\n[演示] 正弦波 @ {freq}Hz | Ctrl+C 退出\n")
    try:
        while True:
            t_start = time.time()
            t = time.time()
            p = math.sin(t * 0.8) * 2.0
            y = math.sin(t * 0.5) * 2.0
            controller.go_to(p, y)

            loop_count += 1
            if loop_count % freq == 0:
                pp, yp = controller.get_positions()
                print(f"[演示] 目标: P={p:+.2f} Y={y:+.2f} | 实际: P={pp:+.2f} Y={yp:+.2f}")

            elapsed = time.time() - t_start
            if dt - elapsed > 0:
                time.sleep(dt - elapsed)
    except KeyboardInterrupt:
        print("\n[演示] Ctrl+C")


# ============================================================
# 参数解析 & 主入口
# ============================================================
def main():
    p = argparse.ArgumentParser(description="达妙电机 × Vision Pro 头部随动")
    p.add_argument("--serial", default="/dev/ttyACM2", help="串口路径")
    p.add_argument("--udp-port", type=int, default=9000, help="UDP 监听端口（默认 9000）")
    p.add_argument("--scale", type=float, default=1.0, help="角度放大倍数")
    p.add_argument("--max-angle", type=float, default=1.0, help="电机最大角度 rad（默认 1.0 rad ≈ 57°）")
    p.add_argument("--freq", type=int, default=200, help="控制频率 Hz")
    p.add_argument("--timeout", type=float, default=0.5, help="数据超时 秒")
    p.add_argument("--kp", type=float, required=True, help="MIT 位置刚度 (0~500)")
    p.add_argument("--kd", type=float, required=True, help="MIT 速度阻尼 (0~5)")
    p.add_argument("--demo", action="store_true", help="演示模式（正弦波）")
    args = p.parse_args()

    print("=" * 50)
    print("  达妙电机 × Vision Pro 头部随动")
    print("=" * 50)
    print(f"  串口:     {args.serial}")
    print(f"  UDP:      127.0.0.1:{args.udp_port}")
    print(f"  频率:     {args.freq} Hz")
    print(f"  放大:     ×{args.scale}")
    print(f"  限幅:     ±{args.max_angle} rad")
    print(f"  MIT:      KP={args.kp}, KD={args.kd}")
    print(f"  模式:     {'演示' if args.demo else 'Vision Pro 随动'}")
    print("=" * 50)

    controller = DmMotorController(args.serial, kp=args.kp, kd=args.kd)

    def on_exit(sig, frame):
        print("\n[主程序] 安全关闭...")
        controller.close()
        sys.exit(0)
    signal.signal(signal.SIGINT, on_exit)
    signal.signal(signal.SIGTERM, on_exit)

    try:
        controller.connect()
        print("[主程序] 等待 2 秒...")
        time.sleep(2)
        controller.enable()

        # 初始化零位
        print("[主程序] 初始化零位...")
        for _ in range(200):
            controller.go_to(0, 0)
            time.sleep(0.005)
        print("[主程序] 就绪\n")

        if args.demo:
            demo_loop(controller, args.freq)
        else:
            state = SharedState()
            start_udp_thread(state, "127.0.0.1", args.udp_port)
            control_loop(controller, state, args.freq, args.scale, args.max_angle, args.timeout)

    except Exception as e:
        print(f"\n[错误] {e}")
        import traceback
        traceback.print_exc()
    finally:
        controller.close()
        print("[主程序] 已退出")


if __name__ == "__main__":
    main()