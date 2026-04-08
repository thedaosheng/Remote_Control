#!/usr/bin/env python3
"""
舵轮底盘自动测试脚本（无键盘）

依次演示:
  1. 前进 (vx=1)
  2. 侧移 (vy=1)
  3. 旋转 (omega=1)
  4. 合成 (vx=0.7, vy=0.7)
  5. 前进+旋转

每个阶段持续 3 秒，自动截图并打印底盘位置。
"""

import os
import sys
import math
import time
import numpy as np

import mujoco
import mujoco.viewer

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
MJCF_PATH = os.path.join(SCRIPT_DIR, "20260408-cc-swerve_chassis.xml")

WHEEL_RADIUS = 0.084
WHEEL_POSITIONS = np.array([
    [ 0.16,  0.15],
    [ 0.16, -0.15],
    [-0.16,  0.15],
    [-0.16, -0.15],
])


def swerve_ik(vx, vy, omega):
    sa, ds = np.zeros(4), np.zeros(4)
    for i, (rx, ry) in enumerate(WHEEL_POSITIONS):
        wvx = vx - omega * ry
        wvy = vy + omega * rx
        speed = math.hypot(wvx, wvy)
        if speed < 1e-4:
            sa[i] = 0
            ds[i] = 0
        else:
            sa[i] = math.atan2(wvy, wvx)
            ds[i] = speed / WHEEL_RADIUS
    return sa, ds


def shortest_steer(target, current):
    diff = target - current
    while diff > math.pi:
        diff -= 2 * math.pi
    while diff < -math.pi:
        diff += 2 * math.pi
    if abs(diff) > math.pi / 2:
        # 反转 180° + 反向驱动
        if diff > 0:
            target -= math.pi
        else:
            target += math.pi
        return target, True  # True = 反向驱动
    return target, False


# 测试序列：(vx, vy, omega, duration_sec, label)
TEST_PHASES = [
    (0.0, 0.0, 0.0, 1.0, "STATIC"),
    (0.5, 0.0, 0.0, 3.0, "FORWARD (W)"),
    (0.0, 0.0, 0.0, 1.0, "STOP"),
    (0.0, 0.5, 0.0, 3.0, "STRAFE LEFT (A)"),
    (0.0, 0.0, 0.0, 1.0, "STOP"),
    (0.0, 0.0, 1.0, 3.0, "ROTATE LEFT (Q)"),
    (0.0, 0.0, 0.0, 1.0, "STOP"),
    (0.4, 0.4, 0.0, 3.0, "DIAGONAL (W+A)"),
    (0.0, 0.0, 0.0, 1.0, "STOP"),
    (0.5, 0.0, 0.5, 3.0, "FORWARD+ROTATE (W+Q)"),
    (0.0, 0.0, 0.0, 2.0, "FINAL STOP"),
]


def main():
    model = mujoco.MjModel.from_xml_path(MJCF_PATH)
    data = mujoco.MjData(model)

    print(f"MJCF loaded: nq={model.nq}, nv={model.nv}, nu={model.nu}")

    current_steer = np.zeros(4)
    phase_idx = 0
    phase_start_t = 0.0

    # 找到 chassis_link 的 body id（用于读取位置）
    chassis_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "chassis_link")

    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.distance = 3.5
        viewer.cam.azimuth = 135
        viewer.cam.elevation = -30
        viewer.cam.lookat[:] = [0, 0, 0.1]

        last_log = 0.0

        while viewer.is_running() and phase_idx < len(TEST_PHASES):
            sim_time = data.time
            phase_t = sim_time - phase_start_t
            vx_t, vy_t, om_t, duration, label = TEST_PHASES[phase_idx]

            # 切换阶段
            if phase_t >= duration:
                phase_idx += 1
                phase_start_t = sim_time
                if phase_idx < len(TEST_PHASES):
                    print(f"\n>>> Phase {phase_idx}: {TEST_PHASES[phase_idx][4]}")
                continue

            # 逆运动学
            steer_angles, drive_speeds = swerve_ik(vx_t, vy_t, om_t)

            # 转向角优化
            for i in range(4):
                if abs(drive_speeds[i]) > 1e-4:
                    steer_angles[i], reversed_ = shortest_steer(steer_angles[i], current_steer[i])
                    if reversed_:
                        drive_speeds[i] = -drive_speeds[i]
                current_steer[i] = steer_angles[i]

            # 写入执行器
            data.ctrl[0:4] = steer_angles
            data.ctrl[4:8] = drive_speeds

            # 步进
            mujoco.mj_step(model, data)
            viewer.sync()

            # 日志
            if sim_time - last_log > 0.5:
                pos = data.xpos[chassis_id]
                qw = data.xquat[chassis_id, 0]
                qz = data.xquat[chassis_id, 3]
                yaw = 2.0 * math.atan2(qz, qw)
                print(f"t={sim_time:5.2f} [{label:25s}] "
                      f"pos=({pos[0]:+.2f},{pos[1]:+.2f},{pos[2]:.2f}) "
                      f"yaw={math.degrees(yaw):+6.1f}°")
                last_log = sim_time

            time.sleep(model.opt.timestep)

        print("\n=== Test sequence complete ===")
        # 保持 viewer 开启 5 秒供截图
        end_t = time.time() + 5
        while viewer.is_running() and time.time() < end_t:
            viewer.sync()
            time.sleep(0.05)


if __name__ == "__main__":
    main()
