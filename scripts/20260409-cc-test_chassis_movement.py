#!/usr/bin/env python3
"""
自动化测试：验证底盘在 8 个方向 + 旋转 的运动表现。

测试方法：
  1. 加载 MJCF 模型
  2. 对每个方向施加 3 秒 velocity actuator 控制
  3. 测量实际位移，与预期方向对比
  4. 输出 PASS/FAIL 结果

预期：所有方向位移 > 0.3m（0.6 m/s × 3s × 考虑加速），方向误差 < 15°
"""

import os
import sys
import math
import numpy as np
import mujoco

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
MJCF_PATH = os.path.join(SCRIPT_DIR, "20260408-cc-swerve_chassis.xml")

# 执行器索引（对应 XML actuator 顺序，共 28 个）
CTRL_CHASSIS_VX    = 0
CTRL_CHASSIS_VY    = 1
CTRL_CHASSIS_OMEGA = 2

# 测试用例：(名称, vx, vy, omega, 预期dx方向, 预期dy方向)
# 方向角 = atan2(dy, dx) in degrees
TEST_CASES = [
    ("前进 (+X)",      0.6,  0.0,  0.0,    0.0),   # 0°
    ("后退 (-X)",     -0.6,  0.0,  0.0,  180.0),   # 180°
    ("左移 (+Y)",      0.0,  0.6,  0.0,   90.0),   # 90°
    ("右移 (-Y)",      0.0, -0.6,  0.0,  -90.0),   # -90°
    ("左前 (+X+Y)",    0.42, 0.42, 0.0,   45.0),   # 45°
    ("右前 (+X-Y)",    0.42,-0.42, 0.0,  -45.0),   # -45°
    ("左后 (-X+Y)",   -0.42, 0.42, 0.0,  135.0),   # 135°
    ("右后 (-X-Y)",   -0.42,-0.42, 0.0, -135.0),   # -135°
]

ROTATION_TESTS = [
    ("逆时针旋转",  0.0, 0.0,  1.5),   # omega > 0
    ("顺时针旋转",  0.0, 0.0, -1.5),   # omega < 0
]

# 测试参数
TEST_DURATION = 3.0     # 秒
MIN_DISPLACEMENT = 0.3  # 最小位移阈值 (m)
MAX_DIR_ERROR = 20.0    # 最大方向误差 (°)
MIN_ROTATION = 1.0      # 最小旋转量 (rad)


def run_test(model, test_name, vx, vy, omega, duration):
    """运行单个测试，返回 (dx, dy, d_yaw)"""
    data = mujoco.MjData(model)
    mujoco.mj_resetData(model, data)
    mujoco.mj_forward(model, data)

    # 找到 chassis body
    chassis_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "chassis_link")
    pos0 = data.xpos[chassis_id].copy()

    # 找到 chassis_yaw 关节读取旋转
    yaw_jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "chassis_yaw")
    yaw_qpos_idx = model.jnt_qposadr[yaw_jid]
    yaw0 = data.qpos[yaw_qpos_idx]

    steps = int(duration / model.opt.timestep)
    for _ in range(steps):
        data.ctrl[CTRL_CHASSIS_VX] = vx
        data.ctrl[CTRL_CHASSIS_VY] = vy
        data.ctrl[CTRL_CHASSIS_OMEGA] = omega
        # 无 Z 轴 actuator — 底盘 Z 由 slide joint + 重力平衡维持
        mujoco.mj_step(model, data)

    pos1 = data.xpos[chassis_id].copy()
    yaw1 = data.qpos[yaw_qpos_idx]

    dx = pos1[0] - pos0[0]
    dy = pos1[1] - pos0[1]
    d_yaw = yaw1 - yaw0

    return dx, dy, d_yaw


def main():
    print(f"加载模型: {MJCF_PATH}")
    model = mujoco.MjModel.from_xml_path(MJCF_PATH)
    print(f"模型: nq={model.nq}, nv={model.nv}, nu={model.nu}")
    print(f"timestep={model.opt.timestep}, 测试时长={TEST_DURATION}s")
    print()

    n_pass = 0
    n_fail = 0
    n_total = len(TEST_CASES) + len(ROTATION_TESTS)

    # ===== 方向性移动测试 =====
    print("=" * 70)
    print("  方向性移动测试（8 个方向）")
    print("=" * 70)

    for name, vx, vy, omega, expected_deg in TEST_CASES:
        dx, dy, d_yaw = run_test(model, name, vx, vy, omega, TEST_DURATION)
        disp = math.hypot(dx, dy)
        actual_deg = math.degrees(math.atan2(dy, dx))
        dir_err = abs(actual_deg - expected_deg)
        if dir_err > 180:
            dir_err = 360 - dir_err

        ok_disp = disp >= MIN_DISPLACEMENT
        ok_dir = dir_err <= MAX_DIR_ERROR
        passed = ok_disp and ok_dir
        status = "PASS ✓" if passed else "FAIL ✗"

        if passed:
            n_pass += 1
        else:
            n_fail += 1

        print(f"  [{status}] {name:12s}  "
              f"cmd=({vx:+.2f},{vy:+.2f})  "
              f"位移={disp:.3f}m (dx={dx:+.3f}, dy={dy:+.3f})  "
              f"方向={actual_deg:+.1f}° (期望{expected_deg:+.1f}°, 误差{dir_err:.1f}°)")

    # ===== 旋转测试 =====
    print()
    print("=" * 70)
    print("  旋转测试")
    print("=" * 70)

    for name, vx, vy, omega in ROTATION_TESTS:
        dx, dy, d_yaw = run_test(model, name, vx, vy, omega, TEST_DURATION)
        rot_amount = abs(d_yaw)
        sign_ok = (omega > 0 and d_yaw > 0) or (omega < 0 and d_yaw < 0)
        passed = rot_amount >= MIN_ROTATION and sign_ok
        status = "PASS ✓" if passed else "FAIL ✗"

        if passed:
            n_pass += 1
        else:
            n_fail += 1

        print(f"  [{status}] {name:12s}  "
              f"cmd_ω={omega:+.2f}  "
              f"实际旋转={math.degrees(d_yaw):+.1f}° ({d_yaw:+.3f} rad)  "
              f"位移=(dx={dx:+.3f}, dy={dy:+.3f})")

    # ===== 汇总 =====
    print()
    print("=" * 70)
    print(f"  结果: {n_pass}/{n_total} 通过, {n_fail} 失败")
    if n_fail == 0:
        print("  ★ 所有测试通过！底盘全向运动正常。")
    else:
        print("  ✗ 存在失败测试，需要排查。")
    print("=" * 70)

    return 0 if n_fail == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
