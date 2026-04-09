#!/usr/bin/env python3
"""
Touch 力反馈笔 — 8 种力渲染交互式 Demo

直接驱动 Touch 设备，让用户亲身感受每种力渲染效果。
按数字键 1-8 切换效果，按 Q 退出。

效果列表:
  1. 刚度墙    — 在 Y=0 平面下感受弹性阻力
  2. 不同材质  — 按按钮切换硬/软/弹性表面
  3. 摩擦面    — 水平移动感受摩擦阻力
  4. 弹簧锚点  — 被弹簧拉向空间中心
  5. 粘滞力场  — 如同在蜂蜜中移动
  6. 磁吸效果  — 靠近目标点被吸附
  7. 重力井    — 被引力拉向空间中心
  8. 虚拟引导槽 — 只能沿一条线自由移动

依赖: touch_hd_driver.py 在同目录下
用法: python3 20260409-cc-force_rendering_demos.py
"""

import sys
import os
import time
import threading
import numpy as np

# 确保能导入同目录的驱动
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from touch_hd_driver import TouchDevice

# ============================================================
# 8 种力渲染效果函数
# 每个函数接收 (position_mm, velocity_mm_s) 返回 [Fx, Fy, Fz] (N)
# ============================================================

def effect_stiffness_wall(pos, vel, params=None):
    """
    效果 1: 刚度墙
    在 Y < 0 时产生向上的弹性恢复力。
    就像用笔尖去戳一个桌面——越往下压越硬。

    OpenHaptics 坐标系: Y 向上，所以 Y<0 = 笔尖在"桌面"以下
    """
    force = np.zeros(3)
    wall_y = 0.0    # 墙面在 Y=0
    k = 0.5         # 刚度 0.5 N/mm — Touch 安全范围内

    if pos[1] < wall_y:
        penetration = wall_y - pos[1]  # mm
        force[1] = k * penetration     # 向上推
        # 限幅
        force[1] = min(force[1], 2.5)
    return force


def effect_variable_stiffness(pos, vel, params=None):
    """
    效果 2: 可变刚度（软/中/硬）
    同样是 Y<0 时的墙面，但按 Touch 前按钮循环切换刚度：
    - 软（海绵）: k=0.05 N/mm
    - 中（木材）: k=0.3 N/mm
    - 硬（金属）: k=0.8 N/mm

    params['stiffness_level'] 由主循环通过按钮控制
    """
    stiffness_levels = [
        (0.05, "海绵"),
        (0.3, "木材"),
        (0.8, "金属"),
    ]
    level = params.get('stiffness_level', 0) if params else 0
    k, name = stiffness_levels[level % 3]

    force = np.zeros(3)
    if pos[1] < 0:
        penetration = -pos[1]
        force[1] = k * penetration
        force[1] = min(force[1], 2.5)
    return force


def effect_friction_surface(pos, vel, params=None):
    """
    效果 3: 摩擦表面
    在 Y<0 时：法向弹性力 + 水平摩擦力（与滑动速度方向相反）。
    感觉像在桌面上拖动笔尖。
    """
    force = np.zeros(3)
    k = 0.4       # 法向刚度
    mu = 0.3      # 摩擦系数

    if pos[1] < 0:
        # 法向力（Y 方向）
        fn = k * (-pos[1])
        fn = min(fn, 2.0)
        force[1] = fn

        # 切向摩擦力（XZ 平面，与速度反向）
        v_tangent = np.array([vel[0], 0, vel[2]])
        v_speed = np.linalg.norm(v_tangent)
        if v_speed > 5.0:  # 超过 5mm/s 才产生动摩擦
            friction_mag = mu * fn
            force[0] -= friction_mag * vel[0] / v_speed
            force[2] -= friction_mag * vel[2] / v_speed
    return force


def effect_spring_anchor(pos, vel, params=None):
    """
    效果 4: 弹簧锚点
    在空间中心 (0, 60, 0) 放一个虚拟弹簧。
    笔尖离中心越远，拉力越大——像被橡皮筋拽住。
    """
    anchor = np.array([0.0, 60.0, 0.0])  # mm, 设备工作空间中心偏上
    k = 0.08       # N/mm — 柔软弹簧
    b = 0.002      # N·s/mm — 轻微阻尼防振荡

    r = anchor - pos  # 指向锚点
    dist = np.linalg.norm(r)

    force = np.zeros(3)
    if dist > 1.0:  # 超过 1mm 才有力
        force = k * r - b * vel
        # 限幅
        f_mag = np.linalg.norm(force)
        if f_mag > 2.5:
            force = force / f_mag * 2.5
    return force


def effect_viscosity(pos, vel, params=None):
    """
    效果 5: 粘滞力场
    与速度方向相反的阻力——像在蜂蜜中移动。
    移动越快阻力越大，静止时无力。
    """
    eta = 0.008  # 粘滞系数 N·s/mm — 调到明显但不过大
    force = -eta * vel

    # 限幅
    f_mag = np.linalg.norm(force)
    if f_mag > 2.5:
        force = force / f_mag * 2.5
    return force


def effect_magnetic_snap(pos, vel, params=None):
    """
    效果 6: 磁吸效果
    在 (0, 40, 0) 处放一个磁铁。
    靠近时被吸引，接触后被锁定。模拟装配对齐。
    """
    magnet = np.array([0.0, 40.0, 0.0])  # mm
    snap_radius = 40.0  # mm — 吸附范围
    strength = 150.0    # 力的强度参数

    r = magnet - pos
    dist = np.linalg.norm(r)

    force = np.zeros(3)
    if 2.0 < dist < snap_radius:
        # 吸引力：反比于距离平方，越近越强
        f_mag = strength / (dist ** 2)
        # 边缘衰减
        edge_factor = 1.0 - (dist / snap_radius) ** 2
        f_mag *= max(edge_factor, 0)
        f_mag = min(f_mag, 2.0)
        force = f_mag * r / dist
    elif dist <= 2.0:
        # 极近时用弹簧力锁定
        force = 0.5 * r
    return force


def effect_gravity_well(pos, vel, params=None):
    """
    效果 7: 重力井
    空间中心有一个引力源，把笔尖缓慢拉过去。
    比弹簧更"自然"——远处微弱，近处强烈。
    """
    well = np.array([0.0, 50.0, 0.0])
    G = 500.0           # 引力强度
    well_radius = 80.0  # mm — 有效范围

    r = well - pos
    dist = np.linalg.norm(r)

    force = np.zeros(3)
    if 3.0 < dist < well_radius:
        f_mag = G / max(dist ** 2, 100.0)  # 在 10mm 处饱和
        f_mag = min(f_mag, 1.5)
        force = f_mag * r / dist
    elif dist <= 3.0:
        # 极近处用弹簧锁定
        force = 0.3 * r
    return force


def effect_guide_channel(pos, vel, params=None):
    """
    效果 8: 虚拟引导槽
    只允许沿 X 轴自由移动。偏离 X 轴（Z 或 Y 方向偏移）会被推回。
    模拟手术路径引导或装配轨迹约束。
    """
    # 沟槽中心线: Y=50, Z=0（沿 X 方向）
    channel_y = 50.0  # mm
    channel_z = 0.0   # mm
    k = 0.3           # N/mm — 沟槽壁刚度

    force = np.zeros(3)
    dy = pos[1] - channel_y
    dz = pos[2] - channel_z

    # Y 方向约束
    if abs(dy) > 3.0:  # 3mm 沟槽半宽
        force[1] = -k * (dy - np.sign(dy) * 3.0)
    # Z 方向约束
    if abs(dz) > 3.0:
        force[2] = -k * (dz - np.sign(dz) * 3.0)

    # 限幅
    f_mag = np.linalg.norm(force)
    if f_mag > 2.5:
        force = force / f_mag * 2.5
    return force


# ============================================================
# 效果注册表
# ============================================================
EFFECTS = [
    ("刚度墙 (Stiffness Wall)", "笔尖向下压 → 感受弹性阻力", effect_stiffness_wall),
    ("可变刚度 (Variable Stiffness)", "向下压 + 按前按钮切换软/中/硬", effect_variable_stiffness),
    ("摩擦面 (Friction Surface)", "向下压后水平滑动 → 感受摩擦", effect_friction_surface),
    ("弹簧锚点 (Spring Anchor)", "被拉向空间中心 → 橡皮筋感", effect_spring_anchor),
    ("粘滞力场 (Viscosity)", "快速移动 → 蜂蜜中拖拽感", effect_viscosity),
    ("磁吸效果 (Magnetic Snap)", "靠近中心 → 被吸附锁定", effect_magnetic_snap),
    ("重力井 (Gravity Well)", "缓慢被拉向中心 → 引力感", effect_gravity_well),
    ("引导槽 (Guide Channel)", "只能沿 X 轴移动 → 路径约束", effect_guide_channel),
]


# ============================================================
# 主程序
# ============================================================

def main():
    print("=" * 60)
    print("  Touch 力反馈笔 — 8 种力渲染交互式 Demo")
    print("=" * 60)
    print()
    print("效果列表:")
    for i, (name, hint, _) in enumerate(EFFECTS):
        print(f"  {i+1}. {name}")
        print(f"     → {hint}")
    print()
    print("操作:")
    print("  数字 1-8 : 切换效果")
    print("  0        : 关闭力输出（自由移动）")
    print("  q        : 退出")
    print()

    # 初始化设备
    dev = TouchDevice()
    dev.start()

    current_effect = 0  # 默认效果 1
    params = {'stiffness_level': 0}
    last_button1 = False
    force_enabled = True

    # 输入线程：监听键盘
    input_queue = []
    def input_thread():
        while True:
            try:
                ch = input()
                input_queue.append(ch.strip())
            except EOFError:
                break

    t = threading.Thread(target=input_thread, daemon=True)
    t.start()

    print(f"\n>>> 当前效果: {EFFECTS[current_effect][0]}")
    print(f"    提示: {EFFECTS[current_effect][1]}")
    print()

    try:
        while True:
            # 读取设备状态
            pos = dev.position
            vel = dev.velocity
            btn = dev.buttons

            # 检查按钮（用于效果 2 切换刚度）
            button1_pressed = bool(btn & 0x01)
            if button1_pressed and not last_button1:
                params['stiffness_level'] = params.get('stiffness_level', 0) + 1
                levels = ["海绵(0.05)", "木材(0.3)", "金属(0.8)"]
                level_name = levels[params['stiffness_level'] % 3]
                if current_effect == 1:
                    print(f"\r  → 切换刚度: {level_name}                    ")
            last_button1 = button1_pressed

            # 计算力
            if force_enabled:
                _, _, effect_func = EFFECTS[current_effect]
                force = effect_func(pos, vel, params)
            else:
                force = np.zeros(3)

            dev.set_force(force)

            # 打印状态
            f_mag = np.linalg.norm(force)
            print(f"\r  位置:[{pos[0]:6.1f},{pos[1]:6.1f},{pos[2]:6.1f}]mm  "
                  f"力:[{force[0]:5.2f},{force[1]:5.2f},{force[2]:5.2f}]N  "
                  f"|F|={f_mag:.2f}N  ",
                  end="", flush=True)

            # 处理键盘输入
            while input_queue:
                cmd = input_queue.pop(0)
                if cmd == 'q':
                    raise KeyboardInterrupt
                elif cmd == '0':
                    force_enabled = False
                    print(f"\n>>> 力输出已关闭（自由移动）")
                elif cmd in '12345678':
                    idx = int(cmd) - 1
                    current_effect = idx
                    force_enabled = True
                    params['stiffness_level'] = 0
                    print(f"\n>>> 切换到效果 {cmd}: {EFFECTS[idx][0]}")
                    print(f"    提示: {EFFECTS[idx][1]}")

            time.sleep(0.02)  # 50Hz 主循环（力输出在 1kHz 伺服中）

    except KeyboardInterrupt:
        print("\n\n退出中...")
    finally:
        dev.set_force([0, 0, 0])  # 清零力输出
        time.sleep(0.1)
        dev.stop()
        print("已安全退出。")


if __name__ == "__main__":
    main()
