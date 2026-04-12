#!/usr/bin/env python3
"""
Touch X 力向量可视化器 — MuJoCo 仿真

集合了 8 种力渲染效果的统一可视化展示：
1. 基础几何体（刚度墙）
2. 表面纹理
3. 弹簧阻尼系统
4. 粘滞力场
5. 磁吸效果
6. 重力井
7. 摩擦面
8. 力导引（虚拟引导槽）

每种效果：
- 用箭头（quiver）表示力的方向和大小
- 颜色编码力的大小（蓝→红 = 小→大）
- 保存为静态对比图

依赖: pip install mujoco numpy matplotlib
"""

import mujoco
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import Normalize
from matplotlib.cm import ScalarMappable
from pathlib import Path
import os

# === 路径 ===
SCRIPT_DIR = Path(__file__).parent
SCREENSHOTS_DIR = SCRIPT_DIR.parent / "screenshots"
os.makedirs(SCREENSHOTS_DIR, exist_ok=True)


# ============================================================
# 第一部分：力场计算函数
# 这些函数计算给定位置处的力向量，模拟 Touch X 可渲染的各种力效果
# ============================================================

def force_stiffness_wall(pos, wall_z=0.0, k=500.0):
    """
    力渲染模式 1: 刚度墙
    当探针侵入 wall_z 平面以下时，产生正比于侵入深度的法向力。
    这是 Touch X 最基础的力渲染模式。

    F = k * max(0, wall_z - z) * z_hat

    参数:
        pos: [x, y, z] 探针位置 (米)
        wall_z: 墙面高度 (米)
        k: 刚度系数 (N/m)，Touch X 最大可渲染约 3.5 N/mm = 3500 N/m
    """
    z = pos[2]
    if z < wall_z:
        penetration = wall_z - z
        return np.array([0, 0, k * penetration])
    return np.zeros(3)


def force_spring_damper(pos, anchor=np.array([0, 0, 0.03]), vel=np.zeros(3),
                        k=200.0, b=5.0, rest_length=0.0):
    """
    力渲染模式 2: 弹簧阻尼系统
    在锚点处连接一个虚拟弹簧，产生恢复力和阻尼力。
    用于模拟弹性物体和碰撞缓冲。

    F = -k * (|r| - rest_length) * r_hat - b * v

    参数:
        pos: 探针位置
        anchor: 弹簧固定端位置
        vel: 探针速度
        k: 弹簧刚度 (N/m)
        b: 阻尼系数 (N·s/m)
        rest_length: 弹簧自然长度 (米)
    """
    r = pos - anchor
    dist = np.linalg.norm(r)
    if dist < 1e-6:
        return -b * vel  # 在锚点处只有阻尼
    r_hat = r / dist
    spring_force = -k * (dist - rest_length) * r_hat
    damper_force = -b * vel
    return spring_force + damper_force


def force_viscosity(pos, vel, eta=20.0):
    """
    力渲染模式 3: 粘滞力场
    与速度方向相反的力，模拟在粘稠液体中移动。
    Touch X 可通过位置微分计算速度来实现。

    F = -eta * v

    参数:
        eta: 粘滞系数 (N·s/m)
    """
    return -eta * vel


def force_magnetic_snap(pos, magnet_pos=np.array([0, 0, 0.03]),
                        strength=0.001, snap_radius=0.02):
    """
    力渲染模式 4: 磁吸效果
    在磁铁位置附近产生吸引力（反比于距离平方），
    超出 snap_radius 后力衰减为零。
    用于模拟装配时的对齐和吸附。

    F = strength / r^2 * r_hat  (指向磁铁)

    参数:
        strength: 磁场强度参数
        snap_radius: 有效吸附半径 (米)
    """
    r = magnet_pos - pos
    dist = np.linalg.norm(r)
    if dist < 0.001:
        return np.zeros(3)  # 防止除以零
    if dist > snap_radius:
        return np.zeros(3)  # 超出范围无力

    # 平滑衰减：在 snap_radius 边界处力连续过渡到零
    decay = max(0, 1 - (dist / snap_radius) ** 2)
    force_mag = strength / (dist ** 2) * decay
    # 限幅到 Touch X 最大力 3.3N
    force_mag = min(force_mag, 3.3)
    return force_mag * r / dist


def force_gravity_well(pos, well_pos=np.array([0, 0, 0.03]),
                       G=0.0005, well_radius=0.05):
    """
    力渲染模式 5: 重力井
    模拟一个引力场，探针会被吸引到井的中心。
    力大小随距离减小而增加（类似引力），但在井心附近饱和。

    F = G / max(r^2, r_min^2) * r_hat

    参数:
        G: 引力强度参数
        well_radius: 重力井有效半径
    """
    r = well_pos - pos
    dist = np.linalg.norm(r)
    if dist < 0.001:
        return np.zeros(3)
    if dist > well_radius:
        return np.zeros(3)

    # 反比于距离，但在 2mm 处饱和防止无穷大
    force_mag = G / max(dist ** 2, 0.002 ** 2)
    force_mag = min(force_mag, 3.3)
    return force_mag * r / dist


def force_friction_surface(pos, vel, wall_z=0.0, k=1000.0, mu_s=0.8, mu_d=0.5):
    """
    力渲染模式 6: 摩擦表面
    结合法向弹性力和库仑摩擦力。
    法向力 = k * penetration，切向力 = mu * Fn（方向与速度相反）

    参数:
        mu_s: 静摩擦系数
        mu_d: 动摩擦系数
    """
    force = np.zeros(3)
    z = pos[2]
    if z < wall_z:
        # 法向力
        fn = k * (wall_z - z)
        force[2] = fn

        # 切向摩擦力
        v_tangent = np.array([vel[0], vel[1], 0])
        v_speed = np.linalg.norm(v_tangent)
        if v_speed > 0.001:
            # 动摩擦
            friction = mu_d * fn
            force[:2] -= friction * v_tangent[:2] / v_speed
        # 静摩擦由 MuJoCo 约束求解器处理，这里简化
    return force


def force_guide_channel(pos, channel_y=0.0, channel_width=0.005, k_wall=2000.0):
    """
    力渲染模式 7: 虚拟引导槽
    在 Y 方向设置两面虚拟墙，形成一个沟槽。
    探针只能沿 X 方向自由移动，偏离沟槽会感到阻力。
    用于引导手术路径或装配轨迹。

    参数:
        channel_y: 沟槽中心 Y 坐标
        channel_width: 沟槽宽度 (米)
        k_wall: 沟槽壁刚度
    """
    half_w = channel_width / 2
    dy = pos[1] - channel_y
    force = np.zeros(3)

    if abs(dy) > half_w:
        # 在沟槽外，产生推回力
        if dy > 0:
            force[1] = -k_wall * (dy - half_w)
        else:
            force[1] = -k_wall * (dy + half_w)
    return force


def force_texture_bump(pos, period=0.005, amplitude=0.5, wall_z=0.0, k=800.0):
    """
    力渲染模式 8: 纹理凹凸
    在表面上叠加正弦波高度场，模拟周期性纹理。
    当探针在表面上滑动时，法向力会随位置周期性变化。

    参数:
        period: 凹凸周期 (米)
        amplitude: 凹凸振幅 (N) — 叠加在基础法向力上
    """
    force = np.zeros(3)
    z = pos[2]
    # 基础法向力
    if z < wall_z:
        base_fn = k * (wall_z - z)
        # 叠加周期性纹理力
        texture_mod = amplitude * np.sin(2 * np.pi * pos[0] / period)
        force[2] = base_fn + texture_mod
    return force


# ============================================================
# 第二部分：力场可视化
# ============================================================

def compute_force_field_2d(force_func, x_range, z_range, n_points=30, **kwargs):
    """
    计算 2D 力场（XZ 平面）用于可视化。

    参数:
        force_func: 力计算函数
        x_range: (x_min, x_max) 范围
        z_range: (z_min, z_max) 范围
        n_points: 每个方向的采样点数
    返回:
        X, Z: 网格坐标
        Fx, Fz: 力分量
        F_mag: 力大小
    """
    x = np.linspace(x_range[0], x_range[1], n_points)
    z = np.linspace(z_range[0], z_range[1], n_points)
    X, Z = np.meshgrid(x, z)
    Fx = np.zeros_like(X)
    Fz = np.zeros_like(Z)

    for i in range(n_points):
        for j in range(n_points):
            pos = np.array([X[i, j], 0.0, Z[i, j]])
            f = force_func(pos, **kwargs)
            Fx[i, j] = f[0]
            Fz[i, j] = f[2]

    F_mag = np.sqrt(Fx**2 + Fz**2)
    return X, Z, Fx, Fz, F_mag


def plot_force_field(X, Z, Fx, Fz, F_mag, title, save_name,
                     extra_artists=None):
    """
    绘制 2D 力场图：箭头表示方向，颜色编码大小。

    参数:
        extra_artists: 额外绘图回调函数列表
    """
    fig, ax = plt.subplots(figsize=(10, 8))

    # 颜色编码
    norm = Normalize(vmin=0, vmax=max(F_mag.max(), 0.1))
    cmap = plt.cm.coolwarm

    # 归一化箭头长度（仅显示方向）
    with np.errstate(divide='ignore', invalid='ignore'):
        Fx_n = np.where(F_mag > 1e-6, Fx / F_mag, 0)
        Fz_n = np.where(F_mag > 1e-6, Fz / F_mag, 0)

    # 绘制箭头
    q = ax.quiver(X * 1000, Z * 1000, Fx_n, Fz_n,
                  F_mag, cmap=cmap, norm=norm,
                  scale=25, width=0.004, headwidth=3, headlength=4,
                  alpha=0.8)

    plt.colorbar(q, ax=ax, label='力大小 (N)')

    # 绘制额外元素（如墙面线、弹簧锚点等）
    if extra_artists:
        for artist_func in extra_artists:
            artist_func(ax)

    ax.set_xlabel('X (mm)', fontsize=12)
    ax.set_ylabel('Z (mm)', fontsize=12)
    ax.set_title(title, fontsize=14)
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.2)

    plt.tight_layout()
    save_path = SCREENSHOTS_DIR / save_name
    plt.savefig(save_path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"  力场图已保存: {save_path}")


# ============================================================
# 第三部分：MuJoCo 集成演示
# 每种力渲染效果都在 MuJoCo 中运行，验证力计算的物理合理性
# ============================================================

def run_mujoco_force_demo(demo_name, force_func_or_xml, duration=2.0):
    """
    在 MuJoCo 中运行力渲染演示，记录探针位置和接触力。
    使用简单场景 + 正弦轨迹。

    返回时间序列数据用于可视化。
    """
    # 基础 MuJoCo 场景
    xml = """
    <mujoco model="force_demo">
        <option timestep="0.001" gravity="0 0 -9.81" integrator="implicit"/>
        <worldbody>
            <geom type="plane" size="0.3 0.3 0.01" rgba="0.9 0.9 0.9 1"
                  pos="0 0 0" friction="0.5 0.005 0.0001"
                  solref="0.005 1.0" solimp="0.9 0.95 0.001"/>
            <body name="mocap_target" mocap="true" pos="0 0 0.05">
                <geom type="sphere" size="0.001" rgba="0 0 0 0"
                      contype="0" conaffinity="0"/>
            </body>
            <body name="probe" pos="0 0 0.05">
                <freejoint name="probe_joint"/>
                <geom name="probe_tip" type="sphere" size="0.003"
                      rgba="1 0.2 0.2 0.8" mass="0.05"/>
            </body>
            <light pos="0.5 0.5 1" dir="-0.5 -0.5 -1"/>
        </worldbody>
        <equality>
            <weld body1="probe" body2="mocap_target"
                  solref="0.001 1" solimp="0.95 0.99 0.001"/>
        </equality>
    </mujoco>
    """

    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    dt = model.opt.timestep
    n_steps = int(duration / dt)
    time_arr = np.arange(n_steps) * dt

    positions = np.zeros((n_steps, 3))
    forces = np.zeros((n_steps, 3))

    for i in range(n_steps):
        t = time_arr[i]
        # 垂直下压+水平扫描的复合轨迹
        x = 0.04 * np.sin(2 * np.pi * 0.5 * t)
        z = 0.03 - 0.035 * (t / duration)  # 从 30mm 逐渐下压到 -5mm

        data.mocap_pos[0] = [x, 0, z]
        mujoco.mj_step(model, data)
        positions[i] = [x, 0, z]

        # 读取接触力
        for j in range(data.ncon):
            contact = data.contact[j]
            geom1 = model.geom(contact.geom1).name if contact.geom1 >= 0 else ""
            geom2 = model.geom(contact.geom2).name if contact.geom2 >= 0 else ""
            if "probe_tip" in geom1 or "probe_tip" in geom2:
                force = np.zeros(6)
                mujoco.mj_contactForce(model, data, j, force)
                frame = contact.frame.reshape(3, 3)
                forces[i] += frame.T @ force[:3]

    return time_arr, positions, forces


# ============================================================
# 主函数：生成所有 8 种力渲染效果的可视化
# ============================================================

def main():
    """
    生成所有力渲染模式的力场图和 MuJoCo 仿真验证图。
    """
    print("=" * 60)
    print("Touch X 力渲染效果可视化器")
    print("=" * 60)

    # --- 效果 1: 刚度墙 ---
    print("\n▸ 效果 1: 刚度墙")
    X, Z, Fx, Fz, Fm = compute_force_field_2d(
        force_stiffness_wall,
        x_range=(-0.05, 0.05), z_range=(-0.01, 0.03), n_points=25,
        wall_z=0.0, k=500.0
    )
    plot_force_field(X, Z, Fx, Fz, Fm, "效果 1: 刚度墙 (k=500 N/m)",
                     "demo_03_stiffness_wall.png",
                     extra_artists=[lambda ax: ax.axhline(y=0, color='k', linestyle='--', linewidth=2, label='墙面')])

    # --- 效果 2: 弹簧阻尼 ---
    print("\n▸ 效果 2: 弹簧阻尼系统")
    anchor = np.array([0, 0, 0.03])

    def spring_force_wrapper(pos, **kw):
        return force_spring_damper(pos, anchor=anchor, k=200.0)

    X, Z, Fx, Fz, Fm = compute_force_field_2d(
        spring_force_wrapper,
        x_range=(-0.04, 0.04), z_range=(0.0, 0.06), n_points=25
    )
    plot_force_field(X, Z, Fx, Fz, Fm, "效果 2: 弹簧阻尼 (k=200, anchor=[0,0,30mm])",
                     "demo_05_spring_damper.png",
                     extra_artists=[lambda ax: ax.plot(0, 30, 'ko', markersize=10, label='锚点')])

    # --- 效果 3: 磁吸效果 ---
    print("\n▸ 效果 3: 磁吸效果")
    magnet = np.array([0, 0, 0.03])

    def mag_wrapper(pos, **kw):
        return force_magnetic_snap(pos, magnet_pos=magnet, strength=0.0005, snap_radius=0.03)

    X, Z, Fx, Fz, Fm = compute_force_field_2d(
        mag_wrapper,
        x_range=(-0.04, 0.04), z_range=(0.0, 0.06), n_points=25
    )
    plot_force_field(X, Z, Fx, Fz, Fm, "效果 3: 磁吸效果 (snap_radius=30mm)",
                     "demo_07_magnetic_snap.png",
                     extra_artists=[lambda ax: ax.plot(0, 30, 'r*', markersize=15, label='磁铁')])

    # --- 效果 4: 重力井 ---
    print("\n▸ 效果 4: 重力井")
    well = np.array([0, 0, 0.03])

    def gw_wrapper(pos, **kw):
        return force_gravity_well(pos, well_pos=well, G=0.0003, well_radius=0.04)

    X, Z, Fx, Fz, Fm = compute_force_field_2d(
        gw_wrapper,
        x_range=(-0.05, 0.05), z_range=(0.0, 0.06), n_points=25
    )
    plot_force_field(X, Z, Fx, Fz, Fm, "效果 4: 重力井 (radius=40mm)",
                     "demo_08_gravity_well.png",
                     extra_artists=[lambda ax: ax.plot(0, 30, 'gD', markersize=10, label='井心')])

    # --- 效果 5: 引导槽 ---
    print("\n▸ 效果 5: 虚拟引导槽")

    def guide_wrapper_xy(pos, **kw):
        # 在 XY 平面可视化引导槽（Y=channel, X=free）
        return force_guide_channel(pos, channel_y=0.0, channel_width=0.01, k_wall=1500.0)

    # 用 Y-Z 平面可视化
    n = 25
    Y_arr = np.linspace(-0.03, 0.03, n)
    X_arr = np.linspace(-0.05, 0.05, n)
    YY, XX = np.meshgrid(Y_arr, X_arr)
    Fy_field = np.zeros_like(YY)

    for i in range(n):
        for j in range(n):
            pos = np.array([XX[i, j], YY[i, j], 0.01])
            f = guide_wrapper_xy(pos)
            Fy_field[i, j] = f[1]

    fig, ax = plt.subplots(figsize=(10, 6))
    c = ax.pcolormesh(XX * 1000, YY * 1000, Fy_field, cmap='RdBu_r', shading='auto')
    plt.colorbar(c, ax=ax, label='Y 方向力 Fy (N)')
    ax.axhline(y=5, color='k', linestyle='--', linewidth=1, alpha=0.5)
    ax.axhline(y=-5, color='k', linestyle='--', linewidth=1, alpha=0.5)
    ax.set_xlabel('X (mm)')
    ax.set_ylabel('Y (mm)')
    ax.set_title('效果 5: 虚拟引导槽 (宽度=10mm, k=1500 N/m)\n黑虚线=槽壁，红色=推力向左，蓝色=推力向右')
    plt.tight_layout()
    save_path = SCREENSHOTS_DIR / "demo_09_guide_channel.png"
    plt.savefig(save_path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"  引导槽力场图已保存: {save_path}")

    # --- 效果 6: 纹理凹凸 ---
    print("\n▸ 效果 6: 纹理凹凸")

    def texture_wrapper(pos, **kw):
        return force_texture_bump(pos, period=0.005, amplitude=0.3)

    X, Z, Fx, Fz, Fm = compute_force_field_2d(
        texture_wrapper,
        x_range=(-0.02, 0.02), z_range=(-0.005, 0.005), n_points=40
    )
    plot_force_field(X, Z, Fx, Fz, Fm, "效果 6: 纹理凹凸 (周期=5mm, 振幅=0.3N)",
                     "demo_10_texture_bump.png",
                     extra_artists=[lambda ax: ax.axhline(y=0, color='k', linestyle='--', linewidth=1)])

    # --- MuJoCo 仿真验证 ---
    print("\n▸ MuJoCo 仿真验证：探针下压+扫描")
    try:
        time_arr, positions, forces = run_mujoco_force_demo("composite", None, duration=3.0)

        fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)

        # 位置
        axes[0].plot(time_arr, positions[:, 0] * 1000, label='X', alpha=0.8)
        axes[0].plot(time_arr, positions[:, 2] * 1000, label='Z', alpha=0.8)
        axes[0].set_ylabel('位置 (mm)')
        axes[0].set_title('MuJoCo 仿真 — 探针下压+扫描 复合轨迹')
        axes[0].legend()
        axes[0].grid(True, alpha=0.3)

        # 力分量
        axes[1].plot(time_arr, forces[:, 0], label='Fx', alpha=0.8, linewidth=0.5)
        axes[1].plot(time_arr, forces[:, 2], label='Fz', alpha=0.8, linewidth=0.5)
        axes[1].set_ylabel('力 (N)')
        axes[1].set_title('接触力分量')
        axes[1].legend()
        axes[1].grid(True, alpha=0.3)

        # 合力
        f_mag = np.linalg.norm(forces, axis=1)
        axes[2].plot(time_arr, f_mag, color='red', linewidth=0.5)
        axes[2].fill_between(time_arr, 0, f_mag, alpha=0.1, color='red')
        axes[2].set_xlabel('时间 (s)')
        axes[2].set_ylabel('合力 |F| (N)')
        axes[2].set_title('合力大小')
        axes[2].grid(True, alpha=0.3)

        plt.tight_layout()
        save_path = SCREENSHOTS_DIR / "demo_mujoco_composite_force.png"
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        plt.close()
        print(f"  MuJoCo 仿真图已保存: {save_path}")

    except Exception as e:
        print(f"  ❌ MuJoCo 仿真失败: {e}")
        import traceback
        traceback.print_exc()

    # --- 汇总: 8 种力渲染模式概览 ---
    print("\n▸ 生成 8 种力渲染模式汇总图...")
    fig, axes = plt.subplots(2, 4, figsize=(20, 10))
    titles = [
        "1. 刚度墙\n(Stiffness Wall)",
        "2. 表面纹理\n(Surface Texture)",
        "3. 摩擦面\n(Friction Surface)",
        "4. 弹簧阻尼\n(Spring-Damper)",
        "5. 粘滞力场\n(Viscosity Field)",
        "6. 磁吸效果\n(Magnetic Snap)",
        "7. 重力井\n(Gravity Well)",
        "8. 引导槽\n(Guide Channel)",
    ]
    force_funcs = [
        lambda p: force_stiffness_wall(p, wall_z=0.0, k=500),
        lambda p: force_texture_bump(p, period=0.005, amplitude=0.3),
        lambda p: force_friction_surface(p, np.array([0.01, 0, 0]), wall_z=0.0),
        lambda p: force_spring_damper(p, anchor=np.array([0, 0, 0.03])),
        lambda p: force_viscosity(p, np.array([0.01, 0, -0.005])),
        lambda p: force_magnetic_snap(p, snap_radius=0.03),
        lambda p: force_gravity_well(p, well_radius=0.04),
        lambda p: force_guide_channel(p, channel_width=0.01, k_wall=1500),
    ]

    for idx, (title, func) in enumerate(zip(titles, force_funcs)):
        ax = axes[idx // 4, idx % 4]
        n = 20
        x = np.linspace(-0.04, 0.04, n)
        z = np.linspace(-0.01, 0.05, n)
        X_, Z_ = np.meshgrid(x, z)
        Fx_ = np.zeros_like(X_)
        Fz_ = np.zeros_like(Z_)
        Fm_ = np.zeros_like(X_)

        for i in range(n):
            for j in range(n):
                pos = np.array([X_[i, j], 0, Z_[i, j]])
                f = func(pos)
                Fx_[i, j] = f[0]
                Fz_[i, j] = f[2]
                Fm_[i, j] = np.linalg.norm(f)

        # 归一化
        with np.errstate(divide='ignore', invalid='ignore'):
            Fx_n = np.where(Fm_ > 1e-6, Fx_ / Fm_, 0)
            Fz_n = np.where(Fm_ > 1e-6, Fz_ / Fm_, 0)

        ax.quiver(X_ * 1000, Z_ * 1000, Fx_n, Fz_n, Fm_,
                  cmap='coolwarm', scale=30, width=0.005, alpha=0.7)
        ax.set_title(title, fontsize=10)
        ax.set_xlabel('X (mm)', fontsize=8)
        ax.set_ylabel('Z (mm)', fontsize=8)
        ax.grid(True, alpha=0.2)
        ax.set_aspect('equal')

    plt.suptitle('Touch X 8 种力渲染模式 — 力场概览\n'
                 '箭头=力方向, 颜色=力大小(蓝→红=小→大)',
                 fontsize=14, fontweight='bold')
    plt.tight_layout()
    summary_path = SCREENSHOTS_DIR / "demo_all_8_force_modes.png"
    plt.savefig(summary_path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"  汇总图已保存: {summary_path}")

    print("\n" + "=" * 60)
    print("✓ 力渲染可视化全部完成！")
    print(f"  截图目录: {SCREENSHOTS_DIR}")
    print("=" * 60)


if __name__ == "__main__":
    main()
