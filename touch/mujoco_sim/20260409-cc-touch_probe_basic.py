#!/usr/bin/env python3
"""
Touch X 基础几何体碰撞力测试 — MuJoCo 仿真

在 MuJoCo 中创建多种基础几何体场景，用球形探针（模拟 Touch 笔尖）
与之交互，记录并可视化接触力。

测试的几何体：
1. 无限大水平面 — 法向力基准测试
2. 球体（直径 5cm）— 曲面法向力
3. 立方体（5cm）— 棱边检测
4. 圆柱体 — 柱面滑动
5. 锥体（近似）— 尖端接触

每个几何体输出：
- 接触力向量 [Fx, Fy, Fz] 随时间的变化曲线
- 接触点位置
- 法向力 vs 侵入深度关系图（刚度曲线）

依赖: pip install mujoco numpy matplotlib
"""

import mujoco
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from pathlib import Path
import os

# === 全局路径 ===
SCRIPT_DIR = Path(__file__).parent
SCREENSHOTS_DIR = SCRIPT_DIR.parent / "screenshots"
os.makedirs(SCREENSHOTS_DIR, exist_ok=True)


# === MuJoCo 场景 XML 定义 ===
# 每个场景都包含一个可控球形探针和一个目标几何体

def get_scene_xml(geom_type="plane", geom_params=""):
    """
    生成 MuJoCo XML 场景描述。
    包含：
    - 一个球形探针（半径 3mm，模拟 Touch 笔尖的 SCP 代理点）
    - 一个目标几何体
    - 接触对和力传感器

    参数:
        geom_type: 目标几何体类型 (plane/sphere/box/cylinder/cone)
        geom_params: 额外的几何体参数字符串
    """
    # 根据几何体类型设置默认参数
    geom_defs = {
        "plane": '<geom name="target" type="plane" size="0.5 0.5 0.01" rgba="0.7 0.8 0.9 1" pos="0 0 0"/>',
        "sphere": '<geom name="target" type="sphere" size="0.025" rgba="0.9 0.5 0.3 1" pos="0 0 0.025"/>',
        "box": '<geom name="target" type="box" size="0.025 0.025 0.025" rgba="0.3 0.7 0.5 1" pos="0 0 0.025"/>',
        "cylinder": '<geom name="target" type="cylinder" size="0.02 0.03" rgba="0.5 0.3 0.8 1" pos="0 0 0.03"/>',
        # 锥体用多个堆叠的圆柱近似（MuJoCo 无原生锥体）
        "cone": """
            <geom name="target" type="cylinder" size="0.025 0.005" rgba="0.8 0.6 0.2 1" pos="0 0 0.005"/>
            <geom name="cone_mid" type="cylinder" size="0.018 0.005" rgba="0.8 0.6 0.2 1" pos="0 0 0.015"/>
            <geom name="cone_top" type="cylinder" size="0.010 0.005" rgba="0.8 0.6 0.2 1" pos="0 0 0.025"/>
            <geom name="cone_tip" type="sphere" size="0.004" rgba="0.8 0.6 0.2 1" pos="0 0 0.032"/>
        """,
    }

    geom_xml = geom_defs.get(geom_type, geom_defs["plane"])

    xml = f"""
    <mujoco model="touch_probe_{geom_type}">
        <option timestep="0.001" gravity="0 0 -9.81" integrator="implicit">
            <flag contact="enable"/>
        </option>

        <default>
            <!-- 默认接触参数：中等刚度，适合力渲染测试 -->
            <geom condim="3" friction="0.5 0.005 0.0001"
                  solref="0.005 1.0" solimp="0.9 0.95 0.001"/>
        </default>

        <worldbody>
            <!-- 目标几何体（静态，固定在世界坐标系） -->
            <body name="target_body" pos="0 0 0">
                {geom_xml}
            </body>

            <!-- 球形探针（模拟 Touch 笔尖） -->
            <!-- 使用 mocap body 来精确控制位置 -->
            <body name="probe" mocap="true" pos="0 0 0.1">
                <geom name="probe_tip" type="sphere" size="0.003"
                      rgba="1 0.2 0.2 0.8" mass="0.01"
                      condim="3" friction="0.5 0.005 0.0001"
                      solref="0.002 1.0" solimp="0.95 0.99 0.001"/>
            </body>

            <!-- 光源 -->
            <light pos="0.5 0.5 1" dir="-0.5 -0.5 -1" diffuse="0.8 0.8 0.8"/>
        </worldbody>

        <!-- 力传感器 -->
        <sensor>
            <touch name="probe_force" site="probe_site"/>
        </sensor>

        <!-- 可视化设置 -->
        <visual>
            <global offwidth="640" offheight="480"/>
        </visual>
    </mujoco>
    """

    # 关键：mocap body 不产生碰撞！
    # 正确方案：用 weld equality 把普通 body 的 freejoint 焊接到 mocap body
    # 这样探针既能被 mocap 控制位置，又能产生物理碰撞
    xml_simple = f"""
    <mujoco model="touch_probe_{geom_type}">
        <option timestep="0.001" gravity="0 0 -9.81" integrator="implicit">
            <flag contact="enable"/>
        </option>

        <default>
            <geom condim="3" friction="0.5 0.005 0.0001"
                  solref="0.005 1.0" solimp="0.9 0.95 0.001"/>
        </default>

        <worldbody>
            <!-- 地面（辅助视觉参考） -->
            <geom type="plane" size="0.3 0.3 0.01" rgba="0.95 0.95 0.95 1"
                  pos="0 0 -0.001" contype="0" conaffinity="0"/>

            <!-- 目标几何体 -->
            <body name="target_body" pos="0 0 0">
                {geom_xml}
            </body>

            <!-- mocap 目标体（不可见，不碰撞，仅用于控制） -->
            <body name="mocap_target" mocap="true" pos="0 0 0.1">
                <geom type="sphere" size="0.001" rgba="0 0 0 0"
                      contype="0" conaffinity="0"/>
            </body>

            <!-- 探针：普通 body + freejoint，可以产生碰撞 -->
            <body name="probe" pos="0 0 0.1">
                <freejoint name="probe_joint"/>
                <geom name="probe_tip" type="sphere" size="0.003"
                      rgba="1 0.2 0.2 0.8" mass="0.05"/>
            </body>

            <light pos="0.5 0.5 1" dir="-0.5 -0.5 -1" diffuse="0.8 0.8 0.8"/>
        </worldbody>

        <!-- 用 weld 把探针焊接到 mocap body，实现位置跟随+碰撞 -->
        <equality>
            <weld body1="probe" body2="mocap_target"
                  solref="0.001 1" solimp="0.95 0.99 0.001"/>
        </equality>

        <visual>
            <global offwidth="800" offheight="600"/>
        </visual>
    </mujoco>
    """
    return xml_simple


def run_probe_test(geom_type, trajectory_func, duration=2.0, desc=""):
    """
    运行单个探针测试：
    1. 加载场景
    2. 沿给定轨迹移动探针
    3. 记录接触力和接触点
    4. 返回时间序列数据

    参数:
        geom_type: 目标几何体类型
        trajectory_func: 轨迹生成函数，接收 (time_array, model, data) 返回 (x, y, z)
        duration: 仿真持续时间 (秒)
        desc: 测试描述
    返回:
        dict: 包含 time, force, position, contact_pos, penetration 数组
    """
    print(f"\n{'='*50}")
    print(f"测试: {desc} (几何体: {geom_type})")
    print(f"{'='*50}")

    # 加载 MuJoCo 模型
    xml = get_scene_xml(geom_type)
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    # 仿真参数
    dt = model.opt.timestep  # 0.001s = 1kHz
    n_steps = int(duration / dt)
    time_array = np.arange(n_steps) * dt

    # 数据记录数组
    forces = np.zeros((n_steps, 3))       # 接触力 [Fx, Fy, Fz]
    positions = np.zeros((n_steps, 3))     # 探针位置
    contact_points = np.zeros((n_steps, 3))  # 接触点位置
    penetrations = np.zeros(n_steps)        # 侵入深度
    contact_flags = np.zeros(n_steps, dtype=bool)  # 是否有接触

    # 仿真主循环
    for i in range(n_steps):
        t = time_array[i]

        # 获取轨迹位置并设置 mocap body
        pos = trajectory_func(t, model, data)
        data.mocap_pos[0] = pos
        positions[i] = pos

        # 前进一步
        mujoco.mj_step(model, data)

        # 读取接触信息
        total_force = np.zeros(3)
        total_contact_pos = np.zeros(3)
        n_contacts = 0

        for j in range(data.ncon):
            contact = data.contact[j]
            # 检查是否涉及探针
            geom1 = model.geom(contact.geom1).name if contact.geom1 >= 0 else ""
            geom2 = model.geom(contact.geom2).name if contact.geom2 >= 0 else ""

            if "probe_tip" in geom1 or "probe_tip" in geom2:
                # 获取接触力（6维：法向力 + 摩擦力）
                force = np.zeros(6)
                mujoco.mj_contactForce(model, data, j, force)

                # 将接触力从接触坐标系转换到世界坐标系
                # contact.frame 是 3x3 旋转矩阵（展平为 9 元素数组）
                frame = contact.frame.reshape(3, 3)
                # force[:3] = [法向力, 切向力1, 切向力2] 在接触坐标系中
                world_force = frame.T @ force[:3]

                total_force += world_force
                total_contact_pos += contact.pos
                penetrations[i] = max(penetrations[i], -contact.dist)
                n_contacts += 1

        if n_contacts > 0:
            forces[i] = total_force
            contact_points[i] = total_contact_pos / n_contacts
            contact_flags[i] = True

    # 统计
    contact_ratio = contact_flags.sum() / n_steps * 100
    max_force = np.linalg.norm(forces, axis=1).max()
    print(f"  接触率: {contact_ratio:.1f}%")
    print(f"  最大力: {max_force:.4f} N")
    print(f"  最大侵入深度: {penetrations.max()*1000:.3f} mm")

    return {
        "time": time_array,
        "force": forces,
        "position": positions,
        "contact_pos": contact_points,
        "penetration": penetrations,
        "contact_flag": contact_flags,
        "geom_type": geom_type,
        "desc": desc,
    }


# === 轨迹生成函数 ===

def traj_vertical_press(z_start=0.06, z_end=-0.005, duration=2.0):
    """垂直下压轨迹：从高处匀速下压到表面以下"""
    def func(t, model, data):
        progress = t / duration
        z = z_start + (z_end - z_start) * progress
        return np.array([0.0, 0.0, z])
    return func


def traj_horizontal_sweep(z_level, x_range=(-0.06, 0.06), duration=2.0):
    """水平扫描轨迹：在固定高度从左到右扫过"""
    def func(t, model, data):
        progress = t / duration
        x = x_range[0] + (x_range[1] - x_range[0]) * progress
        return np.array([x, 0.0, z_level])
    return func


def traj_circular_descent(z_start=0.06, z_end=0.02, radius=0.02, duration=3.0):
    """螺旋下降轨迹：画圆同时下降，用于球体/圆柱体测试"""
    def func(t, model, data):
        progress = t / duration
        z = z_start + (z_end - z_start) * progress
        angle = 2 * np.pi * 2 * progress  # 2 圈
        x = radius * np.cos(angle)
        y = radius * np.sin(angle)
        return np.array([x, y, z])
    return func


def traj_edge_scan(z_level, box_half=0.025, y_offset=0.0, duration=2.0):
    """边缘扫描：在立方体顶面边缘处水平扫过"""
    def func(t, model, data):
        progress = t / duration
        x = -0.06 + 0.12 * progress  # 从左到右扫过
        return np.array([x, y_offset, z_level])
    return func


# === 可视化函数 ===

def plot_force_timeseries(result, save_name):
    """
    绘制力-时间曲线图（三分量 + 合力）

    参数:
        result: run_probe_test 返回的字典
        save_name: 保存文件名
    """
    t = result["time"]
    f = result["force"]
    f_mag = np.linalg.norm(f, axis=1)

    fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True)

    # 上图：三个分量
    axes[0].plot(t, f[:, 0], label='Fx', alpha=0.8, linewidth=0.8)
    axes[0].plot(t, f[:, 1], label='Fy', alpha=0.8, linewidth=0.8)
    axes[0].plot(t, f[:, 2], label='Fz', alpha=0.8, linewidth=0.8)
    axes[0].set_ylabel('力 (N)')
    axes[0].set_title(f'{result["desc"]} — 接触力分量')
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)

    # 下图：合力大小
    axes[1].plot(t, f_mag, color='red', linewidth=0.8)
    axes[1].fill_between(t, 0, f_mag, alpha=0.15, color='red')
    axes[1].set_xlabel('时间 (s)')
    axes[1].set_ylabel('合力 |F| (N)')
    axes[1].set_title(f'{result["desc"]} — 合力')
    axes[1].grid(True, alpha=0.3)

    plt.tight_layout()
    save_path = SCREENSHOTS_DIR / save_name
    plt.savefig(save_path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"  力-时间图已保存: {save_path}")


def plot_stiffness_curve(result, save_name):
    """
    绘制刚度曲线：法向力 vs 侵入深度
    斜率即为渲染刚度 (N/mm)

    参数:
        result: run_probe_test 返回的字典
        save_name: 保存文件名
    """
    pen = result["penetration"] * 1000  # 转为 mm
    f_z = np.abs(result["force"][:, 2])  # 法向力（取绝对值）
    contact = result["contact_flag"]

    fig, ax = plt.subplots(figsize=(8, 6))

    # 只画有接触的点
    if contact.any():
        ax.scatter(pen[contact], f_z[contact], s=1, alpha=0.3, color='steelblue')

        # 线性拟合计算刚度
        if pen[contact].max() > 0:
            mask = contact & (pen > 0.01)  # 排除极小侵入
            if mask.any():
                coeffs = np.polyfit(pen[mask], f_z[mask], 1)
                stiffness = coeffs[0]
                fit_x = np.linspace(0, pen[contact].max(), 100)
                ax.plot(fit_x, np.polyval(coeffs, fit_x), 'r--',
                        label=f'拟合刚度: {stiffness:.2f} N/mm', linewidth=2)
                ax.legend(fontsize=12)

    ax.set_xlabel('侵入深度 (mm)', fontsize=12)
    ax.set_ylabel('法向力 |Fz| (N)', fontsize=12)
    ax.set_title(f'{result["desc"]} — 刚度曲线', fontsize=14)
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    save_path = SCREENSHOTS_DIR / save_name
    plt.savefig(save_path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"  刚度曲线已保存: {save_path}")


def plot_contact_position(result, save_name):
    """
    绘制接触点 3D 位置图

    参数:
        result: run_probe_test 返回的字典
        save_name: 保存文件名
    """
    pos = result["position"]
    cpos = result["contact_pos"]
    contact = result["contact_flag"]
    f_mag = np.linalg.norm(result["force"], axis=1)

    fig, ax = plt.subplots(figsize=(10, 8))

    # 探针路径 XZ 侧视图（灰色）
    ax.plot(pos[:, 0] * 1000, pos[:, 2] * 1000,
            color='gray', alpha=0.3, linewidth=0.5, label='探针路径')

    # 接触点（颜色编码力大小）— XZ 侧视图
    if contact.any():
        sc = ax.scatter(cpos[contact, 0] * 1000,
                        cpos[contact, 2] * 1000,
                        c=f_mag[contact], cmap='hot', s=5, alpha=0.6)
        plt.colorbar(sc, ax=ax, label='接触力 (N)')

    ax.set_xlabel('X (mm)')
    ax.set_ylabel('Z (mm)')
    ax.set_title(f'{result["desc"]} — 接触点分布 (XZ 侧视图)')
    ax.legend()
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    save_path = SCREENSHOTS_DIR / save_name
    plt.savefig(save_path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"  接触点图已保存: {save_path}")


# === 主函数 ===

def main():
    """
    运行所有 5 个基础几何体测试，生成力-时间曲线、刚度曲线和接触点分布图。
    """
    print("=" * 60)
    print("Touch X 基础几何体碰撞力测试 — MuJoCo 仿真")
    print("=" * 60)

    # 测试配置：几何体类型、轨迹函数、描述
    tests = [
        {
            "geom": "plane",
            "traj": traj_vertical_press(z_start=0.04, z_end=-0.003, duration=2.0),
            "duration": 2.0,
            "desc": "平面 — 垂直下压",
            "prefix": "01_plane"
        },
        {
            "geom": "sphere",
            "traj": traj_vertical_press(z_start=0.08, z_end=0.02, duration=2.0),
            "duration": 2.0,
            "desc": "球体 — 垂直下压",
            "prefix": "02_sphere"
        },
        {
            "geom": "box",
            "traj": traj_horizontal_sweep(z_level=0.052, duration=2.0),
            "duration": 2.0,
            "desc": "立方体 — 水平扫描",
            "prefix": "03_box_sweep"
        },
        {
            "geom": "box",
            "traj": traj_vertical_press(z_start=0.08, z_end=0.04, duration=2.0),
            "duration": 2.0,
            "desc": "立方体 — 垂直下压",
            "prefix": "03_box_press"
        },
        {
            "geom": "cylinder",
            "traj": traj_horizontal_sweep(z_level=0.062, duration=2.0),
            "duration": 2.0,
            "desc": "圆柱体 — 水平扫描",
            "prefix": "04_cylinder"
        },
        {
            "geom": "cone",
            "traj": traj_vertical_press(z_start=0.06, z_end=0.025, duration=2.0),
            "duration": 2.0,
            "desc": "锥体 — 垂直下压",
            "prefix": "05_cone"
        },
    ]

    results = []
    for test in tests:
        try:
            result = run_probe_test(
                geom_type=test["geom"],
                trajectory_func=test["traj"],
                duration=test["duration"],
                desc=test["desc"],
            )
            results.append(result)

            # 生成三张图
            plot_force_timeseries(result, f"demo_{test['prefix']}_force.png")
            plot_stiffness_curve(result, f"demo_{test['prefix']}_stiffness.png")
            plot_contact_position(result, f"demo_{test['prefix']}_contact.png")

        except Exception as e:
            print(f"  ❌ 测试失败: {e}")
            import traceback
            traceback.print_exc()

    # 生成对比汇总图：所有几何体的刚度曲线对比
    print("\n" + "=" * 50)
    print("生成汇总对比图...")
    fig, ax = plt.subplots(figsize=(10, 7))
    colors = plt.cm.Set1(np.linspace(0, 1, len(results)))

    for result, color in zip(results, colors):
        pen = result["penetration"] * 1000
        f_z = np.abs(result["force"][:, 2])
        contact = result["contact_flag"]
        if contact.any():
            ax.scatter(pen[contact], f_z[contact], s=2, alpha=0.3, color=color,
                       label=result["desc"])

    ax.set_xlabel('侵入深度 (mm)', fontsize=12)
    ax.set_ylabel('法向力 |Fz| (N)', fontsize=12)
    ax.set_title('所有几何体 — 刚度曲线对比', fontsize=14)
    ax.legend(fontsize=10, markerscale=5)
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    summary_path = SCREENSHOTS_DIR / "demo_01_stiffness_comparison.png"
    plt.savefig(summary_path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"  汇总图已保存: {summary_path}")

    print("\n" + "=" * 60)
    print("✓ 基础几何体测试全部完成！")
    print(f"  截图目录: {SCREENSHOTS_DIR}")
    print("=" * 60)


if __name__ == "__main__":
    main()
