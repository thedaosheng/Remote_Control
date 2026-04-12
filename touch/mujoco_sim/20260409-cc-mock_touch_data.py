#!/usr/bin/env python3
"""
Touch X 力反馈笔模拟轨迹数据生成器

生成多种标准测试轨迹，用于在 MuJoCo 中模拟 Touch 笔的探测行为。
每条轨迹输出为 numpy array: [timestamp, x, y, z] at 1000Hz

轨迹类型：
1. 直线扫描：笔尖从左到右扫过一个平面
2. 垂直下压：笔尖从上方下压到表面，测试不同刚度
3. 圆形扫描：在表面上画圆，测试摩擦力
4. 快速敲击：连续快速碰触表面，测试瞬态响应
5. 边缘探测：沿物体边缘滑动，测试几何感知

依赖: pip install numpy matplotlib
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from pathlib import Path
import os

# === 全局参数 ===
FREQ = 1000  # 采样频率 (Hz)，匹配 Touch X 的 1kHz 伺服循环
SAVE_DIR = Path(__file__).parent.parent / "screenshots"  # 截图保存目录


def generate_linear_scan(duration=2.0, y_offset=0.0, z_surface=0.0,
                         x_range=(-0.08, 0.08)):
    """
    轨迹 1: 直线扫描
    笔尖在 z_surface 平面上从左(x_min)到右(x_max)匀速扫过。
    用于测试平面法向力和表面摩擦力。

    参数:
        duration: 扫描持续时间 (秒)
        y_offset: Y 方向偏移 (米)
        z_surface: 表面高度 (米)，笔尖会略低于此值以产生接触
        x_range: X 方向扫描范围 (米)
    返回:
        trajectory: shape (N, 4) 的 numpy 数组 [time, x, y, z]
    """
    n_samples = int(duration * FREQ)
    t = np.linspace(0, duration, n_samples)
    x = np.linspace(x_range[0], x_range[1], n_samples)
    y = np.full(n_samples, y_offset)
    # 笔尖下压 2mm 到表面以下，确保产生接触力
    z = np.full(n_samples, z_surface - 0.002)
    return np.column_stack([t, x, y, z])


def generate_vertical_press(duration=3.0, x_pos=0.0, y_pos=0.0,
                            z_start=0.05, z_surface=0.0, z_depth=-0.005):
    """
    轨迹 2: 垂直下压
    笔尖从高处缓慢下压到表面，再继续下压测试刚度响应。
    包含三个阶段：接近（自由空间）→ 接触 → 深压。

    参数:
        z_start: 起始高度 (米)
        z_surface: 表面高度 (米)
        z_depth: 最大下压深度 (米，负值表示表面以下)
    """
    n_samples = int(duration * FREQ)
    t = np.linspace(0, duration, n_samples)

    # 三段式运动：先快速接近，再缓慢下压，最后保持
    phase1_end = int(n_samples * 0.3)   # 30% 时间接近
    phase2_end = int(n_samples * 0.8)   # 50% 时间下压
    # 剩余 20% 时间保持最大深度

    z = np.zeros(n_samples)
    # 阶段1: 从 z_start 快速降到 z_surface
    z[:phase1_end] = np.linspace(z_start, z_surface, phase1_end)
    # 阶段2: 从 z_surface 缓慢下压到 z_depth
    z[phase1_end:phase2_end] = np.linspace(z_surface, z_depth, phase2_end - phase1_end)
    # 阶段3: 保持最大深度
    z[phase2_end:] = z_depth

    x = np.full(n_samples, x_pos)
    y = np.full(n_samples, y_pos)
    return np.column_stack([t, x, y, z])


def generate_circular_scan(duration=4.0, radius=0.03, z_surface=0.0,
                           center=(0.0, 0.0)):
    """
    轨迹 3: 圆形扫描
    笔尖在表面上画圆，用于测试各方向的摩擦力和切向力。
    圆的频率约 1Hz，可以清楚观察摩擦力的方向变化。

    参数:
        radius: 圆的半径 (米)
        center: 圆心 (x, y) 坐标
    """
    n_samples = int(duration * FREQ)
    t = np.linspace(0, duration, n_samples)
    omega = 2 * np.pi * 1.0  # 1Hz 画圆频率

    x = center[0] + radius * np.cos(omega * t)
    y = center[1] + radius * np.sin(omega * t)
    # 笔尖保持在表面以下 1.5mm
    z = np.full(n_samples, z_surface - 0.0015)
    return np.column_stack([t, x, y, z])


def generate_rapid_tapping(duration=3.0, x_pos=0.0, y_pos=0.0,
                           z_top=0.02, z_surface=0.0, tap_freq=5.0):
    """
    轨迹 4: 快速敲击
    笔尖以固定频率上下敲击表面，测试瞬态接触力和碰撞响应。
    模拟用户快速点击物体表面的场景。

    参数:
        tap_freq: 敲击频率 (Hz)
        z_top: 抬起高度 (米)
    """
    n_samples = int(duration * FREQ)
    t = np.linspace(0, duration, n_samples)

    # 使用 abs(sin) 生成周期性敲击运动
    # 负半周期（接触）持续时间短，正半周期（抬起）持续时间长
    raw = np.sin(2 * np.pi * tap_freq * t)
    # 将正弦波映射到 z 坐标：
    # sin > 0 → 抬起（z_top 方向）
    # sin < 0 → 下压（z_surface 以下）
    z = np.where(raw > 0,
                 z_surface + raw * z_top,        # 抬起
                 z_surface + raw * 0.003)         # 下压 3mm

    x = np.full(n_samples, x_pos)
    y = np.full(n_samples, y_pos)
    return np.column_stack([t, x, y, z])


def generate_edge_trace(duration=3.0, z_surface=0.0, box_size=0.05):
    """
    轨迹 5: 边缘探测
    笔尖沿一个立方体的顶面边缘滑动，当经过边缘时会产生力的突变。
    路径为正方形，沿四条边依次移动。

    参数:
        box_size: 立方体边长 (米)
    """
    n_samples = int(duration * FREQ)
    t = np.linspace(0, duration, n_samples)
    half = box_size / 2

    # 正方形路径的四个顶点
    corners = [
        (-half, -half),
        (half, -half),
        (half, half),
        (-half, half),
        (-half, -half),  # 回到起点
    ]

    # 沿四条边均匀分配采样点
    seg_len = n_samples // 4
    x = np.zeros(n_samples)
    y = np.zeros(n_samples)

    for i in range(4):
        start_idx = i * seg_len
        end_idx = (i + 1) * seg_len if i < 3 else n_samples
        length = end_idx - start_idx
        x[start_idx:end_idx] = np.linspace(corners[i][0], corners[i + 1][0], length)
        y[start_idx:end_idx] = np.linspace(corners[i][1], corners[i + 1][1], length)

    # 笔尖在立方体顶面高度，略微下压
    z = np.full(n_samples, z_surface + box_size - 0.001)
    return np.column_stack([t, x, y, z])


def plot_trajectory(traj, title, filename):
    """
    可视化单条轨迹：3D 路径图 + XYZ 分量时间序列

    参数:
        traj: shape (N, 4) [time, x, y, z]
        title: 图表标题
        filename: 保存文件名
    """
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 5))

    # 子图1: XZ 平面路径（侧视图，最能展示下压和扫描动作）
    ax1.plot(traj[:, 1] * 1000, traj[:, 3] * 1000,
             linewidth=0.8, color='steelblue')
    ax1.set_xlabel('X (mm)')
    ax1.set_ylabel('Z (mm)')
    ax1.set_title(f'{title} - XZ 侧视图')
    ax1.grid(True, alpha=0.3)
    ax1.set_aspect('equal')

    # 子图2: XYZ 分量随时间变化
    ax2.plot(traj[:, 0], traj[:, 1] * 1000, label='X', alpha=0.8)
    ax2.plot(traj[:, 0], traj[:, 2] * 1000, label='Y', alpha=0.8)
    ax2.plot(traj[:, 0], traj[:, 3] * 1000, label='Z', alpha=0.8)
    ax2.set_xlabel('时间 (s)')
    ax2.set_ylabel('位置 (mm)')
    ax2.set_title(f'{title} - 时间序列')
    ax2.legend()
    ax2.grid(True, alpha=0.3)

    plt.tight_layout()
    save_path = SAVE_DIR / filename
    plt.savefig(save_path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"  已保存: {save_path}")


def main():
    """
    主函数：生成所有 5 种测试轨迹，保存数据和可视化图。
    """
    os.makedirs(SAVE_DIR, exist_ok=True)

    print("=" * 60)
    print("Touch X 模拟轨迹数据生成器")
    print("=" * 60)

    # 定义所有轨迹
    trajectories = {
        "linear_scan": {
            "func": generate_linear_scan,
            "title": "直线扫描",
            "desc": "笔尖从左到右扫过平面，测试法向力"
        },
        "vertical_press": {
            "func": generate_vertical_press,
            "title": "垂直下压",
            "desc": "笔尖从上方下压到表面，测试刚度响应"
        },
        "circular_scan": {
            "func": generate_circular_scan,
            "title": "圆形扫描",
            "desc": "笔尖在表面上画圆，测试摩擦力"
        },
        "rapid_tapping": {
            "func": generate_rapid_tapping,
            "title": "快速敲击",
            "desc": "笔尖连续快速碰触表面，测试瞬态响应"
        },
        "edge_trace": {
            "func": generate_edge_trace,
            "title": "边缘探测",
            "desc": "笔尖沿物体边缘滑动，测试几何感知"
        },
    }

    # 生成并保存所有轨迹
    data_dir = Path(__file__).parent / "trajectory_data"
    os.makedirs(data_dir, exist_ok=True)

    for name, info in trajectories.items():
        print(f"\n▸ 生成轨迹: {info['title']} ({info['desc']})")
        traj = info["func"]()
        print(f"  采样点数: {traj.shape[0]}, 持续时间: {traj[-1, 0]:.1f}s")
        print(f"  X 范围: [{traj[:, 1].min()*1000:.1f}, {traj[:, 1].max()*1000:.1f}] mm")
        print(f"  Y 范围: [{traj[:, 2].min()*1000:.1f}, {traj[:, 2].max()*1000:.1f}] mm")
        print(f"  Z 范围: [{traj[:, 3].min()*1000:.1f}, {traj[:, 3].max()*1000:.1f}] mm")

        # 保存 numpy 数据
        npy_path = data_dir / f"{name}.npy"
        np.save(npy_path, traj)
        print(f"  数据已保存: {npy_path}")

        # 绘制可视化图
        plot_trajectory(traj, info["title"], f"trajectory_{name}.png")

    # 生成汇总图：所有轨迹在同一个 XZ 侧视图中
    print("\n▸ 生成汇总图...")
    fig, ax = plt.subplots(figsize=(12, 8))
    colors = ['steelblue', 'coral', 'forestgreen', 'darkorange', 'mediumpurple']

    for (name, info), color in zip(trajectories.items(), colors):
        traj = info["func"]()
        ax.plot(traj[:, 1] * 1000, traj[:, 3] * 1000,
                label=info["title"], linewidth=0.8, color=color, alpha=0.7)

    ax.set_xlabel('X (mm)')
    ax.set_ylabel('Z (mm)')
    ax.set_title('全部测试轨迹概览 (XZ 侧视图)')
    ax.legend(loc='upper left')
    ax.grid(True, alpha=0.3)
    summary_path = SAVE_DIR / "trajectory_all_overview.png"
    plt.savefig(summary_path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"  汇总图已保存: {summary_path}")

    print("\n" + "=" * 60)
    print("✓ 所有轨迹生成完毕！")
    print(f"  数据目录: {data_dir}")
    print(f"  截图目录: {SAVE_DIR}")
    print("=" * 60)


if __name__ == "__main__":
    main()
