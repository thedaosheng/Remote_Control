#!/usr/bin/env python3
"""
Touch X 表面纹理力渲染模拟 — MuJoCo 仿真

通过调整 MuJoCo 的接触参数模拟不同材质的触感：
1. 冰面（极低摩擦）
2. 木板（中等摩擦）
3. 橡胶（高摩擦）
4. 海绵（低刚度）
5. 钢铁（高刚度）
6. 瓦楞纹理（周期性凹凸）
7. 砂纸（高摩擦+微振动）
8. 果冻（低刚度+高阻尼）

每种材质输出：
- 法向力和切向力时间序列
- 力的频谱分析（FFT）
- 人体感知判断（基于力幅值和频率）

依赖: pip install mujoco numpy matplotlib scipy
"""

import mujoco
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from pathlib import Path
import os

# === 路径设置 ===
SCRIPT_DIR = Path(__file__).parent
SCREENSHOTS_DIR = SCRIPT_DIR.parent / "screenshots"
os.makedirs(SCREENSHOTS_DIR, exist_ok=True)


# === 材质定义 ===
# 每种材质由 MuJoCo 接触参数定义
# friction: [滑动摩擦, 扭转摩擦, 滚动摩擦]
# solref: [时间常数, 阻尼比] — 控制接触刚度
# solimp: [d_min, d_max, width] — 控制接触阻尼和侵入

MATERIALS = {
    "ice": {
        "name": "冰面",
        "friction": "0.05 0.001 0.0001",
        "solref": "0.002 1.0",
        "solimp": "0.95 0.99 0.001",
        "rgba": "0.85 0.92 0.98 0.9",
        "perception": "FA-I 可感知（极低阻力，滑溜感）",
        "desc": "极低摩擦系数(0.05)，高刚度，模拟光滑冰面"
    },
    "wood": {
        "name": "木板",
        "friction": "0.4 0.005 0.0001",
        "solref": "0.003 1.0",
        "solimp": "0.92 0.97 0.001",
        "rgba": "0.72 0.53 0.30 1",
        "perception": "SA-II 可感知（中等阻力，稳定支撑）",
        "desc": "中等摩擦(0.4)，较高刚度，模拟抛光木面"
    },
    "rubber": {
        "name": "橡胶",
        "friction": "1.5 0.02 0.001",
        "solref": "0.008 0.8",
        "solimp": "0.85 0.95 0.002",
        "rgba": "0.15 0.15 0.15 1",
        "perception": "SA-I/SA-II 强感知（高阻力，明显形变）",
        "desc": "高摩擦(1.5)，中刚度，模拟橡胶垫"
    },
    "sponge": {
        "name": "海绵",
        "friction": "0.6 0.01 0.001",
        "solref": "0.04 0.5",
        "solimp": "0.5 0.8 0.01",
        "rgba": "0.95 0.90 0.30 1",
        "perception": "SA-I 可感知（深侵入，软弹感）",
        "desc": "中摩擦(0.6)，极低刚度(solref=0.04)，大阻尼，模拟海绵"
    },
    "steel": {
        "name": "钢铁",
        "friction": "0.3 0.003 0.0001",
        "solref": "0.001 1.2",
        "solimp": "0.98 0.99 0.0005",
        "rgba": "0.75 0.75 0.78 1",
        "perception": "SA-II 强感知（极硬壁感，几乎零侵入）",
        "desc": "中低摩擦(0.3)，极高刚度(solref=0.001)，模拟钢板"
    },
    "jelly": {
        "name": "果冻",
        "friction": "0.2 0.005 0.001",
        "solref": "0.05 0.3",
        "solimp": "0.3 0.7 0.02",
        "rgba": "0.90 0.30 0.35 0.7",
        "perception": "SA-I 可感知（深侵入+高阻尼，黏滞感）",
        "desc": "低摩擦(0.2)，极低刚度+高阻尼，模拟果冻"
    },
    "sandpaper": {
        "name": "砂纸",
        "friction": "1.8 0.03 0.002",
        "solref": "0.002 1.0",
        "solimp": "0.92 0.97 0.001",
        "rgba": "0.65 0.55 0.40 1",
        "perception": "FA-I 可感知（高阻力+表面粗糙振动）",
        "desc": "极高摩擦(1.8)，高刚度，模拟粗砂纸"
    },
}


def get_flat_surface_xml(material_key):
    """
    生成含有指定材质平面的 MuJoCo XML 场景。

    参数:
        material_key: MATERIALS 字典中的键名
    返回:
        XML 字符串
    """
    mat = MATERIALS[material_key]
    return f"""
    <mujoco model="texture_test_{material_key}">
        <option timestep="0.001" gravity="0 0 -9.81" integrator="implicit">
            <flag contact="enable"/>
        </option>

        <worldbody>
            <!-- 参考地面 -->
            <geom type="plane" size="0.3 0.3 0.01" rgba="0.95 0.95 0.95 1"
                  pos="0 0 -0.001" contype="0" conaffinity="0"/>

            <!-- 材质表面 -->
            <body name="surface" pos="0 0 0">
                <geom name="surface_geom" type="box" size="0.1 0.1 0.005"
                      rgba="{mat['rgba']}" pos="0 0 0.005"
                      friction="{mat['friction']}"
                      solref="{mat['solref']}"
                      solimp="{mat['solimp']}"/>
            </body>

            <!-- mocap 目标（不可见，不碰撞） -->
            <body name="mocap_target" mocap="true" pos="0 0 0.05">
                <geom type="sphere" size="0.001" rgba="0 0 0 0"
                      contype="0" conaffinity="0"/>
            </body>

            <!-- 探针：普通 body + freejoint，可碰撞 -->
            <body name="probe" pos="0 0 0.05">
                <freejoint name="probe_joint"/>
                <geom name="probe_tip" type="sphere" size="0.003"
                      rgba="1 0.2 0.2 0.8" mass="0.05"
                      friction="0.5 0.005 0.0001"
                      solref="0.002 1.0" solimp="0.95 0.99 0.001"/>
            </body>

            <light pos="0.5 0.5 1" dir="-0.5 -0.5 -1"/>
        </worldbody>

        <equality>
            <weld body1="probe" body2="mocap_target"
                  solref="0.001 1" solimp="0.95 0.99 0.001"/>
        </equality>
    </mujoco>
    """


def get_corrugated_surface_xml():
    """
    生成瓦楞纹理表面的 MuJoCo XML。
    用多个小 box 排列形成周期性凹凸表面。
    """
    # 生成一排小凸起，间距 5mm，高度 2mm
    bumps = ""
    for i in range(-10, 11):
        x = i * 0.005  # 每 5mm 一个凸起
        bumps += f"""
            <geom type="box" size="0.001 0.05 0.001" pos="{x} 0 0.011"
                  rgba="0.6 0.4 0.2 1"
                  friction="0.5 0.005 0.0001"
                  solref="0.003 1.0" solimp="0.92 0.97 0.001"/>
        """

    return f"""
    <mujoco model="texture_test_corrugated">
        <option timestep="0.001" gravity="0 0 -9.81" integrator="implicit">
            <flag contact="enable"/>
        </option>

        <worldbody>
            <geom type="plane" size="0.3 0.3 0.01" rgba="0.95 0.95 0.95 1"
                  pos="0 0 -0.001" contype="0" conaffinity="0"/>

            <!-- 基底平面 -->
            <body name="surface" pos="0 0 0">
                <geom name="base_surface" type="box" size="0.08 0.05 0.005"
                      rgba="0.6 0.4 0.2 1" pos="0 0 0.005"
                      friction="0.5 0.005 0.0001"/>
                <!-- 周期性凸起 -->
                {bumps}
            </body>

            <!-- mocap 目标 -->
            <body name="mocap_target" mocap="true" pos="0 0 0.05">
                <geom type="sphere" size="0.001" rgba="0 0 0 0"
                      contype="0" conaffinity="0"/>
            </body>

            <!-- 探针 -->
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


def run_texture_test(xml_string, material_name, duration=3.0, sweep_z=0.009):
    """
    运行单个材质测试：探针在表面上水平扫过，记录法向力和切向力。

    参数:
        xml_string: MuJoCo XML 场景
        material_name: 材质名称（用于输出）
        duration: 仿真时间 (秒)
        sweep_z: 探针扫描高度 (米)，需略低于表面顶部以产生接触
    返回:
        dict: 包含时间序列数据
    """
    print(f"\n  测试材质: {material_name}")

    model = mujoco.MjModel.from_xml_string(xml_string)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    dt = model.opt.timestep
    n_steps = int(duration / dt)
    time_arr = np.arange(n_steps) * dt

    # 记录数组
    normal_force = np.zeros(n_steps)    # 法向力 (Z 方向)
    tangential_force = np.zeros(n_steps)  # 切向力 (XY 平面)
    forces_3d = np.zeros((n_steps, 3))
    penetrations = np.zeros(n_steps)

    for i in range(n_steps):
        t = time_arr[i]
        progress = t / duration

        # 水平扫描轨迹：从左到右
        x = -0.06 + 0.12 * progress
        y = 0.0
        z = sweep_z

        data.mocap_pos[0] = [x, y, z]
        mujoco.mj_step(model, data)

        # 提取接触力
        for j in range(data.ncon):
            contact = data.contact[j]
            geom1 = model.geom(contact.geom1).name if contact.geom1 >= 0 else ""
            geom2 = model.geom(contact.geom2).name if contact.geom2 >= 0 else ""

            if "probe_tip" in geom1 or "probe_tip" in geom2:
                force = np.zeros(6)
                mujoco.mj_contactForce(model, data, j, force)
                frame = contact.frame.reshape(3, 3)
                world_force = frame.T @ force[:3]
                forces_3d[i] += world_force
                normal_force[i] += abs(world_force[2])
                tangential_force[i] += np.sqrt(world_force[0]**2 + world_force[1]**2)
                penetrations[i] = max(penetrations[i], -contact.dist)

    # 统计
    print(f"    平均法向力: {normal_force[normal_force > 0].mean():.4f} N" if (normal_force > 0).any() else "    无接触")
    print(f"    平均切向力: {tangential_force[tangential_force > 0].mean():.4f} N" if (tangential_force > 0).any() else "    无切向力")
    print(f"    最大侵入深度: {penetrations.max()*1000:.3f} mm")

    return {
        "time": time_arr,
        "normal_force": normal_force,
        "tangential_force": tangential_force,
        "forces_3d": forces_3d,
        "penetration": penetrations,
        "name": material_name,
    }


def plot_material_comparison(results, save_name):
    """
    绘制所有材质的力-时间对比图（法向力 + 切向力 + 摩擦系数比）

    参数:
        results: 所有材质测试结果列表
        save_name: 保存文件名
    """
    n_mats = len(results)
    fig, axes = plt.subplots(3, 1, figsize=(14, 12), sharex=True)
    colors = plt.cm.tab10(np.linspace(0, 1, n_mats))

    for result, color in zip(results, colors):
        t = result["time"]
        label = result["name"]

        # 法向力
        axes[0].plot(t, result["normal_force"], label=label,
                     alpha=0.7, linewidth=0.8, color=color)

        # 切向力
        axes[1].plot(t, result["tangential_force"], label=label,
                     alpha=0.7, linewidth=0.8, color=color)

        # 有效摩擦系数 = 切向力/法向力
        with np.errstate(divide='ignore', invalid='ignore'):
            mu = np.where(result["normal_force"] > 0.001,
                          result["tangential_force"] / result["normal_force"], 0)
        axes[2].plot(t, mu, label=label, alpha=0.7, linewidth=0.8, color=color)

    axes[0].set_ylabel('法向力 (N)')
    axes[0].set_title('材质对比 — 法向力')
    axes[0].legend(fontsize=8, ncol=2)
    axes[0].grid(True, alpha=0.3)

    axes[1].set_ylabel('切向力 (N)')
    axes[1].set_title('材质对比 — 切向力（摩擦力）')
    axes[1].legend(fontsize=8, ncol=2)
    axes[1].grid(True, alpha=0.3)

    axes[2].set_xlabel('时间 (s)')
    axes[2].set_ylabel('有效摩擦系数 μ')
    axes[2].set_title('材质对比 — 有效摩擦系数')
    axes[2].legend(fontsize=8, ncol=2)
    axes[2].grid(True, alpha=0.3)
    axes[2].set_ylim(0, 3.0)

    plt.tight_layout()
    save_path = SCREENSHOTS_DIR / save_name
    plt.savefig(save_path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"  材质对比图已保存: {save_path}")


def plot_fft_analysis(results, save_name):
    """
    绘制各材质力信号的频谱分析（FFT），
    用于判断人体是否能感知到力的变化。

    人体动觉感知频率范围：
    - SA-II (Ruffini): 0-10 Hz，方向/拉伸
    - FA-I (Meissner): 10-50 Hz，振动纹理
    - FA-II (Pacinian): 40-400 Hz，高频振动

    Touch X 力更新频率: 1000 Hz → Nyquist 500 Hz
    """
    fig, axes = plt.subplots(2, 1, figsize=(14, 10))
    colors = plt.cm.tab10(np.linspace(0, 1, len(results)))

    for result, color in zip(results, colors):
        fn = result["normal_force"]
        ft = result["tangential_force"]
        dt = result["time"][1] - result["time"][0]
        n = len(fn)

        # 法向力 FFT
        fft_fn = np.abs(np.fft.rfft(fn - fn.mean()))
        fft_ft = np.abs(np.fft.rfft(ft - ft.mean()))
        freqs = np.fft.rfftfreq(n, dt)

        # 只显示 0-200 Hz（人体感知范围）
        mask = freqs <= 200

        axes[0].semilogy(freqs[mask], fft_fn[mask] + 1e-10,
                         label=result["name"], alpha=0.7, linewidth=0.8, color=color)
        axes[1].semilogy(freqs[mask], fft_ft[mask] + 1e-10,
                         label=result["name"], alpha=0.7, linewidth=0.8, color=color)

    # 标注人体感知频段
    for ax in axes:
        ax.axvspan(0, 10, alpha=0.05, color='blue', label='SA-II (0-10Hz)')
        ax.axvspan(10, 50, alpha=0.05, color='green', label='FA-I (10-50Hz)')
        ax.axvspan(40, 200, alpha=0.05, color='red', label='FA-II (40-400Hz)')

    axes[0].set_ylabel('幅值 (对数)')
    axes[0].set_title('法向力频谱分析 — 材质差异')
    axes[0].legend(fontsize=7, ncol=3)
    axes[0].grid(True, alpha=0.3)

    axes[1].set_xlabel('频率 (Hz)')
    axes[1].set_ylabel('幅值 (对数)')
    axes[1].set_title('切向力频谱分析 — 纹理识别')
    axes[1].legend(fontsize=7, ncol=3)
    axes[1].grid(True, alpha=0.3)

    plt.tight_layout()
    save_path = SCREENSHOTS_DIR / save_name
    plt.savefig(save_path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"  FFT 分析图已保存: {save_path}")


def plot_perception_summary(results, save_name):
    """
    绘制人体感知能力评估热力图：
    每种材质 × 每个感知通道 的感知强度评分

    评分依据：
    - 法向力均值和方差 → SA-I/SA-II 通道
    - 切向力均值 → SA-II 通道
    - 力信号高频成分能量 → FA-I/FA-II 通道
    """
    channels = ['SA-I\n(静态压力)', 'SA-II\n(力方向)', 'FA-I\n(振动纹理)', 'FA-II\n(高频振动)']
    mat_names = [r["name"] for r in results]
    scores = np.zeros((len(results), 4))

    for i, result in enumerate(results):
        fn = result["normal_force"]
        ft = result["tangential_force"]
        dt = result["time"][1] - result["time"][0]
        n = len(fn)

        # SA-I: 静态压力感知 — 与法向力均值和侵入深度成正比
        fn_mean = fn[fn > 0].mean() if (fn > 0).any() else 0
        pen_max = result["penetration"].max() * 1000
        scores[i, 0] = min(fn_mean * 5 + pen_max * 2, 10)

        # SA-II: 力方向感知 — 与切向力和法向力的比值成正比
        ft_mean = ft[ft > 0].mean() if (ft > 0).any() else 0
        scores[i, 1] = min((ft_mean + fn_mean) * 3, 10)

        # FA-I: 振动纹理 (10-50 Hz) — FFT 能量
        fft_fn = np.abs(np.fft.rfft(fn - fn.mean()))
        freqs = np.fft.rfftfreq(n, dt)
        fa1_mask = (freqs >= 10) & (freqs <= 50)
        fa1_energy = fft_fn[fa1_mask].sum() if fa1_mask.any() else 0
        scores[i, 2] = min(fa1_energy * 0.5, 10)

        # FA-II: 高频振动 (40-400 Hz) — FFT 能量
        fa2_mask = (freqs >= 40) & (freqs <= 400)
        fa2_energy = fft_fn[fa2_mask].sum() if fa2_mask.any() else 0
        scores[i, 3] = min(fa2_energy * 0.3, 10)

    # 绘制热力图
    fig, ax = plt.subplots(figsize=(10, 7))
    im = ax.imshow(scores, cmap='YlOrRd', aspect='auto', vmin=0, vmax=10)

    ax.set_xticks(range(4))
    ax.set_xticklabels(channels, fontsize=11)
    ax.set_yticks(range(len(mat_names)))
    ax.set_yticklabels(mat_names, fontsize=11)

    # 在每个格子中显示数值
    for i in range(len(mat_names)):
        for j in range(4):
            color = 'white' if scores[i, j] > 5 else 'black'
            ax.text(j, i, f'{scores[i, j]:.1f}', ha='center', va='center',
                    fontsize=12, fontweight='bold', color=color)

    plt.colorbar(im, ax=ax, label='感知强度 (0=无感知, 10=强感知)')
    ax.set_title('Touch X 材质力渲染 — 人体感知通道评估\n'
                 '（注意：Touch X 仅提供动觉反馈，SA-I/FA-I/FA-II 通道需皮肤触觉补充）',
                 fontsize=12)

    plt.tight_layout()
    save_path = SCREENSHOTS_DIR / save_name
    plt.savefig(save_path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"  感知评估图已保存: {save_path}")


# === 主函数 ===

def main():
    """
    运行所有材质纹理测试并生成对比图。
    """
    print("=" * 60)
    print("Touch X 表面纹理力渲染模拟 — MuJoCo 仿真")
    print("=" * 60)

    results = []

    # 测试所有均匀材质
    for mat_key, mat_info in MATERIALS.items():
        try:
            xml = get_flat_surface_xml(mat_key)
            result = run_texture_test(xml, mat_info["name"])
            results.append(result)
        except Exception as e:
            print(f"  ❌ {mat_info['name']} 测试失败: {e}")

    # 测试瓦楞纹理
    try:
        print("\n  测试特殊纹理: 瓦楞表面")
        xml = get_corrugated_surface_xml()
        result = run_texture_test(xml, "瓦楞纹理", sweep_z=0.011)
        results.append(result)
    except Exception as e:
        print(f"  ❌ 瓦楞纹理测试失败: {e}")

    if not results:
        print("❌ 所有测试失败，无法生成对比图")
        return

    # 生成所有对比图
    print("\n" + "=" * 50)
    print("生成对比可视化...")
    plot_material_comparison(results, "demo_02_texture_comparison.png")
    plot_fft_analysis(results, "demo_02_texture_fft.png")
    plot_perception_summary(results, "demo_02_perception_heatmap.png")

    # 为每种材质单独生成力曲线
    for result in results:
        safe_name = result["name"].replace(" ", "_")
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 6), sharex=True)

        t = result["time"]
        ax1.plot(t, result["normal_force"], color='steelblue', linewidth=0.5)
        ax1.set_ylabel('法向力 (N)')
        ax1.set_title(f'{result["name"]} — 力时间序列')
        ax1.grid(True, alpha=0.3)

        ax2.plot(t, result["tangential_force"], color='coral', linewidth=0.5)
        ax2.set_xlabel('时间 (s)')
        ax2.set_ylabel('切向力 (N)')
        ax2.grid(True, alpha=0.3)

        plt.tight_layout()
        save_path = SCREENSHOTS_DIR / f"demo_02_{safe_name}_force.png"
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        plt.close()

    print("\n" + "=" * 60)
    print("✓ 表面纹理测试全部完成！")
    print(f"  截图目录: {SCREENSHOTS_DIR}")
    print("=" * 60)


if __name__ == "__main__":
    main()
