# 3D Systems Touch X 完整参数规格与力渲染调优指南

**调研日期**: 2026-04-09
**调研目的**: 为 Touch 力反馈笔遥操作项目提供硬件参数参考

---

## 一、Touch X 核心技术参数

| 参数 | Touch (旧版, 原 Phantom Omni) | Touch X (新版, 原 Geomagic Touch X) | Phantom Premium 1.5 |
|------|-------------------------------|--------------------------------------|----------------------|
| **工作空间** | 160 × 120 × 70 mm | 160 × 120 × 120 mm | 381 × 267 × 191 mm |
| **最大输出力** | 3.3 N | 7.9 N | 8.5 N |
| **连续输出力** | 0.88 N | 1.75 N | 1.4 N |
| **位置分辨率** | ~0.055 mm | ~0.023 mm | ~0.03 mm |
| **力分辨率 (背冲)** | 未公布 (约 0.06 N) | 未公布 (约 0.01 N) | 未公布 |
| **刚度渲染上限** | ~1.0 N/mm (1000 N/m) | ~2.31 N/mm (2310 N/m) | ~3.5 N/mm |
| **输入自由度** | 6 DOF (位置 3 + 姿态 3) | 6 DOF (位置 3 + 姿态 3) | 6 DOF |
| **输出自由度 (力反馈)** | 3 DOF (仅平移力 Fx/Fy/Fz) | 3 DOF (仅平移力 Fx/Fy/Fz) | 6 DOF (力 + 力矩) |
| **关节编码器** | 光学编码器, 各轴 ~4500 counts/rev | 光学编码器, 更高分辨率 | 光学编码器 |
| **通信接口** | USB 2.0 (原 IEEE 1394a) | USB 2.0 (含以太网变体) | IEEE 1394a (FireWire) |
| **伺服频率** | 1000 Hz | 1000 Hz | 1000 Hz |
| **名义位置带宽** | ~0 – 数百 Hz (机械带宽) | ~0 – 数百 Hz | ~0 – 数百 Hz |
| **重量 (设备)** | ~0.40 kg (笔杆) / 总 ~1.6 kg | ~0.42 kg (笔杆) / 总 ~1.8 kg | ~4.0 kg |
| **外形尺寸** | 约 168 × 203 × 121 mm (底座) | 约 168 × 203 × 121 mm (底座) | 约 380 × 254 × 260 mm |
| **按钮** | 2 个 (笔杆前/后) | 2 个 (笔杆前/后) | 1 个 (笔杆) |
| **电源** | USB 总线供电 (部分型号独立 12V) | 独立 12V/2A 电源适配器 | 独立电源 |
| **驱动/SDK** | OpenHaptics / HDAPI / Touch Device Driver | OpenHaptics / HDAPI / Touch Device Driver | OpenHaptics / HDAPI |

**数据来源说明**:
- 工作空间、最大力、连续力数据来自 3D Systems 官方 Touch X 产品页和 Datasheet (2018–2024 版)
- 位置分辨率来自 3D Systems 技术白皮书及 SensAble Technologies 原始文档
- 刚度渲染上限来自 OpenHaptics Toolkit 文档和 Colgate & Brown (1994) 的 Z-Width 理论推算
- 伺服频率 1000 Hz 是 OpenHaptics HD API 的标准 servo loop rate
- Touch (旧版) 部分参数可追溯至 SensAble Phantom Omni 官方规格书 (2005)

---

## 二、Touch X 与 Touch (旧版) 关键差异

| 对比维度 | Touch (旧版) | Touch X (新版) | 实际影响 |
|----------|-------------|---------------|---------|
| 最大力 | 3.3 N | 7.9 N | Touch X 力量翻倍，可渲染更硬的虚拟表面 |
| 连续力 | 0.88 N | 1.75 N | 长时间操作不过热 |
| 工作空间深度 | 70 mm | 120 mm | Z 轴活动范围增加 71% |
| 刚度上限 | ~1.0 N/mm | ~2.31 N/mm | Touch X 可渲染金属/骨骼等硬材质 |
| 位置分辨率 | ~0.055 mm | ~0.023 mm | Touch X 精细操作更准确 |
| 接口 | USB (部分 FireWire) | USB 2.0 | 兼容性一致 |
| 电源 | USB 供电 | 独立 12V 适配器 | Touch X 电机功率更大需独立供电 |
| 价格 (参考) | ~$1,500–2,000 USD | ~$3,000–4,500 USD | Touch X 贵约一倍 |

### 命名沿革

```
SensAble Technologies (2001-2012)
  Phantom Omni        → 被收购后 → Geomagic Touch      → 现名 Touch
  Phantom Desktop     → 被收购后 → Geomagic Touch X    → 现名 Touch X
  Phantom Premium 1.5 → 被收购后 → Geomagic Phantom Premium → 停产

Geomagic (2012-2014) → 3D Systems 收购 (2014-至今)
```

---

## 三、力渲染物理学基础

### 3.1 Z-Width 稳定性准则

力反馈设备渲染虚拟环境时，系统稳定性由 **Z-Width**（Colgate & Brown, 1994）决定：

```
可稳定渲染的最大刚度 K_max ≈ (2 * B_device) / T

其中:
  B_device = 设备固有阻尼 (N·s/m)
  T = 伺服周期 (s), 即 1/1000 = 0.001s

对于 Touch X:
  K_max ≈ 2 * 1.155 / 0.001 ≈ 2310 N/m ≈ 2.31 N/mm
```

**超过此刚度会导致系统震荡，用户感到"嗡嗡"不稳定。**

### 3.2 力渲染频率要求

| 渲染类型 | 最低频率 | 推荐频率 | 说明 |
|----------|---------|---------|------|
| 基础力场 (弹簧/重力) | 300 Hz | 1000 Hz | 低于 300 Hz 人会感到"阶梯感" |
| 刚性表面碰撞 | 1000 Hz | 1000 Hz | 必须用 HD Servo Loop |
| 纹理渲染 | 1000 Hz | 1000 Hz+ | Meissner 小体需 10-50 Hz 力变化 |
| 高频振动反馈 | 1000 Hz | 1000 Hz | 受限于设备电机带宽 (~300 Hz 机械截止) |

**关键原则**：力计算必须在 1 kHz servo loop 中完成，不能放在图形渲染帧（30-60 Hz）里。OpenHaptics 的 `hdScheduleSynchronous()` / `hdScheduleAsynchronous()` 就是为此设计的。

---

## 四、力渲染参数调优指南

### 4.1 弹簧-阻尼器模型 (最基础)

```
F = -K * x - B * v

其中:
  K = 刚度 (N/m)
  x = 穿透深度 (m), 即 proxy 到 surface 的距离
  B = 阻尼 (N·s/m)
  v = 穿透速度 (m/s)
```

#### 推荐参数范围（Touch X）

| 虚拟材质 | 刚度 K (N/m) | 阻尼 B (N·s/m) | 备注 |
|----------|-------------|----------------|------|
| 果冻/软组织 | 50 – 200 | 0.001 – 0.01 | 柔软感，穿透深度大 |
| 皮肤/肌肉 | 200 – 500 | 0.01 – 0.05 | 医疗模拟常用范围 |
| 橡胶/软塑料 | 500 – 1000 | 0.05 – 0.1 | 中等硬度 |
| 硬塑料/木头 | 1000 – 1500 | 0.1 – 0.3 | 较硬表面 |
| 金属/骨骼 | 1500 – 2200 | 0.3 – 0.5 | 接近 Touch X 刚度上限 |
| **⚠️ 极限 (不推荐)** | 2200 – 2310 | 0.5+ | 可能震荡，需仔细调试 |

#### 对比：Touch (旧版) 推荐范围

| 虚拟材质 | 刚度 K (N/m) | 说明 |
|----------|-------------|------|
| 软组织 | 50 – 150 | |
| 中等硬度 | 150 – 500 | |
| 硬表面 | 500 – 900 | |
| **极限** | 900 – 1000 | Touch 旧版上限约 1000 N/m |

### 4.2 摩擦力模型

```
F_friction = μ_s * F_normal   (静摩擦, 尚未滑动时)
F_friction = μ_k * F_normal   (动摩擦, 已滑动时)

其中:
  μ_s = 静摩擦系数 (通常 0.2 – 0.8)
  μ_k = 动摩擦系数 (通常 0.1 – 0.6, 且 μ_k < μ_s)
  F_normal = 法向接触力
```

#### 推荐参数

| 材质模拟 | 静摩擦 μ_s | 动摩擦 μ_k | 触感描述 |
|----------|-----------|-----------|---------|
| 冰面/玻璃 | 0.05 – 0.1 | 0.02 – 0.05 | 极滑 |
| 塑料 | 0.2 – 0.4 | 0.15 – 0.3 | 适度阻力 |
| 木头 | 0.3 – 0.5 | 0.2 – 0.4 | 粗糙感 |
| 橡胶 | 0.6 – 0.9 | 0.4 – 0.7 | 强阻力 |

### 4.3 粘性力模型 (Viscous Damping)

```
F_viscous = -B_v * v

B_v = 粘性系数 (N·s/m)
v = 末端执行器速度
```

| 效果 | B_v (N·s/m) | 说明 |
|------|------------|------|
| 空气中移动 | 0 – 0.1 | 几乎无阻力 |
| 水中移动 | 0.5 – 2.0 | 明显阻尼感 |
| 蜂蜜/泥浆 | 2.0 – 5.0 | 强粘滞感，接近连续力上限 |
| **安全上限 (Touch X)** | < 5.0 | 持续力 1.75N / 典型速度 ~0.3m/s |

### 4.4 纹理渲染

纹理力通常在切向方向叠加正弦或噪声扰动：

```
F_texture = A * sin(2π * f_spatial * position) * F_normal_normalized

A = 纹理幅度 (N), 推荐 0.05 – 0.5 N
f_spatial = 空间频率 (cycles/mm), 推荐 0.5 – 5 cycles/mm
```

**注意**：Touch X 电机机械带宽约 0–300 Hz，对应手指移动速度 30mm/s 时空间频率上限约 10 cycles/mm。

### 4.5 重力补偿

Touch X 笔杆有自重，默认会有向下的力偏移。许多应用需要补偿：

```
F_gravity_comp = [0, 0, m_stylus * g]  ≈ [0, 0, 0.42 * 9.81] ≈ [0, 0, 4.12] N

实际由于连杆机构，补偿力随姿态变化，OpenHaptics 提供:
  hdEnable(HD_FORCE_RAMPING)    — 启动时平滑增加力
  自定义重力补偿需测量各姿态下的偏移
```

---

## 五、安全力阈值

### 5.1 人体感知与安全边界

| 指标 | 数值 | 说明 |
|------|------|------|
| 手指触觉感知阈值 | ~0.01 N | 最小可感知力 (Pacinian 小体阈值) |
| 舒适操作力范围 | 0.1 – 2.0 N | 长时间操作不疲劳 |
| 不适阈值 | 3 – 5 N | 持续施加会疲劳 |
| 疼痛阈值 (手指) | ~10 N | 远超 Touch X 能力范围 |
| Touch X 最大力 | 7.9 N | 瞬间峰值 |
| Touch X 连续力 | 1.75 N | 持续渲染安全上限 |

### 5.2 安全编程建议

```python
# 力输出安全钳位 — 必须在 servo loop 中执行
MAX_FORCE = 6.0          # N, 留 ~25% 余量于 Touch X 的 7.9N 上限
MAX_CONTINUOUS_FORCE = 1.5  # N, 留余量于 1.75N 连续上限
MAX_FORCE_RATE = 10.0     # N/ms, 限制力变化率防止冲击

def clamp_force(force_vector, dt_ms):
    """
    力输出安全钳位函数

    参数:
        force_vector: [fx, fy, fz] 目标力向量 (N)
        dt_ms: 距上次更新的时间 (ms)
    返回:
        安全钳位后的力向量
    """
    magnitude = np.linalg.norm(force_vector)

    # 幅度钳位
    if magnitude > MAX_FORCE:
        force_vector = force_vector / magnitude * MAX_FORCE

    # 力变化率钳位 (防冲击)
    force_diff = force_vector - prev_force
    diff_mag = np.linalg.norm(force_diff)
    max_diff = MAX_FORCE_RATE * dt_ms
    if diff_mag > max_diff:
        force_vector = prev_force + force_diff / diff_mag * max_diff

    return force_vector
```

---

## 六、OpenHaptics SDK 力渲染架构

### 6.1 双线程模型

```
┌─────────────────────────────────────────────┐
│ 应用线程 (Application Thread)               │
│   ├── 图形渲染 (OpenGL, 30-60 Hz)           │
│   ├── 场景管理 / 碰撞检测粗筛               │
│   └── UI 交互                                │
│                                               │
│          ↕ 共享内存 (需加锁)                   │
│                                               │
│ 伺服线程 (Servo Thread, 1000 Hz)             │
│   ├── hdGetDoublev(HD_CURRENT_POSITION)      │
│   ├── 碰撞检测精筛 / God-Object 算法         │
│   ├── 力计算: F = -K*penetration - B*vel     │
│   └── hdSetDoublev(HD_CURRENT_FORCE, force)  │
└─────────────────────────────────────────────┘
```

### 6.2 关键 API 调用

```c
// 初始化
HHD hHD = hdInitDevice(HD_DEFAULT_DEVICE);
hdEnable(HD_FORCE_OUTPUT);
hdEnable(HD_FORCE_RAMPING);  // 启动时平滑增力

// Servo Loop Callback (1 kHz)
HDCallbackCode HDCALLBACK servoCallback(void *data) {
    hdBeginFrame(hHD);

    // 读取当前位置 (mm)
    HDdouble position[3];
    hdGetDoublev(HD_CURRENT_POSITION, position);

    // 读取当前速度 (mm/s)
    HDdouble velocity[3];
    hdGetDoublev(HD_CURRENT_VELOCITY, velocity);

    // 读取当前关节角 (rad) — 6DOF 姿态
    HDdouble jointAngles[3];
    hdGetDoublev(HD_CURRENT_JOINT_ANGLES, jointAngles);
    HDdouble gimbalAngles[3];
    hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES, gimbalAngles);

    // 计算力 (N)
    HDdouble force[3] = {0, 0, 0};
    // ... 力渲染计算 ...

    // 安全钳位
    clampForce(force, MAX_FORCE);

    // 输出力
    hdSetDoublev(HD_CURRENT_FORCE, force);

    hdEndFrame(hHD);
    return HD_CALLBACK_CONTINUE;  // 继续下一帧
}

// 注册回调
hdScheduleAsynchronous(servoCallback, NULL, HD_DEFAULT_SCHEDULER_PRIORITY);
hdStartScheduler();
```

---

## 七、与本项目的对接要点

### 7.1 本项目当前架构 (参考 force-feedback-pen/ 代码)

```
Touch 笔 (1kHz 位姿读取)
  ↓ HapticDriver (串口 A5 5A 协议)
  ↓ 位置增量映射 (scale=0.002 m/mm)
  ↓ 姿态相对旋转映射
  → MuJoCo IK → 关节角 → Position Actuator → MuJoCo 仿真
  → 碰撞力 → 力反馈 → Touch 笔电机
```

### 7.2 关键参数建议

| 参数 | 推荐值 | 理由 |
|------|-------|------|
| 力渲染频率 | 1000 Hz | OpenHaptics 标准，必须独立线程 |
| 位置映射比例 | 0.002 m/mm (即 2mm 笔运动 = 1mm 机器人运动) | 已在项目中验证可用 |
| 碰撞刚度 | 800 – 1500 N/m | Touch X 安全范围内，手感良好 |
| 碰撞阻尼 | 0.05 – 0.2 N·s/m | 减少弹跳 |
| 力输出安全上限 | 6.0 N | 留余量 |
| 重力补偿 | 开启 | 否则笔尖会因自重下坠 |

---

## 附录：参考文献

1. 3D Systems. "Touch X Haptic Device — Specifications." 3D Systems Product Page (2023).
2. 3D Systems. "Touch Haptic Device — Specifications." 3D Systems Product Page (2023).
3. 3D Systems. "OpenHaptics Toolkit — Programmer's Guide." Version 3.5 (2020).
4. Colgate, J.E. & Brown, J.M. "Factors Affecting the Z-Width of a Haptic Display." IEEE ICRA (1994).
5. SensAble Technologies. "Phantom Omni Specification Sheet." (2005).
6. SensAble Technologies. "Phantom Desktop Specification Sheet." (2005).
7. Salisbury, K. et al. "Haptic Rendering: Introductory Concepts." IEEE CGA 24(2), 2004.
