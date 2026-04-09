# 3D Systems Touch 官方展示场景与力渲染模式分析

**调研日期**: 2026-04-09
**目的**: 整理 Touch/Touch X 力反馈设备的典型应用场景和力渲染模式

---

## 一、医疗培训（手术模拟）

### 1.1 骨科手术模拟

**场景描述**：用 Touch X 模拟骨钻操作。用户握持笔杆如同握持骨钻，穿过皮肤→肌肉→骨骼时感受不同的阻力变化。当钻头穿透骨皮质到达松质骨时，阻力突然减小（"breakthrough" 感觉）。

**力渲染模式**：
- **多层弹簧-阻尼模型**：皮肤层 K=200 N/m, 肌肉层 K=400 N/m, 骨皮质层 K=1800 N/m, 松质骨 K=300 N/m
- **阻力突变**（breakthrough detection）：检测穿透深度超过阈值后突然降低 K 值
- **旋转摩擦**：模拟钻头旋转时的切削阻力
- **速度依赖阻尼**：快速推进时阻尼增大，模拟材料抗力

**代表产品**: 3D Systems 与合作伙伴开发的骨科训练系统；Immersion Medical 的 CathSim/EndoSim 系列

### 1.2 牙科模拟

**场景描述**：使用 Touch 设备模拟牙科钻操作。学生可以感受到牙釉质、牙本质、牙髓腔的不同硬度，学习力控制，避免钻穿牙髓。

**力渲染模式**：
- **高刚度表面接触**：牙釉质 K ≈ 1500-2000 N/m（接近 Touch X 上限）
- **深度依赖刚度**：牙本质比牙釉质软 (K ≈ 800 N/m)，牙髓极软 (K ≈ 100 N/m)
- **材料去除**（voxel carving）：接触点附近体素被"切除"，几何形状实时更新
- **振动反馈**：钻头与硬组织接触时叠加高频正弦力 (50-200 Hz, 振幅 0.1-0.3 N)

**代表产品**: Moog Simodont Dental Trainer（使用 Touch/Touch X 级别的力反馈设备）; DentSim/PerioSim

### 1.3 微创手术 / 腹腔镜模拟

**场景描述**：模拟腔镜器械穿过 trocar 后在体腔内操作。用户需感受器械与组织的接触力、缝合时的线张力。

**力渲染模式**：
- **Trocar 约束力**：在 trocar 入口点施加径向弹簧力，模拟支点运动
- **组织变形**（弹性体碰撞）：基于有限元或质点-弹簧模型实时计算接触力
- **缝合力**：线穿过组织时的拉力反馈 (0.2-1.0 N 范围)
- **滑动摩擦**：器械在组织表面滑动的摩擦反馈

**代表产品**: Mimic Technologies dV-Trainer; Simbionix LAP Mentor

### 1.4 介入心脏手术 / 导管模拟

**场景描述**：模拟导管穿过血管的触觉反馈。操作者推送导管时感受血管壁的接触、分叉处的选择。

**力渲染模式**：
- **管道约束力**：血管壁用圆柱面弹簧模型约束，导管偏离中心时给径向回复力
- **摩擦阻力**：与导管插入长度成正比的摩擦力
- **脉动力**：叠加心跳频率 (~1.2 Hz) 的周期力，模拟血管搏动
- **穿孔警告**：超过安全力阈值时急剧增加阻力

---

## 二、虚拟雕刻（Geomagic Freeform / ClayTools）

### 2.1 自由形态建模

**场景描述**：3D Systems 旗舰产品 Geomagic Freeform（现名 Freeform）。设计师使用 Touch X 如同操作真实雕刻工具，在虚拟粘土上进行增减材操作。主要用于珠宝设计、假肢/矫形器定制、工业设计原型。

**力渲染模式**：
- **体素碰撞**（Voxel-based sculpting）：3D 体素网格定义形状，笔尖接触体素表面时产生法向弹簧力
- **材料去除力**：施加超过阈值的力时"挖掉"体素，模拟雕刻去除材料
- **材料添加**：按下按钮时在笔尖位置添加体素，有轻微阻力反馈
- **表面光滑力**：光滑工具将法向力引导到"磨平"凸起的方向
- **刚度可变**：用户选择不同"工具硬度"时改变 K 值 (200-1500 N/m)

**关键技术**: 使用 God-Object 算法（Zilles & Salisbury, 1995）做表面跟踪，确保 proxy 点始终在几何表面上或外部。

### 2.2 触觉纹理绘制

**场景描述**：在 3D 模型表面"涂画"凹凸纹理。手指通过 Touch 笔感受正在绘制的纹理。

**力渲染模式**：
- **法向力调制**：基于 displacement map 的高度值调制法向力大小
- **切向纹理力**：沿表面移动时根据纹理梯度施加切向力分量
- **频率**: 空间频率 0.5-3 cycles/mm，力幅度 0.05-0.3 N

---

## 三、分子与纳米可视化

### 3.1 分子力场交互

**场景描述**：将分子动力学模拟（MD）与力反馈连接。研究者使用 Touch 笔"抓取"分子或配体，在蛋白质口袋中移动，实时感受分子间作用力（范德华力、静电力、氢键）。

**力渲染模式**：
- **势场梯度力**：F = -∇V(r)，将分子力场的负梯度作为反馈力
- **范德华力**：Lennard-Jones 势的力，近距排斥 + 远距吸引
- **静电库仑力**：带电基团之间的长程力
- **力缩放**：分子力 (pN-nN) 缩放到人可感知范围 (0.1-3 N)
- **位置缩放**：纳米尺度 → 厘米尺度（典型缩放 10^7-10^8）

**代表项目**: Interactive Molecular Dynamics (iMD); VMD + NAMD 集成; SAMSON (OneAngstrom); Nano-Manipulation

### 3.2 原子力显微镜 (AFM) 可视化

**场景描述**：将 AFM 扫描数据转为 3D 力反馈地形图。用户用 Touch 笔"触摸"纳米级表面形貌。

**力渲染模式**：
- **高度图力渲染**：将 AFM 高度数据作为碰撞面，法向弹簧力
- **表面粗糙度纹理**：AFM 数据的高频分量叠加为纹理力
- **粘附力**：模拟 AFM 探针与表面之间的粘附（"snap-in" 和 "pull-off"）

---

## 四、机器人遥操作

### 4.1 主从遥操作

**场景描述**：Touch X 作为"主端"（master），远程机器人作为"从端"（slave）。操作者移动 Touch 笔，机器人末端跟随运动。机器人接触环境时的力通过力传感器反馈到 Touch 笔。

**力渲染模式**：
- **双边遥操作**（Bilateral Teleoperation）：位置-力映射或力-位置映射
  - 位置-位置架构：master 位置 → slave 位置命令，slave 位置偏差 → master 力
  - 力-位置架构：master 位置 → slave 位置，slave 力传感器 → master 力
- **力缩放**：slave 端力 × 缩放因子 = master 端反馈力
- **阻抗控制**：F_feedback = K_env * (x_slave - x_contact) + B_env * v_slave
- **虚拟夹具**（Virtual Fixture）：在工作空间中设定约束面/线，提供引导力

**典型参数**:
- 力缩放: 0.1-10×（取决于 slave 端力范围）
- 通信延迟 < 10ms 时直接透传；> 50ms 需要波变量/时域直通等稳定性方法
- 位置映射: Touch 工作空间 (160mm) → slave 工作空间 (视臂展而定)

**本项目场景**: Touch 笔控制 Airbot Play 机械臂末端（参见 force-feedback-pen/ 目录）

### 4.2 虚拟夹具辅助

**场景描述**：在遥操作中叠加虚拟约束，引导操作者沿安全路径移动。例如，沿预定切割路线的导轨力。

**力渲染模式**：
- **引导力**（Guidance Virtual Fixture）：到参考路径的距离 × 刚度 = 横向回复力
- **禁止力**（Forbidden Region Virtual Fixture）：接近危险区域时产生强排斥力
- **推荐参数**：引导力 K=300-800 N/m，禁止区域 K=1500-2000 N/m

---

## 五、教育与科研

### 5.1 物理学教育

**场景描述**：学生通过 Touch 笔感受各种物理力场 — 万有引力场、电场、磁场、弹簧振子。

**力渲染模式**：
- **点源引力场**：F = G*M*m / r² * r̂，缩放到可感知范围
- **电场力**：F = q * E(r)，可视化库仑力
- **弹簧振子**：F = -k*x，不同阻尼比体验欠阻尼/临界阻尼/过阻尼
- **离心/科里奥利力**：旋转参考系中的虚拟力

### 5.2 力反馈算法研究

**场景描述**：高校和研究机构使用 Touch/Touch X 作为标准力反馈平台，开发和测试新的力渲染算法。

**典型研究方向**：
- God-Object / Proxy 算法改进
- 变形体碰撞力计算
- 多点接触力渲染
- 纹理渲染新方法
- 时延遥操作稳定性
- 基于学习的力渲染

**原因**: Touch 系列因价格适中（相对 Phantom Premium）、SDK 成熟（OpenHaptics），成为学术界使用最广泛的力反馈设备。

### 5.3 虚拟装配

**场景描述**：在虚拟环境中模拟零件装配过程。用户感受零件之间的碰撞、引导槽的约束力。

**力渲染模式**：
- **碰撞力**：零件之间的弹簧-阻尼接触力
- **卡扣力**（snap-in）：接近正确装配位置时产生吸引力
- **引导力**：沿装配路径的约束力
- **摩擦力**：零件滑入槽中时的摩擦阻力

---

## 六、力渲染模式汇总表

| 力渲染模式 | 物理模型 | 典型参数范围 (Touch X) | 应用场景 |
|-----------|---------|----------------------|---------|
| 弹簧-阻尼接触 | F = -Kx - Bv | K: 50-2200 N/m, B: 0.001-0.5 N·s/m | 所有碰撞场景的基础 |
| 库仑摩擦 | F = μN | μ: 0.05-0.9 | 表面滑动、雕刻 |
| 粘性阻尼 | F = -Bv | B: 0.1-5.0 N·s/m | 液体环境、运动阻尼 |
| 纹理力 | F = A·sin(ωx)·n̂ | A: 0.05-0.5 N, ω: 0.5-5 cyc/mm | 表面质感 |
| 势场梯度 | F = -∇V | 依场强缩放 | 分子力、物理场 |
| 虚拟弹簧 (coupling) | F = K(x_proxy - x_device) | K: 300-1000 N/m | God-Object 算法 |
| 虚拟夹具 | F = K·d_⊥ | K: 300-2000 N/m | 遥操作引导/禁止 |
| 重力场 | F = mg (缩放) | 0-3 N | 物理教育 |
| 正弦振动 | F = A·sin(2πft) | f: 10-300 Hz, A: 0.05-1.0 N | 碰撞振动、警告 |
| 突破力 (breakthrough) | K 突变 | K: 1500→300 N/m 突变 | 骨钻、穿刺 |

---

## 附录：参考资料

1. 3D Systems. "Geomagic Freeform — Haptic-enabled 3D Design." Product Documentation.
2. 3D Systems. "OpenHaptics Toolkit — Developer Guide." Version 3.5.
3. Immersion Corporation. "Haptic Technology for Medical Simulation." White Paper.
4. Ruspini, D.C. et al. "The Haptic Display of Complex Graphical Environments." SIGGRAPH 1997.
5. Zilles, C.B. & Salisbury, J.K. "A Constraint-Based God-Object Method For Haptic Display." IROS 1995.
6. Lin, M.C. & Otaduy, M.A. "Haptic Rendering: Foundations, Algorithms, and Applications." A K Peters, 2008.
7. Stone, R.J. "Haptic Feedback: A Brief History from Telepresence to Virtual Reality." Haptic HCI 2001.
