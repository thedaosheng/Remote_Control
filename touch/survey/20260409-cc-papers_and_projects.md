# Touch X 相关开源项目与关键论文

**调研日期**: 2026-04-09
**目的**: 整理力反馈领域的开源资源和学术文献，为项目开发提供参考

---

## 一、核心开源项目

### 1.1 CHAI3D

| 项目信息 | 详情 |
|---------|------|
| **GitHub** | https://github.com/chai3d/chai3d |
| **Stars** | ~700+ |
| **最后更新** | 持续维护 (2024+) |
| **许可证** | BSD 3-Clause |
| **主要功能** | 跨平台力反馈渲染框架；支持多种力反馈设备；内置碰撞检测、力渲染算法、OpenGL 可视化 |
| **支持 Touch X** | ✅ 完整支持 (通过 OpenHaptics 驱动层) |

**简介**：CHAI3D (Computer Haptics and Active Interfaces 3D) 是力反馈领域最知名的开源框架，由斯坦福大学 Conti 教授团队开发。提供从底层设备驱动到高层场景渲染的完整栈。

**核心功能**：
- 多设备抽象层：支持 Touch/Touch X、Phantom Premium、Novint Falcon、Force Dimension 等
- 碰撞检测：AABB-tree、God-Object (proxy) 算法
- 力渲染：弹簧-阻尼、摩擦、粘性、纹理、磁性吸附
- 变形体：有限元 (FEM) 软体碰撞
- OpenGL 3D 渲染引擎
- 丰富的示例程序 (~30+ examples)

**对本项目价值**：可参考其 God-Object 碰撞力渲染实现和设备抽象层设计。

---

### 1.2 OpenHaptics (3D Systems 官方 SDK)

| 项目信息 | 详情 |
|---------|------|
| **官网** | https://www.3dsystems.com/haptics-devices/openhaptics |
| **GitHub** | 非开源，需注册下载 (免费) |
| **最后更新** | 2023+ |
| **许可证** | 商业免费（开发用） |
| **主要功能** | Touch 系列设备的官方 SDK；HD API (底层) + HL API (高层) |
| **支持 Touch X** | ✅ 官方驱动，最完整支持 |

**API 层次**：
- **HDAPI (Haptic Device API)**：底层 API，直接读写设备位置/力，1kHz servo loop 回调
- **HLAPI (Haptic Library API)**：高层 API，类似 OpenGL 的声明式接口，自动碰撞检测和力渲染
- **HDU/HLU**：工具类库

**关键头文件**：
```c
#include <HD/hd.h>      // HDAPI 核心
#include <HDU/hduVector.h>  // 向量工具
#include <HL/hl.h>      // HLAPI 核心
```

---

### 1.3 H3DAPI / H3D

| 项目信息 | 详情 |
|---------|------|
| **GitHub** | https://github.com/SenseGraphics/H3DAPI |
| **Stars** | ~100+ |
| **最后更新** | 2023+ |
| **许可证** | GPL v2 |
| **主要功能** | 基于 X3D/VRML 场景图的力反馈渲染引擎；支持多设备和多点触觉 |
| **支持 Touch X** | ✅ 通过 OpenHaptics 后端 |

**特点**：使用 X3D 场景描述语言，适合 VR 集成。提供 Python 绑定。

---

### 1.4 Haption/ROS 力反馈集成

#### phantom_omni (ROS1)

| 项目信息 | 详情 |
|---------|------|
| **GitHub** | https://github.com/danepowell/phantom_omni |
| **Stars** | ~60+ |
| **最后更新** | ~2020 |
| **许可证** | BSD |
| **主要功能** | Phantom Omni (Touch) 的 ROS1 驱动节点；发布 /phantom/pose, /phantom/button; 订阅 /phantom/force_feedback |
| **支持 Touch X** | ✅ (Phantom Omni 兼容) |

#### ros2_haptic_device (ROS2 社区)

| 项目信息 | 详情 |
|---------|------|
| **搜索关键词** | "haptic device ros2" site:github.com |
| **状态** | ROS2 生态中 Touch 驱动较少，多数仍是 ROS1 |
| **替代方案** | 自行封装 HDAPI 调用为 ROS2 节点（本项目 force-feedback-pen/ 中已有基础实现） |

---

### 1.5 Force Dimension SDK (竞品参考)

| 项目信息 | 详情 |
|---------|------|
| **GitHub** | https://github.com/nickhudspeth/haptic-device-examples (社区) |
| **Stars** | ~20+ |
| **主要功能** | Force Dimension omega/sigma 系列设备驱动示例；C/C++ |
| **支持 Touch X** | ❌ (Force Dimension 设备专用) |

**参考价值**：Force Dimension 的 sigma.7 (7DOF 力反馈) 与 Touch X 在遥操作场景中常做对比，其 API 设计可参考。

---

### 1.6 GraspIt! (抓取力学仿真)

| 项目信息 | 详情 |
|---------|------|
| **GitHub** | https://github.com/graspit-simulator/graspit |
| **Stars** | ~500+ |
| **最后更新** | 2023+ |
| **许可证** | GPL v3 |
| **主要功能** | 机器人抓取规划仿真器；碰撞检测 + 抓取质量评估；可集成力反馈设备 |
| **支持 Touch X** | ✅ (可选，通过 CHAI3D 后端) |

---

### 1.7 Bullet Physics (力反馈 VR 示例)

| 项目信息 | 详情 |
|---------|------|
| **GitHub** | https://github.com/bulletphysics/bullet3 |
| **Stars** | ~12000+ |
| **最后更新** | 持续维护 |
| **许可证** | Zlib |
| **主要功能** | 物理引擎；含 VR + 力反馈示例（examples/SharedMemory/PhysicsServerExample.cpp） |
| **支持 Touch X** | ⚠️ 部分支持 (通过 OpenHaptics 桥接，非原生) |

---

### 1.8 其他相关 GitHub 项目

| 项目 | URL | Stars | 说明 |
|------|-----|-------|------|
| haptic-plugin (Unity) | github.com/DigitalHaptics/HapticPlugin | ~50+ | Unity 力反馈插件，支持 Touch |
| OpenHaptics-Examples | github.com/jnz/OpenHaptics | ~30+ | OpenHaptics 示例代码集合 |
| haptic_teleoperation | 搜索: "haptic teleoperation" site:github.com | 多个 | 各种双边遥操作实现 |
| iMSTK | github.com/Kitware/iMSTK | ~300+ | Kitware 交互式医疗仿真工具包，支持力反馈 |
| SOFA Framework | github.com/sofa-framework/sofa | ~800+ | 软体物理仿真框架，含力反馈模块 |

---

## 二、关键学术论文

### 2.1 力反馈基础理论

#### [1] Salisbury, J.K. & Srinivasan, M.A. "Phantom-Based Haptic Interaction with Virtual Objects"

| 信息 | 详情 |
|------|------|
| **发表** | IEEE Computer Graphics and Applications, 1997 |
| **被引** | ~1500+ |
| **核心贡献** | 首次系统性描述 Phantom 设备的设计原理和应用场景；提出了力反馈交互的基本框架 |
| **关键内容** | (1) Phantom 机械设计：编码器+电机+连杆 (2) 力渲染基本算法：弹簧力模型 (3) 虚拟探针概念 (4) 延迟、采样率对稳定性的影响 |
| **与本项目关联** | Touch X 是 Phantom 系列的直接后继，本文提供的设计哲学和交互模型直接适用 |

#### [2] Ruspini, D.C., Kolarov, K. & Khatib, O. "The Haptic Display of Complex Graphical Environments"

| 信息 | 详情 |
|------|------|
| **发表** | ACM SIGGRAPH 1997 |
| **被引** | ~800+ |
| **核心贡献** | 提出了 **Haptic Display** 的概念框架；解决了复杂几何体的力反馈渲染问题；提出了 proxy-based 渲染的改进方法 |
| **关键内容** | (1) 多物体碰撞检测的层次化方法 (2) 接触点力渲染 (3) 多点接触处理 (4) 软约束与硬约束的统一 (5) 性能优化确保 1kHz 更新 |
| **与本项目关联** | 当遥操作中机械臂接触物体时，需要将接触力渲染给操作者；本文的力渲染架构是基础参考 |

#### [3] Zilles, C.B. & Salisbury, J.K. "A Constraint-Based God-Object Method For Haptic Display"

| 信息 | 详情 |
|------|------|
| **发表** | IEEE/RSJ IROS 1995 |
| **被引** | ~1200+ |
| **核心贡献** | 提出 **God-Object (代理点) 算法** — 力反馈渲染的最基础算法之一 |
| **关键内容** | (1) 设备端点（HIP, Haptic Interface Point）可能穿透虚拟表面 (2) God-Object (proxy) 是 HIP 在表面外侧的最近投影点 (3) 力 = K × (proxy - HIP)，方向始终"推出"表面 (4) 基于约束优化求解 proxy 位置 |
| **算法伪代码** |
```
每个 servo loop (1kHz):
  1. 读取 HIP 位置 (设备端点)
  2. 如果 HIP 在物体外部:
       proxy = HIP, force = 0
  3. 如果 HIP 穿透物体:
       proxy = 最近表面点 (约束优化)
       force = K * (proxy - HIP) + B * (v_proxy - v_HIP)
  4. 输出力到设备
```
| **与本项目关联** | MuJoCo 碰撞检测已提供穿透深度和法向量，可直接用类似 God-Object 的思路计算反馈力 |

---

### 2.2 系统稳定性

#### [4] Colgate, J.E. & Brown, J.M. "Factors Affecting the Z-Width of a Haptic Display"

| 信息 | 详情 |
|------|------|
| **发表** | IEEE ICRA 1994 |
| **被引** | ~600+ |
| **核心贡献** | 提出 **Z-Width** 概念 — 力反馈设备可稳定渲染的阻抗范围 |
| **关键公式** | K_max = 2B/T (B=设备阻尼, T=采样周期) |
| **实际意义** | Touch X: B≈1.155 N·s/m, T=1ms → K_max ≈ 2310 N/m |
| **与本项目关联** | 设定力渲染刚度时不能超过此上限，否则系统震荡 |

#### [5] Adams, R.J. & Hannaford, B. "Stable Haptic Interaction with Virtual Environments"

| 信息 | 详情 |
|------|------|
| **发表** | IEEE Transactions on Robotics and Automation, 1999 |
| **被引** | ~800+ |
| **核心贡献** | 分析了离散时间采样对力反馈稳定性的影响；提出了被动性 (passivity) 分析方法 |
| **关键内容** | (1) 能量观察器方法 (2) 被动性约束下的最大刚度/阻尼 (3) 采样率与稳定性的定量关系 |

---

### 2.3 遥操作

#### [6] Lawrence, D.A. "Stability and Transparency in Bilateral Teleoperation"

| 信息 | 详情 |
|------|------|
| **发表** | IEEE Transactions on Robotics and Automation, 1993 |
| **被引** | ~2000+ |
| **核心贡献** | 建立了双边遥操作的 **稳定性-透明性权衡** 理论框架 |
| **关键概念** | (1) 透明性 = 从端阻抗完美传递到主端 (2) 稳定性 vs 透明性存在根本矛盾 (3) 四通道架构分析 |
| **与本项目关联** | Touch 笔控制 Airbot 机械臂属于双边遥操作，力反馈的增益设定需参考此理论 |

#### [7] Niemeyer, G. & Slotine, J.J.E. "Stable Adaptive Teleoperation"

| 信息 | 详情 |
|------|------|
| **发表** | IEEE Journal of Oceanic Engineering, 1991; 后续 IEEE TRO 2004 |
| **被引** | ~1500+ |
| **核心贡献** | 提出 **波变量 (Wave Variable)** 方法处理时延遥操作中的稳定性问题 |
| **与本项目关联** | 如果 Touch 笔和 Airbot 机械臂之间有网络延迟（如远程遥操作），需要此方法保证稳定性 |

---

### 2.4 力渲染算法

#### [8] Salisbury, J.K., Conti, F. & Barbagli, F. "Haptic Rendering: Introductory Concepts"

| 信息 | 详情 |
|------|------|
| **发表** | IEEE Computer Graphics and Applications, 24(2), 2004 |
| **被引** | ~500+ |
| **核心贡献** | 力渲染领域最佳综述之一，覆盖从基础到高级的全部主题 |
| **内容覆盖** | (1) 碰撞检测 (2) 力响应计算 (3) 控制与稳定性 (4) 纹理渲染 (5) 变形体 (6) 摩擦 |

#### [9] Lin, M.C. & Otaduy, M.A. "Haptic Rendering: Foundations, Algorithms, and Applications"

| 信息 | 详情 |
|------|------|
| **出版** | A K Peters/CRC Press, 2008 |
| **类型** | 教材/专著 |
| **核心贡献** | 力反馈渲染领域最全面的教科书；覆盖理论、算法和实现 |

#### [10] Ho, C., Basdogan, C. & Srinivasan, M.A. "Efficient Point-Based Rendering Techniques for Haptic Display of Virtual Objects"

| 信息 | 详情 |
|------|------|
| **发表** | Presence: Teleoperators and Virtual Environments, 1999 |
| **核心贡献** | 基于点的高效力渲染方法，减少碰撞检测计算量 |

---

### 2.5 医疗应用

#### [11] Okamura, A.M. "Haptic Feedback in Robot-Assisted Minimally Invasive Surgery"

| 信息 | 详情 |
|------|------|
| **发表** | Current Opinion in Urology, 2009 |
| **被引** | ~800+ |
| **核心贡献** | 综述了力反馈在微创手术机器人中的作用和挑战 |

#### [12] Kuchenbecker, K.J. et al. "Verro Touch: High-Frequency Acceleration Feedback for Telerobotic Surgery"

| 信息 | 详情 |
|------|------|
| **发表** | Haptics: Generating and Perceiving Tangible Sensations, 2010 |
| **核心贡献** | 提出用高频加速度信号增强遥操作中的接触反馈，弥补低频力反馈的不足 |

---

## 三、论文-项目-场景交叉索引

| 论文/项目 | 碰撞渲染 | 纹理 | 摩擦 | 变形体 | 遥操作 | 医疗 | 支持 Touch X |
|-----------|---------|------|------|--------|--------|------|-------------|
| CHAI3D | ✅ | ✅ | ✅ | ✅ | ⚠️ | ⚠️ | ✅ |
| OpenHaptics | ✅ | ✅ | ✅ | ❌ | ❌ | ❌ | ✅ |
| H3DAPI | ✅ | ✅ | ✅ | ✅ | ❌ | ⚠️ | ✅ |
| iMSTK | ✅ | ⚠️ | ✅ | ✅ | ❌ | ✅ | ✅ |
| SOFA | ✅ | ⚠️ | ✅ | ✅ | ⚠️ | ✅ | ✅ |
| GraspIt! | ✅ | ❌ | ✅ | ❌ | ❌ | ❌ | ⚠️ |
| Bullet3 | ✅ | ❌ | ✅ | ✅ | ❌ | ❌ | ⚠️ |
| God-Object [3] | ✅ | ❌ | ⚠️ | ❌ | — | — | — |
| Z-Width [4] | ✅ | — | — | — | — | — | — |
| Wave Variable [7] | — | — | — | — | ✅ | — | — |

图例：✅ 核心功能 | ⚠️ 部分支持/可扩展 | ❌ 不支持 | — 不适用

---

## 四、对本项目的推荐优先级

### 立即可用（高优先级）

1. **CHAI3D** — 最成熟的开源力渲染框架，如果需要在 C++ 层实现复杂力渲染可直接用
2. **OpenHaptics HDAPI** — 已在 force-feedback-pen/ 中通过串口协议间接使用，如需更底层控制可直接调用
3. **God-Object 论文 [3]** — 碰撞力渲染的基础算法，MuJoCo 碰撞检测 + God-Object 力计算是最直接的路径

### 中期参考（中优先级）

4. **Z-Width 理论 [4]** — 调参时用来计算安全刚度上限
5. **Lawrence 遥操作理论 [6]** — 设定力反馈增益时参考稳定性-透明性权衡
6. **iMSTK** — 如果需要软体碰撞力渲染（如抓取柔性物体）

### 长期研究（低优先级）

7. **Wave Variable [7]** — 仅在有网络延迟的远程遥操作场景需要
8. **SOFA Framework** — 如果需要精确的有限元软体仿真
9. **VerroTouch [12]** — 高频振动增强方案，需额外硬件

---

## 附录：搜索关键词汇总

以下关键词可在 Google Scholar / GitHub / IEEE Xplore / ACM DL 中搜索更多相关资源：

```
# GitHub 搜索
"haptic rendering" language:c++ stars:>10
"openhaptics" language:c++
"phantom omni" ros
"chai3d" example
"force feedback" teleoperation
"touch haptic" driver linux

# 学术搜索
"haptic rendering" survey
"god object" haptic
"proxy based" haptic rendering
"bilateral teleoperation" stability transparency
"z-width" haptic display
"haptic texture rendering"
"deformable haptic" rendering
"surgical simulation" haptic feedback
```
