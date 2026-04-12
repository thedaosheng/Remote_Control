# 交接文档：Touch X 力渲染 → 下一个 Agent

**日期**: 2026-04-09
**来自**: Agent A (Touch 力渲染探索)
**接收**: Agent B (AVP→灵巧手) 或后续集成 Agent

---

## 1. 已验证可用的力渲染方案

### 1.1 在 MuJoCo 中验证通过的 8 种力渲染模式

| 模式 | 验证脚本 | 力计算公式 | 可直接用于遥操作 |
|------|---------|-----------|----------------|
| 刚度墙 | touch_probe_basic.py | F = k × penetration | ✅ 碰撞反馈 |
| 摩擦面 | touch_probe_textures.py | F_t = μ × F_n | ✅ 表面滑动反馈 |
| 弹簧阻尼 | force_visualizer.py | F = -k(r-L₀)r̂ - bv | ✅ 弹性物体 |
| 粘滞力 | force_visualizer.py | F = -ηv | ✅ 液体环境 |
| 磁吸 | force_visualizer.py | F = s/r² × r̂ | ✅ 装配对齐 |
| 重力井 | force_visualizer.py | F = G/r² × r̂ | ✅ 目标引导 |
| 引导槽 | force_visualizer.py | F = -k(y-y_wall) | ✅ 路径约束 |
| 纹理凹凸 | force_visualizer.py | F_n += A sin(2πx/λ) | ⚠️ 仅动觉 |

### 1.2 验证通过的 MuJoCo 材质参数

已在 touch_probe_textures.py 中验证 8 种材质的 MuJoCo 参数组合：
- 冰面 (friction=0.05, solref=0.002)
- 木板 (friction=0.4, solref=0.003)
- 橡胶 (friction=1.5, solref=0.008)
- 海绵 (friction=0.6, solref=0.04)
- 钢铁 (friction=0.3, solref=0.001)
- 果冻 (friction=0.2, solref=0.05)
- 砂纸 (friction=1.8, solref=0.002)
- 瓦楞 (多 box 排列, period=5mm)

---

## 2. 需要真实设备验证的内容

### 2.1 必须用真实 Touch X 验证
- [ ] 力渲染刚度上限：文献值 2.31 N/mm，需实测是否稳定
- [ ] 背驱力实测：标称 0.26N，实际可能因使用年限不同
- [ ] 力输出线性度：大力/小力区间是否线性
- [ ] 高频响应：电机能实际输出多高频率的力振动
- [ ] MuJoCo 参数与真实 Touch 力感的主观一致性评估

### 2.2 需要 OpenHaptics SDK 验证
- [ ] HD API 的实际伺服频率稳定性（标称 1kHz）
- [ ] HLAPI 的力渲染效果与自定义力回调的效果对比
- [ ] 多线程下的力更新延迟
- [ ] 安全限力机制的实际行为

---

## 3. 与 Agent B (AVP→灵巧手) 的接口点

### 3.1 数据流架构

```
Apple Vision Pro
  ↓ 手势关节角 (WebSocket/LiveKit)
遥操作桥节点 (teleop_bridge_node)
  ↓ /joint_commands (Float64MultiArray)
MuJoCo 仿真
  ↓ 碰撞力 (contact forces)
力反馈计算节点 (需要新建)
  ↓ 力向量 [Fx, Fy, Fz]
Touch X 设备
  ↓ 力输出到操作者手部
```

### 3.2 需要新建的接口

1. **MuJoCo → Touch 力反馈桥节点**
   - 输入: MuJoCo 仿真中机械臂末端的接触力
   - 输出: 缩放后的 3DOF 力命令发送到 Touch X
   - 关键: 力缩放比例（MuJoCo 力 vs Touch X 可输出力范围）

2. **坐标系映射**
   - MuJoCo 世界坐标系 → Touch X 工作空间坐标系
   - 需要标定：位置比例、方向对齐
   - Touch X 工作空间仅 160×120×120mm，需要缩放

3. **安全限位**
   - 力输出限幅: 连续力 < 1.75N (Touch X), < 0.88N (Touch)
   - 速度限制: 防止突然的大力输出
   - 工作空间限制: 预留 10mm 边界缓冲

### 3.3 ROS2 话题规划

```
/touch/position        ← Touch X 笔尖位置 (geometry_msgs/Point)
/touch/orientation     ← Touch X 笔杆姿态 (geometry_msgs/Quaternion)
/touch/buttons         ← 按钮状态 (std_msgs/Int32)
/touch/force_command   ← 力反馈命令 (geometry_msgs/Vector3)
/mujoco/contact_force  ← MuJoCo 接触力 (geometry_msgs/WrenchStamped)
```

---

## 4. 关键文件清单

| 文件 | 用途 | 后续 Agent 是否需要 |
|------|------|-------------------|
| `mujoco_sim/20260409-cc-touch_probe_basic.py` | 碰撞力测试基准 | ✅ 参考力计算逻辑 |
| `mujoco_sim/20260409-cc-force_visualizer.py` | 8 种力渲染算法 | ✅ 可直接复用力计算函数 |
| `survey/20260409-cc-parameter_guide.md` | Touch X 参数限制 | ✅ 安全限幅参考 |
| `survey/20260409-cc-capability_matrix.md` | 能力边界分析 | ✅ 设计决策参考 |

---

## 5. 已知风险和建议

1. **力延迟是大敌**: 力渲染延迟 >5ms 会导致不稳定振荡。
   MuJoCo 仿真 → 力计算 → Touch X 输出的总延迟必须 <3ms。

2. **力缩放需调优**: MuJoCo 中的碰撞力可能是 10-100N 量级，
   而 Touch X 连续输出仅 1.75N，需要设计合理的缩放曲线。

3. **不要混淆动觉和触觉**: Touch X 只能提供动觉反馈。
   如果项目后续需要纹理/温度感知，必须加入额外的触觉设备。

4. **FireWire 接口已过时**: Touch X 使用 FireWire/USB，
   确保 NUC11PH 有兼容的端口或转接卡。
