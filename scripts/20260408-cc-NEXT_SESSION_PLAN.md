# MuJoCo 仿真集成规划 — 底盘 + 升降 + 双臂 + Touch 遥控

**创建日期**: 2026-04-08 下午
**目的**: 当前会话上下文可能用完，下一轮 Claude Code 会话读取本文档可 100% 接住继续工作。

---

## 一、已完成 ✅ (今天下午)

### 1. 四舵轮全向底盘 MJCF 模型
- **文件**: `/home/rhz/teleop/scripts/20260408-cc-swerve_chassis.xml`
- **结构**: 1 个 free joint chassis → 4 个 swerve module（每个 2 DOF: steer + drive）
- **关节总数**: 8 个（4 转向 + 4 驱动）
- **关键参数**:
  - 轮子半径 0.06m, 宽 0.04m
  - 轮距: 0.32m × 0.30m (X × Y)
  - 底盘尺寸: 0.42 × 0.40 × 0.08 m, 质量 15kg
- **碰撞设置**: chassis (contype=1, conaffinity=0, 不碰撞任何东西), wheel (contype=2, conaffinity=4, 只和 floor 碰撞), floor (contype=4, conaffinity=2)
- **执行器**: 4 position（转向 kp=200 kv=10）+ 4 velocity（驱动 kv=20）

### 2. 舵轮逆运动学 + 键盘控制
- **文件**: `/home/rhz/teleop/scripts/20260408-cc-swerve_keyboard_control.py`
- **依赖**: pynput (已在 disc env 安装)
- **运行**: `conda activate disc && python 20260408-cc-swerve_keyboard_control.py`

#### 关键技术点
1. **pynput 全局监听**: 不需要聚焦 MuJoCo 窗口，避开 viewer 内置快捷键冲突
2. **X11 自动重复修正**: viewer 默认会生成 press/release 交替序列，用 `KEY_HOLD_TIMEOUT=100ms` + `is_key_held()` 时间戳判断按住状态
3. **舵轮逆运动学**: `v_wheel = (vx - ω*ry, vy + ω*rx)`, `steer = atan2(v.y, v.x)`, `drive = |v|/r`
4. **最短转向优化**: `shortest_steer()` 带 180° 反转+反向驱动
5. **HUD overlay**: 用 `viewer.set_texts()` 实时显示速度/位置/按键状态

#### 控制键位
- W/S = 前进/后退 (vx)
- A/D = 左移/右移 (vy)
- Q/E = 左转/右转 (ω)
- R = 重置位置
- ESC = 退出
- **使用时要把焦点放在终端窗口，不要点 MuJoCo 窗口**！

---

## 二、待办 (下一会话从这里接住)

### Step 1：升降机构集成 ⏫

**目标**: 在当前 MJCF 中加入升降 prismatic joint，用 G/H 键控制。

**修改点**:
1. 在 `20260408-cc-swerve_chassis.xml` 的 chassis_link 中增加 torso_link 子 body，用 prismatic joint 连接:
   ```xml
   <body name="torso_link" pos="0 0 0.05">
     <joint name="lift_joint" type="slide" axis="0 0 1" range="0 0.4"/>
     <inertial pos="0 0 0.3" mass="5" diaginertia="0.5 0.5 0.2"/>
     <geom type="box" size="0.125 0.09 0.275" pos="0 0 0.3" rgba="0.25 0.45 0.85 1"/>
   </body>
   ```
2. 加 position 执行器：`<position name="lift_act" joint="lift_joint" kp="500" kv="30"/>`
3. 控制脚本增加:
   - `target_lift = 0.15`（初始高度）
   - `is_key_held('g')` → `target_lift += 0.3*dt`（上升）
   - `is_key_held('h')` → `target_lift -= 0.3*dt`（下降）
   - clip 到 [0, 0.4]
   - `data.ctrl[8] = target_lift`（注意：舵轮是 0~7，升降是第 8 个）
4. HUD 加一行显示 `lift = X.XX m`

**验证**: 按住 G 上升，按住 H 下降，顶部蒙牌在上下动。

---

### Step 2：双臂集成 🤖🤖

**源文件参考**:
- ROS2 中的 URDF: `/home/rhz/teleop/ros2_ws/src/airbot_description/urdf/airbot_arm.xacro`
- DISCOVERSE 原始 URDF: `/home/rhz/YZL/home/edward/hil-serl/DISCOVERSE-main/models/mjcf/manipulator/airbot_play_force/_play_force.urdf`
- DISCOVERSE 现成 MJCF: `/home/rhz/YZL/home/edward/hil-serl/DISCOVERSE-main/models/mjcf/manipulator/airbot_play_force/`

**方案 A**（推荐）：直接复用 DISCOVERSE 的 airbot_play_force MJCF
- 复制 mesh + xml 到今天的 scripts 目录
- 用 `<include>` 机制合并 chassis + lift + arms
- 位置：left_arm 贴在 torso 顶部 (0, 0.20, 0.575) 附近，right_arm 贴在 (0, -0.20, 0.575)
- 可能的限制：DISCOVERSE 的 mesh 路径可能不能直接用需要调整

**方案 B**：手动从 URDF 写 MJCF
- 从 airbot_arm.xacro 提取几何参数
- 6 个轴关节 + 2 个夹爪 prismatic
- 较费时但干净

**推荐**：先试 A，如果 mesh 路径问题多再走 B。

**调试要点**:
- 双臂初始位姿调到自然下垂状态
- ros2_control 同样用 position 执行器
- 验证顶部上升后双臂跟随上升

---

### Step 3：Touch 力反馈笔 遥控右臂末端 🖐️

**参考代码**: `/home/rhz/teleop/force-feedback-pen/20260328-cc-airbot_force_sim.py`

#### 关键函数（1000 行附近）
```python
# 位置：增量映射 (Touch 笔 mm → 机械臂 m)
relative_pos = pen_pos - self.pen_origin
self.desired_pos = self.pen_pos_to_robot(relative_pos)  # 默认 scale=0.002 m/mm

# 姿态：相对旋转映射
delta_R_pen = R_pen_cur @ self.pen_R0.T          # 笔相对初始姿态的旋转
delta_R_pen = self._scale_rotation(delta_R_pen, self.rot_scale)
delta_R_robot = self.R_map @ delta_R_pen @ self.R_map.T
R_robot_des = delta_R_robot @ self.robot_R0
```

#### 集成路径
1. 从原脚本提取两个核心类/函数：
   - `HapticDriver` (Touch 笔底层驱动)
   - 位置/姿态映射逻辑
2. 在 swerve_keyboard_control.py 基础上加:
   - 独立线程读 Touch 笔位姿（根据原脚本是 1kHz）
   - 笔坐标变化 → 右臂 TCP target
   - 用 MuJoCo IK (mujoco.mj_jacBodyCom + 伪逆) 求右臂 6 个关节角
   - 输出到 right_joint1~6 的 position 执行器
3. **复用原项目的 IK 策略**: 查看 20260328-cc-airbot_force_sim.py 中是怎么算逆解的

#### 使用场景
- 左手：键盘 WASD/QE/GH 控制底盘和升降
- 右手：Touch 笔控制右臂末端
- 未来可以加第二支 Touch 笔控制左臂

---

## 三、项目文件路径总览

### 今天下午创建的文件
```
/home/rhz/teleop/scripts/
  20260408-cc-swerve_chassis.xml          ← 四舵轮底盘 MJCF (待扩展)
  20260408-cc-swerve_keyboard_control.py  ← 键盘控制主脚本 (待扩展)
  20260408-cc-swerve_auto_test.py         ← 自动测试脚本 (不需要键盘)
  20260408-cc-NEXT_SESSION_PLAN.md        ← 本规划文档
```

### 参考项目文件
```
/home/rhz/teleop/force-feedback-pen/
  20260328-cc-airbot_force_sim.py         ← Touch 笔 + MuJoCo 机械臂遥控参考
  20260328-cc-airbot_mujoco_scene.xml     ← 原机械臂 MJCF 场景
  20260328-cc-haptic_driver.py            ← Touch 笔底层驱动
  20260328-cc-pen_keyboard_client.py      ← 笔按钮 → 键盘事件

/home/rhz/teleop/ros2_ws/src/airbot_description/
  urdf/airbot_arm.xacro                   ← 双臂 URDF (可参考参数)
  meshes/                                  ← STL 文件 (可复用)
```

### 环境
```
conda env: disc
mujoco: 3.4.0
python: 3.10
pynput: 1.8.1 (已安装)
启动脚本主函数: 必须用 /home/rhz/miniconda3/envs/disc/bin/python (不能用 /usr/bin/python3 因为后者没 mujoco)
```

---

## 四、重要结论/经验教训

1. **MuJoCo viewer 键盘 callback 只在按下瞬间触发一次**，不能用来检测按住状态。必须用 pynput 全局监听。

2. **X11 自动重复会交替生成 press/release**，不是真正的按住事件。解决方案：记录最近一次 press 的时间戳，100ms 内算"按住中"。

3. **轮子不能和底盘默认碰撞**，要用 contype/conaffinity 位掩码隔离，否则轮子被卡在底盘里不能转。

4. **舵轮反转优化**：如果目标转向角和当前转向角差绝对值 > 90°，则反转 180° 并反向驱动。避免不必要的大幅转向。

5. **MuJoCo viewer 窗口管理**：在这台机上是 WID 0x5400007 / 0x5600007 之类。用 `xprop` 取得窗口名包含 'MuJoCo' 的来定位。

6. **miniconda 污染**：ROS2 需要 /usr/bin/python3，但 mujoco 需要 disc env 的 python3.10。不要搞错。

---

## 五、验证检查表

### 下一会话开始前需要确认:
- [ ] `conda activate disc` 成功
- [ ] `python -c "import mujoco; print(mujoco.__version__)"` 输出 3.4.0
- [ ] `python -c "import pynput"` 不报错
- [ ] `/home/rhz/teleop/scripts/20260408-cc-swerve_chassis.xml` 存在
- [ ] `/home/rhz/teleop/scripts/20260408-cc-swerve_keyboard_control.py` 存在
- [ ] DISPLAY=:0 可访问
- [ ] 运行现有脚本，验证舵轮控制仍然 OK

### 下一会话任务优先级:
1. 首先加入升降机构 (Step 1) — 最简单，不需要额外依赖
2. 然后加入双臂 (Step 2) — 需要复制 mesh 文件和调整 IK
3. 最后 Touch 笔遥控 (Step 3) — 需要 Touch 设备连接和底层驱动
