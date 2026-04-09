# 四舵轮全向底盘 — MuJoCo 仿真控制 (2026-04-09)

全向移动底盘的 MuJoCo 仿真实现，包含完整的舵轮逆运动学、键盘/Touch 笔遥操作、阻抗控制双臂。

## 架构

```
键盘 / 摇杆 / ROS
    ↓ (vx, vy, omega)  ← 连续模拟量，支持任意方向向量合成
 slew rate limiter      ← 加速度限幅（平滑过渡）
    ↓
 swerve_ik()            ← FRC/WPILib 标准 IK 矩阵
    ↓
 optimize + desaturate  ← >90° 翻转优化 + 轮速归一化
    ↓
 MuJoCo actuators
    ├── ctrl[0:3]  → chassis velocity actuator (直接驱动 X/Y/Yaw)
    ├── ctrl[3:7]  → steer position actuator (视觉同步)
    └── ctrl[7:11] → drive velocity actuator (视觉同步)
```

### 底盘驱动方案

**3DOF 直接驱动**（非接触摩擦传动）：
- chassis_x / chassis_y (slide) + chassis_yaw (hinge) 三个关节
- velocity actuator 直接控制，kv=200，响应快速
- 轮子纯视觉（contype=0），转向和滚动仅做动画同步
- 原因：MuJoCo 球体/圆柱与地面接触无法可靠传递横向驱动力

### 执行器索引表 (共 28 个)

| 索引 | 名称 | 类型 | 功能 |
|------|------|------|------|
| 0-2 | chassis_vx/vy/omega | velocity | 底盘 X/Y/Yaw 速度 |
| 3-6 | fl/fr/rl/rr_steer | position | 舵轮转向（视觉） |
| 7-10 | fl/fr/rl/rr_drive | velocity | 轮子转速（视觉） |
| 11 | lift | position | 升降 0~0.8m |
| 12-17 | left_j1-j6 | position | 左臂关节 |
| 18-23 | right_j1-j6 | motor | 右臂力矩控制 |
| 24-26 | head_yaw/pitch/stem | position | 云台 |
| 27 | right_gripper | position | 右夹爪 |

## 接入全身运动学 / ROS

底盘控制只需要写 3 个浮点数：

```python
data.ctrl[0] = vx      # m/s，前后
data.ctrl[1] = vy      # m/s，横移
data.ctrl[2] = omega   # rad/s，旋转
```

支持任意连续值输入（摇杆模拟量、路径规划输出、ROS cmd_vel 等）。
swerve_ik 会自动分解到 4 轮并做视觉同步。

### ROS 接入示例

```python
# 订阅 /cmd_vel → 写入 MuJoCo ctrl
def cmd_vel_callback(msg):
    data.ctrl[0] = msg.linear.x   # vx
    data.ctrl[1] = msg.linear.y   # vy
    data.ctrl[2] = msg.angular.z  # omega
```

## 运行

```bash
# 环境要求: conda disc (mujoco + pynput + numpy)
conda activate disc
python /home/rhz/teleop/RemoteControl/swerve_chassis/20260408-cc-swerve_keyboard_control.py

# 禁用 Touch 笔
ENABLE_TOUCH=0 python /home/rhz/teleop/RemoteControl/swerve_chassis/20260408-cc-swerve_keyboard_control.py
```

### 操控按键

| 按键 | 功能 |
|------|------|
| W/S | 前后 |
| A/D | 左右横移 |
| Q/E | 原地旋转 |
| G/H | 升降 升/降 |
| I/K | 云台俯仰 |
| J/L | 云台偏航 |
| T/Y | 云台高度 |
| F | 第一人称/第三人称切换 |
| R | 重置 |
| ESC | 退出 |
| Touch 白色按钮 | 校准（右臂开始跟随） |
| Touch 灰色按钮 | 夹爪开/合 |

## 关键参数

- 底盘速度: MAX_VX=1.0 m/s, MAX_VY=1.0 m/s, MAX_OMEGA=2.0 rad/s
- 底盘加速度限幅: 3.0 m/s², 6.0 rad/s²
- 轮距: 0.32m × 0.30m (fl/fr/rl/rr)
- 轮半径: 0.06m
- 升降行程: 0~0.8m
- 右臂阻抗: Kp=5000 N/m, Kd=150 Ns/m
- 仿真步长: 0.002s (500Hz)

## 文件说明

| 文件 | 说明 |
|------|------|
| `20260408-cc-swerve_chassis.xml` | MuJoCo MJCF 模型（底盘+双臂+云台+桌面+物体） |
| `20260408-cc-swerve_keyboard_control.py` | 控制脚本（IK + 阻抗 + Touch + 键盘） |
| `meshes/airbot/` | Airbot Play G2 机械臂 STL |
| `meshes/parts/` | HZY 零件 STL（桌上物体） |
