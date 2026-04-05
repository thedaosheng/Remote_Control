# ROS2 遥操作框架 — 环境搭建与配置文档

本文档描述如何从零搭建 Airbot Play G2 双臂遥操作机器人的 ROS2 框架。
遵循本文档的步骤，Claude Code 或开发者可以 100% 复刻整个环境。

---

## 1. 系统要求

| 项目 | 要求 |
|------|------|
| OS | Ubuntu 22.04 LTS |
| ROS2 | Humble Hawksbill (apt 安装) |
| Python | 系统 Python 3.10 (`/usr/bin/python3`) |
| GPU | NVIDIA (用于 RViz2 渲染，非必须) |
| 硬件 | Airbot Play G2 双臂（mock 模式不需要） |

### 关键警告：miniconda 污染

如果系统装了 miniconda/anaconda，它的 Python 3.13 会覆盖系统 Python 3.10，导致 ROS2 编译和运行失败。

**修复方法：**

```bash
# 方法1：禁用 conda 自动激活
conda config --set auto_activate_base false

# 方法2：在 ~/.bashrc 末尾加（推荐）
if [ -z "$CONDA_DEFAULT_ENV" ] || [ "$CONDA_DEFAULT_ENV" = "base" ]; then
    export PATH="/usr/bin:$PATH"
fi

# 方法3：编译时手动指定
export PATH="/usr/bin:$PATH"
```

验证：`which python3` 应该输出 `/usr/bin/python3`

---

## 2. 安装 ROS2 Humble

```bash
# 如果还没装 ROS2
sudo apt update
sudo apt install -y ros-humble-desktop

# 必需的额外包
sudo apt install -y \
  ros-humble-xacro \
  ros-humble-joint-state-publisher-gui \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-controller-manager \
  ros-humble-forward-command-controller \
  ros-humble-joint-state-broadcaster \
  ros-humble-hardware-interface \
  ros-humble-ros2controlcli \
  python3-colcon-common-extensions \
  ros-humble-rqt-graph

# Python 依赖
pip3 install catkin_pkg lark websockets numpy
```

---

## 3. 编译工作空间

```bash
cd <repo>/device-a-linux/ros2_ws

# 必须用系统 Python！
export PATH="/usr/bin:$PATH"
source /opt/ros/humble/setup.bash

# 编译所有包（顺序：先 msgs，再 description，最后 bringup）
colcon build
source install/setup.bash
```

如果遇到 `No module named 'em'` 错误：
```bash
# 完全隔离 miniconda 编译
bash --norc --noprofile -c '
  export PATH="/usr/bin:/usr/local/bin:$PATH"
  source /opt/ros/humble/setup.bash
  colcon build
'
```

---

## 4. Package 详解

### 4.1 airbot_msgs — 自定义消息包

| 消息 | 字段 | 用途 |
|------|------|------|
| `ArmCommand.msg` | `float64[] positions`, `float64[] velocities`, `string mode` | 双臂关节指令 |
| `EEPose.msg` | `PoseStamped left`, `PoseStamped right` | 末端位姿 |
| `GripperState.msg` | `float64 left/right_aperture`, `float64 left/right_force` | 夹爪状态 |
| `TactileData.msg` | `Image left_raw`, `Image right_raw` | 触觉传感器 |
| `SystemMode.msg` | `string mode` | 系统模式 |
| `EpisodeControl.msg` | `bool start/end`, `string label/task_name` | 数据采集控制 |

### 4.2 airbot_description — 机器人描述

#### URDF 结构

```
world
├── chassis_link（底盘，四舵轮全向驱动）
│   ├── fl_steer_joint → fl_steer_link → fl_drive_joint → fl_wheel_link
│   ├── fr_steer_joint → ...
│   ├── rl_steer_joint → ...
│   └── rr_steer_joint → ...
├── lift_joint（升降，prismatic 0~0.4m）
│   └── torso_link（躯干）
│       ├── left_base_link → joint1~6 → joint7(夹爪左指) → joint8(夹爪右指)
│       ├── right_base_link → joint1~6 → joint7 → joint8
│       └── rentou_base_link → rentou_joint1(yaw) → rentou_joint2(pitch)
```

#### Xacro 文件

| 文件 | 说明 |
|------|------|
| `airbot_arm.xacro` | 单臂宏（6DOF + G2 夹爪双指），参数: prefix, parent_link, xyz, rpy |
| `airbot_dual_arm.xacro` | 主文件：双臂 + 底盘 + 升降 + 云台 + ros2_control |
| `chassis.xacro` | 四舵轮底盘宏（每轮 steer + drive 两个关节） |
| `rentou.xacro` | 2-DOF 云台宏（yaw Z轴 + pitch X轴） |
| `rentou_debug.xacro` | 云台单独调试文件（三色区分） |

#### ros2_control 接口

hardware_plugin 通过 xacro arg 参数化：

| mode | plugin | 场景 |
|------|--------|------|
| `mock` | `mock_components/GenericSystem` | 纯软件验证 |
| `sim_disc` | `gz_ros2_control/GazeboSimSystem` | DISCOVERSE 仿真 |
| `airbot` | `airbot_hardware/AirbotSystem` | 官方 SDK 驱动 |
| `custom` | `custom_hardware/CustomCANSystem` | 自定义 CAN 驱动 |

#### Mesh 文件

STL/OBJ 文件通过 Git LFS 管理（单个文件最大 40MB）。

| 来源 | 文件 | 说明 |
|------|------|------|
| Airbot Play G2 | `base_link.STL` ~ `link6.STL`, `g2_*.STL`, `eef_connect_base_link.STL` | 机械臂 11 个 link |
| SolidWorks 导出 | `rentou_base_link.STL`, `rentou_link1.STL`, `rentou_link2.STL` | 云台 3 个 link |
| DISCOVERSE MMK2 | `chassis/*.STL`, `chassis/*.obj` | 底盘/轮子（参考用） |

**注意**：rentou STL 文件已经过坐标修正（消除 SolidWorks 装配体偏移 + 旋转校正）。
如果从原始 `rentou.zip` 重新导出，需要重新执行修正脚本。

### 4.3 airbot_bringup — 系统启动

#### Launch 文件

```bash
# 完整系统（ros2_control + 控制器）
ros2 launch airbot_bringup system.launch.py mode:=mock

# 可视化调试（joint_state_publisher_gui 滑条）
ros2 launch airbot_description display.launch.py

# 云台单独调试
ros2 launch airbot_description rentou_debug.launch.py
```

#### 节点

| 节点 | 入口 | 功能 |
|------|------|------|
| `teleop_bridge_node` | `ros2 run airbot_bringup teleop_bridge_node` | WebSocket (ws://0.0.0.0:9090) → ROS2 话题 |
| `mock_data_publisher` | `ros2 run airbot_bringup mock_data_publisher` | 正弦波关节角 + 假感知数据（测试用） |

#### WebSocket 协议

teleop_bridge_node 接收的 JSON 格式：

```json
{
  "positions": [23个float, 左臂8+右臂8+升降1+云台2+转向4],
  "mode": "teleop"
}
```

#### 控制器配置（controllers.yaml）

```yaml
forward_command_controller:
  ros__parameters:
    joints:
      - left_joint1 ~ left_joint8    # 左臂 8 关节
      - right_joint1 ~ right_joint8  # 右臂 8 关节
      - lift_joint                   # 升降
      - rentou_joint1, rentou_joint2  # 云台
      - fl/fr/rl/rr_steer_joint      # 转向
    interface_name: position
```

---

## 5. Topic 总览

| Topic | 类型 | 发布者 | 频率 |
|-------|------|--------|------|
| `/joint_states` | `sensor_msgs/JointState` | joint_state_broadcaster | 100Hz |
| `/forward_command_controller/commands` | `std_msgs/Float64MultiArray` | teleop_bridge_node | 50Hz |
| `/robot/mode` | `std_msgs/String` | teleop_bridge_node | 50Hz |
| `/camera/stereo/left/image_raw` | `sensor_msgs/Image` | mock_data_publisher | 50Hz |
| `/proprioception/joints` | `sensor_msgs/JointState` | mock_data_publisher | 50Hz |
| `/proprioception/ee_pose_left` | `geometry_msgs/PoseStamped` | mock_data_publisher | 50Hz |
| `/proprioception/gripper_left` | `airbot_msgs/GripperState` | mock_data_publisher | 50Hz |

---

## 6. 验证步骤

```bash
# 1. 启动系统
ros2 launch airbot_bringup system.launch.py mode:=mock &
sleep 8

# 2. 启动节点
ros2 run airbot_bringup teleop_bridge_node &
ros2 run airbot_bringup mock_data_publisher &
sleep 3

# 3. 验证控制器
ros2 control list_controllers
# 期望: joint_state_broadcaster [active], forward_command_controller [active]

# 4. 验证关节数据
ros2 topic echo /joint_states --once | grep "^name:" -A 30 | grep "^-" | wc -l
# 期望: 27（16臂 + 1升降 + 2云台 + 4转向 + 4驱动）

# 5. 可视化
ros2 launch airbot_description display.launch.py
# → RViz2 + joint_state_publisher_gui 滑条
```

或者使用一键验证脚本：
```bash
bash ros2_ws/src/airbot_bringup/scripts/20260405-cc-verify_mvp.sh
```

---

## 7. 已知问题与待办

### 已知问题

1. **miniconda 污染**：每次新终端需确认 `which python3` → `/usr/bin/python3`
2. **PREEMPT_RT 未配置**：controller_manager 无法设置 FIFO 实时调度（正式部署需要）
3. **云台 STL 朝向**：SolidWorks 导出坐标系和 URDF 不一致，已做顶点修正但可能不完美，后续需从 STEP 重新导出

### 待办

- [ ] 接入 DISCOVERSE Docker 仿真（mode=sim_disc）
- [ ] 实现 AirbotHardware 插件（真实电机 CAN 驱动）
- [ ] MoveIt2 碰撞检测 + moveit_servo 实时遥操作
- [ ] Vision Pro visionOS App 对接 WebSocket
- [ ] 数据采集 pipeline（EpisodeControl + rosbag2）
- [ ] PREEMPT_RT 实时内核配置（目标 500Hz 控制环）
- [ ] 底盘全向运动控制器（swerve drive kinematics）
