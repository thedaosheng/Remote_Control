# Device A - Linux (被控制端 / 机器人端)

包含两大模块：
1. **视频推流** — ZED Mini 立体相机 → WebRTC 实时推流到 Vision Pro
2. **ROS2 遥操作框架** — 双臂机器人的完整控制链路（Airbot Play G2 × 2 + 舵轮底盘 + 云台）

---

## 目录结构

```
device-a-linux/
├── README.md                          ← 本文件
├── zedmini_webrtc_sender.py           ← GStreamer WebRTC 推流（生产版）
├── zedmini_aiortc_sender.py           ← aiortc 纯 Python 推流（备选）
├── ROS2_SETUP.md                      ← ROS2 环境搭建与配置文档（必读）
└── ros2_ws/
    └── src/
        ├── airbot_msgs/               ← 自定义消息包（6 种消息）
        ├── airbot_description/        ← URDF/xacro + meshes + RViz 配置
        └── airbot_bringup/            ← 系统启动 + WebSocket 桥接 + Mock 测试
```

---

## 视频推流

| 文件 | 说明 |
|------|------|
| `zedmini_webrtc_sender.py` | GStreamer webrtcbin sender (production, v7.3) |
| `zedmini_aiortc_sender.py` | aiortc pure-Python sender (alternative) |

### 依赖

- ZED Mini camera (`/dev/video0`)
- NVIDIA GPU with nvh264enc support
- System Python 3.10+ (`/usr/bin/python3`, NOT conda)
- GStreamer 1.20+ with plugins: `gst-plugins-bad`, `gst-plugins-ugly`, `gstreamer1.0-nice`
- Python packages: `websockets`, `PyGObject` (gi)

### 使用

```bash
/usr/bin/python3 -u zedmini_webrtc_sender.py
```

---

## ROS2 遥操作框架

**详细的环境搭建和配置步骤见 [ROS2_SETUP.md](ROS2_SETUP.md)**

### 快速启动（环境已配置好时）

```bash
cd ros2_ws

# 编译（必须用系统 Python，绕开 miniconda）
export PATH="/usr/bin:$PATH"
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash

# 启动系统（mock 模式，不需要真实硬件）
ros2 launch airbot_bringup system.launch.py mode:=mock

# 另一终端：启动 WebSocket 桥接
ros2 run airbot_bringup teleop_bridge_node

# 另一终端：启动 Mock 数据发布（测试用）
ros2 run airbot_bringup mock_data_publisher
```

### 数据流架构

```
Vision Pro / mock_data_publisher
  ↓ WebSocket JSON (ws://0.0.0.0:9090)
teleop_bridge_node
  ↓ /forward_command_controller/commands (Float64MultiArray)
controller_manager (100Hz, ros2_control)
  ↓ mock_components/GenericSystem (或真实硬件驱动)
joint_state_broadcaster
  ↓ /joint_states (sensor_msgs/JointState)
robot_state_publisher
  ↓ /tf
RViz2 可视化
```

### 关节总览（27 个活动关节）

| 模块 | 关节 | 类型 | 数量 |
|------|------|------|------|
| 左臂 | left_joint1~6 | revolute | 6 |
| 左夹爪 | left_joint7, left_joint8 | prismatic | 2 |
| 右臂 | right_joint1~6 | revolute | 6 |
| 右夹爪 | right_joint7, right_joint8 | prismatic | 2 |
| 升降 | lift_joint | prismatic | 1 |
| 云台 | rentou_joint1 (yaw), rentou_joint2 (pitch) | revolute | 2 |
| 底盘转向 | fl/fr/rl/rr_steer_joint | revolute | 4 |
| 底盘驱动 | fl/fr/rl/rr_drive_joint | continuous | 4 |
