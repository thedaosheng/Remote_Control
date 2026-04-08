# Remote Control — 全链路远程遥操作系统

Vision Pro 头显遥操作 → 云端 LiveKit 双向通信 → Linux 工控机 → 达妙电机/机械臂控制。
含完整 MuJoCo 仿真环境（舵轮底盘 + 升降 + 双臂 + 云台）和 3D Systems Touch 力反馈笔集成。

```
┌─────────────────┐    ┌────────────────┐    ┌──────────────────┐
│  Linux 工控机   │    │  阿里云服务器  │    │  Apple Vision Pro│
│  (设备 A 端)    │◄──►│  LiveKit Server│◄──►│  (控制端)         │
│                 │    │                │    │                  │
│  ZED Mini 双目  │    │  39.102.113.104│    │  visionOS 26     │
│  GStreamer 采集 │    │  端口: 7880    │    │  LiveKit Swift   │
│  LiveKit Python │    │  TURN: 3478    │    │  SDK 2.12.1      │
│  NVENC H264     │    │                │    │                  │
└─────────────────┘    └────────────────┘    └──────────────────┘
       推流                  SFU 中继              接收 + 渲染
```

## 当前状态（经过验证）

| 测试项 | 结果 |
|--------|------|
| Linux → LiveKit Server 推流 | ✅ 1344x376 @60fps，NVENC H264 |
| 云端 SFU 中继 | ✅ UDP 直连，srflx 候选 |
| LiveKit → Python receiver 解码 | ✅ 100% 帧解码成功 |
| Vision Pro 接收（窗口模式） | ✅ 双眼可见视频 |
| Vision Pro 接收（沉浸模式） | ⚠️ 视频帧到达但 CompositorServices 渲染黑屏（待修复） |
| 网络延迟 | ✅ Linux → 云端 ~4ms，端到端 < 100ms |

## 目录结构

- `device-a-linux/` — Linux 工控机推流端代码（被控制端，拿摄像头的那台）
- `cloud/` — 阿里云 LiveKit Server 配置和部署说明
- `controller/` — Apple Vision Pro 接收端代码（控制端，戴头显的那个）

## 核心方案演进

之前尝试过两种自建 WebRTC 方案，**都失败了**：

1. **GStreamer webrtcbin 自建信令**：
   - GStreamer 1.20.3 的 DTLS（OpenSSL）和 LiveKit WebRTC（VP 端，BoringSSL）SRTP 协商不兼容
   - 表现：ICE connected，但 `Failed to unprotect RTP packet` 失败
   - 浏览器接收能工作，但 VP native app 不行

2. **aiortc 自建信令**：
   - aiortc 1.14.0 的双 ICE component 模式不稳定
   - 本地回环测试都失败（Connection(0) 和 Connection(1) 时序问题）
   - SRTP_AEAD_AES_256_GCM 默认协商，VP 端不支持

**最终方案：LiveKit Server 作为媒体中继 SFU**
- Linux 端用 LiveKit Python SDK 1.1.5（封装好的 WebRTC，自动用 NVENC 硬编码）
- VP 端用 LiveKit Swift SDK 2.12.1（官方支持 visionOS 1.0+，含 26.x）
- 两端都跟 LiveKit Server 对话，不直接对话 → 兼容性问题彻底消失
- 加密协商由 LiveKit 自己处理，DTLS/SRTP/ICE 全部由 SDK 封装

## 快速启动

### 1. 启动 LiveKit Server（云端，已部署）
```bash
ssh root@39.102.113.104
livekit-server --config /etc/livekit.yaml --node-ip 39.102.113.104
```
详见 [`cloud/README.md`](cloud/README.md)

### 2. 启动 Linux 推流端
```bash
# 必须用系统 Python（conda 没 gi 模块）
/usr/bin/python3 device-a-linux/zedmini_livekit_sender.py
```
详见 [`device-a-linux/README.md`](device-a-linux/README.md)

### 3. Vision Pro 接收端
在 Mac 上用 Xcode 26.4 打开 `controller/visionos-native/RemoteControl.xcodeproj`，target 选 Vision Pro 真机 → Run。
详见 [`controller/README.md`](controller/README.md)

## 网络要求

阿里云安全组入方向规则：
- TCP 7880 (LiveKit HTTP/WebSocket，必须)
- TCP 7881 (LiveKit RTC TCP fallback)
- UDP 50000-60000 (LiveKit RTC UDP，**视频数据走这里**)

如果 Linux 端运行 Clash Verge 等代理软件，需要在 TUN 配置中排除 LiveKit 服务器 IP 和局域网，否则 WebRTC UDP 流量会被劫持：

```yaml
# ~/.config/clash-verge-rev/Merge.yaml
tun:
  enable: true
  stack: mixed
  auto-route: true
  inet4-route-exclude-address:
    - 39.102.113.104/32
    - 192.168.0.0/16
    - 10.0.0.0/8
    - 172.16.0.0/12
```

## 关键参数

| 参数 | 值 |
|------|---|
| LiveKit 服务器 | `ws://39.102.113.104:7880` |
| API Key | `teleop_key` |
| API Secret | `teleop_secret_key_2026` |
| 房间名 | `teleop-room` |
| Sender 身份 | `zed-mini-sender` |
| Receiver 身份 | `vision-pro-receiver` |
| 视频分辨率 | 1344×376 SBS（每眼 672×376）|
| 帧率 | 60 fps |
| 编码 | H264 @ NVENC |
| 端到端延迟 | ~50-100ms |

---

## 项目全景地图（给后续 Agent 看）

```
teleop/
├── device-a-linux/          ← Linux 工控机端（Agent 1 主战场）
│   ├── zedmini_livekit_sender.py   ← ZED Mini → LiveKit 推流
│   ├── dm_motor/                    ← 达妙电机控制（DM_CAN 驱动 + 标定 + VP 遥控）
│   │   ├── scripts/dm_motor_vp_control.py  ← VP Pose → 电机角度映射
│   │   └── DM_Control_Python/              ← 达妙官方 SDK
│   ├── ros2_ws/src/                 ← ROS2 化改造（4 个 package）
│   │   ├── teleop_bringup/         ← 一键启动 launch
│   │   ├── teleop_dm_motor/        ← DM 电机 ROS2 节点
│   │   ├── teleop_livekit_bridge/  ← LiveKit→ROS2 话题桥
│   │   └── teleop_msgs/            ← 自定义消息（HeadPose 等）
│   ├── setup_linux.sh              ← 一键安装脚本
│   └── EXPERIMENTAL_VALIDATION_2026-04-08.md  ← 真机验证记录
│
├── cloud/                   ← 阿里云 LiveKit Server (39.102.113.104)
│   └── livekit.yaml
│
├── controller/              ← Apple Vision Pro 端
│   └── visionos-native/RemoteControl/
│       ├── LiveKit/         ← LiveKit 接收 + 管理
│       ├── Rendering/       ← CompositorServices 立体渲染 + Metal shader
│       └── Tracking/        ← 头部姿态追踪回传
│
├── scripts/                 ← 实验脚本合集（按日期命名）
│   ├── 20260408-cc-swerve_chassis.xml          ← ★ MuJoCo 完整场景（底盘+升降+双臂+云台）
│   ├── 20260408-cc-swerve_keyboard_control.py  ← ★ 键盘控制主脚本
│   ├── 20260408-cc-swerve_auto_test.py         ← 自动测试（无需键盘）
│   ├── 20260408-cc-NEXT_SESSION_PLAN.md        ← ★ MuJoCo 下一步规划（Touch 笔遥控右臂）
│   ├── 20260408-cc-run_full.sh                 ← 全套启动脚本
│   ├── 20260407-cc-dm_motor_*.py               ← 达妙电机独立脚本
│   ├── meshes/airbot/                          ← 机械臂 STL 网格（11 个）
│   └── meshes/parts/                           ← 零件 stl 网格（3 个）
│
├── ros2_ws/src/             ← 本地 ROS2 工作空间（可视化验证用）
│   ├── airbot_description/  ← URDF/xacro + mesh + RViz2 launch
│   ├── airbot_control/      ← 关节控制节点
│   ├── airbot_bringup/      ← 系统启动
│   └── airbot_msgs/         ← 自定义消息
│
├── force-feedback-pen/      ← 3D Systems Touch 力反馈笔
│   ├── 20260328-cc-airbot_force_sim.py    ← ★ Touch 笔 + MuJoCo 机械臂遥控（参考实现）
│   ├── 20260328-cc-haptic_driver.py       ← Touch 笔底层串口驱动
│   └── 20260328-cc-touch_serial_driver.py ← A5 5A 协议实现
│
├── Cangjie-Embodied/        ← 系统拓扑文档
├── HZY/                     ← 零件加工图纸
├── CLAUDE.md                ← 项目工作日志（ROS2 搭建全记录）
└── HANDOFF_PROMPT.md        ← VP 渲染问题交接文档（已修复）
```

## MuJoCo 仿真环境（Agent 2 成果）

### 已完成（2026-04-08）
完整的移动操作平台仿真，28 个关节、24 个执行器：

| 模块 | 关节 | 说明 |
|------|------|------|
| 四舵轮底盘 | 8（4转向+4驱动） | 全向移动，逆运动学+最短转向优化 |
| 升降机构 | 1 prismatic | G/H 键控制，行程 0~0.5m |
| 双臂 6DOF×2 | 12 revolute | airbot_play_force 带 G2 夹爪 mesh |
| 两轴云台 | 2 hinge + 1 slide | head_yaw + head_pitch + 伸缩 |
| 交互物体 | 3 free joint | 锅/盖/管（抓取测试用） |
| 底盘根 | 1 free joint | 物理驱动 |

### 运行方式
```bash
conda activate disc
cd /home/rhz/teleop/scripts
python 20260408-cc-swerve_keyboard_control.py
# 键盘 WASD=平移, QE=旋转, GH=升降, R=重置, ESC=退出
# ⚠️ 焦点放在终端窗口，不要点 MuJoCo 窗口
```

### 下一步任务（优先级排序）
详见 `scripts/20260408-cc-NEXT_SESSION_PLAN.md`：
1. ~~升降机构~~ ✅ 已完成
2. ~~双臂集成~~ ✅ 已完成
3. **Touch 力反馈笔遥控右臂** ← 下一个任务
   - 参考实现：`force-feedback-pen/20260328-cc-airbot_force_sim.py`
   - 核心：Touch 笔位姿增量 → MuJoCo IK → 右臂 6 关节角

## 真机硬件（Agent 1 成果）

### 已完成（2026-04-07 ~ 04-08）
- Linux → LiveKit → VP 端到端视频传输 ✅
- VP CompositorServices 立体渲染 ✅（修了 5 个串联 bug）
- VP 头部姿态回传 → 达妙电机 2 轴云台控制 ✅
- 完整 ROS2 化改造（4 个 package，一行命令起全套）✅
- 端到端真机验证 ground truth ✅

### 遗留问题
- VP 沉浸模式渲染偶尔需要重启 app 才能正常显示
- PREEMPT_RT 实时内核未配置（controller_manager 无法设置 FIFO 调度）

## 环境说明

| 环境 | Python | 用途 |
|------|--------|------|
| `/usr/bin/python3` | 3.10 系统 | ROS2、GStreamer、LiveKit（**不能用 conda**） |
| `conda activate disc` | 3.10 miniconda | MuJoCo 3.4.0、pynput（**不能用系统 python**） |
| ROS2 | Humble | `source /opt/ros/humble/setup.bash && source install/setup.bash` |

⚠️ **miniconda 污染问题**：`ros2` 命令需要走 `~/bin/ros2` wrapper 才能用系统 Python。
