# Remote Control — 全链路远程遥操作系统

仓颉 / Faber 美国超市货架巡检 + 补货 teleop 机器人。操作端在中国（Apple Vision Pro 视频沉浸 + ASUS Tkinter GUI 键盘控制），通过阿里云 LiveKit Server 中继到部署在美国的 Rain（AGX Orin 高层节点 + CangJie Raspberry Pi 4 底盘 + 抬升）。

## 详见

- [`ARCHITECTURE.md`](ARCHITECTURE.md) — 完整架构（机器清单、数据流、Cyclone DDS 配置、网络拓扑）
- [`SECURITY.md`](SECURITY.md) — Secret 处理规范、LiveKit 密钥轮换流程、红线清单
- [`.env.example`](.env.example) — 环境变量模板

## 架构总览

```
        ┌──────────────────┐
        │ Vision Pro       │  visionOS 26 + LiveKit Swift SDK
        │ (operator-vp/)   │   ── video sub + head-pose data pub ──┐
        │ MBA 2 (.0.212)   │                                       │
        └──────────────────┘                                       │
                                                                   │
        ┌──────────────────┐                                       │
        │ ASUS Tkinter GUI │  WASD/QE chassis · GH lift            │
        │ (operator-asus/) │   ── data ch: cmd_vel / lift_cmd ─────┤
        │ ASUS .0.182      │                                       │
        └──────────────────┘                                       │
                                                                   v
                                              ┌───────────────────────────────┐
                                              │ Aliyun LiveKit Server v1.10.1 │
                                              │ (cloud/)                      │
                                              │ ws://39.102.113.104:7880      │
                                              │ room: "teleop-room"           │
                                              └───────────────────────────────┘
                                                              ^  |
                                              ┌───────────────────────────────┐
                                              │ AGX Orin                      │
                                              │ (rain-orin/)                  │
                                              │ ZED Mini → GStreamer NVENC    │
                                              │ + ROS 2 Humble + AIRBOT SDK   │
                                              │ eno1=192.168.8.6              │
                                              └───────────────────────────────┘
                                                              |
                                                ROS 2 over Cyclone DDS
                                                Domain 0, 192.168.8.0/24
                                                              v
                                              ┌───────────────────────────────┐
                                              │ CangJie Raspberry Pi 4        │
                                              │ (rain-cangjie/)               │
                                              │ swerve IK + DM3519 lift       │
                                              │ eth0=192.168.8.7              │
                                              └───────────────────────────────┘
                                                              |
                                                          USB-CAN
                                                              v
                                                4 swerve wheels + 1 lift
```

## 当前状态（经过验证）

| 测试项 | 结果 |
|--------|------|
| Orin → LiveKit Server 推流 | ✅ ZED Mini SBS，NVENC H264，6Mbps Main intra-only |
| 云端 SFU 中继 | ✅ UDP 直连，srflx 候选 |
| LiveKit → Python receiver 解码 | ✅ 100% 帧解码成功 |
| Vision Pro 接收（窗口模式） | ✅ 双眼可见视频 |
| Vision Pro 接收（沉浸模式） | ✅ visionOS 26 + `SwiftUIVideoView` 路径已通 |
| ROS 2 跨机（Orin ↔ CangJie） | ✅ Cyclone DDS 192.168.8.0/24（systemd 服务尚未切换，见 ARCHITECTURE.md known debt）|
| 网络延迟（视频端到端） | ✅ glass-to-glass ~200–260 ms |

## 目录结构

- `operator-vp/`     — Apple Vision Pro Xcode app（在 PR-5 落地）
- `operator-asus/`   — ASUS Tkinter GUI 操控面板（在 PR-4 落地）
- `cloud/`           — 阿里云 LiveKit Server 配置
- `rain-orin/`       — AGX Orin 高层节点（ROS 2、视频、桥、AIRBOT）
- `rain-cangjie/`    — CangJie Raspberry Pi 4（底盘 swerve IK + 抬升）
- `scripts/`         — 历史脚本（2025-03 至 2026-04 早期，仅供参考）
- `controller/`、`RemoteControl/`、`force-feedback-pen/`、`TouchUsage/`、`HZY/`、`docker/` — 旧组件（待 phase-2 review）

完整目录-机器映射详见 [`ARCHITECTURE.md`](ARCHITECTURE.md)。

## 核心方案演进

### 之前尝试过的失败方案（自建 WebRTC，**都失败了**）

1. **GStreamer webrtcbin 自建信令**：
   - GStreamer 1.20.3 的 DTLS（OpenSSL）和 LiveKit WebRTC（VP 端，BoringSSL）SRTP 协商不兼容
   - 表现：ICE connected，但 `Failed to unprotect RTP packet` 失败
   - 浏览器接收能工作，但 VP native app 不行

2. **aiortc 自建信令**：
   - aiortc 1.14.0 的双 ICE component 模式不稳定
   - 本地回环测试都失败（Connection(0) 和 Connection(1) 时序问题）
   - SRTP_AEAD_AES_256_GCM 默认协商，VP 端不支持

### 当前方案：LiveKit Server 作为媒体中继 SFU

- Orin 端用 LiveKit Python SDK / `lk` CLI（封装好的 WebRTC，自动用 NVENC 硬编码）
- VP 端用 LiveKit Swift SDK 2.12.1（官方支持 visionOS 1.0+，含 26.x）
- 两端都跟 LiveKit Server 对话，不直接对话 → 兼容性问题彻底消失
- 加密协商由 LiveKit 自己处理，DTLS/SRTP/ICE 全部由 SDK 封装

## 快速启动

### 1. LiveKit Server（云端，已部署）

```bash
ssh root@39.102.113.104
livekit-server --config /etc/livekit.yaml --node-ip 39.102.113.104
```

详见 [`cloud/README.md`](cloud/README.md)

### 2. Orin 推流端

详见 [`rain-orin/README.md`](rain-orin/README.md)（PR-2 之后会刷新）

### 3. CangJie 底盘 + 抬升

详见 [`rain-cangjie/README.md`](rain-cangjie/README.md)（PR-3 之后会刷新）

### 4. Vision Pro 接收端

在 Mac 上用 Xcode 26.4 打开当前位于 `controller/visionos-native/` 的 Xcode 项目（PR-5 会迁到 `operator-vp/`），target 选 Vision Pro 真机 → Run。详见 [`controller/README.md`](controller/README.md)

### 5. ASUS 操控 GUI

`asus_teleop_gui.py`（PR-4 会落地到 `operator-asus/`）。键盘：WASD（平移）+ QE（旋转）+ G/H（升降）。

## 网络要求

阿里云安全组入方向规则：

- TCP 7880（LiveKit HTTP/WebSocket，必须）
- TCP 7881（LiveKit RTC TCP fallback）
- UDP 50000-60000（LiveKit RTC UDP，**视频数据走这里**）

如果操作端运行 Clash Verge 等代理软件，需要在 TUN 配置中排除 LiveKit 服务器 IP 和局域网，否则 WebRTC UDP 流量会被劫持。详见 `Clash-Claude-Proxy-Setup.md`。

## 关键参数

| 参数 | 值 |
|------|---|
| LiveKit 服务器 | `ws://39.102.113.104:7880` |
| API Key | `teleop_key` |
| API Secret | （见 `.env.example` / `SECURITY.md`，环境变量注入，不硬编码到新代码）|
| 房间名 | `teleop-room` |
| Rain LAN | `192.168.8.0/24`（Huawei CPE 5S → 千兆交换机）|
| Orin Tailscale | `100.75.76.30` |
| CangJie Tailscale | `100.70.149.16` |
| 视频分辨率 / 帧率 / 码率 | 1344×376 SBS（每眼 672×376）/ 30 fps / 6 Mbps |
| 编码 | H264 NVENC（intra-only 1-slice Main preset 4，2026-04-24 锁定）|
| 端到端延迟 | glass-to-glass ~200–260 ms |

## MuJoCo 仿真环境（历史成果，2026-04-08）

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
cd scripts
python 20260408-cc-swerve_keyboard_control.py
# 键盘 WASD=平移, QE=旋转, GH=升降, R=重置, ESC=退出
# ⚠️ 焦点放在终端窗口，不要点 MuJoCo 窗口
```

## 真机硬件

### 已完成（2026-04-07 ~ 04-26）

- Orin → LiveKit → VP 端到端视频传输 ✅
- VP 沉浸渲染 visionOS 26 + `SwiftUIVideoView` 路径 ✅
- VP 头部姿态回传 → 达妙电机 2 轴云台控制 ✅
- 完整 ROS2 化改造（一行命令起全套）✅
- 端到端真机验证 ground truth ✅
- Rain 网络岛（192.168.8.0/24，Huawei CPE 5S + 千兆交换机）上线 2026-04-26
- Cyclone DDS 跨机（Orin ↔ CangJie）192.168.8 子网通信 ✅

### 遗留问题

- Orin 5 个 systemd user services 尚未切到 Cyclone DDS（仍用 Fast DDS 默认），与 CangJie 隔离 — 见 ARCHITECTURE.md known debt
- LiveKit API Secret 历史泄露在 main 分支 — 见 SECURITY.md，生产切换前需轮换
- PREEMPT_RT 实时内核未配置（`controller_manager` 无法设置 FIFO 调度）

## 环境说明

| 环境 | Python | 用途 |
|------|--------|------|
| `/usr/bin/python3` | 3.10 系统 | ROS2、GStreamer、LiveKit（**不能用 conda**）|
| `conda activate disc` | 3.10 miniconda | MuJoCo 3.4.0、pynput（**不能用系统 python**）|
| ROS2 | Humble | `source /opt/ros/humble/setup.bash && source install/setup.bash` |

⚠️ **miniconda 污染问题**：`ros2` 命令需要走 `~/bin/ros2` wrapper 才能用系统 Python。
